import torch
import torch.nn.functional as F
import numpy as np
import os

try:
    from .madl_network import WorkNet
    from .rotation import quaternion_to_matrix
except ImportError:
    from policy.madl_network import WorkNet
    from policy.rotation import quaternion_to_matrix

class InferenceBackbone:
    def __init__(self, net_weight, norm_ckpt=None, no_odom=False, depth_scale=1000.0):
        if not net_weight:
            raise ValueError('net_weight is required')
        self.no_odom = no_odom
        self.depth_scale = float(depth_scale)
        if self.depth_scale <= 0:
            raise ValueError('depth_scale must be > 0 to match training-time depth preprocessing')
        self.model = WorkNet(dim_obs=7+3 if not no_odom else 4+3, dim_action=6, max_seq_len=32).eval()
        state_dict = torch.load(net_weight, map_location='cpu', weights_only=True)
        self.model.load_state_dict(state_dict, strict=True)
        self.state_norm_mean = None
        self.state_norm_var = None
        self.state_norm_eps = 1e-5
        if norm_ckpt and os.path.isfile(norm_ckpt):
            norm_state = torch.load(norm_ckpt, map_location='cpu', weights_only=True)
            if isinstance(norm_state, dict) and 'mean' in norm_state and 'var' in norm_state:
                self.state_norm_mean = norm_state['mean'].float()
                self.state_norm_var = norm_state['var'].float().clamp_min(1e-8)
        self.hx = None

    @staticmethod
    def _field(obj, key):
        if isinstance(obj, dict):
            return obj[key]
        return getattr(obj, key)

    def preprocess_depth(self, depth_img):
        depth = np.float32(depth_img) / self.depth_scale
        depth[depth == 0] = 24.0
        depth = 3 / np.clip(depth, 0.3, 24) - 0.6
        h, w = depth.shape
        _h = round((h - h * 0.82) / 2)
        _w = round((w - w * 0.82) / 2)
        depth = torch.as_tensor(depth[_h:-_h, _w:-_w])[None, None]
        depth = F.interpolate(depth, (36, 48), mode='nearest')
        depth = F.max_pool2d(depth, (3, 3))
        return depth

    def build_state(self, odom, target, margin=0.2, target_speed=1.0):
        p = torch.as_tensor(self._field(odom, 'position'), dtype=torch.float32)
        q = torch.as_tensor(self._field(odom, 'quaternion'), dtype=torch.float32)
        v = torch.as_tensor(self._field(odom, 'linear_velocity'), dtype=torch.float32)
        R = quaternion_to_matrix(q)
        env_R = R.clone()

        p_target = torch.as_tensor(self._field(target, 'position'), dtype=p.dtype, device=p.device)
        target_v_raw = p_target.to(dtype=p.dtype, device=p.device) - p
        target_v_norm = torch.norm(target_v_raw, 2, -1, keepdim=True)
        max_speed = torch.tensor([target_speed], dtype=p.dtype, device=p.device)
        target_v = (target_v_raw / (target_v_norm + 1e-6)) * torch.minimum(target_v_norm, max_speed)

        margin_tensor = torch.tensor([margin], dtype=p.dtype, device=p.device)
        state = [target_v[None] @ R, env_R[None, 2], margin_tensor[None]]
        global_v = v @ env_R
        if not self.no_odom:
            state.insert(0, global_v[None] @ R)
        state = torch.cat(state, -1)
        expected_dim = 7 if self.no_odom else 10
        if state.shape[-1] != expected_dim:
            raise ValueError(f'Unexpected state dimension {state.shape[-1]}, expected {expected_dim}')

        state = torch.nan_to_num(state, nan=0.0, posinf=20.0, neginf=-20.0).clamp(-20.0, 20.0)
        if self.state_norm_mean is not None and self.state_norm_var is not None:
            mean = self.state_norm_mean.to(dtype=state.dtype, device=state.device)
            var = self.state_norm_var.to(dtype=state.dtype, device=state.device)
            if mean.numel() != state.shape[-1] or var.numel() != state.shape[-1]:
                raise ValueError(
                    f'norm_ckpt dimension mismatch: state={state.shape[-1]}, '
                    f'mean={mean.numel()}, var={var.numel()}'
                )
            state = (state - mean) / torch.sqrt(var + self.state_norm_eps)
        state = torch.nan_to_num(state, nan=0.0, posinf=10.0, neginf=-10.0).clamp(-10.0, 10.0)
        return state


    @torch.no_grad()
    def predict(self, depth_img, odom, target, margin=0.2, target_speed=1.0):
        depth_tensor = self.preprocess_depth(depth_img)
        state_tensor = self.build_state(odom, target, margin, target_speed)
        act, _, self.hx = self.model(depth_tensor, state_tensor, self.hx)
        act = torch.nan_to_num(act, nan=0.0, posinf=10.0, neginf=-10.0).clamp(-10.0, 10.0)
        q = torch.as_tensor(self._field(odom, 'quaternion'), dtype=torch.float32)
        R = quaternion_to_matrix(q)
        a_world, v_world, *_ = (R @ act.reshape(3, -1)).unbind(-1) # 解码模式保持一致
        des_acc = a_world - v_world
        # des_acc[2] += 9.81  # 补充 z 轴重力补偿
        des_vel = v_world
        return des_acc.numpy(), des_vel.numpy(), {'a_world': a_world.numpy(), 'v_world': v_world.numpy()}
