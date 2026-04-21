import torch
import torch.nn.functional as F
import numpy as np
import os

try:
    from .madl_network import WorkNet
except ImportError:
    from policy.madl_network import WorkNet

# Quaternion to rotation matrix (wxyz)
def quat_to_rot(q):
    w, x, y, z = q
    R = np.array([
        [1-2*y**2-2*z**2, 2*x*y-2*z*w, 2*x*z+2*y*w],
        [2*x*y+2*z*w, 1-2*x**2-2*z**2, 2*y*z-2*x*w],
        [2*x*z-2*y*w, 2*y*z+2*x*w, 1-2*x**2-2*y**2]
    ])
    return torch.from_numpy(R).float()

class InferenceBackbone:
    def __init__(self, net_weight, norm_ckpt=None, no_odom=False, depth_scale=1000.0):
        if not net_weight:
            raise ValueError('net_weight is required')
        self.no_odom = no_odom
        self.depth_scale = float(depth_scale)
        self.model = WorkNet(dim_obs=7+3 if not no_odom else 4+3, dim_action=6, max_seq_len=32).eval()
        state_dict = torch.load(net_weight, map_location='cpu')
        self.model.load_state_dict(state_dict, strict=True)
        self.state_norm_mean = None
        self.state_norm_var = None
        self.state_norm_eps = 1e-5
        if norm_ckpt and os.path.isfile(norm_ckpt):
            norm_state = torch.load(norm_ckpt, map_location='cpu')
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
        depth = np.float32(depth_img)
        if self.depth_scale > 0:
            depth = depth / self.depth_scale
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
        p = torch.as_tensor(self._field(odom, 'position'))
        q = torch.as_tensor(self._field(odom, 'quaternion'))
        v = torch.as_tensor(self._field(odom, 'linear_velocity'))
        R = quat_to_rot(q)
        p_target = torch.as_tensor(self._field(target, 'position'))
        target_v_raw = p_target - p
        target_v_norm = torch.norm(target_v_raw, 2, -1, keepdim=True)
        max_speed = torch.tensor([target_speed], dtype=p.dtype)
        target_v = (target_v_raw / (target_v_norm + 1e-6)) * torch.minimum(target_v_norm, max_speed)
        state = [target_v[None] @ R, R[None, 2], torch.tensor([[margin]])]
        global_v = v @ R.T
        if not self.no_odom:
            state.insert(0, global_v[None] @ R)
        state = torch.cat(state, -1)
        state = torch.nan_to_num(state, nan=0.0, posinf=20.0, neginf=-20.0).clamp(-20.0, 20.0)
        if self.state_norm_mean is not None and self.state_norm_var is not None:
            mean = self.state_norm_mean.to(dtype=state.dtype)
            var = self.state_norm_var.to(dtype=state.dtype)
            state = (state - mean) / torch.sqrt(var + self.state_norm_eps)
        state = torch.nan_to_num(state, nan=0.0, posinf=10.0, neginf=-10.0).clamp(-10.0, 10.0)
        return state

    @torch.no_grad()
    def predict(self, depth_img, odom, target, margin=0.2, target_speed=1.0):
        depth_tensor = self.preprocess_depth(depth_img)
        state_tensor = self.build_state(odom, target, margin, target_speed)
        act, _, self.hx = self.model(depth_tensor, state_tensor, self.hx)
        act = torch.nan_to_num(act, nan=0.0, posinf=10.0, neginf=-10.0).clamp(-10.0, 10.0)
        q = torch.as_tensor(self._field(odom, 'quaternion'))
        R = quat_to_rot(q)
        a_body = act[0, :3]
        v_body = act[0, 3:]
        a_world = R @ a_body
        v_world = R @ v_body
        des_acc = a_world - v_world
        des_vel = v_world
        return des_acc.numpy(), des_vel.numpy(), {'a_world': a_world.numpy(), 'v_world': v_world.numpy()}
