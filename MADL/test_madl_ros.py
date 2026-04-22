#!/usr/bin/env python3
import os
import rospy
import yaml
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Vector3, Vector3Stamped
from cv_bridge import CvBridge
import numpy as np
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from policy.backbone import InferenceBackbone

class DeployNode:
    def __init__(self, config_path=None):
        rospy.init_node('e2e_inference_deploy')

        default_config_file = "/home/sean/work_ws/diff_deploy/MADL/config/config.yaml"
        config_file = config_path if config_path else default_config_file
        
        if not os.path.exists(config_file):
            rospy.logerr(f"Config file NOT FOUND at: {config_file}")
            sys.exit(1)
            
        with open(config_file, 'r') as f:
            cfg = yaml.safe_load(f)
        
        # 将配置保存到 self 供全局使用
        self.cfg = cfg
        rospy.loginfo(f"Successfully loaded config from: {config_file}")
        rospy.loginfo(f"Target topic set to: {self.cfg.get('target_topic')}")
        
        cfg_dir = os.path.dirname(os.path.abspath(config_file))

        def resolve_path(path):
            if not path:
                return None
            if os.path.isabs(path):
                return path
            return os.path.normpath(os.path.join(cfg_dir, path))

        net_weight = resolve_path(cfg.get('net_weight'))
        norm_ckpt = resolve_path(cfg.get('norm_ckpt'))
        no_odom = cfg.get('no_odom', False)
        depth_scale = cfg.get('depth_scale', 1.0)

        self.backbone = InferenceBackbone(
            net_weight,
            norm_ckpt,
            no_odom=no_odom,
            depth_scale=depth_scale,
        )
        self.bridge = CvBridge()
        self.depth_img = None
        self.odom = None
        self.target = {'position': [10.0, 0.0, 0.5]}
        self.margin = self.cfg.get('margin', 0.1)
        self.target_speed = self.cfg.get('target_speed', 10)
        # 订阅深度、里程计、目标点
        rospy.Subscriber(self.cfg['depth_topic'], Image, self.depth_cb, queue_size=1)
        rospy.Subscriber(self.cfg['odom_topic'], Odometry, self.odom_cb, queue_size=1)
        rospy.Subscriber(self.cfg['target_topic'], Point, self.target_cb, queue_size=1)
        # 发布加速度、速度
        self.acc_pub = rospy.Publisher(self.cfg['des_acc_topic'], Vector3Stamped, queue_size=1)
        self.vel_pub = rospy.Publisher(self.cfg['des_vel_topic'], Vector3Stamped, queue_size=1)
        self.rate_hz = self.cfg.get('rate_hz', 20)

    def depth_cb(self, msg):
        try:
            if msg.encoding == '32FC1':
                img = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
            elif msg.encoding == '16UC1':
                img = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
            else:
                img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.depth_img = img
        except Exception as e:
            rospy.logwarn(f"Depth image conversion failed: {e}")

    def odom_cb(self, msg):
        # NED (X-向前, Y-向右, Z-向下) 转 NWU/FLU (X-向前, Y-向左, Z-向上)
        pos = [msg.pose.pose.position.x, -msg.pose.pose.position.y, -msg.pose.pose.position.z]
        # 四元数 NED 到 NWU 的坐标变换推导：
        # q_nwu = q_rot * q_ned * q_rot^-1，其中绕X轴转180度 q_rot = (0, 1, 0, 0)
        # 展开乘法后恰好等同于仅对 Y 和 Z 取负，即 [qw, qx, -qy, -qz]
        qw, qx, qy, qz = msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z
        q = [qw, qx, -qy, -qz]
        
        v = [msg.twist.twist.linear.x, -msg.twist.twist.linear.y, -msg.twist.twist.linear.z]
        self.odom = {'position': pos, 'quaternion': q, 'linear_velocity': v}

    def target_cb(self, msg):
        # 记录接收到的原始数据，判断是否有数据进来
        rospy.loginfo_once(f"Received target point: x={msg.x}, y={msg.y}, z={msg.z}")
        # 同样需要将目标点从 NED 转为 NWU
        self.target = {'position': [msg.x, -msg.y, -msg.z]}

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            if self.depth_img is not None and self.odom is not None and self.target is not None:
                des_acc, des_vel, _ = self.backbone.predict(self.depth_img, self.odom, self.target, self.margin, self.target_speed)
                
                acc_msg = Vector3Stamped()
                acc_msg.header.stamp = rospy.Time.now()
                acc_msg.vector.x, acc_msg.vector.y, acc_msg.vector.z = des_acc.tolist()
                
                vel_msg = Vector3Stamped()
                vel_msg.header.stamp = rospy.Time.now()
                vel_msg.vector.x, vel_msg.vector.y, vel_msg.vector.z = des_vel.tolist()
                
                self.acc_pub.publish(acc_msg)
                self.vel_pub.publish(vel_msg)
            rate.sleep()

if __name__ == '__main__':
    import sys
    config_path = sys.argv[1] if len(sys.argv) > 1 else None
    node = DeployNode(config_path)
    node.spin()
