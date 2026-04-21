#!/usr/bin/env python3
import os
import rospy
import yaml
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Vector3
from cv_bridge import CvBridge
try:
    from .policy.backbone import InferenceBackbone
except ImportError:
    from ..MADL.policy.backbone import InferenceBackbone

class DeployNode:
    def __init__(self, config_path=None):
        rospy.init_node('e2e_inference_deploy')
        default_cfg = os.path.join(os.path.dirname(__file__), 'config.yaml')
        config_file = rospy.get_param('~config', config_path or default_cfg)
        with open(config_file, 'r') as f:
            cfg = yaml.safe_load(f)
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
        depth_scale = cfg.get('depth_scale', 1000.0)

        self.backbone = InferenceBackbone(
            net_weight,
            norm_ckpt,
            no_odom=no_odom,
            depth_scale=depth_scale,
        )
        self.bridge = CvBridge()
        self.depth_img = None
        self.odom = None
        self.target = None
        self.margin = cfg.get('margin', 0.2)
        self.target_speed = cfg.get('target_speed', 1.0)
        # 订阅深度、里程计、目标点
        rospy.Subscriber(cfg['depth_topic'], Image, self.depth_cb, queue_size=1)
        rospy.Subscriber(cfg['odom_topic'], Odometry, self.odom_cb, queue_size=1)
        rospy.Subscriber(cfg['target_topic'], Point, self.target_cb, queue_size=1)
        # 发布加速度、速度
        self.acc_pub = rospy.Publisher(cfg['des_acc_topic'], Vector3, queue_size=1)
        self.vel_pub = rospy.Publisher(cfg['des_vel_topic'], Vector3, queue_size=1)
        self.rate_hz = cfg.get('rate_hz', 20)

    def depth_cb(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.depth_img = img
        except Exception as e:
            rospy.logwarn(f"Depth image conversion failed: {e}")

    def odom_cb(self, msg):
        pos = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
        q = [msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z]
        v = [msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z]
        self.odom = {'position': pos, 'quaternion': q, 'linear_velocity': v}

    def target_cb(self, msg):
        self.target = {'position': [msg.x, msg.y, msg.z]}

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            if self.depth_img is not None and self.odom is not None and self.target is not None:
                des_acc, des_vel, _ = self.backbone.predict(self.depth_img, self.odom, self.target, self.margin, self.target_speed)
                acc_msg = Vector3(*des_acc.tolist())
                vel_msg = Vector3(*des_vel.tolist())
                self.acc_pub.publish(acc_msg)
                self.vel_pub.publish(vel_msg)
            rate.sleep()

if __name__ == '__main__':
    import sys
    config_path = sys.argv[1] if len(sys.argv) > 1 else None
    node = DeployNode(config_path)
    node.spin()
