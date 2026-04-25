#!/usr/bin/env python3
import os
import rospy
import yaml
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Vector3Stamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from policy.backbone import InferenceBackbone
from utils import enu_quat_to_nwu, enu_vec_to_nwu

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
        # self.depth_img = None
        self.odom = None
        self.target = {'position': [100.0, 0.0, 4.0]}
        self.margin = self.cfg.get('margin', 0.1)
        self.target_speed = self.cfg.get('target_speed', 10)
        self.missing_input_warn_period = cfg.get('missing_input_warn_period', 1.0)
        
        self.height = self.cfg.get('height', 640)
        self.width = self.cfg.get('width', 480)
        self.max_dis = self.cfg.get('max_dis', 10.0)
        self.min_dis = self.cfg.get('min_dis', 0.1)

        self.rate_hz = float(cfg.get('rate_hz', 20.0))
        if self.rate_hz <= 0:
            raise ValueError('rate_hz must be > 0')
        self.min_publish_period = 1.0 / self.rate_hz
        self.last_publish_time = rospy.Time(0)
        
        rospy.Subscriber(self.cfg['odom_topic'], Odometry, self.odom_cb, queue_size=1)
        rospy.Subscriber(self.cfg['target_topic'], Point, self.target_cb, queue_size=1)
        rospy.Subscriber(self.cfg['depth_topic'], Image, self.depth_cb, queue_size=1)
        self.acc_pub = rospy.Publisher(self.cfg['des_acc_topic'], Vector3Stamped, queue_size=1)
        self.vel_pub = rospy.Publisher(self.cfg['des_vel_topic'], Vector3Stamped, queue_size=1)
        # self.rate_hz = self.cfg.get('rate_hz', 20)

    def depth_cb(self, msg):
        try:
            if msg.encoding == '32FC1':
                img = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
            elif msg.encoding == '16UC1':
                img = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width).astype(np.float32) / 1000.0
            else:
                img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            rospy.logwarn(f"Depth image conversion failed: {e}")
            return
            
        if img.shape[0] != self.height or img.shape[1] != self.width:
            img = cv2.resize(img, (self.width, self.height), interpolation=cv2.INTER_NEAREST)
        img = np.minimum(img, self.max_dis) / self.max_dis

        nan_mask = np.isnan(img) | (img < self.min_dis / self.max_dis)
        interpolated_image = cv2.inpaint(np.uint8(img * 255), np.uint8(nan_mask), 1, cv2.INPAINT_NS)
        img = interpolated_image.astype(np.float32) / 255.0
        
        if self.odom is None or self.target is None:
            rospy.logwarn_throttle(
                self.missing_input_warn_period,
                'Waiting for odom/target before depth-triggered inference',
            )
            return
        
        # Keep publish behavior deterministic when depth topic runs faster than control rate.
        now = rospy.Time.now()
        if self.last_publish_time != rospy.Time(0):
            if (now - self.last_publish_time).to_sec() < self.min_publish_period:
                return
        rospy.loginfo_throttle(1.0, f"Running inference at {now.to_sec():.2f}s with target {self.target['position']} and odom pos {self.odom['position']}")
        des_acc, des_vel, _ = self.backbone.predict(
            img,
            self.odom,
            self.target,
            self.margin,
            self.target_speed,
        )
        now = rospy.Time.now()
        acc_msg = Vector3Stamped()
        acc_msg.header.stamp = now
        acc_msg.vector.x, acc_msg.vector.y, acc_msg.vector.z = des_acc.tolist()

        vel_msg = Vector3Stamped()
        vel_msg.header.stamp = now
        vel_msg.vector.x, vel_msg.vector.y, vel_msg.vector.z = des_vel.tolist()
        rospy.loginfo_throttle(1.0, f"Publishing des_acc: {des_acc}, des_vel: {des_vel}")

        self.acc_pub.publish(acc_msg)
        self.vel_pub.publish(vel_msg)
        self.last_publish_time = now

    def odom_cb(self, msg):
        # ENU (X-east, Y-north, Z-up) -> NWU (X-north, Y-west, Z-up)
        pos = enu_vec_to_nwu(
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        )
        qw = msg.pose.pose.orientation.w
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        q = enu_quat_to_nwu(qw, qx, qy, qz)
        v = enu_vec_to_nwu(
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
        )
        self.odom = {'position': pos, 'quaternion': q, 'linear_velocity': v}
        

    def target_cb(self, msg):
        # 记录接收到的原始数据，判断是否有数据进来
        rospy.loginfo_once(f"Received target point: x={msg.x}, y={msg.y}, z={msg.z}")
        # ENU (X-east, Y-north, Z-up) -> NWU (X-north, Y-west, Z-up)
        self.target = {'position': enu_vec_to_nwu(msg.x, msg.y, msg.z)}

    def spin(self):
        rospy.spin()
        # rate = rospy.Rate(self.rate_hz)
        # while not rospy.is_shutdown():
        #     if self.depth_img is not None and self.odom is not None and self.target is not None:
        #         des_acc, des_vel, _ = self.backbone.predict(self.depth_img, self.odom, self.target, self.margin, self.target_speed)
                
        #         acc_msg = Vector3Stamped()
        #         acc_msg.header.stamp = rospy.Time.now()
        #         acc_msg.vector.x, acc_msg.vector.y, acc_msg.vector.z = des_acc.tolist()
                
        #         vel_msg = Vector3Stamped()
        #         vel_msg.header.stamp = rospy.Time.now()
        #         vel_msg.vector.x, vel_msg.vector.y, vel_msg.vector.z = des_vel.tolist()
                
        #         self.acc_pub.publish(acc_msg)
        #         self.vel_pub.publish(vel_msg)
        #     rate.sleep()

if __name__ == '__main__':
    import sys
    config_path = sys.argv[1] if len(sys.argv) > 1 else None
    node = DeployNode(config_path)
    node.spin()
