#!/usr/bin/env python
import rospy
import tf
import numpy as np
import message_filters
from geometry_msgs.msg import Vector3Stamped

def callback(acc_msg, vel_msg):
    # 1. 将 ROS 消息提取为 numpy 数组
    a = np.array([acc_msg.vector.x, acc_msg.vector.y, acc_msg.vector.z])
    v = np.array([vel_msg.vector.x, vel_msg.vector.y, vel_msg.vector.z])

    # 防止加速度过小导致除以零
    norm_a = np.linalg.norm(a)
    if norm_a < 1e-5:
        return

    # --- 核心数学计算 ---
    
    # Z轴：加速度归一化
    z_b = a / norm_a

    # X轴：速度在垂直于 Z 轴平面的正交分量 (投影)
    v_ortho = v - np.dot(v, z_b) * z_b
    norm_v_ortho = np.linalg.norm(v_ortho)
    
    if norm_v_ortho < 1e-5:
        # 奇异状态处理：如果速度为0或完全平行于加速度（悬停或纯垂直升降）
        # 我们给定一个伪X轴并正交化，避免计算崩溃
        x_fallback = np.array([1.0, 0.0, 0.0])
        x_b = x_fallback - np.dot(x_fallback, z_b) * z_b
        x_b = x_b / np.linalg.norm(x_b)
    else:
        x_b = v_ortho / norm_v_ortho

    # Y轴：Z 叉乘 X
    y_b = np.cross(z_b, x_b)

    # 构建 4x4 齐次旋转矩阵，供 tf.transformations 使用
    rotation_matrix = np.identity(4)
    rotation_matrix[0:3, 0:3] = np.column_stack((x_b, y_b, z_b))

    # 从旋转矩阵转换为四元数 [x, y, z, w]
    quat = tf.transformations.quaternion_from_matrix(rotation_matrix)

    # --- TF 广播可视化 ---
    
    br = tf.TransformBroadcaster()
    # 假设世界坐标系为 'world'，生成的机体坐标系名为 'drone_body'
    br.sendTransform(
        (0, 0, 0),          # 由于话题没有位置信息，平移固定为原点(只看姿态旋转)
        quat,               # 旋转四元数
        acc_msg.header.stamp, # 使用对应的时间戳
        "drone_body",       # 子坐标系名称
        "world"             # 父坐标系名称
    )

if __name__ == '__main__':
    rospy.init_node('drone_frame_visualizer')
    
    # 订阅带有时间戳的向量话题
    acc_sub = message_filters.Subscriber('/controller/des_acc', Vector3Stamped)
    vel_sub = message_filters.Subscriber('/controller/des_vel', Vector3Stamped)
    
    # 时间同步器：队列长度设为10，允许的最大时间误差为0.05秒 (可根据你的发布频率微调)
    ts = message_filters.ApproximateTimeSynchronizer([acc_sub, vel_sub], 10, 0.05)
    ts.registerCallback(callback)
    
    rospy.loginfo("开始计算并广播无人机坐标系，请在 RViz 中查看 'drone_body' TF...")
    rospy.spin()