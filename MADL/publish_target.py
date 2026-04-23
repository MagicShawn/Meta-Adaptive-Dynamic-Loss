#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import yaml
import os
from geometry_msgs.msg import Point

def main():
    rospy.init_node('target_publisher', anonymous=True)
    
    # 从 config.yaml 中读取配置属性
    config_file = os.path.join(os.path.dirname(__file__), 'config', 'config.yaml')
    with open(config_file, 'r') as f:
        cfg = yaml.safe_load(f)
        
    topic_name = cfg.get('target_topic', '/target_point')
    pub = rospy.Publisher(topic_name, Point, queue_size=10)
    
    # 设置发布频率 (与控制器 50Hz 匹配)
    rate_hz = cfg.get('rate_hz', 50.0)
    rate = rospy.Rate(rate_hz)
    
    # 轨迹参数配置
    speed = cfg.get('target_speed', 3.0)  # 从 config 中的 target_speed 读取
    max_distance = 50.0   # 直线轨迹总长度50米
    dt = 1.0 / rate_hz
    
    target_pt = Point()
    target_pt.x = 0.0
    target_pt.y = 0.0
    # 此处 Z 轴数值根据无人机坐标系调整。如果是 NED 坐标系，-2 往往代表以相对起飞点 2m 的高度巡航
    target_pt.z = 2.0 
    
    rospy.loginfo("开始在话题 %s 发布 50m 直线轨迹目标点...", topic_name)
    
    while not rospy.is_shutdown():
        # 更新目标点，使其沿 X 轴缓慢向前，形成一条50米的虚拟引导线
        if target_pt.x < max_distance:
            target_pt.x += speed * dt
        else:
            target_pt.x = max_distance # 到达50米后保持在该位置
            
        pub.publish(target_pt)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
