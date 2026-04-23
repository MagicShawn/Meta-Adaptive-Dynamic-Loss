#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Vector3Stamped


def make_msg(x, y, z):
    msg = Vector3Stamped()
    msg.header.stamp = rospy.Time.now()
    msg.vector.x = x
    msg.vector.y = y
    msg.vector.z = z
    return msg


def main():
    rospy.init_node("fixed_cmd_publisher")

    vel_topic = rospy.get_param("~vel_topic", "/controller/des_vel")
    acc_topic = rospy.get_param("~acc_topic", "/controller/des_acc")
    rate_hz = float(rospy.get_param("~rate", 20.0))

    vel_x = float(rospy.get_param("~vel_x", 0.0))
    vel_y = float(rospy.get_param("~vel_y", 0.0))
    vel_z = float(rospy.get_param("~vel_z", 0.0))

    acc_x = float(rospy.get_param("~acc_x", 0.0))
    acc_y = float(rospy.get_param("~acc_y", 0.0))
    acc_z = float(rospy.get_param("~acc_z", 0.0))

    vel_pub = rospy.Publisher(vel_topic, Vector3Stamped, queue_size=10)
    acc_pub = rospy.Publisher(acc_topic, Vector3Stamped, queue_size=10)

    rospy.loginfo("Fixed command publisher started")
    rospy.loginfo("vel topic=%s, cmd=[%.3f, %.3f, %.3f]", vel_topic, vel_x, vel_y, vel_z)
    rospy.loginfo("acc topic=%s, cmd=[%.3f, %.3f, %.3f]", acc_topic, acc_x, acc_y, acc_z)
    rospy.loginfo("rate=%.1f Hz", rate_hz)

    rate = rospy.Rate(rate_hz)
    while not rospy.is_shutdown():
      now = rospy.Time.now()
      vel_msg = make_msg(vel_x, vel_y, vel_z)
      acc_msg = make_msg(acc_x, acc_y, acc_z)
      vel_msg.header.stamp = now
      acc_msg.header.stamp = now
      vel_pub.publish(vel_msg)
      acc_pub.publish(acc_msg)
      rate.sleep()


if __name__ == "__main__":
    try:
      main()
    except rospy.ROSInterruptException:
      pass
