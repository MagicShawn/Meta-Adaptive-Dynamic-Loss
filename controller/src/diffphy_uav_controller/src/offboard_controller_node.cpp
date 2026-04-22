#include <algorithm>
#include <cmath>

#include <geometry_msgs/Vector3Stamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <Eigen/Core>

#include "diffphy_uav_controller/attitude_mapper.h"

namespace {

enum class ControllerState {
  INIT,
  PRE_ARM,
  OFFBOARD_ACTIVE
};

class OffboardControllerNode {
 public:
  OffboardControllerNode()
      : nh_(),
        pnh_("~"),
        state_(ControllerState::INIT),
        has_des_vel_(false),
        has_des_acc_(false),
        last_yaw_(0.0),
        prearm_setpoint_count_(0) {
    pnh_.param("hover_thrust", hover_thrust_, 0.5);
    pnh_.param("ctrl_rate", ctrl_rate_, 50.0);
    pnh_.param("cmd_timeout", cmd_timeout_, 0.5);

    des_vel_sub_ = nh_.subscribe("/controller/des_vel", 10,
                                 &OffboardControllerNode::OnDesiredVelocity, this);
    des_acc_sub_ = nh_.subscribe("/controller/des_acc", 10,
                                 &OffboardControllerNode::OnDesiredAcceleration, this);
    mavros_state_sub_ = nh_.subscribe("/mavros/state", 10,
                                      &OffboardControllerNode::OnMavrosState, this);

    attitude_pub_ = nh_.advertise<mavros_msgs::AttitudeTarget>(
        "/mavros/setpoint_raw/attitude", 20);

    arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    last_cmd_time_ = ros::Time(0);
    last_request_time_ = ros::Time(0);

    timer_ = nh_.createTimer(ros::Duration(1.0 / ctrl_rate_),
                             &OffboardControllerNode::OnControlTimer, this);
  }

 private:
  void OnDesiredVelocity(const geometry_msgs::Vector3Stamped::ConstPtr& msg) {
    desired_velocity_ = Eigen::Vector3d(msg->vector.x, msg->vector.y, msg->vector.z);
    has_des_vel_ = true;
    last_cmd_time_ = ros::Time::now();
  }

  void OnDesiredAcceleration(const geometry_msgs::Vector3Stamped::ConstPtr& msg) {
    desired_acceleration_ = Eigen::Vector3d(msg->vector.x, msg->vector.y, msg->vector.z);
    has_des_acc_ = true;
    last_cmd_time_ = ros::Time::now();
  }

  void OnMavrosState(const mavros_msgs::State::ConstPtr& msg) {
    mavros_state_ = *msg;
  }

  void PublishSafeSetpoint(double thrust) {
    mavros_msgs::AttitudeTarget msg;
    msg.header.stamp = ros::Time::now();
    msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE |
                    mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE |
                    mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;
                    
    // NWU/FLU identity attitude is the same as NED/FRD identity
    // q_ned = (w, x, -y, -z) = (1, 0, 0, 0)
    msg.orientation.w = 1.0;
    msg.orientation.x = 0.0;
    msg.orientation.y = -0.0;
    msg.orientation.z = -0.0;
    msg.thrust = std::max(0.0, std::min(thrust, 1.0));

    attitude_pub_.publish(msg);
  }

  void TrySetOffboardAndArm() {
    const ros::Time now = ros::Time::now();
    if ((now - last_request_time_) < ros::Duration(1.0)) {
      return;
    }

    if (mavros_state_.mode != "OFFBOARD") {
      mavros_msgs::SetMode set_mode_srv;
      set_mode_srv.request.custom_mode = "OFFBOARD";
      if (set_mode_client_.call(set_mode_srv) && set_mode_srv.response.mode_sent) {
        ROS_INFO_THROTTLE(2.0, "OFFBOARD mode request sent");
      }
      last_request_time_ = now;
      return;
    }

    if (!mavros_state_.armed) {
      mavros_msgs::CommandBool arm_srv;
      arm_srv.request.value = true;
      if (arming_client_.call(arm_srv) && arm_srv.response.success) {
        ROS_INFO_THROTTLE(2.0, "Vehicle arm request sent");
      }
      last_request_time_ = now;
    }
  }

  void OnControlTimer(const ros::TimerEvent&) {
    if (!mavros_state_.connected) {
      state_ = ControllerState::INIT;
      PublishSafeSetpoint(0.0);
      return;
    }

    if (state_ == ControllerState::INIT) {
      state_ = ControllerState::PRE_ARM;
    }

    if (state_ == ControllerState::PRE_ARM) {
      PublishSafeSetpoint(0.0);
      ++prearm_setpoint_count_;

      if (prearm_setpoint_count_ > static_cast<int>(ctrl_rate_ * 2.0)) {
        TrySetOffboardAndArm();
      }

      if (mavros_state_.mode == "OFFBOARD" && mavros_state_.armed) {
        state_ = ControllerState::OFFBOARD_ACTIVE;
        ROS_INFO("Controller entered OFFBOARD_ACTIVE");
      }
      return;
    }

    Eigen::Vector3d cmd_vel = desired_velocity_;
    Eigen::Vector3d cmd_acc = desired_acceleration_;

    const bool command_stale = (!has_des_vel_ || !has_des_acc_) ||
                               ((ros::Time::now() - last_cmd_time_) > ros::Duration(cmd_timeout_));
    if (command_stale) {
      // Watchdog fallback to hover.
      cmd_vel.setZero();
      cmd_acc.setZero();
    }

    const diffphy_uav_controller::ControlOutput output =
        mapper_.Compute(cmd_vel, cmd_acc, last_yaw_, hover_thrust_);

    mavros_msgs::AttitudeTarget msg;
    msg.header.stamp = ros::Time::now();
    msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE |
                    mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE |
                    mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;
                    
    // NWU to NED orientation conversion: q_ned = (w_nwu, x_nwu, -y_nwu, -z_nwu)
    msg.orientation.w = output.attitude.w;
    msg.orientation.x = output.attitude.x;
    msg.orientation.y = -output.attitude.y;
    msg.orientation.z = -output.attitude.z;
    msg.thrust = output.thrust;

    attitude_pub_.publish(msg);

    // Keep NWU yaw for next loop's mapper computation (since mapper expects NWU)
    tf2::Quaternion q_nwu(output.attitude.x, output.attitude.y, output.attitude.z,
                          output.attitude.w);
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(q_nwu).getRPY(roll, pitch, yaw);
    last_yaw_ = yaw;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber des_vel_sub_;
  ros::Subscriber des_acc_sub_;
  ros::Subscriber mavros_state_sub_;
  ros::Publisher attitude_pub_;

  ros::ServiceClient arming_client_;
  ros::ServiceClient set_mode_client_;

  ros::Timer timer_;

  diffphy_uav_controller::AttitudeMapper mapper_;
  ControllerState state_;

  Eigen::Vector3d desired_velocity_;
  Eigen::Vector3d desired_acceleration_;
  bool has_des_vel_;
  bool has_des_acc_;

  mavros_msgs::State mavros_state_;
  ros::Time last_cmd_time_;
  ros::Time last_request_time_;

  double hover_thrust_;
  double ctrl_rate_;
  double cmd_timeout_;
  double last_yaw_;
  int prearm_setpoint_count_;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "offboard_controller_node");
  OffboardControllerNode node;
  ros::spin();
  return 0;
}
