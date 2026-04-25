#include "diffphy_uav_controller/attitude_mapper.h"

#include <algorithm>
#include <cmath>

#include <Eigen/Geometry>

namespace diffphy_uav_controller {

ControlOutput AttitudeMapper::Compute(const Eigen::Vector3d& desired_velocity,
                                      const Eigen::Vector3d& desired_acceleration,
                                      double last_yaw,
                                      double hover_thrust) const {
  constexpr double kGravity = 9.81;
  constexpr double kEpsilon = 1e-6;
  constexpr double kVelocityEpsilon = 1e-3;

  ControlOutput output;
  output.attitude.w = 1.0;
  output.attitude.x = 0.0;
  output.attitude.y = 0.0;
  output.attitude.z = 0.0;
  output.thrust = Clamp(hover_thrust, 0.0, 1.0);
  output.valid = false;

  const Eigen::Vector3d gravity(0.0, 0.0, -kGravity);
  // const Eigen::Vector3d total_acc = desired_acceleration - gravity
  const Eigen::Vector3d total_acc = desired_acceleration - gravity;
  const double total_acc_norm = total_acc.norm();
  if (total_acc_norm < kEpsilon) {
    return output;
  }

  Eigen::Vector3d z_b = total_acc / total_acc_norm;
  // Build heading reference on the plane orthogonal to z_b.
  // Eigen::Vector3d x_c = desired_velocity - desired_velocity.dot(z_b) * z_b; // remove component along z_b
  // if (x_c.norm() < kVelocityEpsilon) {
  //   Eigen::Vector3d yaw_ref(std::cos(last_yaw), std::sin(last_yaw), 0.0);
  //   x_c = yaw_ref - yaw_ref.dot(z_b) * z_b;
  //   // if (x_c.norm() < kVelocityEpsilon) {
  //   //   Eigen::Vector3d fallback_axis(1.0, 0.0, 0.0);
  //   //   x_c = fallback_axis - fallback_axis.dot(z_b) * z_b;
  //   //   if (x_c.norm() < kVelocityEpsilon) {
  //   //     fallback_axis = Eigen::Vector3d(0.0, 1.0, 0.0);
  //   //     x_c = fallback_axis - fallback_axis.dot(z_b) * z_b;
  //   //   }
  //   // }
  // }
  Eigen::Vector3d x_c = desired_velocity / desired_velocity.norm(); // desired velocity direction as heading reference
  x_c.normalize();

  Eigen::Vector3d y_b = z_b.cross(x_c);
  if (y_b.norm() < kEpsilon) {
    // Fallback when desired velocity is nearly parallel to body Z.
    x_c = Eigen::Vector3d(1.0, 0.0, 0.0);
    y_b = z_b.cross(x_c);
    if (y_b.norm() < kEpsilon) {
      x_c = Eigen::Vector3d(0.0, 1.0, 0.0);
      y_b = z_b.cross(x_c);
    }
  }
  y_b.normalize();

  const Eigen::Vector3d x_b = y_b.cross(z_b).normalized();

  Eigen::Matrix3d rotation;
  rotation.col(0) = x_b;
  rotation.col(1) = y_b;
  rotation.col(2) = z_b;

  Eigen::Quaterniond q(rotation);
  q.normalize();

  output.attitude.w = q.w();
  output.attitude.x = q.x();
  output.attitude.y = q.y();
  output.attitude.z = q.z();

  const double normalized_thrust = hover_thrust * total_acc_norm / kGravity;
  output.thrust = Clamp(normalized_thrust, 0.0, 1.0);
  output.valid = true;
  return output;
}

// TODO: Add more sophisticated thrust mapping, e.g., to account for motor limits and non-linearities. For now, we just do a simple linear mapping based on hover thrust and total acceleration, with output clamped to [0, 1].


//做输出截断
double AttitudeMapper::Clamp(double value, double min_value, double max_value) const {
  return std::max(min_value, std::min(value, max_value));
}

}  // namespace diffphy_uav_controller
