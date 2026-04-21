#ifndef DIFFPHY_UAV_CONTROLLER_ATTITUDE_MAPPER_H_
#define DIFFPHY_UAV_CONTROLLER_ATTITUDE_MAPPER_H_

#include <geometry_msgs/Quaternion.h>
#include <Eigen/Core>

namespace diffphy_uav_controller {

struct ControlOutput {
  geometry_msgs::Quaternion attitude;
  double thrust;
  bool valid;
};

class AttitudeMapper {
 public:
  ControlOutput Compute(const Eigen::Vector3d& desired_velocity,
                        const Eigen::Vector3d& desired_acceleration,
                        double last_yaw,
                        double hover_thrust) const;
 private:
  double Clamp(double value, double min_value, double max_value) const;
};

}  // namespace diffphy_uav_controller

#endif  // DIFFPHY_UAV_CONTROLLER_ATTITUDE_MAPPER_H_
