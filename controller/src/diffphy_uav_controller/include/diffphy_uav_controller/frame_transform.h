#ifndef DIFFPHY_UAV_CONTROLLER_FRAME_TRANSFORM_H_
#define DIFFPHY_UAV_CONTROLLER_FRAME_TRANSFORM_H_

#include <cmath>

#include <Eigen/Core>
#include <tf2/LinearMath/Quaternion.h>

namespace diffphy_uav_controller {
namespace detail {

inline tf2::Quaternion MultiplyQuaternions(const tf2::Quaternion& lhs,
                                           const tf2::Quaternion& rhs) {
  return tf2::Quaternion(
      lhs.w() * rhs.x() + lhs.x() * rhs.w() + lhs.y() * rhs.z() - lhs.z() * rhs.y(),
      lhs.w() * rhs.y() - lhs.x() * rhs.z() + lhs.y() * rhs.w() + lhs.z() * rhs.x(),
      lhs.w() * rhs.z() + lhs.x() * rhs.y() - lhs.y() * rhs.x() + lhs.z() * rhs.w(),
      lhs.w() * rhs.w() - lhs.x() * rhs.x() - lhs.y() * rhs.y() - lhs.z() * rhs.z());
}

inline tf2::Quaternion NormalizeQuaternion(const tf2::Quaternion& q) {
  tf2::Quaternion normalized = q;
  normalized.normalize();
  return normalized;
}

}  // namespace detail

inline Eigen::Vector3d EnuVecToNWU(const Eigen::Vector3d& vec) {
  return Eigen::Vector3d(vec.y(), -vec.x(), vec.z());
}

inline Eigen::Vector3d NWUVecToENU(const Eigen::Vector3d& vec) {
  return Eigen::Vector3d(-vec.y(), vec.x(), vec.z());
}

inline tf2::Quaternion EnuQuatToNWU(const tf2::Quaternion& q_enu) {
  const double half_sqrt = std::sqrt(0.5);
  const tf2::Quaternion frame_rot(0.0, 0.0, -half_sqrt, half_sqrt);
  return detail::NormalizeQuaternion(detail::MultiplyQuaternions(frame_rot, q_enu));
}

inline tf2::Quaternion NWUQuatToENU(const tf2::Quaternion& q_nwu) {
  const double half_sqrt = std::sqrt(0.5);
  const tf2::Quaternion frame_rot(0.0, 0.0, half_sqrt, half_sqrt);
  return detail::NormalizeQuaternion(detail::MultiplyQuaternions(frame_rot, q_nwu));
}

inline tf2::Quaternion NWUQuatToNEU(const tf2::Quaternion& q_nwu) {
  const tf2::Quaternion frame_rot(1.0, 0.0, 0.0, 0.0);
  return detail::NormalizeQuaternion(detail::MultiplyQuaternions(frame_rot, q_nwu));
}

inline tf2::Quaternion NEUQuatToNWU(const tf2::Quaternion& q_neu) {
  const tf2::Quaternion frame_rot(1.0, 0.0, 0.0, 0.0);
  return detail::NormalizeQuaternion(detail::MultiplyQuaternions(frame_rot, q_neu));
}

inline Eigen::Vector3d EnuVecToNwu(const Eigen::Vector3d& vec) {
  return EnuVecToNWU(vec);
}

inline Eigen::Vector3d NwuVecToEnu(const Eigen::Vector3d& vec) {
  return NWUVecToENU(vec);
}

inline tf2::Quaternion EnuQuatToNwu(const tf2::Quaternion& q_enu) {
  return EnuQuatToNWU(q_enu);
}

inline tf2::Quaternion NwuQuatToEnu(const tf2::Quaternion& q_nwu) {
  return NWUQuatToENU(q_nwu);
}

inline tf2::Quaternion NwuQuatToNeu(const tf2::Quaternion& q_nwu) {
  return NWUQuatToNEU(q_nwu);
}

inline tf2::Quaternion NeuQuatToNwu(const tf2::Quaternion& q_neu) {
  return NEUQuatToNWU(q_neu);
}

}  // namespace diffphy_uav_controller

#endif  // DIFFPHY_UAV_CONTROLLER_FRAME_TRANSFORM_H_