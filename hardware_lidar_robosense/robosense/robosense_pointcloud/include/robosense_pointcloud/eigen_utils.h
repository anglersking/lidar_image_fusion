
#ifndef LIDAR_EIGEN_UTILS_H__
#define LIDAR_EIGEN_UTILS_H__

#include <Eigen/Dense>
#include <cmath>
#include <tf/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf_conversions/tf_eigen.h>
namespace robosense_pointcloud {

inline Eigen::Isometry3d TF2Eigen(const tf::StampedTransform &transform) {
  Eigen::Isometry3d is3d;
  Eigen::Quaterniond quat;
  tf::quaternionTFToEigen(transform.getRotation(), quat);
  quat.normalize();
  Eigen::Vector3d vec;
  tf::vectorTFToEigen(transform.getOrigin(), vec);
  is3d = Eigen::Isometry3d::Identity();
  is3d.rotate(quat);
  is3d.pretranslate(vec);
  return is3d;
}

inline Eigen::Isometry3d
Statictf2Eigen(const geometry_msgs::TransformStamped &tf_static) {
  Eigen::Isometry3d is3d;
  Eigen::Quaterniond quat(
      tf_static.transform.rotation.w, tf_static.transform.rotation.x,
      tf_static.transform.rotation.y, tf_static.transform.rotation.z);
  quat.normalize();
  Eigen::Vector3d vec(tf_static.transform.translation.x,
                      tf_static.transform.translation.y,
                      tf_static.transform.translation.z);
  is3d = Eigen::Isometry3d::Identity();
  is3d.rotate(quat);
  is3d.pretranslate(vec);
  return is3d;
}

template <typename Scalar>
Eigen::Transform<Scalar, 3, Eigen::Isometry>
interpolation(const Eigen::Transform<Scalar, 3, Eigen::Isometry> &T1,
              const Eigen::Transform<Scalar, 3, Eigen::Isometry> &T2,
              float scale) {

  Eigen::Matrix<Scalar, 3, 1> v1 = T1.translation();
  Eigen::Matrix<Scalar, 3, 1> v2 = T2.translation();

  Eigen::Quaternion<Scalar> q1(T1.rotation());
  Eigen::Quaternion<Scalar> q2(T2.rotation());

  Eigen::Transform<Scalar, 3, Eigen::Isometry> T;
  double s = scale;
  Eigen::Translation<Scalar, 3> ti((1 - s) * v1 + s * v2);

  using std::abs;
  using std::acos;
  using std::sin;

  Eigen::Quaternion<Scalar> q(q1.conjugate() * q2);
  q.normalize();
  Eigen::Quaternion<Scalar> q0(Eigen::Quaternion<Scalar>::Identity());
  double d = q0.dot(q);
  double abs_d = abs(d);
  if (abs_d < 1.0 - 1.0e-8) {
    Scalar theta = acos(abs_d);
    Scalar sin_theta = sin(theta);
    Scalar c1_sign = (d > 0) ? 1 : -1;

    Scalar c0 = sin((1 - s) * theta) / sin_theta;
    Scalar c1 = sin(s * theta) / sin_theta * c1_sign;
    Eigen::Quaternion<Scalar> qi(c0 * q0.coeffs() + c1 * q.coeffs());
    qi.normalize();

    T = ti * q1 * qi;
  } else {
    T = ti * q1;
  }

  return T;
}

} // namespace robosense_pointcloud

#endif // LIDAR_EIGEN_UTILS_H__