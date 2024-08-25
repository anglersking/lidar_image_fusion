#ifndef LIDAR_MATH_UTILS_H
#define LIDAR_MATH_UTILS_H

#include <cmath>
#include <pcl/point_types.h>

/** \brief Convert the given radian angle to degrees.
 *
 * @param radians The radian angle to convert.
 * @return The angle in degrees.
 */
inline double rad2deg(double radians) { return radians * 180.0 / M_PI; }

/** \brief Convert the given radian angle to degrees.
 *
 * @param radians The radian angle to convert.
 * @return The angle in degrees.
 */
inline float rad2deg(float radians) { return (float)(radians * 180.0 / M_PI); }

/** \brief Convert the given degree angle to radian.
 *
 * @param degrees The degree angle to convert.
 * @return The radian angle.
 */
inline double deg2rad(double degrees) { return degrees * M_PI / 180.0; }

/** \brief Convert the given degree angle to radian.
 *
 * @param degrees The degree angle to convert.
 * @return The radian angle.
 */
inline float deg2rad(float degrees) { return (float)(degrees * M_PI / 180.0); }

/** \brief Calculate the squared difference of the given two points.
 *
 * @param a The first point.
 * @param b The second point.
 * @return The squared difference between point a and b.
 */
template <typename PointT>
inline float calcSquaredDiff(const PointT &a, const PointT &b) {
  float diffX = a.x - b.x;
  float diffY = a.y - b.y;
  float diffZ = a.z - b.z;

  return diffX * diffX + diffY * diffY + diffZ * diffZ;
}

/** \brief Calculate the squared difference of the given two points.
 *
 * @param a The first point.
 * @param b The second point.
 * @param wb The weighting factor for the SECOND point.
 * @return The squared difference between point a and b.
 */
template <typename PointT>
inline float calcSquaredDiff(const PointT &a, const PointT &b,
                             const float &wb) {
  float diffX = a.x - b.x * wb;
  float diffY = a.y - b.y * wb;
  float diffZ = a.z - b.z * wb;

  return diffX * diffX + diffY * diffY + diffZ * diffZ;
}

/** \brief Calculate the absolute distance of the point to the origin.
 *
 * @param p The point.
 * @return The distance to the point.
 */
template <typename PointT> inline float calcPointDistance(const PointT &p) {
  return std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}

/** \brief Calculate the angle difference of the given two points.
 *
 * @param a The first point.
 * @param b The second point.
 * @return The angle difference between point a and b in rad.
 */
template <typename PointT>
inline float calcCosAngleDiff(const PointT &a, const PointT &b) {
  float ab = a.x * b.x + a.y * b.y + a.z * b.z;
  float disab = calcPointDistance(a) * calcPointDistance(b);

  return ab / disab;
}

inline float calcCosAngleDiff(const Eigen::Vector3f &a, const Eigen::Vector3f &b) {
  float ab = a.dot(b);
  float disab = a.norm() * b.norm();

  return ab / disab;
}
/** \brief Calculate the squared distance of the point to the origin.
 *
 * @param p The point.
 * @return The squared distance to the point.
 */
template <typename PointT>
inline float calcSquaredPointDistance(const PointT &p) {
  return p.x * p.x + p.y * p.y + p.z * p.z;
}

// 计算点到直线(垂直平面)距离
template <typename PointT>
inline double point2line(const PointT p,const double A,const double B,const double C) {
  return fabs(p.x*A+p.y*B+C)/sqrt(A*A+B*B);
}

// 计算点到平面距离 (带符号)
template <typename PointT>
inline double point2plane(const PointT p,const double A,const double B,const double C,const double D) {
  int symble=p.x*A+p.y*B+p.z*C+D>0?1:(-1);
  return symble*fabs(p.x*A+p.y*B+p.z*C+D)/sqrt(A*A+B*B+C*C);
}

// 计算点到平面距离 (带符号)
template <typename PointT>
inline double point2plane(const PointT p,const PointT plane) {
  return point2plane(p,plane.normal_x,plane.normal_y,plane.normal_z,plane.curvature);
}

// 计算法向量斜率的角度值
// 输入：平面的平面方程
// 输出：法向量斜率的角度值
template <typename PointT>
inline double normalangle(const PointT plane){
  return atan(plane.normal_y/plane.normal_x)*180.0/M_PI;
}


// 计算两个平面间的夹角
// 输入：两个平面的平面方程
// 输出：两个平面的夹角(角度值)
template <typename PointT>
inline double planeangle(const PointT a,const PointT b){
  double ans;
	double tmp_top=fabs(a.normal_x*b.normal_x+a.normal_y*b.normal_y+a.normal_z*b.normal_z);
	double tmp_bottom=sqrt(a.normal_x*a.normal_x+a.normal_y*a.normal_y+a.normal_z*a.normal_z)
                    *sqrt(b.normal_x*b.normal_x+b.normal_y*b.normal_y+b.normal_z*b.normal_z);
	ans=acos(tmp_top/tmp_bottom)*180.0/M_PI;
	return ans;
}

#endif // LIDAR_MATH_UTILS_H