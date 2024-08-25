

#include <robosense_pointcloud/pointcloudXYZIR.h>
namespace robosense_pointcloud {
// change for PointXYZIT
void PointcloudXYZIR::addPoint(const float &x, const float &y, const float &z,
                               const double &timestamp,
                               const uint8_t &intensity, const uint16_t &ring,
                               const float &azimuth) {

  // convert polar coordinates to Euclidean XYZ
  robosense_rawdata::VPoint point;
  point.ring = ring;
  point.x = x;
  point.y = y;
  point.z = z;
  point.intensity = intensity;
  point.azimuth = azimuth;
  point.timestamp = timestamp;

  // append this point to the cloud
  pc->points.push_back(point);
  ++pc->width;
}

} // namespace robosense_pointcloud
