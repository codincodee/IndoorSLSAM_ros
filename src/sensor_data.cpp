#include <IndoorSLSAM_ros/sensor_data.h>
#include <cmath>

using namespace std;

namespace slsam_ros {
Scan2D::Scan2D() {}
Scan2D::Scan2D( shared_ptr < sensor_msgs :: LaserScan > msg)
  : scan(msg) {}
std::shared_ptr<slsam::Scan2D> Scan2D::ToSlsamScan() const {
  if (!scan) {
    return shared_ptr<slsam::Scan2D>();
  }
  std::shared_ptr<slsam::Scan2D> slsam_scan(new slsam::Scan2D);
  slsam_scan->points.reserve(scan->ranges.size());
  int i = -1;
  auto angle_min = scan->angle_min;
  auto angle_increment = scan->angle_increment;
  
  for (auto& range : scan->ranges) {
    ++i;
    if (range > scan->range_max || range < scan->range_min) {
      continue;
    }
    slsam::Point2 point;
    auto angle = angle_min + angle_increment * i;
    point.x = range * sin(angle);
    point.y = range * cos(angle);
    slsam_scan->points.push_back(point);
  }
  return slsam_scan;
}
} // namespace slsam_ros
