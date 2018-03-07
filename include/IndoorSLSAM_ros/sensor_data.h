#ifndef INDOOR_SLSAM_SENSOR_DATA_H_
#define INDOOR_SLSAM_SENSOR_DATA_H_

#include <sensor_msgs/LaserScan.h>
#include <memory>
#include <IndoorSLSAM/slsam.h>
#include <nav_msgs/Odometry.h>

namespace slsam_ros {
struct Scan2D {
  Scan2D();
  Scan2D(std::shared_ptr<sensor_msgs::LaserScan> scan);
	std::shared_ptr<sensor_msgs::LaserScan> scan;
  std::shared_ptr<slsam::Scan2D> ToSlsamScan() const;
};

struct Odom {
  Odom();
  Odom(std::shared_ptr<nav_msgs::Odometry> odom);
  std::shared_ptr<nav_msgs::Odometry> odom;
  std::shared_ptr<slsam::Odom2D> ToSlsamOdom() const;
};
} // namespace slsam_ros

#endif // INDOOR_SLSAM_SENSOR_DATA_H_