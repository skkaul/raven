#include <iostream>
#include <ros/ros.h>
#include <gps_common/GPSFix.h>
//#include <gps_common/GPSStatus.h>

void callback(const gps_common::GPSFixConstPtr &fix) {
  // if (fix->status.status == gps_common::GPSStatus::STATUS_NO_FIX) {
  // std::cout << "Unable to get a fix on the location." << std::endl;
    // return;
 
  std:: cout << "Current Latitude: " << fix->latitude << std::endl;
  std::cout << "Current Longitude " << fix->longitude << std::endl;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "gps_subscriber");
  ros::NodeHandle nh;

  ros::Subscriber gps_sub = nh.subscribe("fix", 1, callback);

  ros::spin();
  return 0;
}
