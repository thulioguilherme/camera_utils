#ifndef CAMERA_UTILS_INFO_PUBLISHER_HPP
#define CAMERA_UTILS_INFO_PUBLISHER_HPP

/* includes //{ */

#include <nodelet/nodelet.h>
#include <ros/package.h>
#include <ros/ros.h>

#include <sensor_msgs/CameraInfo.h>

#include <rosgraph_msgs/Clock.h>

#include <camera_calibration_parsers/parse.h>

#include <mutex>

//}

namespace camera_utils
{

/* class InfoPublisher //{ */

class InfoPublisher : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  // flags
  bool is_initialized_{false};

  // ROS parameters
  bool                    _playing_rosbag_;
  std::string             _camera_name_;
  std::string             _camera_frame_;
  std::string             _camera_calibration_file_path_;
  sensor_msgs::CameraInfo camera_info_;

  // | ------------------- subscribers callbacks ------------------ |
  void            callbackSubscriberClock(const rosgraph_msgs::Clock::ConstPtr& msg);
  ros::Subscriber subscriber_clock_;
  ros::Time       rosbag_clock_;
  std::mutex      mutex_rosbag_clock_;

  // | --------------------- timer callbacks ---------------------- |
  void           callbackTimerPublisherCameraInfo(const ros::TimerEvent& te);
  int            _rate_publisher_camera_info_;
  ros::Timer     timer_publisher_camera_info_;
  ros::Publisher publisher_camera_info_;
};

//}

}  // namespace camera_utils

#endif
