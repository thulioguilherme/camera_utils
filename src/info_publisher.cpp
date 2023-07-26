#include "info_publisher.hpp"

namespace camera_utils
{

/* onInit() //{ */

void InfoPublisher::onInit() {
  // obtain node handle
  ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();

  // waits for the ROS to publish clock
  ros::Time::waitForValid();

  // | ------------------- load ros parameters ------------------ |
  bool loaded_successfully = true;

  loaded_successfully &= nh.getParam("camera_name", _camera_name_);
  loaded_successfully &= nh.getParam("camera_frame", _camera_frame_);
  loaded_successfully &= nh.getParam("playing_rosbag", _playing_rosbag_);
  loaded_successfully &= nh.getParam("rate/publisher_camera_info", _rate_publisher_camera_info_);
  loaded_successfully &= nh.getParam("camera_calibration_file_path", _camera_calibration_file_path_);

  if (!loaded_successfully) {
    ROS_ERROR("[InfoPublisher]: failed to load non-optional parameters!");
    ros::shutdown();
  }

  // | -------------------- load camera info -------------------- |
  std::string camera_name_from_file;
  bool        loaded_camera_info = camera_calibration_parsers::readCalibration(_camera_calibration_file_path_, camera_name_from_file, camera_info_);

  if (!loaded_camera_info) {
    ROS_ERROR("[InfoPublisher]: failed to load camera info from file!");
    ros::shutdown();
  }

  // | ----------------- initialize subscribers ----------------- |
  if (_playing_rosbag_) {
    subscriber_clock_ = nh.subscribe("/clock", 1, &InfoPublisher::callbackSubscriberClock, this);
  }

  // | ----------------- initialize publishers ------------------ |
  publisher_camera_info_ = nh.advertise<sensor_msgs::CameraInfo>("camera_info_out", 1);

  // | ------------------- initialize timers -------------------- |
  timer_publisher_camera_info_ = nh.createTimer(ros::Rate(_rate_publisher_camera_info_), &InfoPublisher::callbackTimerPublisherCameraInfo, this);

  ROS_INFO_ONCE("[InfoPublisher]: initialized");

  is_initialized_ = true;
}

//}

// | ------------------- subscribers callbacks ------------------ |

/* callbackSubscriberClock() //{ */

void InfoPublisher::callbackSubscriberClock(const rosgraph_msgs::Clock::ConstPtr& msg) {
  if (!is_initialized_) {
    return;
  }

  {
    std::scoped_lock lock(mutex_rosbag_clock_);
    rosbag_clock_ = msg->clock;
  }
}

//}

// | --------------------- timer callbacks ---------------------- |

/* callbackTimerPublisherCameraInfo() //{ */

void InfoPublisher::callbackTimerPublisherCameraInfo([[maybe_unused]] const ros::TimerEvent& te) {
  if (!is_initialized_) {
    return;
  }

  sensor_msgs::CameraInfo out_msg;

  // write camera info loaded from file
  out_msg = camera_info_;

  // use the clock stamp if playing rosbag
  if (_playing_rosbag_) {
    std::scoped_lock lock(mutex_rosbag_clock_);
    out_msg.header.stamp = rosbag_clock_;
  } else {
    out_msg.header.stamp = ros::Time::now();
  }

  // set camera_frame
  out_msg.header.frame_id = _camera_frame_;

  // publish camera info
  try {
    publisher_camera_info_.publish(out_msg);
  }
  catch (...) {
    ROS_ERROR_THROTTLE(2, "[InfoPublisher]: exception caught during publishing topic %s", publisher_camera_info_.getTopic().c_str());
  }
}

//}

}  // namespace camera_utils

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(camera_utils::InfoPublisher, nodelet::Nodelet)
