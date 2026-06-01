#ifndef XIMEA_ROS2_CAM__XIMEA_ROS2_CAM_HPP_
#define XIMEA_ROS2_CAM__XIMEA_ROS2_CAM_HPP_

#include <atomic>
#include <mutex>
#include <map>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <image_transport/image_transport.hpp>
#include <camera_info_manager/camera_info_manager.hpp>

#include <m3api/xiApi.h>
#include "ximea_ros2_cam/msg/xi_image_info.hpp"

namespace ximea_ros2_cam {

class XimeaRosCam : public rclcpp::Node {
public:
  explicit XimeaRosCam(const rclcpp::NodeOptions & options);
  virtual ~XimeaRosCam() override;

  XimeaRosCam(const XimeaRosCam &) = delete;
  XimeaRosCam & operator=(const XimeaRosCam &) = delete;

private:
  void openDeviceCallback();
  void frameCaptureCallback();
  void shutdown();
  bool configureCamera();
  
  rclcpp::Time hardwareToRosTime(uint32_t ts_sec, uint32_t ts_usec);

  static const std::map<std::string, int> & imageFormatMap();
  static const std::map<std::string, std::string> & encodingMap();
  static const std::map<std::string, int> & bytesPerPixelMap();

  rclcpp::TimerBase::SharedPtr open_device_timer_;
  rclcpp::TimerBase::SharedPtr frame_capture_timer_;

  image_transport::CameraPublisher image_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr image_count_pub_;
  rclcpp::Publisher<ximea_ros2_cam::msg::XiImageInfo>::SharedPtr xi_image_info_pub_;
  std::shared_ptr<camera_info_manager::CameraInfoManager> cam_info_manager_;

  HANDLE xi_handle_{nullptr};
  std::mutex camera_mutex_;
  
  std::atomic<bool> is_active_{false};
  std::atomic<uint64_t> frame_count_{0};
  std::atomic<bool> reconnect_pending_{false};

  // Hardware time anchoring state
  std::atomic<bool> hw_anchor_set_{false};
  rclcpp::Time hw_anchor_ros_time_{0, 0, RCL_ROS_TIME};
  uint64_t hw_anchor_us_{0};

  // Re-anchor timer (only created when use_hardware_timestamps_ && period > 0)
  rclcpp::TimerBase::SharedPtr reanchor_timer_;
  double hw_anchor_resync_period_s_{0.0};

  std::string camera_name_, serial_no_, frame_id_, format_, encoding_;
  int format_int_{0};
  int bytes_per_pixel_{1};
  double poll_open_period_s_{2.0};
  double poll_frame_period_s_{0.001};
  bool use_hardware_timestamps_{false};

  int fail_count_{0};

  bool publish_xi_image_info_{false};

  // --------- Camera configuration (loaded from parameters) ---------
  int    img_capture_timeout_ms_{1000};
  int    trigger_mode_{0};
  int    hw_trigger_edge_{0};
  bool   framerate_control_{false};
  int    framerate_set_{30};
  bool   auto_exposure_{true};
  int    exposure_time_us_{3000};
  double manual_gain_db_{0.0};
  int    auto_exposure_time_limit_us_{30000};
  double auto_exposure_priority_{0.8};
  double auto_gain_limit_db_{2.0};
  int    wb_mode_{0};
  double wb_kr_{1.0}, wb_kg_{1.0}, wb_kb_{1.0};
  int    roi_left_{0}, roi_top_{0}, roi_width_{0}, roi_height_{0};
  int    num_cams_in_bus_{1};
  double bw_safety_ratio_{0.9};
  int transport_buffer_commit_{32};
  std::string camera_info_url_;
};

} // namespace ximea_ros2_cam

#endif // XIMEA_ROS2_CAM__XIMEA_ROS2_CAM_HPP_