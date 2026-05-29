#include "ximea_ros2_cam/ximea_ros2_cam.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <utility> // FIX: Brings in std::move

#include <rclcpp_components/register_node_macro.hpp>

namespace ximea_ros2_cam {

const std::map<std::string, int> & XimeaRosCam::imageFormatMap() {
  static const std::map<std::string, int> m = {
    {"XI_MONO8",      XI_MONO8},
    {"XI_MONO16",     XI_MONO16},
    {"XI_RGB24",      XI_RGB24},
    {"XI_RGB32",      XI_RGB32},
    {"XI_RGB_PLANAR", XI_RGB_PLANAR},
    {"XI_RAW8",       XI_RAW8},
    {"XI_RAW16",      XI_RAW16}
  };
  return m;
}

const std::map<std::string, std::string> & XimeaRosCam::encodingMap() {
  static const std::map<std::string, std::string> m = {
    {"XI_MONO8",      "mono8"},
    {"XI_MONO16",     "mono16"},
    {"XI_RGB24",      "bgr8"},
    {"XI_RGB32",      "bgra8"},
    {"XI_RGB_PLANAR", "not_applicable"},
    {"XI_RAW8",       "mono8"},
    {"XI_RAW16",      "mono16"}
  };
  return m;
}

const std::map<std::string, int> & XimeaRosCam::bytesPerPixelMap() {
  static const std::map<std::string, int> m = {
    {"XI_MONO8",      1},
    {"XI_MONO16",     2},
    {"XI_RGB24",      3},
    {"XI_RGB32",      4},
    {"XI_RGB_PLANAR", 3},
    {"XI_RAW8",       1},
    {"XI_RAW16",      2}
  };
  return m;
}

XimeaRosCam::XimeaRosCam(const rclcpp::NodeOptions & options)
: Node("ximea_cam_node", options) 
{
  // identity / topics
  serial_no_       = this->declare_parameter("serial_no", "");
  camera_name_     = this->declare_parameter("camera_name", "ximea_camera");
  frame_id_        = this->declare_parameter("frame_id", "camera_optical_frame");
  camera_info_url_ = this->declare_parameter("camera_info_url", "");

  // image format
  format_ = this->declare_parameter("image_format", "XI_MONO8");

  // timing
  poll_open_period_s_      = this->declare_parameter("poll_open_period_s", 2.0);
  poll_frame_period_s_     = this->declare_parameter("poll_frame_period_s", 0.001);
  img_capture_timeout_ms_  = this->declare_parameter("img_capture_timeout_ms", 1000);

  // triggering
  trigger_mode_     = this->declare_parameter("trigger_mode", 0);
  hw_trigger_edge_  = this->declare_parameter("hw_trigger_edge", 0);

  // framerate
  framerate_control_ = this->declare_parameter("framerate_control", false);
  framerate_set_     = this->declare_parameter("framerate_set", 30);

  // exposure / gain
  auto_exposure_                = this->declare_parameter("auto_exposure", true);
  exposure_time_us_             = this->declare_parameter("exposure_time_us", 3000);
  manual_gain_db_               = this->declare_parameter("manual_gain_db", 0.0);
  auto_exposure_priority_       = this->declare_parameter("auto_exposure_priority", 0.8);
  auto_exposure_time_limit_us_  = this->declare_parameter("auto_exposure_time_limit_us", 30000);
  auto_gain_limit_db_           = this->declare_parameter("auto_gain_limit_db", 2.0);

  // white balance
  wb_mode_ = this->declare_parameter("wb_mode", 0);
  wb_kr_   = this->declare_parameter("wb_kr", 1.0);
  wb_kg_   = this->declare_parameter("wb_kg", 1.0);
  wb_kb_   = this->declare_parameter("wb_kb", 1.0);

  // ROI (0 width/height means "use full sensor")
  roi_left_   = this->declare_parameter("roi_left", 0);
  roi_top_    = this->declare_parameter("roi_top", 0);
  roi_width_  = this->declare_parameter("roi_width", 0);
  roi_height_ = this->declare_parameter("roi_height", 0);

  // bandwidth
  num_cams_in_bus_ = this->declare_parameter("num_cams_in_bus", 1);
  bw_safety_ratio_ = this->declare_parameter("bw_safety_ratio", 0.9);
  if (bw_safety_ratio_ <= 0.0 || bw_safety_ratio_ > 1.0) {
    RCLCPP_WARN(get_logger(),
      "bw_safety_ratio=%.3f out of (0,1]; clamping to 0.9", bw_safety_ratio_);
    bw_safety_ratio_ = 0.9;
  }

  // toggles
  use_hardware_timestamps_ = this->declare_parameter("use_hardware_timestamps", false);
  publish_xi_image_info_   = this->declare_parameter("publish_xi_image_info", false);

  auto & fmt_map = imageFormatMap();
  auto & enc_map = encodingMap();
  auto & bpp_map = bytesPerPixelMap();

  if (fmt_map.find(format_) == fmt_map.end()) {
    RCLCPP_ERROR(this->get_logger(), "Invalid format '%s'. Defaulting to XI_MONO8", format_.c_str());
    format_ = "XI_MONO8";
  }

  format_int_      = fmt_map.at(format_);
  encoding_        = enc_map.at(format_);
  bytes_per_pixel_ = bpp_map.at(format_);

  auto qos = rmw_qos_profile_sensor_data;
  image_pub_ = image_transport::create_camera_publisher(this, "image_raw", qos);
  
  image_count_pub_ = this->create_publisher<std_msgs::msg::UInt32>("image_count", 10);
  xi_image_info_pub_ = this->create_publisher<ximea_ros2_cam::msg::XiImageInfo>("xi_image_info", 10);

  cam_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(this, camera_name_);

  if (!camera_info_url_.empty() &&
      cam_info_manager_->validateURL(camera_info_url_) &&
      cam_info_manager_->loadCameraInfo(camera_info_url_)) {
    RCLCPP_INFO(get_logger(), "Loaded camera info from %s", camera_info_url_.c_str());
  }

  xiSetParamFloat(xi_handle_, XI_PRM_AE_MAX_LIMIT,
                static_cast<float>(auto_exposure_time_limit_us_));

  open_device_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(poll_open_period_s_),
    std::bind(&XimeaRosCam::openDeviceCallback, this));
    
  RCLCPP_INFO(this->get_logger(), "Driver initialized. Polling for XIMEA camera Serial=%s", serial_no_.c_str());
}

XimeaRosCam::~XimeaRosCam() {
  shutdown();
}

void XimeaRosCam::shutdown() {
  is_active_ = false;

  if (open_device_timer_) {
    open_device_timer_->cancel();
    open_device_timer_.reset();
  }
  if (frame_capture_timer_) {
    frame_capture_timer_->cancel();
    frame_capture_timer_.reset();
  }

  std::lock_guard<std::mutex> lock(camera_mutex_);
  if (xi_handle_) {
    xiStopAcquisition(xi_handle_);
    xiCloseDevice(xi_handle_);
    xi_handle_ = nullptr;
    RCLCPP_INFO(this->get_logger(), "Successfully closed XIMEA hardware device connection.");
  }
}

void XimeaRosCam::openDeviceCallback() {
  std::lock_guard<std::mutex> lock(camera_mutex_);
  if (xi_handle_) return;

  XI_RETURN stat;
  if (serial_no_.empty()) {
    stat = xiOpenDevice(0, &xi_handle_);
  } else {
    stat = xiOpenDeviceBy(XI_OPEN_BY_SN, serial_no_.c_str(), &xi_handle_);
  }

  if (stat == XI_OK && xi_handle_) {
    open_device_timer_->cancel();
    
    if (!configureCamera()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to configure camera parameters. Aborting.");
      xiCloseDevice(xi_handle_);
      xi_handle_ = nullptr;
      return;
    }

    XI_RETURN as = xiStartAcquisition(xi_handle_);
    if (as != XI_OK) {
      RCLCPP_ERROR(get_logger(), "xiStartAcquisition failed with status %d", as);
      xiCloseDevice(xi_handle_);
      xi_handle_ = nullptr;
      return;
    }
    is_active_ = true;

    frame_capture_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(poll_frame_period_s_),
      std::bind(&XimeaRosCam::frameCaptureCallback, this));
      
    RCLCPP_INFO(this->get_logger(), "Successfully connected and started acquisition on camera SN=%s", serial_no_.c_str());
  }
}

namespace {
  // Tiny status helper: logs and returns false on failure.
  bool checkXi(rclcpp::Logger logger, XI_RETURN s, const char * what) {
    if (s == XI_OK) return true;
    RCLCPP_ERROR(logger, "xiAPI: %s failed with status %d", what, s);
    return false;
  }
}  // namespace

bool XimeaRosCam::configureCamera() {
  auto log = this->get_logger();
  XI_RETURN s;

  // -------- Image format (do FIRST; some bayer/RAW modes change reported maxes) --------
  s = xiSetParamInt(xi_handle_, XI_PRM_IMAGE_DATA_FORMAT, format_int_);
  if (!checkXi(log, s, "set XI_PRM_IMAGE_DATA_FORMAT")) return false;

  // -------- ROI: query sensor maxes, fall back to full sensor on 0 --------
  int sensor_max_w = 0, sensor_max_h = 0;
  xiGetParamInt(xi_handle_, XI_PRM_WIDTH  XI_PRM_INFO_MAX, &sensor_max_w);
  xiGetParamInt(xi_handle_, XI_PRM_HEIGHT XI_PRM_INFO_MAX, &sensor_max_h);
  if (roi_width_  <= 0) roi_width_  = sensor_max_w;
  if (roi_height_ <= 0) roi_height_ = sensor_max_h;
  if (roi_left_ < 0 || roi_top_ < 0 ||
      roi_left_ + roi_width_  > sensor_max_w ||
      roi_top_  + roi_height_ > sensor_max_h) {
    RCLCPP_ERROR(log, "ROI out of bounds: left=%d top=%d w=%d h=%d sensor=%dx%d",
                 roi_left_, roi_top_, roi_width_, roi_height_, sensor_max_w, sensor_max_h);
    return false;
  }
  // Width/Height first, then offsets — xiAPI is picky about order.
  xiSetParamInt(xi_handle_, XI_PRM_WIDTH,    roi_width_);
  xiSetParamInt(xi_handle_, XI_PRM_HEIGHT,   roi_height_);
  xiSetParamInt(xi_handle_, XI_PRM_OFFSET_X, roi_left_);
  xiSetParamInt(xi_handle_, XI_PRM_OFFSET_Y, roi_top_);

  // -------- Triggering --------
  switch (trigger_mode_) {
    case 2:  // hardware
      xiSetParamInt(xi_handle_, XI_PRM_TRG_SOURCE,
        (hw_trigger_edge_ == 1) ? XI_TRG_EDGE_FALLING : XI_TRG_EDGE_RISING);
      xiSetParamInt(xi_handle_, XI_PRM_GPI_SELECTOR, 1);
      xiSetParamInt(xi_handle_, XI_PRM_GPI_MODE,    XI_GPI_TRIGGER);
      break;
    case 1:  // software (not fully wired — needs xiSetParamInt(.., XI_PRM_TRG_SOFTWARE, 1) per shot)
      xiSetParamInt(xi_handle_, XI_PRM_TRG_SOURCE, XI_TRG_SOFTWARE);
      break;
    default:
      xiSetParamInt(xi_handle_, XI_PRM_TRG_SOURCE, XI_TRG_OFF);
      break;
  }

  // -------- Exposure / gain (THIS is what was missing — primary cause of black images) --------
  if (auto_exposure_) {
    xiSetParamInt(xi_handle_,   XI_PRM_AEAG, 1);
    xiSetParamFloat(xi_handle_, XI_PRM_EXP_PRIORITY,
                    static_cast<float>(auto_exposure_priority_));
    xiSetParamFloat(xi_handle_, XI_PRM_AE_MAX_LIMIT,
                static_cast<float>(auto_exposure_time_limit_us_));
    xiSetParamFloat(xi_handle_, XI_PRM_AG_MAX_LIMIT,
                    static_cast<float>(auto_gain_limit_db_));
    RCLCPP_INFO(log, "Auto-exposure ON (max %d us, gain max %.2f dB, priority %.2f)",
                auto_exposure_time_limit_us_, auto_gain_limit_db_, auto_exposure_priority_);
  } else {
    xiSetParamInt(xi_handle_,   XI_PRM_AEAG, 0);
    xiSetParamInt(xi_handle_,   XI_PRM_EXPOSURE, exposure_time_us_);
    xiSetParamFloat(xi_handle_, XI_PRM_GAIN, static_cast<float>(manual_gain_db_));
    RCLCPP_INFO(log, "Manual exposure %d us, gain %.2f dB",
                exposure_time_us_, manual_gain_db_);
  }

  // -------- White balance (color sensors only; harmless on mono) --------
  switch (wb_mode_) {
    case 2:
      xiSetParamInt(xi_handle_, XI_PRM_AUTO_WB, 1);
      RCLCPP_INFO(log, "White balance: auto");
      break;
    case 1:
      xiSetParamInt(xi_handle_,   XI_PRM_AUTO_WB, 0);
      xiSetParamFloat(xi_handle_, XI_PRM_WB_KR, static_cast<float>(wb_kr_));
      xiSetParamFloat(xi_handle_, XI_PRM_WB_KG, static_cast<float>(wb_kg_));
      xiSetParamFloat(xi_handle_, XI_PRM_WB_KB, static_cast<float>(wb_kb_));
      RCLCPP_INFO(log, "White balance manual: KR=%.3f KG=%.3f KB=%.3f", wb_kr_, wb_kg_, wb_kb_);
      break;
    default:
      xiSetParamInt(xi_handle_, XI_PRM_AUTO_WB, 0);
      break;
  }

  // -------- Bandwidth limiting (important for >1 camera on the same USB controller) --------
  int avail_bw_mbps = 0;
  xiGetParamInt(xi_handle_, XI_PRM_AVAILABLE_BANDWIDTH, &avail_bw_mbps);
  if (num_cams_in_bus_ > 1) avail_bw_mbps /= num_cams_in_bus_;
  const int limit_mbps = static_cast<int>(avail_bw_mbps * bw_safety_ratio_);
  RCLCPP_INFO(log, "Bandwidth: available=%d Mbps, cams_in_bus=%d -> limit=%d Mbps",
              avail_bw_mbps, num_cams_in_bus_, limit_mbps);
  xiSetParamInt(xi_handle_, XI_PRM_LIMIT_BANDWIDTH,      limit_mbps);
  xiSetParamInt(xi_handle_, XI_PRM_LIMIT_BANDWIDTH_MODE, XI_ON);

  // -------- Framerate (only meaningful in free-run) --------
  if (trigger_mode_ == 0 && framerate_control_) {
    xiSetParamInt(xi_handle_, XI_PRM_ACQ_TIMING_MODE, XI_ACQ_TIMING_MODE_FRAME_RATE);
    xiSetParamInt(xi_handle_, XI_PRM_FRAMERATE, framerate_set_);
    RCLCPP_INFO(log, "Framerate locked to %d Hz", framerate_set_);
  } else if (trigger_mode_ == 0) {
    xiSetParamInt(xi_handle_, XI_PRM_ACQ_TIMING_MODE, XI_ACQ_TIMING_MODE_FREE_RUN);
  }

  // -------- Buffer tuning for high framerates --------
  int qsize_max = 0;
  xiGetParamInt(xi_handle_, XI_PRM_BUFFERS_QUEUE_SIZE XI_PRM_INFO_MAX, &qsize_max);
  if (qsize_max > 0) {
    xiSetParamInt(xi_handle_, XI_PRM_BUFFERS_QUEUE_SIZE, qsize_max);
  }

  return true;
}

rclcpp::Time XimeaRosCam::hardwareToRosTime(uint32_t ts_sec, uint32_t ts_usec) {
  const uint64_t hw_us = static_cast<uint64_t>(ts_sec) * 1000000ULL + ts_usec;
  if (!hw_anchor_set_) {
    hw_anchor_ros_time_ = this->now();
    hw_anchor_us_ = hw_us;
    hw_anchor_set_ = true;
    return hw_anchor_ros_time_;
  }
  const int64_t delta_ns = static_cast<int64_t>(hw_us - hw_anchor_us_) * 1000LL;
  return hw_anchor_ros_time_ + rclcpp::Duration::from_nanoseconds(delta_ns);
}

void XimeaRosCam::frameCaptureCallback() {
  if (!is_active_) return;

  XI_IMG xi_img;
  std::memset(&xi_img, 0, sizeof(xi_img));
  xi_img.size = sizeof(XI_IMG);

  {
    std::lock_guard<std::mutex> lock(camera_mutex_);
    if (!xi_handle_ || xiGetImage(xi_handle_, img_capture_timeout_ms_, &xi_img) != XI_OK) {
      return;
    }
  }

  rclcpp::Time stamp = use_hardware_timestamps_ ? 
    hardwareToRosTime(xi_img.tsSec, xi_img.tsUSec) : this->now();

  sensor_msgs::msg::Image img_msg;
  img_msg.header.stamp    = stamp;
  img_msg.header.frame_id = frame_id_;
  img_msg.height          = xi_img.height;
  img_msg.width           = xi_img.width;
  img_msg.encoding        = encoding_;
  img_msg.step            = xi_img.width * bytes_per_pixel_;
  
  size_t payload_bytes    = img_msg.step * xi_img.height;
  img_msg.data.resize(payload_bytes);
  std::memcpy(img_msg.data.data(), xi_img.bp, payload_bytes);

  sensor_msgs::msg::CameraInfo info_msg = cam_info_manager_->getCameraInfo();
  info_msg.header.stamp    = stamp;
  info_msg.header.frame_id = frame_id_;
  if (info_msg.width == 0) {
    info_msg.width  = xi_img.width;
    info_msg.height = xi_img.height;
  }

  image_pub_.publish(img_msg, info_msg);

  std_msgs::msg::UInt32 count_msg;
  count_msg.data = ++frame_count_;
  image_count_pub_->publish(count_msg);

  if (publish_xi_image_info_) {
    auto info_msg_custom = std::make_unique<ximea_ros2_cam::msg::XiImageInfo>();
    info_msg_custom->header.stamp           = stamp;
    info_msg_custom->header.frame_id        = frame_id_;
    info_msg_custom->size                   = xi_img.size;
    info_msg_custom->bp_size                = xi_img.bp_size;
    info_msg_custom->frm                    = xi_img.frm;
    info_msg_custom->width                  = xi_img.width;
    info_msg_custom->height                 = xi_img.height;
    info_msg_custom->nframe                 = xi_img.nframe;
    info_msg_custom->acq_nframe             = xi_img.acq_nframe;
    info_msg_custom->ts_sec                 = xi_img.tsSec;
    info_msg_custom->ts_usec                = xi_img.tsUSec;
    info_msg_custom->gpi_level              = xi_img.GPI_level;
    info_msg_custom->black_level            = xi_img.black_level;
    info_msg_custom->padding_x              = xi_img.padding_x;
    info_msg_custom->absolute_offset_x      = xi_img.AbsoluteOffsetX;
    info_msg_custom->absolute_offset_y      = xi_img.AbsoluteOffsetY;
    info_msg_custom->exposure_time_us       = xi_img.exposure_time_us;
    info_msg_custom->gain_db                = xi_img.gain_db;
    info_msg_custom->image_user_data        = xi_img.image_user_data;
    
    xi_image_info_pub_->publish(std::move(info_msg_custom));
  }
}

} // namespace ximea_ros2_cam

// EXPORT MACRO: Must always be declared outside of all namespace scopes!
RCLCPP_COMPONENTS_REGISTER_NODE(ximea_ros2_cam::XimeaRosCam)