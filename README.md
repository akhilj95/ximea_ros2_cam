# ximea_ros2_cam

A ROS 2 driver for [XIMEA](https://www.ximea.com/) cameras (USB3 MQ/MC/MX series).
Built as a `rclcpp_components` node, it combines features from the [`wavelab/ximea_ros_cam`](https://github.com/wavelab/ximea_ros_cam) ROS 1 driver, with additional ideas borrowed from the [`African-Robotics-Unit/ximea_ROS2_driver`](https://github.com/African-Robotics-Unit/ximea_ROS2_driver) port.

---

## Features

- Full xiAPI Control: Configure format, ROI, triggers, exposure, white balance, and framerate.
- Multi-Camera Ready: Opens via serial number; automatically divides USB bandwidth across cameras on the same bus.
- **Hardware-timestamp anchoring**: optional `use_hardware_timestamps` mode anchors the camera's hardware clock to ROS time on the first frame and propagates accurate per-frame capture times for downstream sensor fusion.
- Auto-Reconnect: Automatically recovers if the USB connection drops.
- Publishes through `image_transport::CameraPublisher` so compressed transports (`compressed`, `theora`, etc.) work transparently.

---

## Tested Hardware

- MQ013RG-E2 (1.3 MP NIR-enhanced mono)
- MQ013CG-E2 (1.3 MP color, Bayer)

Should work with any xiAPI-supported XIMEA model on USB3 (MQ, MC, MX series). GigE Vision cameras (xiC, xiB) using the same xiAPI also work in principle but have not been verified here.

---

## Installation & Build

### Dependencies

- Ubuntu 22.04 (other Linux distros work if xiAPI is supported)
- ROS 2 Humble or newer (tested on Humble; should work on Iron/Jazzy with no changes)
- XIMEA Linux Software Package (xiAPI). Install via the official installer:
  ```bash
  wget https://www.ximea.com/getattachment/281fd5c5-3335-4279-a494-f49c004f00c6/XIMEA_Linux_SP.tgz
  tar -xzf XIMEA_Linux_SP.tgz
  cd package
  sudo ./install
  ```
- Add  user to the plugdev group.
  ```bash
  sudo gpasswd -a $USER plugdev
  ```
- Setup the USB FS Memory Max Allocation to Infinite (needed once per system boot)
  ```bash
   sudo sh -c 'echo 0 > /sys/module/usbcore/parameters/usbfs_memory_mb'
  ```

Run the following command to check the camera feed.
```bash
xiCamTool -L
```

### Build

```bash
cd ~/ros2_ws/src
git clone https://github.com/akhilj95/ximea_ros2_cam.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select ximea_ros2_cam --symlink-install
source install/setup.bash
```

---

## Quick Start

The bundled launch file brings up two cameras (NIR + RGB) under separate namespaces:

```bash
ros2 launch ximea_ros2_cam xiCam.launch.xml
```

Edit `launch/xiCam.launch.xml` to set your camera serial numbers, then per-camera parameters live in `config/nir_config.yaml` and `config/rgb_config.yaml`.

To run a single camera by hand:
```bash
ros2 run ximea_ros2_cam ximea_ros2_cam_node --ros-args \
    -p serial_no:="'12345678'" \
    -p camera_name:=my_ximea \
    -p image_format:=XI_MONO8 \
    -p framerate_control:=true -p framerate_set:=30
```

> Note: Numeric serial numbers MUST be wrapped in single quotes `"'12345678'"` to prevent type coercion.

---

## Published Topics

Under the node's namespace:

- `~/image_raw` (`sensor_msgs/Image`) — auto-advertises compressed formats.
- `~/camera_info` (`sensor_msgs/CameraInfo`)
- `~/image_count` (`std_msgs/UInt32`)
- `~/xi_image_info` (`ximea_ros2_cam/XiImageInfo`) — optional metadata.

## Core Parameters

- **Identity**: `serial_no`, `camera_name`, `frame_id`, `camera_info_url`
- **Format & ROI**: `image_format` (e.g. `XI_MONO8`, `XI_RGB24`, `XI_RAW8`), `roi_left`, `roi_top`, `roi_width`, `roi_height`
- **Timing & triggers**: `poll_open_period_s`, `trigger_mode` (0 = free-run, 2 = hardware), `hw_trigger_edge`, `framerate_control`, `framerate_set`
- **Exposure & WB**: `auto_exposure` (bool), `exposure_time_us`, `manual_gain_db`, `auto_exposure_time_limit_us`, `wb_mode` (0 = off, 1 = manual, 2 = auto)
- **USB & bandwidth**:
  - `num_cams_in_bus` — divides available bandwidth by this number.
  - `bw_safety_ratio` — safety margin (default `0.9`).
  - `transport_buffer_commit` — in-flight USB requests (default `32`).
- **Metadata**: `use_hardware_timestamps`, `publish_xi_image_info`

## Performance Tuning

Crucial for high FPS or high resolution streams.

1. **Increase USB memory limit:**


2. **Enable real-time priority** — prevents thread-scheduler warnings. Add `<user> - rtprio 99` to `/etc/security/limits.conf`.


## Troubleshooting

- **`ERROR: 11` (xiSetParam framerate)** — `XI_OUT_OF_RANGE`. Your requested framerate exceeds the bandwidth/exposure budget. Lower the framerate, reduce ROI, or switch to `XI_RAW8`.
- **`ERROR: 100` (limit_bandwidth_mode)** — harmless. Some models (e.g. MQ013) don't support the mode toggle, but the bandwidth limit itself still applies.
- **Negative topic delay** — expected when `use_hardware_timestamps: true`. The stamp represents true exposure time (in the past); ROS receives the message later.
- **"Unable to open camera calibration file"** — normal if uncalibrated. Run the standard ROS 2 camera calibrator to generate the file.
- **"Device already opened" on startup** — normal during multi-camera polling. Resolves itself once the other node claims its specific camera.

---

## Acknowledgments

- The original [`wavelab/ximea_ros_cam`](https://github.com/wavelab/ximea_ros_cam) ROS 1 driver — design and parameter model.
- The [`African-Robotics-Unit/ximea_ROS2_driver`](https://github.com/African-Robotics-Unit/ximea_ROS2_driver) ROS 2 port — reference for the xiAPI sequencing in ROS 2.
- XIMEA for the [Linux xiAPI](https://www.ximea.com/support/wiki/apis/XiAPI) and supporting documentation.

---

## License

BSD-3-Clause.