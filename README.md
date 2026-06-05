# ximea_ros2_cam (ROS 2 XIMEA Camera Driver)

A ROS 2 driver for [XIMEA](https://www.ximea.com/) cameras (USB3 MQ/MC/MX series).
Built as a `rclcpp_components` node, it combines features from the [`wavelab/ximea_ros_cam`](https://github.com/wavelab/ximea_ros_cam) ROS 1 driver, with additional ideas borrowed from the [`African-Robotics-Unit/ximea_ROS2_driver`](https://github.com/African-Robotics-Unit/ximea_ROS2_driver) port.

---

## Features

- Full xiAPI Control: Configure format, ROI, triggers, exposure, white balance, and framerate.
- Multi-Camera Ready: Opens via serial number; automatically divides USB bandwidth across cameras on the same bus.
- **Hardware-timestamp anchoring** *(opt-in via `use_hardware_timestamps`)*: stamps derived from the camera's hardware clock with periodic re-anchoring to bound drift. Gives microsecond-precise inter-frame intervals immune to host-side scheduling jitter — see [Timestamps](#timestamps).
- Auto-Reconnect: Automatically recovers if the USB connection drops.
* Flexible image transport via `image_transport`: supports raw, JPEG, and PNG formats.

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

All topics are published under the node's namespace.

| Topic                  | Type                              | Description |
|------------------------|-----------------------------------|-------------|
| `image_raw`            | `sensor_msgs/msg/Image`           | Raw image stream, sensor_data QoS. Compressed variants (`image_raw/compressed`, etc.) are auto-advertised by `image_transport`. |
| `camera_info`          | `sensor_msgs/msg/CameraInfo`      | Published alongside each image via `CameraPublisher`. Calibration content from `camera_info_url` if provided. |
| `image_count`          | `std_msgs/msg/UInt32`             | Monotonic frame counter (driver-side). |
| `xi_image_info`        | `ximea_ros2_cam/msg/XiImageInfo`  | Per-frame xiAPI metadata. Only published when `publish_xi_image_info: true`. |


## Core Parameters

- **Identity**: `serial_no`, `camera_name`, `frame_id`, `camera_info_url`
- **Format & ROI**: `image_format` (e.g. `XI_MONO8`, `XI_RGB24`, `XI_RAW8`), `roi_left`, `roi_top`, `roi_width`, `roi_height`
- **Timing & triggers**: `poll_open_period_s`, `trigger_mode` (0 = free-run, 2 = hardware), `hw_trigger_edge`, `framerate_control`, `framerate_set`
- **Exposure & WB**: `auto_exposure` (bool), `exposure_time_us`, `manual_gain_db`, `auto_exposure_time_limit_us`, `wb_mode` (0 = off, 1 = manual, 2 = auto)
- **USB & bandwidth**:
  - `num_cams_in_bus` — divides available bandwidth by this number.
  - `bw_safety_ratio` — safety margin (default `0.9`).
  - `transport_buffer_commit` — in-flight USB requests (default `32`).
- **Metadata**: `use_hardware_timestamps`, `hw_anchor_resync_period_s`, `publish_xi_image_info`

For additional details check [`docs/parameters.md`](https://github.com/akhilj95/ximea_ros2_cam/tree/main/docs/parameters.md)


## Performance Tuning

- If your machine has multiple USB controllers, put one camera on each — that doubles the usable bandwidth versus sharing a single controller. `lsusb -t` shows the controller topology.
- Place the following in `/etc/security/limits.conf` to make the Ximea camera driver have real time priority
  ```
  *               -       rtprio          0
  @realtime       -       rtprio          81
  *               -       nice            0
  @realtime       -       nice            -16
  ```
  Then add the current user to the group `realtime`:
  ```
  sudo groupadd realtime
  sudo gpasswd -a $USER realtime
  ```

## Timestamps

The driver supports two timestamp modes via `use_hardware_timestamps`:

**`use_hardware_timestamps: false` (default)** — image headers carry `node->now()` at message-construction time. Clean to interpret. The stamp marks the moment the driver received the image, after exposure, USB transfer, and processing.

**`use_hardware_timestamps: true`** — stamps are derived from the camera's hardware clock, anchored to ROS time. Inter-frame intervals are microsecond-precise and immune to host-side scheduling jitter (useful for high-rate timing analysis or sensor fusion where the *relative* timing between frames matters).

What hardware timestamps do **not** give you: a true "absolute exposure start in wallclock time". The anchor is set when the driver's callback runs (i.e. after delivery), so the delivery latency of the anchor frame is baked into every subsequent stamp. The mode preserves frame-to-frame precision, not absolute pipeline latency.

### Re-anchoring

Camera and host crystals tick at slightly different rates (typically ~10 ppm). Left alone, the anchor would drift indefinitely. The driver re-anchors every `hw_anchor_resync_period_s` seconds (default `60`) to keep drift bounded. Set to `0` to disable re-anchoring — fine for short runs or hardware-triggered setups where absolute drift doesn't matter.


## Troubleshooting

- **`ERROR: 11` (xiSetParam framerate)** — `XI_OUT_OF_RANGE`. Your requested framerate exceeds the bandwidth/exposure budget. Lower the framerate, reduce ROI, or switch to `XI_RAW8`.
- **`ERROR: 100` (limit_bandwidth_mode)** — harmless. Some models (e.g. MQ013) don't support the mode toggle, but the bandwidth limit itself still applies.
- **Negative delay during startup or with `hw_anchor_resync_period_s: 0`** — first-frame warmup gets baked into the single anchor. Enable re-anchoring (default `60.0`) or wait one re-anchor period.
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
