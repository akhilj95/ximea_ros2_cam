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
  After installing the XIMEA's drivers, run the following command to check the camera feed.
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

---

## Parameters

### Identity

| Parameter         | Type   | Default                   | Description |
|-------------------|--------|---------------------------|-------------|
| `serial_no`       | string | `""`                      | XIMEA camera serial. Empty string opens device 0. |
| `camera_name`     | string | `"ximea_camera"`          | Name for `camera_info_manager` and default URL. |
| `frame_id`        | string | `"camera_optical_frame"`  | TF frame_id stamped in image/CameraInfo headers. |
| `camera_info_url` | string | `""`                      | URL of calibration YAML. Empty falls back to `~/.ros/camera_info/<camera_name>.yaml`. |

### Image format

| Parameter      | Type   | Default     | Description |
|----------------|--------|-------------|-------------|
| `image_format` | string | `XI_MONO8`  | One of `XI_MONO8`, `XI_MONO16`, `XI_RGB24`, `XI_RGB32`, `XI_RGB_PLANAR`, `XI_RAW8`, `XI_RAW16`. |

### Timing

| Parameter                | Type   | Default | Description |
|--------------------------|--------|---------|-------------|
| `poll_open_period_s`     | double | `2.0`   | Period of the open-device polling timer (seconds). Stagger this across cameras to avoid race on USB enumeration. |
| `poll_frame_period_s`    | double | `0.001` | Period of the frame-grab timer. Set well below your target frame interval. |
| `img_capture_timeout_ms` | int    | `1000`  | Per-frame `xiGetImage` timeout in milliseconds. |

### Triggering

| Parameter         | Type | Default | Description |
|-------------------|------|---------|-------------|
| `trigger_mode`    | int  | `0`     | `0` = free-run, `1` = software (not fully wired in this version), `2` = hardware trigger. |
| `hw_trigger_edge` | int  | `0`     | When `trigger_mode=2`: `0` = rising edge, `1` = falling edge. GPI input pin 1. |

### Framerate

| Parameter           | Type | Default | Description |
|---------------------|------|---------|-------------|
| `framerate_control` | bool | `false` | If `true` and `trigger_mode=0`, locks the camera to `framerate_set`. Otherwise free-run. |
| `framerate_set`     | int  | `30`    | Target framerate (Hz). Must be achievable given bandwidth and exposure; otherwise xiAPI rejects with `XI_OUT_OF_RANGE`. |

### Exposure and gain

| Parameter                     | Type   | Default | Description |
|-------------------------------|--------|---------|-------------|
| `auto_exposure`               | bool   | `true`  | Enable `XI_PRM_AEAG`. |
| `exposure_time_us`            | int    | `3000`  | Manual exposure in µs (used when `auto_exposure: false`). |
| `manual_gain_db`              | double | `0.0`   | Manual gain in dB. |
| `auto_exposure_priority`      | double | `0.8`   | Auto-exposure priority (0.5–1.0; 1.0 = use only exposure, gain held). |
| `auto_exposure_time_limit_us` | int    | `30000` | Auto-exposure max exposure time. |
| `auto_gain_limit_db`          | double | `2.0`   | Auto-exposure max gain. |

### White balance (color sensors only)

| Parameter | Type   | Default | Description |
|-----------|--------|---------|-------------|
| `wb_mode` | int    | `0`     | `0` = off, `1` = manual coefficients (`wb_kr/kg/kb`), `2` = auto. |
| `wb_kr`   | double | `1.0`   | Manual red gain coefficient. |
| `wb_kg`   | double | `1.0`   | Manual green gain coefficient. |
| `wb_kb`   | double | `1.0`   | Manual blue gain coefficient. |

### Region of interest

| Parameter    | Type | Default | Description |
|--------------|------|---------|-------------|
| `roi_left`   | int  | `0`     | ROI X offset. |
| `roi_top`    | int  | `0`     | ROI Y offset. |
| `roi_width`  | int  | `0`     | ROI width. `0` = full sensor width. |
| `roi_height` | int  | `0`     | ROI height. `0` = full sensor height. |

Values are snapped down to the nearest valid increment for the sensor.

### Bandwidth and performance

| Parameter                  | Type   | Default | Description |
|----------------------------|--------|---------|-------------|
| `num_cams_in_bus`          | int    | `1`     | Number of cameras sharing the same USB controller. Available bandwidth is divided by this. |
| `bw_safety_ratio`          | double | `0.9`   | Safety factor applied to allocated bandwidth (0 < r ≤ 1). Clamped if out of range. |
| `transport_buffer_commit`  | int    | `32`    | `XI_PRM_ACQ_TRANSPORT_BUFFER_COMMIT` — number of in-flight USB requests. 32 is XIMEA's recommended value for Linux USB3. |

### Timestamps and metadata

| Parameter                 | Type | Default | Description |
|---------------------------|------|---------|-------------|
| `use_hardware_timestamps` | bool | `false` | Use hardware timestamps (anchored to ROS time on first frame). Otherwise stamps with `node->now()` at receive time. |
| `publish_xi_image_info`   | bool | `false` | Publish the `XiImageInfo` topic with per-frame xiAPI metadata. |

---

## Multi-Camera Setup

Each `XimeaRosCam` instance opens one camera by serial number. To run several:

- Launch one node per camera, each in its own namespace.
- Stagger `poll_open_period_s` so the cameras don't try to enumerate simultaneously (e.g., 2.0 s and 3.0 s).
- Set `num_cams_in_bus` to the number of cameras sharing the USB controller. The driver divides the auto-measured bandwidth by this value before applying `XI_PRM_LIMIT_BANDWIDTH`.

If your machine has multiple USB controllers, put one camera on each — that doubles the usable bandwidth versus sharing a single controller. `lsusb -t` shows the controller topology.

The bundled `launch/xiCam.launch.xml` is a worked example for a 2-camera (NIR + RGB) setup.

---

## Performance Tuning

For high framerate or high resolution streams, three OS-level tweaks make a real difference:

### 1. Increase the USB filesystem memory limit

The xiAPI driver allocates large pinned USB buffers. The default kernel limit (`usbcore.usbfs_memory_mb=16`) is far too small; you'll see this in the log otherwise:

```
xiAPI: PoolAllocUSB30: zerocopy not available
```

Fix permanently by adding to `/etc/default/grub`:
```
GRUB_CMDLINE_LINUX_DEFAULT="... usbcore.usbfs_memory_mb=1024"
```
then `sudo update-grub && reboot`.

Or apply transiently:
```bash
sudo sh -c 'echo 1024 > /sys/module/usbcore/parameters/usbfs_memory_mb'
```

The XIMEA installer normally sets this — verify with `cat /sys/module/usbcore/parameters/usbfs_memory_mb`.

### 2. Real-time priority for the xiAPI worker thread

You'll see this otherwise:
```
xiAPI: Failed to change thread scheduler, check user limit for realtime priority.
```

Add to `/etc/security/limits.conf` (replace `<user>` with your account):
```
<user>  -  rtprio  99
<user>  -  nice    -10
```
Log out and back in.

### 3. Bandwidth math for color cameras

`XI_RGB24` is bandwidth-expensive: 3 bytes/pixel × resolution × framerate. For a 1280×1024 sensor at 30 fps in RGB24 you need ~118 MB/s on the USB bus — more than half of a single USB3 controller's *theoretical* bandwidth and well above what you usually get in practice.

If you're bandwidth-limited and the framerate target keeps being rejected by xiAPI (`XI_OUT_OF_RANGE`), the easiest fix is:

- Use `image_format: XI_RAW8` to publish 1 byte/pixel Bayer data, then demosaic downstream with `image_proc` or `image_pipeline`. This gives you ~3× the framerate at the same resolution.
- Or reduce ROI.
- Or put the camera on its own USB controller and set `num_cams_in_bus: 1`.

---

## Troubleshooting

### `parameter 'serial_no' has invalid type ... is of type {string}, setting it to {integer} is not allowed`

Numeric-looking serial numbers in ROS 2 launch XML get auto-typed as integers. Wrap the value in YAML-style single quotes:
```xml
<param name="serial_no" value="'41102142'" />
```
The deprecated `type="str"` attribute does *not* fix this — it's ignored on modern ROS 2 distros.

### `xiSetParam (framerate) Finished with ERROR: 11`

`XI_OUT_OF_RANGE` — the requested framerate is unachievable at the current ROI, format, and bandwidth budget. The driver falls back to free-run mode and logs a warning with the maximum framerate xiAPI reports as achievable. Lower `framerate_set`, reduce ROI, or switch to a lighter pixel format.

### `xiSetParam (limit_bandwidth_mode) Finished with ERROR: 100`

`XI_NOT_SUPPORTED` — the `XI_PRM_LIMIT_BANDWIDTH_MODE` parameter is not implemented on some camera models (notably the MQ013 series). Harmless: `XI_PRM_LIMIT_BANDWIDTH` is still applied. The log line is from xiAPI itself.

### Negative `ros2 topic delay`

Expected behavior with `use_hardware_timestamps: true`. The hardware timestamp reflects the moment of exposure start, which is in the past relative to when the message reaches `ros2 topic delay`. The magnitude (≈ exposure + USB transfer time) is the actual sensor-to-message latency. If you want stamps that are always `≈ 0` delay, set `use_hardware_timestamps: false`.

### `Unable to open camera calibration file [...ximea_<name>.yaml]`

Empty `camera_info_url`, no calibration file present at the default location. Either:
- Run `ros2 run camera_calibration cameracalibrator` and save the result to `~/.ros/camera_info/<camera_name>.yaml`, or
- Set `camera_info_url` in the launch file to point at an existing YAML.

The driver runs fine without calibration — `camera_info` is just published with empty intrinsics.

### Camera not opening / `xiOpenDeviceBy` keeps failing

```
xiAPI: OpenDeviceUSB30 ERROR opening driver - Flag set - device already opened
```

This is normal *during the polling phase* of multi-camera startup — one node has the device exclusively while another is also probing. The retry will succeed once the conflicting probe stops. If it persists:
- Check the camera shows up: `lsusb` should list "XIMEA Corp.".
- Check udev rules from the XIMEA installer are present (typically `/etc/udev/rules.d/99-ximea.rules`).
- No other process (xiCOP, another ROS node) has the device open.

---

## Acknowledgments

- The original [`wavelab/ximea_ros_cam`](https://github.com/wavelab/ximea_ros_cam) ROS 1 driver — design and parameter model.
- The [`African-Robotics-Unit/ximea_ROS2_driver`](https://github.com/African-Robotics-Unit/ximea_ROS2_driver) ROS 2 port — reference for the xiAPI sequencing in ROS 2.
- XIMEA for the [Linux xiAPI](https://www.ximea.com/support/wiki/apis/XiAPI) and supporting documentation.

---

## License

BSD-3-Clause.