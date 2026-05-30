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