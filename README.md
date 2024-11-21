# Camera Publisher Package

A ROS2 package for publishing camera streams with support for multiple cameras and UITM topics.

## Setup Instructions

### 1. Clone the Repository

```bash
cd ~/ros2_ws/src
git clone https://github.com/HelloWorldRobotics/camera_publisher.git
```

### 2. Install Dependencies

Install all required dependencies using rosdep:
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 3. UDEV Rules Setup (Optional)

To ensure consistent camera device naming, it is convenient to set up UDEV rules for each camera.

Create a new UDEV rules file:
```bash
sudo nano /etc/udev/rules.d/99-usb-cameras.rules
```

To find the correct UDEV rule for each camera:

1. Unplug all USB cameras
2. Plug in one camera at a time
3. Note the new `/dev/video*` device that appears
4. Run this command to get the camera's ID_PATH:
```bash
udevadm info -q property -p $(udevadm info -q path -n /dev/video0) | grep ID_PATH
```

5. Create a UDEV rule using this format:
```
SUBSYSTEM=="video4linux", ENV{ID_PATH}=="pci-0000:03:00.0-usb-0:1.1:1.0", KERNEL=="video*", SYMLINK+="buggy_cam_back"
```

6. Repeat for each camera, using appropriate symlinks:
   - `buggy_cam_front`
   - `buggy_cam_back`
   - `buggy_cam_left`
   - `buggy_cam_right`

Apply the new rules:
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### 4. Launch File Configuration

The launch file (`camera_publisher.launch.py`) supports multiple cameras with configurable parameters. If udev rules are not used, the camera device paths must be updated in the launch file. To disable launching a camera, simply comment out the corresponding camera in the return LaunchDescription. Eg:

```python
return LaunchDescription([
    back_camera,
    left_camera,
    right_camera,
    #front_camera
])
```

#### Parameters Description

| Parameter | Type | Description |
|-----------|------|-------------|
| frame_id | string | TF frame ID for the camera |
| parent_frame_id | string | Parent TF frame ID |
| camera_device | string | Camera device path (e.g., /dev/buggy_cam_back) |
| calibration_file | string | Path to camera calibration YAML file |
| flip_image | bool | Whether to flip the image 180 degrees |
| publish_compressed | bool | Enable compressed image publishing |
| uitm_topics | bool | Enable UITM topic publishing |
| uitm_calibration_file | string | Path to UITM calibration YAML file |

### Main Output Topics

| Topic | Type | Description |
|-------|------|-------------|
| `~/image/raw` | sensor_msgs/Image | Raw camera image (1280x720) |
| `~/image/compressed` | sensor_msgs/CompressedImage | JPEG compressed image |
| `~/camera_info` | sensor_msgs/CameraInfo | Camera calibration information |

### UITM Output Topics (if enabled)

| Topic | Type | Description |
|-------|------|-------------|
| `~/uitm/image/raw` | sensor_msgs/Image | Raw UITM image (640x480) |
| `~/uitm/image/compressed` | sensor_msgs/CompressedImage | JPEG compressed UITM image |
| `~/uitm/camera_info` | sensor_msgs/CameraInfo | UITM camera calibration info |

## Configuration Files

Place your camera calibration files in:
- Main calibration: `config/*.yaml`
- UITM calibration: `config/uitm/*.yaml`

## Building and Running

Build with symlink:
```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select camera_publisher
```

Source workspace:
```bash
source ~/ros2_ws/install/setup.bash
```

Run the publisher:
```bash
ros2 launch camera_publisher camera_publisher.launch.py
```


