# easynav_pointcloud_stack

This stack is part of the Easy Navigation (EasyNav) project developed by the Intelligent Robotics Lab. It is composed of the **pointcloud_maps_builder** package and the **pointcloud_maps_manager** package (currently under construction).

The `PointcloudMapsBuilderNode` subscribes to point cloud sensor data, downsamples and fuses it, and republishes the processed data for mapping or further processing.



## Parameters

| Parameter Name            | Type    | Default   | Description                                      |
|--------------------------|---------|-----------|--------------------------------------------------|
| `sensor_topic`           | string  | `points`  | Topic to subscribe for point cloud input         |
| `downsample_resolution`  | double  | `1.0`     | Resolution to use when downsampling              |
| `perception_default_frame` | string | `map`     | Target frame for fusing point clouds             |

---

## Installation

Clone the repository into your ROS 2 workspace:
```bash
cd ~/ros2_ws/src
git clone https://github.com/EasyNavigation/easynav_pointcloud_stack.git
cd ..
colcon build --packages-select easynav_pointcloud_maps_builder
```

## Usage

Source your workspace:
```bash
source ~/ros2_ws/install/setup.bash
```
Run the lifecycle node:
```bash
ros2 run easynav_pointcloud_maps_builder pointcloud_maps_builder_node
```
