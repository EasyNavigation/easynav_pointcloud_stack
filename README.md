# easynav_pointcloud_stack

This stack is part of the Easy Navigation (EasyNav) project developed by the Intelligent Robotics Lab. It is composed of the **pointcloud_maps_builder** package and the **pointcloud_maps_manager** package (currently under construction).

## Pointcloud Maps Builder

The `PointcloudMapsBuilderNode` subscribes to point cloud sensor data, downsamples and fuses it, and republishes the processed data for mapping or further processing.

### Parameters

| Parameter Name            | Type    | Default   | Description                                      |
|--------------------------|---------|-----------|--------------------------------------------------|
| `sensor_topic`           | string  | `points`  | Topic to subscribe for point cloud input         |
| `downsample_resolution`  | double  | `1.0`     | Resolution to use when downsampling              |
| `perception_default_frame` | string | `map`     | Target frame for fusing point clouds             |

---

### Installation

Clone the repository into your ROS 2 workspace:

```bash
cd ~/ros2_ws/src
git clone https://github.com/EasyNavigation/easynav_pointcloud_stack.git
git clone https://github.com/EasyNavigation/easynav_outdoor_testcase.git
cd ..
colcon build --packages-select easynav_pointcloud_maps_builder
```

### Usage

Source your workspace:

```bash
source ~/ros2_ws/install/setup.bash
```

Run the lifecycle node:

```bash
ros2 run easynav_pointcloud_maps_builder pointcloud_maps_builder_node
```

## Test

1. Create a parameter YAML file (e.g., `params.yaml`) with the following content:

```yaml
pointcloud_maps_builder_node:
  ros__parameters:
    use_sim_time: true
    sensors: [map]
    downsample_resolution: 0.1
    perception_default_frame: map
    map:
      topic: map
      type: sensor_msgs/msg/PointCloud2
      group: points
```

2. Run the node using the parameter file with this command:

``` bash
ros2 run easynav_pointcloud_maps_builder pointcloud_maps_builder_main \
--ros-args --params-file src/easynav_outdoor_testcase/params.yaml
```

## Pointcloud Maps Manager

The `PointcloudMapsManager` subscribes to point cloud topic managed by Maps Builder. There is another option when the point cloud is stored in a Point Cloud Data file (`.pcd`). The manager can handle both cases, allowing for flexible integration with existing point cloud data.

### Parameters

| Parameter Name            | Type    | Default   | Description                                      |
|--------------------------|---------|-----------|--------------------------------------------------|
| `map_types`           | string | ` `                            | Map representation defined by map manager |

By each `map_types` define in the params file is required to define the following parameters:

| Parameter Name            | Type    | Default   | Description                                      |
|--------------------------|---------|-----------|--------------------------------------------------|
| `map_topic_in`        | string | `/<node name>/<plugin name>/topic_in`  | Topic where the point cloud is received |
| `package_name`        | string | ` `                            | Package name where is located the `pcd` file.|
| `map_path_file`       | string | `pointcloud_map.pcd`      | Filename for `pcd` map file.     |

>> **Note:** The `map_path_file` is only required if the `package_name` is defined. If not, the point cloud data will be read from topic defined in `map_topic_in`.

### Installation

Clone the repository into your ROS 2 workspace:

``` bash
cd ~/ros2_ws/src
git clone https://github.com/EasyNavigation/easynav_pointcloud_stack.git
git clone https://github.com/EasyNavigation/easynav_outdoor_testcase.git
cd ..
colcon build --packages-select easynav_pointcloud_maps_manager easynav_pointcloud_common
```

### Usage

Source your workspace:

```bash
source ~/ros2_ws/install/setup.bash
```

Run the main node:

```bash
ros2 run easynav_system system_main --ros-args --params-file <yaml file with parameters>
```

The file `maps_manager.params.yaml` could be modified to include the desired point cloud map types and their respective parameters.

## How to create a parameter file for Pointcloud Maps Manager

1. Create a parameter YAML file (e.g., `params.yaml`) with the following content:

```yaml
maps_manager_node:
  ros__parameters:
    use_sim_time: true # When data source is from file this parameter should be set to false
    map_types: [PointCloudData] # Map types defined by the manager
    PointCloudData:
      plugin: easynav_pointcloud_maps_manager/PointCloudMapsManager # Plugin name
      map_topic_in: /map_builder/cloud_filtered # Topic where the point cloud is received
      package_name: easynav_outdoor_testcase # Package name where is located the `pcd` file.
      map_path_file: maps/pointcloud_map.pcd  # Filename for `pcd` map file.
```
