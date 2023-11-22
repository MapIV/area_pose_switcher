
# PointCloudRelay Node for ROS 2

This node acts as a relay for Point Cloud data, specifically for filtering out point clouds based on the area localization type.

## Overview

The `PointCloudRelay` node subscribes to the Point Cloud data topic and a localization type topic. If the current localization type is not GNSS, it republishes the incoming Point Cloud data to another topic.

## Topics

### Subscriptions:
- `/sensing/lidar/top/outlier_filtered/pointcloud`: The incoming Point Cloud data.
- `/area_localization_type`: The current localization type.

### Publications:
- `/sensing/lidar/top/outlier_filtered/relayed_pointcloud`: The relayed Point Cloud data (only published if the current localization type is not GNSS).

## Dependencies

- `map4_localization_msgs`: Custom message package for localization type.

## Usage

To run the node:

```
ros2 run pointcloud_relay point_cloud_relay
```


## Notes

- The node uses the `SensorDataQoS` quality of service profile for Point Cloud data, which is optimized for large data types like Point Clouds.
- The node keeps track of the current localization type and updates it whenever a new message arrives on the `/area_localization_type` topic.
- Point Cloud data is only republished if the current localization type is not GNSS.
