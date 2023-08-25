# pose_switcher

## Purpose
Package for switching the pose to be entered in ekf of autoware.universe

## Limitation
### Localization type limitation.
- The number of selectable poses.

    In the current implementation, output pose is selected from gnss based pose or lidar based pose.  

- Input localozation type

    Following values of input `localization_type` topic are detected.
    - "2" for gnss pose.
    - "1" for lidar based pose.  


### Input

| Name | Type | Description |
| - | - | - |
| `gnss_pose` | `geometry_msgs::PoseWithCovarianceStamped` | Pose topic from gnss. |
| `lidar_pose` | `geometry_msgs::PoseWithCovarianceStamped` | Pose topic from lidar based localization. |
| `localization_type` | `tier4_localization_msgs::LocalizationTypeStamped` | The type of localization that should be selected. |

### Output

| Name | Type | Description |
| - | - | - |
| `selected_pose` | `geometry_msgs::PoseWithCovarianceStamped` | Pose topic from gnss:This topic is not published out of gnss available area.|
| `current_localization_type"` | `tier4_localization_msgs::LocalizationTypeStamped` | The selected type of localization. |

## Parameters


This package monitors the error between two poses during switching within the switching area. If the error remains below a certain threshold for a specific duration, the switching process is executed. The parameters for the switching process are as follows:

| Name | Type | Description | Default |
| - | - | - | - |
| `error_2d_threshold` | string | Lanelet2 polygon tag name of gnss available area. | 0.2[m] |
| `error_yaw_threshold` | string | Lanelet2 polygon tag name of gnss available area. | 1.0 [deg] | 
| `judge_switching_ealpsed_time_threshold` | string | Lanelet2 polygon tag name of gnss available area. | 2.0 [sec] | 

## Usage
```
ros2 run pose_switcher pose_switcher
```
or
```
ros2 launch pose_switcher pose_switcher.launch
```

## Related packages

 [gnss_area_recognizer](https://github.com/MapIV/gnss_area_recognizer)
 [ndt_pointcloud_relay](https://github.com/MapIV/ndt_pointcloud_relay)
