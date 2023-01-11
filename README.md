# pose_switcher

## Purpose
Package for switching the pose to be entered in ekf of autoware.iv

## Limitation
### Localization type limitation.
- The number of selectable poses.

    In the current implementation, output pose is selected from gnss based pose or lidar based pose.  

- Input localozation type

    Following values of input `localization_type` topic are detected.
    - "GNSS" or "gnss" for gnss pose.
    - "LiDAR", "Lidar", "lidar", "NDT", "Ndt", or "ndt" for lidar based pose.  


### Input

| Name | Type | Description |
| - | - | - |
| `gnss_pose` | `geometry_msgs::PoseStamped` | Pose topic from gnss. |
| `lidar_pose` | `geometry_msgs::PoseStamped` | Pose topic from lidar based localization. |
| `localization_type` | `std_msgs::String` | The type of localization that should be selected. |

### Output

| Name | Type | Description |
| - | - | - |
| `selected_pose` | `geometry_msgs::PoseStamped` | Pose topic from gnss:This topic is not published out of gnss available area.|
| `current_localization_type"` | `std_msgs::String` | The selected type of localization. |

## Parameters
None

## Usage
```
rosrun pose_switcher pose_switcher
```
or
```
roslaunch pose_switcher pose_switcher.launch
```

## Test
When you check the change of Pose message, Use `rostopic pub` method.
```
rostopic pub /localization_type std_msgs/String gnss 
```

```
rostopic pub /localization_type std_msgs/String lidar
```

