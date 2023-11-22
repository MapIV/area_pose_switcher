# gnss_area_recognizer

## Purpose

To reduce the effects of multi-path errors that occur on static objects such as buildings, trees and so on.

This node filters out gnss pose outside polygons that described in Lanelet2.

## Inputs / Outputs

### Input

| Name | Type | Description |
| - | - | - |
| `input_lanelet2_map_bin_topic` | `autoware_lanelet2_msgs::msg::MapBin` | lanelet2 map bin |
| `input_pose_with_covariance_topic` | `geometry_msgs::msg::PoseWithCovarianceStamped` | gnss pose |

### Output

| Name | Type | Description |
| - | - | - |
| `output_pose_with_covariance_topic` | `geometry_msgs::msg::PoseWithCovarianceStamped` | available gnss pose |
| `/localization/util/localization_type"` | `gstd_msgs::msg::String` | The localization_type of the available pose |

## Parameters

### param.yaml

| Name | Type | Description | Default |
| - | - | - | - |
| `polygon_tag_name` | string | Lanelet2 polygon tag name of gnss available area. | `gnss_available_area` | 
| `pub_msg_in_polygon` | string | The type of localization in the area | `GNSS` | 
| `pub_msg_out_of_polygon` | string | The type of localization out of the area | `NDT` | 

### launch.xml args

| Name | Type | Description | Default |
| - | - | - | - |
| `param_file_path` | string | Param.yaml file path. | `$(find-pkg-share gnss_area_filter)/config/gnss_area_recognizer.param.yaml` |
| `input_lanelet2_map_bin_topic` | string | input_lanelet2_map_bin_topic | `/map/vector_map` |
| `input_pose_with_covariance_topic` | string | input_pose_with_covariance_topic | `/localization/util/gnss_distortion_corrector/pose_with_covariance` |
| `output_pose_with_covariance_topic` | string | output_pose_with_covariance_topic | `/localization/util/gnss_area_filter/pose_with_covariance` |
