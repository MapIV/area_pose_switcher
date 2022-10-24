# pose_switcher

Package for switching the pose to be entered in ekf of autoware.iv

```
rosrun pose_switcher pose_switcher
```

```
rostopic pub /localization_type std_msgs/String gnss 
```

```
rostopic pub /localization_type std_msgs/String lidar
```

