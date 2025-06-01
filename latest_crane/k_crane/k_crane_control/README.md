# c_crane_control 

## move along given data

1. launch gazebo
```
roslaunch c_crane_gazebo c_crane_gazebo_with_payload.launch
```

2. move initial pose
```
rosrun c_crane_control move_initial_pose
```

3. move along given data
```
rosrun c_crane_control joint_trajectory_action
```

## trouble shooting

### core dump
remove empty line from given csv data.
there is empty line at the end of file.
