# banana_drone

## setup

```
install px4-autopilot
install drone_project_ws
install ros_gz_bridge
install Micro-XRCE-DDS-Agent
install darknet_ros_yolov4into drone_project_ws
install banana_drone
install gazebo
```

### Adding a camera

the PX4 model doesn't come with a camera, so we can add one. Inside of the `PX4-Autopilot` repo apply the changes from
`changes_for_camera.xml`. 

### Darknet, ros_gz_bridge, banana_drone

`darknet_ros_yolo4into` should be installed into the drone_project_ws/src directory.

### PX4, Micro-XRCE-DDS-Agent 

These can be installed outside the drone_project_ws directory

## running

Each step requires at least this much

```
cd drone_project_ws
. install/setup.bash
```

In 1 terminal run
```ros2 run ros_gz_bridge parameter_bridge /camera@sensor_msgs/msg/Image@ignition.msgs.Image```

In another terminal run
```
cd Micro-XRCE-DDS-Agent
MicroXRCEAgent udp4 -p 8888
```

In another terminal
```
ros2 launch darknet_ros darknet_ros.launch.py
```

In another terminal
```
make px4_sitl gz_x500
```

If it freezes and gazebo never starts, then hit ^Z, but it in the `bg` and run `PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL=x500 ./build/px4_sitl_default/bin/px4`

Finally, in another terminal
```
cd drone_prooject_ws/src
colcon build --packages-select  drone_project
source install/setup.bash
ros2 launch drone_project drone_control_launch.py
```

You should re-run `colcon` everytime you make a change.
