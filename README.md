# banana_drone

https://github.com/maya-undefined/banana_drone/assets/115038582/56d14cc0-3a95-40ff-9c1c-dbdfbb816d2c

banana drone is an autonomous drone. it is built on the ROS2 framework, uses PX4 autopilot, and uses deep learning to do object detection. Inertial guidance allows it to fly patrol via waypoints.

## setup

first, you need to create a new Ros2 workspace. the, roughly follow these steps

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

![x500](https://github.com/maya-undefined/banana_drone/assets/115038582/404ca3ef-6a26-4e49-a8bf-c64c7ec188ee)

the PX4 x500 model doesn't come with a camera, so we can add one. Inside of the `PX4-Autopilot` repo apply the changes from
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
```
ros2 run ros_gz_bridge parameter_bridge /camera@sensor_msgs/msg/Image@ignition.msgs.Image
```

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

![console](https://github.com/maya-undefined/banana_drone/assets/115038582/82c30ed3-ef0a-4dc0-8de0-5dad83268d4c)

You should re-run `colcon` everytime you make a change.

#### fly_waypoints

This code will allow the drone to fly in a pre-determined set of waypoints, like on a patrol. Non-banana objects of interest could be recorded and reported when the drone finishes its patrol. GPS is not needed.

#### fly_hover_mavsdk

The drone will hover and watch for an object, with the idea to chase a detected object. By keeping the object in the center aperature of the camera, the drone can follow or track an object without complicated algoirthms or GPS.
