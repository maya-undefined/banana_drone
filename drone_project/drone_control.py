import time
import sys

from scipy.spatial.transform import Rotation as R


import asyncio

from mavsdk import System
from mavsdk.offboard import (OffboardError, Attitude, VelocityBodyYawspeed, PositionNedYaw)

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_system_default, qos_profile_sensor_data
from rclpy.timer import Rate

import numpy as np

from cv_bridge import CvBridge
import cv2

import darknet_ros_msgs
from darknet_ros_msgs.msg import BoundingBoxes

from px4_msgs.msg import SensorCombined, VehicleLocalPositionSetpoint, VehicleRatesSetpoint, VehicleStatus, VehicleAttitudeSetpoint, VehicleAttitude, VehicleCommand, OffboardControlMode, TrajectorySetpoint, VehicleGlobalPosition, VehicleLocalPosition
from sensor_msgs.msg import Image

from .state_estimation import StateEstimationManager
from .control_module import ControlModule
from .commands import CommandManager, VehicleCommandPublisher, ArmCommand, TakeoffCommand, SetpointCommand, MoveForwardCommand


ROS_RATE = 20

class DroneSystem(Node):
    def __init__(self, mavsdk_drone=None):
        super().__init__('drone_control')
        self.waypoints = []
        self.current_pose = None
        self.vehicle_status = None
        self.current_command = None
        self.control_module = None
        self.current_attitude = None
        self.wp = None
        self.desired_vehicle_atttitude = None
        self.desired_vehicle_rate_attitude = None
        self.delta_yaw = None
        self.coords = None
        self.new_yaw = None
        self.new_pitch = None

        self.taken_off = False
        
        # this is for flying to bananas
        self.target_bb = None
        self.found_target = False

        self.target_counter = 0
        self.counter = 0

        qos_profile = qos_profile_sensor_data

        self.get_logger().info('Initializing...')

        self.mavsdk_drone = mavsdk_drone
        # these have to be declared here since drone_control is
        # a ROS2 node.
        vehicle_command_publisher = self.create_publisher(
            VehicleCommand, 
            '/fmu/in/vehicle_command', 
            qos_profile)

        offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, 
            '/fmu/in/offboard_control_mode', 
            qos_profile)

        trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, 
            '/fmu/in/trajectory_setpoint', 
            qos_profile)

        vehicle_local_position_s = self.create_subscription(
            VehicleLocalPosition, 
            '/fmu/out/vehicle_local_position', 
            self.localtion_cb,
            qos_profile=qos_profile)

        vehicle_status_s = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_cb,
            qos_profile=qos_profile
            )

        self.camera_subscriber = self.create_subscription(
            Image,
            '/camera',
            self.camera_cb,
            qos_profile=qos_profile)

        self.darknet_ros = self.create_subscription(
            BoundingBoxes,
            '/darknet_ros/bounding_boxes',
            self.bounding_boxes_cb,
            qos_profile=qos_profile
            )

        self.vehicle_rates_setpoint_publisher_p = self.create_publisher(
                VehicleRatesSetpoint,
                '/fmu/in/vehicle_rates_setpoint', 10
            )

        self.state_manager = StateEstimationManager(
                vehicle_local_position_s=vehicle_local_position_s,
                logger=self.get_logger()
            )

        self.control_module = ControlModule(
            vehicle_command_publisher=vehicle_command_publisher,
            offboard_control_mode_publisher=offboard_control_mode_publisher,
            trajectory_setpoint_publisher=trajectory_setpoint_publisher,
            vehicle_rates_setpoint_publisher_p=self.vehicle_rates_setpoint_publisher_p,
            state_manager=self.state_manager,
            node=self,
            logger=self.get_logger()
        )

        self.vehicle_attitude = self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self.vehicle_attitude_cb,
            qos_profile=qos_profile)


        self.vehicle_attitude_p = self.create_publisher(
            VehicleAttitudeSetpoint,
            '/fmu/in/vehicle_attitude_setpoint',
            qos_profile=qos_profile
            )

        self.vehicle_rates_p = self.create_publisher(
            VehicleRatesSetpoint, "/fmu/in/vehicle_rates_setpoint", 10)

        self.local_setpoint_p = self.create_publisher(
            VehicleLocalPositionSetpoint, 
            '/fmu/out/vehicle_local_position_setpoint', 10)


        
        self.command_manager = CommandManager(control_module=self.control_module)
        self.trajectory_setpoint_publisher = trajectory_setpoint_publisher
        self.timer = self.create_timer(0.1, self.timer_cb)


    def vehicle_status_cb(self, msg=None):
        self.vehicle_status = msg

    def vehicle_attitude_cb(self, msg):
        self.current_attitude = msg.q

    def bounding_boxes_cb(self, msg=None):
        self.found_target = False
        self.target_bb = None
        for bb in msg.bounding_boxes:
            if bb.class_id == 'banana':
                self.found_target = True
                self.target_bb = bb

    def camera_cb(self, msg=None):
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        (rows,cols,channels) = cv_image.shape

        if self.coords != None:
            cv2.circle(cv_image, (int(self.coords['x']), int(self.coords['y']) ), 10, 255)

        cv2.line(cv_image, (0, 240), (640, 240), (255, 0, 0), 1)
        cv2.line(cv_image, (320, 0), (320, 480), (255, 0, 0), 1)

        cv2.imshow("ROS2 Camera View", cv_image)
        cv2.waitKey(1)
        pass

    def localtion_cb(self, msg):
        self.state_manager.update(msg)  
        self.current_pose = msg


    def arm(self):
        self.run_command(ArmCommand(self.control_module))


    def takeoff(self):
        while self.vehicle_status == None or self.vehicle_status.arming_state != 2:
            rclpy.spin_once(self)

        self.wp = [0,0,-2.5]

        self.run_command( TakeoffCommand(self.control_module) )
        self.taken_off = True

    def run_command(self, command=None, target=100):
        if command is None:
            return

        self.current_command = command
        self.target_counter = self.counter + target

    def clear_command(self):
        self.current_command = None

    async def _follow(self, delta_east, delta_north, drone):
        forward = 5.0
        east = 0.0
        down = 0.0
        yawspeed = 0.0

        if delta_east < 0:
            yawspeed = -30
            east = -2
        else:
            # we turn right
            yawspeed = 30
            east = 2

        if delta_north < 0:
            down = -1
        else:
            down = 1

        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(forward, east, down, yawspeed)
        )
        await asyncio.sleep(4)

        return (forward, east, down, yawspeed)        


    def timer_cb(self):
        self.counter += 1

        if self.current_command is None:
            return
        if self.control_module is None:
            return

        if self.desired_vehicle_atttitude != None:
            self.desired_vehicle_atttitude = None
        else:
            self.control_module.offboard_control()
            # this has to be done every spin, as a heartbeat
            self.current_command.execute()

        self.counter += 1


    def check_waypoint_reached(self, pos_tol=0.3):
        if self.current_pose is None or self.wp is None:
            return 0

        wp = self.wp

        deltax = np.abs(self.current_pose.x - wp[0])
        deltay = np.abs(self.current_pose.y - wp[1])
        deltaz = np.abs(self.current_pose.z - wp[2])
        dmag = np.sqrt(
                np.power(deltax, 2) +
                np.power(deltay, 2) +
                np.power(deltaz, 2)
            )

        # s = "%s %s %s %s %s" % (wp[0], deltax, deltay, deltaz, dmag)
        # self.get_logger().info(str(s))
        if dmag < pos_tol:
            # self.get_logger().info("we got near!")
            return 1
        else:
            return 0

    def set_next_destination(self):
        if len(self.waypoints) == 0:
            return 
            # raise Exception("waypoints are out")

        wp =  self.waypoints[-1]
        # self.get_logger().info("Going to " + str(wp))

        sp = SetpointCommand(
            control_module=self.control_module,
            x=wp[0], y=wp[1], z=wp[2]
        )

        self.run_command(sp, 1048576)

        msg = TrajectorySetpoint()
        msg.position = [wp[0], wp[1], wp[2]]
        msg.velocity = [10.0, 10.0, 10.0]
        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        self.control_module.trajectory_setpoint_publisher.publish(msg)
        self.waypoints.pop()
        self.wp = wp


    def _q_to_e(self, current_orientation):
        # Convert quaternion to Euler angles
        r = R.from_quat(current_orientation)
        current_angles = r.as_euler('xyz', degrees=False)
        current_roll, current_pitch, current_yaw = current_angles
        return (current_roll, current_pitch, current_yaw)

    def _e_to_q(self, current_orientation):
        current_roll, new_pitch, new_yaw = current_orientation
        new_r = R.from_euler('xyz', [current_roll, new_pitch, new_yaw], degrees=False)
        new_orientation = new_r.as_quat()
        return new_orientation

    def _e_to_ned(self, current_attitude, body_direction):
        r = R.from_quat(current_attitude)
        ned_direction = r.apply(body_direction)
        return ned_direction

    def move_forward(self):
        msg = VehicleRatesSetpoint()
        # msg.roll = 0.0  # No roll rate
        # msg.pitch = 0.0  # No pitch rate
        # msg.yaw = 0.0  # No yaw rate
        msg.thrust_body[0] = 1.0  # Half thrust forward
        msg.thrust_body[1] = 0.0  # No sideways thrust
        msg.thrust_body[2] = 0.0  # No upward thrust
        msg.timestamp = int(Clock().now().nanoseconds / 1000)  # time in microseconds
        self.vehicle_rates_setpoint_publisher_p.publish(msg)

    async def fly_to_label(self):
        if not self.target_bb:
            return
        bb = self.target_bb
        if self.current_attitude is None:
            return
        current_attitude = self.current_attitude
        current_angles = self._q_to_e(current_attitude)

        if bb is None:
            return
        image_w = 640
        image_h = 480
        # see PX4/Tools/simulation/gz/worlds/default2.sdf

        gain = 0.05
        current_roll, current_pitch, current_yaw = current_angles
        box_x = (bb.xmax - bb.xmin) / 2 + bb.xmin
        box_y = (bb.ymax - bb.ymin) / 2 + bb.ymin
        error_x = box_x - ((image_w / 2) - 1)
        error_y = ((image_h / 2) - 1) - box_y
        delta_east = gain * error_x
        delta_north = gain * error_y

        new_east = box_x + delta_east
        new_north = box_y + delta_north

        s = "box_x: {}, box_y: {}, error_x: {}, error_y: {}, new_east: {}, new_north: {}, delta_east: {}, delta_north: {} ".format(box_x, 
            box_y, error_x, error_y, new_east, new_north, delta_east, delta_north)
        self.get_logger().info(s)

        self.coords = {'x': box_x, 'y': box_y}

        if self.mavsdk_drone != None:
            (forward, east, down, yawspeed) = await self._follow(delta_east, delta_north, self.mavsdk_drone)
            s = "forward: {}, east: {}, down: {}, yawspeed: {} ".format(forward, east, down, yawspeed)
            self.get_logger().info(s)


async def print_status_text(drone):
    try:
        async for status_text in drone.telemetry.status_text():
            print(f"Status: {status_text.type}: {status_text.text}")
    except asyncio.CancelledError:
        return

def fly_waypoints(drone_system):

    drone_system.waypoints.append([0, 0, -10, 180])
    drone_system.waypoints.append([0, 50, -10, 90])
    drone_system.waypoints.append([50, 50, -10, 0])
    drone_system.waypoints.append([50, 0, -10, -90])
    drone_system.waypoints.append([0, 0, -10, 0])
    # dont duplicate these!

    counter = 0

    drone_system.arm()
    while rclpy.ok():
        rclpy.spin_once(drone_system)
        if drone_system.target_counter == drone_system.counter:
            break

    drone_system.run_command( TakeoffCommand(drone_system.control_module) )
    while rclpy.ok():
        rclpy.spin_once(drone_system)
        if drone_system.target_counter == drone_system.counter:
            break

    print(drone_system.waypoints)

    drone_system.set_next_destination()
    while rclpy.ok():
        counter +=1 
        rclpy.spin_once(drone_system)
        # print(str(drone_system.check_waypoint_reached()), drone_system.waypoints)
        if (drone_system.check_waypoint_reached() == 1):
            if len(drone_system.waypoints) == 0:
                break
            drone_system.set_next_destination()



    # bleep bloop
    drone_system.destroy_node()
    rclpy.shutdown()

async def fly_hover_mavsdk():
    mavsdk_drone = System()
    await mavsdk_drone.connect(system_address="udp://:14540")
    drone_system = DroneSystem(mavsdk_drone=mavsdk_drone)

    drone_system.waypoints.append([0, 0, -2.5, 0])
    # need at least one or the drone will never take off

    counter = 0

    drone_system.arm()
    while rclpy.ok():
        rclpy.spin_once(drone_system)
        if drone_system.target_counter == drone_system.counter:
            break

    await mavsdk_drone.offboard.set_position_ned(
        PositionNedYaw(0, 0, 0, 0))

    await mavsdk_drone.offboard.set_position_ned(
        PositionNedYaw(0, 0, -2.5, 0))
    await asyncio.sleep(4)
    print(drone_system.waypoints)

    drone_system.set_next_destination()
    while rclpy.ok():
        counter +=1 
        rclpy.spin_once(drone_system)

        await drone_system.fly_to_label() 

        if drone_system.target_bb != None:
            rclpy.spin_once(drone_system)



def main(args=None):
    rclpy.init(args=args)
    drone_system = DroneSystem()

    # fly_waypoints(drone_system)

    asyncio.run( fly_hover_mavsdk() )

if __name__ == '__main__':
    main()