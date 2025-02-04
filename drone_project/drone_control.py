import sys
import math

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import qos_profile_system_default, qos_profile_sensor_data

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleStatus, VehicleCommand, VehicleCommandAck, VehicleLocalPosition, VehicleAttitude

from sensor_msgs.msg import Image

from cv_bridge import CvBridge
import cv2

import darknet_ros_msgs
from darknet_ros_msgs.msg import BoundingBoxes

from .commands import ArmCommand, TakeoffCommand, HoverCommand, TargetCommand, LandCommand
from .state_machine import DroneSM


class DroneSystem(Node):
    def __init__(self):
        super().__init__('drone_control')

        self.vehicle_status = None
        self.current_command = None
        self.trajectory_setpoint_publisher = None
        self.vehicle_command_publisher = None
        self.offboard_control_mode_publisher = None
        self.local_position = None
        self.coords = None
        self.target_bb = None
        self.found_target = False
        self.chasing_target = False
        self.landing = False
        self.prev_command = []
        self.roll = None
        self.pitch = None
        self.yaw = None
        self.machine = DroneSM(initial_state="land", drone_system=self)

        qos_profile = qos_profile_system_default

        self.vehicle_status_s = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_cb,
            qos_profile=qos_profile
        )
        # Tell me how you feel, honey

        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            qos_profile)
        # I need to tell you something...

        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, 
            '/fmu/in/vehicle_command', 
            qos_profile)
        # This is what I am going to tell you...

        self.command_ack_subscriber = self.create_subscription(
                    VehicleCommandAck,
                    '/fmu/out/vehicle_command_ack',
                    self.command_ack_callback,
                    qos_profile)

        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            qos_profile)
        # Go here, destination: my heart

        self.vehicle_local_position_s = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.location_cb,
            qos_profile=qos_profile)
        # Where am I, bb? I'm lost wthout you!

        self.camera_subscriber = self.create_subscription(
            Image,
            # '/model/gz_x500_mono_cam/sensor/camera/link/camera/image',
            '/camera',
            self.camera_cb,
            qos_profile=qos_profile_sensor_data)
        # I want to see you!

        self.darknet_ros = self.create_subscription(
            BoundingBoxes,
            '/darknet_ros/bounding_boxes',
            self.bounding_boxes_cb,
            qos_profile=qos_profile)
        # here i am!

        self.vehicle_attitude_subscriber = self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self.attitude_cb,
            qos_profile=qos_profile_sensor_data
            )
        # tell me how your sensor feels

        self.timer = self.create_timer(0.1, self.timer_cb)
        # I think of you 10 times a second, Evelyn.


    def command_ack_callback(self, msg):
        # Log the acknowledgment result
        self.get_logger().info(
            f'VehicleCommandAck {msg.command} acknowledged with result {msg.result} (from_external={msg.from_external})'
        )
        if msg.command == VehicleCommand.VEHICLE_CMD_NAV_LAND and msg.result == 0: # Land 
            self.landing = True
            # stuff like this should go into a lookup based on key per command
            # that is, without explicitly coding each and every command
            # let's instead have a way for commands to register to listen
            # based on a key like VEHICLE_CMD_NAV_LAND forr the LandCommand
            # and when the msg gets in here from px4 land, we notify the command
            # that was listening/registered on the VEHICLE_CMD_NAV_LAND 'key'
            # 
            # it can also be used to pass data between drone and commands
            # in fact, using this to abstract vehicle components away, such as
            # yaw/pitch/roll (since not all drones will have these), this allows
            # an abstract way to interface Commands() to DroneControl() parts.

        if msg.result == 0:  # Success
            self.get_logger().info('\tCommand executed successfully')
        elif msg.result == 1:  # Failure
            self.get_logger().warning('\tCommand failed')
        elif msg.result == 2:  # In progress
            self.get_logger().info('\tCommand in progress')

    def on_takeoff(self):
        # Issue the Takeoff command
        pass

    def bounding_boxes_cb(self, msg=None):
        # self.get_logger().info('BB Called!')

        for bb in msg.bounding_boxes:
            # if bb.class_id == 'banana':
            self.found_target = True
            # self.current_command_state = CommandState.TARGET
            self.target_bb = bb
            self._calculate_coords()

    def _calculate_coords(self):
        if self.target_bb == None:
            return None
        bb = self.target_bb
        image_w = 640*2
        image_h = 480*2
        box_x = (bb.xmax - bb.xmin) / 2 + bb.xmin
        box_y = (bb.ymax - bb.ymin) / 2 + bb.ymin
        x_width = math.fabs(bb.xmax - bb.xmin)
        y_height = math.fabs(bb.ymax - bb.ymin)
        self.coords = {
            'x': box_x, 'y': box_y, 
            'x_width': x_width, 'y_height': y_height
            }

        # error_x = box_x - ((image_w / 2) - 1)
        # error_y = ((image_h / 2) - 1) - box_y
        # gain = 1
        # delta_east = gain * error_x
        # delta_north = gain * error_y
        # new_east = 

        # s = "box_x: {}, box_y: {}, error_x: {}, error_y: {}, new_east: {}, new_north: {}, delta_east: {}, delta_north: {} ".format(box_x, 
        #     box_y, error_x, error_y, new_east, new_north, delta_east, delta_north)
        # self.get_logger().info(s)


    def camera_cb(self, msg=None):
        # TODO: There needs to be a way to handle multiple cameras
        # This camera call back should be a 'Widget' or some extension, distinct
        # from the 'Command'
        # A possible apparatus would be 
        '''
        class Widget():
            def init(self):
                ...
            def register_my_subscriptions(self):
                register to things like '/camera' or '/darknet_ros/bounding_boxes'

            def register_my_callbacks(self):
                register my functions (like this very function) to run as a cb via
                ros2 subscriptions
            def register_my_publishers(self):
                publish my new coords{} object and notify any listeners, possibly in my
                funky KV idea

        '''
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        (rows,cols,channels) = cv_image.shape
        # self.get_logger().info(f'{rows}, {cols}')
        # print("camera!")
        self._calculate_coords()
        if self.coords != None:
            cv2.circle(cv_image, (int(self.coords['x']), int(self.coords['y']) ), 10, 255)

        cv2.line(cv_image, (0, rows//2), (cols, rows//2), (255, 0, 0), 1)
        cv2.line(cv_image, (cols//2, 0), (cols//2, rows), (255, 0, 0), 1) # vertical line

        cv2.imshow("ROS2 Camera View", cv_image)
        cv2.waitKey(1)

    def vehicle_status_cb(self, msg=None):
        self.vehicle_status = msg

    def location_cb(self, msg=None):
        self.local_position = msg
        msg_size = sys.getsizeof(msg)
        '''
        self.get_logger().info(f'VehicleLocalPosition size: {msg_size} bytes')
        # Print out some key fields to verify content
        self.get_logger().info(f'\tTimestamp: {msg.timestamp}')
        self.get_logger().info(f'\tPosition: x={msg.x}, y={msg.y}, z={msg.z}')
        '''

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command  # command ID
        msg.target_system = 1  # system which should execute the command
        msg.target_component = 1  # component which should execute the command, 0 for all components
        msg.source_system = 1  # system sending the command
        msg.source_component = 1  # component sending the command
        msg.from_external = True
        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        self.vehicle_command_publisher.publish(msg)

    def offboard_control(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = True
        msg.acceleration = True
        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        self.offboard_control_mode_publisher.publish(msg)

    def run_command(self, command):
        if self.current_command:
            self.prev_command.append(self.current_command)
            self.current_command.canceled = True

        self.current_command = command

    def quaternion_to_euler(self, q):
        """Convert a quaternion into Euler angles (roll, pitch, yaw)."""
        w, x, y, z = q

        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        pitch = math.asin(sinp) if abs(sinp) <= 1 else math.copysign(math.pi / 2, sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def attitude_cb(self, msg):
        self.vehicle_attitude = msg
        q = np.array([msg.q[0], msg.q[1], msg.q[2], msg.q[3]])
        roll, pitch, yaw = self.quaternion_to_euler(q)

        # Convert radians to degrees
        roll_deg = math.degrees(roll)
        pitch_deg = math.degrees(pitch)
        yaw_deg = math.degrees(yaw)
        # self.get_logger().info(f"Roll: {roll_deg:.2f}°, Pitch: {pitch_deg:.2f}°, Yaw: {yaw_deg:.2f}°")
        self.roll, self.pitch, self.yaw = roll, pitch, yaw

    def timer_cb(self):
        '''
        if image/drone is ready: 

            analyze_image()

            if state == found:
                move_to_balloon
            else:
                patrol()

        else:
            execute current command
        '''

        self.offboard_control()

        if self.found_target and not self.chasing_target:
            self.get_logger().info('found_target')
            # self.get_logger().info("Found a Target")
            # pre-empt current command by just deleting it
            target_command = TargetCommand(self)
            # target_command = LandCommand(self)
            self.run_command(target_command)
            self.chasing_target = True
        # self.get_logger().info(" %s - %s " % (self.current_command, self.current_command.done()))

        if self.current_command and not self.current_command.done():
            # if not self.current_command.executed:
            self.current_command.execute()
        else:
            self.current_command = None



def main(args=None):
    rclpy.init(args=args)
    drone_system = DroneSystem()

    arm_cmd = ArmCommand(drone_system)
    drone_system.run_command(arm_cmd)

    while rclpy.ok():
        rclpy.spin_once(drone_system)
        if arm_cmd.done():
            break

    takeoff_cmd = TakeoffCommand(drone_system)
    drone_system.run_command(takeoff_cmd)

    while rclpy.ok():
        rclpy.spin_once(drone_system)
        if takeoff_cmd.done():
            break

    hover_cmd = HoverCommand(drone_system)
    drone_system.run_command(hover_cmd)
    while rclpy.ok():
        rclpy.spin_once(drone_system)
        # if hover_cmd.done():
        #     break

    drone_system.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
