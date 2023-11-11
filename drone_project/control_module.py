from rclpy.timer import Rate
from .commands import CommandManager
from px4_msgs.msg import SensorCombined, VehicleCommand, VehicleRatesSetpoint, OffboardControlMode, TrajectorySetpoint, VehicleGlobalPosition, VehicleLocalPosition
from rclpy.clock import Clock

class ControlModule:
    def __init__(self, vehicle_command_publisher, offboard_control_mode_publisher, trajectory_setpoint_publisher, vehicle_rates_setpoint_publisher_p, state_manager, node, logger):
        self.vehicle_command_publisher = vehicle_command_publisher
        self.offboard_control_mode_publisher = offboard_control_mode_publisher
        self.trajectory_setpoint_publisher = trajectory_setpoint_publisher
        self.vehicle_rates_setpoint_publisher_p = vehicle_rates_setpoint_publisher_p
        self.state_manager = state_manager
        self.node = node
        self.logger = logger
        self.rate = Rate

    def get_logger(self):
        return self.logger

    def arm(self):
        # ARM
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        arm_cmd = VehicleCommand()
        arm_cmd.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        arm_cmd.param1 = 1.0 # VehicleCommand.ARMING_ACTION_ARM  # Arm
        arm_cmd.param2 = 0.0 # 0: arm-disarm unless prevented by safety checks, 21196: force arming or disarming
        self.vehicle_command_publisher.publish(arm_cmd)
        # self.get_logger().info('Arming...')


    def takeoff(self):
        takeoff_cmd = VehicleCommand()
        takeoff_cmd.command = VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF
        takeoff_cmd.param7 = 30.46
        self.vehicle_command_publisher.publish(takeoff_cmd)
        self.get_logger().info('Taking off...')

    def move_forward(self):
        msg = VehicleRatesSetpoint()
        msg.thrust_body[0] = 1.0  # Half thrust forward
        msg.thrust_body[1] = 0.0  # No sideways thrust
        msg.thrust_body[2] = 0.0  # No upward thrust
        msg.timestamp = int(Clock().now().nanoseconds / 1000)  # time in microseconds
        self.vehicle_rates_setpoint_publisher_p.publish(msg)

    def control_arc_to_ground(self):

        self.get_logger().info('Arcing to Ground')
        num_points = 10
        
        start_angle_deg = 90.0
        end_angle_deg = 0.0

        x0,y0,z0 = 0., 0., 5.
        x1,y1,z0 = 1., 1., 0.

        start_angle_rad = np.deg2rad(start_angle_deg)
        end_angle_rad = np.deg2rad(end_angle_deg)
        angles_rad = np.linspace(start_angle_rad, end_angle_rad, num_points)  

        radius = -2.0

        points = []
        for angle_rad in angles_rad:
            x = radius * np.cos(angle_rad)
            # y = radius * np.sin(angle_rad)
            # x = -2.0
            y = -0.0
            z = radius * np.sin(angle_rad)
            points.append((x,y,z))

        self.get_logger().info(str(points))
        for point in points:
            msg = TrajectorySetpoint()
            msg.position = list(point)
            msg.velocity = [10.0, 10.0, 10.0]
            msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
            self.trajectory_setpoint_publisher.publish(msg)
            self.get_logger().info("Going to " + str(point))

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

    def control_takeoff(self):
        # self.get_logger().info("Controlling takeoff")
        # Add the control logic for takeoff here
        msg = TrajectorySetpoint()      
        msg.position = [0.0, 0.0, -20.0]
        msg.yaw = -3.14
        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        self.trajectory_setpoint_publisher.publish(msg)
        
    def control_land(self):
        print("Controlling landing")
        # Add the control logic for landing here


    def trajectory(self, x, y, z, yaw):
        self.get_logger().info("Flying to...")
        def _send_command():
            # set way point to go to point
            msg = TrajectorySetpoint()
            msg.position = [x, y, z]
            # msg.acceleration =  [0.5, 0.5, 0.5]
            msg.velocity = [10.0, 10.0, 10.0]
            # msg.yaw = 

            msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
            self.trajectory_setpoint_publisher.publish(msg)
                
            rclpy.spin_once(self.node)

            self.rate.sleep()

        while rclpy.ok() and self.state_manager.distance(x,y,z) > 0.1:
            _send_command()

        for i in range(ROS_RATE):
            _send_command()

    def offboard_control(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = True
        msg.acceleration = True
        msg.attitude = True
        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        self.offboard_control_mode_publisher.publish(msg)
