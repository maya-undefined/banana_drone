import math

from px4_msgs.msg import TrajectorySetpoint, VehicleCommand, VehicleStatus, VehicleCommand
from rclpy.clock import Clock

from .pid import pid

class Command:
    def __init__(self, name):
        self.name = name
        self.executed = False
        # print(f"Running {self.name}")
        self.finished = False
        self.canceled = False

    def execute(self):
        pass

    def done(self):
        return False

    def cancel(self):
        pass


class ArmCommand(Command):
    def __init__(self, drone_system):
        super().__init__("Arm")
        self.drone_system = drone_system

    def execute(self):
        self.drone_system.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        arm_cmd = VehicleCommand()
        arm_cmd.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        arm_cmd.param1 = 1.0 # VehicleCommand.ARMING_ACTION_ARM  # Arm
        arm_cmd.param2 = 0.0 # 0: arm-disarm unless prevented by safety checks, 21196: force arming or disarming
        self.drone_system.vehicle_command_publisher.publish(arm_cmd)
        self.drone_system.get_logger().info('Arming...')
        self.executed = True

    def done(self):
        if self.drone_system.vehicle_status is None:
            return False

        if self.drone_system.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED:
            self.drone_system.get_logger().info('Arming done!')
            self.finished = True
            return True
        return False
 

class TakeoffCommand(Command):
    def __init__(self, drone_system):
        super().__init__("Takeoff")
        self.drone_system = drone_system
        self.take_off_altitude = 2.5

    def execute(self):
        msg = TrajectorySetpoint()
        msg.position = [0.0, 0.0, -self.take_off_altitude]
        msg.velocity = [0.0, 0.0, -5.0]
        # msg.yaw = -3.14
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        if self.drone_system.trajectory_setpoint_publisher is None:
            return
        self.drone_system.trajectory_setpoint_publisher.publish(msg)
        self.drone_system.get_logger().info('Takeoff...')
        self.executed = True

    def done(self):
        if self.drone_system.local_position is None: return False
        # todo we should compare to a movinga verage of current altitude
        if abs(self.drone_system.local_position.z) < self.take_off_altitude*.9:
            return False

        self.finished = True
        self.drone_system.get_logger().info("Takeoff done!")

        return True


class HoverCommand(Command):
    def __init__(self, drone_system):
        super().__init__("Hover")
        self.drone_system = drone_system
        self.hover_height = 2.5
        self.pos = None

    def execute(self):

        if self.drone_system.local_position:
            if not self.pos:
                self.pos = self.drone_system.local_position
        else:
            return # Eeep.

        if abs(self.drone_system.local_position.z) > self.hover_height*.99:
            return

        msg = TrajectorySetpoint()
        msg.position = [self.pos.x, self.pos.y, self.pos.z]

        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        if self.drone_system.trajectory_setpoint_publisher is None:
            return
        self.drone_system.trajectory_setpoint_publisher.publish(msg)
        self.drone_system.get_logger().info('Hover...')
        self.executed = True

    def done(self):
        if self.finished or self.canceled:
            return True

        return False


class LandCommand(Command):
    def __init__(self, drone_system):
        super().__init__("Land")
        self.drone_system = drone_system

    def execute(self):
        self.drone_system.get_logger().info('Landing...')

        command = VehicleCommand()
        command.timestamp = int(Clock().now().nanoseconds / 1000)

        command.command = VehicleCommand.VEHICLE_CMD_NAV_LAND
        command.target_system = 1
        command.target_component = 1
        command.source_system = 1
        command.source_component = 1
        command.confirmation = 0  # No confirmation required
        self.drone_system.vehicle_command_publisher.publish(command)

    def done(self):
        if self.drone_system.landing:
            return True
        return False

class TargetCommand(Command):
    def __init__(self, drone_system):
        super().__init__("Target")
        self.drone_system = drone_system
        self.current_target_heading = None
        self.current_target_pitch = None
        self.target_radius_expected = 0.3 # balloon_config.config.get_float('balloon','radius_m',0.3)
        self.target_distance = None
        self.vel_dist_ratio = 0.5 # balloon_config.config.get_float('general','VEL_DIST_RATIO', 0.5)
        self.img_width = 1920 # Tools/simulation/gz/models/OakD-Lite/model.sdf
        self.img_height = 1080 # L 42, 43
        self.cam_hfov = None
        self.cam_vfov = None
        self.vel_speed_last = 0.0

        xy_p = 1.0
        xy_i = 0.0
        xy_d = 0.0
        xy_imax = 10.0 # maximum is 10 degree lean
        self.vel_xy_pid = pid(xy_p, xy_i, xy_d, math.radians(xy_imax))

        z_p = 2.5
        z_i = 0.0
        z_d = 0.0
        z_imax = 10.0
        self.vel_z_pid = pid(z_p, z_i, z_d, math.radians(z_imax))

    # rotate_xy - rotates an x,y pixel position around the center of the image by the specified angle (in radians)
    # x and y : position integer or floats holding the pixel location as an offset from center or angle offset from center
    # angle : float holding the angle (in radians) to rotate the position.  +ve = rotate clockwise, -ve = rotate counter clockwise
    # returns x,y rotated by the angle
    def rotate_xy(self, x, y, angle, img_width, img_height):
        cos_ang = math.cos(angle)
        sin_ang = math.sin(angle)
        x_centered = x - img_width
        y_centered = y - img_height
        x_rotated = x * cos_ang - y * sin_ang
        y_rotated = x * sin_ang + y * cos_ang
        return x_rotated, y_rotated

    def pixels_to_direction(self, pixels_x, pixels_y, vehicle_roll, vehicle_pitch, vehicle_yaw):
        # rotate position by +ve roll angle
        coords = self.drone_system.coords
        img_width = self.img_width 
        img_height = self.img_height # line 42, 43

        hfov = 1.204
        cam_hfov = hfov *180 / math.pi

        tan_vfov = math.tan(hfov/2.) * (img_height/img_width)
        vfov = 2. * math.atan(tan_vfov)

        cam_vfov = vfov * 180 / math.pi

        x_rotated, y_rotated = self.rotate_xy(
            pixels_x - img_width/2,
            pixels_y - img_height/2,
            vehicle_roll,
            img_width,
            img_height
        )

        self.cam_vfov, self.cam_hfov = cam_vfov, cam_hfov

        # calculate vertical pixel shift from pitch angle
        pitch_pixel_shift = int(math.degrees(vehicle_pitch) / float(cam_vfov) * img_height)
        pitch_dir = (-y_rotated + pitch_pixel_shift) / float(img_height) * cam_vfov

        # calculate yaw shift in degrees
        yaw_dir = x_rotated / float(img_width) * float(cam_hfov) + math.degrees(vehicle_yaw)

        # return vertical angle to target and heading in degrees
        return pitch_dir, yaw_dir

    def wrap_PI(self, angle):
        if (angle > math.pi):
            return (angle - (math.pi * 2.0))
        if (angle < -math.pi):
            return (angle + (math.pi * 2.0))
        return angle

    # pixels_to_angle_x - converts a number of pixels into an angle in radians 
    def pixels_to_angle_x(self, num_pixels):
        return num_pixels * math.radians(self.cam_hfov) / self.img_width
    
    # pixels_to_angle_y - converts a number of pixels into an angle in radians 
    def pixels_to_angle_y(self, num_pixels):
        return num_pixels * math.radians(self.cam_vfov) / self.img_height

    def get_distance_from_pixels(self, size_in_pixels, actual_size):
        # avoid divide by zero by returning 9999.9 meters for zero sized object 
        if (size_in_pixels == 0):
            return 9999.9
        # convert num_pixels to angular size
        return actual_size / self.pixels_to_angle_x(size_in_pixels)

    # get_ef_velocity_vector - calculates the earth frame velocity vector in m/s given a yaw angle, pitch angle and speed in m/s
    #    pitch : earth frame pitch angle from vehicle to object.  +ve = above vehicle, -ve = below vehicle
    #    yaw : earth frame heading.  +ve = clockwise from north, -ve = counter clockwise from north
    #    velocity : scalar velocity in m/s
    def get_ef_velocity_vector(self, pitch, yaw, speed):
        cos_pitch = math.cos(pitch)
        x = speed * math.cos(yaw) * cos_pitch
        y = speed * math.sin(yaw) * cos_pitch
        z = speed * math.sin(pitch)
        return [x,y,-z]

    def execute(self):
        self.drone_system.get_logger().info('Target...')

        # if you borad cast [1,0,0] as a NED frame,  you will move the drone forward a little
        # msg = TrajectorySetpoint()
        # # msg.velocity = self.guided_target_vel
        # self.guided_target_vel = [1, 0, -1]
        # msg.velocity[0], msg.velocity[1], msg.velocity[2] = self.guided_target_vel[0], self.guided_target_vel[1], self.guided_target_vel[2]
        # self.drone_system.trajectory_setpoint_publisher.publish(msg)
        # return

        if not self.drone_system.found_target:
            pass
            # uhh, probably have a state for when we loose the target

        coords = self.drone_system.coords
        # if not coords:
        #     return

        if None in (self.drone_system.roll, self.drone_system.pitch, self.drone_system.yaw):
            return

        vehicle_roll, vehicle_pitch, vehicle_yaw = self.drone_system.roll, self.drone_system.pitch, self.drone_system.yaw

        self.new_target_pitch, self.new_target_heading = self.pixels_to_direction(
                coords['x'],
                coords['y'],
                vehicle_roll,
                vehicle_pitch,
                vehicle_yaw
            )


        if not self.current_target_heading: self.current_target_heading = self.new_target_heading
        if not self.current_target_pitch: self.current_target_pitch = self.new_target_pitch

        yaw_error =  self.wrap_PI(self.new_target_heading - self.current_target_heading) if self.current_target_heading else 0

        pitch_error = self.wrap_PI(self.new_target_pitch - self.current_target_pitch) if self.current_target_pitch else 0
        self.target_distance = self.get_distance_from_pixels(self.drone_system.coords['x_width'], self.target_radius_expected)

        speed = self.target_distance * self.vel_dist_ratio

        dt = 2 # self.vel_xy_pid.get_dt(2.0)
        vel_accel = 0.5 # max acceleration in m/s/s
        speed_chg_max = vel_accel * dt # max of 1 m / s / s
        speed = min(speed, self.vel_speed_last + speed_chg_max)
        speed = max(speed, self.vel_speed_last - speed_chg_max)
        self.vel_speed_last = speed

        yaw_correction = self.vel_xy_pid.get_pid(yaw_error, dt)
        yaw_final = self.wrap_PI(self.current_target_heading + yaw_correction)

        pitch_correction = self.vel_z_pid.get_pid(pitch_error, dt)
        pitch_final = self.wrap_PI(self.current_target_pitch + pitch_correction)

        # calculate velocity vector we wish to move in
        self.guided_target_vel = self.get_ef_velocity_vector(pitch_final, yaw_final, speed)  
        self.drone_system.get_logger().info("target_vel: %s" % str(self.guided_target_vel))
        msg = TrajectorySetpoint()
        # msg.velocity = self.guided_target_vel
        msg.velocity[0], msg.velocity[1], msg.velocity[2] = self.guided_target_vel[0], self.guided_target_vel[1], self.guided_target_vel[2]
        self.drone_system.trajectory_setpoint_publisher.publish(msg)
        # actually go to that point
        # msg = TrajectorySetpoint()
        # msg.position = guided_target_vel
        # self.drone_system.trajectory_setpoint_publisher.publish(msg)
        # self.send_nav_velocity(self.guided_target_vel[0], self.guided_target_vel[1], self.guided_target_vel[2])


    def done(self):
        # 💀
        return False

# import rclpy
# from rclpy.node import Node
# from px4_msgs.msg import MissionItem

# class WaypointPublisher(Node):
#     def __init__(self):
#         super().__init__('waypoint_publisher')
#         self.publisher = self.create_publisher(MissionItem, '/fmu/in/mission_item', 10)

#         # Define waypoints for the patrol mission
#         self.waypoints = [
#             self.create_waypoint(47.397742, 8.545594, 10.0),  # Waypoint 1
#             self.create_waypoint(47.398000, 8.546000, 10.0),  # Waypoint 2
#             self.create_waypoint(47.397742, 8.546500, 10.0),  # Waypoint 3
#             self.create_waypoint(47.397500, 8.545800, 10.0),  # Waypoint 4
#         ]

#         # Timer to publish waypoints sequentially
#         self.index = 0
#         self.timer = self.create_timer(1.0, self.publish_waypoints)

#     def create_waypoint(self, lat, lon, alt):
#         """Helper function to create a MissionItem."""
#         waypoint = MissionItem()
#         waypoint.timestamp = self.get_clock().now().nanoseconds // 1000
#         waypoint.seq = self.index
#         waypoint.frame = 3  # MAV_FRAME_GLOBAL_RELATIVE_ALT
#         waypoint.command = 16  # MAV_CMD_NAV_WAYPOINT
#         waypoint.current = 0 if self.index > 0 else 1  # Mark the first waypoint as current
#         waypoint.autocontinue = True
#         waypoint.param1 = 0.0  # Hold time
#         waypoint.param2 = 5.0  # Acceptance radius in meters
#         waypoint.param3 = 0.0  # Pass radius
#         waypoint.param4 = float('nan')  # Yaw (set NaN for auto)
#         waypoint.x = lat
#         waypoint.y = lon
#         waypoint.z = alt
#         self.index += 1
#         return waypoint

#     def publish_waypoints(self):
#         """Publish each waypoint in sequence."""
#         if self.index < len(self.waypoints):
#             self.publisher.publish(self.waypoints[self.index])
#             self.get_logger().info(f"Published waypoint {self.index}")
#         else:
#             self.get_logger().info("All waypoints sent.")
#             self.timer.cancel()

# def main(args=None):
#     rclpy.init(args=args)
#     node = WaypointPublisher()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()
