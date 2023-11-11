from px4_msgs.msg import SensorCombined, VehicleStatus, VehicleCommand, OffboardControlMode, TrajectorySetpoint, VehicleGlobalPosition, VehicleLocalPosition

class Command:
    def __init__(self, name):
        self.name = name
        self.executed = False

    def execute(self):
        pass

class VehicleCommandPublisher(Command):
    def __init__(self, control_module=None, 
        command=None, param1=None, param2=None):
        super().__init__("VehicleCommandPublisher")
        self.control_module = control_module
        self.param1 = param1
        self.param2 = param2
        self.command = command

    def execute(self):
        if self.command is None:
            raise Exception("command is missing")

        self.control_module.publish_vehicle_command(
                command=command,
                param1=param1,
                param2=param2
            )


class SetpointCommand(Command):
    def __init__(self, control_module, x, y, z, yaw=None):
        super().__init__("SetpointCommand")
        self.x = x
        self.y = y
        self.z = z
        self.yaw = yaw
        
        self.control_module = control_module

    def execute(self):
        msg = TrajectorySetpoint()
        msg.position = [self.x, self.y, self.z]
        msg.yaw = 1.57079  # (90 degree)
        # msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.control_module.trajectory_setpoint_publisher.publish(msg)
        # self.get_logger().info(f"Publishing position setpoints {[x, y, z]}")

class MoveForwardCommand(Command):
    def __init__(self, control_module):
        super().__init__("MoveForwardCommand")
        self.control_module = control_module

    def execute(self):
        self.control_module.move_forward()
        self.executed = True
class ArcToGroundCommand(Command):
    def __init__(self, control_module, target_position, arc_radius):
        super().__init__("ArcToGround")
        self.control_module = control_module
        self.target_position = target_position
        self.arc_radius = arc_radius

    def execute(self):
        print("Executing arc to ground command")
        # self.control_module.control_arc_to_ground()
        self.control_module.trajectory(1.0, 1.0, 0.0, 0)#-3.14)
        self.executed = True

class ArmCommand(Command):
    def __init__(self, control_module):
        super().__init__("Arm")
        self.control_module = control_module

    def execute(self):
        VehicleCommandPublisher(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        self.control_module.arm()

class TakeoffCommand(Command):
    def __init__(self, control_module):
        super().__init__("Takeoff")
        self.control_module = control_module

    def execute(self):
        self.control_module.offboard_control()
        # SetpointCommand(control_module=self.control_module, 
        #     x=0, y=0, z=-2.5).execute()
        self.control_module.control_takeoff()
        self.executed = True

class LandCommand(Command):
    def __init__(self, control_module):
        super().__init__("Land")
        self.control_module = control_module

    def execute(self):
        print("Executing land command")
        self.control_module.control_land()
        self.executed = True

class CommandManager:
    def __init__(self, control_module):
        self.commands_queue = []
        self.control_module = control_module

    def add_command(self, command):
        self.commands_queue.append(command)

    def execute_next_command(self):
        if len(self.commands_queue) > 0:
            next_command = self.commands_queue.pop(0)
            next_command.execute()
        else:
            raise Exception("out of commands")
