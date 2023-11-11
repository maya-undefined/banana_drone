
class StateEstimationManager:
    def __init__(self, vehicle_local_position_s, logger):
        self.vehicle_local_position_s = vehicle_local_position_s
        self.logger = logger
        self.current_state = "IDLE"
        self.position = [0.0, 0.0, 0.0]
        self.lat = None
        self.lon = None
        self.alt = None

    def update(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.z = msg.z
        # print(f"Updated position: {self.position}")

    def get(self):
        return (self.x, self.y, self.z)

    # def set_state(self, state):
    #     self.current_state = state
    #     print(f"Current state: {self.current_state}")

    def distance(self, x, y, z):
        return (
                (self.x-x)**2 +
                (self.y-y)**2 +
                (self.z-z)**2
            )**0.5

    def get_logger(self):
        return self.logger

    @staticmethod
    def timer_cb(self, msg=None):
        print("asdf=", msg, "|")
