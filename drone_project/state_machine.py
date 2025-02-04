class StateMachine:
    def __init__(self, initial_state):
        self.state = initial_state
        self.transitions = {}

    def add_transition(self, trigger, source, dest, before=None, after=None):
        if source not in self.transitions:
            self.transitions[source] = {}
        self.transitions[source][trigger] = (dest, before, after)
        setattr(self, trigger, lambda trigger=trigger: self.trigger(trigger))

    def trigger(self, trigger):
        state_transitions = self.transitions.get(self.state)
        if state_transitions and trigger in state_transitions:
            dest, before, after = state_transitions[trigger]
            if before:
                before()  # Call the 'before' function if it exists
            print(f"Transitioning from {self.state} to {dest}")
            self.state = dest
            if after:
                after()  # Call the 'after' function if it exists
        else:
            print(f"No transition for trigger '{trigger}' from state '{self.state}'")

class DroneSM:
    def __init__(self, initial_state=None, drone_system=None):
        if initial_state is None:
            initial_state = 'land'

        self.drone_system = drone_system

        self.machine = StateMachine(initial_state=initial_state)
        self.machine.add_transition('armCmd', source='land', dest='arm', after=self.on_arm)
        self.machine.add_transition('takeoffCmd', source='arm', dest='takeoff', after=self.on_takeoff)
        self.machine.add_transition('patrolCmd', source='takeoff', dest='patrol', after=self.on_patrol)
        self.machine.add_transition('flytoCmd', source='patrol', dest='flyto', after=self.on_flyto)

        self.machine.add_transition('patrolLand', source='patrol', dest='land', after=self.on_land)
        self.machine.add_transition('takeoffLand', source='takeoff', dest='land', after=self.on_land)
        self.machine.add_transition('armErr', source='arm', dest='land', after=self.on_land)

    def on_arm(self):
        pass

    def on_takeoff(self):
        pass

    def on_patrol(self):
        pass

    def on_flyto(self):
        pass

    def on_land(self):
        pass

# class Matter:
#     def __init__(self):
#         self.machine = StateMachine(initial_state='solid')
#         self.machine.add_transition('melt', source='solid', dest='liquid', before=self.on_melt)
#         self.machine.add_transition('evaporate', source='liquid', dest='gas', before=self.on_evaporate)
#         self.machine.add_transition('condense', source='gas', dest='liquid', before=self.on_condense)

#     def on_melt(self):
#         print("Executing 'melt' transition")

#     def on_evaporate(self):
#         print("Executing 'evaporate' transition")

#     def on_condense(self):
#         print("Executing 'condense' transition")

# # Usage
# matter = Matter()
# matter.machine.melt()       # Calls melt transition; Output: Executing 'melt' transition; Transitioning from solid to liquid
# matter.machine.evaporate()  # Calls evaporate transition; Output: Executing 'evaporate' transition; Transitioning from liquid to gas
# matter.machine.condense()   # Calls condense transition; Output: Executing 'condense' transition; Transitioning from gas to liquid
