import agxSDK

"""KeyboardListener used to control the forces on motor on the boat"""


class Boat_Controller(agxSDK.StepEventListener):
    def __init__(self, ship, pid_boat, stepper):
        super().__init__()
        self.ship = ship
        self.pid_boat = pid_boat
        self.last_output = 0
        self.start = False
        self.stepper = stepper

    def pre(self, t):
        current_velocity = self.ship.getRigidBody('boat').getVelocity()[0] * 1.94384449
        self.pid_boat.compute(current_velocity)
        if (self.pid_boat.output > 0):
            self.ship.increase_propulsion(self.ship)
            self.last_output = self.pid_boat.output
        else:
            self.ship.decrease_propulsion(self.ship)
            self.last_output = self.pid_boat.output
