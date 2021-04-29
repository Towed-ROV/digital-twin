import agxSDK
from Rov.rov_assembly import RovAssembly
from pid import PID_Controller
from functions import deg2rad, rad2deg
import demoutils

"""Class RovController for Rov and by using StepEventListener it updates the wing position every step of the simulation"""


class RovController(agxSDK.StepEventListener):
    def __init__(self, rov: RovAssembly, pid: PID_Controller, pid_trim: PID_Controller, depth):
        """

        Args:
            rov:
            pid:
            pid_trim:
            depth:
        """
        super().__init__()
        self.rov = rov
        self.pid = pid
        self.pid_trim = pid_trim
        self.last_output = 0
        self.__start = False
        pid.set_output_limits(-45, 45)
        self.depth = depth

    """Runs every time before the simulation takes a step"""

    def pre(self, t):
        """

        Args:
            t:
        """
        current_depth = self.rov.getRigidBody('rovBody').getPosition()[2]
        self.pid.compute(current_depth)
        output_left = deg2rad(-self.pid.output)  # - self.pid_trim.output)
        output_right = deg2rad(-self.pid.output)  # + self.pid_trim.output)
        self.rov.update_wings(port_p=output_right, sb_p=output_left)
        self.rov.update_wings(output_left, output_right)

    def post(self, t):
        """ can send depth and everything form here aswell

        Args:
            t:
        """
        pos = self.rov.link1.getPosition()
        rot = self.rov.link1.getRotation()
        decorator = demoutils.app().getSceneDecorator()
        decorator.setText(9, "pid : {}, wing: {}".format(str(self.pid.output), round(
            rad2deg(self.rov.left_wing_angle()), 2)))
        decorator.setText(3, "Rov Position in Z direction : {} M".format(str(round(pos[2], 2))))
        decorator.setText(4, "Pitch : {}".format(str(round(rot[0] * 100, 2))))
        decorator.setText(5, "Roll : {}".format(str(round(rot[1] * 100, 2))))
        x, y = int(520 + pos[0]), int(pos[1])
        decorator.setText(5, "seafloor actual : {}".format(str(round(self.depth.getHeight(x, y) - pos[2], 2))))
        decorator.setText(7, "distance : {}M".format(str(round(self.rov.link1.getPosition()[0], 2))))
