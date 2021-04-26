import agxSDK
from Rov.rov_assembly import rovAssembly
from pid import PID_Controller
from functions import deg2rad, rad2deg
import demoutils
"""Class RovController for Rov and by using StepEventListener it updates the wing position every step of the simulation"""


class RovController(agxSDK.StepEventListener):
    def __init__(self, rov: rovAssembly, pid: PID_Controller, pid_trim: PID_Controller):
        super().__init__()
        self.rov = rov
        self.pid = pid
        self.pid_trim = pid_trim
        self.last_output = 0
        self.__start = False

    """Runs every time before the simulation takes a step"""

    def pre(self, t):
        current_depth = self.rov.getPosition()[2]
        self.pid.compute(current_depth)
        output_left = deg2rad(self.pid.output)# - self.pid_trim.output)
        output_right = deg2rad(self.pid.output) #+ self.pid_trim.output)
        self.rov.update_wings(port_p=output_right, sb_p=output_left)
        demoutils.app().getSceneDecorator().setText(9,"pid : {}, wing: {}".format(str(self.pid.output),round(rad2deg(self.rov.left_wing_angle()),2)))
