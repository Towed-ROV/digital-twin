# AGX imports
import demoutils
import agx
import agxSDK
from pid import PID_Controller
from keyboard_listener import KeyboardListener
from functions import deg2rad, rad2deg, limit
import numpy as np
import pandas as pd
import math
from Assembly import create_rov_body, create_wing_right, create_wing_left
from modules.agxPythonModules.utils.callbacks import StepEventCallback as Sec
from Sensor import Sensor

"""Class rovAssembly, creates a agx assembly of the rov, with to hinges, one on each wings"""


class rovAssembly(agxSDK.Assembly):
    def __init__(self, keyboard, seaFloor, wing_scale=1, depth=60):
        super().__init__()
        len1 = 2
        self.keyboard = keyboard
        self.plot_pitch = []
        self.plot_wing_angle = []
        self.plot_depth = []
        self.plot_roll = []
        self.plotted = False
        self.ehco_lod = Sensor(seaFloor, self, depth)
        aluminum = agx.Material('AluminumMaterial')
        aluminum.getBulkMaterial().setDensity(1027)
        aluminum1 = agx.Material('AluminumMaterial')
        aluminum1.getBulkMaterial().setDensity(1027)  # 706.8)
        self.link1 = create_rov_body(aluminum)
        self.link1.setName('rovBody')
        self.link1.setCmLocalTranslate(agx.Vec3(0.27511, -0.18095, 0.0494))
        self.link2 = create_wing_right(aluminum1, scale=wing_scale)
        self.link2.setPosition(0.05, 0.25, 0)
        link2rot = agx.Vec3(0.1, 0.25, 0)
        self.link2.setRotation(agx.EulerAngles(0, 0, 0))
        self.link2.setName("wing_r")
        self.link3 = create_wing_left(aluminum1, scale=wing_scale)
        self.link3.setPosition(0.05, -0.25, 0)
        link3rot = agx.Vec3(0.1, -0.25, 0)
        self.link3.setRotation(agx.EulerAngles(0, 0, 0))
        self.link3.setName("wing_l")

        self.link1.getGeometry('Rov_body').setEnableCollisions(self.link2.getGeometry('Wing_R'), False)
        self.link1.getGeometry('Rov_body').setEnableCollisions(self.link3.getGeometry('wing_L'), False)

        demoutils.create_visual(self.link1)
        demoutils.create_visual(self.link2)
        demoutils.create_visual(self.link3)
        #self.link1.getMassProperties().setMass(self.link1.getMassProperties().getMass() / 2)

        self.hinge1 = self.build_hinge(link=self.link1, part=self.link2,
                                       pos=link2rot, axis=agx.Vec3(0, 1, 0))
        self.hinge1.setCompliance(1e-5)
        self.hinge1.getLock1D().setEnable(False)
        self.hinge1.getMotor1D().setEnable(True)
        self.hinge1.getRange1D().setEnable(True)
        self.hinge1.getRange1D().setRange(deg2rad(-45), deg2rad(45))

        self.hinge2 = self.build_hinge(self.link1, self.link3,
                                       pos=link3rot, axis=agx.Vec3(0, 1, 0))
        self.hinge2.setCompliance(1e-5)
        self.hinge2.getLock1D().setEnable(False)
        self.hinge2.getMotor1D().setEnable(True)
        self.hinge2.getRange1D().setEnable(True)
        self.hinge2.getRange1D().setRange(deg2rad(-45), deg2rad(45))

        self.ehco_lod.beam.setPosition(agx.Vec3(0, 0, 0))
        self.link4=agx.RigidBody(self.ehco_lod.beam)
        self.link4.setPosition(agx.Vec3(0,0,-depth))
        f2 = agx.Frame()
        f3 = agx.Frame()
        f2.setTranslate(agx.Vec3(0,0,0))
        f3.setTranslate(agx.Vec3(0,0,0))
        self.sonar_hinge = self.build_lock_joint(self.link1, self.link4,f2,f3)

        self.add(self.link1)
        self.add(self.link2)
        self.add(self.link3)
        self.add(self.link4)
        self.add(self.hinge1)
        self.add(self.hinge2)
        self.add(self.sonar_hinge)
        self.left_wing_angle = lambda: self.hinge1.getAngle()
        self.right_wing_angle = lambda: self.hinge2.getAngle()
        self.wing_step_length = deg2rad(2)
        self.setName('rov')


    def disable_col(self, geo, ruged: agx.RigidBody):
        for geometries in ruged.getGeometries():
            geo.setEnableCollisions(geometries, False)

    def build_hinge(self, link, part, pos, axis):
        return demoutils.create_constraint(
            pos=pos,
            axis=axis,
            rb1=link,
            rb2=part,
            c=agx.Hinge)  # type: agx.Hinge

    def build_lock_joint(self, part1, part2, pos1, pos2):
        return demoutils.create_constraint(
            pos1=pos1,
            pos2=pos2,
            rb1=part1,
            rb2=part2,
            c=agx.LockJoint)  # type: agx.LockJoint

    def getGeometries(self) -> "agxCollide::GeometryRefSet const &":
        a = agxSDK.Assembly.getGeometries(self)
        return a

    def update_wings(self, sb_p, port_p):

        a1 = -self.hinge1.getAngle()
        a2 = -self.hinge2.getAngle()
        d1 = limit(a1 - sb_p,-2,2)*10
        d2 = limit(a2 - port_p, -2, 2)*10
        self.hinge1.getMotor1D().setSpeed(d1)
        self.hinge2.getMotor1D().setSpeed(d2)

    def plotter(self, plot):
        self.plot_depth.append(self.link1.getPosition()[2])
        self.plot_pitch.append(self.link1.getRotation()[0] * 10)
        self.plot_roll.append(self.link1.getRotation()[1] * 100)

        if plot:

            """plots stored values to csv file"""
            plot_wing_angle = np.array(self.plot_wing_angle)
            plot_depth = np.array(self.plot_depth)
            plot_pitch = np.array(self.plot_pitch)
            plot_roll = np.array(self.plot_roll)

            pd.DataFrame(plot_depth).to_csv("D:\ROV_BATCHELOR\Code\AGX-towed-rov-simulation\plotDepth.csv")
            pd.DataFrame(plot_pitch).to_csv("D:\ROV_BATCHELOR\Code\AGX-towed-rov-simulation\plotPitch.csv")
            pd.DataFrame(plot_wing_angle).to_csv("D:\ROV_BATCHELOR\Code\AGX-towed-rov-simulation\plot_wing_angle.csv")
            pd.DataFrame(plot_roll).to_csv("D:\ROV_BATCHELOR\Code\AGX-towed-rov-simulation\pplot_roll.csv")
            self.plotted = True


if __name__ == "__main__":
    kp = 1000
    ki = 1000
    kd = 1

    pid = PID_Controller(kp, ki, kd, 0)
    pid.setName('pid')
    pid.set_output_limits(-45, 45)
    pid.set_mode(1, 0, 0)
    pid.set_setpoint(-20)
    keyboard = KeyboardListener(pid, True)

    """Creates the rov"""
    rov = rovAssembly(pid, keyboard)
    rov.setPosition(agx.Vec3(0, 0, 0))
    rov.setName("rov")
    rov.setRotation(agx.EulerAngles(0, 0, math.pi))

    rov.displayForces(1)
