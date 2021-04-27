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
from Assembly import create_rov_body, create_wing_right, create_wing_left, create_spoiler, _map
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
        # self.ehco_lod.setPossition(agx.Vec3(0, 0, -self.ehco_lod.getHeight()/2))

        self.spoiler = create_spoiler()
        # self.spoiler.setPosition(0.7, -0.3,- 0.3)
        # self.spoiler.setRotation(agx.EulerAngles(0, -0.5, 0))
        # self.spoiler.setName("spoiler_wing")

        self.link1.getGeometry('Rov_body').setEnableCollisions(self.link2.getGeometry('Wing_R'), False)
        self.link1.getGeometry('Rov_body').setEnableCollisions(self.link3.getGeometry('wing_L'), False)
        # self.link1.getGeometry('Rov_body').setEnableCollisions(self.spoiler.getGeometry('spoiler'), False)

        demoutils.create_visual(self.link1)
        demoutils.create_visual(self.link2)
        demoutils.create_visual(self.link3)
        demoutils.create_visual(self.spoiler)
        self.link1.getMassProperties().setMass(self.link1.getMassProperties().getMass() / 2)
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

        self.hinge3 = self.build_hinge(self.link1, self.spoiler,
                                       pos=self.spoiler.getPosition(), axis=agx.Vec3(0, 0, 0))
        self.link1.add(self.ehco_lod.beam)
        # self.hinge3.setCompliance(1e-6)
        # self.hinge3.getLock1D().setEnable(False)
        # self.hinge3.getMotor1D().setEnable(False)
        f1 = agx.Frame()
        f1.setTranslate(agx.Vec3(0.05, 0, 0.05))
        f2 = agx.Frame()
        f2.setTranslate(agx.Vec3(0, 0, 0))
        f3 = agx.Frame()
        f3.setTranslate(agx.Vec3(0, 0, 0))
        f4 = agx.Frame()
        f4.setTranslate(agx.Vec3(0, 0, 0))
        f5 = agx.Frame()
        f5.setTranslate(agx.Vec3(0, 0, 0))

        self.add(self.link1)
        self.add(self.link2)
        self.add(self.link3)
        self.add(self.hinge1)
        self.add(self.hinge2)
        self.ehco_lod.addSim((demoutils.sim()))

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
        self.plot_wing_angle.append(_map(self.distance1.getAngle(), 0.753, 1.05, -45, 45))

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
