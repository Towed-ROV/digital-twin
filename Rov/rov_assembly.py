#AGX imports
import demoutils
import agx
import agxCollide
import agxSDK
from pid import PID_Controller
#python imports
from keyboard_listener import KeyboardListener
import numpy as np
import pandas as pd
import math
#local imports
from Assembly import build_rov, create_spoiler
from modules.agxPythonModules.utils.callbacks import StepEventCallback as Sec
from functions import _map

"""Class rovAssembly, creates a agx assembly of the rov, with to hinges, one on each wings"""
class rovAssembly(agxSDK.Assembly):
    def __init__(self,keyboard):
        super().__init__()
        len1 = 2
        self.keyboard = keyboard
        self.plot_pitch = []
        self.plot_wing_angle = []
        self.plot_depth = []
        self.plot_roll = []
        self.plotted = False

        aluminum = agx.Material('AluminumMaterial')
        aluminum.getBulkMaterial().setDensity(1020)
        aluminum1 = agx.Material('AluminumMaterial')
        aluminum1.getBulkMaterial().setDensity(706.8)
        tetst, tet, tett = build_rov(aluminum)
        self.link1 = agx.RigidBody()
        link1 = agxCollide.Geometry(agxCollide.Box(0.6,0.4,0.18))
        link1.setMaterial(aluminum)
        self.link1.add(link1)
        self.link2 = agx.RigidBody()
        link2 = agxCollide.Geometry(agxCollide.Box(0.2, 0.1, 0.006))
        link2.setMaterial(aluminum)
        self.link2.add(link2)
        self.link3 = agx.RigidBody()
        link3  = agxCollide.Geometry(agxCollide.Box(0.2, 0.1, 0.006))
        link3.setMaterial(aluminum)
        self.link3.add(link3)
        print("pos: ", self.link1.getCmPosition())

        print("mass: ", self.link1.getMassProperties().getMass())
        self.link1.setName('rovBody')
        # self.link1.setCmLocalTranslate(agx.Vec3(0.27511,-0.18095, 0.0494))
        print("cmpos: ", self.link1.getCmPosition())
        self.link2.setPosition(-0.2,0.5,0)
        self.link2.setRotation(agx.EulerAngles(0, 0, math.pi))
        self.link3.setPosition(-0.2,-0.5,0)
        self.link3.setRotation(agx.EulerAngles(0, 0, math.pi))
        self.spoiler = create_spoiler()
        self.spoiler.setPosition(1, -0.29, 0.45)
        self.spoiler.setRotation(agx.EulerAngles(0,-0.5,0))

        # self.link1.getGeometry('rov_body').setEnableCollisions(self.link2.getGeometry('wingR'), False)
        # self.link1.getGeometry('rov_body').setEnableCollisions(self.link3.getGeometry('wingL'), False)

        demoutils.create_visual(self.link1)
        demoutils.create_visual(self.link2)
        demoutils.create_visual(self.link3)
        # demoutils.create_visual(self.spoiler)

        self.wire_pos = (0, 0, 0)
        self.wire_pos2 = [20, 0, 20]

        self.hinge1 = demoutils.create_constraint(
            pos=agx.Vec3(-0.2,-0.5,0),
            axis=agx.Vec3(0, 1, 0),
            rb1=self.link1,
            rb2=self.link2,
            c=agx.Hinge)  # type: agx.Hinge
        self.hinge1.setCompliance(1e-6)
        self.hinge1.getLock1D().setEnable(False)
        self.hinge1.getMotor1D().setEnable(False)
        self.hinge1.getRange1D().setEnable(True)

        self.hinge2 = demoutils.create_constraint(
            pos=agx.Vec3(-0.2,0.5,0),
            axis=agx.Vec3(0, 1, 0),
            rb1=self.link1,
            rb2=self.link3,
            c=agx.Hinge)  # type: agx.Hinge
        self.hinge2.setCompliance(1e-6)
        self.hinge2.getLock1D().setEnable(False)
        self.hinge2.getMotor1D().setEnable(False)
        self.hinge2.getRange1D().setEnable(True)

        self.hinge3 = demoutils.create_constraint(
            pos=agx.Vec3(0.2, -0.985, 0.22),
            axis=agx.Vec3(0, 1, 0),
            rb1=self.link1,
            rb2=self.spoiler,
            c=agx.Hinge)  # type: agx.Hinge
        self.hinge3.setCompliance(1e-6)
        self.hinge3.getLock1D().setEnable(False)
        self.hinge3.getMotor1D().setEnable(False)

        f1 = agx.Frame()
        f1.setTranslate(agx.Vec3(0.5, 0, len1 - (len1 * 0.7)))
        f2 = agx.Frame()
        f2.setTranslate(agx.Vec3(-0.5, 0, 0))
        f3 = agx.Frame()
        f3.setTranslate(agx.Vec3(-0.5, 0, 0))
        f4 = agx.Frame()
        f4.setTranslate(agx.Vec3(-0.5, 0, 0))
        self.distance1 = agx.DistanceJoint(self.link1, f1, self.link2, f2)
        # self.distance1.getMotor1D().setEnable(False)
        # self.distance1.getLock1D().setEnable(True)
        self.distance2 = agx.DistanceJoint(self.link1, f1, self.link3, f3)
        self.distance2.getMotor1D().setEnable(False)
        self.distance2.getLock1D().setEnable(True)
        self.distance3 = agx.DistanceJoint(self.link1, f1, self.spoiler, f4)
        self.distance3.getLock1D().setEnable(True)

        self.add(self.link1)
        self.add(self.link2)
        self.add(self.link3)
        self.add(self.spoiler)
        self.add(self.hinge1)
        self.add(self.hinge2)
        self.add(self.hinge3)
        self.add(self.distance1)
        self.add(self.distance2)
        self.add(self.distance3)
        self.setName('rov')
        Sec.postCallback(lambda t: self.displayForces(t))

    def displayForces(self, t):
        plot = self.keyboard.plot
        pos = self.link1.getPosition()[2]
        demoutils.app().getSceneDecorator().setText(3, "Rov Position in Z direction : {} M".format(str(round(pos, 2))))
        demoutils.app().getSceneDecorator().setText(4, "Pitch : {}".format(str(round(self.link1.getRotation()[0], 2))))
        demoutils.app().getSceneDecorator().setText(5, "Roll : {}".format(str(round(self.link1.getRotation()[1], 2))))
        demoutils.app().getSceneDecorator().setText(7, "distance : {}".format(str(round(self.link1.getPosition()[0], 2))))
        self.plot_depth.append(pos)
        self.plot_pitch.append(self.link1.getRotation()[0] * 100)
        self.plot_roll.append(self.link1.getRotation()[1] * 100)
        self.plot_wing_angle.append(_map(self.distance1.getAngle(), 0.753, 1.05, -45, 45))

        if plot and not self.plotted:
            """plots stored values to csv file"""
            plot_wing_angle = np.array(self.plot_wing_angle)
            plot_depth = np.array(self.plot_depth)
            plot_pitch = np.array(self.plot_pitch)
            plot_roll = np.array(self.plot_roll)

            pd.DataFrame(plot_depth).to_csv("D:\ROV_BATCHELOR\Code\AGX-towed-rov-simulation\plotDepth.csv")
            pd.DataFrame(plot_pitch).to_csv("D:\ROV_BATCHELOR\Code\AGX-towed-rov-simulation\plotPitch.csv")
            pd.DataFrame(plot_wing_angle).to_csv("D:\ROV_BATCHELOR\Code\AGX-towed-rov-simulation\plot_wing_angle.csv")
            pd.DataFrame(plot_roll).to_csv("D:\ROV_BATCHELOR\Code\AGX-towed-rov-simulation\pplot_roll.csv")
            print('check csv')
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