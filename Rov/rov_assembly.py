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
from Rov.assembly import rov_builder
from modules.agxPythonModules.utils.callbacks import StepEventCallback as Sec
from rov_simulation_parameters import *

"""Class rovAssembly, creates a agx assembly of the rov, with to hinges, one on each wings"""


class RovAssembly(agxSDK.Assembly):
    def __init__(self, keyboard):
        """
        initializes adn adds all bodies and constaints to the ROV.
        Args:
            keyboard: keyboard event listener for direct controll
       """
        super().__init__()

        self.keyboard = keyboard
        self.plot_pitch = []
        self.plot_wing_angle = []
        self.plot_depth = []
        self.plot_roll = []
        self.plotted = False
        print("initilaized master and vields")

        # building models
        rov_material = self.build_material('AluminumMaterial', ROV_BODY_DENSITY)
        wing_material = self.build_material('AluminumMaterial', ROV_WING_DENSITY)
        tank_material = self.build_material('AluminumMaterial', ROV_TANK_DENSITY)
        builder = rov_builder()
        print("rov builder ready")

        # rov body
        self.link1 = builder.create_rov_body(rov_material, name='rovBody', scale=ROV_SCALE,
                                             cm=(0.27511, -0.18095, 0.0494), tank_material=tank_material)

        print("buildt rov body")
        print("volumebounds: ", self.link1.getGeometry("Rov_body").getBoundingVolume().size())
        # wing left
        self.link2 = builder.create_wing_right(wing_material, WING_SCALE, "wing_r", rot=(0, 0, 0))
        link2rot = self.link2.getPosition()
        print("buildt right wing", link2rot)
        # wing right
        self.link3 = builder.create_wing_left(wing_material, WING_SCALE, "wing_l", rot=(0, 0, 0))
        link3rot = self.link3.getPosition()
        print("buildt left wing")

        # disabling internal collisions
        self.link1.getGeometry('Rov_body').setEnableCollisions(self.link2.getGeometry('wing_r'), False)
        self.link1.getGeometry('Rov_body').setEnableCollisions(self.link3.getGeometry('wing_l'), False)

        print("removed internal collitions")

        # adding visualisation
        demoutils.create_visual(self.link1)
        demoutils.create_visual(self.link2)
        demoutils.create_visual(self.link3)
        print("created visuals")

        # conecting models
        # left wing
        self.hinge1 = self.build_hinge(link=self.link1, part=self.link2,
                                       pos=link2rot, axis=agx.Vec3(0, 1, 0), lock=False, motor=True,
                                       range=(MIN_WING_ANGLE, MAX_WING_ANGLE), compliance=1e-5)
        # right wing
        self.hinge2 = self.build_hinge(self.link1, self.link3,
                                       pos=link3rot, axis=agx.Vec3(0, 1, 0), lock=False, motor=True,
                                       range=(MIN_WING_ANGLE, MAX_WING_ANGLE), compliance=1e-5)
        # sonar
        print("buildt joints")
        # adding models to assembly
        self.add(self.link1)
        self.add(self.link2)
        self.add(self.link3)
        print("added models to assembly")
        self.add(self.hinge1)
        self.add(self.hinge2)
        print("aded links to assembly")
        self.left_wing_angle = lambda: self.hinge1.getAngle()
        self.right_wing_angle = lambda: self.hinge2.getAngle()
        print("set the wing controll functions")
        self.wing_step_length = deg2rad(2)
        print("added wing step lenght")
        self.setName('rov')
        print("set name")
        Sec.postCallback(self.post)
        self.last = 0

    @staticmethod
    def disable_col(geo, ruged: agx.RigidBody):
        """

        Args:
            geo:
            ruged:
        """
        for geometries in ruged.getGeometries():
            geo.setEnableCollisions(geometries, False)

    @staticmethod
    def build_hinge(link, part, pos, axis, lock, motor, compliance, range=None) -> agx.Hinge:
        """

        Args:
            range:
            compliance:
            lock:
            motor:
            link:
            part:
            pos:
            axis:

        Returns:

        """
        hinge = demoutils.create_constraint(pos=pos, axis=axis, rb1=link, rb2=part, c=agx.Hinge)  # type: agx.Hinge
        hinge.setCompliance(compliance)
        hinge.getLock1D().setEnable(lock)
        hinge.getMotor1D().setEnable(motor)
        if range:
            hinge.getRange1D().setEnable(True)
            hinge.getRange1D().setRange(deg2rad(range[0]), deg2rad(range[1]))
        else:
            hinge.getRange1D().setEnable(False)
        return hinge

    @staticmethod
    def build_lock_joint(part1, part2, pos1, pos2) -> agx.LockJoint:
        """
        creates a agx lock joint between two parts and returns the resoult
        Args:
            part1:
            part2:
            pos1: The lock possition on part 1
            pos2: The lock possition on part 2

        Returns:
            a  lock joint
        """
        f1 = agx.Frame()
        f2 = agx.Frame()
        f1.setTranslate(agx.Vec3(*pos1))
        f2.setTranslate(agx.Vec3(*pos2))
        return demoutils.create_constraint(
            pos1=f1,
            pos2=f2,
            rb1=part1,
            rb2=part2,
            c=agx.LockJoint)  # type: agx.LockJoint

    def getGeometries(self) -> "agxCollide::GeometryRefSet const &":
        """
        returns the geometries in the assembly

        """
        a = agxSDK.Assembly.getGeometries(self)
        return a

    def getPitch(self) -> "agx::Quat":
        """
        returns the pithc of the assembly
        """
        return round(self.link1.getRotation()[0] * 100, 1)

    def get_wing_agles(self):
        port = rad2deg(self.link2.getRotation().getAngle() - math.pi)
        sb = rad2deg(self.link3.getRotation().getAngle() - math.pi)
        return port, sb

    def update_wings(self, sb_p, port_p):
        """
        uptates the angle of the wings.
        Args:
            sb_p: starboard wing angle
            port_p: portside wing angle

        """
        sb_p = deg2rad(sb_p)
        port_p = deg2rad(port_p)
        a1 = -self.hinge1.getAngle()
        a2 = -self.hinge2.getAngle()
        d1 = limit(a1 - sb_p, -2, 2) * 100
        d2 = limit(a2 - port_p, -2, 2) * 100
        # print(d1,d2,a1)
        self.hinge1.getMotor1D().setSpeed( d1)
        self.hinge2.getMotor1D().setSpeed( d2)

    def plotter(self, t):
        """
        plots datafrom the assembly to a csvfile.
        Args:
            t: in simulation time
        """
        if t - self.last > 0.1:
            #print(t)
            self.last = t
            self.plot_depth.append(round(self.link1.getPosition()[2], 1))
            self.plot_pitch.append(round(self.link1.getRotation()[0] * 100, 1))
            self.plot_roll.append(round(self.link1.getRotation()[1] * 100, 1))
            self.plot_wing_angle.append(round(self.link2.getRotation()[0] * 100, 1))

            #print(len(self.plot_depth))
            """plots stored values to csv file"""
            plot_wing_angle = np.array(self.plot_wing_angle)
            plot_depth = np.array(self.plot_depth)
            plot_pitch = np.array(self.plot_pitch)
            plot_roll = np.array(self.plot_roll)
            pd.DataFrame(plot_depth).to_csv(
                "D:\ROV_BATCHELOR\Code\AGX-towed-rov-simulation\AGX-towed-rov-simulation\plots\plotDepth.csv")
            pd.DataFrame(plot_pitch).to_csv(
                "D:\ROV_BATCHELOR\Code\AGX-towed-rov-simulation\AGX-towed-rov-simulation\plots\plotPitch.csv")
            pd.DataFrame(plot_wing_angle).to_csv(
                "D:\ROV_BATCHELOR\Code\AGX-towed-rov-simulation\AGX-towed-rov-simulation\plots\plot_wing_angle.csv")
            pd.DataFrame(plot_roll).to_csv(
                "D:\ROV_BATCHELOR\Code\AGX-towed-rov-simulation\AGX-towed-rov-simulation\plots\plot_roll.csv")
            self.plotted = True

    @staticmethod
    def build_material(name, density) -> agx.Material:
        """
        build material with a set density and name
        Returns: the material

        """
        material = agx.Material(name)
        material.getBulkMaterial().setDensity(density)
        return material

    def post(self, t):
        """
        sets the camera possiton to follow the rov and plots data to csv.
        Args:
            t: in simulaton time

        """
        self.set_camera()
        self.plotter(t)

    def set_camera(self):
        """
        updates the camera to follow the rov.
        Returns:

        """
        r_p = self.link1.getPosition()
        cam_pos = agx.Vec3(r_p[0], r_p[1] + 100, r_p[2])
        demoutils.init_camera(eye=cam_pos, center=r_p)
