
import demoutils

import agx
import agxSDK
import agxCollide

from agxRender import Color
import math


def load_servo_shape():
    return demoutils.load_shape('../assets/76mm/servo.obj')


def load_bottom_shape():
    return demoutils.load_shape('../assets/76mm/bottom.obj')


def load_upper_shape():
    return demoutils.load_shape('../assets/76mm/upper.obj')


class ServoAssembly(agxSDK.Assembly):

    def __init__(self):
        super().__init__()

        servo = agxCollide.Geometry(load_servo_shape())
        servo.setEnableCollisions(False)
        demoutils.create_visual(servo, diffuse_color=Color.Black())

        self.bottom = agx.RigidBody(agxCollide.Geometry(load_bottom_shape()))
        self.bottom.add(servo)
        demoutils.create_visual(self.bottom, Color.Orange())

        self.upper = agx.RigidBody(agxCollide.Geometry(load_upper_shape()))
        demoutils.create_visual(self.upper, Color.Orange())

        self.hinge = demoutils.create_constraint(
            pos=agx.Vec3(0.0, 0.007, 0.0), axis=agx.agx.Vec3(0, 0, -1),
            rb1=self.bottom, rb2=self.upper, c=agx.Hinge)  # type: agx.Hinge

        self.hinge.setCompliance(1E-8)
        self.hinge.getMotor1D().setEnable(True)
        self.hinge.getLock1D().setEnable(False)
        self.hinge.getRange1D().setEnable(True)
        self.hinge.getRange1D().setCompliance(1e-12)
        self.hinge.getRange1D().setRange(-math.pi / 2, math.pi / 2)

        self.add(self.bottom)
        self.add(self.hinge)
        self.add(self.upper)
