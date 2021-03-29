import demoutils

import agx
import agxSDK
import agxCollide

import math

width = 0.1


def create_link(length: float) -> agx.RigidBody:
    geom = agxCollide.Geometry(agxCollide.Box(width, width, length))
    body = agx.RigidBody(geom)
    return body


class CraneController(agxSDK.GuiEventListener):

    def __init__(self, crane):
        super().__init__(agxSDK.GuiEventListener.KEYBOARD)

        self.interval = 0.01
        self.crane = crane  # type: CraneAssembly

    def keyboard(self, key, modKeyMask, x, y, keydown) -> bool:
        handled = False

        if key == agxSDK.GuiEventListener.KEY_Up:
            pos = self.crane.distance.getAngle()
            self.crane.distance.getLock1D().setPosition(pos + self.interval)
            handled = True
        elif key == agxSDK.GuiEventListener.KEY_Down:
            pos = self.crane.distance.getAngle()
            self.crane.distance.getLock1D().setPosition(pos - self.interval)
            handled = True

        return handled


class CraneAssembly(agxSDK.Assembly):

    def __init__(self):
        super().__init__()

        len1 = 2
        len2 = 1

        self.link1 = create_link(len1)
        self.link2 = create_link(len2)
        self.link2.setPosition(len2, 0, len1)
        self.link2.setRotation(agx.EulerAngles(0, math.radians(-90), 0))

        self.link1.getGeometries()[0].setEnableCollisions(self.link2.getGeometries()[0], False)

        demoutils.create_visual(self.link1)
        demoutils.create_visual(self.link2)

        self.hinge = demoutils.create_constraint(
            pos=agx.Vec3(width, 0, len1),
            axis=agx.Vec3(0, 1, 0),
            rb1=self.link1,
            rb2=self.link2,
            c=agx.Hinge)  # type: agx.Hinge
        self.hinge.setCompliance(1e-8)
        self.hinge.getLock1D().setEnable(False)
        self.hinge.getMotor1D().setEnable(False)
        self.hinge.getRange1D().setEnable(True)
        self.hinge.getRange1D().setRange(math.radians(-45), math.radians(75))

        f1 = agx.Frame()
        f1.setTranslate(agx.Vec3(width, 0, len1 - (len1 * 0.7)))
        f2 = agx.Frame()
        f2.setTranslate(agx.Vec3(-width, 0, 0))
        self.distance = agx.DistanceJoint(self.link1, f1, self.link2, f2)
        self.distance.getMotor1D().setEnable(False)
        self.distance.getLock1D().setEnable(True)

        self.add(self.link1)
        self.add(self.link2)
        self.add(self.hinge)
        self.add(self.distance)


