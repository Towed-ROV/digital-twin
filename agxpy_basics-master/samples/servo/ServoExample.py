
import demoutils
import math

import agx
import agxCollide
from agxRender import Color

from samples.servo.ServoAssembly import ServoAssembly
from samples.servo.SimpleServoController import SimpleServoPositionController, SimpleServoSpeedController


def create_and_add_ground():
    ground = agxCollide.Geometry(agxCollide.Box(1, 1, 0.05))
    demoutils.create_visual(ground, Color.Green())
    demoutils.sim().add(ground)
    return ground


def create_and_add_servo():
    servo = ServoAssembly()
    demoutils.sim().add(servo)
    return servo


def build_scene():
    demoutils.register_additional_scenes("build_scene2", "build_scene3")

    create_and_add_ground()
    servo = create_and_add_servo()
    servo.setPosition(0.0, 0.0, 0.1)


def build_scene2():
    create_and_add_ground()
    servo = create_and_add_servo()
    servo.setPosition(0, 0, 0.085)
    servo.setRotation(agx.EulerAngles(0, -math.pi / 2, 0))

    controller = SimpleServoSpeedController(servo)
    demoutils.sim().addEventListener(controller)


def build_scene3():
    create_and_add_ground()
    servo = create_and_add_servo()
    servo.setPosition(0, 0, 0.085)
    servo.setRotation(agx.EulerAngles(0, -math.pi / 2, 0))

    controller = SimpleServoPositionController(servo)
    demoutils.sim().addEventListener(controller)
