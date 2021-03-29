import agx
import agxCollide
import agxSDK

import math
import demoutils


# Create two boxes at the given positions, with the specified size
def create_bodies(position1, position2, size):
    b1 = agx.RigidBody()
    b2 = agx.RigidBody()

    b1.add(agxCollide.Geometry(agxCollide.Box(size[0], size[1], size[2])))
    b2.add(agxCollide.Geometry(agxCollide.Box(size[0], size[1], size[2])))

    b1.setPosition(agx.Vec3(position1[0], position1[1], position1[2]))
    b2.setPosition(agx.Vec3(position2[0], position2[1], position2[2]))

    return b1, b2


# Create a StepEventListener that will alternate the direction of a motor with a specified interval (seconds)
class AlternatingSpeedController(agxSDK.StepEventListener):
    def __init__(self, motor, speed, interval):
        super().__init__(agxSDK.StepEventListener.PRE_STEP)

        # Enable the motor and set the initial speed
        motor.setEnable(True)
        motor.setSpeed(speed)

        # Assign some variables that the listener needs
        self.interval = interval
        self.speed = speed
        self.last = 0
        self.motor = motor

    # This method will be called every timestep
    def pre(self, time):
        # Time to change direction
        if time - self.last >= self.interval:
            self.last = time
            self.speed = -self.speed
            self.motor.setSpeed(self.speed)


# Create a hinge scene
def create_hinge_scene():
    size = [0.5, 0.25, 1]
    b1, b2 = create_bodies([0, 0, 0], [0, 0, - 2.5], size)

    demoutils.sim().add(b1)
    demoutils.sim().add(b2)

    f1 = agx.Frame()
    f1.setLocalTranslate(0, 0, size[2])
    f1.setLocalRotate(agx.EulerAngles(0, math.radians(90), 0))

    hinge1 = agx.Hinge(b1, f1)
    demoutils.sim().add(hinge1)

    # Make the first motor swing back and forth
    speed_controller = AlternatingSpeedController(hinge1.getMotor1D(), 1, 2)
    demoutils.sim().add(speed_controller)

    distance = (b2.getPosition() - b1.getPosition()).length()
    f1 = agx.Frame()
    f1.setLocalTranslate(0, 0, -distance / 2)
    f1.setLocalRotate(agx.EulerAngles(0, math.radians(90), 0))

    f2 = agx.Frame()
    f2.setLocalTranslate(0, 0, distance / 2)
    f2.setLocalRotate(agx.EulerAngles(0, math.radians(90), 0))

    hinge2 = agx.Hinge(b1, f1, b2, f2)
    demoutils.sim().add(hinge2)


########################################
# Our function which creates the scene
########################################
def build_scene():
    floor = agxCollide.Geometry(agxCollide.Box(10, 10, 0.1))
    floor.setPosition(0, 0, -4)
    demoutils.sim().add(floor)

    # Create each and every one of the scenes
    create_hinge_scene()

    demoutils.app().getSceneDecorator().setEnableShadows(False)
    demoutils.app().setEnableDebugRenderer(True)
