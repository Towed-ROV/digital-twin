
from samples.servo.ServoAssembly import *


class SimpleServoSpeedController(agxSDK.GuiEventListener):

    def __init__(self, servo):
        super().__init__(agxSDK.GuiEventListener.KEYBOARD)

        self.speed = 0.5
        self.servo = servo  # type: ServoAssembly

    def keyboard(self, key, alt, x, y, down):
        handled = False
        if key == agxSDK.GuiEventListener.KEY_Down:
            self.servo.hinge.getMotor1D().setSpeed(-self.speed)
            handled = True
        elif key == agxSDK.GuiEventListener.KEY_Up:
            self.servo.hinge.getMotor1D().setSpeed(self.speed)
            handled = True
        return handled


class SimpleServoPositionController(agxSDK.GuiEventListener):

    def __init__(self, servo):
        super().__init__(agxSDK.GuiEventListener.KEYBOARD)

        self.interval = 0.01
        self.servo = servo  # type: ServoAssembly
        self.servo.hinge.getLock1D().setEnable(True)
        self.servo.hinge.getMotor1D().setEnable(False)

    def keyboard(self, key, alt, x, y, down):
        handled = False
        if key == agxSDK.GuiEventListener.KEY_Down:
            pos = self.servo.hinge.getAngle() + self.interval
            self.servo.hinge.getLock1D().setPosition(pos)
            handled = True
        elif key == agxSDK.GuiEventListener.KEY_Up:
            pos = self.servo.hinge.getAngle() - self.interval
            self.servo.hinge.getLock1D().setPosition(pos)
            handled = True
        return handled
