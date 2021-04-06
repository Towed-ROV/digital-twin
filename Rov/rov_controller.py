import demoutils
import time
import agxSDK

from Assembly import _map

"""Class RovController for Rov and by using StepEventListener it updates the wing position every step of the simulation"""
class RovController(agxSDK.GuiEventListener):
    def __init__(self, rov):
        super().__init__(agxSDK.GuiEventListener.KEYBOARD)
        # super().__init__()
        self.last_millis = 0
        self.time_interval = 0.04
        self.interval = 0.005
        self.interval_port = self.interval * 0.453 / 0.3825
        print(self.interval_port)
        self.interval_sb = self.interval
        self.rov = rov  # type:
        self.last_pos = 0
        self.current_pos = 1.1105
        self.current_pos_1 = 1.232
    """Runs every time before the simulation takes a step"""
    def pre(self, t):
        pid = demoutils.sim().getEventListener('pid')
        pid.compute(demoutils.sim().getAssembly('rov').getRigidBody('rovBody').getPosition()[2] * 1.23)
        pid_trim = demoutils.sim().getEventListener('pidTrim')
        pid_trim.compute(demoutils.sim().getAssembly('rov').getRigidBody('rovBody').getRotation()[1])
        output_port = _map(pid.output-pid_trim.output, -45, 45,  1.387, 1.178)
        output_sb = _map(pid.output+pid_trim.output, -45, 45, 1.03, 0.723)

        # print('--------------')
        # print(pid.output)
        # print('--------------')
        # print(pidTrim.output)
        # print('--------------')
        # print(outputPort)
        # print('--------------')
        # print(outputSb)
        # print('--------------')
        # self.set_output(output_port, output_sb)

    def set_output(self, output_port, output_sb):
        self.rov.output_port = output_port
        self.rov.output_sb = output_sb
        self.current_pos = self.rov.distance1.getAngle()
        self.current_pos_1 = self.rov.distance2.getAngle()
        # print(self.currentPos1)
        # print('--------------')
        # print(self.currentPos)
        # print('--------------')
        current_millis = int(round(time.time() * 1000))
        if (current_millis-self.last_millis >= self.time_interval):
            # print(self.currentPos)
            # print('-------------------')
            # print(self.rov.output)
            # print('-------------------')
            # print(self.rov.distance1.getAngle)
            # print(self.rov.distance2.getAngle)
            # print('-------------------')
            if self.rov.output_port > self.current_pos_1:
                # print("opp port")
                self.current_pos_1 = self.current_pos_1 + self.interval_port
                self.rov.distance2.getLock1D().setPosition(self.current_pos_1)
            elif self.rov.output_port < self.current_pos_1:
                # print("ned port")
                self.current_pos_1 = self.current_pos_1 - self.interval_sb
                self.rov.distance2.getLock1D().setPosition(self.current_pos_1)
            if self.rov.output_sb > self.current_pos:
                # print("opp sb")
                self.current_pos = self.current_pos + self.interval_port
                self.rov.distance1.getLock1D().setPosition(self.current_pos)
            elif self.rov.output_sb < self.current_pos:
                # print("ned sb")
                self.current_pos = self.current_pos - self.interval_sb
                self.rov.distance1.getLock1D().setPosition(self.current_pos)
            self.last_millis = current_millis

    # keyboards is used to tune the position of the wings
    # def keyboard(self, key, modKeyMask, x, y, keydown) -> bool:
    #     handled = False
    #     print("ok")
    #     if key == agxSDK.GuiEventListener.KEY_Right:
    #         print(self.current_pos)
    #         print('------')
    #         print("dsfs")
    #         print('------')
    #         self.current_pos = self.current_pos + 0.0005
    #         self.rov.distance1.getLock1D().setPosition(self.current_pos)
    #         self.rov.distance2.getLock1D().setPosition(self.current_pos)
    #         handled = True
    #
    #     elif key == agxSDK.GuiEventListener.KEY_Left:
    #         print('------')
    #         print("dsfs")
    #         print(self.current_pos)
    #         print('------')
    #         self.current_pos = self.current_pos - 0.0005
    #         self.rov.distance1.getLock1D().setPosition(self.current_pos)
    #         self.rov.distance2.getLock1D().setPosition(self.current_pos)
    #         handled = True
    #     return handled
    def keyboard(self, key, modKeyMask, x, y, keydown) -> bool:
        handled = False
        if key == agxSDK.GuiEventListener.KEY_Right:
            print(self.current_pos)
            if (self.current_pos + self.interval_sb) < 1.1105:
                test = _map(self.current_pos, 0.6575, 1.1105, 0.8495, 1.232)
                print('------')
                print(self.interval_sb)
                print(test)
                print('------')
                self.current_pos_1 = test + self.interval_port
                self.current_pos = self.current_pos + self.interval_sb
                self.rov.distance1.getLock1D().setPosition(self.current_pos)
                self.rov.distance2.getLock1D().setPosition(self.current_pos_1)
            handled = True

        elif key == agxSDK.GuiEventListener.KEY_Left:
            print(self.current_pos)
            if (self.current_pos + self.interval) > 0.6575:

                test = _map(self.current_pos, 0.6575, 1.1105, 0.8495, 1.232)
                print('------')
                print("dsfs")
                print(test)
                print('------')
                self.current_pos = self.current_pos - self.interval
                self.current_pos_1 = test - self.interval
                self.rov.distance1.getLock1D().setPosition(self.current_pos)
                self.rov.distance2.getLock1D().setPosition(self.current_pos_1)
            handled = True
        return handled
