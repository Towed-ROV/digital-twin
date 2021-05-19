import demoutils
import agxSDK
import numpy as np
from modules.agxPythonModules.utils.callbacks import StepEventCallback as Sec
"""KeyboardListener used to control the set point of the controller as well as plotting values"""
class KeyboardListener(agxSDK.GuiEventListener):
    def __init__(self, pid, plot):
        super().__init__(agxSDK.GuiEventListener.KEYBOARD)
        # super().__init__()
        self.pid = pid
        self.plotted = False
        self.plot = plot
        self.plot_time = []
        self.plot_setpoint = []
        Sec.postCallback(lambda t: self.display_forces(t))
    def display_forces(self, t):

        self.plot_setpoint.append(self.pid.setpoint)
        self.plot_time.append(t)
        if self.plot and not self.plotted:
            """plots stored values to csv file"""
            plot_setpoint = np.array(self.plot_setpoint)
            plot_time = np.array(self.plot_time)
            # pd.DataFrame(plot_setpoint).to_csv(
            #     "C:/Users/Ishmael/PycharmProjects/pythonProject1/TowedRov-Sim/plotSetpoint.csv")
            # pd.DataFrame(plot_time).to_csv(
            #     "C:/Users/Ishmael/PycharmProjects/pythonProject1/TowedRov-Sim/plotTime.csv")
            self.plotted = True

    def keyboard(self, key, mod_key, x, y, key_down):
        # if key == agxSDK.GuiEventListener.KEY_Up:
        #     if key_down:
        #         self.ship.increase_propulsion(self.ship)
        #     return True
        # if key == agxSDK.GuiEventListener.KEY_Down:
        #     if key_down:
        #         self.ship.decrease_propulsion(self.ship)
        #     return True
        if key == agxSDK.GuiEventListener.KEY_F1:
            if key_down:
                self.plot = True
        if key == agxSDK.GuiEventListener.KEY_F5:
            if key_down:
                self.pid.set_setpoint(-4)
            return True
        if key == agxSDK.GuiEventListener.KEY_F2:
            if key_down:
                self.pid.set_setpoint(-8)
            return True
        if key == agxSDK.GuiEventListener.KEY_F3:
            if key_down:
                self.pid.set_setpoint(-12)
            return True
        if key == agxSDK.GuiEventListener.KEY_F4:
            if key_down:
                self.pid.set_setpoint(-13)
            return True
        if key == agxSDK.GuiEventListener.KEY_F5:
            if key_down:
                self.pid.set_setpoint(0)
            return True
        if key == agxSDK.GuiEventListener.KEY_F6:
            if key_down:
                self.pid.set_setpoint(+10)
            return True
        if key == agxSDK.GuiEventListener.KEY_F7:
            if key_down:
                self.pid.set_setpoint(-100)
            return True


        return False