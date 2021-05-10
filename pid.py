"""
Code is rewritten by Towed-Rov group

PID controller with integral-windup & derivative-kick prevention and bumpless
manual-to-auto-mode transfer
https://pypi.org/project/dvg-pid-controller/

Original C++ code by::

 /******************************************************************************
 * Arduino PID Library - Version 1.2.1
 * by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 *
 * This Library is licensed under the MIT License
 ******************************************************************************/
"""

import time
import numpy as np
import agxSDK


class Constants:
    MANUAL = 0
    AUTOMATIC = 1

    DIRECT = 0
    REVERSE = 1


class PID_Controller(agxSDK.StepEventListener):
    def __init__(self, Kp, Ki, Kd, controller_direction=Constants.DIRECT):
        super().__init__()
        self.setpoint = 0
        self.output = 0
        # Must be set by set_tunings()
        self.kp = np.nan
        self.ki = np.nan
        self.kd = np.nan

        # Must be set by set_output_limits()
        self.output_limit_min = np.nan
        self.output_limit_max = np.nan

        self.in_auto = False

        self.p_term = 0
        self.i_term = 0
        self.d_term = 0

        self.set_tunings(Kp, Ki, Kd, controller_direction)
        self.set_output_limits(0, 100)
        self.last_time = time.perf_counter()
        self.last_input = np.nan
    # def update(self, agxSDK.StepEventListener):

    def output(self):
        return self.output

    def compute(self, current_input):
        """Compute new PID output. This function should be called repeatedly,
        preferably at a fixed time interval.
        Returns True when the output is computed, false when nothing has been
        done.
        """
        now = time.perf_counter()  # [s]
        time_step = now - self.last_time

        if (not self.in_auto) or np.isnan(self.setpoint):
            self.last_time = now
            return False

        _input = current_input
        error = self.setpoint - _input
        # print(self.output)
        # Proportional term
        self.p_term = self.kp * error

        # Integral term
        # self.iTerm = self.iTerm + (self.ki * error)
        self.i_term = self.i_term + (self.ki * time_step * error)


        # Prevent integral windup
        self.i_term = np.clip(self.i_term, self.output_limit_min, self.output_limit_max)

        # Derivative term
        # Prevent derivative kick: really good to do!
        # self.dTerm = -self.kd * (_input - self.last_input)
        self.d_term = -self.kd / time_step * (_input - self.last_input)

        # Compute PID Output
        self.output = self.p_term + self.i_term + self.d_term
        # Clamp the output to its limits
        # print((self.output), ": ",self.kp)
        self.output = np.clip(self.output, self.output_limit_min, self.output_limit_max)
        # Remember some variables for next time
        self.last_input = _input
        self.last_time = now


        return (self.output)

    def set_setpoint(self, setpoint):
        self.setpoint = setpoint


    def set_tunings(self, Kp, Ki, Kd, direction=Constants.DIRECT):
        """This function allows the controller's dynamic performance to be
        adjusted. It's called automatically from the constructor, but tunings
        can also be adjusted on the fly during normal operation.

        The PID will either be connected to a DIRECT acting process
        (+Output leads to +Input) or a REVERSE acting process (+Output leads to
        -Input). We need to know which one, because otherwise we may increase
        the output when we should be decreasing. This is called from the
        constructor.
        """

        if (Kp < 0) or (Ki < 0) or (Kd < 0):
            return

        self.controller_direction = direction

        if self.controller_direction == Constants.REVERSE:
            self.kp = -Kp
            self.ki = -Ki
            self.kd = -Kd
        else:
            self.kp = Kp
            self.ki = Ki
            self.kd = Kd

    def set_output_limits(self, limit_min, limit_max):
        if limit_min >= limit_max:
            raise ValueError("min is larger than max")

        self.output_limit_min = limit_min
        self.output_limit_max = limit_max

        if self.in_auto:
            self.output = np.clip(self.output, limit_min, limit_max)
            self.i_term = np.clip(self.i_term, limit_min, limit_max)

    def set_mode(self, mode, current_input, current_output):
        """Allows the controller Mode to be set to manual (0) or Automatic
        (non-zero). When the transition from manual to auto occurs, the
        controller is automatically initialized.
        """

        new_auto = mode == Constants.AUTOMATIC

        if new_auto and not self.in_auto:
            # We just went from manual to auto
            self.initialize(current_input, current_output)

        self.in_auto = new_auto

    def initialize(self, current_input, current_output):
        """Does all the things that need to happen to ensure a bumpless
        transfer from manual to automatic mode.
        """
        self.i_term = current_output
        self.last_input = current_input


        self.i_term = np.clip(self.i_term, self.output_limit_min, self.output_limit_max)
