import serial, time, agxSDK
from pid import PID_Controller
from functions import _map, constrain
import demoutils
from functions import deg2rad, rad2deg
from rov_simulation_parameters import WATER_LENGTH
import rov_controller


class ArduinoStepper(agxSDK.StepEventListener):
    def __init__(self, pid: PID_Controller, pid_trim: PID_Controller, rov):
        # super().__init__(agxSDK.GuiEventListener.KEYBOARD)
        super().__init__()
        # PID controller
        self.pid = pid
        self.pid_trim = pid_trim
        self.manual_wing_pos = 0
        self.roll = 0
        self.pitch = 0
        self.depth = 0
        self.wing_pos_port = 0
        self.wing_pos_sb = 0
        self.manual_mode = 0
        self.auto_depth_mode = 1
        self.target_mode = 1
        self.ser = serial.Serial(
            port='COM10',
            baudrate=19200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=0)
        # timer
        self.interval = 0.01
        self.previousMillis = 0

        self.max_wing_angle = 35
        self.has_been_reset = True
        self.max_stepper_pos_port = 1.232
        self.min_stepper_pos_port = 0.8495
        self.max_stepper_pos_sb = 1.1105
        self.min_stepper_pos_sb = 0.6575
        # 0.6575, 1.1105, 0.8495, 1.232
        self.last_millis_port = 0
        self.last_millis_sb = 0
        self.time_interval = 0.04
        self.interval2 = 0.005
        self.interval_port = self.interval * 0.453 / 0.3825
        print(self.interval_port)
        self.interval_sb = self.interval
        self.rov = rov  # type:
        self.last_pos = 0
        self.current_pos_sb = 1.1105
        self.current_pos_port = 1.232
        self.reset = False
        self.start = False
        self.set_point_depth = -6

    def pre(self, t):
        if not self.reset:
            self.send("StepperArduino:0")
            msg = self.read()
            if msg[0] == "reset":
                self.reset = True

        # elif not self.start:
        #     if self.read()[0] == "start":
        #         print("start step")
        #         self.start = True
        else:
            self.depth = round(self.rov.link1.getPosition()[2])  # * 1.23, 2)
            if self.has_been_reset:
                self.pid.set_setpoint(self.set_point_depth)
                if self.target_mode == self.manual_mode:
                    pos = constrain(self.manual_wing_pos, -self.max_wing_angle, self.max_wing_angle)
                    # print("pos: ",pos)
                    self.wing_pos_sb = pos
                    self.wing_pos_port = pos
                elif self.target_mode == self.auto_depth_mode:
                    wing_pos = self.pid.compute(self.depth)
                    trim_pos = self.pid_trim.compute(self.roll)
                    if trim_pos != 0:
                        self.wing_pos_sb, self.wing_pos_port = self.trim_wing_pos(wing_pos, trim_pos)
                    else:
                        self.wing_pos_port = wing_pos
                        self.wing_pos_sb = wing_pos
                self.compensate_wing_to_pitch()
                self.rov.update_wings(self.wing_pos_port, self.wing_pos_sb)
                # step_position_sb = _map(self.wing_pos_sb, -self.max_wing_angle, self.max_wing_angle,
                #                        self.min_stepper_pos_sb, self.max_stepper_pos_sb)
                # step_position_port = _map(self.wing_pos_port, -self.max_wing_angle, self.max_wing_angle,
                #                        self.min_stepper_pos_port, self.max_stepper_pos_port)
                self.current_pos_sb = self.rov.hinge1.getAngle()
                self.current_pos_port = self.rov.hinge2.getAngle()
                # if step_position_sb != self.current_pos_sb:
                # self.move_stepper_pos_sb(step_position_sb)
                # if step_position_port != self.current_pos_port:
                # self.move_stepper_pos_port(step_position_port)
                # self.move_stepper_pos_port(step_position_port)

                current_millis = time.monotonic()
                if current_millis - self.previousMillis >= self.interval:
                    self.update_wing_pos_gui(self.current_pos_port, self.current_pos_sb)
                    self.previousMillis = current_millis
            self.handle_received_message()

    def post(self, time: "agx::TimeStamp const &") -> "void":

        pos = self.rov.link1.getPosition()
        rot = self.rov.link1.getRotation()
        decorator = demoutils.app().getSceneDecorator()
        decorator.setText(9, "pid : {}, wing: {}".format(str(self.pid.output), round(
            rad2deg(self.rov.left_wing_angle()), 2)))
        decorator.setText(3, "Rov Position in Z direction : {} M".format(str(round(pos[2], 2))))
        decorator.setText(4, "Pitch : {}".format(str(round(rot[0] * 100, 2))))
        decorator.setText(5, "Roll : {}".format(str(round(rot[1] * 100, 2))))
        x, y = int(WATER_LENGTH + pos[0]), int(pos[1])
        # print(int(pos[0]),int(pos[1]),int(pos[2]))
        decorator.setText(7, "distance : {}M".format(str(round(self.rov.link1.getPosition()[0], 2))))

    def reset_stepper(self):
        self.send("reset:True")

    def move_stepper_pos_port(self, step_pos):
        current_millis_port = time.monotonic()
        if current_millis_port - self.last_millis_port >= self.time_interval:
            if step_pos > self.current_pos_port + self.pitch:
                # print("opp port")
                self.current_pos_port = self.current_pos_port + self.interval_port
                self.rov.hinge2.getLock1D().setPosition(self.current_pos_port)
            elif step_pos < self.current_pos_port + self.pitch:
                # print("ned port")
                self.current_pos_port = self.current_pos_port - self.interval_sb
                self.rov.hinge2.getLock1D().setPosition(self.current_pos_port)
            self.last_millis_port = current_millis_port

    def move_stepper_pos_sb(self, step_pos):
        current_millis_sb = time.monotonic()
        if current_millis_sb - self.last_millis_sb >= self.time_interval:
            if step_pos > self.current_pos_sb + self.pitch:
                # print("opp sb")
                self.current_pos_sb = self.current_pos_sb + self.interval_port
                self.rov.hinge1.getLock1D().setPosition(self.current_pos_sb)
            elif step_pos < self.current_pos_sb + self.pitch:
                # print("ned sb")
                self.current_pos_sb = self.current_pos_sb - self.interval_sb
                self.rov.hinge1.getLock1D().setPosition(self.current_pos_sb)
            self.last_millis_sb = current_millis_sb

    def update_wing_pos_gui(self, port, sb):
        angle_port = _map(port, self.min_stepper_pos_port, self.max_stepper_pos_port,
                          -self.max_wing_angle, self.max_wing_angle)
        angle_sb = _map(sb, self.min_stepper_pos_sb, self.max_stepper_pos_sb,
                        -self.max_wing_angle, self.max_wing_angle)
        self.send("wing_pos_port:" + str(angle_port))
        self.send("wing_pos_sb:" + str(angle_sb))

    def compensate_wing_to_pitch(self):
        self.wing_pos_sb = self.wing_pos_sb - self.pitch*100
        self.wing_pos_port = self.wing_pos_port - self.pitch*100

    def trim_wing_pos(self, wing_pos, trim_pos):
        if wing_pos + trim_pos > self.max_wing_angle:
            compensate = float(-self.max_wing_angle + wing_pos)
            wing_pos_sb = self.max_wing_angle
            wing_pos_port = compensate
        elif wing_pos - trim_pos < -self.max_wing_angle:
            compensate = float(-self.max_wing_angle + trim_pos)
            wing_pos_sb = compensate
            wing_pos_port = -self.max_wing_angle
        else:
            wing_pos_sb = wing_pos - trim_pos
            wing_pos_port = wing_pos + trim_pos
        return constrain(wing_pos_sb, -self.max_wing_angle, self.max_wing_angle), \
               constrain(wing_pos_port, -self.max_wing_angle, self.max_wing_angle)

    def handle_received_message(self):
        try:
            received_command = self.read()
            # if len(received_command)> 1:
            # print(received_command)
            if received_command[0] == "auto_mode":
                if received_command[1] == "True":
                    self.set_target_mode(self.auto_depth_mode)
                    self.send(received_command + ":True")
                elif received_command[1] == "False":
                    self.set_target_mode(self.manual_mode)
                    self.manual_wing_pos = int(self.wing_pos_sb)
                    self.send(received_command + ":True")
                else:
                    self.send(received_command + ":False")

            elif received_command[0] == "manual_wing_pos":
                check_manual_wing_pos = float(received_command[1])
                if self.max_wing_angle > check_manual_wing_pos > -self.max_wing_angle:
                    self.manual_wing_pos = check_manual_wing_pos
                    self.send(received_command[0] + ":True")
                else:
                    self.send(received_command[0] + ":False")

            elif received_command[0] == "emergency_surface":
                self.set_target_mode(self.manual_mode, self.max_wing_angle)
                self.depth_rov_offset = received_command[1]
                self.send(received_command[0] + ":True")

            elif received_command[0] == "depth":
                self.depth = float(received_command[1])
                self.send(received_command[0] + ":True")
                # print(received_command)

            elif received_command[0] == "roll":
                self.roll = float(received_command[1])
                self.send(received_command[0] + ":True")

            elif received_command[0] == "pitch":
                self.pitch = float(received_command[1])
                self.send(received_command[0] + ":True")

            elif received_command[0] == "set_point_depth":
                self.set_point_depth = float(received_command[1])
                self.send(received_command[0] + ":True")

            elif received_command[0] == "pid_depth_p":
                self.pid_depth_p = float(received_command[1])
                if self.pid_depth_p >= 0:
                    self.pid_trim.set_tunings(self.pid_depth_p, self.pid_depth_i, self.pid_depth_d)
                    self.send(received_command[0] + ":True")
                else:
                    self.send(received_command[0] + ":False")

            elif received_command[0] == "pid_depth_i":
                self.pid_depth_i = float(received_command[1])
                if self.pid_depth_i >= 0:
                    self.pid_trim.set_tunings(self.pid_depth_p, self.pid_depth_i, self.pid_depth_d)
                    self.send(received_command[0] + ":True")
                else:
                    self.send(received_command[0] + ":False")

            elif received_command[0] == "pid_depth_d":
                self.pid_depth_d = float(received_command[1])
                if self.pid_depth_d >= 0:
                    self.pid_trim.set_tunings(self.pid_depth_p, self.pid_depth_i, self.pid_depth_d)
                    self.send(received_command[0] + ":True")
                else:
                    self.send(received_command[0] + ":False")

            elif received_command[0] == "pid_roll_p":
                self.pid_roll_p = float(received_command[1])
                if self.pid_roll_p >= 0:
                    self.pid_trim.set_tunings(self.pid_roll_p, self.pid_roll_i, self.pid_roll_d)
                    self.send(received_command[0] + ":True")
                else:
                    self.send(received_command[0] + ":False")

            elif received_command[0] == "pid_roll_i":
                self.pid_roll_i = float(received_command[1])
                if self.pid_roll_i >= 0:
                    self.pid_trim.set_tunings(self.pid_roll_p, self.pid_roll_i, self.pid_roll_d)
                    self.send(received_command[0] + ":True")
                else:
                    self.send(received_command[0] + ":False")

            elif received_command[0] == "pid_roll_d":
                self.pid_roll_d = float(received_command[1])
                if self.pid_roll_d >= 0:
                    self.pid_trim.set_tunings(self.pid_roll_p, self.pid_roll_i, self.pid_roll_d)
                    self.send(received_command[0] + ":True")
                else:
                    self.send(received_command[0] + ":False")

            elif received_command[0] == "reset":
                self.reset_stepper()
                self.send(received_command[0] + ":True")
        except ValueError:
            pass

    def set_target_mode(self, target_mode, wing_pos=0):
        mode_set = False
        if target_mode == self.manual_mode:
            self.pid.set_mode(0, 0, 0)
            self.pid_trim.set_mode(0, 0, 0)
            self.target_mode = self.manual_mode
            self.manual_wing_pos = wing_pos
            mode_set = True

        if target_mode == self.auto_depth_mode:
            self.set_point_depth = self.depth
            self.pid.set_mode(1, 0, 0)
            self.pid_trim.set_mode(1, 0, 0)
            self.target_mode = self.auto_depth_mode
            self.pid.set_tunings(self.pid_depth_p, self.pid_depth_i, self.pid_depth_d)
            self.pid_trim.set_tunings(self.pid_roll_p, self.pid_roll_i, self.pid_roll_d)

    def send(self, message):
        output = "<" + message + ">\n"

        self.ser.write(output.encode('utf-8'))

    def read(self):
        message = self.ser.readline()
        message = message.strip()
        message = message.decode('utf-8').strip("<").strip(">")
        # if message:
        # print("message: ",message)
        return message.split(":", 1)
