import serial, time
from pid import PID_Controller
from functions import _map, constrain


class ArduinoStepper:
    def __init__(self, pid_depth_p, pid_depth_i, pid_depth_d, pid_trim_p, pid_trim_i, pid_trim_d):
        # PID controller
        self.pid = PID_Controller
        self.pid_trim = PID_Controller
        self.pid_depth_p = pid_depth_p
        self.pid_depth_i = pid_depth_i
        self.pid_depth_d = pid_depth_d
        self.pid_roll_p = pid_trim_p
        self.pid_roll_i = pid_trim_i
        self.pid_roll_d = pid_trim_d

        self.manual_wing_pos = 0
        self.roll = 0
        self.pitch = 0
        self.depth = 0
        self.wing_pos_port = 0
        self.wing_pos_sb = 0
        self.manual_mode = 0
        self.auto_depth_mode = 1
        self.target_mode = self.manual_mode
        self.ser = serial.Serial(
            port='COM31',
            baudrate=19200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=0)
        self.interval = 0.01
        self.max_wing_angle = 35
        self.has_been_reset = False
        self.wing_pos = 0
        self.trim_pos = 0
        self.max_stepper_pos = 800
        self.min_stepper_pos = -800
    def run(self):
        previousMillis = 0
        while True:
            if self.has_been_reset:
                if self.target_mode == self.manual_mode:
                    pos = constrain(self.manual_wing_pos, -self.max_wing_angle, self.max_wing_angle)
                    self.wing_pos_sb = pos
                    self.wing_pos_port = pos
                elif self.target_mode == self.auto_depth_mode:
                    wing_pos = self.pid.compute(self.depth)
                    trim_pos = self.pid_trim.compute(self.roll)
                    if self.pid_trim.output() != 0:
                        self.wing_pos_sb, self.wing_pos_port = self.trim_wing_pos(wing_pos, trim_pos)
                    else:
                        self.wing_pos_port = wing_pos
                        self.wing_pos_sb = wing_pos
                self.compensate_wing_to_pitch()
                step_position_sb = _map(self.wing_pos_sb, -self.max_wing_angle, self.max_wing_angle,
                                        self.min_stepper_pos, self.max_stepper_pos)
                step_position_port = _map(self.wing_pos_port, -self.max_wing_angle, self.max_wing_angle,
                                        self.min_stepper_pos, self.max_stepper_pos)
                if step_position_sb != self.current_pos_sb:
                    self.move_stepper_sb(step_position_sb)
                if step_position_port != self.current_pos_port:
                    self.move_stepper_port(step_position_port)

                currentMillis = time.monotonic()
                if currentMillis - previousMillis >= self.interval:
                    self.update_wing_pos_gui(self.current_pos_port, self.current_pos_sb)
                    previousMillis = currentMillis
            self.handle_received_message()

    def reset_stepper(self):
        pass
    def move_stepper_pos_port(self):
        pass
    def move_stepper_pos_sb(self):
        pass

    def update_wing_pos_gui(self, port, sb):
        angle_port = _map(port, self.min_stepper_pos, self.max_stepper_pos, -self.max_wing_angle, self.max_wing_angle)
        angle_sb = _map(sb, self.min_stepper_pos, self.max_stepper_pos, -self.max_wing_angle, self.max_wing_angle)
        self.send("wing_pos_port:" + angle_port)
        self.send("wing_pos_sb:" + angle_sb)

    def compensate_wing_to_pitch(self):
        self.wing_pos_sb  = self.wing_pos_sb - self.pitch
        self.wing_pos_port  = self.wing_pos_port - self.pitch

    def trim_wing_pos(self, wing_pos, trim_pos):
        if wing_pos + trim_pos > self.max_wing_angle:
            compensate = float(-self.max_wing_angle + 2*wing_pos)
            wing_pos_sb = self.max_wing_angle
            wing_pos_port = compensate
        elif wing_pos - trim_pos < -self.max_wing_angle:
            compensate = float(-self.max_wing_angle + 2*trim_pos)
            wing_pos_sb = compensate
            wing_pos_port = -self.max_wing_angle
        else:
            wing_pos_sb = wing_pos - trim_pos
            wing_pos_port = wing_pos + trim_pos
        return  constrain(wing_pos_sb, -self.max_wing_angle, self.max_wing_angle),\
                constrain(wing_pos_port, -self.max_wing_angle, self.max_wing_angle)

    def handle_received_message(self):
        received_command = self.read()

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
            self.reset_steppers()
            self.send(received_command + ":True")

    def set_target_mode(self, target_mode, wing_pos = 0):
        mode_set = False
        if target_mode == self.manual_mode:
            self.pid.set_mode(0,0,0)
            self.pid_trim.set_mode(0,0,0)
            self.target_mode = self.manual_mode
            self.manual_wing_pos = wing_pos
            mode_set = True

        if target_mode == self.auto_depth_mode:
            self.set_point_depth = self.depth
            self.pid.set_mode(1, 0, 0)
            self.pid_trim.set_mode(1, 0, 0)
            self.target_mode = self.auto_depth_mode
            self.pid.set_tunings(self.pid_depth_p,self.pid_depth_i,self.pid_depth_d)
            self.pid_trim.set_tunings(self.pid_roll_p, self.pid_roll_i, self.pid_roll_d)

    def send(self, message):
        output = "<" + message + ">\n"
        self.ser.write(output.encode('utf-8'))

    def read(self):
        message = self.ser.readline()
        message = message.strip()
        message = message.decode('utf-8').strip("<").strip(">")
        return message.split(":",1)
