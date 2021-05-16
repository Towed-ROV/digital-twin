import serial, time, agxSDK
from pid import PID_Controller
from functions import _map, constrain
import demoutils
from functions import deg2rad, rad2deg
from rov_simulation_parameters import WATER_LENGTH, MAX_WING_ANGLE, ROV_DEPTH_SETPOINT,ROV_K_P,ROV_K_I,ROV_K_D
from time import monotonic

# import rov_controller

class ArduinoStepper(agxSDK.StepEventListener):
    """

    simulates an Arduino located in the ROV. afurther development from the autum project
    """
    def __init__(self, pid: PID_Controller, pid_trim: PID_Controller, rov):
        # super().__init__(agxSDK.GuiEventListener.KEYBOARD)
        super().__init__()
        # PID controller
        self.pid_depth_p = ROV_K_P
        self.pid_depth_i = ROV_K_I
        self.pid_depth_d = ROV_K_D
        self.pid_roll_p = 0
        self.pid_roll_i = 0
        self.pid_roll_d = 0
        self.depth_rov_offset = 0
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

        self.max_wing_angle = MAX_WING_ANGLE
        self.max_stepper_pos_port = 1.232
        self.min_stepper_pos_port = 0.8495
        self.max_stepper_pos_sb = 1.1105
        self.min_stepper_pos_sb = 0.6575
        # 0.6575, 1.1105, 0.8495, 1.232
        self.last_millis_port = 0
        self.last_millis_sb = 0
        self.time_interval = 0.04
        self.interval2 = 0.04
        self.interval_port = self.interval * 0.453 / 0.3825
        print(self.interval_port)
        self.interval_sb = self.interval
        self.rov = rov  # type:
        self.last_pos = 0
        self.current_pos_sb = 1.1105
        self.current_pos_port = 1.232
        self.reset = False
        self.start = False
        self.set_point_depth = ROV_DEPTH_SETPOINT
        self.last_time=monotonic()
        self.last_t = 0
        self.last_post =0
    def pre(self, t):
        """
        recieves data from serial, if the data is a valid command it uses it to controll the simulated rov.
        Args:
            t: in simulation time.

        """

        if not self.reset:
            self.send("StepperArduino:0")
            msg = self.read()
            if msg[0] == "reset":
                self.reset = True

        elif round(t-self.last_t  > self.time_interval):
            self.depth = round(self.rov.link1.getPosition()[2])  # * 1.23, 2)
            if self.target_mode == self.manual_mode:
                pos = constrain(self.manual_wing_pos, -self.max_wing_angle, self.max_wing_angle)
                self.wing_pos_sb = pos
                self.wing_pos_port = pos
            elif self.target_mode == self.auto_depth_mode:
                wing_pos = -self.pid.compute(self.depth)
                trim_pos = self.pid_trim.compute(self.roll)
                if trim_pos != 0:
                    self.wing_pos_sb, self.wing_pos_port = self.trim_wing_pos(wing_pos, trim_pos)
                else:
                    self.wing_pos_port = wing_pos
                    self.wing_pos_sb = wing_pos
            self.compensate_wing_to_pitch()
            self.rov.update_wings(self.wing_pos_port, self.wing_pos_sb)
            self.current_pos_sb = self.rov.hinge1.getAngle()
            self.current_pos_port = self.rov.hinge2.getAngle()
            self.last_t = t
            current_millis = time.monotonic()
            if current_millis - self.previousMillis >= self.interval:
                self.update_wing_pos_gui(self.current_pos_port, self.current_pos_sb)
                self.previousMillis = current_millis
        self.handle_received_message()

    def post(self, time: "agx::TimeStamp const &") -> "void":
        """
        ads data to the simulation Window every in simmulaiton millisecond
        Args:
            time: in simulation time
        """
        if time-self.last_post >  0.001:
            self.last_post=time
            pos = self.rov.link1.getPosition()
            rot = self.rov.link1.getRotation()
            decorator = demoutils.app().getSceneDecorator()
            if round(time%2,2)== 0:
                print("seconds to simulate 2 sec: ",monotonic()-self.last_time)
                self.last_time=monotonic()
            decorator.setText(9,
                              "pid : {}, wing: {}".format(self.pid.output, round(rad2deg(self.rov.left_wing_angle()), 2)))
            decorator.setText(3, "Rov Position in Z direction : {} M".format(str(round(pos[2], 2))))
            decorator.setText(4, "Pitch : {}".format(str(round(rot[0] * 100, 2))))
            decorator.setText(5, "Roll : {}".format(str(round(rot[1] * 100, 2))))
            x, y = int(WATER_LENGTH + pos[0]), int(pos[1])
            decorator.setText(7, "distance : {}M".format(str(round(self.rov.link1.getPosition()[0], 2))))

    def reset_stepper(self):
        self.send("reset:True")

    def update_wing_pos_gui(self, port, sb):
        """
        updates the wing possition and sends it overserial
        Args:
            port: the portside wing pos
            sb:  the starbord wing poss


        """
        angle_port = _map(port, self.min_stepper_pos_port, self.max_stepper_pos_port,
                          -self.max_wing_angle, self.max_wing_angle)
        angle_sb = _map(sb, self.min_stepper_pos_sb, self.max_stepper_pos_sb,
                        -self.max_wing_angle, self.max_wing_angle)
        self.send("wing_pos_port:" + str(angle_port))
        self.send("wing_pos_sb:" + str(angle_sb))

    def compensate_wing_to_pitch(self):
        """
        changes the wing poss to compensate for the pitch of the rov so that the angle of attack better
        """
        self.wing_pos_sb = self.wing_pos_sb - self.pitch * 100
        self.wing_pos_port = self.wing_pos_port - self.pitch * 100

    def trim_wing_pos(self, wing_pos, trim_pos):
        """
        changes the wingpos to compensate for roll using the trim controller
        Args:
            wing_pos: wing angle
            trim_pos: roll compensating angle

        """
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

    def handle_received_message(self,):
        """
        checks the commands recived form a message.
        if the command has a registerd use the system executes relevant code.
        Args:
            msg: recived message

        """
        try:
            received_command = self.read()

            if len(received_command) > 1:
                print("inc: ", received_command)
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
                    self.send(received_command[0] + ":True")

                elif received_command[0] == "depth":
                    # self.depth = float(received_command[1])
                    self.send(received_command[0] + ":True")
                    # print(received_command)

                elif received_command[0] == "roll":
                    #self.roll = float(received_command[1])
                    self.send(received_command[0] + ":True")

                elif received_command[0] == "pitch":
                    #self.pitch = float(received_command[1])
                    self.send(received_command[0] + ":True")

                elif received_command[0] == "set_point_depth":
                    self.set_point_depth = float(received_command[1])
                    self.pid.set_setpoint(-self.set_point_depth)
                    self.send(received_command[0] + ":True")

                elif received_command[0] == "pid_depth_p":
                    if self.pid_depth_p >= 0:
                        self.pid_trim.set_tunings(self.pid_depth_p, self.pid_depth_i, self.pid_depth_d)
                        self.send(received_command[0] + ":True")
                    else:
                        self.send(received_command[0] + ":False")

                elif received_command[0] == "pid_depth_i":
                    if self.pid_depth_i >= 0:
                        self.pid_trim.set_tunings(self.pid_depth_p, self.pid_depth_i, self.pid_depth_d)
                        self.send(received_command[0] + ":True")
                    else:
                        self.send(received_command[0] + ":False")

                elif received_command[0] == "pid_depth_d":
                    if self.pid_depth_d >= 0:
                        self.pid_trim.set_tunings(self.pid_depth_p, self.pid_depth_i, self.pid_depth_d)
                        self.send(received_command[0] + ":True")
                    else:
                        self.send(received_command[0] + ":False")

                elif received_command[0] == "pid_roll_p":
                    if self.pid_roll_p >= 0:
                        self.pid_trim.set_tunings(self.pid_roll_p, self.pid_roll_i, self.pid_roll_d)
                        self.send(received_command[0] + ":True")
                    else:
                        self.send(received_command[0] + ":False")

                elif received_command[0] == "pid_roll_i":
                    if self.pid_roll_i >= 0:
                        self.pid_trim.set_tunings(self.pid_roll_p, self.pid_roll_i, self.pid_roll_d)
                        self.send(received_command[0] + ":True")
                    else:
                        self.send(received_command[0] + ":False")

                elif received_command[0] == "pid_roll_d":
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
        """
        sets the mode for the ROV controll, can be auto or manual
        Args:
            target_mode: the new controll mode
            wing_pos: the possiton fo the wings for manual mode.

        Returns:

        """
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
        """
        writes a message to the serial port.
        Args:
            message: message to be sendt

        """
        output = "<" + message + ">\n"

        self.ser.write(output.encode('utf-8'))

    def read(self):
        """
        reads a line from the serial port, strips it and parses the message.
        Returns:
            parsed message.

        """
        message = self.ser.readline()

        message = message.strip()
        message = message.decode('utf-8').strip("<").strip(">")
        # if message:
        # print("message: ",message)
        return message.split(":", 1)
