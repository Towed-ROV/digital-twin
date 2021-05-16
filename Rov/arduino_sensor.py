import serial, time, json
import agxSDK
from agxCollide import HeightField
from rov_simulation_parameters import WATER_LENGTH


class ArduinoSensor(agxSDK.StepEventListener):
    """
    simulates an Arduino located in the ROV.
    """

    def __init__(self, rov: agxSDK.Assembly, seafloor: HeightField):
        super().__init__()
        self.rov = rov
        self.depth_beneath_rov = 0
        self.depth_beneath_rov_offset = 0
        self.depth = 0
        self.depth_rov_offset = 0
        self.temp = 0
        self.pressure = 0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.turn_to_send = 1
        self.ser = serial.Serial(
            port='COM20',
            baudrate=19200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=0)
        self.interval = 0.01
        self.previousMillis = 0
        self.reset = False
        self.start = False
        self.command_number = 6
        self.seafloor = seafloor

        self.send_switch = {1: ("depth", self.rov.link1.getPosition, 2),
                            2: ("depth_beneath_rov", self.get_depth_under_rov, None),
                            3: ("temperature:", self.get_temp, None),
                            4: ("roll", self.rov.link1.getRotation, 1),
                            5: ("pitch", self.rov.link1.getRotation, 0),
                            6: ("yaw", self.rov.link1.getRotation, 3)}

        self.sender = lambda x: self.send_switch[x]

    def pre(self, t):
        """
        at each step if the in simulation time is a modulus of the intervall time, the simualtion sends sensor data from
        the simulation over serial.
        Args:
            t: in simulation time

        """
        if not self.reset:
            self.send("SensorArduino", "0")
            msg = self.read()
            if msg[0] == "reset":
                self.reset = True

        elif t-self.previousMillis>self.interval:
            self.send_sensor()
            self.handle_received_message()
            self.previousMillis = t

    def get_temp(self):
        return self.temp

    def get_depth_under_rov(self) -> float:
        """
        gets the depth of the water under the simulated rov.

        Returns: the depth under the rov
        """
        pos = self.rov.link1.getPosition()
        x = int(pos[0] + WATER_LENGTH)
        y = int(pos[1])
        z = pos[2]
        return z - self.seafloor.getHeight(x, y)

    def send_data(self, key, value):
        """
        send data over serial from the simulated arduino, each time it's caled it steps to the next value it should send.
        so that it loops throug all the data to send.

        Args:
            key: key of the data.
            value: value of the data.

        """
        self.send(key, value)
        self.turn_to_send += 1
        #print(key,value)
        if self.turn_to_send == self.command_number:
            self.turn_to_send = 1

    def send_sensor(self):
        """
        gets sensor data from the send switch and sends this data if its not None.
        """
        key, data, numb = self.send_switch[self.turn_to_send]
        data = data()
        if not isinstance(numb, type(None)):
            data = data[numb]
        self.send_data(key, data)

    def handle_received_message(self):
        """
        recives  data over serial
        Returns:

        """
        received_command = self.read()
        if len(received_command) > 1:
            # print(received_command)
            if received_command[0] == "depth_rov_offset":
                self.depth_beneath_rov_offset = received_command[1]
                self.send(received_command, ":True")
            elif received_command[0] == "depth_beneath_rov_offset":
                self.depth_rov_offset = received_command[1]
                self.send(received_command, ":True")

    def send(self, key, value):
        """
        writes data to the serial port
        Args:
            key: key of data
            value: value of data

        """
        output = "<{}:{}>\n".format(key, value)
        print(output)
        self.ser.write(output.encode('utf-8'))

    def read(self):
        """
        reads data from serialport
        Returns: data it read.

        """
        message = self.ser.readline()

        message = message.strip()
        message = message.decode('utf-8').strip("<").strip(">")
        return message.split(":", 1)
