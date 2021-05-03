import serial, time, json
import agxSDK
from agxCollide import HeightField
from rov_simulation_parameters import WATER_LENGTH


class ArduinoSensor(agxSDK.StepEventListener):
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
        # self.ser = serial.Serial(
        #    port='COM27',
        #    baudrate=19200,
        #    parity=serial.PARITY_NONE,
        #    stopbits=serial.STOPBITS_ONE,
        #    bytesize=serial.EIGHTBITS,
        #    timeout=0)
        self.interval = 0.01
        self.previousMillis = 0
        self.reset = True
        self.start = True
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
        if not self.reset:
            self.send("SensorArduino:0")
            if self.read()[0] == "reset":
                print("sensor arduino")
                self.reset = True
        elif not self.start:
            if self.read()[0] == "start":
                print("start sens")
                self.start = True
        else:
            currentMillis = time.monotonic()
            if currentMillis - self.previousMillis >= self.interval:
                self.send_sensor()
                self.previousMillis = currentMillis
            self.handle_received_message()

    def get_temp(self):
        return self.temp

    def get_depth_under_rov(self) -> float:
        pos = self.rov.link1.getPosition()
        x = int(pos[0] + WATER_LENGTH)
        y = int(pos[1])
        return self.seafloor.getHeight(x, y)

    def send_data(self, key,value):
        self.send(key,value)
        self.turn_to_send += 1
        if self.turn_to_send == self.command_number:
            self.turn_to_send = 1

    def send_sensor(self):
        key, data, numb = self.send_switch[self.turn_to_send]
        data = data()
        if not isinstance(numb, type(None)):
            data = data[numb]
        self.send_data(key,data)

    def handle_received_message(self):
        received_command = self.read()
        if received_command[0] == "depth_rov_offset":
            self.depth_beneath_rov_offset = received_command[1]
            self.send(received_command + ":True")
        elif received_command[0] == "depth_beneath_rov_offset":
            self.depth_rov_offset = received_command[1]
            self.send(received_command + ":True")

    def send(self, key,value):
        output =  "<{}:{}>".format(key,value)
        print(output)
        # self.ser.write(output.encode('utf-8'))

    def read(self):
        # message = self.ser.readline()
        message = b"cucked"
        message = message.strip()
        message = message.decode('utf-8').strip("<").strip(">")
        return message.split(":", 1)


if __name__ == '__main__':
    test = ArduinoSensor()
    test.run()
