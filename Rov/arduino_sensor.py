import serial, time
import agxSDK
from rov_simulation_parameters import WATER_LENGTH


class ArduinoSensor(agxSDK.StepEventListener):
    def __init__(self, rov, seafloor):
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
        self.seafloor = seafloor
        self.turn_to_send = 1
        self.ser = serial.Serial(
            port='COM20',
            baudrate=19200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=0)
        self.interval = 0.02
        self.previousMillis = 0
        self.reset = False
        self.start = False
        self.count = 0

    def pre(self, t):
        if not self.reset:
            self.send("SensorArduino:0")
            msg = self.read()
            if len(msg) > 1:
                print("sensor", msg)
            if msg[0] == "reset":
                print("sensor arduino")
                self.reset = True
        else:
            currentMillis = time.monotonic()
            if currentMillis - self.previousMillis >= self.interval:
                self.send_sensor()
                self.previousMillis = currentMillis
            self.handle_received_message()

    def send_sensor(self):
        rov_body = self.rov
        if self.turn_to_send == 1:
            self.depth = round(rov_body.link1.getPosition()[2] + self.depth_rov_offset, 3)
            self.send("depth:" + str(-self.depth))
            # print("depth:" + str(-self.depth))
            self.turn_to_send = 2
        elif self.turn_to_send == 2:
            pos = self.rov.link1.getPosition()
            x = int(pos[0] + WATER_LENGTH - 1)
            y = int(pos[1])
            self.depth_beneath_rov = self.depth-round(self.seafloor.getHeight(x, y),3)
            self.send("depth_beneath_rov:" + str(self.depth_beneath_rov + self.depth_beneath_rov_offset))
            self.turn_to_send = 3
        elif self.turn_to_send == 3:
            self.temp = 10
            self.send("temperature:" + str(self.temp))
            self.turn_to_send = 4
        elif self.turn_to_send == 4:
            self.roll = round(rov_body.link1.getRotation()[1]*100, 3)
            self.send("roll:" + str(self.roll))
            self.turn_to_send = 5
        elif self.turn_to_send == 5:
            self.pitch = round(rov_body.link1.getRotation()[0]*100, 3)
            self.send("pitch:" + str(self.pitch))
            self.turn_to_send = 6
        elif self.turn_to_send == 6:
            self.yaw = round(rov_body.link1.getRotation()[3]*100, 3)
            self.send("yaw:" + str(self.yaw))
            self.turn_to_send = 1

    def handle_received_message(self):
        received_command = self.read()
        if received_command[0] == "depth_beneath_rov_offset":
            self.depth_beneath_rov_offset = float(received_command[1])
            self.send(received_command[0] + ":True")
        elif received_command[0] == "depth_rov_offset":
            self.depth_rov_offset = float(received_command[1])
            self.send(received_command[0] + ":True")

    def send(self, message):
        output = "<" + message + ">\n"
        #print(output)
        self.ser.write(output.encode('utf-8'))

    def read(self):
        message = self.ser.readline()
        message = message.strip()
        message = message.decode('utf-8').strip("<").strip(">")
        return message.split(":", 1)


if __name__ == '__main__':
    test = ArduinoSensor()
    test.run()
