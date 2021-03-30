import serial, time
class ArduinoSensor:
    def __init__(self):
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
            port='COM27',
            baudrate=19200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=0)
        self.interval = 0.01

    def run(self):
        previousMillis = 0
        while True:
            print("treigt?")
            currentMillis = time.monotonic()
            if currentMillis - previousMillis >= self.interval:

                self.send_sensor()
                previousMillis = currentMillis
            self.handle_received_message()

    def send_sensor(self):
        # rov_body = demoutils.sim().getAssembly('rov').getRigidBody('rovBody')
        if self.turn_to_send == 1 :
            # self.depth = round(rov_body.link1.getPosition()[2] * 1.23, 2)
            self.send("depth:" + str(self.depth))
            self.turn_to_send = 2
        elif self.turn_to_send == 2:
            self.depth_beneath_rov = 10 ##get distance to botttom remove detph
            self.send("depth_beneath_rov:" + str(self.depth_beneath_rov))
            self.turn_to_send  = 3
        elif self.turn_to_send == 3:
            self.temp = 10
            self.send("temperature:" + str(self.temp))
            self.turn_to_send = 4
        elif self.turn_to_send == 4:
            # self.roll = round(rov_body.link1.getRotation()[1], 2)
            self.send("roll:" + str(self.roll))
            self.turn_to_send =5
        elif self.turn_to_send == 5:
            # self.pitch  = round(rov_body.link1.getRotation()[0], 2)
            self.send("pitch:" + str(self.pitch))
            self.turn_to_send = 6
        elif self.turn_to_send == 6:
            # self.yaw = round(rov_body.link1.getRotation()[3], 2)
            self.send("yaw:" + str(self.yaw))
            self.turn_to_send = 1
    def handle_received_message(self):
        received_command = self.read()
        if received_command[0] == "depth_rov_offset":
            self.depth_beneath_rov_offset = received_command[1]
            self.send(received_command + ":True")
        elif received_command[0] == "depth_beneath_rov_offset":
            self.depth_rov_offset = received_command[1]
            self.send(received_command + ":True")
    def send(self, message):
        output = "<" + message + ">\n"
        self.ser.write(output.encode('utf-8'))

    def read(self):
        message = self.ser.readline()
        message = message.strip()
        message = message.decode('utf-8').strip("<").strip(">")
        return message.split(":",1)


if __name__ == '__main__':
    test = ArduinoSensor()
    test.run()