import serial
from agxSDK import StepEventListener, Assembly
from agxCollide import HeightField
from rov_simulation_parameters import WATER_LENGTH
import zmq
import _operator
from functools import reduce
import demoutils


class Boat_Sensor(StepEventListener):
    def __init__(self, boat: Assembly, seafloor: HeightField):
        super().__init__()
        self.depth_rov_offset = 0
        self.depth_beneath_rov_offset = 0
        self.interval = 0.01
        self.ctx = zmq.Context()
        self.ser = serial.Serial("COM1", 4800)
        self.serGPS = serial.Serial("COM30", 9600)
        self.connection = self.ctx.socket(zmq.PUB)
        self.host = "127.0.0.1"
        self.port = "6969"
        self.connection.bind("tcp://" + self.host + ":" + self.port)
        self.boat = boat
        self.seafloor = seafloor
        self.last_x = boat.getPosition()[0][0]
        self.travel_distance = 4
        self.message_builder = lambda type, data: {"payload_name": type, "payload_data": data}
        self.pay_loader = lambda name, value: {"name": name, "value": value}
        self.last_time = 0
        self.freq = 1 / 2
        self.lat_list = [60.0000000001, 60.010000000001, 60.020000000001, 60.030000000001, 60.040000000001,
                         60.050000000001, 60.060000000001, 60.070000000001, 60.080000000001, 60.090000000001,
                         610000000001]
        self.lon_list = [l * 1 / 10 for l in self.lat_list]

        self.GPS_i = 0

    def pre(self, time: "agx::TimeStamp const &"):

        if time - self.last_time > self.freq:
            delta_x = self.get_travel_distance()
            if delta_x >= self.travel_distance:
                sent = "GPGGA, , {}, N, {}, W, 2, 0 9, 0, 0, M,,,,".format(self.lat_list[self.GPS_i],
                                                                           self.lon_list[self.GPS_i])
                chk = hex(reduce(_operator.xor, map(ord, sent), 0))[2:].upper()
                sent = "${}*{} \n".format(sent, chk)
                self.send_GPS_ser(sent.encode())
                self.GPS_i = self.GPS_i + 1 if self.GPS_i < len(self.lon_list) - 1 else 0

            sent = "SDDBT,{},f,{},M,".format(-self.get_depth_under_boat(), -self.get_depth_under_boat())
            chk = hex(reduce(_operator.xor, map(ord, sent), 0))[2:].upper()
            sent = "${}*{} \n".format(sent, chk)
            self.send_ser(sent.encode())
            self.last_time = time

    def connect(self):
        self.connection.connect(f"tcp://{self.host}:{self.port}")

    def get_travel_distance(self):
        pos, n = self.boat.getPosition()
        x = pos[0]
        delta_x = x - self.last_x
        if delta_x > self.travel_distance:
            self.last_x = x
        return delta_x

    def get_depth_under_boat(self) -> float:
        pos, n = self.boat.getPosition()
        x = int(pos[0] + WATER_LENGTH * (n) - 1)
        y = int(pos[1])
        return self.seafloor.getHeight(x, y)

    def handle_received_message(self):
        received_command = self.read()
        if received_command[0] == "depth_rov_offset":
            self.send_zmq(received_command + ":True")
        elif received_command[0] == "depth_beneath_rov_offset":
            self.send_zmq(received_command + ":True")

    def send_zmq(self, data):
        self.connection.send_json(data)

    def recv(self):
        data = self.connection.recv_json()
        return data

    def send_ser(self, data):
        # print(data)
        self.ser.write(data)

    def send_GPS_ser(self, data):
        self.ser.write(data)

    def post(self,t):
        demoutils.app().getSceneDecorator().setText(8, "Depth under boat : {} m".format(
            str(round(self.get_depth_under_boat() , 2))))
