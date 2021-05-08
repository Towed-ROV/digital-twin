import serial
from agxSDK import StepEventListener, Assembly
from agxCollide import HeightField
from rov_simulation_parameters import WATER_LENGTH
import zmq
import pynmea2
import _operator
from functools import reduce


class Boat_Sensor(StepEventListener):
    def __init__(self, boat: Assembly, seafloor: HeightField):
        super().__init__()
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
        self.lat_list = [60, 60.00001, 60.0001, 60.001, 61]
        self.lon_list = [l * 1 / 10 for l in self.lat_list]

        self.GPS_i = 0

    def pre(self, time: "agx::TimeStamp const &"):
        delta_x = self.get_travel_distance()
        if delta_x >= self.travel_distance:
            # self.send_zmq(self.message_builder("commands", [self.pay_loader("has_traveled_set_distance", True)]))
            # self.send_ser(str("$" + "has_traveled_set_distance:" + True))
            print(self.GPS_i)
            sent = "GPGGA, , {}, N, {}, W, 2, 0 9, 0, 0, M,,,,".format(self.lat_list[self.GPS_i],
                                                                       self.lon_list[self.GPS_i])
            chk = hex(reduce(_operator.xor, map(ord, sent), 0))[2:].upper()
            sent = "${}*{} \n".format(sent, chk)
            self.send_GPS_ser(sent.encode())
            # self.send_zmq(sent)
            self.GPS_i = self.GPS_i +1 if self.GPS_i < len(self.lon_list)-1 else 0

        if time - self.last_time > self.freq:
            # self.send_ser(str("$" + "has_traveled_set_distance:" + True))
            # $GPGGA,092750.000,5321.6802,N,00630.3372,W,1,8,1.03,61.7,M,55.2,M,,*76
            sent = "SDDBT,{},f,{},M,".format(self.get_depth_under_boat(), self.get_depth_under_boat())
            chk = hex(reduce(_operator.xor, map(ord, sent), 0))[2:].upper()
            sent = "${}*{} \n".format(sent, chk)
            self.send_ser(sent.encode())
            # self.send_zmq(
            #    self.message_builder("commands", [self.pay_loader("depth_beneath_boat", self.get_depth_under_boat())]))
            self.last_time = time
            print(self.last_time, time)

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
        pos,n = self.boat.getPosition()
        x = int(pos[0] + WATER_LENGTH*(n)-1)
        y = int(pos[1])
        return self.seafloor.getHeight(x, y)

    def handle_received_message(self):
        received_command = self.read()
        if received_command[0] == "depth_rov_offset":
            self.depth_beneath_rov_offset = received_command[1]
            self.send_zmq(received_command + ":True")
        elif received_command[0] == "depth_beneath_rov_offset":
            self.depth_rov_offset = received_command[1]
            self.send_zmq(received_command + ":True")

    def send_zmq(self, data):
        self.connection.send_json(data)

    def recv(self):
        data = self.connection.recv_json()
        return data

    def send_ser(self, data):
        print(data)
        self.ser.write(data)

    def send_GPS_ser(self, data):
        print(data)
        self.ser.write(data)
