import serial
from agxSDK import StepEventListener, Assembly
from agxCollide import HeightField, Geometry
from rov_simulation_parameters import WATER_LENGTH
import zmq
import json


class Boat_Sensor(StepEventListener):
    def __init__(self, boat: Assembly, seafloor: HeightField):
        super().__init__()
        self.interval = 0.01
        self.ctx = zmq.Context()
        self.connection = self.ctx.socket(zmq.PUB)
        self.host = "127.0.0.1"
        self.port = "6969"
        self.boat = boat
        self.seafloor = seafloor
        self.last_x = boat.getPosition()[0]
        self.travel_distance = 4
        self.message_builder = lambda type, data: str({"payload_name": type, "payload_data": data})
        self.pay_loader = lambda name, value: str({"name":name,"value":value})

    def pre(self, time: "agx::TimeStamp const &"):
        delta_x = self.get_travel_distance()
        print(delta_x)
        if delta_x >= self.travel_distance:
            self.send(self.message_builder("commands", [self.pay_loader("has_traveled_set_distance",True)]))
        self.send(self.message_builder("sensors", [self.pay_loader("depth_under_boat", self.get_depth_under_boat())]))
    def connect(self):
        self.connection.connect(f"tcp://{self.host}:{self.port}")

    def get_travel_distance(self):
        x = self.boat.getPosition()[0]
        delta_x = x - self.last_x
        if delta_x > self.travel_distance:
            self.last_x = x
        return delta_x

    def get_depth_under_boat(self) -> float:
        pos = self.boat.getPosition()
        x = int(pos[0] + WATER_LENGTH)
        y = int(pos[1])
        print(x, y)
        return self.seafloor.getHeight(x, y)

    def handle_received_message(self):
        received_command = self.read()
        if received_command[0] == "depth_rov_offset":
            self.depth_beneath_rov_offset = received_command[1]
            self.send(received_command + ":True")
        elif received_command[0] == "depth_beneath_rov_offset":
            self.depth_rov_offset = received_command[1]
            self.send(received_command + ":True")


    def send(self, data):
        print(data)
        self.connection.send_json(data)

    def recv(self):
        data = self.connection.recv_json()
        print(data)
        return data
