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
        self.travel_distance = 10
        self.message_builder = lambda type, data: {"payload_name": type, "payload_data": data}
        self.pay_loader = lambda name, value: {"name": name, "value": value}
        self.last_time = 0
        self.freq = 1 / 2
        self.lat_list = [60.0000000001, 61.010000000001, 62.020000000001, 63.030000000001, 64.040000000001,
                         65.050000000001, 66.060000000001, 67.070000000001, 68.080000000001, 69.090000000001,
                         700000000001]
        self.lon_list = [l * 1 / 10 for l in self.lat_list]

        self.GPS_i = 0

    def pre(self, time: "agx::TimeStamp const &"):
        """
        gather data and send NMEA data to the surface Unit software.
        Args:
            time: in simulation time

        Returns:

        """
        if time - self.last_time > self.freq:
            delta_x = self.get_travel_distance()
            #print(delta_x,self.travel_distance)
            if delta_x >= self.travel_distance:
                sent = "GPGGA, , {}, N, {}, W, 2, 0 9, 0, 0, M,,,,".format(self.lat_list[self.GPS_i],
                                                                           self.lon_list[self.GPS_i])
                print(sent)
                chk = hex(reduce(_operator.xor, map(ord, sent), 0))[2:].upper()
                sent = "${}*{} \n".format(sent, chk)
                self.send(sent.encode())
                self.GPS_i = self.GPS_i + 1 if self.GPS_i < len(self.lon_list) - 1 else 0

            sent = "SDDBT,{},f,{},M,".format(-self.get_depth_under_boat()*3.2808, -self.get_depth_under_boat())
            chk = hex(reduce(_operator.xor, map(ord, sent), 0))[2:].upper()
            sent = "${}*{} \n".format(sent, chk)
            self.send(sent.encode())
            self.last_time = time

    def connect(self):
        """
        conect over ZMQ.

        """
        self.connection.connect(f"tcp://{self.host}:{self.port}")

    def get_travel_distance(self):
        """
        check how far the boat has traveled in the simulation
        Returns: float of the travel distance

        """
        pos, n = self.boat.getPosition()
        x = pos[0]
        delta_x = x - self.last_x
        if delta_x >= self.travel_distance:
            self.last_x = x
        return delta_x

    def get_depth_under_boat(self) -> float:
        """
        gets the depth from the boat to the seafloor
        Returns: float with the seafloor depth under the boat

        """
        pos, n = self.boat.getPosition()
        x = int(pos[0] + WATER_LENGTH * (n) - 1)
        y = int(pos[1])
        return round(self.seafloor.getHeight(x, y),1)


    def send(self, data:bytes):
        """
        send data over serial as the echolod.
        Args:
            data: encoded data to be sendt


        """
        self.ser.write(data)

    def send_GPS_ser(self, data):
        """

        send data over serial as the gps.
        Args:
            data: serial encoded data


        """
        self.serGPS.write(data)

    def post(self,t):
        """
        adds  the depth under the boat to the window.
        Args:
            t: in simulation time
        """
        demoutils.app().getSceneDecorator().setText(8, "Depth under boat : {} m".format(
            str(round(self.get_depth_under_boat() , 2))))
