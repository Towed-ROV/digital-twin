# python imports
# agx imports
import agx
import agxWire
from agxModel import WindAndWaterController
from agxCollide import Geometry,Box
from agxRender import Color
import agx
import demoutils
import SeafloorBuilder
import ROV_assembly
import time
import threading
from threading import Event


class water_controller:
    def build_water_controller(self,sim_util, water_dim,has_floor=True):
        water_controller = WindAndWaterController()
        water_geomotry = Geometry(Box(*water_dim))
        water_pos = (water_dim[0]/2, water_dim[1]/2, water_dim[2])
        water_geomotry.setPosition(*water_pos)
        water_controller.addWater(water_geomotry)
        water_geomotry.setName("water")
        material = agx.Material("waterMaterial")
        material.getBulkMaterial().setDensity(1027)
        water_geomotry.setMaterial(material)
        water_color = Color.DeepSkyBlue()
        color = Color.DarkKhaki()
        demoutils.create_visual(water_geomotry, diffuse_color=water_color, ambient_color=color,shininess=120,alpha=0.3)
        sim_util.add(water_controller)
        sim_util.add(water_geomotry)
        if has_floor:
            floor_dim = (i * 2 for i in water_dim)
            hf = SeafloorBuilder.generate_seafloor(*floor_dim, amplitude=water_dim[2])
            color = Color.Black()
            demoutils.create_visual(hf, diffuse_color=color, ambient_color=color, alpha=1, shininess=0)
            i = water_pos[2]*2
            hf.setPosition(*water_pos[0:2],i)
            sim_util.add(hf)
        return water_controller,water_geomotry


def build_scene():
     sim = demoutils.sim()
     root = demoutils.root()
     sim.setTimeStep(0.125)
     water_length = 2000
     water_dim = (water_length, 400, 400)
     controller, geomotry = water_controller().build_water_controller(sim, water_dim, has_floor=True)

     r= ROV_assembly.assembler(wire_length=1500, pos=(-water_dim[0]/2 + 100, water_dim[1]/2, water_dim[2]*2), speed=50, sim=sim, root=root)
     bodies = r.build_bodies(geomotry, controller)
     for geomotries in bodies:
         if type(geomotries) is not agxWire.Wire:
             color = Color.Black()
             demoutils.create_visual(geomotries, diffuse_color=color)
     r = runway(sim,water_length,bodies[0])
     #r.run()

class runway(threading.Thread):
    def __init__(self, sim, length, boat):
        threading.Thread.__init__(self)
        self.sim = sim
        self.water_length = length
        self.water = self.sim.getGeometry("water")
        self.boat = boat

    def run(self):
        time.sleep(10)
        print("hello", 80, self.water)
        while True:
            self.check_event()
            time.sleep(1)

    def check_event(self):
        pos = self.boat.getPosition()
        water_pos = self.water.getPosition()
        print(water_pos,pos)
        if self.water_length + water_pos[0] - pos[0] < 100:
            self.water.setPosition(water_pos[0]+self.water_length-pos[0]-10, water_pos[1], water_pos[2])




