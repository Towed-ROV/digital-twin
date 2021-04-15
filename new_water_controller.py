# python imports
# agx imports
import agx
import matplotlib
matplotlib.use('agg')
import matplotlib.pyplot as plt
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
import agxCollide


class water_controller:
    def build_water_controller(self, water_dim, adjust_rov):
        water_controller = WindAndWaterController()
        if adjust_rov:
            water_geometry = agxCollide.Geometry(agxCollide.Box(5, 2, 3))
            water_geometry.setPosition(0, 0, -3)
            seafloor = agxCollide.Geometry(agxCollide.Plane(agx.Vec3.Z_AXIS(), agx.Vec3(0, 0, -3)))
        else:
            water_geometry = Geometry(Box(*water_dim))
            water_pos = (water_dim[0]/2, -water_dim[1]/2, -water_dim[2])
            water_geometry.setPosition(*water_pos)
            floor_dim = (i * 2 for i in water_dim)
            seafloor = SeafloorBuilder.generate_seafloor(*floor_dim, amplitude=water_dim[2])
            color = Color.Black()
            demoutils.create_visual(seafloor, diffuse_color=color, ambient_color=color, alpha=1, shininess=0)
            i = 0
            seafloor.setPosition(*water_pos[0:2], i)
            # sim_util.add(hf)
        water_controller.addWater(water_geometry)
        water_geometry.setName("water")
        material = agx.Material("waterMaterial")
        material.getBulkMaterial().setDensity(1027)
        water_geometry.setMaterial(material)
        water_color = Color.DeepSkyBlue()
        color = Color.DarkKhaki()
        demoutils.create_visual(water_geometry, diffuse_color=water_color, ambient_color=color,shininess=120,alpha=0.3)
        #sim_util.add(water_controller)
        #sim_util.add(water_geomotry)
        return water_geometry,water_controller, seafloor


