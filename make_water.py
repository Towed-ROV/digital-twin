#AGX imports
import agx
import agxCollide
from agxCollide import HeightField as hf
import agxOSG
import agxRender
import demoutils
import agxModel
import SeafloorBuilder
import matplotlib
matplotlib.use('agg')
import matplotlib.pyplot as plt
# python imports
import cv2

class MakeWater:

    def build_water_controller(self, water_dim, adjust_rov):
        water_controller = agxModel.WindAndWaterController()
        if adjust_rov:
            water_geometry = agxCollide.Geometry(agxCollide.Box(5, 2, 3))
            water_geometry.setPosition(0, 0, -3)
            seafloor = agxCollide.Geometry(agxCollide.Plane(agx.Vec3.Z_AXIS(), agx.Vec3(0, 0, -3)))
        else:
            water_geometry = agxCollide.Geometry(agxCollide.Box(*water_dim))
            water_pos = (water_dim[0]/2, -water_dim[1]/2, -water_dim[2])
            water_geometry.setPosition(*water_pos)
            floor_dim = (i * 2 for i in water_dim)
            seafloor = SeafloorBuilder.generate_seafloor(*floor_dim, amplitude=water_dim[2])
            color = agxRender.Color.Black()
            demoutils.create_visual(seafloor, diffuse_color=color, ambient_color=color, alpha=1, shininess=0)
            i = 0
            seafloor.setPosition(*water_pos[0:2], i)
            # sim_util.add(hf)
        water_controller.addWater(water_geometry)
        water_geometry.setName("water")
        material = agx.Material("waterMaterial")
        material.getBulkMaterial().setDensity(1027)
        water_geometry.setMaterial(material)
        water_color = agxRender.Color.DeepSkyBlue()
        color = agxRender.Color.DarkKhaki()
        demoutils.create_visual(water_geometry, diffuse_color=water_color, ambient_color=color,shininess=120,alpha=0.3)
        #sim_util.add(water_controller)
        #sim_util.add(water_geomotry)
        return water_geometry,water_controller, seafloor


    def make_water(self, adjust_rov, water_density, water_length, water_width, water_height):
        """Creates water"""


        water_material = agx.Material("waterMaterial")
        water_material.getBulkMaterial().setDensity(water_density)

        if adjust_rov:
            water_geometry = agxCollide.Geometry(agxCollide.Box(5, 2, 3))
            water_geometry.setPosition(0, 0, -3)
            bottom_geometry = agxCollide.Geometry(agxCollide.Plane(agx.Vec3.Z_AXIS(), agx.Vec3(0, 0, -3)))
            """Surface of water at z = 0."""
            water_geometry.setMaterial(water_material)
            waterNode = agxOSG.createVisual(water_geometry, demoutils.root())
            color_bottom = agxRender.Color.Red()
            color = agxRender.Color.DeepSkyBlue()
            alpha = 0.4
            agxOSG.setDiffuseColor(waterNode, color)
            agxOSG.setAmbientColor(waterNode, agx.Vec4f(1))
            agxOSG.setShininess(waterNode, 120)
            agxOSG.setAlpha(waterNode, alpha)
            return water_geometry, bottom_geometry
        else:
            #print(water_height)
            water_geometry = agxCollide.Geometry(agxCollide.Box(water_length, water_width, water_height))
            water_geometry.setPosition(0, 0, -water_height)
            bottom_geometry = self.make_seafloor(vairance=water_height, length=water_length*2, width=water_width, depth=-water_height*2)
            """Surface of water at z = 0."""
            water_geometry.setMaterial(water_material)
            waterNode = agxOSG.createVisual(water_geometry, demoutils.root())
            color = agxRender.Color.DeepSkyBlue()
            alpha = 0.4
            agxOSG.setDiffuseColor(waterNode, color)
            agxOSG.setAmbientColor(waterNode, agx.Vec4f(1))
            agxOSG.setShininess(waterNode, 120)
            agxOSG.setAlpha(waterNode, alpha)
            return water_geometry, bottom_geometry

    def make_seafloor(self, vairance, length,width, depth):
        if depth > 0:
            depth = -depth
        if depth + vairance > 0:
            raise Warning("seafloor above surface")
        seafloor_img = cv2.imread('seafloor.png', cv2.IMREAD_GRAYSCALE)

        seafloor_field = hf.createFromFile('seafloor.png', length, width, depth, depth+vairance)
        return agxCollide.Geometry(seafloor_field)
