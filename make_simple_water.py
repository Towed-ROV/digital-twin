import agx
import agxCollide
import agxOSG
import agxRender
import demoutils
from SeafloorBuilder import generate_seafloor

class make_simple_water:
    def make_water(self, adjust_rov, water_density, water_length, water_width, water_height):
        """Creates water"""
        water_material = agx.Material("waterMaterial")
        water_material.getBulkMaterial().setDensity(water_density)
        all_water_geometry = []
        all_bottom_geometry = []
        pos_x = 0
        if adjust_rov:
            water_geometry = agxCollide.Geometry(agxCollide.Box(5, 2, 3))
            water_geometry.setPosition(0, 0, -3)
            bottom_geometry = agxCollide.Geometry(agxCollide.Plane(agx.Vec3.Z_AXIS(), agx.Vec3(0, 0, -3)))
            """Surface of water at z = 0."""
            water_geometry.setMaterial(water_material)
            waterNode = agxOSG.createVisual(water_geometry, demoutils.root())
            color = agxRender.Color.DeepSkyBlue()
            alpha = 0.4
            agxOSG.setDiffuseColor(waterNode, color)
            agxOSG.setAmbientColor(waterNode, agx.Vec4f(1))
            agxOSG.setShininess(waterNode, 120)
            agxOSG.setAlpha(waterNode, alpha)
            all_water_geometry.append(water_geometry)
            all_bottom_geometry.append(bottom_geometry)
            print(water_geometry)
            return water_geometry, all_bottom_geometry
        else:
            water_geometry = agxCollide.Geometry(agxCollide.Box(water_length * 2, water_width, water_height * 2))
            water_geometry.setPosition(-water_length/2, -water_width/2, -water_height)
            bottom_geometry = generate_seafloor(depth=-water_height,length=water_length,width=water_width*10)
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
