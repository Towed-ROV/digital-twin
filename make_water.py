import agx
import agxCollide
import agxOSG
import agxRender
import demoutils

class MakeWater:
    def make_water(self, adjust_rov, water_density, water_length, water_width, water_height):
        """Creates water"""
        water_material = agx.Material("waterMaterial")
        water_material.getBulkMaterial().setDensity(water_density)
        if adjust_rov:
            water_geometry = agxCollide.Geometry(agxCollide.Box(5, 2, 3))
            water_geometry.setPosition(0, 0, -3)
        else:
            water_geometry = agxCollide.Geometry(agxCollide.Box(water_length, water_width, water_height))
            water_geometry.setPosition(0, 0, -water_height)
        """Surface of water at z = 0."""
        water_geometry.setMaterial(water_material)
        waterNode = agxOSG.createVisual(water_geometry, demoutils.root())
        color = agxRender.Color.DeepSkyBlue()
        alpha = 0.4
        agxOSG.setDiffuseColor(waterNode, color)
        agxOSG.setAmbientColor(waterNode, agx.Vec4f(1))
        agxOSG.setShininess(waterNode, 120)
        agxOSG.setAlpha(waterNode, alpha)
        return water_geometry