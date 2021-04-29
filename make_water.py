# AGX imports
import agx
import agxCollide
from agxCollide import HeightField as hf
import agxOSG
import agxRender
import demoutils
# python imports
import cv2
# local imports
from seafloor_image_generator import seafloorImageBuilder
"""
autor:  me and  j
"""
class MakeWater:
    """

    """
    @staticmethod
    def make_water(water_density, water_length, water_width, water_height):
        """

        Args:
            adjust_rov:
            water_density:
            water_length:
            water_width:
            water_height:

        Returns:

        """

        seafloorImageBuilder.save_new_seafloor_image(width=water_length)
        water_material = agx.Material("waterMaterial")
        water_material.getBulkMaterial().setDensity(water_density)
        water_geometry = agxCollide.Geometry(agxCollide.Box(water_length, water_width/2, water_height/2))
        water_geometry.setPosition(0, 0, -water_height/2)
        seafloor = MakeWater.make_seafloor(vairance=30, length=water_length * 2, width=water_width,
                                                  depth=-water_height )
        """Surface of water at z = 0."""
        water_geometry.setMaterial(water_material)
        MakeWater.add_color(water_geometry)
        print("build water and seafloor")
        return water_geometry, seafloor


    @staticmethod
    def add_color(geomerty):
        waterNode = agxOSG.createVisual(geomerty, demoutils.root())
        color = agxRender.Color.DeepSkyBlue()
        alpha = 0.4
        agxOSG.setDiffuseColor(waterNode, color)
        agxOSG.setAmbientColor(waterNode, agx.Vec4f(1))
        agxOSG.setShininess(waterNode, 120)
        agxOSG.setAlpha(waterNode, alpha)

    @staticmethod
    def make_seafloor(vairance, length, width, depth):
        """

        Args:
            vairance:
            length:
            width:
            depth:

        Returns:

        """
        if depth > 0:
            depth = -depth
        if depth + vairance > 0:
            raise Warning("seafloor above surface")
        seafloor_img = cv2.imread('seafloor.png', cv2.IMREAD_GRAYSCALE)
        seafloor_field = hf.createFromFile('seafloor.png', length, width, depth, depth + vairance)
        return agxCollide.Geometry(seafloor_field)
