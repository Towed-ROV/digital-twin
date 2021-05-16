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
from make_water.seafloor_image_generator import seafloorImageBuilder
from rov_simulation_parameters import SEAFLOOR_VARIANCE

"""
autor:  me and  j
"""


class MakeWater:
    """
    this class builds water and seafloor geomerties ads the water and adds the water to a controller.
    """

    @staticmethod
    def make_water(water_density, water_length, water_width, water_height):
        """
        generates seafloor and water.
        Args:
            water_density: the water dencity
            water_length: the length of the water and the seafloor
            water_width:  width of the water and the seafloor
            water_height: depth of the water and the max depth of the seafloor

        Returns: A Water geometry and a  seafloor geometry

        """

        seafloorImageBuilder.save_new_seafloor_image(width=water_length)
        water_material = agx.Material("waterMaterial")
        water_material.getBulkMaterial().setDensity(water_density)
        water_geometry = agxCollide.Geometry(agxCollide.Box(water_length, water_width / 2, water_height / 2))
        water_geometry.setPosition(0, 0, -water_height / 2)
        seafloor = MakeWater.make_seafloor(vairance=SEAFLOOR_VARIANCE, length=water_length * 2, width=water_width,
                                           depth=-water_height)
        """Surface of water at z = 0."""
        water_geometry.setMaterial(water_material)
        MakeWater.add_color(water_geometry)
        """ ads visualisation"""
        agxOSG.createVisual(seafloor, demoutils.root())
        print("build water and seafloor")
        return water_geometry, seafloor

    @staticmethod
    def add_color(geomerty):
        """
        adds bluesih transparent color to a provided geomerty
        Args:
            geomerty: the water geometry that you want to add collor to.


        """
        waterNode = agxOSG.createVisual(geomerty, demoutils.root())
        color = agxRender.Color.DeepSkyBlue()
        alpha = 0.4
        agxOSG.setDiffuseColor(waterNode, color)
        agxOSG.setAmbientColor(waterNode, agx.Vec4f(1))
        agxOSG.setShininess(waterNode, 120)
        agxOSG.setAlpha(waterNode, alpha)

    @staticmethod
    def make_seafloor(vairance, length, width, depth)->agxCollide.Geometry:
        """
        builds a heightfield for the water.
        Args:
            vairance: the vaiance in height for the height field, should always be less then the depth of the water,
            or the seafloor would be above the water surface.
            length:
            the length of the height field
            width: the with of the height field
            depth: the depth of the height fileld

        Returns: a Geomery with the heightfield shape.
        """
        if depth > 0:
            depth = -depth
        if depth + vairance > 0:
            raise Warning("seafloor above surface")
        seafloor_field = hf.createFromFile('seafloor.png', length, width, depth, depth + vairance)
        return agxCollide.Geometry(seafloor_field)
