import agxWire
import agx
import agxOSG
import demoutils
from rov_simulation_parameters import WIRE_RESOLUTION
"""Create a pulling wire"""
class MakeWire:
    @staticmethod
    def create_wire(density, size, body_1, pos_1, body_2, pos_2):
        """

        Args:
            density:
            size:
            body_1:
            pos_1:
            body_2:
            pos_2:

        Returns:

        """
        wire_material = agx.Material('wireMaterial')
        wire_material.getBulkMaterial().setDensity(density)
        wire = agxWire.Wire(size, WIRE_RESOLUTION, False)
        wire.setLinearVelocityDamping(0.8)
        wire.setMaterial(wire_material)
        wire.add(agxWire.BodyFixedNode(body_1.m_body, pos_1))
        wire.add(agxWire.BodyFixedNode(body_2.link1, pos_2))
        wire.setEnableCollisions(True)
        #wire.setEnableCollisions(body_1.m_body, False)
        #wire.setEnableCollisions(body_2.link1, False)
        wire_renderer = agxOSG.WireRenderer(wire, demoutils.root())
        return wire, wire_renderer