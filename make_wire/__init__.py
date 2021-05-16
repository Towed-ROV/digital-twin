import agxWire
import agx
import agxOSG
from agxOSG import createVisual
import demoutils
from rov_simulation_parameters import WIRE_RESOLUTION

"""Create a pulling wire"""


class MakeWire:
    @staticmethod
    def create_wire(density, radius, body_1, pos_1, body_2, pos_2):
        """
        creates a wire and a wire renderer.
        Args:
            density: density of the wire material
            radius: the radius of the wire
            body_1: the body in one end of the wire
            pos_1: the position the wire is fastend on that body
            body_2: the body in the other end of the wire
            pos_2: the position the wire is fastend on that body
        Returns: and agx wire and a agx wire renderer for calculating the wire movement.
        """
        wire_material = agx.Material('wireMaterial')
        wire_material.getBulkMaterial().setDensity(density)
        wire = agxWire.Wire(radius, WIRE_RESOLUTION, False)
        wire.setLinearVelocityDamping(0.8)
        wire.setMaterial(wire_material)
        wire.add(agxWire.BodyFixedNode(body_1.m_body, pos_1))
        wire.add(agxWire.BodyFixedNode(body_2.link1, pos_2))
        wire.setEnableCollisions(True)
        wire.setEnableCollisions(body_1.m_body, False)
        wire.setEnableCollisions(body_2.link1, False)
        wire_renderer = agxOSG.WireRenderer(wire, demoutils.root())
        return wire, wire_renderer
