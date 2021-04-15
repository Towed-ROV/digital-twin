import agx
import agxUtil
import agxCollide
import demoutils
import agxModel
from pid import PID_Controller
from agxOSG import WireRenderer
from agxCollide import Geometry, Box, Trimesh
from agxWire import Wire, BodyFixedNode
from agxModel import WindAndWaterParameters
from agx import Material, RigidBody, OrthoMatrix3x3


class assembler:
    def __init__(self, pos, wire_length, speed): # sim, root):
        self.wire_length = wire_length
        self.rov_pos = (*pos[0:2], pos[2]-40)
        self.speed = speed
        self.boat_pos = (pos[0] + self.wire_length, *pos[1:3])
        #self.sim = sim
        #self.root = root

    def build_rov(self, material):
        #material = Material('AluminumMaterial')
        #material.getBulkMaterial().setDensity(708)
        rov = self.rugged_body_from_obj("test_obj_new_simple.obj", 0.001, "rov_body", material,
                                       OrthoMatrix3x3(1, 0, 0, 0, 1, 0, 0, 0, 1))
        wingR = self.rugged_body_from_obj("wing_simp.obj", 1, "wingR", material,
                                    OrthoMatrix3x3(1, 0, 0, 0, 1, 0, 0, 0, 1))
        wingL = self.rugged_body_from_obj("wing_simp.obj", 1, "wingL", material,
                                    OrthoMatrix3x3(1, 0, 0, 0, 1, 0, 0, 0, 1))
        for bodies in rov.getGeometries():
            bodies.setRotation(OrthoMatrix3x3(-1, 0, 0, 0, 1, 0, 0, 0, 1))

        rov.setRotation(rov.getRotation().rotate(PI,0,0,PI))

        widthR = .05
        widthL = .55
        pos = rov.getPosition()
        wing_pos_R = agx.Vec3(pos[0]+0.04, pos[1] - widthR, pos[2] + .1)
        wing_pos_L = agx.Vec3(pos[0]+0.04, pos[1] + widthL, pos[2] + .1)
        wingL.setRotation(wingL.getRotation().rotate(0,0,0,PI))
        wingR.setRotation(wingL.getRotation().rotate(0,0,0,0))
        wingL.setPosition(wing_pos_R)
        wingR.setPosition(wing_pos_L)
        return rov, wingR, wingL



    def wire_builder(self, resolution=2):
        material = agx.Material('wireMaterial')
        wire = Wire(radius=0.25, resolutionPerUnitLength=resolution, enableCollisions=False)
        wire.setEnableCollisions(False)
        wire.setMaterial(material)
        wire.setName("wire")
        return wire

    def build_boat(self, boat_pos=None):
        boat_pos = self.boat_pos if not boat_pos else boat_pos
        boat_shape = Box(*(20, 20, 20))
        material = Material("AluminumMaterial")
        boat_shape = Geometry(boat_shape)

        boat_body = self.build_rigid_body(RigidBody.KINEMATICS,boat_shape, boat_pos, "boat")
        for geo in boat_body.getGeometries():
            geo.setMaterial(material)
        boat_body.setVelocity(agx.Vec3(self.speed, 0, 0))
        boat_body.setName("boat")
        return boat_body

    def build_bodies(self,material): # , water, water_controller):
        wire = self.wire_builder() # water)
        # wire, wire_renderer = self.wire_builder()  # water)
        rov, wingL, wingR = self.build_rov(material)
        boat = self.build_boat()
        wire.add(BodyFixedNode(rov, agx.Vec3(-10, -18, 20)))
        wire.add(BodyFixedNode(boat,  agx.Vec3(-20, 0, 20)))


        return boat, wire, rov

    def displayForces(self, t,rov):

        print(rov.getPosition()[2])
