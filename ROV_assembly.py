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
from sophusUtil import line_d, print_frame, PI


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

    def rugged_body_from_obj(self, filename, scale, model_name, material, rotation_matrix, body:RigidBody = None):
        mesh = agxUtil._agxUtil.createTrimesh(filename)
        mesh_ref = agxCollide.TrimeshRef(mesh)
        if scale is not 1:
            self.scale_mesh(mesh_ref.getMeshData().getVertices(), agx.Vec3(scale))
        mesh_ref.updateMeshGeometry(True, True)
        mesh = mesh_ref.asTrimesh()
        geometry = self.build_model(mesh, self.rov_pos, material, RigidBody.DYNAMICS, model_name, rigid_body=body)
        geometry.setName(model_name)
        geometry.setRotation(rotation_matrix)

        return geometry

    def obj_to_trimesh(self, filename, scale):
        mesh = agxUtil._agxUtil.createTrimesh(filename)
        mesh_ref = agxCollide.TrimeshRef(mesh)
        print_frame(line_d(), mesh, filename, len(mesh_ref.getMeshData().getVertices()))
        if scale is not 1:
            self.scale_mesh(mesh_ref.getMeshData().getVertices(), agx.Vec3(scale))
        mesh_ref.updateMeshGeometry(True, True)
        print_frame(line_d(),mesh_ref.getBoundingVolume())
        return mesh

    """Function to scale obj model to size"""
    def scale_mesh(self, mesh: agx.Vec3Vector, scale: agx.Vec3):
        for i, verticies in enumerate(mesh):
            mesh.RemoveAt(i)
            mesh.Insert(i,agx.Vec3.mul(verticies, scale))

    def build_model(self, shape, pos, material, motion_controll, name, rigid_body: RigidBody=None):
        geometry = self.build_geometry_from_shape(shape, material, name)
        if not rigid_body:
            rigid_body = self.build_rigid_body(motion_controll, geometry, pos, name)
        else:
            rigid_body.add(geometry)
        return rigid_body

    def build_rigid_body(self, motion_controll, geometry, pos, name):
        part_body = RigidBody()
        part_body.setMotionControl(motion_controll)
        part_body.add(geometry)
        part_body.setPosition(*pos)
        part_body.setName(name)
        return part_body

    def build_geometry_from_shape(self, shape, material, name):
        shape = Geometry(shape)
        shape.setMaterial(material)
        shape.setName(name)
        return shape

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

    def build_PID(self):
        kp = 1000
        ki = 1000
        kd = 1000
        self.pid = PID_Controller(kp, ki, kd, 0)
        self.pid.setName('pid_rov')
        self.pid.set_output_limits(-45, 45)
        self.pid.set_mode(1, 0, 0)
        self.pid.set_setpoint(-20)

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
