import agxIO
import agx
import agxCollide
import agxOSG
import agxRender
import agxUtil
import functions
import demoutils
import math


def rugged_body_from_obj(self, filename, scale, model_name, material, rotation_matrix, body: agx.RigidBody = None):
    mesh = agxUtil.createTrimeshFromFile(filename)
    mesh_ref = agxCollide.TrimeshRef(mesh)
    if scale is not 1:
        self.scale_mesh(mesh_ref.getMeshData().getVertices(), agx.Vec3(scale))
    mesh_ref.updateMeshGeometry(True, True)
    mesh = mesh_ref.asTrimesh()
    geometry = self.build_model(mesh, self.rov_pos, material, agx.RigidBody.DYNAMICS, model_name, rigid_body=body)
    geometry.setName(model_name)
    geometry.setRotation(rotation_matrix)

    return geometry


def obj_to_trimesh(self, filename, scale):
    mesh = agxUtil.createTrimeshFromFile(filename)
    mesh_ref = agxCollide.TrimeshRef(mesh)
    functions.print_line(mesh, filename, len(mesh_ref.getMeshData().getVertices()))
    if scale is not 1:
        self.scale_mesh(mesh_ref.getMeshData().getVertices(), agx.Vec3(scale))
    mesh_ref.updateMeshGeometry(True, True)
    functions.print_line(mesh_ref.getBoundingVolume())
    return mesh


"""Function to scale obj model to size"""


def scale_mesh(self, mesh: agx.Vec3Vector, scale: agx.Vec3):
    for i, verticies in enumerate(mesh):
        mesh.RemoveAt(i)
        mesh.Insert(i, agx.Vec3.mul(verticies, scale))


def build_model(self, shape, pos, material, motion_controll, name, rigid_body: agx.RigidBody = None):
    geometry = self.build_geometry_from_shape(shape, material, name)
    if not rigid_body:
        rigid_body = self.build_rigid_body(motion_controll, geometry, pos, name)
    else:
        rigid_body.add(geometry)
    return rigid_body


def build_rigid_body(self, motion_controll, geometry, pos, name):
    part_body = agx.RigidBody()
    part_body.setMotionControl(motion_controll)
    part_body.add(geometry)
    part_body.setPosition(*pos)
    part_body.setName(name)
    return part_body


def build_geometry_from_shape(self, shape, material, name):
    shape = agxCollide.Geometry(shape)
    shape.setMaterial(material)
    shape.setName(name)
    return shape


"""Creates spoiler geometry"""
def create_spoiler():
    spoiler_geom = agxCollide.Geometry(agxCollide.Box(0.05, 0.38, 0.005))
    spoiler = agx.RigidBody(spoiler_geom)
    return spoiler

"""Creates rigidbody of the rovbody from obj file"""


def build_rov(material):
    # material = Material('AluminumMaterial')
    # material.getBulkMaterial().setDensity(708)
    rov = rugged_body_from_obj("test_obj_new_simple.obj", 0.001, "rov_body", material,
                                    agx.OrthoMatrix3x3(1, 0, 0, 0, 1, 0, 0, 0, 1))
    wingR = rugged_body_from_obj("wing_simp.obj", 1, "wingR", material,
                                      agx.OrthoMatrix3x3(1, 0, 0, 0, 1, 0, 0, 0, 1))
    wingL = rugged_body_from_obj("wing_simp.obj", 1, "wingL", material,
                                      agx.OrthoMatrix3x3(1, 0, 0, 0, 1, 0, 0, 0, 1))
    for bodies in rov.getGeometries():
        bodies.setRotation(agx.OrthoMatrix3x3(-1, 0, 0, 0, 1, 0, 0, 0, 1))

    rov.setRotation(rov.getRotation().rotate(math.pi, 0, 0, math.pi))

    widthR = .05
    widthL = .55
    pos = rov.getPosition()
    wing_pos_R = agx.Vec3(pos[0] + 0.04, pos[1] - widthR, pos[2] + .1)
    wing_pos_L = agx.Vec3(pos[0] + 0.04, pos[1] + widthL, pos[2] + .1)
    wingL.setRotation(wingL.getRotation().rotate(0, 0, 0, math.pi))
    wingR.setRotation(wingL.getRotation().rotate(0, 0, 0, 0))
    wingL.setPosition(wing_pos_R)
    wingR.setPosition(wing_pos_L)
    return rov, wingR, wingL


