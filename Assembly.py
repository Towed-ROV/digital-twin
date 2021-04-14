import agxIO
import agx
import agxCollide
import agxUtil
import inspect
from agx import RigidBody
"""Creates fender geometry"""
def create_fenders(self, fender_material, half_width, half_length):
    fender = agxCollide.Geometry(agxCollide.Capsule(0.8, half_width * 2))
    fender.setPosition(half_length, 0, 0.2)
    fender.setMaterial(fender_material)
    return fender

"""Creates capsules geometry"""
def capsules(half_length, half_height):
    length = 2 * half_length
    radius = half_height * 1.2
    return agxCollide.Geometry(agxCollide.Capsule(radius, length - 2 * radius))

"""Creates rigidbody of the boat"""
def ship_body(half_length, half_width, half_height):
    geom = agxCollide.Geometry(agxCollide.Box(half_length, half_width, half_height))
    boat = agx.RigidBody(geom)
    boat.setMotionControl(agx.RigidBody.DYNAMICS)
    return boat

"""Function to scale obj model to size"""
def scale_mesh(vertices: agx.Vec3Vector, scale: agx.Vec3):
    scaled = agx.Vec3Vector()
    for v in vertices:
        scaled.append(agx.Vec3.mul(v, scale))
    return scaled

"""Creates spoiler geometry"""
def create_spoiler():
    spoiler_geom = agxCollide.Geometry(agxCollide.Box(0.05, 0.38, 0.005))
    spoiler = agx.RigidBody(spoiler_geom)
    return spoiler

"""Creates rigidbody of the rovbody from obj file"""
def create_rov_body(aluminum) -> agx.RigidBody:
    # trimesh = agxOSG.readNodeFile("models/test2.stl", False)
    mesh_reader = agxIO.MeshReader()
    mesh_reader.readFile("test_obj_new_simple.obj")
    scaled_vertices = scale_mesh(mesh_reader.getVertices(), agx.Vec3(1))
    trimesh = agxCollide.Trimesh(scaled_vertices, mesh_reader.getIndices(), 'test_obj_new_simple')
    geom = agxCollide.Geometry(trimesh)
    geom.setName('rr')
    geom.setMaterial(aluminum)
    print(geom.calculateVolume())
    rov_body = agx.RigidBody()
    rov_body.setMotionControl(agx.RigidBody.DYNAMICS)
    rov_body.add(geom)
    return rov_body

"""Creates rigidbody of the starboard wing from obj file"""
def create_wing_right(aluminum):
    mesh_reader = agxIO.MeshReader()
    mesh_reader.readFile("wing_simp.obj")
    scaled_vertices = scale_mesh(mesh_reader.getVertices(), agx.Vec3(0.001))
    trimesh = agxCollide.Trimesh(scaled_vertices, mesh_reader.getIndices(), "wingR")
    geom = agxCollide.Geometry(trimesh)
    geom.setMaterial(aluminum)
    geom.setName('rr')
    wing_right = agx.RigidBody()
    wing_right.add(geom)
    return wing_right

"""Creates rigidbody of the port wing from obj file"""
def create_wing_left(aluminum):
    mesh_reader = agxIO.MeshReader()
    mesh_reader.readFile("wing_simp.obj")
    scaled_vertices = scale_mesh(mesh_reader.getVertices(), agx.Vec3(0.001))
    trimesh = agxCollide.Trimesh(scaled_vertices, mesh_reader.getIndices(), "wingL")
    geom = agxCollide.Geometry(trimesh)
    geom.setMaterial(aluminum)
    geom.setName('rr')
    wing_left = agx.RigidBody()
    wing_left.add(geom)
    return wing_left

def ruged_body_from_obj(filename, scale, model_name, material, rotation_matrix, rov_pos,body:RigidBody = None):
    mesh = agxUtil._agxUtil.createTrimesh(filename)
    mesh_ref = agxCollide.TrimeshRef(mesh)
    if scale is not 1:
        scale_mesh(mesh_ref.getMeshData().getVertices(), agx.Vec3(scale))
    mesh_ref.updateMeshGeometry(True, True)
    print_line( mesh_ref.getBoundingVolume().size() ,mesh_ref.getBoundingVolume().thisown,model_name)
    geometry = build_model(mesh, rov_pos, material, RigidBody.DYNAMICS, model_name,rigid_body=body)
    geometry.setName(model_name)
    geometry.setRotation(rotation_matrix)

def build_model(shape, pos, material, motion_controll, name, rigid_body: RigidBody=None):
    geometry = build_geometry_from_shape(shape, material, name)
    if not rigid_body:
        rigid_body = build_rigid_body(motion_controll, geometry, pos, name)
    else:
        rigid_body.add(geometry)
    return rigid_body

line_d = inspect.currentframe
def print_frame(f:line_d, *args):
    info = inspect.getframeinfo(f)
    print("\n------------------------------------------------------------")
    if len(args):
        for arg in args:
            print("  |==>  message: ", arg, "\n  |------------------------------------------------------------")
    print("  |printed at line: %s\n  |in fuction: %s \n  |in document:%s" % (info.lineno, info.function, info.filename))
    print("------------------------------------------------------------\n")


"""function for mapping value with limits, equivalent to Arduinos map function"""
def _map(x, in_min, in_max, out_min, out_max):
    x = min(in_max, max(in_min, x))
    d = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    return d

