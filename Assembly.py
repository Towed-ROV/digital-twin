import agxIO
import agx
import agxCollide
# python imports
import math

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
    mesh_reader.readFile("models/rov_simp.obj")
    scaled_vertices = scale_mesh(mesh_reader.getVertices(), agx.Vec3(0.001))
    trimesh = agxCollide.Trimesh(scaled_vertices, mesh_reader.getIndices(), "rov_simp")
    geom = agxCollide.Geometry(trimesh)
    geom.setName('Rov_body')
    geom.setMaterial(aluminum)
    print("volume: ", geom.calculateVolume())
    rov_body = agx.RigidBody(geom)
    print("rov_body:", rov_body)
    rov_body.setMotionControl(agx.RigidBody.DYNAMICS)
    rov_body.add(geom)
    return rov_body


"""Creates rigidbody of the starboard wing from obj file"""


def create_wing_right(aluminum, scale):
    mesh_reader = agxIO.MeshReader()
    mesh_reader.readFile("models/wing_simp.obj")
    scaled_vertices = scale_mesh(mesh_reader.getVertices(), agx.Vec3(scale))
    trimesh = agxCollide.Trimesh(scaled_vertices, mesh_reader.getIndices(), "wing_simp")
    geom = agxCollide.Geometry(trimesh)
    geom.setMaterial(aluminum)
    geom.setName('Wing_R')

    geom.setRotation(agx.EulerAngles(0, math.pi, math.pi))
    print("volume: ", geom.calculateVolume())
    wing_right = agx.RigidBody()
    wing_right.add(geom)

    return wing_right


"""Creates rigidbody of the port wing from obj file"""


def create_wing_left(aluminum, scale):
    mesh_reader = agxIO.MeshReader()
    mesh_reader.readFile("models/wing_simp.obj")
    scaled_vertices = scale_mesh(mesh_reader.getVertices(), agx.Vec3(scale))
    trimesh = agxCollide.Trimesh(scaled_vertices, mesh_reader.getIndices(), "wing_simp")
    geom = agxCollide.Geometry(trimesh)

    geom.setMaterial(aluminum)
    geom.setName('wing_L')
    print("volume: ", geom.calculateVolume())
    wing_left = agx.RigidBody()
    wing_left.add(geom)
    return wing_left


"""function for mapping value with limits, equivalent to Arduinos map function"""


def _map(x, in_min, in_max, out_min, out_max):
    x = min(in_max, max(in_min, x))
    d = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    return d
