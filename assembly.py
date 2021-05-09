import agxIO
import agx
import agxCollide
# python imports
import math
from rov_simulation_parameters import *

"""Creates fender geometry"""

"""Creates spoiler geometry"""


def create_spoiler():
    spoiler_geom = agxCollide.Geometry(agxCollide.Box(0.05, 0.38, 0.005))
    spoiler = agx.RigidBody(spoiler_geom)
    return spoiler


class rov_builder(agxIO.MeshReader):
    def __init__(self):
        super().__init__()
        self.rov_volume = None

    """Creates rigidbody of the rovbody from obj file"""

    def create_rov_body(self, aluminum: agx.Material, scale, name: str, cm,
                        tank_material: agx.Material) -> agx.RigidBody:
        trimesh = self.get_trimesh(MODEL_LOCATION + ROV_MODEL_NAME, scale, ROV_MODEL_NAME)
        geom = self.build_geomery(trimesh, 'Rov_body', aluminum, (0, 0, 0))
        self.rov_volume = geom.getBoundingVolume().size()
        geom.setPosition(agx.Vec3(0, self.rov_volume[2] / 1.15, 0))
        rov_body = self.build_rigid_body(geom, name, (0, 0, 0), (0, 0, 0))
        rov_body.setMotionControl(agx.RigidBody.DYNAMICS)
        rov_body.setCmLocalTranslate(agx.Vec3(*cm))

        tank1, tank2 = self.build_tanks(material=tank_material)

        print(geom.getShape().getVolume(),
              geom.getShape().getVolume() * (aluminum.getBulkMaterial().getDensity() - 1027))

        print(tank1.getShape().getVolume() * 2, (tank1.getShape().getVolume() + tank2.getShape().getVolume()) * (
                tank_material.getBulkMaterial().getDensity() - 1027))
        rov_body.add(tank1)
        rov_body.add(tank2)
        return rov_body

    """Creates rigidbody of the starboard wing from obj file"""

    def create_wing_right(self, aluminum, scale, name, rot) -> agx.RigidBody:
        trimesh = self.get_trimesh(MODEL_LOCATION + WING_NAME, scale, WING_NAME)
        geom = self.build_geomery(trimesh, name, aluminum, (0, math.pi, math.pi))
        pos = (self.rov_volume[0] / 12, self.rov_volume[1] / 2, self.rov_volume[2] / 2.2)
        wing_right = self.build_rigid_body(geom, name, pos, rot)
        return wing_right

    """Creates rigidbody of the port wing from obj file"""

    def create_wing_left(self, aluminum, scale, name, rot) -> agx.RigidBody:
        trimesh = self.get_trimesh(MODEL_LOCATION + WING_NAME, scale, WING_NAME)
        geom = self.build_geomery(trimesh, name, aluminum, (0, 0, 0))
        pos = (self.rov_volume[0] / 12, -self.rov_volume[1] / 2, self.rov_volume[2] / 2.2)
        wing_left = self.build_rigid_body(geom, name, pos, rot)
        return wing_left

    def get_trimesh(self, filename: str, scale, name: str) -> agxCollide.Trimesh:
        self.readFile(filename)
        if scale is not 1:
            scaled_vertices = self.scale_mesh(self.getVertices(), agx.Vec3(scale))
            trimesh = agxCollide.Trimesh(scaled_vertices, self.getIndices(), name)
        else:
            trimesh = agxCollide.Trimesh(self.getVertices(), self.getIndices(), name)
        return trimesh

    def build_tanks(self, material):
        length = self.rov_volume[0] / 5
        rad = self.rov_volume[2] / 3
        tank1 = self.cylinders(length, rad)
        tank1.setMaterial(material)
        tank2 = tank1.clone()
        tank1.setRotation(agx.EulerAngles(0, 0, 1 / 2 * math.pi))
        tank2.setRotation(agx.EulerAngles(0, math.pi, math.pi * 3 / 2))
        x, y, z = self.rov_volume[0] / 8, self.rov_volume[2] / 1.9, rad / 1.8 + self.rov_volume[2]
        tank1.setPosition(x, +y, z)
        tank2.setPosition(x, -y, z)
        return tank1, tank2

    @staticmethod
    def build_geomery(shape: agxCollide.Shape, name: str, material, rot) -> agxCollide.Geometry:
        geom = agxCollide.Geometry(shape)
        geom.setMaterial(material)
        geom.setName(name)
        geom.setRotation(agx.EulerAngles(*rot))
        return geom

    @staticmethod
    def build_rigid_body(geom: agxCollide.Geometry, name: str, pos, rot) -> agx.RigidBody:
        body = agx.RigidBody()
        body.add(geom)
        body.setName(name)
        body.setPosition(*pos)
        body.setRotation(agx.EulerAngles(*rot))
        return body

    @staticmethod
    def create_fenders(fender_material, half_width, half_length):
        fender = agxCollide.Geometry(agxCollide.Capsule(0.8, half_width * 2))
        fender.setPosition(half_length, 0, 0.2)
        fender.setMaterial(fender_material)
        return fender

    """Creates capsules geometry"""

    @staticmethod
    def cylinders(half_length, half_height) -> agxCollide.Geometry:
        length = 2 * half_length
        radius = half_height * 1.2
        return agxCollide.Geometry(agxCollide.Cylinder(radius, length - 2 * radius))

    """Creates rigidbody of the boat"""

    @staticmethod
    def ship_body(half_length, half_width, half_height) -> agx.RigidBody:
        geom = agxCollide.Geometry(agxCollide.Box(half_length, half_width, half_height))
        boat = agx.RigidBody(geom)
        boat.setMotionControl(agx.RigidBody.DYNAMICS)
        return boat

    """Function to scale obj model to size"""

    @staticmethod
    def scale_mesh(vertices: agx.Vec3Vector, scale: agx.Vec3):
        scaled = agx.Vec3Vector()
        for v in vertices:
            scaled.append(agx.Vec3.mul(v, scale))
        return scaled


"""function for mapping value with limits, equivalent to Arduinos map function"""


def _map(x, in_min, in_max, out_min, out_max):
    x = min(in_max, max(in_min, x))
    d = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    return d
