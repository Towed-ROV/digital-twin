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
    """
    class that build's components for the rov assembly, can read Obj files to get models.
    """

    def __init__(self):
        super().__init__()
        self.rov_volume = None

    """Creates rigidbody of the rovbody from obj file"""

    def create_rov_body(self, aluminum: agx.Material, scale, name: str, cm,
                        tank_material: agx.Material) -> agx.RigidBody:
        """
        builds a main body for the ROV, the main body consists of a provided 3d model and 2 tanks created by AGX.
        Args:
            aluminum: the material of the ROV man body(not the tanks)
            scale: the scale of the ROV body.
            name: the name of the rigid body and gemomery
            cm: the center of mass of the main body(not the tanks).
            tank_material: the material of the tanks

        Returns:
            a Rigid Body of the  ROV body
        """
        trimesh = self.get_trimesh(MODEL_LOCATION + ROV_MODEL_NAME, scale, ROV_MODEL_NAME)
        geom = self.build_geomery(trimesh, 'Rov_body', aluminum, (0, 0, 0))
        self.rov_volume = geom.getBoundingVolume().size()
        geom.setPosition(agx.Vec3(0, self.rov_volume[2] / 1.15, 0))
        rov_body = self.build_rigid_body(geom, name, (0, 0, 0), (0, 0, 0))
        rov_body.setMotionControl(agx.RigidBody.DYNAMICS)
        rov_body.setCmLocalTranslate(agx.Vec3(*CM_ROV))

        tank1, tank2 = self.build_tanks(material=tank_material)

        print(geom.getShape().getVolume(),
              geom.getShape().getVolume() * (aluminum.getBulkMaterial().getDensity() - 1027))

        print(tank1.getShape().getVolume() * 2, (tank1.getShape().getVolume() + tank2.getShape().getVolume()) * (
                tank_material.getBulkMaterial().getDensity() - 1027))
        rov_body.add(tank1)
        rov_body.add(tank2)
        return rov_body

    """Creates rigidbody of the starboard wing from obj file"""

    def create_wing_right(self, wing_material, scale, name, rot) -> agx.RigidBody:
        """
        build's the right wing of the system.
        Args:
            wing_material: the material of the right wing.
            scale: the scale of the wing.
            name: the name of the wing rigid body and geometry.
            rot: the rotation axis.

        Returns:Rigid body with the wing

        """
        trimesh = self.get_trimesh(MODEL_LOCATION + WING_NAME, scale, WING_NAME)
        geom = self.build_geomery(trimesh, name, wing_material, (0, math.pi, math.pi))
        pos = (self.rov_volume[0] / 12, self.rov_volume[1] / 2, self.rov_volume[2] / 2.2)
        wing_right = self.build_rigid_body(geom, name, pos, rot)
        return wing_right

    """Creates rigidbody of the port wing from obj file"""

    def create_wing_left(self, aluminum, scale, name, rot) -> agx.RigidBody:
        """
        build's the right wing of the system.
        Args:
            wing_material: the material of the right wing.
            scale: the scale of the wing.
            name: the name of the wing rigid body and geometry.
            rot: the rotation axis.

        Returns: Rigid body with the wing

        """
        trimesh = self.get_trimesh(MODEL_LOCATION + WING_NAME, scale, WING_NAME)
        geom = self.build_geomery(trimesh, name, aluminum, (0, 0, 0))
        pos = (self.rov_volume[0] / 12, -self.rov_volume[1] / 2, self.rov_volume[2] / 2.2)
        wing_left = self.build_rigid_body(geom, name, pos, rot)
        return wing_left

    def get_trimesh(self, filename: str, scale, name: str) -> agxCollide.Trimesh:
        """
        reads an obj file, scales it, names it returns a trimesh of that file
        Args:
            filename: file to be read.
            scale: the scale of the mesh
            name: name of the mesh

        Returns:a trimesh of the file provided.

        """
        self.readFile(filename)
        if scale is not 1:
            scaled_vertices = self.scale_mesh(self.getVertices(), agx.Vec3(scale))
            trimesh = agxCollide.Trimesh(scaled_vertices, self.getIndices(), name)
        else:
            trimesh = agxCollide.Trimesh(self.getVertices(), self.getIndices(), name)
        return trimesh

    def build_tanks(self, material):
        """
        builds two tanks an equal distance form a center, the tanks are identical but mirrored version of the same cylinder.
        Args:
            material: material of the tanks.

        Returns:

        """
        length = self.rov_volume[0] / 5
        rad = self.rov_volume[2] / 3
        tank1 = self.cylinders(length, rad)
        tank1.setMaterial(material)
        tank2 = tank1.clone()
        tank1.setRotation(agx.EulerAngles(0, 0, 1 / 2 * math.pi))
        tank2.setRotation(agx.EulerAngles(0, math.pi, math.pi * 3 / 2))
        x, y, z = length / 1.5, rad * 1.5, rad * 3.5
        tank1.setPosition(x, +y, z)
        tank2.setPosition(x, -y, z)
        return tank1, tank2

    @staticmethod
    def build_geomery(shape: agxCollide.Shape, name: str, material, rot) -> agxCollide.Geometry:
        """
        uses an agx shape to build an agx geometr, sets the name, material and rotates it if nessesary
        Args:
            shape: the shape that is contained in the geometry
            name: the name of the geomerty
            material: the material of the geomerty
            rot: the rotation of the geomerty

        Returns: a rigid body of the geomerty provided.

        """
        geom = agxCollide.Geometry(shape)
        geom.setMaterial(material)
        geom.setName(name)
        geom.setRotation(agx.EulerAngles(*rot))
        return geom

    @staticmethod
    def build_rigid_body(geom: agxCollide.Geometry, name: str, pos, rot) -> agx.RigidBody:
        """
        uses an agx geometry tobuild an agx rigid body, sets the position,rotation and name of the body
        Args:
            geom: the geomery to build a rigid body from
            name: the name of the body
            pos: the position of the body
            rot: the rotation of the body

        Returns:a rigid body of the geometry provided
        """
        body = agx.RigidBody()
        body.add(geom)
        body.setName(name)
        body.setPosition(*pos)
        body.setRotation(agx.EulerAngles(*rot))
        return body

    """Creates cyliders geometry"""

    @staticmethod
    def cylinders(half_length, half_height) -> agxCollide.Geometry:
        """
        builds AGX cyliders with the provided shape
        Args:
            half_length:  the half length of the cylinder
            half_height:  the half height of the cylinder

        Returns:

        """
        length = 2 * half_length
        radius = half_height * 1.2
        return agxCollide.Geometry(agxCollide.Cylinder(radius, length - 2 * radius))

    """Function to scale obj model to size"""

    @staticmethod
    def scale_mesh(vertices: agx.Vec3Vector, scale: agx.Vec3) -> agx.Vec3Vector:
        """
        goes throug a mesh and scales every vector in that mesh to treduce or increase it's size.
        Args:
            vertices: the vertecies of the mesh.
            scale: the scale multiplier
        Returns: the scaledvertecies.

        """
        scaled = agx.Vec3Vector()
        for v in vertices:
            scaled.append(agx.Vec3.mul(v, scale))
        return scaled
