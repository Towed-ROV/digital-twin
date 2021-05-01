import agxIO
import agx
import agxCollide
#python imports
import math
"""Creates fender geometry"""


"""Creates spoiler geometry"""
def create_spoiler():
    spoiler_geom = agxCollide.Geometry(agxCollide.Box(0.05, 0.38, 0.005))
    spoiler = agx.RigidBody(spoiler_geom)
    return spoiler
class rov_builder(agxIO.MeshReader):
    def __init__(self):
        super().__init__()

    """Creates rigidbody of the rovbody from obj file"""
    def create_rov_body(self,aluminum,scale,name,cm) -> agx.RigidBody:
        trimesh = self.get_trimesh("models/rov_simp.obj", scale, "rov_simp")
        geom = self.build_geomery(trimesh,'Rov_body',aluminum,(0,0,0))
        rov_body = self.build_rigid_body(geom,name,(0,0,0),(0,0,0))
        rov_body.setMotionControl(agx.RigidBody.DYNAMICS)
        rov_body.setCmLocalTranslate(agx.Vec3(*cm))
        [rov_body.add(tank) for tank in self.build_tanks()]
        return rov_body

    """Creates rigidbody of the starboard wing from obj file"""
    def create_wing_right(self,aluminum,scale,name,pos,rot)-> agx.RigidBody:
        trimesh = self.get_trimesh("models/wing_simp.obj",scale,"wing_simp")
        geom = self.build_geomery(trimesh,name,aluminum,(0,math.pi, math.pi))
        wing_right = self.build_rigid_body(geom,name,pos,rot)
        return wing_right

    """Creates rigidbody of the port wing from obj file"""
    def create_wing_left(self,aluminum,scale,name,pos,rot)-> agx.RigidBody:
        trimesh = self.get_trimesh("models/wing_simp.obj",scale,"wing_simp")
        geom = self.build_geomery(trimesh,name,aluminum,(0,0,0))
        wing_left = self.build_rigid_body(geom,name,pos,rot)
        return wing_left

    def get_trimesh(self, filename:str, scale, name:str) -> agxCollide.Trimesh:
        self.readFile(filename)
        if scale is not 1:
            scaled_vertices = self.scale_mesh(self.getVertices(), agx.Vec3(0.001))
            trimesh = agxCollide.Trimesh(scaled_vertices, self.getIndices(), name)
        else:
            trimesh = agxCollide.Trimesh(self.getVertices(), self.getIndices(), name)
        return trimesh

    def build_tanks(self):
        tank1 = self.capsules(155*0.001,170*0.001)
        tank2 = tank1.clone()
        tank1.setRotation(agx.EulerAngles(0,0,1/2*math.pi))
        tank2.setRotation(agx.EulerAngles(0,math.pi, math.pi*3/2))
        tank1.setPosition(0,1,1)
        tank2.setPosition(0,-1,1)
        return tank1,tank2

    @staticmethod
    def build_geomery(shape:agxCollide.Shape, name:str, material,rot)-> agxCollide.Geometry:
        geom = agxCollide.Geometry(shape)
        geom.setMaterial(material)
        geom.setName(name)
        geom.setRotation(agx.EulerAngles(*rot))
        return geom


    @staticmethod
    def build_rigid_body(geom:agxCollide.Geometry,name:str,pos,rot)->agx.RigidBody:
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
    def capsules(half_length, half_height)-> agxCollide.Geometry:
        length = 2 * half_length
        radius = half_height
        return agxCollide.Geometry(agxCollide.Capsule(radius, length - 2 * radius))

    """Creates rigidbody of the boat"""
    @staticmethod
    def ship_body(half_length, half_width, half_height)-> agx.RigidBody:
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
