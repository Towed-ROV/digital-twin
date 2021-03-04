import agx
import agxOSG
import agxUtil
import agxPython
import agxCollide

import agxUtil

from types import SimpleNamespace

gray = agx.Vec4f(0.20, 0.200, 0.200, 1)
orange = agx.Vec4f(0.89, 0.431, 0.145, 1)


class Set():
    """Container class that suppors lookup both using object and dictionary
    style, and iteration. Names must be unique, and inserts with an already
    existing key will be rejected.

    ex:
    bodies = Set()
    bodies.append("first", agx.RigidBody())
    bodies["second"] = agx.RigidBody()

    bodies.first.getName()
    bodies["second"].getName()
    for body in bodies:
        body.getName()
    bodies.append("first", agx.RigidBody("This insert will fail."))
    """
    def __init__(self):
        self.all = []

    def append(self, name, value):
        if name in self.__dict__:
            raise ValueError("There already exists a value named '{}'.".format(name))
        self.__dict__[name] = value
        self.all.append(value)

    def __iter__(self):
        return self.all.__iter__()

    def __len__(self):
        return self.all.__len__()

    def __setitem__(self, name, value):
        self.append(name, value)

    def __getitem__(self, name):
        return self.__dict__[name]


class GenericRobot():
    def __init__(self, simulation):
        self.bodies = Set()
        self.hinges = Set()
        self.robotGroupId = simulation.getSpace().getUniqueGroupID()
        simulation.getSpace().setEnablePair(self.robotGroupId, self.robotGroupId, False)

        self.bodies.append("plate", self.createLink(simulation, "bottom_plate", gray))
        self.bodies.append("bottom", self.createLink(simulation, "bottom", orange))
        self.bodies.append("middle", self.createLink(simulation, "middle", orange))
        self.bodies.append("top", self.createLink(simulation, "top", orange))
        self.bodies.append("neck", self.createLink(simulation, "head_attachment", orange))
        self.bodies.append("head", self.createLink(simulation, "head", gray))
        self.bodies.append("headPlate", self.createLink(simulation, "head_plate", gray))

        self.hinges.append("bottom", self.connect(
            simulation, self.bodies["plate"], self.bodies["bottom"],
            agx.Vec3(0, 0, 0.169), agx.Vec3(0, 0, 1),
            None))
        self.hinges.append("bottomMiddle", self.connect(
            simulation, self.bodies["bottom"], self.bodies["middle"],
            agx.Vec3(0.429, 0, 0.860), agx.Vec3(0, 1, 0),
            agx.RangeReal(-3.05, 0.32)))
        self.hinges.append("middleTop", self.connect(
            simulation, self.bodies["middle"], self.bodies["top"],
            agx.Vec3(0.057, 0, 1.801), agx.Vec3(0, 1, 0),
            agx.RangeReal(3.52, -0.04)))
        self.hinges.append("topHead", self.connect(
            simulation, self.bodies["top"], self.bodies["neck"],
            agx.Vec3(0.906, 0.083, 1.801), agx.Vec3(1, 0, 0),
            None))
        self.hinges.append("head", self.connect(
            simulation, self.bodies["neck"], self.bodies["head"],
            agx.Vec3(1.134, 0, 1.801), agx.Vec3(0, 1, 0),
            agx.RangeReal(2.12, -2.12)))
        self.hinges.append("headPlate", self.connect(
            simulation, self.bodies["head"], self.bodies["headPlate"],
            agx.Vec3(1.32, 0.085, 1.801), agx.Vec3(1, 0, 0), None))

    def createLink(self, simulation, mesh, color):
        filename = "data/models/robots/Generic/" + mesh + ".obj"
        linkRb = agx.RigidBody(filename)
        mesh = agxUtil.createTrimeshFromWavefrontOBJ(filename, agxCollide.Trimesh.NO_WARNINGS, agx.Matrix3x3(), agx.Vec3())
        if mesh is None:
            print("Unable to find file: " + filename)
            return None
        renderData = agxUtil.createRenderDataFromWavefrontOBJ(filename, agx.Matrix3x3(), agx.Vec3())
        mesh.setRenderData(renderData)
        render_material = agxCollide.RenderMaterial()
        render_material.setDiffuseColor(color)
        renderData.setRenderMaterial(render_material)

        meshGeom = agxCollide.Geometry(mesh)
        linkRb.add(meshGeom)
        agxOSG.createVisual(meshGeom, agxPython.getContext().environment.getSceneRoot())
        simulation.add(linkRb)
        agxUtil.addGroup(linkRb, self.robotGroupId)
        return linkRb

    def connect(self, simulation, body1, body2, pos, axis, hingeRange):
        assert (body1 and body2)
        f1 = agx.Frame()
        f2 = agx.Frame()
        agx.Constraint.calculateFramesFromWorld(pos, axis, body1, f1, body2, f2)
        hinge = agx.Hinge(body1, f1, body2, f2)
        if hingeRange:
            hinge.getRange1D().setEnable(True)
            hinge.getRange1D().setRange(hingeRange)

        simulation.add(hinge)
        return hinge
