import osg
import agxOSG
import agxIO
import agx
import agxSDK
import agxPython
import agxCollide
from agxUtil import createTrimeshFromFile

from agxRender import Color


def sim() -> agxSDK.Simulation:
    return agxPython.getContext().environment.getSimulation()


def app() -> agxOSG.ExampleApplication:
    return agxPython.getContext().environment.getApplication()


def root() -> osg.Group:
    return agxPython.getContext().environment.getSceneRoot()


def init_camera(eye=agx.Vec3(-25, -25, 25), center=agx.Vec3(), up=agx.Vec3.Z_AXIS()):
    app().setCameraHome(eye, center, up)


def register_additional_scenes(*additional_scenes):
    if len(additional_scenes) > 0:

        def add_scene(name):
            scene_key = app().getNumScenes() + 1
            app().addScene(script_file_name, name, ord(ascii(scene_key)), True)

        script_file_name = app().getArguments().getArgumentName(1)
        script_file_name = script_file_name.replace('agxscene:', '')
        print(script_file_name)

        for scene in additional_scenes:
            add_scene(scene)


def load_model(path) -> agxCollide.Geometry:
    return agxCollide.Geometry(load_shape(path))


def load_shape(path) -> agxCollide.Trimesh:
    return createTrimeshFromFile(
        path, agxCollide.Trimesh.REMOVE_DUPLICATE_VERTICES, agx.Matrix3x3())


def create_visual(obj, diffuse_color: Color = None, ambient_color: Color = None,
                  shininess=None, alpha: float = None):
    node = agxOSG.createVisual(obj, root())

    if diffuse_color is not None:
        agxOSG.setDiffuseColor(node, diffuse_color)

    if ambient_color is not None:
        agxOSG.setAmbientColor(node, ambient_color)

    if shininess is not None:
        agxOSG.setShininess(node, shininess)

    if alpha is not None:
        agxOSG.setAlpha(node, alpha)

    return node


def create_constraint(**kwds) -> agx.Constraint:
    if 'pos' in kwds:
        pos = kwds['pos']
    else:
        pos = agx.Vec3()

    if 'axis' in kwds:
        axis = kwds['axis']
    else:
        axis = agx.Vec3.Z_AXIS()

    c = kwds['c']
    rb1 = kwds['rb1']
    rb2 = kwds['rb2']
    f1 = agx.Frame()
    f2 = agx.Frame()
    agx.Constraint.calculateFramesFromBody(pos, axis, rb1, f1, rb2, f2)

    return c(rb1, f1, rb2, f2)


if agxPython.getContext() is None:

    import os

    if os.name == "posix":
        agx_dir = os.environ['AGX_DIR']
        print(agx_dir)
        agxIO.Environment_instance().getFilePath(agxIO.Environment.RESOURCE_PATH).addFilePath(agx_dir)
        agxIO.Environment_instance().getFilePath(agxIO.Environment.RESOURCE_PATH).addFilePath(agx_dir + "/data")

    init = agx.AutoInit()

    import sys

    argParser = agxIO.ArgumentParser([sys.executable] + sys.argv)
    print(argParser.getArgumentName(1))
    print('ok')
    example_app = agxOSG.ExampleApplication()
    example_app.addScene(argParser.getArgumentName(1), "build_scene", ord('1'), True)

    if example_app.init(argParser):
        example_app.run()
        pass
    else:
        print("An error occurred while initializing ExampleApplication.")
