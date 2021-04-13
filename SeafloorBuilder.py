# AGX modules
import agx
import agxModel
import agxPython
import agxCollide
import agxOSG
import agxIO
import agxSDK
import agxUtil
import agxRender

# Python modules
import sys
import math
import numpy as np

# Local modules
from Seafloor_tracking.sftA_py.src import seafloor_generater

def generate_seafloor(length, width,depth, amplitude=30): #, sim=None, app=None, root=None):
    # Create a heightfield from a picture
    if depth > 0:
        depth = -depth
    depth_max = depth
    depth_min = depth_max + amplitude
    resolution = 2
    print(length, width, resolution)
    seafloor_generater.get_and_build(width=width, length=length/resolution, resolution=resolution)
    hf = agxCollide.HeightField.createFromFile("seafloor.png", length, width, depth_max, depth_min)
    hf = agxCollide.Geometry(hf)
    return hf


# Entry point if script is launched using agxViewer
def buildScene():
    # sim - A pointer to an instance of a agxSDK::Simulation
    # app - A pointer to an instance of a agxOSG::ExampleApplication
    # root - A pointer to an instance of agxOSG::Group
    sim = agxPython.getContext().environment.getSimulation()
    app = agxPython.getContext().environment.getApplication()
    root = agxPython.getContext().environment.getSceneRoot()

    # We just call buildScene1 to create a scene
    # Multiple scenes can be added with app.addScene, now we add only this one
    hf = buildScene1(10, 100, 10)
    sim.add(hf)
    agxOSG.createVisual(hf, root)


def main(args):
    # Create an application with graphics etc.
    app = agxOSG.ExampleApplication()

    # Create a command line parser. sys.executable will point to python executable
    # in this case, because getArgumentName(0) needs to match the C argv[0] which
    # is the name of the program running
    argParser = agxIO.ArgumentParser([sys.executable] + args)

    app.addScene(argParser.getArgumentName(1), "buildScene", ord('1'), True)

    # Call the init method of ExampleApplication
    # It will setup the viewer, windows etc.
    if app.init(argParser):
        app.run()
    else:
        print("An error occurred while initializing ExampleApplication.")


# Entry point when this script is loaded with python
if agxPython.getContext() is None:
    init = agx.AutoInit()
    main(sys.argv)