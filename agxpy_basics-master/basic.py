import agxOSG
import agxIO
import agx
import agxPython


def build_scene():
    pass


if agxPython.getContext() is None:

    init = agx.AutoInit()

    import sys

    argParser = agxIO.ArgumentParser([sys.executable] + sys.argv)
    example_app = agxOSG.ExampleApplication()
    example_app.addScene(argParser.getArgumentName(1), "build_scene", ord('1'), True)

    if example_app.init(argParser):
        example_app.run()
    else:
        print("An error occurred while initializing ExampleApplication.")
