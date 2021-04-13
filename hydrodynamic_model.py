import agxOSG
import agx
import demoutils
from agxModel import WindAndWaterController
from agxRender import Color
def generateHydrodymanics(sim,root,model,controller):
    scale = 1.01
    color = Color.Red()
    preshure_field = agxOSG.PressureFieldRenderer(root,color,scale)
    controller.registerPressureFieldRenderer(preshure_field)


if __name__=="__main__":
    controller = WindAndWaterController()
    generateHydrodymanics(demoutils.sim(), demoutils.root(), controller=controller)