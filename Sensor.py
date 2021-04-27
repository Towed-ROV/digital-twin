import agx
import agxCollide
import demoutils
import agxSDK
import agxOSG
import agxRender
from agxSDK import ContactEventListener,GeometryFilter
from agxCollide import Geometry,GeometryContact
class Sensor(ContactEventListener):
    def __init__(self,ground,rov,depth):
        super().__init__()
        self.setMask(ContactEventListener.CONTACT)
        self.beam = Geometry(agxCollide.Box(.1,.1,depth))
        self.beam.setPosition(0,0,-depth)
        self.beam.setSensor(True)
        self.beam.setEnableCollisions((False))
        self.setFilter(GeometryFilter(ground,self.beam))
        color = agxRender.Color.IndianRed()
        node = agxOSG.createVisual(self.beam,demoutils.root())
        agxOSG.setDiffuseColor(node, color)
        agxOSG.setAmbientColor(node, agx.Vec4f(1))
        agxOSG.setShininess(node, 120)
        agxOSG.setAlpha(node, 1)
        self.depth = 0

    def contact(self, time: "agx::TimeStamp const &", geometryContact: "GeometryContact"):
        print("contact")

    def setPossition(self,pos):
        self.beam.setPosition(pos)

    def getPossition(self):
        self.beam.getPosition()

    def getHeight(self):
        return self.depth

    def addSim(self,sim):
        sim.add(self)