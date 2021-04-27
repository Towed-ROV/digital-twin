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
        self.beam = Geometry(agxCollide.Box(1,1,depth))
        self.beam.setPosition(0,0,-depth)
        self.beam.setSensor(True)
        self.setEnable(True)
        self.setFilter(GeometryFilter(ground,self.beam))
        node = agxOSG.createVisual(self.beam,demoutils.root())
        color = agxRender.Color.IndianRed()
        alpha = 0.4
        agxOSG.setDiffuseColor(node, color)
        agxOSG.setAmbientColor(node, agx.Vec4f(1))
        agxOSG.setShininess(node, 120)
        agxOSG.setAlpha(node, alpha)

        self.beam_body=agx.RigidBody(self.beam)

        self.depth = 0

    def contact(self, time: "agx::TimeStamp const &", geometryContact: "GeometryContact"):
        print("contact")
        self.depth = geometryContact.points()
        print(self.depth)
    def impact(self, time: "agx::TimeStamp const &", geometryContact: "GeometryContact"):
        print("impact")
        self.depth = geometryContact.points(0)
        print(self.depth)

    def setPossition(self,pos):
        self.beam_body.setPosition(pos)
        self.beam.setPosition(pos)

    def getPossition(self):
        self.beam.getPosition()
    def getHeight(self):
        return self.depth
    def getBoxBody(self):
        return self.beam_body

    def addSim(self,sim):
        sim.add(self)