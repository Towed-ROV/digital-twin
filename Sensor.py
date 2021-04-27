import agx
import agxCollide
import demoutils
import agxSDK
import agxOSG
import agxRender
from agxSDK import ContactEventListener, GeometryFilter
from agxCollide import Geometry, GeometryContact, ContactPoint


class Sensor(ContactEventListener):
    def __init__(self, ground: agxCollide.Geometry, rov, depth):
        super().__init__()
        self.setMask(ContactEventListener.CONTACT)
        b = agxCollide.Box(.1, .1, depth)
        self.beam = Geometry(b)
        # print(self.beam.getShapes(),self.beam)
        self.beam.setPosition(0, 0, -depth)
        self.beam.setSensor(True)
        self.setFilter(GeometryFilter(self.beam, ground))
        color = agxRender.Color.IndianRed()
        node = agxOSG.createVisual(self.beam, demoutils.root())
        agxOSG.setDiffuseColor(node, color)
        agxOSG.setAmbientColor(node, agx.Vec4f(1))
        agxOSG.setShininess(node, 120)
        agxOSG.setAlpha(node, 0.6)
        self.ground = ground.getShape().asHeightField()

    def contact(self, time: "agx::TimeStamp const &",
                geometryContact: "GeometryContact") -> "agxSDK::ContactEventListener::KeepContactPolicy":
        for p in geometryContact.points():
            d = p.getPoint()[2]
            x = p.getPoint()[0]
            y = p.getPoint()[1]
            """ send depth awayhere as D"""
            # demoutils.app().getSceneDecorator().setText(10, "depth to seafloor : {}".format(str(d),))
        return True
