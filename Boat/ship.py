import agxOSG
import agxRender
import demoutils
import agx
import agxSDK
import agxCollide
import math
from modules.agxPythonModules.utils.callbacks import StepEventCallback as Sec
"""Class Ship, create a agx assembly of the boat used to tow the Rov."""

class Ship(agxSDK.Assembly):
    def __init__(self):
        super().__init__()
        width = 2
        right_fender = width * 0.25
        length = 10 - right_fender * 0.4
        ship_color = agxRender.Color.LightYellow()
        self.m_propulsion_force = 0
        self.m_turn_fraction = 0.1
        self.m_turn = 0
        self.m_propulsion_step = 30
        self.m_max_propulsion = 200000
        self.m_min_propulsion = -500
        self.init(width, length, right_fender)
        self.m_body.getMassProperties().setMass(10000)
        self.m_body.getCmFrame().setLocalTranslate(agx.Vec3(-0.2, 0, 0))
        self.m_body.getGeometries()[0].setEnableCollisions(True)
        agxOSG.setDiffuseColor(agxOSG.createVisual(self.m_body, demoutils.root()), ship_color)

        """Create callbacks"""
        Sec.preCallback(lambda t: self.update_propulsion())
        Sec.postCallback(lambda t: self.display_forces(t))

    def init(self, width, length, rFender):
        ship = agx.RigidBody()
        self.add(ship)
        self.m_body = ship
        self.m_body.setName('boat')

        half_length = length * 0.5
        half_width = width * 0.5
        half_height = 0.25 * half_width
        b = agxCollide.Geometry(agxCollide.Box(half_length, half_width, half_height))
        b.setName('ship')
        """Capsules"""
        radius = half_height * 1.2
        left_c = agxCollide.Geometry(agxCollide.Capsule(radius, length - 2 * radius))
        left_c.setRotation(agx.Quat(math.pi * 0.5, agx.Vec3.Z_AXIS()))
        left_c.setPosition(0, half_width - radius, - (half_height + radius))

        right_capsules = agxCollide.Geometry(agxCollide.Capsule(radius, length - 2 * radius))
        right_capsules.setRotation(agx.Quat(math.pi * 0.5, agx.Vec3.Z_AXIS()))
        right_capsules.setPosition(0, radius - half_width, - (half_height + radius))

        """Fender"""
        fender_material = agx.Material("fenderMaterial")
        landing_material = agx.Material("landingMaterial")
        contact_material = demoutils.sim().getMaterialManager().getOrCreateContactMaterial(fender_material,
                                                                                           landing_material)
        contact_material.setYoungsModulus(5e5)
        contact_material.setFrictionCoefficient(1.0)
        contact_material.setDamping(0.4)
        self.create_fenders(fender_material, rFender, half_width, half_height, half_length)

        """Top"""
        t_box = agxCollide.Geometry(agxCollide.Box(half_length * 0.5, half_width * 0.5, half_height))
        t_box.setPosition(-0.4, 0, 2 * half_height)

        tt_box = agxCollide.Geometry(agxCollide.Box(half_length * 0.2, half_width * 0.4, half_height * 1.1))
        tt_box.setPosition(0, 0, 4.1 * half_height)

        """Assemble ship"""
        ship.add(b)  # base
        ship.add(left_c)  # left capsule
        ship.add(right_capsules)  # left fender
        ship.add(t_box)  # box on top of base
        ship.add(tt_box)  # box on top of box on top of base
        ship.setPosition(-90, 0, 0)

        self.m_left_propeller = agx.Vec3(-half_length, half_width - radius, - (half_height + 2 * radius))
        self.m_right_propeller = agx.Vec3(-half_length, radius - half_width, - (half_height + 2 * radius))

    def create_fenders(self, fender_material, r_fender, half_width, half_height, half_length):
        fender_color = agxRender.Color.Black()

        fender = agxCollide.Geometry(agxCollide.Capsule(0.2, half_width * 2))
        fender.setPosition(half_length, 0, 0)
        self.m_body.add(fender)
        agxOSG.setDiffuseColor(agxOSG.createVisual(fender, demoutils.root()), fender_color)

        fender.setMaterial(fender_material)

    def update_propulsion(self):
        local_forward = agx.Vec3(1, 0, 0)
        rotation = self.getRotation()
        world_forward = rotation * local_forward
        force = world_forward * self.m_propulsion_force
        force = force * -1
        right_force = agx.Vec3()
        left_force = agx.Vec3()
        if self.m_turn == 0:
            right_force = force
            left_force = force
        self.m_body.addForceAtLocalPosition(left_force, self.m_left_propeller)
        self.m_body.addForceAtLocalPosition(right_force, self.m_right_propeller)

    def display_forces(self, t):
        tot_contact_force = agx.Vec3()
        contacts = demoutils.sim().getSpace().getGeometryContacts()
        for contact in contacts:
            points = contact.points()
            for p in points:
                f = p.getForce()
                tot_contact_force = tot_contact_force + f
        demoutils.app().getSceneDecorator().setText(0, "Towed-Rov simulation")
        demoutils.app().getSceneDecorator().setText(1, "Thrust       : {} kN".format(
            self.m_propulsion_force / 500))  # 2/1000 ( 2 because of the 2 propellers )
        demoutils.app().getSceneDecorator().setText(2, "Speed in X direction : {} knots".format(str(round(self.m_body.getVelocity()[0] * 1.94384449, 2))))

    def get_min(self):
        return self.m_propulsion_force

    def increase_propulsion(self, ship):
        self.m_propulsion_force = min(self.m_max_propulsion, self.m_propulsion_force + self.m_propulsion_step)

    def decrease_propulsion(self, ship):

        self.m_propulsion_force = max(self.m_min_propulsion, self.m_propulsion_force - self.m_propulsion_step)
