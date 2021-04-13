import agxModel

import demoutils
import agx
<<<<<<< HEAD
import agxOSG
=======
from agxOSG import createVisual
>>>>>>> Sophus
from Boat.ship import Ship
from Boat.boat_controller import Boat_Controller
from Rov.rov_assembly import rovAssembly
from Rov.rov_controller import RovController
from keyboard_listener import KeyboardListener
from make_water import MakeWater
from make_simple_water import make_simple_water
from make_wire import MakeWire
from pid import PID_Controller
import matplotlib as plt
import math

def setBodyViscousDrag(body, controller, viscousDrag):
    geometries = body.getGeometries()
    for geom in geometries:
        shapes = geom.getShapes()
        for shape in shapes:
            # controller.getOrCreateHydrodynamicsParameters(shape).setCoefficient(agxModel.WindAndWaterParameters.LIFT, 2)
            controller.getOrCreateHydrodynamicsParameters(shape).setCoefficient(agxModel.WindAndWaterParameters.PRESSURE_DRAG, 0.9)
            # controller.getOrCreateHydrodynamicsParameters(shape).setCoefficient(agxModel.WindAndWaterParameters.VISCOUS_DRAG, 0.03)
            # Prepare for pressure rendering of the hull too
            # controller.registerPressureFieldRenderer(agxOSG.PressureFieldRenderer(demoutils.root,1.01), shape)
            pass


def buildScene():
    """Building scene for simulation and adding it to the simulation. Also setting the simulation step"""""
    demoutils.sim().setTimeStep(0.05)
    print('hello')
    build_scene()

"""Building scene"""
def build_scene():
    """write plot to csv file variable"""
    plot = True
    adjust_rov = False
    kp = 1000
    ki = 1000
    kd = 1
    kp_trim = 0
    ki_trim = 0
    kd_trim = 0
    kp_boat = 0.02
    ki_boat = 0.0000001
    kd_boat = 0
    print('dsd')
<<<<<<< HEAD
    water_geometry, bottom_geometry = make_simple_water().make_water(adjust_rov, 1025, 500, 2, 30)
    controller = agxModel.WindAndWaterController()
    #for water_geometry in water_geometries:
    #    controller.addWater(water_geometry)
    controller.addWater(water_geometry)
    #controller.addObject(bottom_geometry)
=======
    water_geometry, bottom_geometry = MakeWater().make_water(adjust_rov, 1025, 500, 2, 30)
    controller = agxModel.WindAndWaterController()
    controller.addWater(water_geometry)

>>>>>>> Sophus

    """Creates a pid controller for depth"""

    pid = PID_Controller(kp, ki, kd, 0)
    pid.setName('pid')
    pid.set_output_limits(-45, 45)
    pid.set_mode(1, 0, 0)
    pid.set_setpoint(-11)

    """Creates a pid controller for trim"""

    pid_trim = PID_Controller(kp_trim, ki_trim, kd_trim, 0)
    pid_trim.setName('pidTrim')
    pid_trim.set_output_limits(-8, 8)
    pid_trim.set_mode(1, 0, 0)
    pid_trim.set_setpoint(0)

    keyboard = KeyboardListener(pid, plot)

    """Creates the rov"""
    rov = rovAssembly(pid, keyboard)
    rov.setPosition(agx.Vec3(-100, 0, 0))
    rov.setName("rov")
    rov.setRotation(agx.EulerAngles(0, 0, math.pi))

    setBodyViscousDrag(rov.link1,controller, 0.01)

    """Creates a pid controller for trim"""
    if not adjust_rov:

        pid_boat = PID_Controller(kp_boat, ki_boat, kd_boat, 0)
        pid_boat.setName('pidBoat')
        pid_boat.set_output_limits(-2, 2)
        pid_boat.set_mode(1, 0, 0)
        pid_boat.set_setpoint(50)

        """Creates the boat to tow the rov"""
        ship = Ship()
        ship.setName('ship')
        ship.setRotation(agx.EulerAngles(0,0,math.pi))

        wire, wire_renderer = MakeWire().create_wire(1020,0.005, ship, agx.Vec3(2, 0, 0),
                                                     rov, agx.Vec3(0,-0.181,0.185))


        demoutils.sim().add(wire)
        demoutils.sim().add(wire_renderer)
        demoutils.sim().add(keyboard)
        demoutils.sim().add(ship)
        demoutils.sim().add(pid_boat)
        demoutils.sim().add(Boat_Controller(ship, pid_boat))

    """Creates a controller to control the wings of the Rov"""
    wing_controll = RovController(rov)
    wing_controll.setName('wingControll')

    """Adds rov, boat, controller, and pid to the simulation"""
<<<<<<< HEAD

    demoutils.sim().add(bottom_geometry)
    agxOSG.createVisual(bottom_geometry, demoutils.root())
    demoutils.sim().add(water_geometry)

=======
    demoutils.sim().add(bottom_geometry)
    demoutils.sim().add(water_geometry)
>>>>>>> Sophus
    demoutils.sim().add(rov)
    demoutils.sim().add(pid)
    demoutils.sim().add(pid_trim)
    demoutils.sim().add(wing_controll)
    demoutils.sim().add(controller)
    createVisual(bottom_geometry, demoutils.root())


    lock = agx.LockJoint(rov.getRigidBody('rovBody'))
    lock.setCompliance(1e-2, agx.LockJoint.ALL_DOF)
    """Locks roll"""
    lock.setCompliance(1e-12, agx.LockJoint.ROTATIONAL_1)
    """Locks pitch"""
    # lock.setCompliance(1e-12, agx.LockJoint.ROTATIONAL_2)
    """locks yaw"""
    lock.setCompliance(1e-12, agx.LockJoint.ROTATIONAL_3)
    demoutils.sim().add(lock)
    demoutils.sim().setTimeStep(0.005)

    rov.displayForces(1)
    """locks the rov in fixed position, for mounting wing and cable to rov"""
    # lock1 = agx.LockJoint(rov.link1)
    # demoutils.sim().add(lock1)
    #
    # lock2 = agx.LockJoint(ship.m_body)
    # demoutils.sim().add(lock2)
