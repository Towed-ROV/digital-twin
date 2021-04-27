import agxModel

import demoutils
import agx
from agxOSG import createVisual
from Boat.ship import Ship
from Boat.boat_controller import Boat_Controller
from Rov.rov_assembly import rovAssembly
from Rov.rov_controller import RovController
from keyboard_listener import KeyboardListener
from make_water import MakeWater
import agxCollide
from make_wire import MakeWire
from pid import PID_Controller
from Rov.arduino_sensor import ArduinoSensor
from Rov.arduino_stepper import ArduinoStepper
from threading import Thread
from Sensor import Sensor
import matplotlib as plt
import math


def setBodyWaterParameters(controller: agxModel.WindAndWaterController, body: agx.RigidBody,
                           lift: float = 0.01, viscous_drag: float = 0.1, pressure_drag: float = 0.6):
    agxModel.WindAndWaterParameters.setHydrodynamicCoefficient(controller, body,
                                                               agxModel.WindAndWaterParameters.VISCOUS_DRAG,
                                                               viscous_drag)
    agxModel.WindAndWaterParameters.setHydrodynamicCoefficient(controller, body,
                                                               agxModel.WindAndWaterParameters.LIFT,
                                                               lift)
    agxModel.WindAndWaterParameters.setHydrodynamicCoefficient(controller, body,
                                                               agxModel.WindAndWaterParameters.PRESSURE_DRAG,
                                                               pressure_drag)


def setBodyViscousDrag(body, controller: agxModel.WindAndWaterController):
    bodies = body.getRigidBodies()
    for rigid_bodies in bodies:
        if "wing" in rigid_bodies.getName().lower():
            setBodyWaterParameters(controller, rigid_bodies, pressure_drag=2)
            print("wing")
        else:
            setBodyWaterParameters(controller, rigid_bodies, pressure_drag=1)


def setWireViscousDrag(wire, controller):
    for geo in wire.nodes:
        rig = geo.getRigidBody()
        setBodyWaterParameters(controller, rig, lift=0)


def decorator(decerator, sim):
    rov_body = sim.getRigidBody("rov_body")
    if rov_body:
        while True:
            decerator.setText(3, "depth : {} M".format(str(round(-rov_body.getPosition[2], 2))))
            decerator.setText(7, "distance : {} M".format(str(round(rov_body.getPosition()[0], 2))))
            decerator.setText(4, "Pitch : {}".format(str(round(rov_body.getRotation()[0] * 100, 2))))
            decerator.setText(5, "Roll : {}".format(str(round(rov_body.getRotation()[1] * 100, 2))))
    else:
        rov_body = sim.getRigidBody("rov_body")


def buildScene():
    """Building scene for simulation and adding it to the simulation. Also setting the simulation step"""""
    demoutils.sim().setTimeStep(0.02)
    build_scene()


"""Building scene"""


def build_scene():
    wingscale = 1.5

    """write plot to csv file variable"""
    start = False
    plot = False
    adjust_rov = False
    kp = 10
    ki = 1
    kd = 5
    kp_trim = 0.02
    ki_trim = 0.02
    kd_trim = 1
    kp_boat = 0.02
    ki_boat = 0.0000001
    kd_boat = 0
    print('dsd')
    depth = 60
    length = 520
    width = 30
    density = 1027
    water_geometry, bottom_geometry = MakeWater().make_water(adjust_rov, density, length, width, depth)
    controller = agxModel.WindAndWaterController()
    controller.addWater(water_geometry)

    """Creates a pid controller for depth"""

    pid = PID_Controller(kp, ki, kd, 1)
    pid.setName('pid')
    pid.set_output_limits(-45, 45)
    pid.set_mode(1, 0, 0)
    pid.set_setpoint(-6)

    """Creates a pid controller for trim"""

    pid_trim = PID_Controller(kp_trim, ki_trim, kd_trim, 0)
    pid_trim.setName('pidTrim')
    pid_trim.set_output_limits(-8, 8)
    pid_trim.set_mode(1, 0, 0)
    pid_trim.set_setpoint(0)

    keyboard = KeyboardListener(pid, plot)

    """Creates the rov"""
    rov = rovAssembly(keyboard, wing_scale=wingscale,depth=depth,seaFloor=bottom_geometry)
    rov.setPosition(agx.Vec3(-length + 10, 0, 0))
    rov.setName("rov")
    rov.setRotation(agx.EulerAngles(0, 0, math.pi))
    setBodyViscousDrag(rov, controller)
    """Creates a pid controller for trim"""
    if not adjust_rov:
        arduino_sensor = ArduinoSensor(rov)
        arduino_stepper = ArduinoStepper(pid, pid_trim, rov)

        pid_boat = PID_Controller(kp_boat, ki_boat, kd_boat, 0)
        pid_boat.setName('pidBoat')
        pid_boat.set_output_limits(-2, 2)
        pid_boat.set_mode(1, 0, 0)
        pid_boat.set_setpoint(10)

        """Creates the boat to tow the rov"""
        ship = Ship()
        ship.setName('ship')
        ship.setRotation(agx.EulerAngles(0, 0, math.pi))
        ship.setPosition(agx.Vec3(-length + 20 + 300, 0, 0))
        wire, wire_renderer = MakeWire().create_wire(1030, 0.001, ship, agx.Vec3(2, 0, 0),
                                                     rov, agx.Vec3(0, -0.1 * 0, 0.1 * 0))
        setWireViscousDrag(wire, controller)

        ship.setVelocity(agx.Vec3(-50, 0, 0))
        # demoutils.sim().add(arduino_sensor)
        # demoutils.sim().add(arduino_stepper)
        demoutils.sim().add(wire)
        demoutils.sim().add(wire_renderer)
        demoutils.sim().add(keyboard)
        demoutils.sim().add(ship)
        demoutils.sim().add(pid_boat)
        demoutils.sim().add(Boat_Controller(ship, pid_boat, arduino_stepper))
    """Creates a controller to control the wings of the Rov"""
    wing_controll = RovController(rov, pid, pid_trim)
    wing_controll.setName('wingControll')

    """Adds rov, boat, controller, and pid to the simulation"""
    demoutils.sim().add(bottom_geometry)
    demoutils.sim().add(water_geometry)
    demoutils.sim().add(rov)
    demoutils.sim().add(pid)
    demoutils.sim().add(pid_trim)
    demoutils.sim().add(wing_controll)
    demoutils.sim().add(controller)
    demoutils.sim().add(rov.ehco_lod.beam)

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
    "hold simulator untill start"
    # if not start:
    #     lock2 = agx.LockJoint(ship.m_body)
    #     demoutils.sim().add(lock2)
    #     lock1 = agx.LockJoint(rov.link1)
    #     demoutils.sim().add(lock1)

    rov.displayForces(1)

    t = Thread(decorator(decerator=demoutils.app().getSceneDecorator(), sim=demoutils.sim()))
    t.daemon = True
    t.start()

    """locks the rov in fixed position, for mounting wing and cable to rov"""
    if adjust_rov:
        lock1 = agx.LockJoint(rov.link1)
        demoutils.sim().add(lock1)
