import agx
import agxModel
from agxOSG import createVisual

import demoutils
from demoutils import sim
from rov_simulation_parameters import *
from make_water import MakeWater
from make_wire import MakeWire
from pid import PID_Controller
from Rov.arduino_sensor import ArduinoSensor
from Rov.arduino_stepper import ArduinoStepper
from Boat.ship import Ship
from Boat.boat_controller import Boat_Controller
from Rov.rov_assembly import RovAssembly
from Rov.rov_controller import RovController

from keyboard_listener import KeyboardListener
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
            setBodyWaterParameters(controller, rigid_bodies, pressure_drag=WING_PDRAG,lift=WING_LIFT,viscous_drag=WING_VDRAG)
            print("wing")
        else:
            setBodyWaterParameters(controller, rigid_bodies, pressure_drag=BODY_PDRAG,lift=BODY_LIFT,viscous_drag=BODY_VDRAG)


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


def build_pid_controller(p, i, d, mode, setpoint, direction, name, max_out=None, min_out=None):
    pid = PID_Controller(p, i, d, direction)
    pid.setName(name)
    pid.set_mode(*mode)
    pid.set_setpoint(setpoint)
    if max and min:
        pid.set_output_limits(max_out, min_out)
    return pid


def buildScene():
    """Building scene for simulation and adding it to the simulation. Also setting the simulation step"""""
    sim().setTimeStep(SIM_TIME_STEP)
    build_scene()


"""Building scene"""


def build_scene():
    """write plot to csv file variable"""
    print("build staring")

    water_geometry, seafloor = MakeWater.make_water(WATER_DENSITY, WATER_LENGTH, WATER_WIDTH, WATER_DEPTH)
    controller = agxModel.WindAndWaterController()
    controller.addWater(water_geometry)
    print("buildt water controller")

    """Creates a pid controller for depth"""

    rov_pid = build_pid_controller(p=ROV_K_P, i=ROV_K_I, d=ROV_K_D, mode=(1, 0, 0), setpoint=ROV_DEPTH_SETPOINT,
                                   direction=1, max_out=MAX_WING_ANGLE, min_out=MIN_WING_ANGLE, name="rov_pid")
    """Creates a pid controller for trim"""

    pid_trim = build_pid_controller(p=K_P_TRIM, i=K_I_TRIM, d=K_D_TRIM, direction=0, mode=(1, 0, 0),
                                    max_out=TRIM_MAX_OUT, min_out=TRIM_MIN_OUT, name="pidTrim", setpoint=0)
    print("buildt pid's for rov")
    keyboard = KeyboardListener(rov_pid, plot)

    """Creates the rov"""
    rov = RovAssembly(keyboard, seafloor=seafloor)
    print("base rov ready")

    rov.setPosition(agx.Vec3(-WATER_LENGTH + 10, 0, 0))
    rov.setName("rov")
    rov.setRotation(agx.EulerAngles(0, 0, math.pi))
    setBodyViscousDrag(rov, controller)
    print("buildt rov")

    lock = agx.LockJoint(rov.getRigidBody('rovBody'))
    lock.setCompliance(1e-2, agx.LockJoint.ALL_DOF)
    """Locks roll"""
    lock.setCompliance(1e-12, agx.LockJoint.ROTATIONAL_1)
    """Locks pitch"""
    # lock.setCompliance(1e-12, agx.LockJoint.ROTATIONAL_2)
    """locks yaw"""
    lock.setCompliance(1e-12, agx.LockJoint.ROTATIONAL_3)
    print("buildt lock for rov")

    """Creates a pid controller for trim"""
    arduino_sensor = ArduinoSensor(rov)
    arduino_stepper = ArduinoStepper(rov_pid, pid_trim, rov)

    pid_boat = build_pid_controller(p=BOAT_K_P, i=BOAT_K_I, d=BOAT_K_D, direction=0, name="pidBoat", max_out=2,
                                    min_out=-2, setpoint=BOAT_SPEED, mode=(1, 0, 0))
    print("buildt pid for boat")
    """Creates the boat to tow the rov"""
    ship = Ship()
    ship.setName('ship')
    ship.setRotation(agx.EulerAngles(0, 0, math.pi))
    ship.setPosition(agx.Vec3(-WATER_LENGTH + 20 + WIRE_LENGTH, 0, 0))
    print(ship.getPosition())
    print("buildt ship")
    wire, wire_renderer = MakeWire().create_wire(1030, 0.001, ship, agx.Vec3(2, 0, 0),
                                                 rov, agx.Vec3(-0.1, 0, 0.1))
    print("buildt wire")
    setWireViscousDrag(wire, controller)
    ship.setVelocity(agx.Vec3(-1, 0, 0))
    print("buildt controll for wire")

    """Creates a controller to control the wings of the Rov"""
    wing_controll = RovController(rov, rov_pid, pid_trim, seafloor.getShape().asHeightField())
    wing_controll.setName('wingControll')
    print("buildt wing control")

    # rov.ehco_lod.beam.setEnableCollisions(water_geometry, False)
    # print("set echolod and water collisions to false")

    """Adds rov, boat, controller, and pid to the simulation"""
    # sim().add(arduino_sensor)
    # sim().add(arduino_stepper)
    sim().add(wire)
    print("added wire")
    sim().add(wire_renderer)
    print("added wire rederer")
    sim().add(keyboard)
    print("added keyboard listener")
    sim().add(ship)
    print("added ship")
    #sim().add(pid_boat)
    #print("added ship pid")
    sim().add(Boat_Controller(ship, pid_boat, arduino_stepper))
    print("added ship controller")
    sim().add(seafloor)
    print("added ship seafloor")
    sim().add(water_geometry)
    print("added ship water geometry")
    sim().add(controller)
    print("added water controller")
    sim().add(rov)
    print("added ship rov")
    #sim().add(rov_pid)
    #print("added ship rov pid")
    #sim().add(pid_trim)
    #print("added ship rov trim pid")
    sim().add(wing_controll)
    print("added wing controller")
    sim().add(lock)
    print("added lock for rov")
    # sim().addEventListener(rov.ehco_lod)
    # print("added echolod event listener")
    createVisual(seafloor, demoutils.root())
    print("created visuals")
    sim().setTimeStep(SIM_TIME_STEP)
    r_p = rov.getPosition()
    cam_pos = agx.Vec3(r_p[0]+30,r_p[1]+60,r_p[2]+20)
    demoutils.init_camera(eye=cam_pos,center=rov.getPosition())

    """locks the rov in fixed position, for mounting wing and cable to rov"""
    if adjust_rov:
        lock1 = agx.LockJoint(rov.link1)
        sim().add(lock1)
