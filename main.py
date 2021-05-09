# agx imports:
from agxOSG import createVisual
import demoutils
from demoutils import sim
from agxCollide import Geometry

# local imports:
from rov_simulation_parameters import *
from utils import *
from make_water import MakeWater
from make_wire import MakeWire
from Rov.arduino_sensor import ArduinoSensor
from Rov.arduino_stepper import ArduinoStepper
from Boat.ship import Ship
from Boat.boat_controller import Boat_Controller
from Boat.Boat_sensor import Boat_Sensor
from Rov.rov_assembly import RovAssembly
from Rov.rov_controller import RovController

# python imports
from keyboard_listener import KeyboardListener
import math

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
    rov = RovAssembly(keyboard)
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
    arduino_sensor = ArduinoSensor(rov, seafloor.getShape().asHeightField())
    arduino_stepper = ArduinoStepper(rov_pid, pid_trim, rov)

    pid_boat = build_pid_controller(p=BOAT_K_P, i=BOAT_K_I, d=BOAT_K_D, direction=0, name="pidBoat", max_out=2,
                                    min_out=-2, setpoint=BOAT_SPEED, mode=(1, 0, 0))
    print("buildt pid for boat")
    """Creates the boat to tow the rov"""
    ship = Ship(controller)
    ship.setName('ship')
    ship.setRotation(agx.EulerAngles(0, 0, math.pi))
    ship.setPosition(agx.Vec3(-WATER_LENGTH + 20 + WIRE_LENGTH, 0, 0))
    print(ship.getPosition())
    ship_echo = Boat_Sensor(ship, seafloor.getShape().asHeightField())
    print("buildt ship")
    wire, wire_renderer = MakeWire().create_wire(1030, 0.001, ship, agx.Vec3(2, 0, 0),
                                                 rov, agx.Vec3(*WIRE_POS_ROV))
    print("buildt wire")
    setWireViscousDrag(wire, controller)
    ship.setVelocity(agx.Vec3(-2, 0, 0))
    print("buildt controll for wire")

    """Creates a controller to control the wings of the Rov"""
    wing_controll = RovController(rov, rov_pid, pid_trim, seafloor.getShape().asHeightField())
    wing_controll.setName('wingControll')
    print("buildt wing control")

    # rov.ehco_lod.beam.setEnableCollisions(water_geometry, False)
    # print("set echolod and water collisions to false")

    """Adds rov, boat, controller, and pid to the simulation"""
    sim().add(arduino_sensor)
    sim().add(arduino_stepper)
    sim().add(wire)
    print("added wire")
    sim().add(wire_renderer)
    print("added wire rederer")
    sim().add(keyboard)
    print("added keyboard listener")
    sim().add(ship)
    print("added ship")
    sim().add(Boat_Controller(ship, pid_boat, arduino_stepper))
    print("added ship controller")
    sim().add(seafloor)
    print("added ship seafloor")
    sim().add(water_geometry)
    print("added ship water geometry")
    sim().add(controller)
    print("added water controller")
    sim().add(rov)
    print("added rov")
    sim().addEventListener(ship_echo)
    print("added ship sensor")
    sim().add(rov_pid)
    print("added ship rov pid")
    # sim().add(pid_trim)
    # print("added ship rov trim pid")
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
    cam_pos = agx.Vec3(r_p[0] + 30, r_p[1] + 60, r_p[2] + 20)
    demoutils.init_camera(eye=cam_pos, center=r_p)
    v =0
    m = 0
    for geo in rov.link1.getGeometries():
        v += geo.calculateVolume() * 1027
        m += geo.calculateMass()
        print(m,v,m-v)
    """locks the rov in fixed position, for mounting wing and cable to rov"""
    if adjust_rov:
        lock1 = agx.LockJoint(rov.link1)
        sim().add(lock1)
