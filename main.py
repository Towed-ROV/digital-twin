# agx imports:
from agxOSG import createVisual
import demoutils
from demoutils import sim

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

# python imports
from keyboard_listener import KeyboardListener
import math

"""Building scene"""


def build_scene():
    """
    builds the simulation, is run by agx internaly.
    Returns:

    """
    print("build staring")

    """ Ads Water and seafloor to the simulation"""
    water_geometry, seafloor = MakeWater.make_water(WATER_DENSITY, WATER_LENGTH, WATER_WIDTH, WATER_DEPTH)
    controller = agxModel.WindAndWaterController()
    controller.addWater(water_geometry)
    print("buildt water controller")

    """Creates a pid controller for depth and trim"""
    rov_pid = build_pid_controller(p=ROV_K_P, i=ROV_K_I, d=ROV_K_D, mode=(1, 0, 0), setpoint=ROV_DEPTH_SETPOINT,
                                   direction=1, max_out=MAX_WING_ANGLE, min_out=MIN_WING_ANGLE, name="rov_pid")
    pid_trim = build_pid_controller(p=K_P_TRIM, i=K_I_TRIM, d=K_D_TRIM, direction=0, mode=(1, 0, 0),
                                    max_out=TRIM_MAX_OUT, min_out=TRIM_MIN_OUT, name="pidTrim", setpoint=0)

    keyboard = KeyboardListener(rov_pid, plot)
    print("buildt pid's for rov")

    """Creates the rov"""
    rov = RovAssembly(keyboard)
    rov.setPosition(agx.Vec3(-WATER_LENGTH + 10, 0, 0))
    rov.setName("rov")
    rov.setRotation(agx.EulerAngles(0, 0, math.pi))
    setBodyViscousDrag(rov, controller)
    print("buildt rov")

    """builds a lock for the system to stablilize it the lock is soft so that the rov can move but it takes more 
        to move  it"""
    lock = build_angular_lockJoint(rov.getRigidBody('rovBody'), 1e-2, 1e-12, 1e-2, 1e-12)
    print("buildt lock for rov")

    """ creates arduino simulations"""
    arduino_sensor = ArduinoSensor(rov, seafloor.getShape().asHeightField())
    arduino_stepper = ArduinoStepper(rov_pid, pid_trim, rov)
    print("buildt pid for boat")
    pid_boat = build_pid_controller(p=BOAT_K_P, i=BOAT_K_I, d=BOAT_K_D, direction=0, name="pidBoat", max_out=2,
                                    min_out=-2, setpoint=BOAT_SPEED, mode=(1, 0, 0))

    """Creates the boat to tow the rov"""
    ship = Ship(controller)
    ship.setName('ship')
    ship.setRotation(agx.EulerAngles(0, 0, math.pi))
    ship.setPosition(agx.Vec3(-WATER_LENGTH + 20 + WIRE_LENGTH, 0, 0))
    ship_echo = Boat_Sensor(ship, seafloor.getShape().asHeightField())
    ship.setVelocity(agx.Vec3(-1, 0, 0))
    ship_controller = Boat_Controller(ship, pid_boat, arduino_stepper)
    print("buildt ship")

    """builds the wire and wire controller"""
    wire, wire_renderer = MakeWire().create_wire(1030, 0.001, ship, agx.Vec3(2, 0, 0), rov, agx.Vec3(*WIRE_POS_ROV))
    setWireViscousDrag(wire, controller)
    print("build Wire")

    """Adds rov, boat, controller, and pid to the simulation"""
    add_objects_to_sim(arduino_sensor, arduino_stepper, wire, wire_renderer)
    print("added sensor, stepper, wire and wire rederer")
    add_objects_to_sim(ship, ship_controller, seafloor, water_geometry, controller)
    print("added ship and ship controller, seafloor, water geometry and controller")
    add_objects_to_sim(rov, rov_pid, lock)
    print("added rov and rov_pid and lock")
    sim().addEventListener(ship_echo)
    print("added ship sensor")
    sim().addEventListener(keyboard)
    print("added keyboard listener")
    sim().setTimeStep(SIM_TIME_STEP)
    set_camera_to_rov(rov)
    print_builancy(rov.link1)

    """locks the rov in fixed position, for mounting wing and cable to rov"""
    if adjust_rov:
        lock1 = agx.LockJoint(rov.link1)
        sim().add(lock1)
