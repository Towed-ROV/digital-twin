import agx, agxModel
from rov_simulation_parameters import WING_LIFT, WING_PDRAG, WING_SCALE, WING_VDRAG, BODY_LIFT, BODY_PDRAG, BODY_VDRAG, \
    WATER_DENSITY
from pid import PID_Controller
from demoutils import sim, init_camera


def setBodyWaterParameters(controller: agxModel.WindAndWaterController, body: agx.RigidBody,
                           lift: float = 0.01, viscous_drag: float = 0.1, pressure_drag: float = 0.6):
    """
    sets the water parameters of a rigid body.
    Args:
        controller: the water around the body
        body: the body
        lift: new lift
        viscous_drag: new v drag
        pressure_drag: new p drag

    Returns:None

    """
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
    """
    sets the body drag for the wings and the body seperately.
    Args:
        body: the rov
        controller: the water aroun the rov
    Returns:
    """
    bodies = body.getRigidBodies()
    for rigid_bodies in bodies:
        if "wing" in rigid_bodies.getName().lower():
            setBodyWaterParameters(controller, rigid_bodies, pressure_drag=WING_PDRAG, lift=WING_LIFT,
                                   viscous_drag=WING_VDRAG)
            print("wing")
        else:
            setBodyWaterParameters(controller, rigid_bodies, pressure_drag=BODY_PDRAG, lift=BODY_LIFT,
                                   viscous_drag=BODY_VDRAG)


def setWireViscousDrag(wire, controller):
    """
    sets water parameters for a wire.
    Args:
        wire: the wire
        controller:  the water around the wire

    Returns:

    """
    for geo in wire.nodes:
        rig = geo.getRigidBody()
        setBodyWaterParameters(controller, rig, lift=0)


def decorator(decerator, sim):
    """
    ads text to the main window.
    Args:
        decerator: the window decorator
        sim:  the simulation

    Returns: None

    """
    rov_body = sim.getRigidBody("rov_body")
    if rov_body:
        while True:
            decerator.setText(2, "depth : {} M".format(str(round(-rov_body.getPosition[2], 2))))
            decerator.setText(3, "distance : {} M".format(str(round(rov_body.getPosition()[0], 2))))
            decerator.setText(4, "Pitch : {}".format(str(round(rov_body.getRotation()[0] * 100, 2))))
            decerator.setText(5, "Roll : {}".format(str(round(rov_body.getRotation()[1] * 100, 2))))
            decerator.setText(10, "wing : L:{} || R:{}".format(str(round(rov_body.hinge1.getAngle(), 2)),
                                                               str(round(rov_body.hinge2.getAngle(), 2))))


def build_pid_controller(p, i, d, mode, setpoint, direction, name, max_out=None, min_out=None) -> PID_Controller:
    """
    builds a pid controller
    Args:
        p:
        i:
        d:
        mode:
        setpoint:
        direction:
        name:
        max_out:
        min_out:

    Returns:the pid Controller

    """
    pid = PID_Controller(p, i, d, direction)
    pid.setName(name)
    pid.set_mode(*mode)
    pid.set_setpoint(setpoint)
    if max is not None and min is not None:
        pid.set_output_limits(limit_max=max_out, limit_min=min_out)
    return pid


def add_objects_to_sim(*args):
    """
    adds all objects provided to the simulation.
    Args:
        *args:

    Returns:

    """
    for obj in args:
        sim().add(obj)


def set_camera_to_rov(rov):
    """
    sets the position of the camera to the rov
    Args:
        rov:

    Returns:

    """
    r_p = rov.getPosition()
    cam_pos = agx.Vec3(r_p[0] + 5, r_p[1] + 10, r_p[2] + 2)
    init_camera(eye=cam_pos, center=r_p)


def print_builancy(rigid_body):
    """
    gets the weight of a body compared to that of water fillig the same volume
    Args:
        rigid_body:

    Returns:

    """
    v = 0
    m = 0
    for geo in rigid_body.getGeometries():
        v += geo.calculateVolume() * WATER_DENSITY
        m += geo.calculateMass()
        print(m, v, m - v)


def build_angular_lockJoint(body, dof, roll, pitch, yaw):
    """
    builds a lock for a body to the background.
    Args:
        body: body
        dof: constraints in dof
        roll:  roll constraint
        pitch: pitch constraint
        yaw: yaw constraint

    Returns: the lock.

    """
    lock = agx.AngularLockJoint(body)
    lock.setCompliance(dof, agx.LockJoint.ALL_DOF)
    """Locks roll"""
    lock.setCompliance(roll, agx.LockJoint.ROTATIONAL_1)
    """Locks pitch"""
    lock.setCompliance(pitch, agx.LockJoint.ROTATIONAL_2)
    """locks yaw"""
    lock.setCompliance(yaw, agx.LockJoint.ROTATIONAL_3)
    return lock
