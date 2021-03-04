import agx

def createConstraint(**kwargs):
    """Create constraint given type, rb1, local/world axis and local/world position.

    Examples:
        hinge = createConstraint( type = agx.Hinge,
                                  rb1 = firstRigidBody,
                                  rb2 = otherRigidBody,
                                  axis = axisRelRb1,
                                  point = pointRelRb1 )
    
    Arguments:
        type: agx.Constraint -- Constraint type.
        rb1: agx.RigidBody -- First rigid body (not None)
        rb2: agx.RigidBody -- Second rigid body (world if not given or None)
        axis: agx.Vec3 -- Axis given in rb1 frame (either localAxis or worldAxis must be given).
        worldAxis: agx.Vec3 -- Axis given in world frame (either localAxis or worldAxis must be given).
        point: agx.Vec3 -- Point given in rb1 frame (either localPoint or worldPoint must be given).
        worldPoint: agx.Vec3 -- Point given in world frame (either localPoint or worldPoint must be given).
        position: agx.Vec3 -- See point.
        worldPosition: agx.Vec3 -- See worldPoint.
    
    Returns:
        [agx.Constraint] -- Constraint of given type if successful - otherwise None.
    """

    rb1 = kwargs[ 'rb1' ]
    if rb1 is None:
        print( 'Unable to create constraint - rb1 not given or None' )
        return None

    rb2 = kwargs.get( 'rb2', None )

    worldAxis = kwargs.get( 'worldAxis', agx.Vec3.Z_AXIS() )
    worldPoint = kwargs.get( 'worldPoint', agx.Vec3() ) if 'worldPoint' in kwargs else kwargs.get( 'worldPosition', agx.Vec3() )
    if 'axis' in kwargs:
        worldAxis = rb1.getFrame().transformVectorToWorld( kwargs['axis'] )
    if 'point' in kwargs:
        worldPoint = rb1.getFrame().transformPointToWorld( kwargs['point' ] )
    elif 'position' in kwargs:
        worldPoint = rb1.getFrame().transformPointToWorld( kwargs['position' ] )

    f1 = agx.Frame()
    f2 = agx.Frame()
    if not agx.Constraint.calculateFramesFromWorld( worldPoint, worldAxis, rb1, f1, rb2, f2 ):
        print( 'Unable to create constraint - calculateFramesFromWorld failed to initialize constraint frames.' )
        return None
    
    return kwargs[ 'type' ]( rb1, f1, rb2, f2 )

def constraint_type(t):
    def decorator(function):
        def decorate(**kwargs):
            kwargs.update( { 'type': t } )
            return function(**kwargs)
        return decorate
    return decorator

@constraint_type(agx.Hinge)
def createHinge(**kwargs) -> agx.Hinge:
    return createConstraint( **kwargs )

@constraint_type(agx.Prismatic)
def createPrismatic(**kwargs) -> agx.Prismatic:
    return createConstraint( **kwargs )

@constraint_type(agx.CylindricalJoint)
def createCylindricalJoint(**kwargs) -> agx.CylindricalJoint:
    return createConstraint( **kwargs )

@constraint_type(agx.LockJoint)
def createLockJoint(**kwargs) -> agx.LockJoint:
    return createConstraint( **kwargs )

@constraint_type(agx.AngularLockJoint)
def createAngularLockJoint(**kwargs) -> agx.AngularLockJoint:
    return createConstraint( **kwargs )

@constraint_type(agx.BallJoint)
def createBallJoint(**kwargs) -> agx.BallJoint:
    return createConstraint( **kwargs )

@constraint_type(agx.DistanceJoint)
def createDistanceJoint(**kwargs) -> agx.DistanceJoint:
    return createConstraint( **kwargs )
