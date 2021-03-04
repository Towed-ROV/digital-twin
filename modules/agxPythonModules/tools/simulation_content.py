import agx
import agxCollide
import agxSDK
import agxWire
import agxCable
import agxTerrain

_has_agxTerrain = True
try:
    agxTerrain.Terrain.find( None, '' )
except AttributeError:
    _has_agxTerrain = False

from collections import OrderedDict

class ShapeInfo:
    def __init__(self, shape: agxCollide.Shape):
        self.instance  = shape
        self.type      = shape.getType() if shape else -1
        self.type_name = shape.getTypeName() if shape else 'None'
        self.center    = shape.getCenter() if shape else agx.Vec3()

        if shape is None:
            pass
        elif shape.asBox():
            self.half_extents = shape.asBox().getHalfExtents()
        elif shape.asCapsule():
            self.radius = shape.asCapsule().getRadius()
            self.height = shape.asCapsule().getHeight()
        elif shape.asCylinder():
            self.radius = shape.asCylinder().getRadius()
            self.height = shape.asCylinder().getHeight()
        elif shape.asPlane():
            self.normal   = shape.asPlane().getNormal()
            self.distance = shape.asPlane().getDistance()
        elif shape.asSphere():
            self.radius = shape.asSphere().getRadius()
        elif shape.asMesh():
            self.num_triangles       = shape.asMesh().getNumTriangles()
            self.num_vertices        = shape.asMesh().getNumVertices()
            self.is_valid_and_closed = shape.asMesh().isValidAndClosed()
            self.has_half_edge       = shape.asMesh().hasHalfEdge()
    
    def print(self, indent: int = 0):
        print( ' ' * indent + '+ Type:          {}'.format( self.type_name ) )
        print( ' ' * indent + '+ Center:        [{:.16f}, {:.16f}, {:.16f}]'.format( self.center.x(),
                                                                                     self.center.y(),
                                                                                     self.center.z() ) )
        if self.type == agxCollide.Shape.BOX:
            print( ' ' * indent + '+ Half extents:  [{:.16f}, {:.16f}, {:.16f}]'.format( self.half_extents.x(),
                                                                                         self.half_extents.y(),
                                                                                         self.half_extents.z() ) )
        elif self.type == agxCollide.Shape.CAPSULE or\
             self.type == agxCollide.Shape.CYLINDER or\
             self.type == agxCollide.Shape.WIRE_SHAPE:
            print( ' ' * indent + '+ Radius:        {:.16f}'.format( self.radius ) )
            print( ' ' * indent + '+ Height:        {:.16f}'.format( self.height ) )
        elif self.type == agxCollide.Shape.SPHERE:
            print( ' ' * indent + '+ Radius:        {:.16f}'.format( self.radius ) )
        elif self.type == agxCollide.Shape.TRIMESH or\
             self.type == agxCollide.Shape.CONVEX or\
             self.type == agxCollide.Shape.HEIGHT_FIELD:
            print( ' ' * indent + '+ # triangles:   {}'.format( self.num_triangles ) )
            print( ' ' * indent + '+ # vertices:    {}'.format( self.num_vertices ) )
            print( ' ' * indent + '+ is valid:      {}'.format( self.is_valid_and_closed ) )
            print( ' ' * indent + '+ has half edge: {}'.format( self.has_half_edge ) )
        

class GeometryInfo:
    def __init__(self, geometry: agxCollide.Geometry):
        self.instance           = geometry
        self.name               = geometry.getName() if geometry else '[None]'
        self.id                 = geometry.getId() if geometry else -1
        self.material           = geometry.getMaterial().getName() if geometry and geometry.getMaterial() else '[Default or None]'
        self.collisions_enabled = geometry.getEnableCollisions() if geometry else False
        self.shapes             = []
        if geometry is None:
            return
        for shape in geometry.getShapes():
            self.shapes.append( ShapeInfo( shape ) )
    
    def print(self, indent: int = 0):
        print( ' ' * indent + '* Name:       {}'.format( self.name ) )
        print( ' ' * indent + '* Id:         {}'.format( self.id ) )
        print( ' ' * indent + '* Material:   {}'.format( self.material ) )
        print( ' ' * indent + '* Collisions: {}'.format( 'enabled' if self.collisions_enabled else 'disabled' ) )
        print( ' ' * indent + '* # shapes:   {}'.format( len( self.shapes ) ) )
        shape_index = 0
        for si in self.shapes:
            print( ' ' * (indent + 4) + '+ Shape[{}] +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++'.format( shape_index ) )
            si.print( indent + 4 )
            shape_index += 1
            if shape_index == len( self.shapes ):
                print( ' ' * (indent + 4) + '+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++' )

class RigidBodyInfo:
    def __init__(self, rb: agx.RigidBody):
        self.instance                 = rb
        self.name                     = rb.getName() if rb else '[None]'
        self.id                       = rb.getId() if rb else -1
        self.motion_control           = rb.getMotionControl() if rb else agx.RigidBody.STATIC
        self.mass                     = rb.getMassProperties().getMass() if rb else 0.0
        self.inertia                  = rb.getMassProperties().getPrincipalInertiae() if rb else agx.Vec3()
        self.cm_offset                = rb.getCmFrame().getLocalTranslate() if rb else agx.Vec3()
        self.linear_velocity          = rb.getVelocity() if rb else agx.Vec3()
        self.angular_velocity         = rb.getAngularVelocity() if rb else agx.Vec3()
        self.linear_velocity_damping  = rb.getLinearVelocityDamping() if rb else agx.Vec3f()
        self.angular_velocity_damping = rb.getAngularVelocityDamping() if rb else agx.Vec3f()
        self.force                    = rb.getForce() if rb else agx.Vec3()
        self.torque                   = rb.getTorque() if rb else agx.Vec3()
        self.world_position           = rb.getPosition() if rb else agx.Vec3()
        self.world_rotation           = rb.getRotation() if rb else agx.Quat()
        self.geometries               = []
        if rb is None:
            return
        for geometry in rb.getGeometries():
            self.geometries.append( GeometryInfo( geometry ) )
    
    def print(self, indent: int = 0):
        print( ' ' * indent + '- Name:                     {}'.format( self.name ) )
        print( ' ' * indent + '- Id:                       {}'.format( self.id ) )
        print( ' ' * indent + '- Motion control:           {}'.format( 'DYNAMICS' if self.motion_control == agx.RigidBody.DYNAMICS else\
                                                                       'KINEMATICS' if self.motion_control == agx.RigidBody.KINEMATICS else\
                                                                       'STATIC' ) )
        print( ' ' * indent + '- Position:                 [{:.16f}, {:.16f}, {:.16f}]'.format( self.world_position.x(),
                                                                                                self.world_position.y(),
                                                                                                self.world_position.z() ) )
        print( ' ' * indent + '- Rotation:                 [{:.16f}, {:.16f}, {:.16f}, {:.16f}]'.format( self.world_rotation.x(),
                                                                                                         self.world_rotation.y(),
                                                                                                         self.world_rotation.z(),
                                                                                                         self.world_rotation.w() ) )
        print( ' ' * indent + '- Mass:                     {:.16f}'.format( self.mass ) )
        print( ' ' * indent + '- Inertia diagonal:         [{:.16f}, {:.16f}, {:.16f}]'.format( self.inertia.x(),
                                                                                                self.inertia.y(),
                                                                                                self.inertia.z() ) )
        print( ' ' * indent + '- CM offset:                [{:.16f}, {:.16f}, {:.16f}]'.format( self.cm_offset.x(),
                                                                                                self.cm_offset.y(),
                                                                                                self.cm_offset.z() ) )
        print( ' ' * indent + '- Linear velocity:          [{:.16f}, {:.16f}, {:.16f}]'.format( self.linear_velocity.x(),
                                                                                                self.linear_velocity.y(),
                                                                                                self.linear_velocity.z() ) )
        print( ' ' * indent + '- Angular velocity:         [{:.16f}, {:.16f}, {:.16f}]'.format( self.angular_velocity.x(),
                                                                                                self.angular_velocity.y(),
                                                                                                self.angular_velocity.z() ) )
        print( ' ' * indent + '- Linear velocity damping:  [{:.16f}, {:.16f}, {:.16f}]'.format( self.linear_velocity_damping.x(),
                                                                                                self.linear_velocity_damping.y(),
                                                                                                self.linear_velocity_damping.z() ) )
        print( ' ' * indent + '- Angular velocity damping: [{:.16f}, {:.16f}, {:.16f}]'.format( self.angular_velocity_damping.x(),
                                                                                                self.angular_velocity_damping.y(),
                                                                                                self.angular_velocity_damping.z() ) )
        print( ' ' * indent + '- Force:                    [{:.16f}, {:.16f}, {:.16f}]'.format( self.force.x(),
                                                                                                self.force.y(),
                                                                                                self.force.z() ) )
        print( ' ' * indent + '- Torque:                   [{:.16f}, {:.16f}, {:.16f}]'.format( self.torque.x(),
                                                                                                self.torque.y(),
                                                                                                self.torque.z() ) )
        print( ' ' * indent + '- Geometries:               {}'.format( len( self.geometries ) ) )
        geometry_index = 0
        for geometry in self.geometries:
            print( ' ' * (indent + 4) + '* Geometry[{}] **************************************************************'.format( geometry_index ) )
            geometry.print( indent + 4 )
            geometry_index += 1
            if geometry_index == len( self.geometries ):
                print( ' ' * (indent + 4) + '*****************************************************************************' )

class ConstraintControllerInfo:
    @classmethod
    def cast(cls, controller: agx.ElementaryConstraint):
        m = agx.Motor1D.safeCast( controller )
        if m: return m
        l = agx.LockController.safeCast( controller )
        if l: return l
        r = agx.RangeController.safeCast( controller )
        if r: return r
        e = agx.ElectricMotorController.safeCast( controller )
        if e: return e
        f = agx.FrictionController.safeCast( controller )
        if f: return f
        return controller
    
    @classmethod
    def value_to_str(cls, value):
        t = type( value )
        return '[{}, {}]'.format( value.lower(), value.upper() ) if t is agx.RangeReal else\
               '{}'.format( value )

    def __init__(self, c: agx.ElementaryConstraint):
        self.instance    = self.cast( c )
        self.name        = c.getName() if c else '[None]'
        self.type_name   = '{}::{}'.format( type( self.instance ).__module__,
                                            type( self.instance ).__name__ ) if c else '[None]'
        self.enabled     = c.getEnable() if c else False
        self.active      = c.isActive() if c else False
        self.compliance  = c.getCompliance() if c else 1
        self.damping     = c.getDamping() if c else 1
        self.force_range = c.getForceRange() if c else agx.RangeReal()
        if self.instance is None:
            return
        t = type( self.instance )
        if t is agx.RangeController:
            self.range = self.instance.getRange()
            self.additional_attributes = OrderedDict( [('Range', 'range')] )
        elif t is agx.Motor1D:
            self.target_speed = self.instance.getSpeed()
            self.lock_at_zero_speed = self.instance.getLockedAtZeroSpeed()
            self.additional_attributes = OrderedDict( [('Target speed', 'target_speed'),
                                                       ('Lock at zero speed', 'lock_at_zero_speed')] )
        elif t is agx.LockController:
            self.position = self.instance.getPosition()
            self.additional_attributes = OrderedDict( [('Position', 'position')] )
        elif t is agx.ElectricMotorController:
            self.voltage = self.instance.getVoltage()
            self.armature_resistance = self.instance.getArmatureResistance()
            self.torque_constant = self.instance.getTorqueConstant()
            self.additional_attributes = OrderedDict( [('Voltage', 'voltage'),
                                                       ('Armature resistance', 'armature_resistance'),
                                                       ('Torque constant', 'torque_constant')] )
        elif t is agx.FrictionController:
            self.friction_coefficient = self.instance.getFrictionCoefficient()
            self.non_linear = self.instance.getEnableNonLinearDirectSolveUpdate()
            self.additional_attributes = OrderedDict( [('Friction coefficient', 'friction_coefficient'),
                                                       ('Non-linear', 'non_linear')] )
    
    def print(self, indent: int = 0):
        print( ' ' * indent + '- ' + self.type_name + ' (name: {})'.format( self.name ) )
        indent += 4
        print( ' '* indent + '* Enabled:              {}'.format( self.enabled ) )
        print( ' '* indent + '* Active:               {}'.format( self.active ) )
        print( ' '* indent + '* Compliance:           {:E}'.format( self.compliance ) )
        print( ' '* indent + '* Damping:              {}'.format( self.damping ) )
        print( ' '* indent + '* Force range:          [{}, {}]'.format( self.force_range.lower(), self.force_range.upper() ) )
        if not hasattr( self, 'additional_attributes' ):
            return
        for name, attribute in self.additional_attributes.items():
            value = getattr( self, attribute )
            print( ' ' * indent + '* {}:{}{}'.format( name, ' ' * (21 - len( name )), self.value_to_str( value ) ) )

class ConstraintInfo:
    @classmethod
    def solve_type_to_str(cls, solve_type: int) -> str:
        return solve_type == agx.Constraint.DIRECT and               'DIRECT' or\
               solve_type == agx.Constraint.DIRECT_AND_ITERATIVE and 'DIRECT_AND_ITERATIVE' or\
               solve_type == agx.Constraint.ITERATIVE and            'ITERATIVE' or\
                                                                     'UNKNOWN_SOLVE_TYPE'

    def __init__(self, c: agx.Constraint):
        # E.g., constraint.asHinge(), constraint.asPrismatic(), constraint.asLockJoint() etc.
        try:
            self.instance = getattr( c, 'as{}'.format( c.getClassName().split( '::' )[-1] ) )()
        except:
            self.instance = c
        self.name            = c.getName() if c else '[None]'
        self.type_name       = c.getClassName() if c else '[None]'
        self.bodies          = [RigidBodyInfo( c.getBodyAt( 0 ) ), RigidBodyInfo( c.getBodyAt( 1 ) )] if c else None
        self.solve_type      = c.getSolveType() if c else -1
        self.solve_type_name = self.solve_type_to_str( self.solve_type )
        self.compliance      = [c.getCompliance( dof ) for dof in range( 0, c.getNumDOF() )] if c else None
        self.damping         = [c.getDamping( dof ) for dof in range( 0, c.getNumDOF() )] if c else None
        self.controllers     = [ConstraintControllerInfo( c.getSecondaryConstraint( i ) ) for i in range( 0, c.getNumSecondaryConstraints() )] if c else None

    def print(self, indent: int = 0):
        print( ' ' * indent + '{}'.format( self.name ) )
        indent += 4
        print( ' ' * indent + '- Type:       {}'.format( self.type_name ) )
        print( ' ' * indent + '- Bodies:     {} <-> {}'.format( self.bodies[ 0 ].name if self.bodies else '[None]',
                                                                self.bodies[ 1 ].name if self.bodies else '[None]' ) )
        print( ' ' * indent + '- Solve type: {}'.format( self.solve_type_name ) )
        if self.instance is None:
            return
        print( ' ' * indent + '- Compliance: {:E}'.format( self.compliance[ 0 ] ) )
        for i in range( 1, len( self.compliance ) ):
            print( ' ' * indent + '              {:E}'.format( self.compliance[ i ] ) )
        print( ' ' * indent + '- Damping:    {}'.format( self.damping[ 0 ] ) )
        for i in range( 1, len( self.damping ) ):
            print( ' ' * indent + '              {}'.format( self.damping[ i ] ) )
        for ci in self.controllers:
            ci.print( indent )

class DisabledCollisionsStateHandler:
    """Object containing disabled collisions state of a simulation.

    Examples:
        from agxPythonModules.tools.simulation_content import DisabledCollisionsStateHandler

        handler = DisabledCollisionsStateHandler()
        state = handler.findState( geometry1, geometry2 )
        for disabledPair in state:
            print( 'disabled pair:', disabledPair[ 0 ], disabledPair[ 1 ] )
        if not state is None:
            state.print()
    """

    class State:
        def __init__(self,
                     ids: [(int, int)],
                     names: [(str, str)],
                     geometries: [(agxCollide.Geometry, agxCollide.Geometry)]):
            self.ids        = ids # type: [(int, int)]
            self.names      = names # type: [(str, str)]
            self.geometries = geometries # type: [(agxCollide.Geometry, agxCollide.Geometry)]
        
        def __iter__(self):
            for g in self.ids:
                yield g
            for g in self.names:
                yield g
            for g in self.geometries:
                yield g
        
        @property
        def empty(self):
            return len( self.ids ) == 0 and len( self.names ) == 0 and len( self.geometries ) == 0

        def print(self):
            print( 'Disabled collisions state:' )

            if self.empty:
                print( '    <empty>' )
                return

            def printLocal(prefix: str, groups):
                for group in groups:
                    print( prefix, group[ 0 ], group[ 1 ] )
            if len( self.ids ) > 0:
                printLocal( '    id:        ', self.ids )
            if len( self.names ) > 0:
                printLocal( '    name:      ', self.names )
            if len( self.geometries ) > 0:
                printLocal( '    geometries:', self.geometries )

    def __init__(self, simulation: agxSDK.Simulation):
        self.simulation = simulation
        self.synchronize()
    
    def __iter__(self):
        for group in [tuple( g ) for g in self.ids]:
            if len( group ) > 1:
                yield group
            else:
                yield (group[ 0 ], group[ 0 ])
        for group in [tuple( g ) for g in self.names]:
            yield group
        for group in self.geometries:
            yield group
    
    def synchronize(self):
        self.ids        = set()
        self.names      = set()
        self.geometries = []

        if self.simulation is None:
            return

        state = self.simulation.getSpace().findDisabledCollisionsState() # type: agxCollide.DisabledCollisionsState
        for p in state.getDisabledIds():
            self.ids.add( frozenset( { p[ 0 ], p[ 1 ] } ) )
        for p in state.getDisabledNames():
            self.names.add( frozenset( { p[ 0 ], p[ 1 ] } ) )
        for p in state.getDisabledGeometryPairs():
            self.geometries.append( p )

    def findState(self, g1: agxCollide.Geometry, g2: agxCollide.Geometry) -> State:
        if g1 is None or g2 is None:
            return None

        g1IdCollection = g1.findGroupIdCollection()
        g2IdCollection = g2.findGroupIdCollection()

        idIntersection         = self._findIntersection( g1IdCollection.getIds(), g2IdCollection.getIds(), self.ids )
        nameIntersection       = self._findIntersection( g2IdCollection.getNames(), g2IdCollection.getNames(), self.names )
        geometriesIntersection = [ ( g1, g2 ) ] if not g1.getEnableCollisions( g2 ) else []

        if len( geometriesIntersection ) == 0 and len( idIntersection ) == 0 and len( nameIntersection ) == 0:
            return None

        return self.State( idIntersection, nameIntersection, geometriesIntersection )
   
    def _findIntersection(self, groups1, groups2, disabledGroups: set()) -> [tuple]:
        pairs = set()
        for group1 in groups1:
            for group2 in groups2:
                pairs.add( frozenset( { group1, group2 } ) )

        intersection = disabledGroups.intersection( pairs )

        # frozenset with id1 == id2 becomes a tuple with one element.
        # The statement below converts from frozenset to tuple (size 1 or 2)
        # and from there if size == 1 to a tuple with two identical values.
        return [ g if len( g ) > 1 else (g[0], g[0]) for g in [tuple( group ) for group in intersection] ]

class SimulationContent:
    """Simulation content in a more accessible way.
    
    Examples:
        from agxPythonModules.tools.simulation_content import SimulationContent

        content = SimulationContent( simulation = mySimulation )
        if not content.initialize():
            return

        content.print()
        content.printStats()

        for name, rb in content.getBodiesContainingName( 'Component' ):
            print( name, rb.getMassProperties().getMass() )
        
        myCable = content.cables['myCable']
        simulation.remove( myCable )
    """

    @classmethod
    def filterDict(cls, dict: dict, pred: callable) -> dict:
        return OrderedDict( [ ( k, v ) for k, v in dict.items() if pred( k ) ] )

    @classmethod
    def printContactMaterial(cls, cm: agx.ContactMaterial):
        def cmSolveTypeName(cm: agx.ContactMaterial):
            return 'SPLIT' if cm.getFrictionModel() is None or cm.getFrictionModel().getSolveType() == agx.FrictionModel.SPLIT else\
                   'DIRECT' if cm.getFrictionModel().getSolveType() == agx.FrictionModel.DIRECT else\
                   'DIRECT_AND_ITERATIVE' if cm.getFrictionModel().getSolveType() == agx.FrictionModel.DIRECT_AND_ITERATIVE else\
                   'ITERATIVE'

        print( '    {} <-> {}'.format( cm.getMaterial1().getName(), cm.getMaterial2().getName() ) )
        print( '        - Solve Type:           {}'.format( cmSolveTypeName( cm ) ) )
        print( '        - Restitution:          {} [u = {}, v = {}]'.format( cm.getRestitution(),
                                                                             cm.getTangentialRestitution( agx.ContactMaterial.PRIMARY_DIRECTION ),
                                                                             cm.getTangentialRestitution( agx.ContactMaterial.SECONDARY_DIRECTION ) ) )
        print( '        - Youngs Modulus:       {:E}'.format( cm.getYoungsModulus() ) )
        print( '        - Damping:              {}'.format( cm.getDamping() ) )
        print( '        - Friction Coefficient: [{}, {}]'.format( cm.getFrictionCoefficient( agx.ContactMaterial.PRIMARY_DIRECTION ),
                                                                    cm.getFrictionCoefficient( agx.ContactMaterial.SECONDARY_DIRECTION ) ) )
        print( '        - Surface Viscosity:    [{:E}, {:E}]'.format( cm.getSurfaceViscosity( agx.ContactMaterial.PRIMARY_DIRECTION ),
                                                                    cm.getSurfaceViscosity( agx.ContactMaterial.SECONDARY_DIRECTION ) ) )
        print( '        - Contact Area:         {}'.format( cm.getUseContactAreaApproach() ) )
        print( '        - Contact Reduction:    {}'.format( cm.getContactReductionBinResolution() ) )
        print( '        - Adhesion:             {}'.format( cm.getAdhesion() ) )
        print( '        - Adhesive overlap:     {}'.format( cm.getAdhesiveOverlap() ) )

    @classmethod
    def printWire(cls, wire: agxWire.Wire):
        print( '    {}'.format( wire.getName() ) )
        print( '        - Radius:                {}'.format( wire.getRadius() ) )
        print( '        - Mass:                  {}'.format( wire.getMass() ) )
        print( '        - Rest length:           {}'.format( wire.getRestLength() ) )
        print( '        - Current length:        {}'.format( wire.getCurrentLength() ) )
        print( '        - Num nodes:             {}'.format( wire.getNumNodes() ) )
        print( '        - Resolution per length: {}'.format( wire.getResolutionPerUnitLength() ) )
        print( '        - Material:              {}'.format( wire.getMaterial().getName() ) )
        print( '            * Y Stretch:         {:E}'.format( wire.getMaterial().getWireMaterial().getYoungsModulusStretch() ) )
        print( '            * D Stretch:         {}'.format( wire.getMaterial().getWireMaterial().getDampingStretch() ) )
        print( '            * Y Bend:            {:E}'.format( wire.getMaterial().getWireMaterial().getYoungsModulusBend() ) )
        print( '            * D Bend:            {}'.format( wire.getMaterial().getWireMaterial().getDampingBend() ) )

    @classmethod
    def printCable(cls, cable: agxCable.Cable):
        print( '    {}'.format( cable.getName() ) )
        print( '        - Radius:                {}'.format( cable.getRadius() ) )
        print( '        - Mass:                  {}'.format( cable.getMass() ) )
        print( '        - Rest length:           {}'.format( cable.getRestLength() ) )
        print( '        - Current length:        {}'.format( cable.getCurrentLength() ) )
        print( '        - Num segments:          {}'.format( cable.getNumSegments() ) )
        print( '        - Material:              {}'.format( cable.getMaterial().getName() if not cable.getMaterial() is None else 'None' ) )
        print( '        - Properties:            ' )
        print( '            * Y Stretch:         {:E}'.format( cable.getCableProperties().getYoungsModulus( agxCable.STRETCH ) ) )
        print( '            * D Stretch:         {}'.format( cable.getCableProperties().getDamping( agxCable.STRETCH ) ) )
        print( '            * Y Bend:            {:E}'.format( cable.getCableProperties().getYoungsModulus( agxCable.BEND ) ) )
        print( '            * D Bend:            {}'.format( cable.getCableProperties().getDamping( agxCable.BEND ) ) )
        print( '            * Y Twist:           {:E}'.format( cable.getCableProperties().getYoungsModulus( agxCable.TWIST ) ) )
        print( '            * D Twist:           {}'.format( cable.getCableProperties().getDamping( agxCable.TWIST ) ) )

    @classmethod
    def printConstraint(cls, c: agx.Constraint):
        ci = ConstraintInfo( c )
        print( '~ Constraint ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~' )
        ci.print( 4 )
        print( '~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~' )
            
    @classmethod
    def printRigidBody(cls, rb: agx.RigidBody):
        rbi = RigidBodyInfo( rb )
        print( '- Rigid Body ------------------------------------------------------------------------' )
        rbi.print( 4 )
        print( '-------------------------------------------------------------------------------------' )
    
    @classmethod
    def printGeometry(cls, geometry: agxCollide.Geometry):
        gi = GeometryInfo( geometry )
        print( '* Geometry **************************************************************************' )
        gi.print( 4 )
        print( '*************************************************************************************' )

    @classmethod
    def printShape(cls, shape: agxCollide.Shape):
        si = ShapeInfo( shape )
        print( '+ Shape +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++' )
        si.print( 4 )
        print( '+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++' )

    def __init__(self, **kwargs):
        self.simulation = kwargs.get( 'simulation', kwargs.get( 'assembly', None ) )

        self.bodies           = OrderedDict()
        self.geometries       = OrderedDict()
        self.constraints      = OrderedDict()
        self.materials        = OrderedDict()
        self.contactMaterials = OrderedDict()
        self.interactions     = OrderedDict()
        self.wires            = OrderedDict()
        self.cables           = OrderedDict()
        self.observers        = OrderedDict()
        self.terrains         = OrderedDict()

        self.disabledCollisionsStateHandler = None

        if kwargs.get( 'initialize', False ):
            if not self.initialize():
                raise TypeError( 'Unable to initialize SimulationContent: simulation/assembly not valid.' )
    
    @property
    def simulation_is_assembly(self) -> bool:
        return isinstance( self.simulation, agxSDK.Assembly )

    def getInteractions(self, *args):
        def ret_interactions(rb_name: str):
            try:
                return self.interactions[ rb_name ]
            except:
                return []
        if len( args ) == 1:
            return ret_interactions( args[ 0 ] ) if isinstance( args[ 0 ], str ) else ret_interactions( args[ 0 ].getName() )
        elif len( args ) == 2:
            rb1_interactions = self.getInteractions( args[ 0 ] )
            rb2_interactions = self.getInteractions( args[ 1 ] )
            return [ interaction for interaction in rb1_interactions if interaction in rb2_interactions ]
        else:
            raise TypeError( 'SimulationContent.getInteractions takes either one or two arguments (rigid body names)' )

    def getBodies(self):
        return self.bodies.values()

    def getBodiesContainingName(self, name: str, matchCase: bool = True) -> dict:
        def caseMatched( key ):
            return name in key
        def caseIgnored( key ):
            return name.lower() in key.lower()
        return self.filterDict( self.bodies, matchCase and caseMatched or caseIgnored )

    def getObservers(self):
        return self.observers.values()

    def getGeometries(self):
        return self.geometries.values()

    def getConstraints(self):
        return self.constraints.values()

    def getMaterials(self):
        return self.materials.values()

    def getContactMaterials(self):
        return self.contactMaterials.values()

    def getContactMaterial(self, name1: str, name2: str) -> agx.ContactMaterial:
        try:
            return self.contactMaterials[ frozenset( { name1, name2 } ) ]
        except:
            return None

    def getWires(self):
        return self.wires.values()

    def getCables(self):
        return self.cables.values()
    
    def getTerrains(self):
        return self.terrains.values()

    def register(self, *args):
        for arg in args:
            if isinstance( arg, agx.Constraint ):
                self.constraints[ self._setName( arg ) ] = arg

                def getOrCreate( rb ):
                    rbName = rb == None and 'None' or rb.getName()
                    if not rbName in self.interactions:
                        self.interactions[ rbName ] = []
                    return self.interactions[ rbName ]

                getOrCreate( arg.getBodyAt( 0 ) ).append( arg )
                getOrCreate( arg.getBodyAt( 1 ) ).append( arg )

    def unregister(self, *args):
        for arg in args:
            if isinstance( arg, agx.Constraint ):
                if not arg.getName() in self.constraints:
                    print( 'Unable to unregister: {} - not found.' )
                    continue
                self.constraints[ arg.getName() ] = None
                self.interactions[ arg.getBodyAt( 0 ).getName()\
                                       if arg.getBodyAt( 0 ) else 'None' ].remove( arg )
                self.interactions[ arg.getBodyAt( 1 ).getName()\
                                       if arg.getBodyAt( 1 ) else 'None' ].remove( arg )

    def initialize(self):
        if self.simulation is None:
            print( 'Unable to initialize SimulationContent: simulation is None' )
            return False

        for rb in self.simulation.getRigidBodies():
            self.bodies[ self._setName( rb ) ] = rb

        for geometry in self.simulation.getGeometries():
            self.geometries[ self._setName( geometry ) ] = geometry
            if geometry.getMaterial():
                self.materials[ geometry.getMaterial().getName() ] = geometry.getMaterial()

        for observer in self.simulation.getObserverFrames():
            self.observers[ self._setName( observer ) ] = observer

        for constraint in self.simulation.getConstraints():
            self.register( constraint )

        # Things below assumes self.simulation is agxSDK.Simulation
        # and not either agxSDK.Assembly or agxSDK.Simulation.
        if self.simulation_is_assembly:
            return True

        for material1 in self.materials.values():
            for material2 in self.materials.values():
                if self.simulation.getMaterialManager().getContactMaterial( material1, material2 ):
                    self.contactMaterials[ frozenset( { material1.getName(),
                                                        material2.getName() } ) ] = self.simulation.getMaterialManager().getOrCreateContactMaterial( material1,
                                                                                                                                                        material2 )
        
        wires = agxWire.Wire.findAll( self.simulation )
        for wire in wires:
            self.wires[ self._setName( wire ) ] = wire

        cables = agxCable.Cable.getAll( self.simulation )
        for cable in cables:
            self.cables[ self._setName( cable ) ] = cable
        
        if _has_agxTerrain:
            terrains = agxTerrain.Terrain.findAll( self.simulation )
            for terrain in terrains:
                self.terrains[ self._setName( terrain ) ] = terrain
        
        self.disabledCollisionsStateHandler = DisabledCollisionsStateHandler( self.simulation )

        return True

    def print(self):
        print( '{:32s}  {}'.format( 'Body name', 'Mass' ) )
        print( '-' * (32 + 2 + 30) )
        for rb in self.bodies.values():
            print( '{:32s}: {} kg'.format( rb.getName(),
                                           rb.getMassProperties().getMass() ) )
        
        print()

        def constraintBodyName(constraint: agx.Constraint, index: int):
            return constraint.getBodyAt( index ).getName() if index < constraint.getNumBodies() else 'World'
        def constraintSolveTypeName(constraint: agx.Constraint):
            return 'DIRECT' if constraint.getSolveType() == agx.Constraint.DIRECT else\
                   'DIRECT_AND_ITERATIVE' if constraint.getSolveType() == agx.Constraint.DIRECT_AND_ITERATIVE else\
                   'ITERATIVE'

        print( '{:32s}  {}'.format( 'Constraint name', 'Bodies' ) )
        print( '-' * (32 + 2 + 30) )
        for constraint in self.constraints.values():
            if constraint.getNumBodies() < 1:
                continue

            print( '{:32s}: {} <-> {}'.format( constraint.getName(),
                                               constraintBodyName( constraint, 0 ),
                                               constraintBodyName( constraint, 1 ) ) )

        print()

        print( '{:32s}  {}'.format( 'Body name', 'Constraints' ) )
        print( '-' * (32 + 2 + 30) )
        for bodyName, constraints in self.interactions.items():
            hasValidConstraint = False
            for constraint in constraints:
                hasValidConstraint = hasValidConstraint or constraint.getNumBodies() > 0
            
            if not hasValidConstraint:
                continue
            
            print( '{:32s}:'.format( bodyName ), end = '' )
            for constraint in constraints:
                if constraint.getNumBodies() < 1:
                    continue

                formatStr = '{:33s} {}' if constraint != constraints[ 0 ] else '{} {}'
                print( formatStr.format( '', constraint.getName() ) )

        print()

        print( 'Contact materials' )
        print( '-' * (32 + 2 + 30) )
        for cm in self.contactMaterials.values():
            self.printContactMaterial( cm )

        print()

        print( 'Wires' )
        print( '-' * (32 + 2 + 30) )
        for _, wire in self.wires.items():
            self.printWire( wire )

        print()

        print( 'Cables' )
        print( '-' * (32 + 2 + 30) )
        for _, cable in self.cables.items():
            self.printCable( cable )

        print()

    def printStats(self):
        print( 'Data collected from {}'.format( self.simulation.getName() if self.simulation_is_assembly else 'simulation' ) )
        if not self.simulation_is_assembly:
            print( '  Time step: %f (%d Hz)' %( self.simulation.getTimeStep(), 1.0 / self.simulation.getTimeStep() ) )
            print( '  Gravity:  [{:2.4f} {:2.4f} {:2.4f}]'.format( self.simulation.getUniformGravity().x(),
                                                                self.simulation.getUniformGravity().y(),
                                                                self.simulation.getUniformGravity().z() ) )
        print( '  Objects:' )
        print( '    #bodies:           ', len( self.bodies ) )
        print( '    #geometries:       ', len( self.geometries ) )
        print( '    #constraints:      ', len( self.constraints ) )
        print( '    #materials:        ', len( self.materials ) )
        print( '    #contact materials:', len( self.contactMaterials ) )
        print( '    #wires:            ', len( self.wires ) )
        print( '    #cables:           ', len( self.cables ) )
        print( '    #observers:        ', len( self.observers ) )

    def _setName(self, obj):
        container = None
        if isinstance( obj, agx.RigidBody ):
            container = self.bodies
        elif isinstance( obj, agxCollide.Geometry ):
            container = self.geometries
        elif isinstance( obj, agx.Constraint ):
            container = self.constraints
        elif isinstance( obj, agxWire.Wire ):
            container = self.wires
        elif isinstance( obj, agxCable.Cable ):
            container = self.cables
        elif isinstance( obj, agx.ObserverFrame ):
            container = self.observers
        elif _has_agxTerrain and isinstance( obj, agxTerrain.Terrain ):
            container = self.terrains
        else:
            raise TypeError( 'Unknown typename: {}'.format( type( obj ).__name__ ) )

        # The objects name is unique.
        if len( obj.getName() ) > 0 and not obj.getName() in container:
            return obj.getName()

        orgName = obj.getName()\
                      if len( obj.getName() ) > 0\
                      else 'Unknown_{}'.format( type( obj ).__name__ )
        newName = orgName
        counter = 1
        while newName in container:
            newName = '{}_{}'.format( orgName, counter )
            counter += 1

        obj.setName( newName )

        return newName