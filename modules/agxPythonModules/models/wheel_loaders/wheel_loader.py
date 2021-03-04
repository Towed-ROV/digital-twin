"""
Wheel loader base model with two-body tires and driveline:
                                           engine
                                             ^
                                             |
                                             |
                                      torque_converter
                                             ^
                                             |
                                             |
         front_right_tire_actuator       gear_box              rear_right_tire_actuator
                    ^                        ^                            ^
                    |                        | <-- brake_hinge            |
                    |                        |                            |
           front_differential <----- center_differential --------> rear_differential
                    |                                                     |
                    |                                                     |
         front_left_tire_actuator                               rear_left_tire_actuator
"""

import agxCollide
import agx
import agxSDK
import agxModel
import agxPowerLine
import agxDriveTrain

from agxPythonModules.tools.read_file import SimulationFile
from agxPythonModules.tools.simulation_content import RigidBodyInfo, DisabledCollisionsStateHandler
from agxPythonModules.utils.environment import root, simulation as global_simulation
from agxPythonModules.utils.callbacks import StepEventCallback, KeyboardCallback as Input, GamepadCallback as Gamepad
from agxPythonModules.utils.constraints import createHinge

from enum import Enum
from agxPythonModules.utils.Struct import Struct


class Tire(agxModel.TwoBodyTire):
    """
    Two body tire model where outer tire radius, if not given,
    is estimated given shapes with radius attribute. The hub radius,
    if not given, is estimated as half the outer radius.

    Examples:
        tire1 = Tire( tire_body = outer_tire_body,
                      hub_body  = inner_tire_body )

        tire2 = Tire( tire_body   = outer_tire_body,
                      hub_body    = inner_tire_body,
                      tire_radius = 1.2,
                      hub_radius  = 0.4 )

        tire1.hinge.getMotor1D().setSpeed( 1 )
        tire2.hinge.getMotor1D().setSpeed( 1 )
    """

    class Location(Enum):
        FRONT_LEFT  = 0
        FRONT_RIGHT = 1
        REAR_LEFT   = 2
        REAR_RIGHT  = 3

    @classmethod
    def find_radius(cls, rb: agx.RigidBody):
        """
        Tries to find radius of a rigid body. Raises AttributeError
        if the radius wasn't possible to find.

        Arguments:
            rb: agx.RigidBody - rigid body with geometries.

        Returns:
            float - radius if found, otherwise AttributeError is raised.
        """
        info   = RigidBodyInfo( rb )
        radius = -1
        for geometry in info.geometries:
            if not geometry.collisions_enabled:
                continue

            for shape in geometry.shapes:
                try:
                    radius = max( radius, shape.radius )
                except AttributeError:
                    pass
        if radius > 0:
            return radius

        raise AttributeError( 'Radius not found in body: {}'.format( rb.getName() ) )

    @property
    def hinge(self) -> agx.Hinge:
        """
        Returns:
            agx.Hinge - hub <-> chassis hinge if set. Otherwise None.
        """
        return self.m_hinge

    @hinge.setter
    def hinge(self, hinge: agx.Hinge):
        """
        Assign hub <-> chassis hinge.

        Arguments:
            hinge: agx.Hinge - hub <-> chassis hinge.
        """
        self.m_hinge = hinge

    @property
    def material(self) -> agx.Material:
        """
        Tire material collected from first geometry with collisions
        enabled and material with name != 'Unknown Material'.

        Returns:
            agx.Material - tire material
        """
        return self.m_material

    def __init__(self, **kwargs):
        """
        Construct given tire_body and hub_body.

        Arguments:
            tire_body: agx.RigidBody - outer body
            hub_body: agx.RigidBody - inner body
            tire_radius: float - [Optional] tire radius, if not given the radius is estimated given enabled
                                 shapes with radius attribute in tire_body.
            hub_radius: float - [Optional] hub radius, if not given the radius is set to half of tire_radius.
            local_transform: agx.AffineMatrix4x4 - [Optional] local transform so that tire y axis is the rotation axis.
        """
        if not 'tire_radius' in kwargs:
            kwargs[ 'tire_radius' ] = Tire.find_radius( kwargs[ 'tire_body' ] )

        if not 'hub_radius' in kwargs:
            kwargs[ 'hub_radius' ] = 0.5 * kwargs[ 'tire_radius' ]

        self.m_hinge    = None
        self.m_material = None

        tire_body_info = RigidBodyInfo( kwargs[ 'tire_body' ] )
        for tire_geometry in tire_body_info.geometries:
            if tire_geometry.collisions_enabled and tire_geometry.material != 'Unknown Material':
                self.m_material = tire_geometry.instance.getMaterial()
                break

        super().__init__( kwargs[ 'tire_body' ],
                          kwargs[ 'tire_radius' ],
                          kwargs[ 'hub_body' ],
                          kwargs[ 'hub_radius' ],
                          kwargs.get( 'local_transform', agx.AffineMatrix4x4() ) )

class WheelLoader(agxSDK.Assembly):
    """
    Wheel loader class that loads 'model_path' and configures
    tires and driveline etc. The tires are listed as:
        [front left, front right, rear left, rear right]

    The driveline is configured with a combustion engine, torque converter,
    gear box, center differential, front and rear differentials and
    tire hinge actuators. The brake is located on the shaft between the
    gear box and the center differential which enables brakes and steering
    without jamming.
                                           engine
                                             ^
                                             |
                                             |
                                      torque_converter
                                             ^
                                             |
                                             |
         front_right_tire_actuator       gear_box              rear_right_tire_actuator
                    ^                        ^                            ^
                    |                        | <-- brake_hinge            |
                    |                        |                            |
           front_differential <----- center_differential --------> rear_differential
                    |                                                     |
                    |                                                     |
         front_left_tire_actuator                               rear_left_tire_actuator
    """

    @property
    def front_body(self) -> agx.RigidBody:
        """
        Returns:
            agx.RigidBody - front body of this wheel loader.
        """
        return self.m_body_front

    @property
    def rear_body(self) -> agx.RigidBody:
        """
        Returns:
            agx.RigidBody - rear body of this wheel loader.
        """
        return self.m_body_rear

    @property
    def bucket_body(self) -> agx.RigidBody:
        """
        Returns:
            agx.RigidBody - bucket body of this wheel loader.
        """
        return self.m_body_bucket

    @property
    def front_forward_local(self) -> agx.Vec3:
        """
        Returns:
            agx.Vec3 - front body forward vector in front body coordinate frame
        """
        return self.m_forward_front

    @property
    def rear_forward_local(self) -> agx.Vec3:
        """
        Returns:
            agx.Vec3 - rear body forward vector in rear body coordinate frame
        """
        return self.m_forward_rear

    @property
    def front_forward_world(self) -> agx.Vec3:
        """
        Returns:
            agx.Vec3 - front body forward vector in world coordinate frame
        """
        return self.front_body.getFrame().transformVectorToWorld( self.front_forward_local )

    @property
    def rear_forward_world(self) -> agx.Vec3:
        """
        Returns:
            agx.Vec3 - rear body forward vector in world coordinate frame
        """
        return self.rear_body.getFrame().transformVectorToWorld( self.rear_forward_local )

    @property
    def speed(self) -> float:
        """
        Speed of this wheel loader, taken from front body
        when gear is not in reverse - otherwise from rear
        body when gear is reverse.

        Returns:
            float - speed of this wheel loader.
        """
        return self.front_body.getVelocity() * self.front_forward_world\
               if self.gear_box.getGear() > 0 else\
               self.rear_body.getVelocity() * self.rear_forward_world

    @property
    def tires(self) -> [Tire]:
        """
        Returns:
            [Tire] - list of tires.
        """
        return self.m_tires

    def tire(self, location) -> Tire:
        """
        Tire given location or index.

        Examples:
            tire1 = wheel_loader.tire( 0 )
            tire1_again = wheel_loader.tire( Tire.Location.FRONT_LEFT )
            assert tire1 == tire1_again

        Arguments:
            location: int, Tire.Location - tire location [front left, front right, rear left, rear right]

        Returns:
            Tire - tire at given location.
        """
        try:
            return self.tires[ location.value ]
        except AttributeError:
            return self.tires[ location ]

    @property
    def front_tire_material(self) -> agx.Material:
        """
        Returns:
            agx.Material - front wheels tire material assuming both front tires has the same material instance.
        """
        return self.tire( Tire.Location.FRONT_LEFT ).material

    @property
    def rear_tire_material(self) -> agx.Material:
        """
        Returns:
            agx.Material - rear wheels tire material assuming both rear tires has the same material instance.
        """
        return self.tire( Tire.Location.REAR_LEFT ).material

    @property
    def tire_hinges(self) -> [agx.Hinge]:
        """
        Returns:
            [agx.Hinge] - list of tire <-> chassis hinges.
        """
        return [ tire.hinge for tire in self.tires ]

    @property
    def steering_hinge(self) -> agx.Hinge:
        """
        Returns:
            agx.Hinge - hinge between front and rear body.
        """
        return self.m_steering_hinge

    @property
    def tilt_prismatics(self) -> [agx.Prismatic]:
        """
        Returns:
            [agx.Prismatic] - list of prismatics controlling bucket tilt angle.
        """
        return self.m_tilt_prismatics

    @property
    def elevate_prismatics(self) -> [agx.Prismatic]:
        """
        Returns:
            [agx.Prismatic] - list of prismatics controlling bucket elevation.
        """
        return self.m_elevate_prismatics

    @property
    def powerline(self) -> agxPowerLine.PowerLine:
        """
        Returns:
            agxPowerLine.PowerLine - powerline instance.
        """
        return self.m_powerline

    @property
    def engine(self) -> agxDriveTrain.CombustionEngine:
        """
        Returns:
            agxDriveTrain.CombustionEngine - the combustion engine.
        """
        return self.m_engine

    @property
    def torque_converter(self) -> agxDriveTrain.TorqueConverter:
        """
        Returns:
            agxDriveTrain.TorqueConverter - the torque converter.
        """
        return self.m_torque_converter

    @property
    def gear_box(self) -> agxDriveTrain.GearBox:
        """
        Returns:
            agxDriveTrain.GearBox - the gear box.
        """
        return self.m_gear_box

    @property
    def center_diff(self) -> agxDriveTrain.Differential:
        """
        Returns:
            agxDriveTrain.Differential - the center differential.
        """
        return self.m_center_diff

    @property
    def front_diff(self) -> agxDriveTrain.Differential:
        """
        Returns:
            agxDriveTrain.Differential - the front differential.
        """
        return self.m_front_diff

    @property
    def rear_diff(self) -> agxDriveTrain.Differential:
        """
        Returns:
            agxDriveTrain.Differential - the rear differential.
        """
        return self.m_rear_diff

    @property
    def tire_actuators(self) -> [agxPowerLine.RotationalActuator]:
        """
        Returns:
            [agxPowerLine.RotationalActuator] - list of tire rotational actuators.
        """
        return self.m_tire_actuators

    def tire_actuator(self, location) -> agxPowerLine.RotationalActuator:
        """
        Arguments:
            location: int, Tire.Location - tire location [front left, front right, rear left, rear right]

        Returns:
            agxPowerLine.RotationalActuator - tire actuator at given location.
        """
        try:
            return self.tire_actuators[ location.value ]
        except AttributeError:
            return self.tire_actuators[ location ]

    @property
    def keyboard_controls(self):
        """
        Examples:
            wheel_loader = WheelLoaderL70( keyboard_controls = WheelLoaderL70.default_keyboard_controls() )
            kb = wheel_loader.keyboard_controls
            kb.engine.forward.key = Input.KEY_Space
            # Assign again for the changes to have an effect.
            wheel_loader.keyboard_controls = kb

        Returns:
            Struct - current keyboard controls (changes will have no effect unless the
                     instance is assigned again) - None if not set.
        """
        return self.m_keyboard_controls

    @property
    def gamepad_controls(self):
        """
        Returns:
            Struct - current gamepad controls (changes will have no effect unless the
                     instance is assigned again) - None if not set.
        """
        return self.m_gamepad_controls
    
    @property
    def bucket_tilt_controller(self):
        """
        Returns:
            BucketTiltController - bucket tilt controller
        """
        return self.m_bucket_tilt_controller

    def configure_ground_contact_materials(self, ground) -> [agx.ContactMaterial]:
        """
        Utility: Configures ground <-> tires contact materials given ground
        object (agx.Material, agxCollide.Geometry or agx.RigidBody) and
        returns a list of the created contact materials. The contact materials
        are activated in the simulation.

        Arguments:
            ground: agx.Material, agxCollide.Geometry, agx.RigidBody - ground object
        """
        ground_materials = [ground] if isinstance( ground, agx.Material ) else\
                           [ground.getMaterial()] if isinstance( ground, agxCollide.Geometry ) else\
                           [geometry.getMaterial() for geometry in obj.getGeometries()] if isinstance( ground, agx.RigidBody ) else\
                           None
        if ground_materials is None:
            raise TypeError( 'Unable to find material for type: {}'.format( type( obj ) ) )

        tire_materials = [tire.material for tire in self.tires]
        tire_ground_contact_materials = []
        for ground_material in ground_materials:
            for tire_material in tire_materials:
                tire_ground_cm = global_simulation().getMaterialManager().getOrCreateContactMaterial( tire_material,
                                                                                                      ground_material ) # type: agx.ContactMaterial
                tire_ground_cm.setYoungsModulus( 1.0E5 )
                tire_ground_cm.setFrictionModel( agx.ScaleBoxFrictionModel( agx.FrictionModel.DIRECT ) )
                tire_ground_cm.setFrictionCoefficient( 1, agx.ContactMaterial.PRIMARY_DIRECTION )
                tire_ground_cm.setFrictionCoefficient( 1, agx.ContactMaterial.SECONDARY_DIRECTION )
                tire_ground_contact_materials.append( tire_ground_cm )
        return tire_ground_contact_materials

    def create_shovel(self):
        import agxRender
        import agxTerrain
        def center(line: agx.Line):
            return 0.5 * ( line.p1 + line.p2 )

        shovel = agxTerrain.Shovel( self.bucket_body,
                                    self.model_top_edge,
                                    self.model_cutting_edge,
                                    self.model_cutting_dir )

        tew = shovel.getTopEdgeWorld()
        cew = shovel.getCuttingEdgeWorld()
        cdw = shovel.getCuttingDirectionWorld()

        def render_edge(edge, color):
            agxRender.RenderSingleton.instance().add( edge.p1,
                                                      edge.p2,
                                                      0.015,
                                                      color )
        render_edge( tew, agxRender.Color.Yellow() )
        render_edge( cew, agxRender.Color.Red() )
        agxRender.RenderSingleton.instance().add( center( cew ),
                                                  center( cew ) + 0.5 * cdw,
                                                  0.015,
                                                  agxRender.Color.Blue() )
        return shovel
        
    def __init__(self, **kwargs):
        """
        Construct given optional arguments.

        Arguments:
            model_path: str - path to model.
            driveline_settings: Struct - driveline settings.
            tire_settings: Struct - tire settings.
            keyboard_controls: Struct - keyboard controls (optional)
            gamepad_controls: Struct - gamepad controls (optional)
        """
        super().__init__()

        temp_simulation = agxSDK.Simulation()

        data = SimulationFile( simulation = temp_simulation, root = root() )
        if not data.load( filename = kwargs[ 'model_path' ], parent = self ):
            raise FileNotFoundError( 'Unable to load wheel loader model: {}'.format( kwargs[ 'model_path' ] ) )

        states = {}
        for rb in self.getRigidBodies():
            states[ rb.getUuid() ] = rb.getEnable()

        disabled_collisions = DisabledCollisionsStateHandler( temp_simulation )
        for disabled in disabled_collisions:
            disabled1 = disabled[ 0 ]
            disabled2 = disabled[ 1 ] if len( disabled ) > 1 else disabled[ 0 ]
            if isinstance( disabled1, agxCollide.Geometry ):
                continue
            global_simulation().getSpace().setEnablePair( disabled1, disabled2, False )

        self._initialize( data, **kwargs )

        del disabled_collisions
        del data
        del temp_simulation

        for rb in self.getRigidBodies():
            if not rb.getUuid() in states:
                continue
            rb.setEnable( states[ rb.getUuid() ] )
        
        from . import BucketTiltController
        self.bucket_tilt_controller = kwargs.get( 'bucket_tilt_controller', BucketTiltController( self ) )

        assert self.tire( Tire.Location.FRONT_LEFT ).material == self.tire( Tire.Location.FRONT_RIGHT ).material,\
               'Front left and right tire materials expected to be the same.'
        assert self.tire( Tire.Location.REAR_LEFT ).material == self.tire( Tire.Location.REAR_RIGHT ).material,\
               'Rear left and right tire materials expected to be the same.'
        assert self.tire( Tire.Location.FRONT_LEFT ).material != self.tire( Tire.Location.REAR_LEFT ).material,\
               'Front and rear tire materials not expected to be the same.'

    def _initialize(self, data: SimulationFile, **kwargs):
        self.m_body_front  = data.bodies[ 'FrontBody' ]
        self.m_body_rear   = data.bodies[ 'RearBody' ]
        self.m_body_bucket = data.bodies[ 'Bucket' ]

        self.m_forward_front = self.front_body.getFrame().transformVectorToLocal( agx.Vec3.X_AXIS() )
        self.m_forward_rear  = self.rear_body.getFrame().transformVectorToLocal( agx.Vec3.X_AXIS() )

        self.m_steering_hinge = [ c.asHinge()\
                                  for c in data.getInteractions( self.m_body_front, self.m_body_rear )\
                                      if c.asHinge() and 'Waist' in c.getName() ][ 0 ]
        assert self.m_steering_hinge, 'Unable to find steering hinge between {} and {}.'.format( self.m_body_front.getName(),
                                                                                                 self.m_body_rear.getName() )

        # Creating tires: [front_left, front_right, rear_left, rear_right]
        # The local transform makes sure the y axes are pointing out from
        # the tire center positions.
        local_tire_transform = agx.AffineMatrix4x4.rotate( agx.Vec3.Z_AXIS(), agx.Vec3.Y_AXIS() )
        self.m_tires = [
            Tire( tire_body       = data.bodies[ 'LeftFrontTire' ],
                  hub_body        = data.bodies[ 'LeftFrontRim' ],
                  local_transform = local_tire_transform ),
            Tire( tire_body       = data.bodies[ 'RightFrontTire' ],
                  hub_body        = data.bodies[ 'RightFrontRim' ],
                  local_transform = local_tire_transform ),
            Tire( tire_body       = data.bodies[ 'LeftRearTire' ],
                  hub_body        = data.bodies[ 'LeftRearRim' ],
                  local_transform = local_tire_transform ),
            Tire( tire_body       = data.bodies[ 'RightRearTire' ],
                  hub_body        = data.bodies[ 'RightRearRim' ],
                  local_transform = local_tire_transform ),
        ]

        # 1. Parameters for tire.
        # 2. Adding the tire instance to this assembly.
        # 3. Collecting driving hinges.
        # 4. Removing any constraint between tire <-> hub.
        tire_settings = kwargs[ 'tire_settings' ]
        for tire in self.tires: # type: Tire
            tire.setStiffness( tire_settings.stiffness.radial, agxModel.TwoBodyTire.RADIAL )
            tire.setStiffness( tire_settings.stiffness.lateral, agxModel.TwoBodyTire.LATERAL )
            tire.setStiffness( tire_settings.stiffness.bending, agxModel.TwoBodyTire.BENDING )
            tire.setStiffness( tire_settings.stiffness.torsional, agxModel.TwoBodyTire.TORSIONAL )

            tire.setDampingCoefficient( tire_settings.damping_coefficient.radial, agxModel.TwoBodyTire.RADIAL )
            tire.setDampingCoefficient( tire_settings.damping_coefficient.lateral, agxModel.TwoBodyTire.LATERAL )
            tire.setDampingCoefficient( tire_settings.damping_coefficient.bending, agxModel.TwoBodyTire.BENDING )
            tire.setDampingCoefficient( tire_settings.damping_coefficient.torsional, agxModel.TwoBodyTire.TORSIONAL )

            tire.getHinge().setSolveType( agx.Constraint.DIRECT )

            self.add( tire )

            hub_constraints = data.getInteractions( tire.getHubRigidBody() )
            for hub_constraint in hub_constraints:
                if hub_constraint.asHinge():
                    tire.hinge = hub_constraint.asHinge()

            tire_hub_constraints = data.getInteractions( tire.getTireRigidBody(),
                                                         tire.getHubRigidBody() )
            for tire_hub_constraint in tire_hub_constraints:
                self.remove( tire_hub_constraint )

        # Bucket elevate and tilt prismatics.
        self.m_elevate_prismatics = [
            data.constraints[ 'LeftLowerPrismatic' ].asPrismatic(),
            data.constraints[ 'RightLowerPrismatic' ].asPrismatic()
        ]

        self.m_tilt_prismatics = [
            data.constraints[ 'CenterPrismatic' ].asPrismatic()
        ]

        ###################################################################################
        #                                  Drivetrain                                     #
        ###################################################################################
        #                                    engine
        #                                      ^
        #                                      |
        #                                      |
        #                               torque_converter
        #                                      ^
        #                                      |
        #                                      |
        #  front_right_tire_actuator       gear_box              rear_right_tire_actuator
        #             ^                        ^                            ^
        #             |                        | <-- brakes                 |
        #             |                        |                            |
        #    front_differential <----- center_differential --------> rear_differential
        #             |                                                     |
        #             |                                                     |
        #  front_left_tire_actuator                               rear_left_tire_actuator
        ###################################################################################
        driveline_settings = kwargs[ 'driveline_settings' ]

        self.m_powerline = agxPowerLine.PowerLine()
        self.add( self.m_powerline )

        self.m_engine           = agxDriveTrain.CombustionEngine( driveline_settings.engine.displacement_volume )
        self.m_torque_converter = agxDriveTrain.TorqueConverter()
        self.m_gear_box         = agxDriveTrain.GearBox()
        self.m_center_diff      = agxDriveTrain.Differential()
        self.m_front_diff       = agxDriveTrain.Differential()
        self.m_rear_diff        = agxDriveTrain.Differential()
        self.m_tire_actuators   = []
        for tire in self.tires:
            self.tire_actuators.append( agxPowerLine.RotationalActuator( hinge = tire.hinge ) )
            tire.hinge.getMotor1D().setEnable( False )

        engine_torque_converter_shaft      = agxDriveTrain.Shaft()
        torque_converter_gear_box_shaft    = agxDriveTrain.Shaft()
        gear_box_center_diff_shaft         = agxDriveTrain.Shaft()
        center_diff_front_diff_shaft       = agxDriveTrain.Shaft()
        center_diff_rear_diff_shaft        = agxDriveTrain.Shaft()
        front_diff_front_left_wheel_shaft  = agxDriveTrain.Shaft()
        front_diff_front_right_wheel_shaft = agxDriveTrain.Shaft()
        rear_diff_rear_left_wheel_shaft    = agxDriveTrain.Shaft()
        rear_diff_rear_right_wheel_shaft   = agxDriveTrain.Shaft()

        self.powerline.setSource( self.engine )

        u_input  = agxPowerLine.INPUT
        u_output = agxPowerLine.OUTPUT

        # Connecting engine -> torque converter -> gear box -> center differential.
        self.engine.connect( u_output, u_input, engine_torque_converter_shaft )
        engine_torque_converter_shaft.connect( self.torque_converter )
        self.torque_converter.connect( u_output, u_input, torque_converter_gear_box_shaft )
        torque_converter_gear_box_shaft.connect( self.gear_box )
        self.gear_box.connect( u_output, u_input, gear_box_center_diff_shaft )
        gear_box_center_diff_shaft.connect( self.center_diff )

        # Connecting center differential -> front differential -> left and right front tires.
        self.center_diff.connect( u_output, u_input, center_diff_front_diff_shaft )
        center_diff_front_diff_shaft.connect( self.front_diff )
        self.front_diff.connect( u_output, u_input, front_diff_front_left_wheel_shaft )
        self.front_diff.connect( u_output, u_input, front_diff_front_right_wheel_shaft )
        front_diff_front_left_wheel_shaft.connect( self.tire_actuator( Tire.Location.FRONT_LEFT ) )
        front_diff_front_right_wheel_shaft.connect( self.tire_actuator( Tire.Location.FRONT_RIGHT ) )

        # Connecting center differential -> rear differential -> left and right rear tires.
        self.center_diff.connect( u_output, u_input, center_diff_rear_diff_shaft )
        center_diff_rear_diff_shaft.connect( self.rear_diff )
        self.rear_diff.connect( u_output, u_input, rear_diff_rear_left_wheel_shaft )
        self.rear_diff.connect( u_output, u_input, rear_diff_rear_right_wheel_shaft )
        rear_diff_rear_left_wheel_shaft.connect( self.tire_actuator( Tire.Location.REAR_LEFT ) )
        rear_diff_rear_right_wheel_shaft.connect( self.tire_actuator( Tire.Location.REAR_RIGHT ) )

        self.engine.setEnable( True )
        self.engine.setVolumetricEfficiency( driveline_settings.engine.volumetric_efficiency )
        self.engine.setThrottle( 0.0 )

        mu_nu = agx.RealPairVector()
        for ratio_multiplication_pair in driveline_settings.torque_converter.multiplication_table:
            mu_nu.append( agx.RealPair( ratio_multiplication_pair[ 0 ], ratio_multiplication_pair[ 1 ] ) )
        self.torque_converter.setMuTable( mu_nu )
        self.torque_converter.setMaxMultiplication( driveline_settings.torque_converter.max_multiplication )
        self.torque_converter.setPumpTorqueReferenceRPM( driveline_settings.torque_converter.reference_rpm )

        gear_ratios = agx.RealVector()
        gear_ratios.append( driveline_settings.gear_box.gear_ratios[ 0 ] )
        gear_ratios.append( driveline_settings.gear_box.gear_ratios[ 1 ] )
        self.gear_box.setGearRatios( gear_ratios )
        self.gear_box.gearUp()

        self.center_diff.setGearRatio( driveline_settings.center_differential.gear_ratio )
        self.center_diff.setLock( driveline_settings.center_differential.locked )

        self.front_diff.setGearRatio( driveline_settings.front_differential.gear_ratio )
        self.front_diff.setLock( driveline_settings.front_differential.locked )

        self.rear_diff.setGearRatio( driveline_settings.rear_differential.gear_ratio )
        self.rear_diff.setLock( driveline_settings.rear_differential.locked )

        # Brake hinge located on the shaft between the gear box
        # and the center differential. Use, e.g., the motor to
        # brake but still have a working torque distribution
        # to the tires when braking and turining.
        self.m_brake_hinge = createHinge( rb1      = gear_box_center_diff_shaft.getRotationalDimension().getOrReserveBody(),
                                          axis     = agx.Vec3.X_AXIS(),
                                          position = agx.Vec3() )
        self.m_brake_hinge.setEnable( False )
        self.add( self.m_brake_hinge )

        self.m_keyboard_controls      = None
        self.m_gamepad_controls       = None
        self.keyboard_controls        = kwargs.get( 'keyboard_controls', None )
        self.gamepad_controls         = kwargs.get( 'gamepad_controls', None )
        self.m_bucket_tilt_controller = None # Default created in __init__.

    @bucket_tilt_controller.setter
    def bucket_tilt_controller(self, bucket_tilt_controller):
        if self.m_bucket_tilt_controller:
            self.m_bucket_tilt_controller._on_remove()
        self.m_bucket_tilt_controller = bucket_tilt_controller
        if self.m_bucket_tilt_controller:
            self.m_bucket_tilt_controller._on_add()

    def _engine_state_handler(self, control) -> Struct:
        """
        Engine state handler given forward and reverse values
        from either keyboard or gamepad inputs.
        """
        state = Struct( {
            'release_time': 0.0,
            'forward': {
                'value':  0.0
            },
            'reverse': {
                'value':  0.0
            }
        } )

        def set_throttle(value: float):
            self.engine.setThrottle( value )

        def set_brake(value: float):
            brake_torque = value * 1.5E5

            self.m_brake_hinge.setEnable( True )
            self.m_brake_hinge.getMotor1D().setEnable( value > 0 )
            self.m_brake_hinge.getMotor1D().setSpeed( 0 )
            self.m_brake_hinge.getMotor1D().setForceRange( -brake_torque, brake_torque )

        has_decrease_rate = hasattr( control, 'throttle_decrease_rate' )
        is_gamepad = not has_decrease_rate
        
        def engine_state_handler(_):
            # If we did not manage to init the gamepad we should leave engine control to default/keyboard
            if is_gamepad and Gamepad.instance() is None:
                return

            speed = self.speed

            # Idle and close to zero speed. Hit the brakes.
            if abs( speed ) < 0.15 and state.forward.value == 0.0 and state.reverse.value == 0.0:
                set_brake( 1.0 )
                set_throttle( 0 )
            else:
                if state.forward.value > 0:
                    # Pressing forward but we're going in reverse. Brake.
                    if speed < -0.15:
                        set_throttle( 0.0 )
                        set_brake( state.forward.value )
                    else:
                        self.gear_box.setGear( 1 )
                        set_brake( 0.0 )
                        set_throttle( state.forward.value )
                elif state.reverse.value > 0:
                    # Pressing reverse but we're going forward. Brake.
                    if speed > 0.15:
                        set_throttle( 0.0 )
                        set_brake( state.reverse.value )
                    else:
                        self.gear_box.setGear( 0 )
                        set_brake( 0 )
                        set_throttle( state.reverse.value )
                # Idle input - engine brake until minimum speed.
                else:
                    set_brake( 0.0 )
                    if has_decrease_rate:
                        set_throttle( max( 1.0 - control.throttle_decrease_rate * state.release_time, 0.0 ) )
                    else:
                        set_throttle( 0.0 )

            state.forward.value = 0.0
            state.reverse.value = 0.0
            state.release_time += global_simulation().getTimeStep()

        control.engine_state_handler = engine_state_handler
        StepEventCallback.preCallback( control.engine_state_handler )

        return state

    @keyboard_controls.setter
    def keyboard_controls(self, controls):
        """
        Assign new keyboard controls, any previous settings will be removed.

        Arguments:
            control: Struct - keyboard controls, None to deactivate
        """
        def engine_keyboard_control_callback(control: Struct) -> callable:
            """
            Defines on-keyboard callback for the engine.
            """
            state = self._engine_state_handler( control )

            def on_keyboard(data: Input.Data):
                is_forward = data.key == control.forward.key
                if is_forward:
                    state.forward.value = min( control.throttle_increase_rate * data.timeDown, 1.0 ) if data.down else 0.0
                else:
                    state.reverse.value = min( control.throttle_increase_rate * data.timeDown, 1.0 ) if data.down else 0.0

                state.release_time = 0.0 if data.isKeyUp else state.release_time

            return on_keyboard

        def keyboard_control_callback(control: Struct) -> callable:
            """
            Defines on-keyboard callback for constraint controls.
            """
            def on_keyboard(input_data: Input.Data):
                def set_speed(speed: float):
                    for constraint in control.constraints:
                        constraint.getLock1D().setEnable( speed == 0.0 )
                        constraint.getLock1D().setPosition( constraint.getAngle() )
                        constraint.getMotor1D().setEnable( not constraint.getLock1D().getEnable() )
                        constraint.getMotor1D().setSpeed( speed )

                def decelerate_callback(t: float):
                    all_zero = True
                    for constraint in control.constraints:
                        speed = constraint.getMotor1D().getSpeed()
                        speed = max( 1 - t / control.deceleration_time, 0.0 ) * speed if control.deceleration_time > 0 else 0
                        all_zero &= abs( speed ) <= 1.0E-6
                        set_speed( speed if abs( speed ) > 1.0E-6 else 0.0 )
                    return all_zero

                if input_data.down:
                    try:
                        if input_data.decelerate_callback:
                            StepEventCallback.remove( input_data.decelerate_callback )
                            input_data.decelerate_callback = None
                    except:
                        pass
                    speed = min( input_data.timeDown / control.acceleration_time, 1.0 ) * control.speed
                    set_speed( speed )
                elif input_data.isKeyUp:
                    input_data.decelerate_callback = decelerate_callback
                    StepEventCallback.callFor( control.deceleration_time + 1.0, input_data.decelerate_callback )

            return on_keyboard

        if self.m_keyboard_controls:
            StepEventCallback.remove( self.m_keyboard_controls.engine.engine_state_handler )

            for name, control in self.m_keyboard_controls:
                if name is 'engine':
                    Input.unbind( name + '.forward' )
                    Input.unbind( name + '.reverse' )
                else:
                    Input.unbind( name )

        self.m_keyboard_controls = controls
        if controls is None:
            return

        for name, control in self.keyboard_controls:
            if not isinstance( control, Struct ):
                continue

            if name is 'engine':
                callback = engine_keyboard_control_callback( control )
                Input.bind( name     = name + '.forward',
                            key      = control.forward.key,
                            callback = callback )
                Input.bind( name     = name + '.reverse',
                            key      = control.reverse.key,
                            callback = callback )
            else:
                control.constraints = self.tire_hinges\
                                        if name in ('forward', 'reverse')\
                                        else [self.steering_hinge] if 'steer' in name\
                                        else self.elevate_prismatics if 'elevate' in name\
                                        else self.tilt_prismatics if 'tilt' in name\
                                        else None
                if control.constraints is None:
                    print( 'Unable to keyboard bind: {}.'.format( name ) )
                else:
                    Input.bind( name     = name,
                                key      = control.key,
                                callback = keyboard_control_callback( control ) )

    @gamepad_controls.setter
    def gamepad_controls(self, controls):
        """
        Assign new gamepad controls - any previous settings will be removed.

        Arguments:
            control: Struct - gamepad controls, None to deactivate
        """
        if controls is None and self.m_gamepad_controls is None:
            return

        def engine_gamepad_control_callback(control: Struct) -> callable:
            """
            Defines on-axis callback for the engine.
            """
            is_single_axis = control.forward.axis == control.reverse.axis
            state = self._engine_state_handler( control )

            def on_axis(data: Gamepad.AxisData):
                value  = data.value if abs( data.value ) >= control.deadzone else 0.0
                value *= -1.0 if is_single_axis and control.invert else 1.0
                raw    = -data.value_raw if is_single_axis and control.invert else data.value_raw

                is_forward = ( is_single_axis and raw >= 0 ) or\
                             ( not is_single_axis and data.axis == control.forward.axis )

                abs_value = abs( value )
                # Setting values^2 for smoother feel of the controls.
                if is_forward:
                    state.forward.value = abs_value * abs_value
                else:
                    state.reverse.value = abs_value * abs_value

            return on_axis

        def gamepad_control_callback(control: Struct) -> callable:
            """
            Defines on-axis callback for the constraint controls.
            """
            def set_speed(speed: float):
                for constraint in control.constraints:
                    constraint.getLock1D().setEnable( speed == 0.0 )
                    
                    if (speed != 0):
                        constraint.getLock1D().setPosition( constraint.getAngle() )
                    constraint.getMotor1D().setEnable( not constraint.getLock1D().getEnable() )
                    constraint.getMotor1D().setSpeed( speed )
                    


            def on_axis(data: Gamepad.AxisData):
                sign = -1.0 if control.invert else 1.0
                set_speed( sign * data.value * control.max_speed if abs( data.value ) >= control.deadzone else 0 )

            return on_axis

        # Unbind current controls.
        if self.m_gamepad_controls:
            StepEventCallback.remove( self.m_gamepad_controls.engine.engine_state_handler )

            for name, control in self.m_gamepad_controls:
                if name is 'engine' and control.forward.axis != control.reverse.axis:
                    Gamepad.unbind( name + '.forward' )
                    Gamepad.unbind( name + '.reverse' )
                else:
                    Gamepad.unbind( name )

        self.m_gamepad_controls = controls
        if controls is None:
            return

        for name, control in self.m_gamepad_controls:
            if not isinstance( control, Struct ):
                continue

            if name is 'engine':
                callback = engine_gamepad_control_callback( control )
                if control.forward.axis == control.reverse.axis:
                    Gamepad.bind( name     = name,
                                  axis     = control.forward.axis,
                                  callback = callback )
                else:
                    Gamepad.bind( name     = name + '.forward',
                                  axis     = control.forward.axis,
                                  callback = callback )
                    Gamepad.bind( name     = name + '.reverse',
                                  axis     = control.reverse.axis,
                                  callback = callback )
            else:
                control.constraints = [self.steering_hinge] if 'steer' in name else\
                                      self.elevate_prismatics if 'elevate' in name else\
                                      self.tilt_prismatics if 'tilt' in name else\
                                      None
                if control.constraints is None:
                    print( 'Unable to gamepad bind: {}'.format( name ) )
                else:
                    Gamepad.bind( name     = name,
                                  axis     = control.axis,
                                  callback = gamepad_control_callback( control ) )
