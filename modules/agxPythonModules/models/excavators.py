"""
Excavator model module containing ExcavatorCAT365

ExcavatorCAT365:
    Driveline:
        - A Motorized hinge per sprocket (Left, Right)
    Default Controls (has to be activated, see examples):
        Keyboard:
            - Left Forward:          Home
            - Left Reverse:          End
            - Right Forward:         PageUp
            - Right Reverse:         PageDown

            - Arm down:   'z'
            - Arm up:     'a'
            - Bucket down: 's'
            - Bucket up:   'x'
            - Boom down:   Down
            - Boom up:   Up
        Gamepad (Xbox like):

            - Arm:   Left_Vertical
            - Bucket: Left_Horizontal
            - Boom:  Right_Vertical
            - Cabin: Right_Horizontal

    Examples:
        from agxPythonModules.utils.callbacks import KeyboardCallback as Input
        from agxPythonModules.models.excavators import ExcavatorCAT365

        excavator_keyboard = ExcavatorCAT365( keyboard_settings = ExcavatorCAT365.default_keyboard_settings() )
        simulation().add( excavator_keyboard )

        excavator_gamepad = ExcavatorCAT365( gamepad_settings = ExcavatorCAT365.default_gamepad_settings() )
        excavator_gamepad.setPosition( 0, -4, 0 )
        simulation().add( excavator_gamepad )

        my_track_settings     = ExcavatorCAT365.default_track_settings()
        my_keyboard_settings  = ExcavatorCAT365.default_keyboard_settings()
        my_driveline_settings    = ExcavatorCAT365.default_driveline_settins()

        # Bouncy tracks.
        my_track_settings.stiffness.radial = 1.0E6


        # Change forward/reverse to 'a'/'z' and elevate/lower bucket to arrow keys up/down.
        my_keyboard_settings.engine.forward.key = ord( 'a' )
        my_keyboard_settings.engine.reverse.key = ord( 'z' )
        my_keyboard_settings.elevate_up.key     = Input.KEY_Up
        my_keyboard_settings.elevate_down.key   = Input.KEY_Down

        # Change max torque for hinge motors
        my_driveline_settings.max_torque = 1E6

        # Create our custom CAT 365 excavator.
        my_excavator = ExcavatorCAT365(   track_settings      = my_track_settings,
                                          driveline_settings  = my_driveline_settings,
                                          keyboard_settings   = my_keyboard_settings )
        simulation().add( my_excavator )
"""

import agxCollide
import agx
import agxSDK
import agxModel
import agxPowerLine
import agxDriveTrain
import agxVehicle
import agxOSG
import agxTerrain
import agxRender
import math
import numpy

from ..tools.read_file import SimulationFile
from ..tools.simulation_content import RigidBodyInfo, DisabledCollisionsStateHandler
from ..utils.environment import root, simulation as global_simulation
from ..utils.callbacks import StepEventCallback, KeyboardCallback as Input, GamepadCallback as Gamepad
from ..utils.constraints import createHinge
from agxPythonModules.utils.Struct import Struct

from enum import Enum

class Track(agxSDK.Assembly):
    """
    
    """

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
            agx.Hinge - sprocket hinge
        """
        return self.m_sprocket_hinge

    

    @property
    def roller_material(self) -> agx.Material:
        """
        Returns:
            agx.Material - roller material.
        """
        return self.m_roller_material

    @property
    def track_material(self) -> agx.Material:
        """
        Track material collected from first geometry with collisions
        enabled and material with name != 'Unknown Material'.

        Returns:
            agx.Material - track material
        """
        return self.m_track.getMaterial()

    @track_material.setter
    def track_material(self, material : agx.Material):
        self.m_track.setMaterial( material )

    def __init__(self, **kwargs):
        """
        Construct given

        Arguments:
            local_transform: agx.AffineMatrix4x4 - [Optional] local transform so that track y axis is the rotation axis.
        """
        super().__init__()

        self.m_sprocket_hinge    = kwargs[ 'sprocket_hinge' ]

        track_material = kwargs.get('track_material', agx.Material("DefaultTrackMaterial"))

        sprocket_body = kwargs[ 'sprocket_body' ]
        self.add(sprocket_body)

        sprocket_body_info = RigidBodyInfo( sprocket_body )
        # We will assume all sprockets/Rollers/Idlers has the same material!
        self.m_roller_material = None

        # Get the material from the first geometry which has collision enabled        
        for g in sprocket_body_info.geometries:
            if (g.collisions_enabled):
                self.m_roller_material = g.instance.getMaterial()
                break

        assert(self.m_roller_material)

        idler_body = kwargs['idler_body']
        self.add(idler_body)

        roller_bodies = kwargs['roller_bodies']            
        local_transform = kwargs.get( 'local_transform', agx.AffineMatrix4x4() )

        track_settings = kwargs.get( 'track_settings')

        self.m_track = agxVehicle.Track( track_settings.track_parameters.number_of_shoes,
                                    track_settings.track_parameters.width,
                                    track_settings.track_parameters.thickness,
                                    track_settings.track_parameters.initial_offset )
        
        self.m_track.setMaterial(track_material)

        radius = Track.find_radius( sprocket_body )
        sprocket = agxVehicle.TrackWheel( agxVehicle.TrackWheel.SPROCKET,
                                          radius, sprocket_body, local_transform )
        self.m_track.add( sprocket )

        radius = Track.find_radius( idler_body )
        idler = agxVehicle.TrackWheel( agxVehicle.TrackWheel.IDLER,
                                          radius, idler_body, local_transform )
        self.m_track.add( idler )

        idler.getLocalFrame().setLocalMatrix( agx.AffineMatrix4x4.rotate( agx.Vec3.Y_AXIS(),
                                                                          agx.Vec3.X_AXIS() ) )               
        for b in roller_bodies:
            self.add(b)
            radius = Track.find_radius( b )
            roller = agxVehicle.TrackWheel( agxVehicle.TrackWheel.ROLLER,
                                            radius, b, local_transform )

            roller.setEnableProperty( agxVehicle.TrackWheel.SPLIT_SEGMENTS, True )

            self.m_track.add( roller )

        track_merge_parameters_settings = track_settings.track_parameters.internal_merge

        mergeProperties = self.m_track.getInternalMergeProperties()
        mergeProperties.setEnableMerge( track_merge_parameters_settings.enabled)
        mergeProperties.setNumNodesPerMergeSegment( track_merge_parameters_settings.number_of_nodes_per_segment )
        mergeProperties.setEnableLockToReachMergeCondition( track_merge_parameters_settings.use_lock )
        mergeProperties.setLockToReachMergeConditionCompliance( track_merge_parameters_settings.lock_compliance )
        mergeProperties.setLockToReachMergeConditionDamping( track_merge_parameters_settings.lock_damping )
        mergeProperties.setMaxAngleMergeCondition( track_merge_parameters_settings.merge_angle )
        mergeProperties.setContactReduction( track_merge_parameters_settings.contact_reduction_level )
        
        track_properties_settings = track_settings.track_parameters.properties
        properties = self.m_track.getProperties()
        properties.setHingeCompliance( track_properties_settings.compliance )
        properties.setHingeDamping( track_properties_settings.damping )
        properties.setEnableHingeRange( track_properties_settings.range_enable )
        properties.setHingeRangeRange( track_properties_settings.range_range.lower,
                                       track_properties_settings.range_range.upper)
        
        properties.setEnableOnInitializeMergeNodesToWheels( track_properties_settings.on_initialize_merge_nodes_to_wheels )
        properties.setEnableOnInitializeTransformNodesToWheels( track_properties_settings.on_initialize_transform_nodes_to_wheels )
        properties.setTransformNodesToWheelsOverlap( track_properties_settings.target_node_wheel_overlap )
        properties.setNodesToWheelsMergeThreshold( track_properties_settings.node_wheel_merge_threshold )
        properties.setNodesToWheelsSplitThreshold( track_properties_settings.node_wheel_split_threshold)
        properties.setNumNodesIncludedInAverageDirection( track_properties_settings.num_nodes_in_averge_direction)
        properties.setMinStabilizingHingeNormalForce( track_properties_settings.min_stabilizing_normal_force)
        properties.setStabilizingHingeFrictionParameter( track_properties_settings.stabilizing_friction_parameter)
        

        self.add(self.m_track)

        self.m_track.initialize()

        print( "Loading shoe visual: ", track_settings.shoe_visual_filename)

        shoe_visual_data =  agxOSG.readNodeFile( track_settings.shoe_visual_filename, False )
        assert( shoe_visual_data )
        print( "  ...done." )

        shoe_visual_transform = agxOSG.Transform()
        shoe_visual_transform.addChild( shoe_visual_data )
        nodes = self.m_track.nodes()
        it = nodes.begin()
        while it != nodes.end():
            node = it.get()
            gn = agxOSG.GeometryNode( node.getRigidBody().getGeometries()[ 0 ] )
            gn.addChild( shoe_visual_transform )
            root().addChild( gn )

            it.inc()


class ExcavatorCAT365(agxSDK.Assembly):
    """
    Excavator class that loads 'model_path' and configures
    tracks and driveline etc. The tracks are listed as:
        [left, right]

    
    Examples:
        from agxPythonModules.utils.callbacks import KeyboardCallback as Input
        from agxPythonModules.models.excavators import ExcavatorCAT365

        track_settings = ExcavatorCAT365.default_track_settings()
        track_settings.stiffness.radial = 1.0E6

        keyboard_settings = ExcavatorCAT365.default_keyboard_settings()
        keyboard_settings.steer_right_forward.key = Input.KEY_PageUp
        keyboard_settings.steer_left_forward.key = Input.KEY_Home

        excavator1 = ExcavatorCAT365(   track_settings     = track_settings,
                                        keyboard_settings = keyboard_settings )
        excavator1.setPosition( 0, 5, 0 )
        simulation().add( excavator1 )

        excavator2 = ExcavatorCAT365()
        simulation().add( excavator2 )
    """

    model_path = 'data/models/excavator_CAT365.agx'

    class Location(Enum):
        LEFT  = 0
        RIGHT = 1

    @classmethod
    def default_driveline_settings(cls):
        """
        Returns:
            Struct - default driveline settings
        """
        return Struct(
            {
                'hinge': {
                    'max_torque': 3E5, # Nm
                    'compliance': 1E-6,
                    'speed': 0,
                    'lockedAtZeroSpeed': True
                }              
            }
        )  

    @classmethod
    def default_track_settings(cls):
        """
        Returns:
            Struct - default trac settings.
        """
        return Struct(
            {
                'shoe_visual_filename': "data/models/excavator_CAT365_shoe.obj",
                'chassis_name'       : "UnderCarriage",
                'track_parameters'   : 
                    {
                        'number_of_shoes': 90,
                        'width'        : 0.58,
                        'thickness'    : 0.05,
                        'initial_offset': 0.0015, # tension distance
                        'properties'   : 
                            {
                                'compliance'                         : 8.0E-10,
                                'damping'                            : 2.0 / 60.0,
                                'range_enable'                        : True,
                                'range_range'                         : 
                                    {
                                        'lower' : math.radians( -75 ),
                                        'upper' : math.radians( 20 )
                                    },
                                'on_initialize_merge_nodes_to_wheels'     : False,
                                'on_initialize_transform_nodes_to_wheels' : True,
                                'target_node_wheel_overlap'             : 1.0E-3,
                                'node_wheel_merge_threshold'            : -0.1,
                                'node_wheel_split_threshold'            : -0.05,
                                'num_nodes_in_averge_direction'          : 3,
                                'min_stabilizing_normal_force'          : 1.0E5,
                                'stabilizing_friction_parameter'       : 1
                            },
                        'internal_merge' : 
                            {
                                'enabled'                 : True,
                                'number_of_nodes_per_segment' : 2,
                                'merge_angle'              : 1.0E-5,
                                'use_lock'                 : True,
                                'lock_compliance'          : 1.0E-8,
                                'lock_damping'             : 0.05,
                                'contact_reduction_level'   : agxVehicle.TrackInternalMergeProperties.MODERATE
                            },

                    }
            }
        )

    @classmethod
    def default_keyboard_settings(cls):
        """
        Default keyboard settings:
            KEY_Up:    forward
            KEY_Down:  reverse
            KEY_Left:  steer left
            KEY_Right: steer right
            'a':       arm elevate up
            'z':       arm elevate down
            's':       bucket tilt up
            'x':       bucket tilt down

        Returns:
            Struct - default keyboard controls settings.
        """
        return Struct(
            {
                'steer_right_forward': { 
                    'key':               Input.KEY_Page_Up,
                    'speed':             10.0,
                    'acceleration_time': 0.5,
                    'deceleration_time': 0.5
                },
                'steer_right_backward': { 
                    'key':               Input.KEY_Page_Down,
                    'speed':             -1.0,
                    'acceleration_time': 0.5,
                    'deceleration_time': 0.5
                },
                'steer_left_forward': { 
                    'key':               Input.KEY_Home,
                    'speed':             10.0,
                    'acceleration_time': 0.5,
                    'deceleration_time': 0.5
                },
                'steer_left_backward': { 
                    'key':               Input.KEY_End,
                    'speed':             -1.0,
                    'acceleration_time': 0.5,
                    'deceleration_time': 0.5
                },
                'bucket_up': {
                    'key':               ord( 'a' ),
                    'speed':             -0.45,
                    'acceleration_time': 0.6,
                    'deceleration_time': 1.8
                },
                'bucket_down': {
                    'key':               ord( 'z' ),
                    'speed':             0.45,
                    'acceleration_time': 0.6,
                    'deceleration_time': 1.8
                },
                'arm_up': {
                    'key':               ord( 's' ),
                    'speed':             -0.35,
                    'acceleration_time': 0.2,
                    'deceleration_time': 0.8
                },
                'arm_down': {
                    'key':               ord( 'x' ),
                    'speed':             0.35,
                    'acceleration_time': 0.2,
                    'deceleration_time': 0.7
                },
                'boom_up': {
                    'key':               Input.KEY_Up,
                    'speed':             0.3,
                    'acceleration_time': 0.7,
                    'deceleration_time': 0.8
                },
                'boom_down': {
                    'key':               Input.KEY_Down,
                    'speed':             -0.3,
                    'acceleration_time': 0.7,
                    'deceleration_time': 0.8
                },
                'cabin_rotate_right': {
                    'key':               Input.KEY_Right,
                    'speed':             0.35,
                    'acceleration_time': 0.5,
                    'deceleration_time': 0.5
                },
                'cabin_rotate_left': {
                    'key':               Input.KEY_Left,
                    'speed':             -0.35,
                    'acceleration_time': 0.5,
                    'deceleration_time': 0.5
                }
            }
        )

    @classmethod
    def default_gamepad_controls(cls, deadzone: float = 0.2):
        return Struct(
            {
                'bucket_up': {
                    'axis':      Gamepad.Axis.LeftHorizontal,
                    'deadzone':  deadzone,
                    'invert':    False,
                    'max_speed': 0.45
                },
                'arm_up': {
                    'axis':      Gamepad.Axis.LeftVertical,
                    'deadzone':  deadzone,
                    'invert':    False,
                    'max_speed': 0.35
                },
                'boom_up': {
                    'axis':      Gamepad.Axis.RightVertical,
                    'deadzone':  deadzone,
                    'invert':    False,
                    'max_speed': -0.2
                },
                'cabin_rotate_right': {
                    'axis':      Gamepad.Axis.RightHorizontal,
                    'deadzone':  deadzone,
                    'invert':    False,
                    'max_speed': 0.6806784074999989 #6.5 rpm
                }
            }
        )

    def get_mass(self):
        return self.chassie_body.getMassProperties().getMass() + self.under_carriage_body.getMassProperties().getMass()

    @property
    def chassie_body(self) -> agx.RigidBody:
        """
        Returns:
            agx.RigidBody - chassie body of this excavator.
        """
        return self.m_body_chassie

    @property
    def under_carriage_body(self) -> agx.RigidBody:
        """
        Returns:
            agx.RigidBody - chassie body of this excavator.
        """
        return self.m_body_under_carriage


    @property
    def bucket_body(self) -> agx.RigidBody:
        """
        Returns:
            agx.RigidBody - bucket body of this excavator.
        """
        return self.m_body_bucket

    @property
    def bucket_material(self) -> agx.Material:
        """
        Returns:
            agx.Material - material for the bucket.
        """
        return self.m_material_bucket

    @property
    def speed(self) -> float:
        """
        Speed of this excavator, taken from chassie body
        when gear is not in reverse - otherwise from rear
        body when gear is reverse.

        Returns:
            float - speed of this excavator.
        """
        vel = self.m_under_carriage_observer.getVelocity()

        speed = self.m_under_carriage_observer.transformVectorToLocal(vel)[1]
        return speed

    @property
    def tracks(self) -> [Track]:
        """
        Returns:
            [Track] - list of tracks.
        """
        return self.m_tracks

    @property
    def track_material(self) -> agx.Material:
        """
        Returns:
            agx.Material - track material.
        """
        return self.track( self.Location.LEFT ).material

    def track(self, location) -> Track:
        """
        Track given location or index.

        Examples:
            track1 = excavator.track( 0 )
            trac1_again = excavator.track( Track.Location.LEFT )
            assert track1 == track1_again

        Arguments:
            location: int, Track.Location - track location [left, right]

        Returns:
            Track - track at given location.
        """
        try:
            return self.tracks[ location.value ]
        except AttributeError:
            return self.tracks[ location ]


 
    @property 
    def boom_prismatics(self) -> [agx.Prismatic]:
        return self.m_boom_prismatics

    @property 
    def bucket_prismatic(self) -> agx.Prismatic:
        return self.m_bucket_prismatic
    
    @property
    def arm_prismatic(self) -> agx.Prismatic:
        return self.m_arm_prismatic

    @property
    def sprocket_hinges(self) -> [agx.Hinge]:
        """
        Returns:
            [agx.Hinge] - sprocket hinges
        """
        return self.m_sprocket_hinges
    
    @property
    def cabin_hinge(self) -> [agx.Hinge]:
        """
        Returns:
            [agx.Hinge] - cabin hinge
        """
        return self.m_cabin_hinge
    
    def sprocket_hinge(self, location: Location) -> agx.Hinge:
        """
        Returns:
            [agx.Hinge] - list of track <-> chassis hinges.
        """
        return self.m_sprocket_hinges[location.value]

    def sprocket_body(self, location: Location) -> agx.RigidBody:
        """
        Returns:
            agx.RigidBody - specified rigid body
        """

        return self.m_body_sprockets[location.value]

    def idler_body(self, location : Location) -> agx.RigidBody:
        """
        Returns:
            agx.RigidBody - specified rigid body
        """
        return self.m_body_idlers[location.value]

    def roller_bodies(self, location : Location) -> [agx.RigidBody]:
        """
        Returns:
            [agx.RigidBody] - List of bodies
        """
        if (location == self.Location.LEFT):
            return self.m_left_roller_bodies
        else:
            return self.m_right_roller_bodies

    @property
    def keyboard_controls(self):
        """
        Examples:
            excavator = ExcavatorCAT365( keyboard_controls = ExcavatorCAT365.default_keyboard_settings() )
            kb = excavator.keyboard_controls
            kb.engine.forward.key = Input.KEY_Space
            # Assign again for the changes to have an effect.
            excavator.keyboard_controls = kb

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

    def setTrackMaterial( self, material ):
        """
        Set the material of the tracks

        Arguments:
            material: The new material for the tracks
        """
        for track in self.tracks:
            track.material = material

    def get_tracks(self) -> [agxVehicle.Track]:
        """
        Returns:
            Vector of tracks for the vehicle
        """
        return self.m_tracks

    def configure_track_roller_contact_materials(self, track_material, roller_material):
        """
        Set the contact material between the rollers and the track material
        """
        track_roller_cm = global_simulation().getMaterialManager().getOrCreateContactMaterial( track_material, roller_material )
        track_roller_cm.setRestitution( 0 )
        track_roller_cm.setFrictionCoefficient( 1 )
        track_roller_cm.setSurfaceViscosity(1E-3)
        track_roller_cm.setYoungsModulus( 4.0E8 )
        track_roller_cm.setDamping( 0.05 )
        track_roller_cm.setUseContactAreaApproach( False )


    def configure_track_ground_contact_materials(self, ground) -> [agx.ContactMaterial]:
        """
        Utility: Configures ground <-> tracks contact materials given ground
        object (agx.Material, agxCollide.Geometry or agx.RigidBody) and
        returns a list of the created contact materials. The contact materials
        are activated in the simulation.

        Arguments:
            ground: agx.Material, agxCollide.Geometry, agx.RigidBody - ground object
        """
        ground_materials = [ground] if isinstance( ground, agx.Material ) else\
                           [ground.getMaterial()] if isinstance( ground, agxCollide.Geometry ) else\
                           [ground.getMaterial(agxTerrain.Terrain.MaterialType_TERRAIN)] if isinstance( ground, agxTerrain.Terrain ) else\
                           [geometry.getMaterial() for geometry in obj.getGeometries()] if isinstance( ground, agx.RigidBody ) else\
                           None
        if ground_materials is None:
            raise TypeError( 'Unable to find material for type: {}'.format( type( obj ) ) )

        track_materials = [track.track_material for track in self.m_tracks]
        track_ground_contact_materials = []
        for ground_material in ground_materials:
            for track_material in track_materials:
                track_ground_cm = global_simulation().getMaterialManager().getOrCreateContactMaterial( track_material,
                                                                                                       ground_material ) # type: agx.ContactMaterial


                track_ground_cm.setRestitution( 0 )
                track_ground_cm.setFrictionCoefficient( 1.0, agx.ContactMaterial.PRIMARY_DIRECTION )
                track_ground_cm.setFrictionCoefficient( 0.25, agx.ContactMaterial.SECONDARY_DIRECTION )
                track_ground_cm.setSurfaceViscosity( 1.0E-6, agx.ContactMaterial.PRIMARY_DIRECTION )
                track_ground_cm.setSurfaceViscosity( 6.0E-6, agx.ContactMaterial.SECONDARY_DIRECTION )
                
                # Track ground uses a very specific friction model for performance purposes.
                # It requires the mass of the machine 
                track_ground_cm.setFrictionModel( agx.ConstantNormalForceOrientedBoxFrictionModel( 0.5 * self.get_mass(),
                                                                                                            self.chassie_body.getFrame(),
                                                                                                            agx.Vec3.Y_AXIS(),
                                                                                                            agx.FrictionModel.DIRECT,
                                                                                                            False ) )

                track_ground_cm.setYoungsModulus( 1.0E10 )
                track_ground_contact_materials.append( track_ground_cm )

        return track_ground_contact_materials

    def __init__(self, **kwargs):
        """
        Construct given optional arguments.

        Arguments:
            driveline_settings: Struct - driveline settings, ExcavatorCAT365.default_driveline_settings() is used if not given.
            track_settings: Struct - track settings, ExcavatorCAT365.default_track_settings() is used if not given.
            keyboard_controls: Struct - keyboard controls (e.g., ExcavatorCAT365.default_keyboard_settings())
            gamepad_controls: Struct - gamepad controls (e.g., ExcavatorCAT365.default_gamepad_controls())
        """
        super().__init__()

        temp_simulation = agxSDK.Simulation()

        data = SimulationFile( simulation = temp_simulation, root = root() )
        if not data.load( filename = self.model_path, parent = self ):
            raise FileNotFoundError( 'Unable to load excavator model: {}'.format( self.model_path ) )

        states = []
        for rb in self.getRigidBodies():
            states.append( rb.getEnable() )

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

        for i, rb in enumerate( self.getRigidBodies() ):
            rb.setEnable( states[ i ] )

       
    @property
    def cutting_edge(self):
        return agx.Line( self.m_cutting_edge_observers[0].getLocalPosition(),self.m_cutting_edge_observers[1].getLocalPosition() )

    @property
    def top_edge(self):
        return agx.Line( self.m_top_edge_observers[0].getLocalPosition() ,self.m_top_edge_observers[1].getLocalPosition() )

    def render_debug(self):
        agxRender.RenderSingleton.instance().addStatic(self.m_forward_cutting_observer.getPosition(), 
        self.m_forward_cutting_observer.getPosition()+self.m_forward_cutting_observer.transformVectorToWorld(agx.Vec3(0,1,0))*0.5,
        0.05, agxRender.Color.Blue() )


    @property
    def forward_cutting_vector(self):
        v = self.bucket_body.getFrame().transformVectorToLocal(self.m_forward_cutting_observer.transformVectorToWorld(agx.Vec3(0,1,0)))
        return v

    def initialize_bucket_data(self, data):
        
        self.m_cutting_edge_observers = [data.observers['CuttingEdgeBegin'], data.observers['CuttingEdgeEnd']]
        self.m_top_edge_observers = [data.observers['TopEdgeBegin'], data.observers['TopEdgeEnd']]

        forward_observer = data.observers['ForwardCutVector']
        self.m_forward_cutting_observer = forward_observer

        self.m_material_bucket = data.materials['BucketMaterial']        

    def _initialize(self, data: SimulationFile, **kwargs):

        self.m_body_chassie  = data.bodies[ 'ChassieBody' ]
        self.m_body_under_carriage   = data.bodies[ 'UnderCarriageBody' ]
        self.m_body_bucket = data.bodies[ 'Bucket' ]
        self.m_body_sprockets = [data.bodies['LeftFrontSprocket'], data.bodies['RightFrontSprocket']]

        self.m_body_idlers = [data.bodies['LeftRearIdler'], data.bodies['RightRearIdler']]
        self.m_sprocket_hinges = [data.constraints['LeftSprocketHinge'].asHinge(), data.constraints['RightSprocketHinge'].asHinge()]

        self.m_left_roller_bodies = []
        self.m_right_roller_bodies = []
        for i in range(1,10):
            name = 'RightRoller'+str(i)
            if name in data.bodies:
                self.m_right_roller_bodies.append(data.bodies[name])
            name = 'LeftRoller'+str(i)
            if name in data.bodies:
                self.m_left_roller_bodies.append(data.bodies[name])


        self.m_under_carriage_observer = data.observers['UnderCarriageObserver']


        # Creating tracks: [left,right]
        # The local transform makes sure the y axes are pointing out from
        # the track center positions.
        local_track_transform = agx.AffineMatrix4x4.rotate( agx.Vec3.Z_AXIS(), agx.Vec3.Y_AXIS() )
        track_settings = kwargs.get( 'track_settings', self.default_track_settings() )

        track_material = agx.Material("TrackMaterial")

        self.m_tracks = [
            Track(  track_settings = track_settings,
                    sprocket_hinge = self.sprocket_hinge(self.Location.LEFT),
                    sprocket_body  = self.sprocket_body(self.Location.LEFT),
                    idler_body      = self.idler_body(self.Location.LEFT),
                    roller_bodies   = self.roller_bodies(self.Location.LEFT),
                    local_transform = local_track_transform,
                    track_material = track_material ),
            
            Track(  track_settings = track_settings,
                    sprocket_hinge = self.sprocket_hinge(self.Location.RIGHT),
                    sprocket_body  = self.sprocket_body(self.Location.RIGHT),
                    idler_body      = self.idler_body(self.Location.RIGHT),
                    roller_bodies   = self.roller_bodies(self.Location.RIGHT),
                    local_transform = local_track_transform,
                    track_material = track_material ) 

            ]
        
        
        for t in self.m_tracks:
            self.add(t)
            t.track_material = track_material


        self.configure_track_roller_contact_materials(self.m_tracks[0].track_material, self.m_tracks[0].roller_material)        

        self.initialize_bucket_data(data)


        ###################################################################################
        #                                  Drivetrain                                     #
        ###################################################################################
        
        ###################################################################################
        driveline_settings = kwargs.get( 'driveline_settings', self.default_driveline_settings() )

        def setup_sprocket_hinge(hinge, driveline_settings):
            hinge.getMotor1D().setForceRange( agx.RangeReal(driveline_settings.max_torque))
            hinge.getMotor1D().setCompliance( driveline_settings.compliance )
            hinge.getMotor1D().setSpeed( driveline_settings.speed )
            hinge.getMotor1D().setLockedAtZeroSpeed( driveline_settings.lockedAtZeroSpeed )

        setup_sprocket_hinge(self.sprocket_hinge(self.Location.LEFT), driveline_settings.hinge)
        setup_sprocket_hinge(self.sprocket_hinge(self.Location.RIGHT), driveline_settings.hinge)


        # Get the constraints for controlling the cabin/arm/boom
        self.m_boom_prismatics   = [data.constraints['BoomPrismatic1'].asPrismatic(), data.constraints['BoomPrismatic2'].asPrismatic()]
        self.m_bucket_prismatic = data.constraints['BucketPrismatic'].asPrismatic()
        self.m_arm_prismatic   = data.constraints['ArmPrismatic'].asPrismatic()
        self.m_cabin_hinge = data.constraints['CabinHinge'].asHinge()


        self.m_keyboard_controls = None
        self.m_gamepad_controls  = None
        self.keyboard_controls   = kwargs.get( 'keyboard_controls', None )
        self.gamepad_controls    = kwargs.get( 'gamepad_controls', None )

        
    @keyboard_controls.setter
    def keyboard_controls(self, controls):
        """
        Assign new keyboard controls, any previous settings will be removed.

        Arguments:
            control: Struct - keyboard controls, None to deactivate
        """        

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


        self.m_keyboard_controls = controls
        if controls is None:
            return

        for name, control in self.keyboard_controls:
            if not isinstance( control, Struct ):
                continue


            control.constraints =   [self.sprocket_hinge(self.Location.LEFT)] if 'steer_left' in name\
                                    else [self.sprocket_hinge(self.Location.RIGHT)] if 'steer_right' in name\
                                    else [self.cabin_hinge] if 'cabin' in name\
                                    else [self.bucket_prismatic] if 'bucket' in name\
                                    else [self.arm_prismatic] if 'arm' in name\
                                    else self.boom_prismatics if 'boom' in name\
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
        self.m_gamepad_controls = controls
        if controls is None:
            Gamepad.unbind_all()
            return


        for name, control in self.m_gamepad_controls:
            if not isinstance( control, Struct ):
                continue

            control.constraints =   [self.sprocket_hinge(self.Location.LEFT)] if 'steer_left' in name\
                                    else [self.sprocket_hinge(self.Location.RIGHT)] if 'steer_right' in name\
                                    else [self.cabin_hinge] if 'cabin' in name\
                                    else [self.bucket_prismatic] if 'bucket' in name\
                                    else [self.arm_prismatic] if 'arm' in name\
                                    else self.boom_prismatics if 'boom' in name\
                                    else None
            if control.constraints is None:
                print( 'Unable to gamepad bind: {}'.format( name ) )
            else:
                Gamepad.bind( name     = name,
                                axis     = control.axis,
                                callback = gamepad_control_callback( control ) )

                                
