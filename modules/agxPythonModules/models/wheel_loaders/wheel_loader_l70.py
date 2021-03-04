"""
WheelLoaderL70:
    Driveline:
        - Combustion engine
        - Torque converter
        - Gear box
        - Three differentials (center, front and rear)
        - Four rotational actuators - one at each tire hinge.
    Default Controls (has to be activated, see examples):
        Keyboard:
            - Forward:          Arrow key up
            - Reverse:          Arrow key down
            - Steer left:       Arrow key left
            - Steer right:      Arrow key right
            - Bucket elevate:   'a'
            - Bucket lower:     'z'
            - Bucket tilt up:   's'
            - Bucket tilt down: 'x'
        Gamepad (Xbox like):
            - Forward:          Right trigger
            - Reverse:          Left trigger
            - Steer:            Left horizontal axis
            - Bucket elevate:   Right vertical axis
            - Bucket tilt:      Right horizontal axis

    Examples:
        from agxPythonModules.utils.callbacks import KeyboardCallback as Input
        from agxPythonModules.models.wheel_loaders import WheelLoaderL70

        wheel_loader_keyboard = WheelLoaderL70( keyboard_controls = WheelLoaderL70.default_keyboard_controls() )
        simulation().add( wheel_loader_keyboard )

        wheel_loader_gamepad = WheelLoaderL70( gamepad_controls = WheelLoaderL70.default_gamepad_controls() )
        wheel_loader_gamepad.setPosition( 0, -4, 0 )
        simulation().add( wheel_loader_gamepad )

        my_tire_settings      = WheelLoaderL70.default_tire_settings()
        my_driveline_settings = WheelLoaderL70.default_driveline_settings()
        my_keyboard_controls  = WheelLoaderL70.default_keyboard_controls()

        # Bouncy tires.
        my_tire_settings.stiffness.radial = 1.0E6

        # Large displacement volume.
        my_driveline_settings.engine.displacement_volume *= 2.0

        # Lower gear ratio = not as strong but faster.
        my_driveline_settings.gear_box.gear_ratios = [ 0.5 * ratio for ratio in my_driveline_settings.gear_box.gear_ratios ]

        # Change forward/reverse to 'a'/'z' and elevate/lower bucket to arrow keys up/down.
        my_keyboard_controls.engine.forward.key = ord( 'a' )
        my_keyboard_controls.engine.reverse.key = ord( 'z' )
        my_keyboard_controls.elevate_up.key     = Input.KEY_Up
        my_keyboard_controls.elevate_down.key   = Input.KEY_Down

        # Create our custom wheel loader L70.
        my_wheel_loader = WheelLoaderL70( tire_settings      = my_tire_settings,
                                          driveline_settings = my_driveline_settings,
                                          keyboard_controls  = my_keyboard_controls )
        simulation().add( my_wheel_loader )
"""

import agx

from .wheel_loader import WheelLoader, Struct, Tire
from agxPythonModules.utils.callbacks import KeyboardCallback as Input, GamepadCallback as Gamepad

class WheelLoaderL70(WheelLoader):
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

    Examples:
        from agxPythonModules.utils.callbacks import KeyboardCallback as Input
        from agxPythonModules.models.wheel_loaders import WheelLoaderL70

        tire_settings = WheelLoaderL70.default_tire_settings()
        tire_settings.stiffness.radial = 1.0E6

        keyboard_settings = WheelLoaderL70.default_keyboard_settings()
        keyboard_settings.engine.forward.key = ord( 'a' )
        keyboard_settings.engine.reverse.key = ord( 'z' )
        keyboard_settings.elevate_up.key = Input.KEY_Up
        keyboard_settings.elevate_down.key = Input.KEY_Down

        wheel_loader1 = WheelLoaderL70( tire_settings     = tire_settings,
                                        keyboard_settings = keyboard_settings )
        wheel_loader1.setPosition( 0, 5, 0 )
        simulation().add( wheel_loader1 )

        wheel_loader2 = WheelLoaderL70()
        simulation().add( wheel_loader2 )
    """

    model_path         = 'data/models/wheel_loader_L70.agx'
    model_top_edge     = agx.Line( agx.Vec3( -1.3, 0.858258, 0.583871 ),
                                   agx.Vec3( 1.3, 0.858258, 0.583871 ) )
    model_cutting_edge = agx.Line( agx.Vec3( -1.3, -0.763665, 0.433567 ),
                                   agx.Vec3( 1.3, -0.763665, 0.433567 ) )
    model_cutting_dir  = agx.Vec3( 0, -0.473151, 0.880981 ).normal()

    @classmethod
    def default_driveline_settings(cls):
        """
        Returns:
            Struct - default driveline settings
        """
        return Struct(
            {
                'engine': {
                    'displacement_volume':   0.015,
                    'volumetric_efficiency': 0.9
                },
                'torque_converter': {
                    'max_multiplication':    2.0,
                    'reference_rpm':         1000,
                    'multiplication_table':  [ ( -0.0001, 0.00 ),
                                               ( 0.00001, 0.50 ),
                                               ( 0.00011, 2.00 ),
                                               ( 0.00100, 2.00 ),
                                               ( 0.20000, 1.10 ),
                                               ( 0.40000, 1.15 ),
                                               ( 0.60000, 1.05 ),
                                               ( 0.80000, 1.01 ),
                                               ( 0.90000, 0.99 ),
                                               ( 1.00000, 0.98 ),
                                               ( 1.00100, 0.98 ) ]
                },
                'gear_box': {
                    # Currently only one reverse and one forward gear.
                    'gear_ratios':           [-10.0, 10.0]
                },
                'center_differential': {
                    'gear_ratio':            10.0,
                    # Locking center differential for even torque distribution over
                    # front and rear tires. Otherwise it's likely to have one tire
                    # spinning - especially operating in slopes.
                    'locked':                True
                },
                'front_differential': {
                    'gear_ratio':            1.0,
                    'locked':                False
                },
                'rear_differential': {
                    'gear_ratio':            1.0,
                    'locked':                False
                }
            }
        )

    @classmethod
    def default_tire_settings(cls):
        """
        Returns:
            Struct - default tire settings.
        """
        return Struct(
            {
                'stiffness': {
                    'radial':    2.0E6,
                    'lateral':   4.0E6,
                    'bending':   1.0E6,
                    'torsional': 1.0E6
                },
                'damping_coefficient': {
                    'radial':    9.0E4,
                    'lateral':   9.0E4,
                    'bending':   9.0E4,
                    'torsional': 9.0E4
                }
            }
        )

    @classmethod
    def default_keyboard_controls(cls):
        """
        Default keyboard settings:
            KEY_Up:    forward
            KEY_Down:  reverse
            KEY_Left:  steer left
            KEY_Right: steer right
            'a':       bucket elevate up
            'z':       bucket elevate down
            's':       bucket tilt up
            'x':       bucket tilt down

        Returns:
            Struct - default keyboard controls settings.
        """
        return Struct(
            {
                'engine': {
                    'throttle_increase_rate': 2.0, # When pressing throttle - throttle value is this constant times key-down-time.
                    'throttle_decrease_rate': 4.0, # When releasing throttle - throttle value is one minus this constant times key-released-time.
                    'forward': {
                        'key':           Input.KEY_Up,
                    },
                    'reverse': {
                        'key':           Input.KEY_Down
                    }
                },
                'steer_left': {
                    'key':               Input.KEY_Left,
                    'speed':             -1.0,
                    'acceleration_time': 0.5,
                    'deceleration_time': 0.5
                },
                'steer_right': {
                    'key':               Input.KEY_Right,
                    'speed':             1.0,
                    'acceleration_time': 0.5,
                    'deceleration_time': 0.5
                },
                'elevate_up': {
                    'key':               ord( 'a' ),
                    'speed':             0.35,
                    'acceleration_time': 1.0,
                    'deceleration_time': 1.5
                },
                'elevate_down': {
                    'key':               ord( 'z' ),
                    'speed':             -0.35,
                    'acceleration_time': 1.0,
                    'deceleration_time': 0.5
                },
                'tilt_up': {
                    'key':               ord( 's' ),
                    'speed':             -0.35,
                    'acceleration_time': 0.5,
                    'deceleration_time': 0.5
                },
                'tilt_down': {
                    'key':               ord( 'x' ),
                    'speed':             0.35,
                    'acceleration_time': 0.5,
                    'deceleration_time': 0.5
                }
            }
        )

    @classmethod
    def default_gamepad_controls(cls, deadzone: float = 0.2):
        return Struct(
            {
                'engine': {
                    'invert':   True,
                    'deadzone': deadzone,
                    'forward': {
                        'axis': Gamepad.Axis.RightTrigger
                    },
                    'reverse': {
                        'axis': Gamepad.Axis.LeftTrigger
                    }
                },
                'steer': {
                    'axis':      Gamepad.Axis.LeftHorizontal,
                    'deadzone':  deadzone,
                    'invert':    False,
                    'max_speed': 1.0
                },
                'elevate': {
                    'axis':      Gamepad.Axis.RightVertical,
                    'deadzone':  deadzone,
                    'invert':    True,
                    'max_speed': 0.5
                },
                'tilt': {
                    'axis':      Gamepad.Axis.RightHorizontal,
                    'deadzone':  deadzone,
                    'invert':    False,
                    'max_speed': 0.4
                }
            }
        )

    def __init__(self, **kwargs):
        """
        Construct given optional arguments.

        Arguments:
            driveline_settings: Struct - driveline settings, WheelLoaderL70.default_driveline_settings() is used if not given.
            tire_settings: Struct - tire settings, WheelLoaderL70.default_tire_settings() is used if not given.
            keyboard_controls: Struct - keyboard controls (e.g., WheelLoaderL70.default_keyboard_controls())
            gamepad_controls: Struct - gamepad controls (e.g., WheelLoaderL70.default_gamepad_controls())
        """
        if not 'driveline_settings' in kwargs:
            kwargs[ 'driveline_settings' ] = self.default_driveline_settings()
        if not 'tire_settings' in kwargs:
            kwargs[ 'tire_settings' ] = self.default_tire_settings()
        if not 'model_path' in kwargs:
            kwargs[ 'model_path' ] = self.model_path

        super().__init__( **kwargs )
