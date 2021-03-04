import agxSDK

from ..environment import simulation

_has_agx_sensor = True
try:
    import agxSensor
    from ..environment import application
except ImportError:
    _has_agx_sensor = False

from enum import Enum

class GamepadCallback(agxSensor.JoystickListener if _has_agx_sensor else object):
    """
    Receive callbacks from gamepad axes and button states.

    Axes bindings will be executed in pre-step, every time stepForward is called.

    The trigger axis can either be bound as an axis with values [-1, 1] or separated
    with values [0, 1].

    The native axis values, in whatever range the gamepad reports them, is available as
    value_native in the parameter passed to axis callbacks.

    Remarks:
        When agxSensor isn't available or when there're no gamepads connected
        to the device, GamepadCallback.instance() is None and calls to bind/unbind
        are silently ignored. Handle any warnings in your application/script (see example).

    Examples:
        from agxPythonModules.utils.callbacks import GamepadCallback as Gamepad

        if Gamepad.instance() is None:
            print( 'WARNING: Gamepad controls deactivated.' )

        # Bind button A to callback that prints which button, is down state and
        # the time it has been down.
        Gamepad.bind( name     = 'A',
                      button   = Gamepad.Button.A,
                      callback = lambda data: print( data.button, data.down, data.timeDown ) )
        # Bind left horizontal axis and print normalized and raw values.
        Gamepad.bind( name     = 'An axis',
                      axis     = Gamepad.Axis.LeftHorizontal,
                      callback = lambda data: print( data.axis, data.value, data.value_raw ) )
    """

    class Button(Enum):
        """
        Xbox like gamepad button layout.
        """
        A           = 0
        B           = 1
        X           = 2
        Y           = 3
        LeftBumper  = 4
        RightBumper = 5
        NorthPov    = 6
        EastPov     = 7
        SouthPov    = 8
        WestPov     = 9

        @classmethod
        def from_pov(cls, pov_mask: int):
            pov_list = [agxSensor.JoystickState.NORTH,
                        agxSensor.JoystickState.EAST,
                        agxSensor.JoystickState.SOUTH,
                        agxSensor.JoystickState.WEST]
            buttons = []
            for pov in pov_list:
                unmask = pov_mask & pov
                if unmask:
                    buttons.append( cls.NorthPov if unmask == agxSensor.JoystickState.NORTH else\
                                    cls.EastPov  if unmask == agxSensor.JoystickState.EAST  else\
                                    cls.SouthPov if unmask == agxSensor.JoystickState.SOUTH else\
                                    cls.WestPov  if unmask == agxSensor.JoystickState.WEST  else None )
            return buttons

    class Axis(Enum):
        """
        Xbox like gamepad axis layout.

        Trigger axis:
            Axis.Trigger:      Values (-1, 1), right trigger (-1, 0) and left trigger (0, 1).
            Axis.LeftTrigger:  Values (0, 1)
            Axis.RightTrigger: Values (0, 1)

        It's not valid to bind Trigger if LeftTrigger or RightTrigger is bound.
        It's not valid to bind LeftTrigger or RightTrigger if Trigger is bound.
        """
        LeftVertical    = 0
        LeftHorizontal  = 1
        RightVertical   = 2
        RightHorizontal = 3
        Trigger         = 4
        LeftTrigger     = 5
        RightTrigger    = 6

        @classmethod
        def triggers(cls):
            return (cls.Trigger, cls.LeftTrigger, cls.RightTrigger)

    class ButtonData:
        def __init__(self, name: str, button, callback: callable):
            self.name      = name
            self.button    = button
            self.callback  = callback
            self.down      = False
            self.isKeyDown = False
            self.isKeyUp   = False
            self.timeDown  = 0.0

        def print(self):
            print( '{}:'.format( self.name ) )
            print( '    button:      {}'.format( self.button ) )
            print( '    down:        {}'.format( self.down ) )
            print( '    time down:   {}'.format( self.timeDown ) )
            print( '    is key down: {}'.format( self.isKeyDown ) )
            print( '    is key up:   {}'.format( self.isKeyUp ) )

    class AxisData:
        def __init__(self, name: str, axis, callback: callable):
            self.name = name
            self.axis = axis
            self.callback = callback
            self.value = 0.0
            self.delta = 0.0
            self.value_raw = 0
            self.delta_raw = 0
            self.value_native = 0
            self.delta_native = 0

        def print(self):
            print( '{}:'.format( self.name ) )
            print( '    axis:         {}'.format( self.axis ) )
            print( '    value:        {}'.format( self.value ) )
            print( '    raw value:    {}'.format( self.value_raw ) )
            print( '    native value: {}'.format( self.value_native ) )
            print( '    delta:        {}'.format( self.delta ) )
            print( '    raw delta:    {}'.format( self.delta_raw ) )
            print( '    native delta: {}'.format( self.delta_native ) )

    _instance             = None # type: agxSensor.JoystickListener
    _manager              = None # type: agxSensor.JoystickManager
    _no_devices_connected = not _has_agx_sensor

    @classmethod
    def instance(cls):
        """
        Returns:
            GamepadCallback - initialized instance or None if agxSensor is unavailable or no gamepads connected
        """
        if not _has_agx_sensor or cls._no_devices_connected:
            return None

        if cls._manager and cls._manager.getSimulation():
            assert cls._manager.getSimulation() == simulation()
            assert cls._instance and cls._instance.getManager() == cls._manager
            return cls._instance

        cls._manager = agxSensor.JoystickManager( application().getHWND() )
        simulation().add( cls._manager, agxSDK.EventManager.HIGHEST_PRIORITY )

        if not cls._manager.valid():
            cls._no_devices_connected = True
            simulation().remove( cls._manager )
            cls._manager = None
            return None

        cls._instance = cls()
        cls._manager.add( cls._instance )

        from ..callbacks import StepEventCallback

        StepEventCallback.preCallback( cls._instance._onStepUpdate )

        return cls._instance

    @classmethod
    def bind(cls, **kwargs):
        """
        Bind button or axis given button or axis, name and callback.

        Arguments:
            name: str - unique name of the binding
            axis: GamepadCallback.Axis - axis to bind (if axis, otherwise see button).
            button: GamepadCallback.Button - button to bind (if button, otherwise see axis).
            callback: callable - on update callback
        """
        if cls.instance() is None:
            return

        name     = kwargs[ 'name' ]
        callback = kwargs[ 'callback' ]
        axis     = kwargs.get( 'axis', None )
        button   = kwargs.get( 'button', None )
        if axis is None and button is None:
            raise KeyError( "'button' or 'axis' not given for bind: {}".format( name ) )
        elif axis and button:
            raise KeyError( "Both 'button' and 'axis' given for bind: {}".format( name ) )

        if axis:
            if name in cls.instance().m_axis_bindings:
                raise KeyError( 'Axis name: {} already bound.'.format( name ) )

            if axis in cls.Axis.triggers():
                lr_trigger_bound = cls.Axis.LeftTrigger in cls.instance().m_axis_name_table or\
                                    cls.Axis.RightTrigger in cls.instance().m_axis_name_table

                # Binding trigger when right or left trigger already mapped.
                if axis == cls.Axis.Trigger and lr_trigger_bound:
                    raise KeyError( 'Invalid to bind Axis.Trigger when Axis.LeftTrigger or Axis.RightTrigger is bound.' )
                # Binding left or right trigger when trigger already mapped.
                elif cls.Axis.Trigger in cls.instance().m_axis_name_table:
                    raise KeyError( 'Invalid to bind {} when Axis.Trigger is bound.'.format( axis ) )

            cls.instance().m_axis_bindings[ name ]   = cls.AxisData( name, axis, callback )
            cls.instance().m_axis_name_table[ axis ] = name
        else:
            if name in cls.instance().m_button_bindings:
                raise KeyError( 'Button name: {} already bound.'.format( name ) )

            cls.instance().m_button_bindings[ name ]     = cls.ButtonData( name, button, callback )
            cls.instance().m_button_name_table[ button ] = name

    @classmethod
    def unbind_all(cls):
        """
        Unbind all keys and axis bindings.
        """
        if cls.instance() is None:
            return

        cls.instance().m_axis_bindings.clear()
        cls.instance().m_axis_name_table.clear()

        cls.instance().m_button_bindings.clear()
        cls.instance().m_button_name_table.clear()

    @classmethod
    def unbind(cls, name: str):
        """
        Unbind key or axis binding given name.
        """
        if cls.instance() is None:
            return

        if not name in cls.instance().m_axis_bindings and not name in cls.instance().m_button_bindings:
            print( 'Unable to unbind: {}. Name not bound to axis or button.'.format( name ) )
            return

        if name in cls.instance().m_axis_bindings:
            axis = cls.instance().m_axis_bindings[ name ].axis
            del cls.instance().m_axis_bindings[ name ]
            del cls.instance().m_axis_name_table[ axis ]
        if name in cls.instance().m_button_bindings:
            button = cls.instance().m_button_bindings[ name ].button
            del cls.instance().m_button_bindings[ name ]
            del cls.instance().m_button_name_table[ button ]

    def __init__(self):
        super().__init__()

        self.m_axis_bindings   = {}
        self.m_axis_name_table = {}

        self.m_button_bindings   = {}
        self.m_button_name_table = {}

        self.m_prev_pov = agxSensor.JoystickState.CENTERED

    def buttonChanged(self, state, button_native: int, down: bool) -> bool:
        try:
            button = self.Button( button_native )
        except ValueError:
            return False

        if not button in self.m_button_name_table:
            return False

        button_data = self.m_button_bindings[ self.m_button_name_table[ button ] ]
        if button_data.down != down:
            button_data.down = down
            if down:
                button_data.isKeyDown = True
                button_data.timeDown  = 0.0
            else:
                button_data.isKeyUp = True

        return True

    def axisUpdate(self, state, native_axis: int) -> bool:
        try:
            unhandled_axis = self.Axis( native_axis )
        except ValueError:
            return False

        normalized_value = self._manager.normalize( state.axes[ native_axis ] )

        # If axis is Trigger but Trigger is not bound, check
        # if left or right trigger is mapped.
        axes = []
        if unhandled_axis == self.Axis.Trigger and not unhandled_axis in self.m_axis_name_table:
            axes += [self.Axis.LeftTrigger, self.Axis.RightTrigger]
        else:
            axes.append( unhandled_axis )

        handled = False
        for axis in axes:
            if not axis in self.m_axis_name_table:
                continue

            # Right trigger goes from 0 -> -1 which is confusing
            # when binding to this special axis. Use absolute
            # value.
            is_lr_trigger = axis in (self.Axis.LeftTrigger, self.Axis.RightTrigger)

            axis_data      = self.m_axis_bindings[ self.m_axis_name_table[ axis ] ]
            prev_value     = axis_data.value
            prev_value_raw = axis_data.value_raw
            prev_value_native = axis_data.value_native
            if is_lr_trigger:
                # Right trigger has negative value if pressed.
                if axis == self.Axis.RightTrigger:
                    axis_data.value     = abs( min( normalized_value, 0 ) )
                    axis_data.value_raw = abs( min( state.axes[ native_axis ], 0 ) )
                else:
                    axis_data.value     = abs( max( normalized_value, 0 ) )
                    axis_data.value_raw = abs( max( state.axes[ native_axis ], 0 ) )
            else:
                axis_data.value     = normalized_value
                axis_data.value_raw = state.axes[ native_axis ]

            axis_data.value_native = state.axes[ native_axis ]

            axis_data.delta     = axis_data.value - prev_value
            axis_data.delta_raw = axis_data.value_raw - prev_value_raw
            axis_data.delta_native = axis_data.value_native - prev_value_native

            axis_data.callback( axis_data )

        return handled

    def povMoved(self, state, pov_index: int) -> bool:
        pov          = state.pov[ pov_index ]
        curr_buttons = self.Button.from_pov( pov )
        prev_buttons = self.Button.from_pov( self.m_prev_pov )
        handled      = False
        if self.m_prev_pov != pov:
            added_buttons   = list( set( curr_buttons ) - set( prev_buttons ) )
            removed_buttons = list( set( prev_buttons ) - set( curr_buttons ) )
            for button in added_buttons + removed_buttons:
                handled = self.buttonChanged( state, button.value, button in added_buttons ) or handled

        self.m_prev_pov = pov

        return handled

    def sliderMoved(self, state, slider: int) -> bool:
        return False

    def _onStepUpdate(self, _):
        for _, button_data in self.m_button_bindings.items():
            dt = simulation().getTimeStep()
            if button_data.down or button_data.isKeyUp:
                button_data.callback( button_data )
                button_data.timeDown += dt
                button_data.isKeyDown = False
                button_data.isKeyUp = False
