import agx
import agxCollide
import agxSDK

from ..environment import simulation

class StepEventCallback(agxSDK.StepEventListener):
    """Receive step events in several different ways.

    Examples:
        from agxPythonModules.utils.callbacks import StepEventCallback as sec

        # preCollide, pre, post and last callbacks
        sec.preCollideCallback( lambda t: print( 'preCollide:', t ) )
        # Same as: sec.preCallback
        sec.stepCallback( agxSDK.StepEventListener.PRE_STEP, lambda t: print( 'pre:', t ) )
        sec.postCallback( lambda t: print( 'post:', t ) )
        sec.lastCallback( lambda t: print( 'last:', t ) )

        # Call at simulation time t:
        sec.callAt( 1.0, lambda: print( 'Simulation time is:', simulation().getTimeStamp() ) )

        # Independent of current simulation time, callback in t seconds:
        sec.callIn( 3.0, lambda: print( 'Simulation time is now 3 seconds later.' ) )

        # Sequence callbacks t seconds after last.
        sec.sequence( 1.0, print( 'Simulation time is 1' ) )
        seq.sequence( 0.1, print( '0.1 after previous, simulation time is 1.1.' ) )
    """

    name = '_SecGlobal'
    OUTSIDE_LOOP_CALLBACK = agxSDK.StepEventListener.LAST_STEP + 1
    _instance = None

    @classmethod
    def instance(cls):
        """Finds or creates the single instance of this object.
        
        Returns:
            StepEventCallback -- Instance of this object.
        """
        if cls._instance and cls._instance.getSimulation():
            assert cls._instance.getSimulation() == simulation()
            return cls._instance
        
        cls._instance = cls()
        cls._instance.setName( cls.name )
        simulation().add( cls._instance )
        return cls._instance

    @classmethod
    def preCollideCallback(cls, func: callable):
        """Adds callback to receive calls in PRE_COLLIDE.
        
        Arguments:
            func {float} -- callback with current time stamp as argument.
        """

        cls.instance().step_event_callbacks[ agxSDK.StepEventListener.PRE_COLLIDE ].append( func )

    @classmethod
    def preCallback(cls, func: callable):
        """Adds callback to receive calls in PRE_STEP.
        
        Arguments:
            func {float} -- callback with current time stamp as argument.
        """

        cls.instance().step_event_callbacks[ agxSDK.StepEventListener.PRE_STEP ].append( func )

    @classmethod
    def postCallback(cls, func: callable):
        """Adds callback to receive calls in POST_STEP.
        
        Arguments:
            func {float} -- callback with current time stamp as argument.
        """

        cls.instance().step_event_callbacks[ agxSDK.StepEventListener.POST_STEP ].append( func )

    @classmethod
    def lastCallback(cls, func: callable):
        """Adds callback to receive calls in LAST_STEP.
        
        Arguments:
            func {float} -- callback with current time stamp as argument.
        """

        cls.instance().step_event_callbacks[ agxSDK.StepEventListener.LAST_STEP ].append( func )
    
    @classmethod
    def stepCallback(cls, callback_type, func: callable):
        """Adds callback of given agxSDK.StepEventListener.ActivationMask.
        
        Arguments:
            callback_type -- callback type
            func: callable {float} -- callback with current time stamp as argument.
        """

        cls.instance().step_event_callbacks[ callback_type ].append( func )

    @classmethod
    def updateCallback(cls, func: callable):
        """Adds callback to receive calls in agxSDK.GuiEventListener.UPDATE.
        
        Arguments:
            func: callable {float} -- callback with current time stamp as argument
        """

        cls.instance().step_event_callbacks[ cls.OUTSIDE_LOOP_CALLBACK ].append( func )

    @classmethod
    def sequence(cls, time_delta_from_prev: float, func: callable, **kwargs):
        """Add callback to sequence.
        
        Arguments:
            time_delta_from_prev: float -- Time in seconds from last callback in sequence.
            func: callable -- Callback without arguments timeFromPrev seconds after previous callback in sequence.
            type -- [Optional] Callback type PRE_COLLIDE, PRE_STEP, POST_STEP, LAST_STEP or OUTSIDE_LOOP_CALLBACK,
                    default: PRE_COLLIDE
        """

        cls.instance()._sequence( time_delta_from_prev, func, **kwargs )
    
    @classmethod
    def callAt(cls, time: float, func: callable, **kwargs):
        """Callback at a given simulation time (in seconds).
        
        Arguments:
            time: float -- Simulation time when the callback should be invoked.
            func: callable -- Callback without arguments when the given simulation time has been reached.
            type -- [Optional] Callback type PRE_COLLIDE, PRE_STEP, POST_STEP, LAST_STEP or OUTSIDE_LOOP_CALLBACK,
                    default: PRE_COLLIDE
        """

        cls.instance()._call_at( time, func, **kwargs )
    
    @classmethod
    def callIn(cls, time: float, func: callable, **kwargs):
        """Callback at a given simulation time in seconds from now.
        
        Arguments:
            time: float -- Simulation time from now when callback should be invoked.
            func: callable -- Callback without arguments.
            type -- [Optional] Callback type PRE_COLLIDE, PRE_STEP, POST_STEP, LAST_STEP or OUTSIDE_LOOP_CALLBACK,
                    default: PRE_COLLIDE
        """

        cls.instance()._call_at( simulation().getTimeStamp() + time, func, **kwargs )
    
    @classmethod
    def callFor(cls, time: float, func: callable, **kwargs):
        """Callbacks for a given time duration in seconds from now.
        
        Arguments:
            time: float -- Time duration in which the callback will be called, once each time step from pre-collide.
            func: callable -- Callback with elapsed time as argument.
            type -- [Optional] Callback type PRE_COLLIDE, PRE_STEP, POST_STEP, LAST_STEP or OUTSIDE_LOOP_CALLBACK,
                    default: PRE_COLLIDE
        """

        initTimeStamp = simulation().getTimeStamp()
        def stepEvent(t):
            return func( simulation().getTimeStamp() - initTimeStamp )
        cls.instance().call_for_closures[ func ] = stepEvent
        cls.stepCallback( kwargs.get( 'type', cls.PRE_COLLIDE ), stepEvent )
        cls.callIn( time, lambda: cls.remove( stepEvent ) )
    
    @classmethod
    def remove(cls, func: callable):
        """Remove callback instance from any type of callback (preCollide, pre, post, last, etc.).
        Arguments:
            func: callable -- Callback to remove.
        """
        if func != None:
            if func in cls.instance().call_for_closures:
                cls.instance().to_be_removed.append( cls.instance().call_for_closures[ func ] )
            else:
                cls.instance().to_be_removed.append( func )

    class Data:
        def __init__(self, **kwargs):
            self.__dict__.update( kwargs )

    def __init__(self):
        super().__init__( agxSDK.StepEventListener.ALL )

        self._clear()

        from ..callbacks import KeyboardCallback

        KeyboardCallback.updateCallback( self.on_gui_update )
        self.outside_loop_time = simulation().getTimeStamp()
    
    def removeNotification(self):
        self._clear()

    def preCollide(self, t):
        self._process_callbacks( self.PRE_COLLIDE, t )

    def pre(self, t):
        self._process_callbacks( self.PRE_STEP, t )

    def post(self, t):
        self._process_callbacks( self.POST_STEP, t )

    def last(self, t):
        self._process_callbacks( self.LAST_STEP, t )
    
    def on_gui_update(self, x, y):
        time_stamp = self.getSimulation().getTimeStamp()
        if self.outside_loop_time < time_stamp:
            self._process_callbacks( self.OUTSIDE_LOOP_CALLBACK, time_stamp )
            self.outside_loop_time = time_stamp

    def _sequence(self, time_from_prev, func, **kwargs):
        prev_time = 0.0
        if len( self.sequence_data ) > 0:
            prev_time = self.sequence_data[ len( self.sequence_data ) - 1 ].time
        call_at_time = prev_time + time_from_prev
        self.sequence_data.append( self.Data( time     = call_at_time,
                                              callback = func,
                                              type     = kwargs.get( 'type', self.PRE_COLLIDE ) ) )

    def _call_at(self, time: float, func, **kwargs):
        self.call_at_data.append( self.Data( time     = time,
                                             callback = func,
                                             type     = kwargs.get( 'type', self.PRE_COLLIDE ) ) )
    
    def _process_callbacks(self, callback_type, time: float):
        self._process_removed()

        dt = simulation().getTimeStep()
        def process_timed(callbacks):
            to_remove = []
            for callback_data in callbacks:
                execute = callback_data.type == callback_type and\
                          time >= callback_data.time - 0.5 * dt
                if execute:
                    callback_data.callback()
                    to_remove.append( callback_data )
            for callback_data in to_remove:
                callbacks.remove( callback_data )
        
        process_timed( self.call_at_data )
        process_timed( self.sequence_data )

        for callback in self.step_event_callbacks[ callback_type ]:
            remove_me = callback( time )
            if remove_me and isinstance( remove_me, bool ):
                self.to_be_removed.append( callback )
    
    def _process_removed(self):
        for to_remove in self.to_be_removed:
            if to_remove in self.sequence_data:
                self.sequence_data.remove( to_remove )
            elif to_remove in self.call_at_data:
                self.call_at_data.remove( to_remove )
            
            for _, step_callbacks in self.step_event_callbacks.items():
                if to_remove in step_callbacks:
                    step_callbacks.remove( to_remove )
        self.to_be_removed = []
    
    def _clear(self):
        self.sequence_data = []
        self.call_at_data = []
        self.step_event_callbacks = {
            agxSDK.StepEventListener.PRE_COLLIDE: [],
            agxSDK.StepEventListener.PRE_STEP:    [],
            agxSDK.StepEventListener.POST_STEP:   [],
            agxSDK.StepEventListener.LAST_STEP:   [],
            self.OUTSIDE_LOOP_CALLBACK:           []
        }
        self.call_for_closures = {}
        self.to_be_removed = []
