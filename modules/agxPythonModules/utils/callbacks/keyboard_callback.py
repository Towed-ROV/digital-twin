import agxSDK

from ..environment import simulation

from datetime import datetime
from collections import OrderedDict

class KeyboardCallback(agxSDK.GuiEventListener):
    """Receive keyboard callbacks in native or extended mode.

    Mode (default mode is EXTENDED):
        NATIVE: Behaves like agxSDK.GuiEventListener but with some additional data
                such as data.isKeyDown (True once), data.timeDown (wallclock time down) and
                data.isKeyUp (True when released.)
        EXTENDED: Callbacks are made in a pre-collide step event with additional data such as
                  data.timeDown (simulation time the key has been pressed). This is the default mode.
    
    Examples:
        from agxPythonModules.utils.callbacks import KeyboardCallback as kc

        kc.bind( name = 'F1',
                 key = kc.KEY_F1,
                 mode = kc.Mode.NATIVE,
                 callback = lambda data: data.print() )
        kc.bind( name = 'Is F1 down?',
                 key = 'a',
                 callback = lambda _: print( 'YES' ) if kc.getData( 'F1' ).down else print( 'NO' ) )
    """


    class Mode:
        NATIVE = 1   # Callback when the key is pressed.
        EXTENDED = 2 # Callback in preCollide with additional data such as 'time down'.
        
    class Data:
        def __init__(self, name: str, key: int, modKey: int, callback: callable, mode):
            self.name      = name
            self.key       = key
            self.modKey    = modKey
            self.callback  = callback
            self.mode      = mode
            self.down      = False
            self.isKeyDown = False
            self.isKeyUp   = False
            self.timeDown  = 0.0
        
        def match(self, key: int, modKey: int) -> bool:
            return ( self.modKey > 0 and self.key == key and self.modKey == modKey ) or\
                   ( self.modKey == 0 and self.key == key )
        
        def print(self):
            print( 'key:', self.key )
            print( 'down:', self.down )
            print( 'timeDown:', self.timeDown )
            print( 'isKeyDown:', self.isKeyDown )
            print( 'isKeyUp:', self.isKeyUp )
            print()

    name = '_GelGlobal'
    _instance = None

    @classmethod
    def instance(cls):
        """Finds or creates the single instance of this object.
        
        Returns:
            KeyboardCallback -- KeyboardCallback instance.
        """

        if cls._instance and cls._instance.getSimulation():
            assert cls._instance.getSimulation() == simulation()
            return cls._instance
        
        cls._instance = cls()
        cls._instance.setName( cls.name )
        simulation().add( cls._instance )
        
        from ..callbacks import StepEventCallback

        StepEventCallback.preCollideCallback( cls._instance._process_bindings )
        
        return cls._instance

    @classmethod
    def bind(cls, **kwargs):
        """Bind key to callback.
        
        Arguments:
            name: str -- Name of the callback used to unbind or fetch data.
            key: str or int -- Key as string or integer (e.g., 'a' or KeyboardCallback.KEY_Up).
            modKey: int -- Optional, untested.
            callback: callable -- Callback fired when the given key is down (argument KeyboardCallback.Data).
            mode: NATIVE or EXTENDED, EXTENDED if this argument isn't given.
        """

        try:
            name     = kwargs[ 'name' ]
            key      = ord( kwargs[ 'key' ] ) if isinstance( kwargs[ 'key' ], str ) else kwargs[ 'key' ]
            modKey   = kwargs.get( 'modKey', 0 )
            callback = kwargs[ 'callback' ]
            mode     = kwargs.get( 'mode', KeyboardCallback.Mode.EXTENDED )

            if name in cls.instance().bindings:
                raise Exception( 'Key already bound.' )

            cls.instance().bindings[ name ] = cls.Data( name, key, modKey, callback, mode )
            cls.instance().key_name_bindings[ key ] = name

        except Exception as e:
            print( '\nUnable to bind key: %s\n' % str( e ) )
    
    @classmethod
    def unbind(cls, name: str):
        """Unbind key callback.
        
        Arguments:
            name: str -- Name of the binding to remove.
        """

        if not name in cls.instance().bindings:
            print( 'Unable to unbind key name: %s - key not bound.' % name )
            return
        
        key = cls.instance().bindings[ name ].key
        del cls.instance().bindings[ name ]
        del cls.instance().key_name_bindings[ key ]
    
    @classmethod
    def getData(cls, name: str):
        """Receive data given named binding.
        
        Arguments:
            name: str -- Name of the binding.
        
        Returns:
            KeyboardCallback.Data -- Key callback data if bound, otherwise None.
        """

        if not name in cls.instance().bindings:
            return None
        return cls.instance().bindings[ name ][ 0 ]

    @classmethod
    def updateCallback(cls, func: callable):
        """Receive agxSDK.GuiEventListener.UPDATE callbacks.
        
        Arguments:
            func: callable -- Callback function called on each UPDATE.
        """

        cls.instance().update_callbacks.append( func )
    
    @classmethod
    def printBindings(cls, **kwargs):
        """Print current bindings to console or application window (SceneDecorator).

        Arguments:
            scene_decorator: agxOSG.SceneDecorator - Optional to print to application window.
            index_offset: int - Scene decorator setText start index offset if scene_decorator is given (Default: 0).
        """

        cls.instance()._print_bindings( **kwargs )

    def __init__(self):
        super().__init__( agxSDK.GuiEventListener.KEYBOARD | agxSDK.GuiEventListener.UPDATE )

        self.bindings          = OrderedDict()
        self.key_name_bindings = {}
        self.update_callbacks  = []
        self.key_name_table    = None
        self.modkey_name_table = None
        self.print_help        = False
    
    def keyboard(self, key, modKey, x, y, down):
        if not key in self.key_name_bindings:
            self.print_help = not down and key == ord( 'H' )
            return False

        name = self.key_name_bindings[ key ]
        data = self.bindings[ name ]
        if not data.match( key, modKey ):
            return True

        if data.mode == KeyboardCallback.Mode.NATIVE:
            data.isKeyDown = down and not data.down
            data.isKeyUp   = not down and data.down
            data.down      = down

            if data.isKeyDown:
                data._timeAtKeyDown = datetime.now()
                data.timeDown = 0.0
            else:
                data.timeDown = ( datetime.now() - data._timeAtKeyDown ).total_seconds()

            data.callback( data )

            if data.isKeyUp:
                data._timeAtKeyDown = None

            data.isKeyDown = False
            data.isKeyUp   = False

        elif data.down != down:
            data.down = down

            if down:
                data.isKeyDown = True
                data.timeDown  = 0.0
            else:
                data.isKeyUp = True
        
        return True
    
    def update(self, x, y):
        if self.print_help:
            self._print_bindings()
            self.print_help = False
        for callback in self.update_callbacks:
            callback(x, y)

    def _process_bindings(self, t):
        sim = self.getSimulation()
        for _, data in self.bindings.items():
            if data.mode == KeyboardCallback.Mode.EXTENDED and ( data.down or data.isKeyUp ):
                data.callback( data )
                data.timeDown += sim.getTimeStep()
                data.isKeyDown = False
                data.isKeyUp = False
    
    def _print_bindings(self, **kwargs):
        if self.key_name_table is None:
            import inspect

            self.key_name_table    = {}
            self.modkey_name_table = {}
            for name, kind, _, value in inspect.classify_class_attrs( type( self ) ):
                if kind != 'data':
                    continue
                if name.startswith( 'KEY_' ):
                    self.key_name_table[ value ] = name
                elif name.startswith( 'MODKEY_' ):
                    self.modkey_name_table[ value ] = name

        max_key_name  = 2 # Current max is 'Key' in the header.
        max_desc_name = 0
        key_names     = []
        for name, data in self.bindings.items():
            key_name = self.key_name_table[ data.key ] if data.key in self.key_name_table else chr( data.key )
            if data.modKey > 0 and data.modKey in self.modkey_name_table:
                key_name += ' + ' + self.modkey_name_table[ data.modKey ]
            max_key_name  = max( max_key_name, len( key_name ) )
            max_desc_name = max( max_desc_name, len( data.name ) )
            key_names.append( (key_name, data.name) )
        
        max_key_name += 1
        line_len      = max_key_name + max_desc_name + 2
        format_str    = '{key:{key_width}}| {desc}'

        sd = kwargs.get( 'scene_decorator', None )
        if sd:
            offset = kwargs.get( 'index_offset', 0 )
            sd.setText( offset, format_str.format( key = 'Key', key_width = max_key_name, desc = 'Description' ) )
            sd.setText( offset + 1, '-' * line_len )
            for i, (key, desc) in enumerate( key_names ):
                sd.setText( i + offset + 2, format_str.format( key = key, key_width = max_key_name, desc = desc ) )
        else:
            print( '+' * line_len )
            print( format_str.format( key = 'Key', key_width = max_key_name, desc = 'Description' ) )
            print( '-' * line_len )
            for key, desc in key_names:
                print( format_str.format( key = key, key_width = max_key_name, desc = desc ) )
            print( '+' * line_len )

