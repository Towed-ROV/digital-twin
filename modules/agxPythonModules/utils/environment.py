import agx
import agxIO
import agxPython
import agxSDK
import osg
import agxOSG

_unittestAvailable = True
try:
    import agxUnit
except Exception as e:
    _unittestAvailable = False

import sys

def unittestEnabled() -> bool:
    if _unittestAvailable:
        return agxUnit.isUnittestingEnabled()
    else:
        return False

def simulation() -> agxSDK.Simulation:
    return agxPython.getContext().environment.getSimulation()

def root() -> osg.Group:
    return agxPython.getContext().environment.getSceneRoot()

def application() -> agxOSG.ExampleApplication:
    return agxPython.getContext().environment.getApplication()

def _applicationMain(**kwargs):
    ## Create an application with graphics etc.
    app = agxOSG.ExampleApplication()
    app.setAutoStepping( kwargs.get( 'autoStepping', False ) )

    ## Create a command line parser. sys.executable will point to python executable
    ## in this case, because getArgumentName(0) needs to match the C argv[0] which
    ## is the name of the program running
    argParser = agxIO.ArgumentParser( [sys.executable] + sys.argv )

    global _unittestAvailable
    if (_unittestAvailable):
        agxUnit.initUnittestFrameWork(argParser)

    if 'scenes' in kwargs:
        for sceneData in kwargs['scenes']:
            scene = sceneData[ 0 ].__name__ if callable( sceneData[ 0 ] ) else sceneData[ 0 ]
            key = ord( sceneData[ 1 ] ) if isinstance( sceneData[ 1 ], str ) else sceneData[ 1 ]
            asTest = sceneData[ 2 ] if len( sceneData ) > 2 else True # 'is part of unittest' True by default
            stopAfter = sceneData[ 3 ] if len( sceneData ) > 3 else 0.0
            app.addScene( argParser.getArgumentName( 1 ), scene, key, asTest, stopAfter )

    ## Call the init method of ExampleApplication
    ## It will setup the viewer, windows etc.
    if app.init( argParser ):
        # Avoid 'start paused' during tests.
        if unittestEnabled() or argParser.find( '--agxOnly' ) >= 0:
            app.setAutoStepping( True )

        if 'onInitialized' in kwargs and callable( kwargs['onInitialized'] ):
            kwargs['onInitialized']( app )

        app.run()
        
        if 'onShutdown' in kwargs and callable( kwargs['onShutdown'] ):
            kwargs['onShutdown']( app )
    else:
        print( "An error occurred while initializing ExampleApplication." )

def init_app(**kwargs):
    """Initialize AGX and ExampleApplication.

    Template:
        init = init_app( name = __name__,
                         scenes = [ ( 'buildScene', '1' ) ] )

    IMPORTANT:
        - agx.AutoInit is returned at MUST be captured! Otherwise AGX will crash during exit.
        - The name has to be __name__ in the script you're calling this function.

    Examples:
        from agxPythonModules.utils.environment import init_app

        init = init_app( name          = __name__,
                         scenes        = [ ( buildScene, '1' ),
                                           ( 'anotherScene', agxSDK.GuiEventListener.KEY_F1 ) ],
                         autoStepping  = True, # Default: False
                         onInitialized = lambda app: print( 'App successfully initialized.' ),
                         onShutdown = lambda app: print( 'App successfully shut down.' ) )

    Arguments:
        name: str -- __name__ of the script this function is called from
        scenes: [] -- List of scene tuples (scene name or function, string key or int), e.g.,
                      [ (buildScene1, '1'), 'buildScene2', ord( '2' ) ]
        onInitialized: callable -- Callback when ExampleApplication has been initialized. This callback
                                   is only fired once - i.e., not when reloading the script.
        autoStepping: bool -- False to start paused. Default False.

    Returns:
        agx.AutoInit -- Instance of agx.AutoInit that MUST be captured.
    """

    ## Entry point when this script is loaded with python
    if kwargs['name'] == "__main__":
        if agxPython.getContext() is None:
            init = agx.AutoInit()
            _applicationMain( **kwargs )
            return init

    return None
