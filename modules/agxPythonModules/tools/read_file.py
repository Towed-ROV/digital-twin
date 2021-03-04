import agx
import agxIO
import agxOSG

import os.path

from collections import OrderedDict
from .simulation_content import SimulationContent

def absolutePath(filename):
    return os.path.abspath( os.path.curdir ).replace( '\\', '/' ) + '/' + filename

class SimulationFile(SimulationContent):
    def __init__(self, **kwargs):
        self.root      = kwargs.get( 'root', None )
        self.resources = OrderedDict()

        super().__init__( simulation = kwargs['simulation'] )

    def load(self, filename: str, **kwargs):
        """Load .agx or .aagx file.
        
        Arguments:
            filename: str -- filename including (relative) path to AGX file.
            parent: agxSDK.Assembly -- parent assembly (optional)
        
        Returns:
            bool -- True if successful, otherwise False
        """

        _, ext = os.path.splitext( os.path.basename( filename ) )
        if ext != '.agx' and ext != '.aagx':
            print( 'Unknown file extention: %s' % ext )
            return False

        print( '\nLoading file %s ... ' % filename, end = '', flush = True )
        timer = agx.Timer( True )

        success = agxOSG.readFile( filename, self.simulation, self.root, kwargs.get( 'parent', None ) )

        timer.stop()
        print( str.format( 'Done! ({:.2f} s)\n', timer.getTime() / 1000.0 ) if success else 'Failed!\n' )

        if not success:
            print( 'Unable to load file: %s' % absolutePath( filename ) )
            return False
        
        if not self.initialize():
            print( 'Unable to extract simulation content.' )
            return False
        
        return True

    def loadResource(self, filename: str):
        """Load resource from file, e.g., .obj.
        
        Arguments:
            filename: str -- filename including (relative) path to resource file.
        
        Returns:
            bool -- True if successful, otherwise False
        """

        name, ext = os.path.splitext( os.path.basename( filename ) )

        if name in self.resources:
            print( 'Source with name: %s, already loaded. Ignoring file %r.' %( name, filename ) )
            return None

        resource = None
        if ext == ".obj":
            resource = agxOSG.readNodeFile( filename, False )
            if resource == None:
                print( 'Unable to load obj-file: %s' % filename )
                return None

            self.resources[ name ] = resource
        else:
            print( 'Unknown resource file extension - ignoring %r' % absolutePath( filename ) )

        return resource
