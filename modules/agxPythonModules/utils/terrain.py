import agx
import agxCollide
import agxSDK
import agxUtil
import agxTerrain
import agxRender
import agxOSG

from .environment import simulation
from .constraints import createPrismatic

from collections import namedtuple

class TerrainToolBody(agx.RigidBody):
    """
    Tool rigid body object created from list of boxes half extents and
    absolute angles (tuple( agx.Vec3, angle_degrees )).

    half_extents.x = half thickness of the tool segment
    half_extents.y = half width of the tool segment
    half_extents.z = half height of the tool segment

    Examples:
        # Create bowl-like tool.
        segment_he     = agx.Vec3( 0.01, 1.0, 0.2 )
        num_segments   = 20
        delta_angle    = -180 / ( num_segments - 1 )
        tool_size_list = [ ( segment_he, i * delta_angle ) for i in range( num_segments ) ]
        tool_body      = TerrainToolBody( size = tool_size_list,
                                          create_walls = True )
    """
    @property
    def top_edge(self) -> agx.Line:
        """
        Top edge in local coordinates, i.e., upper edge of first box.

        Returns:
            agx.Line - top edge of this tool
        """
        return self._instance.getTopEdge()\
                   if self._instance\
                   else self._top_edge

    @top_edge.setter
    def top_edge(self, top_edge: agx.Line):
        if self._instance:
            self._instance.setTopEdge( top_edge )
        self._top_edge = top_edge

    @property
    def top_edge_world(self) -> agx.Line:
        """
        Top edge in world coordinates.

        Returns:
            agx.Line - top edge of this tool, in world coordinates
        """
        return self._instance.getTopEdgeWorld()\
                   if self._instance\
                   else agx.Line( self._tpw( self.top_edge.p1 ), self._tpw( self.top_edge.p2 ) )

    @property
    def top_edge_center(self) -> agx.Vec3:
        """
        Top edge center position in local coordinates.

        Returns:
            agx.Vec3 - top edge center position in local coordinates.
        """
        te = self.top_edge
        return 0.5 * ( te.p1 + te.p2 )

    @property
    def top_edge_center_world(self) -> agx.Vec3:
        """
        Top edge center position in world coordinates.

        Returns:
            agx.Vec3 - top edge center position in world coordinates.
        """
        return self._tpw( self.top_edge_center )

    @property
    def cutting_edge(self) -> agx.Line:
        """
        Cutting edge in local coordinates, i.e., lower edge of last box.

        Returns:
            agx.Line - cutting edge of this tool
        """
        return self._instance.getCuttingEdge() if self._instance else self._cutting_edge

    @cutting_edge.setter
    def cutting_edge(self, cutting_edge: agx.Line):
        if self._instance:
            self._instance.setCuttingEdge( cutting_edge )
        self._cutting_edge = cutting_edge

    @property
    def cutting_edge_world(self) -> agx.Line:
        """
        Cutting edge in world coordinates.

        Returns:
            agx.Line - cutting edge of this tool, in world coordinates
        """
        return self._instance.getCuttingEdgeWorld()\
                   if self._instance\
                   else agx.Line( self._tpw( self.cutting_edge.p1 ), self._tpw( self.cutting_edge.p2 ) )

    @property
    def cutting_edge_center(self) -> agx.Vec3:
        """
        Cutting edge center position in local coordinates.

        Returns:
            agx.Vec3 - cutting edge center position in local coordinates.
        """
        ce = self.cutting_edge
        return 0.5 * ( ce.p1 + ce.p2 )

    @property
    def cutting_edge_center_world(self) -> agx.Vec3:
        """
        Cutting edge center position in world coordinates.

        Returns:
            agx.Vec3 - cutting edge center position in world coordinates.
        """
        return self._tpw( self.cutting_edge_center )

    @property
    def forward_vector(self) -> agx.Vec3:
        """
        Forward vector in local coordinates

        Returns:
            agx.Vec3 - forward in local coordinates.
        """
        return self._instance.getForwardVector() if self._instance else self._forward_vector

    @forward_vector.setter
    def forward_vector(self, forward_vector: agx.Vec3):
        if self._instance:
            self._instance.setForwardVector( forward_vector )
        self._forward_vector = forward_vector

    @property
    def forward_vector_world(self) -> agx.Vec3:
        """
        Forward vector in world coordinates.

        Returns:
            agx.Vec3 - the forward vector of this tool in world coordinates.
        """
        return self._tvw( self.forward_vector )

    @property
    def inner_volume_vertices(self) -> agx.Vec3Vector:
        """
        Inner volume vertices to estimate tool volume or to spawn
        particles/granulars inside this tool.

        Returns:
            [agx.Vec3] - List of vertices defining the inner volume of this tool.

        Examples:
            inner_geometry = agxCollide.Geometry( agxUtil.createConvexFromVerticesOnly( body.inner_volume_vertices ) )
            granular_system.spawnParticlesInGeometry( inner_geometry,
                                                      particle_radius,
                                                      particle_spacing,
                                                      particle_jitter )
        """
        return self._inner_volume_vertices

    def create_visual(self, color: agxRender.Color = agxRender.Color.Gold() ):
        """
        Create visual given color.
        Arguments:
            color: agxRender.Color, agx.Vec4f - color of this tool
        """
        from .environment import root
        node = agxOSG.createVisual( self, root() )
        agxOSG.setDiffuseColor( node, color )
        return node

    def __init__(self, **kwargs):
        """
        Construct tool body.

        Arguments:
            size: [( agx.Vec3, float )]                 - list of segment half extents and absolute angle in degrees.
            name: str                                   - [Optional] Name of this tool body (default: 'tool_body').
            motion_control: agx.RigidBody.MotionControl - [Optional] Motion control of this tool body (default: DYNAMICS).
            create_walls: bool                          - [Optional] Create side walls (default: True).
            add_walls: bool                             - See 'create_walls'.
        """
        if not 'size' in kwargs:
            raise TypeError( "'size' list of tuple ( agx.Vec3( segment_half_extent ), angle_degrees ) not given." )

        super().__init__( kwargs.get( 'name', 'tool_body' ) )

        self.setMotionControl( kwargs.get( 'motion_control', self.DYNAMICS ) )

        self._instance = None # type: agxTerrain.Shovel

        self._left_wall_vertices    = agx.Vec3Vector()
        self._right_wall_vertices   = agx.Vec3Vector()
        self._inner_volume_vertices = agx.Vec3Vector()

        # At the end, this is the bottom center, or last geometry center position.
        self._position = agx.Vec3()
        sizes          = kwargs[ 'size' ]
        from math import radians, sin, cos
        for size in sizes:
            he        = size[ 0 ]
            angle     = radians( size[ 1 ] )
            thickness = 2.0 * he.x()
            box       = agxCollide.Geometry( agxCollide.Box( he ) )

            self._position = self._position + agx.Vec3( -he.z() * sin( angle ),
                                                        0,
                                                        -he.z() * cos( angle ) )
            geometry_transform = agx.AffineMatrix4x4.rotate( angle,
                                                             agx.Vec3.Y_AXIS() ) *\
                                agx.AffineMatrix4x4.translate( self._position )

            def add_vertices(z_sign: float):
                self._left_wall_vertices.append(    agx.Vec3( -he.x(),  he.y(),              z_sign * he.z() ) * geometry_transform )
                self._left_wall_vertices.append(    agx.Vec3( -he.x(),  he.y() + thickness,  z_sign * he.z() ) * geometry_transform )
                self._right_wall_vertices.append(   agx.Vec3( -he.x(), -he.y(),              z_sign * he.z() ) * geometry_transform )
                self._right_wall_vertices.append(   agx.Vec3( -he.x(), -he.y() - thickness,  z_sign * he.z() ) * geometry_transform )
                self._inner_volume_vertices.append( agx.Vec3( -he.x(),  he.y(),              z_sign * he.z() ) * geometry_transform )
                self._inner_volume_vertices.append( agx.Vec3( -he.x(), -he.y(),              z_sign * he.z() ) * geometry_transform )

            # First vertex, add top vertices.
            if len( self._left_wall_vertices ) == 0:
                add_vertices( 1.0 )
            add_vertices( -1.0 )

            self.add( box, geometry_transform )
            self._position = self._position + agx.Vec3( -he.z() * sin( angle ), 0, -he.z() * cos( angle ) )

        # 'create_walls' or 'add_walls' accepted.
        if kwargs.get( 'create_walls' if 'create_walls' in kwargs else 'add_walls', True ):
            self.add( agxCollide.Geometry( agxUtil.createConvexFromVerticesOnly( self._left_wall_vertices ) ) )
            self.add( agxCollide.Geometry( agxUtil.createConvexFromVerticesOnly( self._right_wall_vertices ) ) )

        top_he             = sizes[ 0 ][ 0 ]
        top_angle          = radians( sizes[ 0 ][ 1 ] )
        bottom_he          = sizes[ len( sizes ) - 1 ][ 0 ]
        bottom_angle       = radians( sizes[ len( sizes ) - 1 ][ 1 ] )
        self._top_edge     = agx.Line( self.getPosition() + agx.Vec3(  top_he.x() * cos( top_angle ),
                                                                      -top_he.y(),
                                                                      -top_he.x() * sin( top_angle ) ),
                                       self.getPosition() + agx.Vec3(  top_he.x() * cos( top_angle ),
                                                                       top_he.y(),
                                                                      -top_he.x() * sin( top_angle ) ) )
        self._cutting_edge = agx.Line( self._position + agx.Vec3(  bottom_he.x() * cos( bottom_angle ),
                                                                  -bottom_he.y(),
                                                                  -bottom_he.x() * sin( bottom_angle ) ),
                                       self._position + agx.Vec3(  bottom_he.x() * cos( bottom_angle ),
                                                                   bottom_he.y(),
                                                                  -bottom_he.x() * sin( bottom_angle ) ) )

        # use the angle of the last box to estimate the forward vector of the created shovel
        self._forward_vector =  agx.Vec3( sin( -bottom_angle ), 0, -cos( -bottom_angle ) )

    def _tpw(self, p: agx.Vec3) -> agx.Vec3:
        return self.getFrame().transformPointToWorld( p )

    def _tvw(self, v: agx.Vec3) -> agx.Vec3:
        return self.getFrame().transformVectorToWorld( v )

class TerrainTool(agxTerrain.Shovel):
    """
    Terrain tool object containing TerrainToolBody.

    Examples:
        tool_body = TerrainToolBody( size = [ ( agx.Vec3( 0.01, 1.5, 0.2 ), 10 ),
                                              ( agx.Vec3( 0.01, 1.5, 0.4 ), -20 ) ],
                                     create_walls = False )
        tool = TerrainTool( body = tool_body )
        terrain.add( tool )

        # Or similar:
        tool2 = TerrainTool( size = [ ( agx.Vec3( 0.01, 1.5, 0.2 ), 10 ),
                                      ( agx.Vec3( 0.01, 1.5, 0.4 ), -20 ) ],
                                      create_walls = False )
        terrain.add( tool2 )
    """
    @property
    def body(self) -> TerrainToolBody:
        """
        Returns:
            TerrainToolBody - terrain tool rigid body (similar to getRigidBody but with additional data).
        """
        return self._body

    def create_visual(self, color: agxRender.Color = agxRender.Color.Gold() ):
        """
        Create visual given color.
        Arguments:
            color: agxRender.Color, agx.Vec4f - color of this tool
        """
        return self.body.create_visual( color )

    def __init__(self, **kwargs):
        self._body = None

        if not 'body' in kwargs:
            kwargs[ 'body' ] = TerrainToolBody( **kwargs )

        if isinstance( kwargs[ 'body' ], TerrainToolBody ):
            self._body = kwargs[ 'body' ] # type: TerrainToolBody
            super().__init__( self.body, self.body.top_edge, self.body.cutting_edge, self.body.forward_vector )
            from weakref import proxy
            self._body._instance = proxy( self )
        else:
            raise TypeError( 'Unknown body type: {}'.format( type( kwargs[ 'body' ] ) ) )

TerrainGeneratorData = namedtuple( 'TerrainGeneratorData', ['i', 'j', 'resolution', 'element_size', 'size'] )

def create_terrain(**kwargs) -> agxTerrain.Terrain:
    """
    Create terrain and terrain renderer utility function. The
    arguments passed to this function is also passed to create_visual.

    Arguments:
        resolution: int, [int, int], (int, int) - terrain resolution (default: [80, 80])
        element_size: float - terrain element size (default: 0.15)
        max_depth: float - terrain maximum depth (default: 5.0)
        generator: callable - [Optional] Callback returning height at given vertex.

    Examples:
        from math import exp
        def generator(data: TerrainGeneratorData) -> float:
            return 1.0 * exp( -( data.i * data.element_size - 0.5 * data.size )**2 / 4 -
                               ( data.j * data.element_size - 0.5 * data.size )**2 / 10 )
        terrain = create_terrain( generator = generator,
                                  render_particles = True )
    """
    resolution = kwargs.get( 'resolution', [80, 80] )
    if not isinstance( resolution, (list, tuple) ):
        resolution = [ int(resolution), int(resolution) ]
    max_res      = max( resolution[ 0 ], resolution[ 1 ] )
    element_size = kwargs.get( 'element_size', 0.15 )
    max_depth    = kwargs.get( 'max_depth', 5.0 )

    terrain = None
    if 'generator' in kwargs:
        generator = kwargs[ 'generator' ]
        hf_size   = element_size * ( max_res - 1 )
        hf        = agxCollide.HeightField( resolution[ 0 ],
                                            resolution[ 1 ],
                                            hf_size,
                                            hf_size )
        for i in range( resolution[ 0 ] ):
            for j in range( resolution[ 1 ] ):
                hf.setHeight( i, j, generator( TerrainGeneratorData( i, j, resolution, element_size, hf_size ) ) )
        terrain = agxTerrain.Terrain.createFromHeightField( hf, max_depth )
    else:
        terrain = agxTerrain.Terrain( resolution[ 0 ], resolution[ 1 ], element_size, max_depth )
    simulation().add( terrain )
    create_visual( terrain, **kwargs )
    return terrain

def create_visual(obj, **kwargs):
    """
    Create visual given object. World for any agxOSG.createVisual supported object
    but handles agxTerrain.Terrain instances differently by creating a TerrainVoxelRenderer
    instance and adding it to the simulation.

    Terrain instance:
        Arguments:
            obj: agxTerrain.Terrain instance
            render_particles: bool - True to render particles, False to not render particles (default: True).
            render_particles_mesh: bool - True to render particles as triangle meshes (default: False).
            render_heights: bool - True to render heights color range (default: False).
            render_compaction: bool - True to render compaction color range (default: False).
            render_voxel_compaction: bool - True to render voxel compaction from grid (default: False).
            render_height_field: bool - True to render height field, False to not render height field (default: True).
            render_solid_mass: bool - True to render solid mass, False to not render solid mass (default: False).
            render_fluid_mass: bool - True to render fluid mass, False to not render fluid mass (default: False).
            render_nodes: bool - Unknown (default: False).
            render_bounding_box - True to render voxel bounding boxes, False to not (default: False).
            heights_range: agx.RangeReal - Heights range when render_heights = True (default: (-inf, inf)).
            compaction_range: agx.RangeReal - Compaction range when render_compaction = True (default: [0.75, 1.5]).
    Other instances:
        Arguments:
            obj: agxCollide.Geometry, agx.RigidBody, agx.ParticleSystem, agx.GranularBodySystem - object instance
            color: agxRender.Color, agx.Vec4f - color (default: agxRender.Color.Gold)
    """
    from .environment import root

    if isinstance( obj, ( TerrainTool, TerrainToolBody ) ):
        return obj.create_visual( color = kwargs.get( 'color', agxRender.Color.Gold() ) )
    elif isinstance( obj, agxTerrain.Terrain ):
        from .environment import simulation
        render_particles        = kwargs.get( 'render_particles', True )
        render_particles_mesh   = kwargs.get( 'render_particles_mesh', False )
        render_heights          = kwargs.get( 'render_heights', False )
        render_compaction       = kwargs.get( 'render_compaction', False )
        render_voxel_compaction = kwargs.get( 'render_voxel_compaction', False )
        render_height_field     = kwargs.get( 'render_height_field', True )
        render_solid_mass       = kwargs.get( 'render_solid_mass', False )
        render_fluid_mass       = kwargs.get( 'render_fluid_mass', False )
        render_nodes            = kwargs.get( 'render_nodes', False )
        render_bounding_box     = kwargs.get( 'render_bounding_box', False )
        heights_range           = kwargs.get( 'heights_range', agx.RangeReal() )
        compaction_range        = kwargs.get( 'compaction_range', agx.RangeReal( 0.75, 1.5 ) )

        renderer = agxOSG.TerrainVoxelRenderer( obj, root() )
        renderer.setName( 'terrain_renderer' )
        renderer.setRenderSoilParticles( render_particles )
        renderer.setRenderSoilParticlesMesh( render_particles_mesh )
        renderer.setRenderHeights( render_heights, heights_range )
        renderer.setRenderCompaction( render_compaction, compaction_range )
        renderer.setRenderVoxelCompaction( render_voxel_compaction )
        renderer.setRenderHeightField( render_height_field )
        renderer.setRenderVoxelSolidMass( render_solid_mass )
        renderer.setRenderVoxelFluidMass( render_fluid_mass )
        renderer.setRenderNodes( render_nodes )
        renderer.setRenderVoxelBoundingBox( render_bounding_box )

        simulation().add( renderer )

        return renderer
    else:
        node = agxOSG.createVisual( obj, root() )
        if 'color' in kwargs:
            agxOSG.setDiffuseColor( node, kwargs[ 'color' ] )
        return node

class TerrainTests:
    @staticmethod
    def create_shovel(**kwargs) -> TerrainTool:
        shovel = TerrainTool( size = [ ( agx.Vec3( 0.01, 1.5, 0.2 ), 10 ),
                                        ( agx.Vec3( 0.01, 1.5, 0.4 ), -20 ) ],
                              create_walls = False )
        simulation().add( shovel.body )
        create_visual( shovel )
        return shovel

    @staticmethod
    def create_tool_prismatic(tool) -> agx.Prismatic:
        body = tool.body if hasattr( tool, 'body' ) else tool
        prismatic = createPrismatic( rb1      = body,
                                    axis     = agx.Vec3.X_AXIS(),
                                    position = body.getCmFrame().getLocalTranslate() )
        prismatic.setCompliance( 0 )
        prismatic.getMotor1D().setEnable( True )
        simulation().add( prismatic )
        return prismatic
