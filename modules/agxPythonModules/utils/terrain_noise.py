import agx
import agxCollide
import agxSDK
import agxUtil
import agxTerrain
import agxRender
import agxOSG

import math

def addTerrainSurfaceNoise( terrain : agxTerrain.Terrain, **kwargs):
    """
    Util method applying a noise layer that modifies the terrain surface to make it look more organic.
    When noise is adding height to the terrain, it uses the 'TerrainGridControl::addSolidOccupancyLayerInColum'
    to apply the noise as a height layer. Negative values, if used, simply uses get/set height functions to remove mass.

    Inspiration and information about Perlin noise generation for terrain:
    https://medium.com/@yvanscher/playing-with-perlin-noise-generating-realistic-archipelagos-b59f004d8401

    Quote:
    "Perlin noise combines multiple functions called ‘octaves’ to produce natural looking surfaces. Each octave
    adds a layer of detail to the surface. For example: octave 1 could be mountains, octave 2 could be boulders,
    octave 3 could be the rocks."

    Arugments:
        terrain             - the terrain object to apply noise
        use_negative_values - True if negative noise values should be used in the surface modification. (default: False)
        start_range         - Start [x,y] terrain index for appyling noise in terrain (default: [0,0])
        end_range           - End [x,y] terrain index for appyling noise in terrain (default: [resolutionX-1,resolutionY-1])
        should_avalanche    - Set to True if avalanching triggered in the indices where noise has been applied. (default: True)
        scaling_factor      - The scaling factor applied to the generated noise values before application.
                              to the terrain surface. (default: 1.0)
        perlin_octaves      - The number of perlin octave functions (noise generation functions) to be applied.
                              Determines how many levels of detail you want. (default: 10)
        perlin_scale        - The scale of the perlin noise. Determines at what scale to view the perlin noise.
                              Higher values mean smaller details. (default: 500)
        perlin_presistance  - The presistance of the perlin noise. Determines how much each octave contributes
                              to the overall shape (adjusts amplitude). Values above 1 will make each sucessive
                              octave contribute more and make the image more like "regular" noise. (default: 1.0)
        perlin_lancurality  - The lancurality of the perlin noise. Determines how much detail is added or
                              removed at each octave (adjusts frequency). Values above 1 means that each
                              successive octave will have higher detail. (default: 2.0)
        perlin_base         - The base of the perlin noise. Can be interpreted as a seed value. (default: 0)
    """
    if terrain == None:
        print("Given terrain obj in addTerrainSurfaceNoise is None. Returning without applying noise ...")
        return

    res_x = terrain.getResolutionX()
    res_y = terrain.getResolutionX()
    default_compaction = 1.0 / terrain.getTerrainMaterial().getBulkProperties().getSwellFactor()

    use_negative_values   = kwargs.get( 'use_negative_values', False )

    start_range = kwargs.get( 'start_range', [0, 0] )
    if not isinstance( start_range, (list, tuple) ):
        start_range = [ int(start_range), int(start_range) ]

    end_range = kwargs.get( 'end_range', [ res_x-1, res_y-1 ] )
    if not isinstance( end_range, (list, tuple) ):
        end_range = [ int(end_range), int(end_range) ]

    if start_range[0] < 0 or start_range[1] < 0:
        print("Invalid min range: {}. Negative range values. Skipping applying noise.".format( start_range ) )
        return

    if end_range[0] >= res_x or end_range[1] >= res_y:
        print("Invalid max range: {}. max allowed range is {}. Skipping applying noise.".format( end_range, [ res_x-1, res_y-1 ] ) )
        return

    should_avalanche   = kwargs.get( 'should_avalanche', True )
    scaling_factor     = kwargs.get( 'scaling_factor', 1.0 )
    compaction         = kwargs.get( 'compaction',  default_compaction )
    perlin_octaves     = kwargs.get( 'perlin_octaves',  10 )
    perlin_scale       = kwargs.get( 'perlin_scale',  500 )
    perlin_presistance = kwargs.get( 'perlin_presistance',  1 )
    perlin_lancurality = kwargs.get( 'perlin_lancurality',  2 )
    perlin_base        = kwargs.get( 'perlin_base',  0 )

    try:
        import noise
        noise_enabled = True
    except Exception as e:
        print("Noise module could not be imported, skip applying noise to terrain surface")
        noise_enabled = False

    if noise_enabled:

        # Add surface irregularities to make it more organic using Perlin noise
        shape = ( terrain.getResolutionX(), terrain.getResolutionY() )

        gridDataInterface = terrain.getTerrainGridControl()

        h_max = -math.inf
        h_min = math.inf

        # Generate a noise value in each grid point and add that as a layer height on the surface
        for i in range ( start_range[0], end_range[0] ):
            for j in range( start_range[1], end_range[1] ):
                terrain_index = agx.Vec2i( i, j )
                h = noise.pnoise2(i/perlin_scale,
                                  j/perlin_scale,
                                  octaves=perlin_octaves,
                                  persistence=perlin_presistance,
                                  lacunarity=perlin_lancurality,
                                  repeatx=1024,
                                  repeaty=1024,
                                  base=perlin_base)

                h_min = min( h_min, h )
                h_max = max( h_max, h )

                # only keep positive values
                if use_negative_values and h < 0:
                    terrain.setHeight( terrain_index, terrain.getHeight( terrain_index ) + h )
                    h_after = terrain.getHeight( terrain_index )
                elif h > 0:
                    gridDataInterface.addSolidOccupancyLayerInColum( terrain_index, h * scaling_factor, compaction, should_avalanche )

        print( "Perlin noise max value: " + str( h_max ) )
        print( "Perlin noise min value: " + str( h_min ) )
