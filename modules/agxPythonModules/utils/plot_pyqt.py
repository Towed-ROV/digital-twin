"""Layer above pyqtgraph with windows, plots (n per window), curves (m per plot) and crosshair.

Examples:
    #######################################################################################

    # Example how to plot motor torque in a hinge.

    from agxPythonModules.utils.plot_pyqt import Window, Plot, Curve

    plotWindow = Window( title            = 'Plot',
                         size             = ( 1024, 1024 ),
                         closeWhenRemoved = True )
    # plotWindow.isValid is False when one or more dependencies are missing.
    if plotWindow.isValid:
        # Create plot. With multiple plots in an window, 'row' and 'col' can be
        # used to specify where each plot is located in the window.
        torquePlot = plotWindow.createPlot( title  = 'Roller 1',
                                            units  = ( 's', 'N' ),
                                            labels = ( 'Simulation time', 'Torque' ),
                                            row    = 0,
                                            col    = 0 )
        torquePlot.createCurve( 'Motor torque',
                                lambda t: ( t, hinge.getMotor1D().getCurrentForce() ),
                                callbackType = agxSDK.StepEventListener.POST_STEP )

    #######################################################################################

    # Example how to plot wire tension along a wire using curve.setData in a post step event.

    from agxPythonModules.utils.plot_pyqt import Window, Plot, Curve
    from agxPythonModules.utils.callbacks import StepEventCallback

    window = Window( title = 'Wire data', size = ( 1248, 1248 ) )
    assert window.isValid, 'Missing dependencies.'

    tensionPlot  = window.createPlot( title = 'Wire tension', units = ( 'm', 'N' ) )
    tensionCurve = tensionPlot.createCurve( 'tension' )
    def tensionData(*_):
        x, y = [], []
        it = wire.getRenderBeginIterator()
        while it != wire.getRenderEndIterator():
            node    = it.get()
            dist    = wire.findRestLengthFromStart( node.getWorldPosition() )
            tension = wire.getTension( node ) # type: agxWire.WireNodeTensionData
            x.append( dist )
            y.append( tension.getRaw() )
            it.inc()

        tensionCurve.setData( x, y )

    StepEventCallback.postCallback( tensionData )

TODO:
    - Handle windows better using Qt rather than agxSDK.GuiEventListener.
    - Is the agxViewer window changing size when the Qt window is opened?
"""

import agxSDK
from .callbacks import StepEventCallback
from .environment import simulation, unittestEnabled
from math import sqrt

import io

_plotEnabled = True

try:
    import numpy as np
    import pyqtgraph as pg
    from pyqtgraph.Qt import QtGui, QtCore
except Exception as e:
    print( "Plotting is disabled:", e )
    _plotEnabled = False

if unittestEnabled():
    print("In unittest mode: Plotting disabled")
    _plotEnabled = False

def infinity():
    return float( 'inf' )

def pointToPointDistanceSq( p1, p2 ) -> float:
    return ( (p2.x() - p1.x()) * (p2.x() - p1.x()) +
             (p2.y() - p1.y()) * (p2.y() - p1.y()) )
def pointToPointDistance( p1, p2 ) -> float:
    return sqrt( pointToPointDistanceSq( p1, p2 ) )

class Crosshair:
    def __init__(self, curveItem, plotItem ):
        self.curveItem = curveItem
        self.plotItem  = plotItem
        self.lines     = [ pg.InfiniteLine( angle = 90, movable = False ),
                           pg.InfiniteLine( angle = 0, movable = False ) ]
        for line in self.lines:
            plotItem.addItem( line, ignoreBounds = True )

        self.curvePoint = None
        self.textItem   = None

        self.setVisible( False )

    def setVisible( self, visible: bool ):
        for line in self.lines:
            line.setVisible( visible )

        if self._createCurvePoint():
            self.curvePoint.setVisible( visible )
            self.textItem.setVisible( visible )

    @property
    def isVisible(self):
        return self.lines[ 0 ].isVisible()

    def setPosition( self, position ):
        self.lines[ 0 ].setPos( position.x() )
        self.lines[ 1 ].setPos( position.y() )

    def setColor( self, pen ):
        for line in self.lines:
            line.setPen( pen )

    def updateText(self, position: float, value):
        if not self._createCurvePoint():
            return

        # This is a hack since CurvePoint.event handles 'index' as float
        # but CurvePoint.setIndex will cast the value to an int => snap
        # to data point (which probably isn't desired behavior).
        self.curvePoint.setProperty( 'index', position )
        xUnit = self.plotItem.getAxis( 'bottom' ).labelUnits
        yUnit = self.plotItem.getAxis( 'left' ).labelUnits
        text  = '<span style="font-size: 16pt;">x = %.6f%s,<br>y = %.6f%s</span>'
        self.textItem.setHtml( text % (value.x(),
                                       ' (' + xUnit + ')' if xUnit is not '' else '',
                                       value.y(),
                                       ' (' + yUnit + ')' if yUnit is not '' else '') )

    def _createCurvePoint(self):
        if self.curvePoint:
            return True

        # It's not possible to create pg.CurvePoint with and empty set.
        (x, _) = self.curveItem.getData()
        if len( x ) == 0:
            return False

        self.curvePoint = pg.CurvePoint( self.curveItem, rotate = False )
        self.plotItem.addItem( self.curvePoint )

        self.textItem = pg.TextItem( text   = '',
                                     anchor = ( 0, 1 ),
                                     border = self.curveItem.opts['pen'],
                                     fill   = (255, 255, 255, 64) )
        self.textItem.setParentItem( self.curvePoint )

        self.curvePoint.setVisible( self.isVisible )
        self.textItem.setVisible( self.isVisible )

        return True

class Curve:
    def __init__(self, item, plotItem, callback, callbackType, filename):
        if not _plotEnabled:
            return

        self.item = item
        self.callback = callback
        StepEventCallback.stepCallback( callbackType, self.onStep )
        self.crosshair = Crosshair( self.item, plotItem )
        if filename is not None:
            self.filename = filename
            self.file = io.open(self.filename, mode='w',  buffering=1)
        else:
            self.file = None

    def setData(self, x, y ):
        self.item.setData( x, y )

    def onStep(self, t):
        (x, y) = self.item.getData()
        if self.callback:
            xVal, yVal = self.callback( t )
            x = np.append( x, xVal )
            y = np.append( y, yVal )
            if self.file:
                self.file.write("{} {}\n".format(xVal, yVal))

        self.setData( x, y )

    class ProjectionResult:
        numPoints    = 0
        prevIndex    = -1
        nextIndex    = -1
        prevPoint    = (0, 0)
        nextPoint    = (0, 0)
        pointOnCurve = None
        dx           = 0.0
        dy           = 0.0
        curve        = None

    def project(self, point, snapToControlPoint: bool = False):
        (x, y)           = self.item.getData()
        result           = self.ProjectionResult()
        result.curve     = self
        result.numPoints = len( x )

        if result.numPoints == 0:
            return None

        result.prevIndex = Curve.findIndex( x, y, point, snapToControlPoint )
        if snapToControlPoint:
            result.nextIndex = result.prevIndex
            result.prevPoint = ( x[ result.prevIndex ], y[ result.prevIndex ] )
            result.nextPoint = result.prevPoint
            result.pointOnCurve = QtCore.QPointF( *result.prevPoint )
            return result

        if result.prevIndex > 0 and point.x() < x[ result.prevIndex ]:
            result.prevIndex -= 1
        result.nextIndex = result.prevIndex + 1

        result.prevPoint = ( x[ result.prevIndex ], y[ result.prevIndex ] )
        if result.nextIndex >= result.numPoints:
            result.pointOnCurve = QtCore.QPointF( *result.prevPoint )
            return result

        result.nextPoint = ( x[ result.nextIndex ], y[ result.nextIndex ] )

        result.dx = result.nextPoint[ 0 ] - result.prevPoint[ 0 ]
        result.dy = result.nextPoint[ 1 ] - result.prevPoint[ 1 ]

        if result.dx == 0.0:
            result.pointOnCurve = QtCore.QPointF( *result.prevPoint )
            return result

        xVal = point.x() if point.x() > result.prevPoint[ 0 ] and point.x() < result.nextPoint[ 0 ]\
                         else result.prevPoint[ 0 ] if point.x() < result.prevPoint[ 0 ] else result.nextPoint[ 0 ]
        yVal = result.prevPoint[ 1 ] + ( result.dy / result.dx ) * ( xVal - result.prevPoint[ 0 ] )
        result.pointOnCurve = QtCore.QPointF( xVal, yVal )

        return result

    @classmethod
    def findIndex(cls, x, y, point, useXValOnly: bool = False):
        numPoints = len( x )
        if numPoints == 0:
            return -1
        elif numPoints < 2:
            return 0

        if useXValOnly:
            return np.abs( x - point.x() ).argmin()

        esitmatedSegmentLength = pointToPointDistance( QtCore.QPointF( x[ 0 ], y[ 0 ] ),
                                                       QtCore.QPointF( x[ 1 ], y[ 1 ] ) )
        absDiff = np.abs( x - point.x() )
        potentialIndices = np.where( absDiff < 2.0 * esitmatedSegmentLength )[ 0 ]
        if len( potentialIndices ) == 0:
            potentialIndices = [ absDiff.argmin() ]
        elif len( potentialIndices ) == 1:
            return potentialIndices[ 0 ]

        bestDistance = infinity()
        bestIndex = 0
        for index in potentialIndices:
            p = QtCore.QPointF( x[ index ], y[ index ] )
            dist = pointToPointDistanceSq( p, point )
            if dist < bestDistance:
                bestDistance = dist
                bestIndex = index

        return bestIndex

class Plot:
    def __init__(self, item):
        if not _plotEnabled:
            return

        self.item = item
        self.curves = []
        self.mouseMoveProxy = pg.SignalProxy( item.scene().sigMouseMoved,
                                              rateLimit = 60,
                                              slot = self.onMouseMove )
        self.crosshairEnabled = False
        self.crosshairSnap = False

    def setEnableCrosshair(self, enable: bool):
        self.crosshairEnabled = enable
        for curve in self.curves:
            curve.crosshair.setVisible( False )

    def getEnableCrosshair(self) -> bool:
        return self.crosshairEnabled

    def setEnableCrosshairSnap(self, enable: bool):
        self.crosshairSnap = enable

    def getEnableCrosshairSnap(self) -> bool:
        return self.crosshairSnap

    def createCurve(self,
                    name: str,
                    callback = None,
                    **kwargs ):
        if not _plotEnabled:
            return None

        callbackType = kwargs.get( 'callbackType', agxSDK.StepEventListener.PRE_COLLIDE )
        if not 'pen' in kwargs:
            kwargs['pen'] = len( self.curves )
        curve = Curve( self.item.plot( np.array( [] ),
                                       np.array( [] ),
                                       name = name,
                                       **kwargs ),
                       self.item,
                       callback,
                       callbackType,
                       filename = kwargs.get('filename') )

        self.curves.append( curve )

        return curve

    def onMouseMove(self, evt):
        position = evt[ 0 ]
        for curve in self.curves:
            curve.crosshair.setVisible( False )

        if not self.getEnableCrosshair() or not self.item.sceneBoundingRect().contains( position ):
            return

        mousePoint = self.item.vb.mapSceneToView( position )

        bestDistance = infinity()
        closestProjectionData = None
        for curve in self.curves:
            projectionData = curve.project( mousePoint, self.getEnableCrosshairSnap() )
            if projectionData == None or projectionData.pointOnCurve == None:
                continue

            distance = pointToPointDistance( projectionData.pointOnCurve, mousePoint )
            if distance < bestDistance:
                bestDistance = distance
                closestProjectionData = projectionData

        if closestProjectionData:
            projectionData = closestProjectionData
            curve = projectionData.curve
            curve.crosshair.setVisible( True )
            curve.crosshair.setPosition( projectionData.pointOnCurve )
            curve.crosshair.setColor( curve.item.opts['pen'] )
            textPosition = 0.0
            interpolatePoint = projectionData.prevIndex < projectionData.nextIndex and \
                               projectionData.nextIndex < projectionData.numPoints
            snappedToPoint   = projectionData.prevIndex < projectionData.numPoints and \
                               projectionData.prevIndex == projectionData.nextIndex
            isOutOfBounds    = projectionData.nextIndex >= projectionData.numPoints
            if interpolatePoint:
                textPosition = float( projectionData.prevIndex ) +\
                               np.clip( ( projectionData.pointOnCurve.x() -
                                          projectionData.prevPoint[ 0 ] ) / projectionData.dx, 0.0, 1.0 )\
                               if projectionData.dx > 1.0E-10 else float( projectionData.prevIndex )
            elif snappedToPoint or isOutOfBounds:
                textPosition = float( projectionData.prevIndex )

            curve.crosshair.updateText( position = textPosition, value = projectionData.pointOnCurve )

class Window(agxSDK.GuiEventListener):
    def __init__(self,
                 title: str = '',
                 size: (int, int) = (1024, 768),
                 closeWhenRemoved: bool = False):
        if not _plotEnabled:
            return

        super().__init__( agxSDK.GuiEventListener.UPDATE )

        self.closeOnRemoveNotification = closeWhenRemoved

        self.graphicsWindow = pg.GraphicsWindow( title )
        self.graphicsWindow.resize( size[ 0 ], size[ 1 ] )

        self.plots = []

        simulation().add( self )

    @property
    def isValid(self) -> bool:
        return _plotEnabled

    def createPlot(self,
                   **kwargs ):
        if not _plotEnabled:
            return None

        labels = kwargs.get( 'labels', ( 'x', 'f(x)' ) )
        if 'labels' in kwargs:
            kwargs.pop('labels')
        item = self.graphicsWindow.addPlot( **kwargs )
        if kwargs.get( 'showGrid', True ):
            item.showGrid( x = True, y = True )

        if kwargs.get( 'showLegend', True ):
            item.addLegend()

        units = kwargs.get( 'units', ( '', '' ) )
        item.setLabel( 'bottom', labels[ 0 ], units = units[ 0 ] )
        item.setLabel( 'left', labels[ 1 ], units = units[ 1 ] )

        plot = Plot( item )
        plot.setEnableCrosshair( kwargs.get( 'showCrosshair', True ) )
        plot.setEnableCrosshairSnap( kwargs.get( 'crosshairSnap', False ) )

        self.plots.append( plot )

        return plot

    def update(self, *_):
        if hasattr( pg.QtGui, 'qApp' ):
            pg.QtGui.qApp.processEvents()
        else:
            pg.QtGui.QGuiApplication.processEvents()

    def removeNotification(self):
        if self.closeOnRemoveNotification:
            self.graphicsWindow.close()
