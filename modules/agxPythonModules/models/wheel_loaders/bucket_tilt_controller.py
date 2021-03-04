import agx
from . import WheelLoader

from agxPythonModules.utils.callbacks import StepEventCallback
from agxPythonModules.tools.simulation_content import SimulationContent

import math

class BucketTiltController:
    """
    Bucket tilt controller, making sure the wheel loader bucket maintain its angle
    and prevents jaming during elevation at extreme tilt angles.
    """
    BUCKET_FORWARD_DIRECTION   = agx.Vec3.Z_AXIS()
    OBSERVER_FORWARD_DIRECTION = agx.Vec3.Z_AXIS()
    OBSERVER_UP_DIRECTION      = agx.Vec3.Y_AXIS()
    
    @property
    def wheel_loader(self) -> WheelLoader:
        return self._wheel_loader
    
    @wheel_loader.setter
    def wheel_loader(self, wheel_loader: WheelLoader):
        if wheel_loader is None or not isinstance( wheel_loader, WheelLoader ):
            info = 'wrong type: {} (WheelLoader expected)'.format( type( wheel_loader ).__name__ )\
                       if wheel_loader\
                       else 'None'
            raise TypeError( 'Argument wheel_loader is {}.'.format( info ) )
        self._wheel_loader = wheel_loader

    @property
    def ref_observer(self) -> agx.ObserverFrame:
        """
        Observer frame located somewhere on the wheel loader, e.g., front body.
        The observer axes must align with the bucket frame, e.g, x left,
        y up and z forward.
        """
        return self._ref_observer
    
    @ref_observer.setter
    def ref_observer(self, ref_observer: agx.ObserverFrame):
        if ref_observer is None or not isinstance( ref_observer, agx.ObserverFrame ):
            info = 'wrong type: {} (agx.ObserverFrame expected)'.format( type( ref_observer ).__name__ )\
                       if ref_observer\
                       else 'None'
            raise TypeError( 'Argument ref_observer is {}.'.format( info ) )
        self._ref_observer = ref_observer
    
    @property
    def bucket_frame(self) -> agx.Frame:
        return self.wheel_loader.bucket_body.getFrame()

    @property
    def bucket_forward(self) -> agx.Vec3:
        return self.BUCKET_FORWARD_DIRECTION

    @property
    def bucket_forward_world(self) -> agx.Vec3:
        return self.bucket_frame.transformVectorToWorld( self.bucket_forward )
    
    @property
    def ref_forward(self) -> agx.Vec3:
        return self.OBSERVER_FORWARD_DIRECTION
    
    @property
    def ref_forward_world(self) -> agx.Vec3:
        return self.ref_observer.transformVectorToWorld( self.ref_forward )

    @property
    def ref_up(self) -> agx.Vec3:
        return self.OBSERVER_UP_DIRECTION

    @property
    def ref_up_world(self) -> agx.Vec3:
        return self.ref_observer.transformVectorToWorld( self.ref_up )
    
    @property
    def tilt_prismatic(self) -> agx.Prismatic:
        return self.wheel_loader.tilt_prismatics[ 0 ]
    
    @property
    def target_angle(self) -> float:
        return self._target_angle
    
    @target_angle.setter
    def target_angle(self, target_angle: float):
        self._target_angle = target_angle
    
    @property
    def enabled(self) -> bool:
        return self._enabled
    
    @enabled.setter
    def enabled(self, enable: bool):
        self._enabled = enable
        if enable:
            self.target_angle = self.calculate_angle()
    
    @property
    def ranged_hinges(self) -> [agx.Hinge]:
        return self._ranged_hinges
    
    @property
    def is_state_tilt_control_override(self) -> bool:
        """
        If elevate prismatics are driving and at lease one
        of our ranged hinges has an active range controller,
        the tilt control state should be overridden to avoid
        jamming.
        """
        if not self.wheel_loader.elevate_prismatics[ 0 ].getMotor1D().getEnable():
            return False
        for h in self.ranged_hinges:
            h.getAttachmentPair().transform()
            if h.getRange1D().isActive():
                return True
        return False

    def calculate_angle(self) -> float:
        sign  = 1.0 if self.bucket_forward_world * self.ref_up_world >= 0.0 else -1.0
        angle = math.acos( self.bucket_forward_world * self.ref_forward_world )
        return sign * angle

    def __init__(self, wheel_loader: WheelLoader, **kwargs):
        self.wheel_loader = wheel_loader
        self.ref_observer = self.wheel_loader.getObserverFrame( kwargs.get( 'ref_observer', 'FrontBodyObserver' ) )

        self._ranged_hinges = []
        wheel_loader_content = SimulationContent( assembly   = self.wheel_loader,
                                                  initialize = True )
        for name, constraint in wheel_loader_content.constraints.items():
            if 'TrackedRangeHinge' in name:
                self._ranged_hinges.append( constraint.asHinge() )
        del wheel_loader_content

        self.enabled = kwargs.get( 'enabled', True )

        if kwargs.get( 'validate', True ):
            self._valid_model()

    def _on_add(self):
        StepEventCallback.postCallback( self._on_post_step )

    def _on_remove(self):
        StepEventCallback.remove( self._on_post_step )

    def _on_post_step(self, _):
        # Update target angle when the motor is active.
        if self.tilt_prismatic.getMotor1D().getEnable():
            self.target_angle = self.calculate_angle()
        # Tilt prismatic motor is inactive, make sure we
        # preserve the target angle.
        elif self.enabled:
            diff = 0.0
            if self.is_state_tilt_control_override:
                self.target_angle = self.calculate_angle()
            else:
                current_angle = self.calculate_angle()
                diff = ( current_angle - self.target_angle ) * 2.5 / agx.PI
            self._apply_tilt_diff( diff )
    
    def _apply_tilt_diff(self, diff: float ):
        self.tilt_prismatic.getLock1D().setPosition( self.tilt_prismatic.getAngle() + diff )
    
    def _valid_model(self):
        tmp_obs = agx.ObserverFrame( 'tmp_obs' )
        tmp_obs.attachWithWorldTransform( self.wheel_loader.front_body,
                                          agx.AffineMatrix4x4.rotate( agx.Vec3.X_AXIS(), agx.Vec3.Y_AXIS() ) *\
                                          agx.AffineMatrix4x4.rotate( agx.Vec3.Z_AXIS(), agx.Vec3.X_AXIS() ) *\
                                          agx.AffineMatrix4x4.translate( self.ref_observer.getPosition() ) )
        assert agx.equivalent( tmp_obs.transformVectorToWorld( agx.Vec3.X_AXIS() ) *\
                               self.ref_observer.transformVectorToWorld( agx.Vec3.X_AXIS() ), 1.0, 1.0E-4 ),\
            'Front body observer seems wrong.'
        assert agx.equivalent( tmp_obs.transformVectorToWorld( agx.Vec3.Y_AXIS() ) *\
                               self.ref_observer.transformVectorToWorld( agx.Vec3.Y_AXIS() ), 1.0, 1.0E-4 ),\
            'Front body observer seems wrong.'
        assert agx.equivalent( tmp_obs.transformVectorToWorld( agx.Vec3.Z_AXIS() ) *\
                               self.ref_observer.transformVectorToWorld( agx.Vec3.Z_AXIS() ), 1.0, 1.0E-4 ),\
            'Front body observer seems wrong.'

        assert not self.tilt_prismatic.getMotor1D().getEnable(), 'Tilt prismatic motor.'
        assert self.tilt_prismatic.getLock1D().getEnable(), 'Tilt prismatic lock.'
        assert not self.tilt_prismatic.getRange1D().getEnable(), 'Tilt prismatic range.'

        for prismatic in self.wheel_loader.elevate_prismatics:
            assert not prismatic.getMotor1D().getEnable(), '{}: Elevate prismatic motor.'.format( prismatic.getName() )
            assert prismatic.getLock1D().getEnable(), '{}: Elevate prismatic lock.'.format( prismatic.getName() )
            assert prismatic.getRange1D().getEnable(), '{}: Elevate prismatic range.'.format( prismatic.getName() )

        assert len( self.ranged_hinges ) == 3, '#tracked ranged hinges: 3 != {}'.format( len( self.ranged_hinges ) )

        for h in self.ranged_hinges:
            assert not h.getMotor1D().getEnable(), '{}: Hinge motor.'.format( h.getName() )
            assert not h.getLock1D().getEnable(),  '{}: Hinge lock.'.format( h.getName() )
            assert h.getRange1D().getEnable(),     '{}: Hinge range.'.format( h.getName() )
