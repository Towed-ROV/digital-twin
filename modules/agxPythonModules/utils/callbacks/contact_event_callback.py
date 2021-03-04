import agx
import agxCollide
import agxSDK
import agxWire
import agxCable

from ..environment import simulation

class ContactEventCallback(agxSDK.ContactEventListener):
    """
    Contact event callbacks given object(s) (agx.RigidBody or agxCollide.Geometry)
    and callable. An agxSDK.BoolPropertyFilter is used as filter and each
    geometry listened for will have a property bool added to its property container.

    Examples:
        from agxPythonModules.utils.callbacks import ContactEventCallback as ContactEvent

        ground = agx.RigidBody( 'ground' )
        # Add several geometries.
        add_ground_geometries( ground )
        # Listen to all contacts with ground rigid body.
        ContactEvent.contactCallback( ground, lambda t, geometryContact: print( 'In contact with ground' ) )

        # Verify property container created.
        geometry = agxCollide.Geometry( agxCollide.Box( 1, 1, 1 ) )
        assert not geometry.hasPropertyContainer(), "Shouldn't have a property container."
        ContactEvent.impactCallback( geometry, lambda t, gc: print( 'Impact' ) )
        assert geometry.hasPropertyContainer(), "ContactEvent should've created property container."
        assert geometry.getPropertyContainer().hasPropertyBool( ContactEvent._property_id )
    """
    _name        = '_CecGlobal'
    _instance    = None
    _property_id = '_cecpid'

    @classmethod
    def instance(cls):
        """
        Creates and add step event listener to simulation or returns
        and already created instance.
        
        Returns:
            ContactEventCallback - contact event callback instance.
        """
        if cls._instance and cls._instance.getSimulation():
            assert cls._instance.getSimulation() == simulation()
            return cls._instance
        
        cls._instance = cls( cls.ALL )
        cls._instance.setName( cls._name )
        cls._instance.setFilter( agxSDK.BoolPropertyFilter( cls._property_id, True ) )
        simulation().add( cls._instance )

        return cls._instance

    @classmethod
    def impactCallback(cls, *args):
        """
        Register impact callback given object(s) and callable
        (taking time: float, geometry_contact: agxCollide.GeometryContact).

        It's optional to return KeepContactPolicy from the callback.

        The callback will be executed when:
            #given objects = 1: One of the objects geometries is part of the geometry contact.
            #given objects = 2: Both objects are part of the geometry contact.
            #given objects > 2: Two of the objects are part of the geometry contact.
        
        Arguments:
            *args: agxCollide.Geometry, agx.RigidBody and/or callable - arbitrary number of arguments where at least one is
                                                                        either agxCollide.Geometry or agx.RigidBody and one
                                                                        callable callback.
        
        Examples:
            def on_impact(t: float, gc: agxCollide.GeometryContact):
                return ContactEventCallback.REMOVE_CONTACT
            # Remove all impacting contacts with 'ground'.
            ContactEventCallback.impactCallback( ground, on_impact )
        """
        data = cls.CallbackData( *args )
        cls._tryAdd( data, cls.instance().m_imp_callbacks )

    @classmethod
    def contactCallback(cls, *args):
        """
        Register contact callback given object(s) and callable
        (taking time: float, geometry_contact: agxCollide.GeometryContact).

        It's optional to return KeepContactPolicy from the callback.

        The callback will be executed when:
            #given objects = 1: One of the objects geometries is part of the geometry contact.
            #given objects = 2: Both objects are part of the geometry contact.
            #given objects > 2: Two of the objects are part of the geometry contact.
        
        Arguments:
            *args: agxCollide.Geometry, agx.RigidBody and/or callable - arbitrary number of arguments where at least one is
                                                                        either agxCollide.Geometry or agx.RigidBody and one
                                                                        callable callback.
        
        Examples:
            def on_contact(t: float, gc: agxCollide.GeometryContact):
                return ContactEventCallback.REMOVE_CONTACT
            # Remove all contact contacts with 'ground'.
            ContactEventCallback.contactCallback( ground, on_contact )
        """
        data = cls.CallbackData( *args )
        cls._tryAdd( data, cls.instance().m_con_callbacks )

    @classmethod
    def separationCallback(cls, *args):
        """
        Register separation callback given object(s) and callable
        (taking time: float, geometry_pair: agxCollide.GeometryPair).

        The callback will be executed when:
            #given objects = 1: One of the objects geometries is part of the geometry contact.
            #given objects = 2: Both objects are part of the geometry contact.
            #given objects > 2: Two of the objects are part of the geometry contact.
        
        Arguments:
            *args: agxCollide.Geometry, agx.RigidBody and/or callable - arbitrary number of arguments where at least one is
                                                                        either agxCollide.Geometry or agx.RigidBody and one
                                                                        callable callback.
        
        Examples:
            def on_separation(t: float, gp: agxCollide.GeometryPair):
                print( '{6.2f}: Separation: {} <-> {}'.format( t, gp.first().getName(), gp.second().getName() ) )
            # Print separations between ground and a box.
            ContactEventCallback.contactCallback( ground, box, on_separation )
        """
        data = cls.CallbackData( *args )
        cls._tryAdd( data, cls.instance().m_sep_callbacks )
    
    @classmethod
    def remove(cls, *args):
        """
        Remove occurrences of object(s) and callback in active contact event
        callbacks of any type.

        Remarks:
            Note that using same callback for variable number of objects
            will remove any occurrence of the object to remove. For example:
                def my_callback(t, gc):
                    pass
                # Box <-> ground contacts to my_callback.
                ContactEventCallback.contactCallback( box, ground, my_callback )
                # Any box contact to my_callback.
                ContactEventCallback.contactCallback( box, my_callback )
                # This will also remove box <-> ground to my_callback:
                ContactEventCallback.remove( box, my_callback )

        Arguments:
            *args: agxCollide.Geometry, agx.RigidBody and/or callable - arbitrary number of arguments where at least one is
                                                                        either agxCollide.Geometry, agx.RigidBody or callable.
        
        Examples:
            # Similar to the one under 'Remarks' but with separate callables.
            def box_all_callback(t, gc):
                pass
            def box_ground_callback(t, gc):
                pass

            # Any box contact to box_all_callback.
             ContactEventCallback.contactCallback( box, box_all_callback )
            # Box <-> ground contacts to box_ground_callback.
            ContactEventCallback.contactCallback( box, ground, box_ground_callback )
            # Un-register box <-> all events.
            ContactEventCallback.remove( box, box_all_callback )
        """
        remove_data = cls.CallbackData( *args )
        if remove_data.callback is None and remove_data.num_objects == 0:
            print( 'Unable to find matching contact event callback: #given objects = {}, callback = {}'.format( remove_data.num_objects,
                                                                                                                remove_data.callback ) )
            return

        removed_objects = []
        def is_remove_match(data):
            # We know that either callback != None or num_objects > 0 from
            # check earlier in 'remove' so we can match callback == None
            # and num_objects == 0 without accidentally removing all callbacks.
            callback_match = remove_data.callback is None or remove_data.callback == data.callback
            object_match   = remove_data.num_objects == 0 or len( data.object_set.intersection( remove_data.object_set ) ) > 0
            remove_match   = callback_match and object_match

            if remove_match:
                nonlocal removed_objects
                removed_objects += [obj for obj in data.objects if not obj in removed_objects]
            
            return remove_match
        def keep(data):
            return not is_remove_match( data )
        
        cls.instance().m_imp_callbacks = [data for data in cls.instance().m_imp_callbacks if keep( data )]
        cls.instance().m_con_callbacks = [data for data in cls.instance().m_con_callbacks if keep( data )]
        cls.instance().m_sep_callbacks = [data for data in cls.instance().m_sep_callbacks if keep( data )]

        if len( removed_objects ) == 0:
            print( 'Unable to remove contact event callback. No matching objects and/or callbacks found.' )
            return
        
        for removed_object in removed_objects:
            removed_object_id = removed_object.getUuid()
            ref_count = 0
            for data in cls.instance().all_callbacks:
                ref_count += 1 if removed_object_id in data.object_set else 0
            
            if ref_count == 0:
                def removed_object_property_holders():
                    if isinstance( removed_object, ( agxWire.Wire, agxCollide.Geometry ) ):
                        yield removed_object
                    elif isinstance( removed_object, agx.RigidBody ):
                        for geometry in removed_object.getGeometries():
                            yield geometry
                    elif isinstance( removed_object, agxCable.Cable ):
                        for segment in removed_object.segments:
                            yield segment.getGeometry()
                for property_holder in removed_object_property_holders():
                    assert property_holder.hasPropertyContainer() and property_holder.getPropertyContainer().hasPropertyBool( cls._property_id )
                    property_holder.getPropertyContainer().removePropertyBool( cls._property_id )

    @classmethod
    def _tryAdd(cls, data, callbacks):
        if data.callback is None or data.num_objects == 0:
            raise TypeError( 'Unable to add contact event callback: #given objects = {}, callback = {}'.format( data.num_objects,
                                                                                                                data.callback ) )
        elif data in callbacks:
            raise TypeError( 'Unable to add contact event callback: Matching callback already added.' )
        
        for geometry in data.property_holders:
            pc = geometry.getPropertyContainer() # type: agx.PropertyContainer
            if pc.hasPropertyBool( cls._property_id ):
                pc.setPropertyBool( cls._property_id, True )
            else:
                pc.addPropertyBool( cls._property_id, True )

        callbacks.append( data )

    @property
    def all_callbacks(self):
        for callbacks in [self.m_imp_callbacks, self.m_con_callbacks, self.m_sep_callbacks]:
            for data in callbacks:
                yield data

    def __init__(self, *args):
        super().__init__( *args )

        self.m_imp_callbacks = []
        self.m_con_callbacks = []
        self.m_sep_callbacks = []
    
    def impact(self, t, gc):
        return self._process_callbacks( t, gc, self.m_imp_callbacks )

    def contact(self, t, gc):
        return self._process_callbacks( t, gc, self.m_con_callbacks )

    def separation(self, t, gp):
        self._process_callbacks( t, gp, self.m_sep_callbacks )
    
    def _process_callbacks(self, t: float, contact_data, callbacks):
        state = self.KEEP_CONTACT
        for data in callbacks:
            if data.match( contact_data ):
                ret = data.callback( t, contact_data )
                state = max( ret, state ) if ret else state
        return state

    class CallbackData:
        """
        Internal class containing callback data for matching
        with geometry contacts and geometry pairs.
        """
        @property
        def num_objects(self) -> int:
            """
            Returns:
                int - Number of objects in this instance.
            """
            return len( self.objects )
        
        @property
        def property_holders(self):
            """
            Iterate the property holders (geometries or wires) of objects in this instance.
            """
            for obj in self.objects:
                if isinstance( obj, ( agxWire.Wire, agxCollide.Geometry ) ):
                    yield obj
                elif isinstance( obj, agx.RigidBody ):
                    for geometry in obj.getGeometries():
                        yield geometry
                elif isinstance( obj, agxCable.Cable ):
                    for segment in obj.segments:
                        yield segment.getGeometry()
        
        def match(self, contact_data) -> bool:
            """
            Matching contact data with objects in this instance.
            
                - Single object in this instance:
                      Match if object is part of contact_data.
                - Two objects in this instance:
                      Match if BOTH objects are part of contact_data.
                - Three or more objects in this instance:
                      Match if both objects from contact_data is part of our objects.
            
            Arguments:
                contact_data: agxCollide.GeometryContact or agxCollide.GeometryPair - contact data
            
            Returns:
                bool - True if contact data matches objects in this instance, otherwise False.
            """
            contacting_geometries = [contact_data.geometry( 0 ), contact_data.geometry( 1 )]\
                                        if hasattr( contact_data, 'geometry' ) else\
                                    [contact_data.first(), contact_data.second()]
            id_set = frozenset( geometry.getUuid() for geometry in contacting_geometries )
            is_match = len( self.geometry_set.intersection( id_set ) ) == min( self.num_objects, 2 )
            # Matching wire-geometry and wire-wire collisions.
            if not is_match and len( self.wires ) > 0:
                wires = [ agxWire.Wire.getWire( contacting_geometries[ 0 ] ),
                          agxWire.Wire.getWire( contacting_geometries[ 1 ] ) ]
                # Possibly one wire to check.
                if None in wires:
                    wire = wires[ 1 ] or wires[ 0 ]
                    other_geometry = contacting_geometries[ 0 ] if wires[ 1 ] else contacting_geometries[ 1 ]
                    is_match = wire and\
                               other_geometry and\
                               wire in self.wires and\
                               other_geometry.getUuid() in self.geometry_set
                # Both are wires, explicitly don't match wire1 == wire2
                # which filters away self contacts so that, e.g.,
                # ContactEventCallback.impactCallback( wire_contact, wire )
                # only executes when other objects are interacting with it.
                else:
                    is_match = wires[ 0 ] != wires[ 1 ] and\
                               wires[ 0 ] in self.wires and wires[ 1 ] in self.wires
            return is_match

        def __init__(self, *args):
            self.objects  = []
            self.wires    = []
            self.callback = None
            for arg in args:
                if self.callback is None and callable( arg ):
                    self.callback = arg
                elif isinstance( arg, ( agxCable.Cable, agxWire.Wire, agx.RigidBody, agxCollide.Geometry ) ):
                    if not arg in self.objects:
                        self.objects.append( arg )
                        if isinstance( arg, agxWire.Wire ):
                            self.wires.append( arg )

            self.object_set   = frozenset( obj.getUuid() for obj in self.objects )
            self.geometry_set = frozenset( geometry.getUuid() for geometry in self._geometries )
        
        def __eq__(self, other):
            return self.object_set == other.object_set and self.callback == other.callback
        
        @property
        def _geometries(self):
            for obj in self.objects:
                if isinstance( obj, agxCollide.Geometry ):
                    yield obj
                elif isinstance( obj, agx.RigidBody ):
                    for geometry in obj.getGeometries():
                        yield geometry
                elif isinstance( obj, agxCable.Cable ):
                    for segment in obj.segments:
                        yield segment.getGeometry()
