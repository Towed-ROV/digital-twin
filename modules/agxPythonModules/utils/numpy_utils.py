'''
Utility functions for creating numpy arrays of shared memory
'''

import numpy as np
import ctypes
from functools import reduce

import sys

# https://stackoverflow.com/questions/33247262/the-corresponding-ctypes-type-of-a-numpy-dtype
_simple_types = [
    ctypes.c_byte, ctypes.c_short, ctypes.c_int, ctypes.c_long, ctypes.c_longlong,
    ctypes.c_ubyte, ctypes.c_ushort, ctypes.c_uint, ctypes.c_ulong, ctypes.c_ulonglong,
    ctypes.c_float, ctypes.c_double,
]
_typescodes = {np.dtype(ctype).str: ctype for ctype in _simple_types}

agx_types = {
    'Vec3u' : {'columns' : 4, '64bit' : np.uint64, '32bit' : np.uint32},
    'Vec3i' : {'columns' : 4, '64bit' : np.int64, '32bit' : np.int32},
    'Vec3'  : {'columns' : 4, '64bit' : np.float, '32bit' : np.float32},
    'Vec4'  : {'columns' : 4, '64bit' : np.float, '32bit' : np.float32},
    'Quat'  : {'columns' : 4, '64bit' : np.float, '32bit' : np.float32},
    'Real'  : {'columns' : 1, '64bit' : np.float, '32bit' : np.float32},
    'UInt'  : {'columns' : 1, '64bit' : np.uint64, '32bit' : np.uint32},
    'Bool'  : {'columns' : 1, '8bit' : np.bool_, '8bit' : np.bool_},
}

def create_numpy_array(ptr, shape, np_type):
    '''
    Returns a numpy array created from the data pointed at by ptr.
    The user is responsible for providing the correct shape and type
    of the data pointed at.

    Returns None if any error or exception occurs

    Arguments:
        ptr:        Swig object ptr
        shape:      tuple of ints
        np_type:    numpy data type. For example np.float for c double or  np.float32 for c floats
    '''
    # Make sure ptr can be casted to int
    ptr = int(ptr)

    # Make sure that shape is a tuple of int
    if type(shape) is tuple:
        if any([(type(s) is not int) for s in shape]):
            raise TypeError('shape must be a tuple of int')
    else:
        raise TypeError('shape must be a tuple')

    if np.dtype(np_type).str not in _typescodes:
        raise TypeError('np_type: ', np_type, 'is not supported!')

    c_type = _typescodes[np.dtype(np_type).str]
    size = reduce(lambda x, y: x * y, shape)
    ptr = ctypes.cast(ptr, ctypes.POINTER(c_type * size))
    array = np.asarray(ptr.contents, dtype=np_type).reshape(shape)
    return array

class BufferWrapper():
    '''
    Wrapper for an AGX buffer. The class creates a numpy array sharing the memory of the AGX buffer
    at path. Canges to the numpy array will change the AGX buffer.

    Usage example that sets every particles position to (1.0, 1.0, 1.0):
    particle_position_wrapper = BufferWrapper(sim, "Particle.position")
    particle_position_wrapper.array[:] = 1.0

    It is important to always call buffer.array every time AGX has been in control. That
    recreates the numpy array, so it is always valid. Even if AGX has reallocated the memory
    of the buffer.

    Arguments:
        simulation: One agxSDK.Simulation that contains the buffer
        path: String path for the buffer
        columns: if None it is inferred from the format name of the buffer. It is not possible to infer for all format names,
        in that case one can set it. Must be 1 or 4
        np_type: if None it is inferred from the format name of the buffer. It is not possible to infer to all format names,
        in that case one can set it. Valid value is np.float
    '''
    def __init__(self, simulation, path, columns = None, np_type = None):
        self.simulation = simulation
        self.path = path

        b, size, formatName = self.simulation.getBuffer(self.path)
        # print(formatName)

        if np_type is None and columns is None:
            try:
                t, f = formatName.split(':')
            except:
                raise TypeError('Expected a format on form Vec3:Real. Buffer sharing is not implemented for the format: %s' % (formatName))
            try:
                self.columns = agx_types[t]['columns']
                self.np_type = agx_types[t][f]
            except:
                raise TypeError('Buffer sharing is not implemented for the format: %s' % (formatName))
        else:
            self.np_type = np_type
            if columns not in [1 ,4]:
                raise AttributeError('Only supports column=1 or column=4')
            else:
                self.columns = columns

    @property
    def array(self):
        b, size, formatName = self.simulation.getBuffer(self.path)
        return create_numpy_array(b, (size, self.columns), self.np_type)