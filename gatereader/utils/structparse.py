from collections import namedtuple
from struct import Struct
from enum import Enum

STRUCT_FORMAT = '<' # little-endian, no alignment (i.e. packed)

class t:
    """pieces of struct format strings: docs.python.org/3/library/struct.html"""
    uint8 = 'B'
    bytes = lambda sz: '{}s'.format(sz)

class MyStructMixin:
    _struct = None

    @classmethod
    def set_struct(cls, formatstring):
        cls._struct = Struct(formatstring)

    @classmethod
    def unpack_from(cls, buf):
        """Constructs a new instance by unpacking the given buffer.
        Returns the new instance and the rest of the buffer.
        """
        sz = cls._struct.size
        head, tail = buf[:sz], buf[sz:]
        return cls(*cls._struct.unpack(head)), tail

    def pack(self):
        """Returns itself packed as `bytes`."""
        return self._struct.pack(*self)

def mystruct(name, fields, types):
    """Creates a namedtuple that can be packed to and unpacked from `bytes`."""
    class Cls(namedtuple(name, fields), MyStructMixin): pass
    Cls.__name__ = name
    Cls.set_struct(STRUCT_FORMAT + ''.join(types))
    return Cls
