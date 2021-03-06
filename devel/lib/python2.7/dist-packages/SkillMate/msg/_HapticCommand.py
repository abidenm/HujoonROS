# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from SkillMate/HapticCommand.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class HapticCommand(genpy.Message):
  _md5sum = "56596198094ce2af05806d426c471047"
  _type = "SkillMate/HapticCommand"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """float64[6] array
int32[2] btn"""
  __slots__ = ['array','btn']
  _slot_types = ['float64[6]','int32[2]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       array,btn

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(HapticCommand, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.array is None:
        self.array = [0.,0.,0.,0.,0.,0.]
      if self.btn is None:
        self.btn = [0,0]
    else:
      self.array = [0.,0.,0.,0.,0.,0.]
      self.btn = [0,0]

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      buff.write(_struct_6d.pack(*self.array))
      buff.write(_struct_2i.pack(*self.btn))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      start = end
      end += 48
      self.array = _struct_6d.unpack(str[start:end])
      start = end
      end += 8
      self.btn = _struct_2i.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      buff.write(self.array.tostring())
      buff.write(self.btn.tostring())
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      start = end
      end += 48
      self.array = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=6)
      start = end
      end += 8
      self.btn = numpy.frombuffer(str[start:end], dtype=numpy.int32, count=2)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_6d = struct.Struct("<6d")
_struct_2i = struct.Struct("<2i")
