# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from test_rosmaster/TestArrays.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import genpy
import test_rosmaster.msg

class TestArrays(genpy.Message):
  _md5sum = "4cc9b5e2cebe791aa3e994f5bc159eb6"
  _type = "test_rosmaster/TestArrays"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """# caller_id of most recent node to send this message
string caller_id
# caller_id of the original node to send this message
string orig_caller_id

int32[] int32_array
float32[] float32_array
time[] time_array
TestString[] test_string_array
# TODO: array of arrays

================================================================================
MSG: test_rosmaster/TestString
# Integration test message
# caller_id of most recent node to send this message
string caller_id
# caller_id of the original node to send this message
string orig_caller_id
string data
"""
  __slots__ = ['caller_id','orig_caller_id','int32_array','float32_array','time_array','test_string_array']
  _slot_types = ['string','string','int32[]','float32[]','time[]','test_rosmaster/TestString[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       caller_id,orig_caller_id,int32_array,float32_array,time_array,test_string_array

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(TestArrays, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.caller_id is None:
        self.caller_id = ''
      if self.orig_caller_id is None:
        self.orig_caller_id = ''
      if self.int32_array is None:
        self.int32_array = []
      if self.float32_array is None:
        self.float32_array = []
      if self.time_array is None:
        self.time_array = []
      if self.test_string_array is None:
        self.test_string_array = []
    else:
      self.caller_id = ''
      self.orig_caller_id = ''
      self.int32_array = []
      self.float32_array = []
      self.time_array = []
      self.test_string_array = []

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
      _x = self.caller_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.orig_caller_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      length = len(self.int32_array)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(struct.pack(pattern, *self.int32_array))
      length = len(self.float32_array)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(struct.pack(pattern, *self.float32_array))
      length = len(self.time_array)
      buff.write(_struct_I.pack(length))
      for val1 in self.time_array:
        _x = val1
        buff.write(_get_struct_2I().pack(_x.secs, _x.nsecs))
      length = len(self.test_string_array)
      buff.write(_struct_I.pack(length))
      for val1 in self.test_string_array:
        _x = val1.caller_id
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _x = val1.orig_caller_id
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _x = val1.data
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.time_array is None:
        self.time_array = None
      if self.test_string_array is None:
        self.test_string_array = None
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.caller_id = str[start:end].decode('utf-8')
      else:
        self.caller_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.orig_caller_id = str[start:end].decode('utf-8')
      else:
        self.orig_caller_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      end += struct.calcsize(pattern)
      self.int32_array = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      end += struct.calcsize(pattern)
      self.float32_array = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.time_array = []
      for i in range(0, length):
        val1 = genpy.Time()
        _x = val1
        start = end
        end += 8
        (_x.secs, _x.nsecs,) = _get_struct_2I().unpack(str[start:end])
        self.time_array.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.test_string_array = []
      for i in range(0, length):
        val1 = test_rosmaster.msg.TestString()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.caller_id = str[start:end].decode('utf-8')
        else:
          val1.caller_id = str[start:end]
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.orig_caller_id = str[start:end].decode('utf-8')
        else:
          val1.orig_caller_id = str[start:end]
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.data = str[start:end].decode('utf-8')
        else:
          val1.data = str[start:end]
        self.test_string_array.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self.caller_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.orig_caller_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      length = len(self.int32_array)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(self.int32_array.tostring())
      length = len(self.float32_array)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(self.float32_array.tostring())
      length = len(self.time_array)
      buff.write(_struct_I.pack(length))
      for val1 in self.time_array:
        _x = val1
        buff.write(_get_struct_2I().pack(_x.secs, _x.nsecs))
      length = len(self.test_string_array)
      buff.write(_struct_I.pack(length))
      for val1 in self.test_string_array:
        _x = val1.caller_id
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _x = val1.orig_caller_id
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _x = val1.data
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.time_array is None:
        self.time_array = None
      if self.test_string_array is None:
        self.test_string_array = None
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.caller_id = str[start:end].decode('utf-8')
      else:
        self.caller_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.orig_caller_id = str[start:end].decode('utf-8')
      else:
        self.orig_caller_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      end += struct.calcsize(pattern)
      self.int32_array = numpy.frombuffer(str[start:end], dtype=numpy.int32, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      end += struct.calcsize(pattern)
      self.float32_array = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.time_array = []
      for i in range(0, length):
        val1 = genpy.Time()
        _x = val1
        start = end
        end += 8
        (_x.secs, _x.nsecs,) = _get_struct_2I().unpack(str[start:end])
        self.time_array.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.test_string_array = []
      for i in range(0, length):
        val1 = test_rosmaster.msg.TestString()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.caller_id = str[start:end].decode('utf-8')
        else:
          val1.caller_id = str[start:end]
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.orig_caller_id = str[start:end].decode('utf-8')
        else:
          val1.orig_caller_id = str[start:end]
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.data = str[start:end].decode('utf-8')
        else:
          val1.data = str[start:end]
        self.test_string_array.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_2I = None
def _get_struct_2I():
    global _struct_2I
    if _struct_2I is None:
        _struct_2I = struct.Struct("<2I")
    return _struct_2I
