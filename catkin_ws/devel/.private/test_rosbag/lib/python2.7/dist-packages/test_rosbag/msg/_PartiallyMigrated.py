# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from test_rosbag/PartiallyMigrated.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import std_msgs.msg
import test_rosbag.msg

class PartiallyMigrated(genpy.Message):
  _md5sum = "45f99fcf57ef956dd2a6a16472643507"
  _type = "test_rosbag/PartiallyMigrated"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """int32            field1 # 40
MigratedExplicit field2 # (58.2, "aldfkja 17", 82)
string           field3 # "radasdk"
float32          field4 # 63.4
float64          field5 # 123.4

================================================================================
MSG: test_rosbag/MigratedExplicit
Header  header
float32 field2 #58.2
string  combo_field3 #"aldfkja 17"
int32   field4 #82

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id
"""
  __slots__ = ['field1','field2','field3','field4','field5']
  _slot_types = ['int32','test_rosbag/MigratedExplicit','string','float32','float64']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       field1,field2,field3,field4,field5

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(PartiallyMigrated, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.field1 is None:
        self.field1 = 0
      if self.field2 is None:
        self.field2 = test_rosbag.msg.MigratedExplicit()
      if self.field3 is None:
        self.field3 = ''
      if self.field4 is None:
        self.field4 = 0.
      if self.field5 is None:
        self.field5 = 0.
    else:
      self.field1 = 0
      self.field2 = test_rosbag.msg.MigratedExplicit()
      self.field3 = ''
      self.field4 = 0.
      self.field5 = 0.

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
      _x = self
      buff.write(_get_struct_i3I().pack(_x.field1, _x.field2.header.seq, _x.field2.header.stamp.secs, _x.field2.header.stamp.nsecs))
      _x = self.field2.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.field2.field2
      buff.write(_get_struct_f().pack(_x))
      _x = self.field2.combo_field3
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.field2.field4
      buff.write(_get_struct_i().pack(_x))
      _x = self.field3
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_get_struct_fd().pack(_x.field4, _x.field5))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.field2 is None:
        self.field2 = test_rosbag.msg.MigratedExplicit()
      end = 0
      _x = self
      start = end
      end += 16
      (_x.field1, _x.field2.header.seq, _x.field2.header.stamp.secs, _x.field2.header.stamp.nsecs,) = _get_struct_i3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.field2.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.field2.header.frame_id = str[start:end]
      start = end
      end += 4
      (self.field2.field2,) = _get_struct_f().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.field2.combo_field3 = str[start:end].decode('utf-8')
      else:
        self.field2.combo_field3 = str[start:end]
      start = end
      end += 4
      (self.field2.field4,) = _get_struct_i().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.field3 = str[start:end].decode('utf-8')
      else:
        self.field3 = str[start:end]
      _x = self
      start = end
      end += 12
      (_x.field4, _x.field5,) = _get_struct_fd().unpack(str[start:end])
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
      _x = self
      buff.write(_get_struct_i3I().pack(_x.field1, _x.field2.header.seq, _x.field2.header.stamp.secs, _x.field2.header.stamp.nsecs))
      _x = self.field2.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.field2.field2
      buff.write(_get_struct_f().pack(_x))
      _x = self.field2.combo_field3
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.field2.field4
      buff.write(_get_struct_i().pack(_x))
      _x = self.field3
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_get_struct_fd().pack(_x.field4, _x.field5))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.field2 is None:
        self.field2 = test_rosbag.msg.MigratedExplicit()
      end = 0
      _x = self
      start = end
      end += 16
      (_x.field1, _x.field2.header.seq, _x.field2.header.stamp.secs, _x.field2.header.stamp.nsecs,) = _get_struct_i3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.field2.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.field2.header.frame_id = str[start:end]
      start = end
      end += 4
      (self.field2.field2,) = _get_struct_f().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.field2.combo_field3 = str[start:end].decode('utf-8')
      else:
        self.field2.combo_field3 = str[start:end]
      start = end
      end += 4
      (self.field2.field4,) = _get_struct_i().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.field3 = str[start:end].decode('utf-8')
      else:
        self.field3 = str[start:end]
      _x = self
      start = end
      end += 12
      (_x.field4, _x.field5,) = _get_struct_fd().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_f = None
def _get_struct_f():
    global _struct_f
    if _struct_f is None:
        _struct_f = struct.Struct("<f")
    return _struct_f
_struct_fd = None
def _get_struct_fd():
    global _struct_fd
    if _struct_fd is None:
        _struct_fd = struct.Struct("<fd")
    return _struct_fd
_struct_i = None
def _get_struct_i():
    global _struct_i
    if _struct_i is None:
        _struct_i = struct.Struct("<i")
    return _struct_i
_struct_i3I = None
def _get_struct_i3I():
    global _struct_i3I
    if _struct_i3I is None:
        _struct_i3I = struct.Struct("<i3I")
    return _struct_i3I
