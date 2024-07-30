# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from mav_planning_msgs/PolynomialSegment.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import genpy
import std_msgs.msg

class PolynomialSegment(genpy.Message):
  _md5sum = "1bfc920140297f14773c46c1eacc4c1d"
  _type = "mav_planning_msgs/PolynomialSegment"
  _has_header = True  # flag to mark the presence of a Header object
  _full_text = """Header header
int32 num_coeffs        # order of the polynomial + 1, should match size of x[]
duration segment_time   # duration of the segment
float64[] x             # coefficients for the x-axis, INCREASING order
float64[] y             # coefficients for the y-axis, INCREASING order
float64[] z             # coefficients for the z-axis, INCREASING order
float64[] rx            # coefficients for the rotation x-vector, INCREASING order
float64[] ry            # coefficients for the rotation y-vector, INCREASING order
float64[] rz            # coefficients for the rotation z-vector, INCREASING order
## For backwards compatibility with underactuated (4DOF) commands):
float64[] yaw           # coefficients for the yaw, INCREASING order 
                        

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
  __slots__ = ['header','num_coeffs','segment_time','x','y','z','rx','ry','rz','yaw']
  _slot_types = ['std_msgs/Header','int32','duration','float64[]','float64[]','float64[]','float64[]','float64[]','float64[]','float64[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,num_coeffs,segment_time,x,y,z,rx,ry,rz,yaw

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(PolynomialSegment, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.num_coeffs is None:
        self.num_coeffs = 0
      if self.segment_time is None:
        self.segment_time = genpy.Duration()
      if self.x is None:
        self.x = []
      if self.y is None:
        self.y = []
      if self.z is None:
        self.z = []
      if self.rx is None:
        self.rx = []
      if self.ry is None:
        self.ry = []
      if self.rz is None:
        self.rz = []
      if self.yaw is None:
        self.yaw = []
    else:
      self.header = std_msgs.msg.Header()
      self.num_coeffs = 0
      self.segment_time = genpy.Duration()
      self.x = []
      self.y = []
      self.z = []
      self.rx = []
      self.ry = []
      self.rz = []
      self.yaw = []

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
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_get_struct_3i().pack(_x.num_coeffs, _x.segment_time.secs, _x.segment_time.nsecs))
      length = len(self.x)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.x))
      length = len(self.y)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.y))
      length = len(self.z)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.z))
      length = len(self.rx)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.rx))
      length = len(self.ry)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.ry))
      length = len(self.rz)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.rz))
      length = len(self.yaw)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.yaw))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.segment_time is None:
        self.segment_time = genpy.Duration()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 12
      (_x.num_coeffs, _x.segment_time.secs, _x.segment_time.nsecs,) = _get_struct_3i().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.x = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.y = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.z = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.rx = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.ry = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.rz = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.yaw = struct.unpack(pattern, str[start:end])
      self.segment_time.canon()
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
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_get_struct_3i().pack(_x.num_coeffs, _x.segment_time.secs, _x.segment_time.nsecs))
      length = len(self.x)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.x.tostring())
      length = len(self.y)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.y.tostring())
      length = len(self.z)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.z.tostring())
      length = len(self.rx)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.rx.tostring())
      length = len(self.ry)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.ry.tostring())
      length = len(self.rz)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.rz.tostring())
      length = len(self.yaw)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.yaw.tostring())
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.segment_time is None:
        self.segment_time = genpy.Duration()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 12
      (_x.num_coeffs, _x.segment_time.secs, _x.segment_time.nsecs,) = _get_struct_3i().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.x = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.y = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.z = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.rx = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.ry = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.rz = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.yaw = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      self.segment_time.canon()
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_3I = None
def _get_struct_3I():
    global _struct_3I
    if _struct_3I is None:
        _struct_3I = struct.Struct("<3I")
    return _struct_3I
_struct_3i = None
def _get_struct_3i():
    global _struct_3i
    if _struct_3i is None:
        _struct_3i = struct.Struct("<3i")
    return _struct_3i