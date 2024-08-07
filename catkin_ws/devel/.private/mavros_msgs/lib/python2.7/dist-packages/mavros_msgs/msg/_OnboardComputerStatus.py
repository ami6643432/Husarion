# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from mavros_msgs/OnboardComputerStatus.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import std_msgs.msg

class OnboardComputerStatus(genpy.Message):
  _md5sum = "aebe864fac2952ca9de45a2b65875a60"
  _type = "mavros_msgs/OnboardComputerStatus"
  _has_header = True  # flag to mark the presence of a Header object
  _full_text = """# Mavros message: ONBOARDCOMPUTERSTATUS

std_msgs/Header header

uint8 component               # See enum MAV_COMPONENT

uint32 uptime               # [ms] Time since system boot
uint8 type                  # Type of the onboard computer: 0: Mission computer primary, 1: Mission computer backup 1, 2: Mission computer backup 2, 3: Compute node, 4-5: Compute spares, 6-9: Payload computers.
uint8[8] cpu_cores          # CPU usage on the component in percent (100 - idle). A value of UINT8_MAX implies the field is unused.
uint8[10] cpu_combined      # Combined CPU usage as the last 10 slices of 100 MS (a histogram). This allows to identify spikes in load that max out the system, but only for a short amount of time. A value of UINT8_MAX implies the field is unused
uint8[4] gpu_cores          # GPU usage on the component in percent (100 - idle). A value of UINT8_MAX implies the field is unused
uint8[10] gpu_combined      # Combined GPU usage as the last 10 slices of 100 MS (a histogram). This allows to identify spikes in load that max out the system, but only for a short amount of time. A value of UINT8_MAX implies the field is unused.
int8 temperature_board      # [degC] Temperature of the board. A value of INT8_MAX implies the field is unused.
int8[8] temperature_core    # [degC] Temperature of the CPU core. A value of INT8_MAX implies the field is unused.
int16[4] fan_speed          # [rpm] Fan speeds. A value of INT16_MAX implies the field is unused.
uint32 ram_usage            # [MiB] Amount of used RAM on the component system. A value of UINT32_MAX implies the field is unused.
uint32 ram_total            # [MiB] Total amount of RAM on the component system. A value of UINT32_MAX implies the field is unused.
uint32[4] storage_type      # Storage type: 0: HDD, 1: SSD, 2: EMMC, 3: SD card (non-removable), 4: SD card (removable). A value of UINT32_MAX implies the field is unused.
uint32[4] storage_usage     # [MiB] Amount of used storage space on the component system. A value of UINT32_MAX implies the field is unused.
uint32[4] storage_total     # [MiB] Total amount of storage space on the component system. A value of UINT32_MAX implies the field is unused.
uint32[6] link_type         # Link type: 0-9: UART, 10-19: Wired network, 20-29: Wifi, 30-39: Point-to-point proprietary, 40-49: Mesh proprietary.
uint32[6] link_tx_rate      # [KiB/s] Network traffic from the component system. A value of UINT32_MAX implies the field is unused.
uint32[6] link_rx_rate      # [KiB/s] Network traffic to the component system. A value of UINT32_MAX implies the field is unused.
uint32[6] link_tx_max       # [KiB/s] Network capacity from the component system. A value of UINT32_MAX implies the field is unused.
uint32[6] link_rx_max       # [KiB/s] Network capacity to the component system. A value of UINT32_MAX implies the field is unused.
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
  __slots__ = ['header','component','uptime','type','cpu_cores','cpu_combined','gpu_cores','gpu_combined','temperature_board','temperature_core','fan_speed','ram_usage','ram_total','storage_type','storage_usage','storage_total','link_type','link_tx_rate','link_rx_rate','link_tx_max','link_rx_max']
  _slot_types = ['std_msgs/Header','uint8','uint32','uint8','uint8[8]','uint8[10]','uint8[4]','uint8[10]','int8','int8[8]','int16[4]','uint32','uint32','uint32[4]','uint32[4]','uint32[4]','uint32[6]','uint32[6]','uint32[6]','uint32[6]','uint32[6]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,component,uptime,type,cpu_cores,cpu_combined,gpu_cores,gpu_combined,temperature_board,temperature_core,fan_speed,ram_usage,ram_total,storage_type,storage_usage,storage_total,link_type,link_tx_rate,link_rx_rate,link_tx_max,link_rx_max

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(OnboardComputerStatus, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.component is None:
        self.component = 0
      if self.uptime is None:
        self.uptime = 0
      if self.type is None:
        self.type = 0
      if self.cpu_cores is None:
        self.cpu_cores = b'\0'*8
      if self.cpu_combined is None:
        self.cpu_combined = b'\0'*10
      if self.gpu_cores is None:
        self.gpu_cores = b'\0'*4
      if self.gpu_combined is None:
        self.gpu_combined = b'\0'*10
      if self.temperature_board is None:
        self.temperature_board = 0
      if self.temperature_core is None:
        self.temperature_core = [0] * 8
      if self.fan_speed is None:
        self.fan_speed = [0] * 4
      if self.ram_usage is None:
        self.ram_usage = 0
      if self.ram_total is None:
        self.ram_total = 0
      if self.storage_type is None:
        self.storage_type = [0] * 4
      if self.storage_usage is None:
        self.storage_usage = [0] * 4
      if self.storage_total is None:
        self.storage_total = [0] * 4
      if self.link_type is None:
        self.link_type = [0] * 6
      if self.link_tx_rate is None:
        self.link_tx_rate = [0] * 6
      if self.link_rx_rate is None:
        self.link_rx_rate = [0] * 6
      if self.link_tx_max is None:
        self.link_tx_max = [0] * 6
      if self.link_rx_max is None:
        self.link_rx_max = [0] * 6
    else:
      self.header = std_msgs.msg.Header()
      self.component = 0
      self.uptime = 0
      self.type = 0
      self.cpu_cores = b'\0'*8
      self.cpu_combined = b'\0'*10
      self.gpu_cores = b'\0'*4
      self.gpu_combined = b'\0'*10
      self.temperature_board = 0
      self.temperature_core = [0] * 8
      self.fan_speed = [0] * 4
      self.ram_usage = 0
      self.ram_total = 0
      self.storage_type = [0] * 4
      self.storage_usage = [0] * 4
      self.storage_total = [0] * 4
      self.link_type = [0] * 6
      self.link_tx_rate = [0] * 6
      self.link_rx_rate = [0] * 6
      self.link_tx_max = [0] * 6
      self.link_rx_max = [0] * 6

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
      buff.write(_get_struct_BIB().pack(_x.component, _x.uptime, _x.type))
      _x = self.cpu_cores
      # - if encoded as a list instead, serialize as bytes instead of string
      if type(_x) in [list, tuple]:
        buff.write(_get_struct_8B().pack(*_x))
      else:
        buff.write(_get_struct_8s().pack(_x))
      _x = self.cpu_combined
      # - if encoded as a list instead, serialize as bytes instead of string
      if type(_x) in [list, tuple]:
        buff.write(_get_struct_10B().pack(*_x))
      else:
        buff.write(_get_struct_10s().pack(_x))
      _x = self.gpu_cores
      # - if encoded as a list instead, serialize as bytes instead of string
      if type(_x) in [list, tuple]:
        buff.write(_get_struct_4B().pack(*_x))
      else:
        buff.write(_get_struct_4s().pack(_x))
      _x = self.gpu_combined
      # - if encoded as a list instead, serialize as bytes instead of string
      if type(_x) in [list, tuple]:
        buff.write(_get_struct_10B().pack(*_x))
      else:
        buff.write(_get_struct_10s().pack(_x))
      _x = self.temperature_board
      buff.write(_get_struct_b().pack(_x))
      buff.write(_get_struct_8b().pack(*self.temperature_core))
      buff.write(_get_struct_4h().pack(*self.fan_speed))
      _x = self
      buff.write(_get_struct_2I().pack(_x.ram_usage, _x.ram_total))
      buff.write(_get_struct_4I().pack(*self.storage_type))
      buff.write(_get_struct_4I().pack(*self.storage_usage))
      buff.write(_get_struct_4I().pack(*self.storage_total))
      buff.write(_get_struct_6I().pack(*self.link_type))
      buff.write(_get_struct_6I().pack(*self.link_tx_rate))
      buff.write(_get_struct_6I().pack(*self.link_rx_rate))
      buff.write(_get_struct_6I().pack(*self.link_tx_max))
      buff.write(_get_struct_6I().pack(*self.link_rx_max))
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
      end += 6
      (_x.component, _x.uptime, _x.type,) = _get_struct_BIB().unpack(str[start:end])
      start = end
      end += 8
      self.cpu_cores = str[start:end]
      start = end
      end += 10
      self.cpu_combined = str[start:end]
      start = end
      end += 4
      self.gpu_cores = str[start:end]
      start = end
      end += 10
      self.gpu_combined = str[start:end]
      start = end
      end += 1
      (self.temperature_board,) = _get_struct_b().unpack(str[start:end])
      start = end
      end += 8
      self.temperature_core = _get_struct_8b().unpack(str[start:end])
      start = end
      end += 8
      self.fan_speed = _get_struct_4h().unpack(str[start:end])
      _x = self
      start = end
      end += 8
      (_x.ram_usage, _x.ram_total,) = _get_struct_2I().unpack(str[start:end])
      start = end
      end += 16
      self.storage_type = _get_struct_4I().unpack(str[start:end])
      start = end
      end += 16
      self.storage_usage = _get_struct_4I().unpack(str[start:end])
      start = end
      end += 16
      self.storage_total = _get_struct_4I().unpack(str[start:end])
      start = end
      end += 24
      self.link_type = _get_struct_6I().unpack(str[start:end])
      start = end
      end += 24
      self.link_tx_rate = _get_struct_6I().unpack(str[start:end])
      start = end
      end += 24
      self.link_rx_rate = _get_struct_6I().unpack(str[start:end])
      start = end
      end += 24
      self.link_tx_max = _get_struct_6I().unpack(str[start:end])
      start = end
      end += 24
      self.link_rx_max = _get_struct_6I().unpack(str[start:end])
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
      buff.write(_get_struct_BIB().pack(_x.component, _x.uptime, _x.type))
      _x = self.cpu_cores
      # - if encoded as a list instead, serialize as bytes instead of string
      if type(_x) in [list, tuple]:
        buff.write(_get_struct_8B().pack(*_x))
      else:
        buff.write(_get_struct_8s().pack(_x))
      _x = self.cpu_combined
      # - if encoded as a list instead, serialize as bytes instead of string
      if type(_x) in [list, tuple]:
        buff.write(_get_struct_10B().pack(*_x))
      else:
        buff.write(_get_struct_10s().pack(_x))
      _x = self.gpu_cores
      # - if encoded as a list instead, serialize as bytes instead of string
      if type(_x) in [list, tuple]:
        buff.write(_get_struct_4B().pack(*_x))
      else:
        buff.write(_get_struct_4s().pack(_x))
      _x = self.gpu_combined
      # - if encoded as a list instead, serialize as bytes instead of string
      if type(_x) in [list, tuple]:
        buff.write(_get_struct_10B().pack(*_x))
      else:
        buff.write(_get_struct_10s().pack(_x))
      _x = self.temperature_board
      buff.write(_get_struct_b().pack(_x))
      buff.write(self.temperature_core.tostring())
      buff.write(self.fan_speed.tostring())
      _x = self
      buff.write(_get_struct_2I().pack(_x.ram_usage, _x.ram_total))
      buff.write(self.storage_type.tostring())
      buff.write(self.storage_usage.tostring())
      buff.write(self.storage_total.tostring())
      buff.write(self.link_type.tostring())
      buff.write(self.link_tx_rate.tostring())
      buff.write(self.link_rx_rate.tostring())
      buff.write(self.link_tx_max.tostring())
      buff.write(self.link_rx_max.tostring())
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
      end += 6
      (_x.component, _x.uptime, _x.type,) = _get_struct_BIB().unpack(str[start:end])
      start = end
      end += 8
      self.cpu_cores = str[start:end]
      start = end
      end += 10
      self.cpu_combined = str[start:end]
      start = end
      end += 4
      self.gpu_cores = str[start:end]
      start = end
      end += 10
      self.gpu_combined = str[start:end]
      start = end
      end += 1
      (self.temperature_board,) = _get_struct_b().unpack(str[start:end])
      start = end
      end += 8
      self.temperature_core = numpy.frombuffer(str[start:end], dtype=numpy.int8, count=8)
      start = end
      end += 8
      self.fan_speed = numpy.frombuffer(str[start:end], dtype=numpy.int16, count=4)
      _x = self
      start = end
      end += 8
      (_x.ram_usage, _x.ram_total,) = _get_struct_2I().unpack(str[start:end])
      start = end
      end += 16
      self.storage_type = numpy.frombuffer(str[start:end], dtype=numpy.uint32, count=4)
      start = end
      end += 16
      self.storage_usage = numpy.frombuffer(str[start:end], dtype=numpy.uint32, count=4)
      start = end
      end += 16
      self.storage_total = numpy.frombuffer(str[start:end], dtype=numpy.uint32, count=4)
      start = end
      end += 24
      self.link_type = numpy.frombuffer(str[start:end], dtype=numpy.uint32, count=6)
      start = end
      end += 24
      self.link_tx_rate = numpy.frombuffer(str[start:end], dtype=numpy.uint32, count=6)
      start = end
      end += 24
      self.link_rx_rate = numpy.frombuffer(str[start:end], dtype=numpy.uint32, count=6)
      start = end
      end += 24
      self.link_tx_max = numpy.frombuffer(str[start:end], dtype=numpy.uint32, count=6)
      start = end
      end += 24
      self.link_rx_max = numpy.frombuffer(str[start:end], dtype=numpy.uint32, count=6)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_10B = None
def _get_struct_10B():
    global _struct_10B
    if _struct_10B is None:
        _struct_10B = struct.Struct("<10B")
    return _struct_10B
_struct_10s = None
def _get_struct_10s():
    global _struct_10s
    if _struct_10s is None:
        _struct_10s = struct.Struct("<10s")
    return _struct_10s
_struct_2I = None
def _get_struct_2I():
    global _struct_2I
    if _struct_2I is None:
        _struct_2I = struct.Struct("<2I")
    return _struct_2I
_struct_3I = None
def _get_struct_3I():
    global _struct_3I
    if _struct_3I is None:
        _struct_3I = struct.Struct("<3I")
    return _struct_3I
_struct_4B = None
def _get_struct_4B():
    global _struct_4B
    if _struct_4B is None:
        _struct_4B = struct.Struct("<4B")
    return _struct_4B
_struct_4I = None
def _get_struct_4I():
    global _struct_4I
    if _struct_4I is None:
        _struct_4I = struct.Struct("<4I")
    return _struct_4I
_struct_4h = None
def _get_struct_4h():
    global _struct_4h
    if _struct_4h is None:
        _struct_4h = struct.Struct("<4h")
    return _struct_4h
_struct_4s = None
def _get_struct_4s():
    global _struct_4s
    if _struct_4s is None:
        _struct_4s = struct.Struct("<4s")
    return _struct_4s
_struct_6I = None
def _get_struct_6I():
    global _struct_6I
    if _struct_6I is None:
        _struct_6I = struct.Struct("<6I")
    return _struct_6I
_struct_8B = None
def _get_struct_8B():
    global _struct_8B
    if _struct_8B is None:
        _struct_8B = struct.Struct("<8B")
    return _struct_8B
_struct_8b = None
def _get_struct_8b():
    global _struct_8b
    if _struct_8b is None:
        _struct_8b = struct.Struct("<8b")
    return _struct_8b
_struct_8s = None
def _get_struct_8s():
    global _struct_8s
    if _struct_8s is None:
        _struct_8s = struct.Struct("<8s")
    return _struct_8s
_struct_BIB = None
def _get_struct_BIB():
    global _struct_BIB
    if _struct_BIB is None:
        _struct_BIB = struct.Struct("<BIB")
    return _struct_BIB
_struct_b = None
def _get_struct_b():
    global _struct_b
    if _struct_b is None:
        _struct_b = struct.Struct("<b")
    return _struct_b
