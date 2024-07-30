# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from msg_check/SwDataMsg.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import geometry_msgs.msg
import std_msgs.msg

class SwDataMsg(genpy.Message):
  _md5sum = "da1fe80469bc6c23b275683797085357"
  _type = "msg_check/SwDataMsg"
  _has_header = True  # flag to mark the presence of a Header object
  _full_text = """Header header
geometry_msgs/Vector3 sp
geometry_msgs/Vector3 sq
geometry_msgs/Vector3 acceleration
geometry_msgs/Vector3 angular_acceleration
geometry_msgs/Vector3 position_error
geometry_msgs/Vector3 velocity_error
geometry_msgs/Vector3 position_error_integral
geometry_msgs/Vector3 angle_error
geometry_msgs/Vector3 angle_rate_error
geometry_msgs/Vector3 Kp0_1
geometry_msgs/Vector3 Kp1_1
geometry_msgs/Vector3 Kp0_2
geometry_msgs/Vector3 Kp1_2
geometry_msgs/Vector3 Kp0_3
geometry_msgs/Vector3 Kp1_3
geometry_msgs/Vector3 Kq0_1
geometry_msgs/Vector3 Kq1_1
geometry_msgs/Vector3 Kq2_1
geometry_msgs/Vector3 Kq0_2
geometry_msgs/Vector3 Kq1_2
geometry_msgs/Vector3 Kq2_2
geometry_msgs/Vector3 Kq0_3
geometry_msgs/Vector3 Kq1_3
geometry_msgs/Vector3 Kq2_3
geometry_msgs/Vector3 rho_p0_1
geometry_msgs/Vector3 rho_p1_1
geometry_msgs/Vector3 rho_p0_2
geometry_msgs/Vector3 rho_p1_2
geometry_msgs/Vector3 rho_p0_3
geometry_msgs/Vector3 rho_p1_3
geometry_msgs/Vector3 rho_q0_1
geometry_msgs/Vector3 rho_q1_1
geometry_msgs/Vector3 rho_q2_1
geometry_msgs/Vector3 rho_q0_2
geometry_msgs/Vector3 rho_q1_2
geometry_msgs/Vector3 rho_q2_2
geometry_msgs/Vector3 rho_q0_3
geometry_msgs/Vector3 rho_Q1_3
geometry_msgs/Vector3 rho_q2_3
geometry_msgs/Vector3 zeta_p
geometry_msgs/Vector3 zeta_q
geometry_msgs/Vector3 delTau_p
geometry_msgs/Vector3 delTau_q
geometry_msgs/Vector3 moments
float64 thrust
float64 hatM_1
float64 hatM_2
float64 hatM_3

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

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z"""
  __slots__ = ['header','sp','sq','acceleration','angular_acceleration','position_error','velocity_error','position_error_integral','angle_error','angle_rate_error','Kp0_1','Kp1_1','Kp0_2','Kp1_2','Kp0_3','Kp1_3','Kq0_1','Kq1_1','Kq2_1','Kq0_2','Kq1_2','Kq2_2','Kq0_3','Kq1_3','Kq2_3','rho_p0_1','rho_p1_1','rho_p0_2','rho_p1_2','rho_p0_3','rho_p1_3','rho_q0_1','rho_q1_1','rho_q2_1','rho_q0_2','rho_q1_2','rho_q2_2','rho_q0_3','rho_Q1_3','rho_q2_3','zeta_p','zeta_q','delTau_p','delTau_q','moments','thrust','hatM_1','hatM_2','hatM_3']
  _slot_types = ['std_msgs/Header','geometry_msgs/Vector3','geometry_msgs/Vector3','geometry_msgs/Vector3','geometry_msgs/Vector3','geometry_msgs/Vector3','geometry_msgs/Vector3','geometry_msgs/Vector3','geometry_msgs/Vector3','geometry_msgs/Vector3','geometry_msgs/Vector3','geometry_msgs/Vector3','geometry_msgs/Vector3','geometry_msgs/Vector3','geometry_msgs/Vector3','geometry_msgs/Vector3','geometry_msgs/Vector3','geometry_msgs/Vector3','geometry_msgs/Vector3','geometry_msgs/Vector3','geometry_msgs/Vector3','geometry_msgs/Vector3','geometry_msgs/Vector3','geometry_msgs/Vector3','geometry_msgs/Vector3','geometry_msgs/Vector3','geometry_msgs/Vector3','geometry_msgs/Vector3','geometry_msgs/Vector3','geometry_msgs/Vector3','geometry_msgs/Vector3','geometry_msgs/Vector3','geometry_msgs/Vector3','geometry_msgs/Vector3','geometry_msgs/Vector3','geometry_msgs/Vector3','geometry_msgs/Vector3','geometry_msgs/Vector3','geometry_msgs/Vector3','geometry_msgs/Vector3','geometry_msgs/Vector3','geometry_msgs/Vector3','geometry_msgs/Vector3','geometry_msgs/Vector3','geometry_msgs/Vector3','float64','float64','float64','float64']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,sp,sq,acceleration,angular_acceleration,position_error,velocity_error,position_error_integral,angle_error,angle_rate_error,Kp0_1,Kp1_1,Kp0_2,Kp1_2,Kp0_3,Kp1_3,Kq0_1,Kq1_1,Kq2_1,Kq0_2,Kq1_2,Kq2_2,Kq0_3,Kq1_3,Kq2_3,rho_p0_1,rho_p1_1,rho_p0_2,rho_p1_2,rho_p0_3,rho_p1_3,rho_q0_1,rho_q1_1,rho_q2_1,rho_q0_2,rho_q1_2,rho_q2_2,rho_q0_3,rho_Q1_3,rho_q2_3,zeta_p,zeta_q,delTau_p,delTau_q,moments,thrust,hatM_1,hatM_2,hatM_3

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(SwDataMsg, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.sp is None:
        self.sp = geometry_msgs.msg.Vector3()
      if self.sq is None:
        self.sq = geometry_msgs.msg.Vector3()
      if self.acceleration is None:
        self.acceleration = geometry_msgs.msg.Vector3()
      if self.angular_acceleration is None:
        self.angular_acceleration = geometry_msgs.msg.Vector3()
      if self.position_error is None:
        self.position_error = geometry_msgs.msg.Vector3()
      if self.velocity_error is None:
        self.velocity_error = geometry_msgs.msg.Vector3()
      if self.position_error_integral is None:
        self.position_error_integral = geometry_msgs.msg.Vector3()
      if self.angle_error is None:
        self.angle_error = geometry_msgs.msg.Vector3()
      if self.angle_rate_error is None:
        self.angle_rate_error = geometry_msgs.msg.Vector3()
      if self.Kp0_1 is None:
        self.Kp0_1 = geometry_msgs.msg.Vector3()
      if self.Kp1_1 is None:
        self.Kp1_1 = geometry_msgs.msg.Vector3()
      if self.Kp0_2 is None:
        self.Kp0_2 = geometry_msgs.msg.Vector3()
      if self.Kp1_2 is None:
        self.Kp1_2 = geometry_msgs.msg.Vector3()
      if self.Kp0_3 is None:
        self.Kp0_3 = geometry_msgs.msg.Vector3()
      if self.Kp1_3 is None:
        self.Kp1_3 = geometry_msgs.msg.Vector3()
      if self.Kq0_1 is None:
        self.Kq0_1 = geometry_msgs.msg.Vector3()
      if self.Kq1_1 is None:
        self.Kq1_1 = geometry_msgs.msg.Vector3()
      if self.Kq2_1 is None:
        self.Kq2_1 = geometry_msgs.msg.Vector3()
      if self.Kq0_2 is None:
        self.Kq0_2 = geometry_msgs.msg.Vector3()
      if self.Kq1_2 is None:
        self.Kq1_2 = geometry_msgs.msg.Vector3()
      if self.Kq2_2 is None:
        self.Kq2_2 = geometry_msgs.msg.Vector3()
      if self.Kq0_3 is None:
        self.Kq0_3 = geometry_msgs.msg.Vector3()
      if self.Kq1_3 is None:
        self.Kq1_3 = geometry_msgs.msg.Vector3()
      if self.Kq2_3 is None:
        self.Kq2_3 = geometry_msgs.msg.Vector3()
      if self.rho_p0_1 is None:
        self.rho_p0_1 = geometry_msgs.msg.Vector3()
      if self.rho_p1_1 is None:
        self.rho_p1_1 = geometry_msgs.msg.Vector3()
      if self.rho_p0_2 is None:
        self.rho_p0_2 = geometry_msgs.msg.Vector3()
      if self.rho_p1_2 is None:
        self.rho_p1_2 = geometry_msgs.msg.Vector3()
      if self.rho_p0_3 is None:
        self.rho_p0_3 = geometry_msgs.msg.Vector3()
      if self.rho_p1_3 is None:
        self.rho_p1_3 = geometry_msgs.msg.Vector3()
      if self.rho_q0_1 is None:
        self.rho_q0_1 = geometry_msgs.msg.Vector3()
      if self.rho_q1_1 is None:
        self.rho_q1_1 = geometry_msgs.msg.Vector3()
      if self.rho_q2_1 is None:
        self.rho_q2_1 = geometry_msgs.msg.Vector3()
      if self.rho_q0_2 is None:
        self.rho_q0_2 = geometry_msgs.msg.Vector3()
      if self.rho_q1_2 is None:
        self.rho_q1_2 = geometry_msgs.msg.Vector3()
      if self.rho_q2_2 is None:
        self.rho_q2_2 = geometry_msgs.msg.Vector3()
      if self.rho_q0_3 is None:
        self.rho_q0_3 = geometry_msgs.msg.Vector3()
      if self.rho_Q1_3 is None:
        self.rho_Q1_3 = geometry_msgs.msg.Vector3()
      if self.rho_q2_3 is None:
        self.rho_q2_3 = geometry_msgs.msg.Vector3()
      if self.zeta_p is None:
        self.zeta_p = geometry_msgs.msg.Vector3()
      if self.zeta_q is None:
        self.zeta_q = geometry_msgs.msg.Vector3()
      if self.delTau_p is None:
        self.delTau_p = geometry_msgs.msg.Vector3()
      if self.delTau_q is None:
        self.delTau_q = geometry_msgs.msg.Vector3()
      if self.moments is None:
        self.moments = geometry_msgs.msg.Vector3()
      if self.thrust is None:
        self.thrust = 0.
      if self.hatM_1 is None:
        self.hatM_1 = 0.
      if self.hatM_2 is None:
        self.hatM_2 = 0.
      if self.hatM_3 is None:
        self.hatM_3 = 0.
    else:
      self.header = std_msgs.msg.Header()
      self.sp = geometry_msgs.msg.Vector3()
      self.sq = geometry_msgs.msg.Vector3()
      self.acceleration = geometry_msgs.msg.Vector3()
      self.angular_acceleration = geometry_msgs.msg.Vector3()
      self.position_error = geometry_msgs.msg.Vector3()
      self.velocity_error = geometry_msgs.msg.Vector3()
      self.position_error_integral = geometry_msgs.msg.Vector3()
      self.angle_error = geometry_msgs.msg.Vector3()
      self.angle_rate_error = geometry_msgs.msg.Vector3()
      self.Kp0_1 = geometry_msgs.msg.Vector3()
      self.Kp1_1 = geometry_msgs.msg.Vector3()
      self.Kp0_2 = geometry_msgs.msg.Vector3()
      self.Kp1_2 = geometry_msgs.msg.Vector3()
      self.Kp0_3 = geometry_msgs.msg.Vector3()
      self.Kp1_3 = geometry_msgs.msg.Vector3()
      self.Kq0_1 = geometry_msgs.msg.Vector3()
      self.Kq1_1 = geometry_msgs.msg.Vector3()
      self.Kq2_1 = geometry_msgs.msg.Vector3()
      self.Kq0_2 = geometry_msgs.msg.Vector3()
      self.Kq1_2 = geometry_msgs.msg.Vector3()
      self.Kq2_2 = geometry_msgs.msg.Vector3()
      self.Kq0_3 = geometry_msgs.msg.Vector3()
      self.Kq1_3 = geometry_msgs.msg.Vector3()
      self.Kq2_3 = geometry_msgs.msg.Vector3()
      self.rho_p0_1 = geometry_msgs.msg.Vector3()
      self.rho_p1_1 = geometry_msgs.msg.Vector3()
      self.rho_p0_2 = geometry_msgs.msg.Vector3()
      self.rho_p1_2 = geometry_msgs.msg.Vector3()
      self.rho_p0_3 = geometry_msgs.msg.Vector3()
      self.rho_p1_3 = geometry_msgs.msg.Vector3()
      self.rho_q0_1 = geometry_msgs.msg.Vector3()
      self.rho_q1_1 = geometry_msgs.msg.Vector3()
      self.rho_q2_1 = geometry_msgs.msg.Vector3()
      self.rho_q0_2 = geometry_msgs.msg.Vector3()
      self.rho_q1_2 = geometry_msgs.msg.Vector3()
      self.rho_q2_2 = geometry_msgs.msg.Vector3()
      self.rho_q0_3 = geometry_msgs.msg.Vector3()
      self.rho_Q1_3 = geometry_msgs.msg.Vector3()
      self.rho_q2_3 = geometry_msgs.msg.Vector3()
      self.zeta_p = geometry_msgs.msg.Vector3()
      self.zeta_q = geometry_msgs.msg.Vector3()
      self.delTau_p = geometry_msgs.msg.Vector3()
      self.delTau_q = geometry_msgs.msg.Vector3()
      self.moments = geometry_msgs.msg.Vector3()
      self.thrust = 0.
      self.hatM_1 = 0.
      self.hatM_2 = 0.
      self.hatM_3 = 0.

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
      buff.write(_get_struct_136d().pack(_x.sp.x, _x.sp.y, _x.sp.z, _x.sq.x, _x.sq.y, _x.sq.z, _x.acceleration.x, _x.acceleration.y, _x.acceleration.z, _x.angular_acceleration.x, _x.angular_acceleration.y, _x.angular_acceleration.z, _x.position_error.x, _x.position_error.y, _x.position_error.z, _x.velocity_error.x, _x.velocity_error.y, _x.velocity_error.z, _x.position_error_integral.x, _x.position_error_integral.y, _x.position_error_integral.z, _x.angle_error.x, _x.angle_error.y, _x.angle_error.z, _x.angle_rate_error.x, _x.angle_rate_error.y, _x.angle_rate_error.z, _x.Kp0_1.x, _x.Kp0_1.y, _x.Kp0_1.z, _x.Kp1_1.x, _x.Kp1_1.y, _x.Kp1_1.z, _x.Kp0_2.x, _x.Kp0_2.y, _x.Kp0_2.z, _x.Kp1_2.x, _x.Kp1_2.y, _x.Kp1_2.z, _x.Kp0_3.x, _x.Kp0_3.y, _x.Kp0_3.z, _x.Kp1_3.x, _x.Kp1_3.y, _x.Kp1_3.z, _x.Kq0_1.x, _x.Kq0_1.y, _x.Kq0_1.z, _x.Kq1_1.x, _x.Kq1_1.y, _x.Kq1_1.z, _x.Kq2_1.x, _x.Kq2_1.y, _x.Kq2_1.z, _x.Kq0_2.x, _x.Kq0_2.y, _x.Kq0_2.z, _x.Kq1_2.x, _x.Kq1_2.y, _x.Kq1_2.z, _x.Kq2_2.x, _x.Kq2_2.y, _x.Kq2_2.z, _x.Kq0_3.x, _x.Kq0_3.y, _x.Kq0_3.z, _x.Kq1_3.x, _x.Kq1_3.y, _x.Kq1_3.z, _x.Kq2_3.x, _x.Kq2_3.y, _x.Kq2_3.z, _x.rho_p0_1.x, _x.rho_p0_1.y, _x.rho_p0_1.z, _x.rho_p1_1.x, _x.rho_p1_1.y, _x.rho_p1_1.z, _x.rho_p0_2.x, _x.rho_p0_2.y, _x.rho_p0_2.z, _x.rho_p1_2.x, _x.rho_p1_2.y, _x.rho_p1_2.z, _x.rho_p0_3.x, _x.rho_p0_3.y, _x.rho_p0_3.z, _x.rho_p1_3.x, _x.rho_p1_3.y, _x.rho_p1_3.z, _x.rho_q0_1.x, _x.rho_q0_1.y, _x.rho_q0_1.z, _x.rho_q1_1.x, _x.rho_q1_1.y, _x.rho_q1_1.z, _x.rho_q2_1.x, _x.rho_q2_1.y, _x.rho_q2_1.z, _x.rho_q0_2.x, _x.rho_q0_2.y, _x.rho_q0_2.z, _x.rho_q1_2.x, _x.rho_q1_2.y, _x.rho_q1_2.z, _x.rho_q2_2.x, _x.rho_q2_2.y, _x.rho_q2_2.z, _x.rho_q0_3.x, _x.rho_q0_3.y, _x.rho_q0_3.z, _x.rho_Q1_3.x, _x.rho_Q1_3.y, _x.rho_Q1_3.z, _x.rho_q2_3.x, _x.rho_q2_3.y, _x.rho_q2_3.z, _x.zeta_p.x, _x.zeta_p.y, _x.zeta_p.z, _x.zeta_q.x, _x.zeta_q.y, _x.zeta_q.z, _x.delTau_p.x, _x.delTau_p.y, _x.delTau_p.z, _x.delTau_q.x, _x.delTau_q.y, _x.delTau_q.z, _x.moments.x, _x.moments.y, _x.moments.z, _x.thrust, _x.hatM_1, _x.hatM_2, _x.hatM_3))
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
      if self.sp is None:
        self.sp = geometry_msgs.msg.Vector3()
      if self.sq is None:
        self.sq = geometry_msgs.msg.Vector3()
      if self.acceleration is None:
        self.acceleration = geometry_msgs.msg.Vector3()
      if self.angular_acceleration is None:
        self.angular_acceleration = geometry_msgs.msg.Vector3()
      if self.position_error is None:
        self.position_error = geometry_msgs.msg.Vector3()
      if self.velocity_error is None:
        self.velocity_error = geometry_msgs.msg.Vector3()
      if self.position_error_integral is None:
        self.position_error_integral = geometry_msgs.msg.Vector3()
      if self.angle_error is None:
        self.angle_error = geometry_msgs.msg.Vector3()
      if self.angle_rate_error is None:
        self.angle_rate_error = geometry_msgs.msg.Vector3()
      if self.Kp0_1 is None:
        self.Kp0_1 = geometry_msgs.msg.Vector3()
      if self.Kp1_1 is None:
        self.Kp1_1 = geometry_msgs.msg.Vector3()
      if self.Kp0_2 is None:
        self.Kp0_2 = geometry_msgs.msg.Vector3()
      if self.Kp1_2 is None:
        self.Kp1_2 = geometry_msgs.msg.Vector3()
      if self.Kp0_3 is None:
        self.Kp0_3 = geometry_msgs.msg.Vector3()
      if self.Kp1_3 is None:
        self.Kp1_3 = geometry_msgs.msg.Vector3()
      if self.Kq0_1 is None:
        self.Kq0_1 = geometry_msgs.msg.Vector3()
      if self.Kq1_1 is None:
        self.Kq1_1 = geometry_msgs.msg.Vector3()
      if self.Kq2_1 is None:
        self.Kq2_1 = geometry_msgs.msg.Vector3()
      if self.Kq0_2 is None:
        self.Kq0_2 = geometry_msgs.msg.Vector3()
      if self.Kq1_2 is None:
        self.Kq1_2 = geometry_msgs.msg.Vector3()
      if self.Kq2_2 is None:
        self.Kq2_2 = geometry_msgs.msg.Vector3()
      if self.Kq0_3 is None:
        self.Kq0_3 = geometry_msgs.msg.Vector3()
      if self.Kq1_3 is None:
        self.Kq1_3 = geometry_msgs.msg.Vector3()
      if self.Kq2_3 is None:
        self.Kq2_3 = geometry_msgs.msg.Vector3()
      if self.rho_p0_1 is None:
        self.rho_p0_1 = geometry_msgs.msg.Vector3()
      if self.rho_p1_1 is None:
        self.rho_p1_1 = geometry_msgs.msg.Vector3()
      if self.rho_p0_2 is None:
        self.rho_p0_2 = geometry_msgs.msg.Vector3()
      if self.rho_p1_2 is None:
        self.rho_p1_2 = geometry_msgs.msg.Vector3()
      if self.rho_p0_3 is None:
        self.rho_p0_3 = geometry_msgs.msg.Vector3()
      if self.rho_p1_3 is None:
        self.rho_p1_3 = geometry_msgs.msg.Vector3()
      if self.rho_q0_1 is None:
        self.rho_q0_1 = geometry_msgs.msg.Vector3()
      if self.rho_q1_1 is None:
        self.rho_q1_1 = geometry_msgs.msg.Vector3()
      if self.rho_q2_1 is None:
        self.rho_q2_1 = geometry_msgs.msg.Vector3()
      if self.rho_q0_2 is None:
        self.rho_q0_2 = geometry_msgs.msg.Vector3()
      if self.rho_q1_2 is None:
        self.rho_q1_2 = geometry_msgs.msg.Vector3()
      if self.rho_q2_2 is None:
        self.rho_q2_2 = geometry_msgs.msg.Vector3()
      if self.rho_q0_3 is None:
        self.rho_q0_3 = geometry_msgs.msg.Vector3()
      if self.rho_Q1_3 is None:
        self.rho_Q1_3 = geometry_msgs.msg.Vector3()
      if self.rho_q2_3 is None:
        self.rho_q2_3 = geometry_msgs.msg.Vector3()
      if self.zeta_p is None:
        self.zeta_p = geometry_msgs.msg.Vector3()
      if self.zeta_q is None:
        self.zeta_q = geometry_msgs.msg.Vector3()
      if self.delTau_p is None:
        self.delTau_p = geometry_msgs.msg.Vector3()
      if self.delTau_q is None:
        self.delTau_q = geometry_msgs.msg.Vector3()
      if self.moments is None:
        self.moments = geometry_msgs.msg.Vector3()
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
      end += 1088
      (_x.sp.x, _x.sp.y, _x.sp.z, _x.sq.x, _x.sq.y, _x.sq.z, _x.acceleration.x, _x.acceleration.y, _x.acceleration.z, _x.angular_acceleration.x, _x.angular_acceleration.y, _x.angular_acceleration.z, _x.position_error.x, _x.position_error.y, _x.position_error.z, _x.velocity_error.x, _x.velocity_error.y, _x.velocity_error.z, _x.position_error_integral.x, _x.position_error_integral.y, _x.position_error_integral.z, _x.angle_error.x, _x.angle_error.y, _x.angle_error.z, _x.angle_rate_error.x, _x.angle_rate_error.y, _x.angle_rate_error.z, _x.Kp0_1.x, _x.Kp0_1.y, _x.Kp0_1.z, _x.Kp1_1.x, _x.Kp1_1.y, _x.Kp1_1.z, _x.Kp0_2.x, _x.Kp0_2.y, _x.Kp0_2.z, _x.Kp1_2.x, _x.Kp1_2.y, _x.Kp1_2.z, _x.Kp0_3.x, _x.Kp0_3.y, _x.Kp0_3.z, _x.Kp1_3.x, _x.Kp1_3.y, _x.Kp1_3.z, _x.Kq0_1.x, _x.Kq0_1.y, _x.Kq0_1.z, _x.Kq1_1.x, _x.Kq1_1.y, _x.Kq1_1.z, _x.Kq2_1.x, _x.Kq2_1.y, _x.Kq2_1.z, _x.Kq0_2.x, _x.Kq0_2.y, _x.Kq0_2.z, _x.Kq1_2.x, _x.Kq1_2.y, _x.Kq1_2.z, _x.Kq2_2.x, _x.Kq2_2.y, _x.Kq2_2.z, _x.Kq0_3.x, _x.Kq0_3.y, _x.Kq0_3.z, _x.Kq1_3.x, _x.Kq1_3.y, _x.Kq1_3.z, _x.Kq2_3.x, _x.Kq2_3.y, _x.Kq2_3.z, _x.rho_p0_1.x, _x.rho_p0_1.y, _x.rho_p0_1.z, _x.rho_p1_1.x, _x.rho_p1_1.y, _x.rho_p1_1.z, _x.rho_p0_2.x, _x.rho_p0_2.y, _x.rho_p0_2.z, _x.rho_p1_2.x, _x.rho_p1_2.y, _x.rho_p1_2.z, _x.rho_p0_3.x, _x.rho_p0_3.y, _x.rho_p0_3.z, _x.rho_p1_3.x, _x.rho_p1_3.y, _x.rho_p1_3.z, _x.rho_q0_1.x, _x.rho_q0_1.y, _x.rho_q0_1.z, _x.rho_q1_1.x, _x.rho_q1_1.y, _x.rho_q1_1.z, _x.rho_q2_1.x, _x.rho_q2_1.y, _x.rho_q2_1.z, _x.rho_q0_2.x, _x.rho_q0_2.y, _x.rho_q0_2.z, _x.rho_q1_2.x, _x.rho_q1_2.y, _x.rho_q1_2.z, _x.rho_q2_2.x, _x.rho_q2_2.y, _x.rho_q2_2.z, _x.rho_q0_3.x, _x.rho_q0_3.y, _x.rho_q0_3.z, _x.rho_Q1_3.x, _x.rho_Q1_3.y, _x.rho_Q1_3.z, _x.rho_q2_3.x, _x.rho_q2_3.y, _x.rho_q2_3.z, _x.zeta_p.x, _x.zeta_p.y, _x.zeta_p.z, _x.zeta_q.x, _x.zeta_q.y, _x.zeta_q.z, _x.delTau_p.x, _x.delTau_p.y, _x.delTau_p.z, _x.delTau_q.x, _x.delTau_q.y, _x.delTau_q.z, _x.moments.x, _x.moments.y, _x.moments.z, _x.thrust, _x.hatM_1, _x.hatM_2, _x.hatM_3,) = _get_struct_136d().unpack(str[start:end])
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
      buff.write(_get_struct_136d().pack(_x.sp.x, _x.sp.y, _x.sp.z, _x.sq.x, _x.sq.y, _x.sq.z, _x.acceleration.x, _x.acceleration.y, _x.acceleration.z, _x.angular_acceleration.x, _x.angular_acceleration.y, _x.angular_acceleration.z, _x.position_error.x, _x.position_error.y, _x.position_error.z, _x.velocity_error.x, _x.velocity_error.y, _x.velocity_error.z, _x.position_error_integral.x, _x.position_error_integral.y, _x.position_error_integral.z, _x.angle_error.x, _x.angle_error.y, _x.angle_error.z, _x.angle_rate_error.x, _x.angle_rate_error.y, _x.angle_rate_error.z, _x.Kp0_1.x, _x.Kp0_1.y, _x.Kp0_1.z, _x.Kp1_1.x, _x.Kp1_1.y, _x.Kp1_1.z, _x.Kp0_2.x, _x.Kp0_2.y, _x.Kp0_2.z, _x.Kp1_2.x, _x.Kp1_2.y, _x.Kp1_2.z, _x.Kp0_3.x, _x.Kp0_3.y, _x.Kp0_3.z, _x.Kp1_3.x, _x.Kp1_3.y, _x.Kp1_3.z, _x.Kq0_1.x, _x.Kq0_1.y, _x.Kq0_1.z, _x.Kq1_1.x, _x.Kq1_1.y, _x.Kq1_1.z, _x.Kq2_1.x, _x.Kq2_1.y, _x.Kq2_1.z, _x.Kq0_2.x, _x.Kq0_2.y, _x.Kq0_2.z, _x.Kq1_2.x, _x.Kq1_2.y, _x.Kq1_2.z, _x.Kq2_2.x, _x.Kq2_2.y, _x.Kq2_2.z, _x.Kq0_3.x, _x.Kq0_3.y, _x.Kq0_3.z, _x.Kq1_3.x, _x.Kq1_3.y, _x.Kq1_3.z, _x.Kq2_3.x, _x.Kq2_3.y, _x.Kq2_3.z, _x.rho_p0_1.x, _x.rho_p0_1.y, _x.rho_p0_1.z, _x.rho_p1_1.x, _x.rho_p1_1.y, _x.rho_p1_1.z, _x.rho_p0_2.x, _x.rho_p0_2.y, _x.rho_p0_2.z, _x.rho_p1_2.x, _x.rho_p1_2.y, _x.rho_p1_2.z, _x.rho_p0_3.x, _x.rho_p0_3.y, _x.rho_p0_3.z, _x.rho_p1_3.x, _x.rho_p1_3.y, _x.rho_p1_3.z, _x.rho_q0_1.x, _x.rho_q0_1.y, _x.rho_q0_1.z, _x.rho_q1_1.x, _x.rho_q1_1.y, _x.rho_q1_1.z, _x.rho_q2_1.x, _x.rho_q2_1.y, _x.rho_q2_1.z, _x.rho_q0_2.x, _x.rho_q0_2.y, _x.rho_q0_2.z, _x.rho_q1_2.x, _x.rho_q1_2.y, _x.rho_q1_2.z, _x.rho_q2_2.x, _x.rho_q2_2.y, _x.rho_q2_2.z, _x.rho_q0_3.x, _x.rho_q0_3.y, _x.rho_q0_3.z, _x.rho_Q1_3.x, _x.rho_Q1_3.y, _x.rho_Q1_3.z, _x.rho_q2_3.x, _x.rho_q2_3.y, _x.rho_q2_3.z, _x.zeta_p.x, _x.zeta_p.y, _x.zeta_p.z, _x.zeta_q.x, _x.zeta_q.y, _x.zeta_q.z, _x.delTau_p.x, _x.delTau_p.y, _x.delTau_p.z, _x.delTau_q.x, _x.delTau_q.y, _x.delTau_q.z, _x.moments.x, _x.moments.y, _x.moments.z, _x.thrust, _x.hatM_1, _x.hatM_2, _x.hatM_3))
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
      if self.sp is None:
        self.sp = geometry_msgs.msg.Vector3()
      if self.sq is None:
        self.sq = geometry_msgs.msg.Vector3()
      if self.acceleration is None:
        self.acceleration = geometry_msgs.msg.Vector3()
      if self.angular_acceleration is None:
        self.angular_acceleration = geometry_msgs.msg.Vector3()
      if self.position_error is None:
        self.position_error = geometry_msgs.msg.Vector3()
      if self.velocity_error is None:
        self.velocity_error = geometry_msgs.msg.Vector3()
      if self.position_error_integral is None:
        self.position_error_integral = geometry_msgs.msg.Vector3()
      if self.angle_error is None:
        self.angle_error = geometry_msgs.msg.Vector3()
      if self.angle_rate_error is None:
        self.angle_rate_error = geometry_msgs.msg.Vector3()
      if self.Kp0_1 is None:
        self.Kp0_1 = geometry_msgs.msg.Vector3()
      if self.Kp1_1 is None:
        self.Kp1_1 = geometry_msgs.msg.Vector3()
      if self.Kp0_2 is None:
        self.Kp0_2 = geometry_msgs.msg.Vector3()
      if self.Kp1_2 is None:
        self.Kp1_2 = geometry_msgs.msg.Vector3()
      if self.Kp0_3 is None:
        self.Kp0_3 = geometry_msgs.msg.Vector3()
      if self.Kp1_3 is None:
        self.Kp1_3 = geometry_msgs.msg.Vector3()
      if self.Kq0_1 is None:
        self.Kq0_1 = geometry_msgs.msg.Vector3()
      if self.Kq1_1 is None:
        self.Kq1_1 = geometry_msgs.msg.Vector3()
      if self.Kq2_1 is None:
        self.Kq2_1 = geometry_msgs.msg.Vector3()
      if self.Kq0_2 is None:
        self.Kq0_2 = geometry_msgs.msg.Vector3()
      if self.Kq1_2 is None:
        self.Kq1_2 = geometry_msgs.msg.Vector3()
      if self.Kq2_2 is None:
        self.Kq2_2 = geometry_msgs.msg.Vector3()
      if self.Kq0_3 is None:
        self.Kq0_3 = geometry_msgs.msg.Vector3()
      if self.Kq1_3 is None:
        self.Kq1_3 = geometry_msgs.msg.Vector3()
      if self.Kq2_3 is None:
        self.Kq2_3 = geometry_msgs.msg.Vector3()
      if self.rho_p0_1 is None:
        self.rho_p0_1 = geometry_msgs.msg.Vector3()
      if self.rho_p1_1 is None:
        self.rho_p1_1 = geometry_msgs.msg.Vector3()
      if self.rho_p0_2 is None:
        self.rho_p0_2 = geometry_msgs.msg.Vector3()
      if self.rho_p1_2 is None:
        self.rho_p1_2 = geometry_msgs.msg.Vector3()
      if self.rho_p0_3 is None:
        self.rho_p0_3 = geometry_msgs.msg.Vector3()
      if self.rho_p1_3 is None:
        self.rho_p1_3 = geometry_msgs.msg.Vector3()
      if self.rho_q0_1 is None:
        self.rho_q0_1 = geometry_msgs.msg.Vector3()
      if self.rho_q1_1 is None:
        self.rho_q1_1 = geometry_msgs.msg.Vector3()
      if self.rho_q2_1 is None:
        self.rho_q2_1 = geometry_msgs.msg.Vector3()
      if self.rho_q0_2 is None:
        self.rho_q0_2 = geometry_msgs.msg.Vector3()
      if self.rho_q1_2 is None:
        self.rho_q1_2 = geometry_msgs.msg.Vector3()
      if self.rho_q2_2 is None:
        self.rho_q2_2 = geometry_msgs.msg.Vector3()
      if self.rho_q0_3 is None:
        self.rho_q0_3 = geometry_msgs.msg.Vector3()
      if self.rho_Q1_3 is None:
        self.rho_Q1_3 = geometry_msgs.msg.Vector3()
      if self.rho_q2_3 is None:
        self.rho_q2_3 = geometry_msgs.msg.Vector3()
      if self.zeta_p is None:
        self.zeta_p = geometry_msgs.msg.Vector3()
      if self.zeta_q is None:
        self.zeta_q = geometry_msgs.msg.Vector3()
      if self.delTau_p is None:
        self.delTau_p = geometry_msgs.msg.Vector3()
      if self.delTau_q is None:
        self.delTau_q = geometry_msgs.msg.Vector3()
      if self.moments is None:
        self.moments = geometry_msgs.msg.Vector3()
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
      end += 1088
      (_x.sp.x, _x.sp.y, _x.sp.z, _x.sq.x, _x.sq.y, _x.sq.z, _x.acceleration.x, _x.acceleration.y, _x.acceleration.z, _x.angular_acceleration.x, _x.angular_acceleration.y, _x.angular_acceleration.z, _x.position_error.x, _x.position_error.y, _x.position_error.z, _x.velocity_error.x, _x.velocity_error.y, _x.velocity_error.z, _x.position_error_integral.x, _x.position_error_integral.y, _x.position_error_integral.z, _x.angle_error.x, _x.angle_error.y, _x.angle_error.z, _x.angle_rate_error.x, _x.angle_rate_error.y, _x.angle_rate_error.z, _x.Kp0_1.x, _x.Kp0_1.y, _x.Kp0_1.z, _x.Kp1_1.x, _x.Kp1_1.y, _x.Kp1_1.z, _x.Kp0_2.x, _x.Kp0_2.y, _x.Kp0_2.z, _x.Kp1_2.x, _x.Kp1_2.y, _x.Kp1_2.z, _x.Kp0_3.x, _x.Kp0_3.y, _x.Kp0_3.z, _x.Kp1_3.x, _x.Kp1_3.y, _x.Kp1_3.z, _x.Kq0_1.x, _x.Kq0_1.y, _x.Kq0_1.z, _x.Kq1_1.x, _x.Kq1_1.y, _x.Kq1_1.z, _x.Kq2_1.x, _x.Kq2_1.y, _x.Kq2_1.z, _x.Kq0_2.x, _x.Kq0_2.y, _x.Kq0_2.z, _x.Kq1_2.x, _x.Kq1_2.y, _x.Kq1_2.z, _x.Kq2_2.x, _x.Kq2_2.y, _x.Kq2_2.z, _x.Kq0_3.x, _x.Kq0_3.y, _x.Kq0_3.z, _x.Kq1_3.x, _x.Kq1_3.y, _x.Kq1_3.z, _x.Kq2_3.x, _x.Kq2_3.y, _x.Kq2_3.z, _x.rho_p0_1.x, _x.rho_p0_1.y, _x.rho_p0_1.z, _x.rho_p1_1.x, _x.rho_p1_1.y, _x.rho_p1_1.z, _x.rho_p0_2.x, _x.rho_p0_2.y, _x.rho_p0_2.z, _x.rho_p1_2.x, _x.rho_p1_2.y, _x.rho_p1_2.z, _x.rho_p0_3.x, _x.rho_p0_3.y, _x.rho_p0_3.z, _x.rho_p1_3.x, _x.rho_p1_3.y, _x.rho_p1_3.z, _x.rho_q0_1.x, _x.rho_q0_1.y, _x.rho_q0_1.z, _x.rho_q1_1.x, _x.rho_q1_1.y, _x.rho_q1_1.z, _x.rho_q2_1.x, _x.rho_q2_1.y, _x.rho_q2_1.z, _x.rho_q0_2.x, _x.rho_q0_2.y, _x.rho_q0_2.z, _x.rho_q1_2.x, _x.rho_q1_2.y, _x.rho_q1_2.z, _x.rho_q2_2.x, _x.rho_q2_2.y, _x.rho_q2_2.z, _x.rho_q0_3.x, _x.rho_q0_3.y, _x.rho_q0_3.z, _x.rho_Q1_3.x, _x.rho_Q1_3.y, _x.rho_Q1_3.z, _x.rho_q2_3.x, _x.rho_q2_3.y, _x.rho_q2_3.z, _x.zeta_p.x, _x.zeta_p.y, _x.zeta_p.z, _x.zeta_q.x, _x.zeta_q.y, _x.zeta_q.z, _x.delTau_p.x, _x.delTau_p.y, _x.delTau_p.z, _x.delTau_q.x, _x.delTau_q.y, _x.delTau_q.z, _x.moments.x, _x.moments.y, _x.moments.z, _x.thrust, _x.hatM_1, _x.hatM_2, _x.hatM_3,) = _get_struct_136d().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_136d = None
def _get_struct_136d():
    global _struct_136d
    if _struct_136d is None:
        _struct_136d = struct.Struct("<136d")
    return _struct_136d
_struct_3I = None
def _get_struct_3I():
    global _struct_3I
    if _struct_3I is None:
        _struct_3I = struct.Struct("<3I")
    return _struct_3I
