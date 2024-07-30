// Generated by gencpp from file test_rospy/EmptyReqSrvResponse.msg
// DO NOT EDIT!


#ifndef TEST_ROSPY_MESSAGE_EMPTYREQSRVRESPONSE_H
#define TEST_ROSPY_MESSAGE_EMPTYREQSRVRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace test_rospy
{
template <class ContainerAllocator>
struct EmptyReqSrvResponse_
{
  typedef EmptyReqSrvResponse_<ContainerAllocator> Type;

  EmptyReqSrvResponse_()
    : fake_secret(0)  {
    }
  EmptyReqSrvResponse_(const ContainerAllocator& _alloc)
    : fake_secret(0)  {
  (void)_alloc;
    }



   typedef int32_t _fake_secret_type;
  _fake_secret_type fake_secret;





  typedef boost::shared_ptr< ::test_rospy::EmptyReqSrvResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::test_rospy::EmptyReqSrvResponse_<ContainerAllocator> const> ConstPtr;

}; // struct EmptyReqSrvResponse_

typedef ::test_rospy::EmptyReqSrvResponse_<std::allocator<void> > EmptyReqSrvResponse;

typedef boost::shared_ptr< ::test_rospy::EmptyReqSrvResponse > EmptyReqSrvResponsePtr;
typedef boost::shared_ptr< ::test_rospy::EmptyReqSrvResponse const> EmptyReqSrvResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::test_rospy::EmptyReqSrvResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::test_rospy::EmptyReqSrvResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::test_rospy::EmptyReqSrvResponse_<ContainerAllocator1> & lhs, const ::test_rospy::EmptyReqSrvResponse_<ContainerAllocator2> & rhs)
{
  return lhs.fake_secret == rhs.fake_secret;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::test_rospy::EmptyReqSrvResponse_<ContainerAllocator1> & lhs, const ::test_rospy::EmptyReqSrvResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace test_rospy

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::test_rospy::EmptyReqSrvResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::test_rospy::EmptyReqSrvResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::test_rospy::EmptyReqSrvResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::test_rospy::EmptyReqSrvResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::test_rospy::EmptyReqSrvResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::test_rospy::EmptyReqSrvResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::test_rospy::EmptyReqSrvResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "1280578cbe4073963a4321b9248d566e";
  }

  static const char* value(const ::test_rospy::EmptyReqSrvResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x1280578cbe407396ULL;
  static const uint64_t static_value2 = 0x3a4321b9248d566eULL;
};

template<class ContainerAllocator>
struct DataType< ::test_rospy::EmptyReqSrvResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "test_rospy/EmptyReqSrvResponse";
  }

  static const char* value(const ::test_rospy::EmptyReqSrvResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::test_rospy::EmptyReqSrvResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 fake_secret\n"
"\n"
;
  }

  static const char* value(const ::test_rospy::EmptyReqSrvResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::test_rospy::EmptyReqSrvResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.fake_secret);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct EmptyReqSrvResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::test_rospy::EmptyReqSrvResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::test_rospy::EmptyReqSrvResponse_<ContainerAllocator>& v)
  {
    s << indent << "fake_secret: ";
    Printer<int32_t>::stream(s, indent + "  ", v.fake_secret);
  }
};

} // namespace message_operations
} // namespace ros

#endif // TEST_ROSPY_MESSAGE_EMPTYREQSRVRESPONSE_H
