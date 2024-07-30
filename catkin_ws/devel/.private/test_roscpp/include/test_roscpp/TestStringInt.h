// Generated by gencpp from file test_roscpp/TestStringInt.msg
// DO NOT EDIT!


#ifndef TEST_ROSCPP_MESSAGE_TESTSTRINGINT_H
#define TEST_ROSCPP_MESSAGE_TESTSTRINGINT_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace test_roscpp
{
template <class ContainerAllocator>
struct TestStringInt_
{
  typedef TestStringInt_<ContainerAllocator> Type;

  TestStringInt_()
    : str()
    , counter(0)  {
    }
  TestStringInt_(const ContainerAllocator& _alloc)
    : str(_alloc)
    , counter(0)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _str_type;
  _str_type str;

   typedef int32_t _counter_type;
  _counter_type counter;





  typedef boost::shared_ptr< ::test_roscpp::TestStringInt_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::test_roscpp::TestStringInt_<ContainerAllocator> const> ConstPtr;

}; // struct TestStringInt_

typedef ::test_roscpp::TestStringInt_<std::allocator<void> > TestStringInt;

typedef boost::shared_ptr< ::test_roscpp::TestStringInt > TestStringIntPtr;
typedef boost::shared_ptr< ::test_roscpp::TestStringInt const> TestStringIntConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::test_roscpp::TestStringInt_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::test_roscpp::TestStringInt_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::test_roscpp::TestStringInt_<ContainerAllocator1> & lhs, const ::test_roscpp::TestStringInt_<ContainerAllocator2> & rhs)
{
  return lhs.str == rhs.str &&
    lhs.counter == rhs.counter;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::test_roscpp::TestStringInt_<ContainerAllocator1> & lhs, const ::test_roscpp::TestStringInt_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace test_roscpp

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::test_roscpp::TestStringInt_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::test_roscpp::TestStringInt_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::test_roscpp::TestStringInt_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::test_roscpp::TestStringInt_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::test_roscpp::TestStringInt_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::test_roscpp::TestStringInt_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::test_roscpp::TestStringInt_<ContainerAllocator> >
{
  static const char* value()
  {
    return "2f0ceb8aa4bbf4dbd240039d0bf240ca";
  }

  static const char* value(const ::test_roscpp::TestStringInt_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x2f0ceb8aa4bbf4dbULL;
  static const uint64_t static_value2 = 0xd240039d0bf240caULL;
};

template<class ContainerAllocator>
struct DataType< ::test_roscpp::TestStringInt_<ContainerAllocator> >
{
  static const char* value()
  {
    return "test_roscpp/TestStringInt";
  }

  static const char* value(const ::test_roscpp::TestStringInt_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::test_roscpp::TestStringInt_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string str\n"
"int32 counter\n"
;
  }

  static const char* value(const ::test_roscpp::TestStringInt_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::test_roscpp::TestStringInt_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.str);
      stream.next(m.counter);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct TestStringInt_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::test_roscpp::TestStringInt_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::test_roscpp::TestStringInt_<ContainerAllocator>& v)
  {
    s << indent << "str: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.str);
    s << indent << "counter: ";
    Printer<int32_t>::stream(s, indent + "  ", v.counter);
  }
};

} // namespace message_operations
} // namespace ros

#endif // TEST_ROSCPP_MESSAGE_TESTSTRINGINT_H
