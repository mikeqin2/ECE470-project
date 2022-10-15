// Generated by gencpp from file ur3_driver/command.msg
// DO NOT EDIT!


#ifndef UR3_DRIVER_MESSAGE_COMMAND_H
#define UR3_DRIVER_MESSAGE_COMMAND_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace ur3_driver
{
template <class ContainerAllocator>
struct command_
{
  typedef command_<ContainerAllocator> Type;

  command_()
    : destination()
    , v(0.0)
    , a(0.0)
    , io_0(false)  {
    }
  command_(const ContainerAllocator& _alloc)
    : destination(_alloc)
    , v(0.0)
    , a(0.0)
    , io_0(false)  {
  (void)_alloc;
    }



   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _destination_type;
  _destination_type destination;

   typedef double _v_type;
  _v_type v;

   typedef double _a_type;
  _a_type a;

   typedef uint8_t _io_0_type;
  _io_0_type io_0;





  typedef boost::shared_ptr< ::ur3_driver::command_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ur3_driver::command_<ContainerAllocator> const> ConstPtr;

}; // struct command_

typedef ::ur3_driver::command_<std::allocator<void> > command;

typedef boost::shared_ptr< ::ur3_driver::command > commandPtr;
typedef boost::shared_ptr< ::ur3_driver::command const> commandConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ur3_driver::command_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ur3_driver::command_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace ur3_driver

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'ur3_driver': ['/home/ur3/project/src/drivers/ur3_driver/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::ur3_driver::command_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ur3_driver::command_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ur3_driver::command_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ur3_driver::command_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ur3_driver::command_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ur3_driver::command_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ur3_driver::command_<ContainerAllocator> >
{
  static const char* value()
  {
    return "c8890f8a9e9a6dc32b97081f6283bc11";
  }

  static const char* value(const ::ur3_driver::command_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xc8890f8a9e9a6dc3ULL;
  static const uint64_t static_value2 = 0x2b97081f6283bc11ULL;
};

template<class ContainerAllocator>
struct DataType< ::ur3_driver::command_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ur3_driver/command";
  }

  static const char* value(const ::ur3_driver::command_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ur3_driver::command_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64[] destination\n\
float64 v\n\
float64 a\n\
bool io_0\n\
";
  }

  static const char* value(const ::ur3_driver::command_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ur3_driver::command_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.destination);
      stream.next(m.v);
      stream.next(m.a);
      stream.next(m.io_0);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct command_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ur3_driver::command_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ur3_driver::command_<ContainerAllocator>& v)
  {
    s << indent << "destination[]" << std::endl;
    for (size_t i = 0; i < v.destination.size(); ++i)
    {
      s << indent << "  destination[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.destination[i]);
    }
    s << indent << "v: ";
    Printer<double>::stream(s, indent + "  ", v.v);
    s << indent << "a: ";
    Printer<double>::stream(s, indent + "  ", v.a);
    s << indent << "io_0: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.io_0);
  }
};

} // namespace message_operations
} // namespace ros

#endif // UR3_DRIVER_MESSAGE_COMMAND_H
