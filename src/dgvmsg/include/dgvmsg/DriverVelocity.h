// Generated by gencpp from file dgvmsg/DriverVelocity.msg
// DO NOT EDIT!


#ifndef DGVMSG_MESSAGE_DRIVERVELOCITY_H
#define DGVMSG_MESSAGE_DRIVERVELOCITY_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <dgvmsg/DriverNode.h>

namespace dgvmsg
{
template <class ContainerAllocator>
struct DriverVelocity_
{
  typedef DriverVelocity_<ContainerAllocator> Type;

  DriverVelocity_()
    : driver()  {
    }
  DriverVelocity_(const ContainerAllocator& _alloc)
    : driver(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector< ::dgvmsg::DriverNode_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::dgvmsg::DriverNode_<ContainerAllocator> >::other >  _driver_type;
  _driver_type driver;




  typedef boost::shared_ptr< ::dgvmsg::DriverVelocity_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::dgvmsg::DriverVelocity_<ContainerAllocator> const> ConstPtr;

}; // struct DriverVelocity_

typedef ::dgvmsg::DriverVelocity_<std::allocator<void> > DriverVelocity;

typedef boost::shared_ptr< ::dgvmsg::DriverVelocity > DriverVelocityPtr;
typedef boost::shared_ptr< ::dgvmsg::DriverVelocity const> DriverVelocityConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::dgvmsg::DriverVelocity_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::dgvmsg::DriverVelocity_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace dgvmsg

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'dgvmsg': ['/home/pc-robot/workspace/ubuntudev/03-controler/02-controller_232/src/dgvmsg/msg'], 'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/indigo/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::dgvmsg::DriverVelocity_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dgvmsg::DriverVelocity_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dgvmsg::DriverVelocity_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dgvmsg::DriverVelocity_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dgvmsg::DriverVelocity_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dgvmsg::DriverVelocity_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::dgvmsg::DriverVelocity_<ContainerAllocator> >
{
  static const char* value()
  {
    return "97330beae4312907dc78de56d0603bd5";
  }

  static const char* value(const ::dgvmsg::DriverVelocity_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x97330beae4312907ULL;
  static const uint64_t static_value2 = 0xdc78de56d0603bd5ULL;
};

template<class ContainerAllocator>
struct DataType< ::dgvmsg::DriverVelocity_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dgvmsg/DriverVelocity";
  }

  static const char* value(const ::dgvmsg::DriverVelocity_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::dgvmsg::DriverVelocity_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# This expresses velocity in free space broken into its linear and angular parts.\n\
\n\
DriverNode[] driver\n\
================================================================================\n\
MSG: dgvmsg/DriverNode\n\
# This represents a vector in free space. \n\
# It is only meant to represent a direction. Therefore, it does not\n\
# make sense to apply a translation to it (e.g., when applying a \n\
# generic rigid transformation to a Vector3, tf2 will only apply the\n\
# rotation). If you want your data to be translatable too, use the\n\
\n\
string name\n\
int32 add\n\
int32 VRPM\n\
float64 Vspeed\n\
";
  }

  static const char* value(const ::dgvmsg::DriverVelocity_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::dgvmsg::DriverVelocity_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.driver);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct DriverVelocity_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::dgvmsg::DriverVelocity_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::dgvmsg::DriverVelocity_<ContainerAllocator>& v)
  {
    s << indent << "driver[]" << std::endl;
    for (size_t i = 0; i < v.driver.size(); ++i)
    {
      s << indent << "  driver[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::dgvmsg::DriverNode_<ContainerAllocator> >::stream(s, indent + "    ", v.driver[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // DGVMSG_MESSAGE_DRIVERVELOCITY_H