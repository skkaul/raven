/* Auto-generated by genmsg_cpp for file /home/avinash/ros/raven-read-only/roboteq/msg/config.msg */
#ifndef ROBOTEQ_MESSAGE_CONFIG_H
#define ROBOTEQ_MESSAGE_CONFIG_H
#include <string>
#include <vector>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/message.h"
#include "ros/time.h"

#include "roslib/Header.h"

namespace roboteq
{
template <class ContainerAllocator>
struct config_ : public ros::Message
{
  typedef config_<ContainerAllocator> Type;

  config_()
  : header()
  , MotorAmps1(0)
  , MotorAmps2(0)
  , BatteryAmps1(0)
  , BatteryAmps2(0)
  , motorpower1(0)
  , motorpower2(0)
  , drivervoltage(0)
  , BatteryVoltage(0)
  , DSUBvoltage(0)
  {
  }

  config_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , MotorAmps1(0)
  , MotorAmps2(0)
  , BatteryAmps1(0)
  , BatteryAmps2(0)
  , motorpower1(0)
  , motorpower2(0)
  , drivervoltage(0)
  , BatteryVoltage(0)
  , DSUBvoltage(0)
  {
  }

  typedef  ::roslib::Header_<ContainerAllocator>  _header_type;
   ::roslib::Header_<ContainerAllocator>  header;

  typedef int32_t _MotorAmps1_type;
  int32_t MotorAmps1;

  typedef int32_t _MotorAmps2_type;
  int32_t MotorAmps2;

  typedef int32_t _BatteryAmps1_type;
  int32_t BatteryAmps1;

  typedef int32_t _BatteryAmps2_type;
  int32_t BatteryAmps2;

  typedef int32_t _motorpower1_type;
  int32_t motorpower1;

  typedef int32_t _motorpower2_type;
  int32_t motorpower2;

  typedef int32_t _drivervoltage_type;
  int32_t drivervoltage;

  typedef int32_t _BatteryVoltage_type;
  int32_t BatteryVoltage;

  typedef int32_t _DSUBvoltage_type;
  int32_t DSUBvoltage;


private:
  static const char* __s_getDataType_() { return "roboteq/config"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROSCPP_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "8886c8836227fe18873a3b2abc061efe"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROSCPP_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "Header header\n\
int32 MotorAmps1\n\
int32 MotorAmps2\n\
int32 BatteryAmps1\n\
int32 BatteryAmps2\n\
int32 motorpower1\n\
int32 motorpower2\n\
int32 drivervoltage\n\
int32 BatteryVoltage\n\
int32 DSUBvoltage\n\
\n\
\n\
\n\
================================================================================\n\
MSG: roslib/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.secs: seconds (stamp_secs) since epoch\n\
# * stamp.nsecs: nanoseconds since stamp_secs\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROSCPP_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROSCPP_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, header);
    ros::serialization::serialize(stream, MotorAmps1);
    ros::serialization::serialize(stream, MotorAmps2);
    ros::serialization::serialize(stream, BatteryAmps1);
    ros::serialization::serialize(stream, BatteryAmps2);
    ros::serialization::serialize(stream, motorpower1);
    ros::serialization::serialize(stream, motorpower2);
    ros::serialization::serialize(stream, drivervoltage);
    ros::serialization::serialize(stream, BatteryVoltage);
    ros::serialization::serialize(stream, DSUBvoltage);
    return stream.getData();
  }

  ROSCPP_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, header);
    ros::serialization::deserialize(stream, MotorAmps1);
    ros::serialization::deserialize(stream, MotorAmps2);
    ros::serialization::deserialize(stream, BatteryAmps1);
    ros::serialization::deserialize(stream, BatteryAmps2);
    ros::serialization::deserialize(stream, motorpower1);
    ros::serialization::deserialize(stream, motorpower2);
    ros::serialization::deserialize(stream, drivervoltage);
    ros::serialization::deserialize(stream, BatteryVoltage);
    ros::serialization::deserialize(stream, DSUBvoltage);
    return stream.getData();
  }

  ROSCPP_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(header);
    size += ros::serialization::serializationLength(MotorAmps1);
    size += ros::serialization::serializationLength(MotorAmps2);
    size += ros::serialization::serializationLength(BatteryAmps1);
    size += ros::serialization::serializationLength(BatteryAmps2);
    size += ros::serialization::serializationLength(motorpower1);
    size += ros::serialization::serializationLength(motorpower2);
    size += ros::serialization::serializationLength(drivervoltage);
    size += ros::serialization::serializationLength(BatteryVoltage);
    size += ros::serialization::serializationLength(DSUBvoltage);
    return size;
  }

  typedef boost::shared_ptr< ::roboteq::config_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::roboteq::config_<ContainerAllocator>  const> ConstPtr;
}; // struct config
typedef  ::roboteq::config_<std::allocator<void> > config;

typedef boost::shared_ptr< ::roboteq::config> configPtr;
typedef boost::shared_ptr< ::roboteq::config const> configConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::roboteq::config_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::roboteq::config_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace roboteq

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator>
struct MD5Sum< ::roboteq::config_<ContainerAllocator> > {
  static const char* value() 
  {
    return "8886c8836227fe18873a3b2abc061efe";
  }

  static const char* value(const  ::roboteq::config_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x8886c8836227fe18ULL;
  static const uint64_t static_value2 = 0x873a3b2abc061efeULL;
};

template<class ContainerAllocator>
struct DataType< ::roboteq::config_<ContainerAllocator> > {
  static const char* value() 
  {
    return "roboteq/config";
  }

  static const char* value(const  ::roboteq::config_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::roboteq::config_<ContainerAllocator> > {
  static const char* value() 
  {
    return "Header header\n\
int32 MotorAmps1\n\
int32 MotorAmps2\n\
int32 BatteryAmps1\n\
int32 BatteryAmps2\n\
int32 motorpower1\n\
int32 motorpower2\n\
int32 drivervoltage\n\
int32 BatteryVoltage\n\
int32 DSUBvoltage\n\
\n\
\n\
\n\
================================================================================\n\
MSG: roslib/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.secs: seconds (stamp_secs) since epoch\n\
# * stamp.nsecs: nanoseconds since stamp_secs\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
";
  }

  static const char* value(const  ::roboteq::config_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::roboteq::config_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::roboteq::config_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.MotorAmps1);
    stream.next(m.MotorAmps2);
    stream.next(m.BatteryAmps1);
    stream.next(m.BatteryAmps2);
    stream.next(m.motorpower1);
    stream.next(m.motorpower2);
    stream.next(m.drivervoltage);
    stream.next(m.BatteryVoltage);
    stream.next(m.DSUBvoltage);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct config_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::roboteq::config_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::roboteq::config_<ContainerAllocator> & v) 
  {
    s << indent << "header: ";
s << std::endl;
    Printer< ::roslib::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "MotorAmps1: ";
    Printer<int32_t>::stream(s, indent + "  ", v.MotorAmps1);
    s << indent << "MotorAmps2: ";
    Printer<int32_t>::stream(s, indent + "  ", v.MotorAmps2);
    s << indent << "BatteryAmps1: ";
    Printer<int32_t>::stream(s, indent + "  ", v.BatteryAmps1);
    s << indent << "BatteryAmps2: ";
    Printer<int32_t>::stream(s, indent + "  ", v.BatteryAmps2);
    s << indent << "motorpower1: ";
    Printer<int32_t>::stream(s, indent + "  ", v.motorpower1);
    s << indent << "motorpower2: ";
    Printer<int32_t>::stream(s, indent + "  ", v.motorpower2);
    s << indent << "drivervoltage: ";
    Printer<int32_t>::stream(s, indent + "  ", v.drivervoltage);
    s << indent << "BatteryVoltage: ";
    Printer<int32_t>::stream(s, indent + "  ", v.BatteryVoltage);
    s << indent << "DSUBvoltage: ";
    Printer<int32_t>::stream(s, indent + "  ", v.DSUBvoltage);
  }
};


} // namespace message_operations
} // namespace ros

#endif // ROBOTEQ_MESSAGE_CONFIG_H

