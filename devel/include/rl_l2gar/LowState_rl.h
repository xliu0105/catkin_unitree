// Generated by gencpp from file rl_l2gar/LowState_rl.msg
// DO NOT EDIT!


#ifndef RL_L2GAR_MESSAGE_LOWSTATE_RL_H
#define RL_L2GAR_MESSAGE_LOWSTATE_RL_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <unitree_legged_msgs/IMU.h>
#include <unitree_legged_msgs/MotorState.h>
#include <rl_l2gar/userValue_msg.h>

namespace rl_l2gar
{
template <class ContainerAllocator>
struct LowState_rl_
{
  typedef LowState_rl_<ContainerAllocator> Type;

  LowState_rl_()
    : imu()
    , motorState()
    , userValue()
    , userCmd(0)  {
    }
  LowState_rl_(const ContainerAllocator& _alloc)
    : imu(_alloc)
    , motorState()
    , userValue(_alloc)
    , userCmd(0)  {
  (void)_alloc;
      motorState.assign( ::unitree_legged_msgs::MotorState_<ContainerAllocator> (_alloc));
  }



   typedef  ::unitree_legged_msgs::IMU_<ContainerAllocator>  _imu_type;
  _imu_type imu;

   typedef boost::array< ::unitree_legged_msgs::MotorState_<ContainerAllocator> , 20>  _motorState_type;
  _motorState_type motorState;

   typedef  ::rl_l2gar::userValue_msg_<ContainerAllocator>  _userValue_type;
  _userValue_type userValue;

   typedef int32_t _userCmd_type;
  _userCmd_type userCmd;





  typedef boost::shared_ptr< ::rl_l2gar::LowState_rl_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::rl_l2gar::LowState_rl_<ContainerAllocator> const> ConstPtr;

}; // struct LowState_rl_

typedef ::rl_l2gar::LowState_rl_<std::allocator<void> > LowState_rl;

typedef boost::shared_ptr< ::rl_l2gar::LowState_rl > LowState_rlPtr;
typedef boost::shared_ptr< ::rl_l2gar::LowState_rl const> LowState_rlConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::rl_l2gar::LowState_rl_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::rl_l2gar::LowState_rl_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::rl_l2gar::LowState_rl_<ContainerAllocator1> & lhs, const ::rl_l2gar::LowState_rl_<ContainerAllocator2> & rhs)
{
  return lhs.imu == rhs.imu &&
    lhs.motorState == rhs.motorState &&
    lhs.userValue == rhs.userValue &&
    lhs.userCmd == rhs.userCmd;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::rl_l2gar::LowState_rl_<ContainerAllocator1> & lhs, const ::rl_l2gar::LowState_rl_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace rl_l2gar

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::rl_l2gar::LowState_rl_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rl_l2gar::LowState_rl_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::rl_l2gar::LowState_rl_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::rl_l2gar::LowState_rl_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rl_l2gar::LowState_rl_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rl_l2gar::LowState_rl_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::rl_l2gar::LowState_rl_<ContainerAllocator> >
{
  static const char* value()
  {
    return "b77f6762640229fefac4685d9f05c4e0";
  }

  static const char* value(const ::rl_l2gar::LowState_rl_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xb77f6762640229feULL;
  static const uint64_t static_value2 = 0xfac4685d9f05c4e0ULL;
};

template<class ContainerAllocator>
struct DataType< ::rl_l2gar::LowState_rl_<ContainerAllocator> >
{
  static const char* value()
  {
    return "rl_l2gar/LowState_rl";
  }

  static const char* value(const ::rl_l2gar::LowState_rl_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::rl_l2gar::LowState_rl_<ContainerAllocator> >
{
  static const char* value()
  {
    return "unitree_legged_msgs/IMU imu\n"
"unitree_legged_msgs/MotorState[20] motorState\n"
"userValue_msg userValue\n"
"int32 userCmd\n"
"================================================================================\n"
"MSG: unitree_legged_msgs/IMU\n"
"float32[4] quaternion\n"
"float32[3] gyroscope\n"
"float32[3] accelerometer\n"
"int8 temperature\n"
"================================================================================\n"
"MSG: unitree_legged_msgs/MotorState\n"
"uint8 mode           # motor current mode \n"
"float32 q            # motor current position（rad）\n"
"float32 dq           # motor current speed（rad/s）\n"
"float32 ddq          # motor current speed（rad/s）\n"
"float32 tauEst       # current estimated output torque（N*m）\n"
"float32 q_raw        # motor current position（rad）\n"
"float32 dq_raw       # motor current speed（rad/s）\n"
"float32 ddq_raw      # motor current speed（rad/s）\n"
"int8 temperature     # motor temperature（slow conduction of temperature leads to lag）\n"
"uint32[2] reserve\n"
"================================================================================\n"
"MSG: rl_l2gar/userValue_msg\n"
"float64 lx\n"
"float64 ly\n"
"float64 rx\n"
"float64 ry\n"
"float64 L2\n"
;
  }

  static const char* value(const ::rl_l2gar::LowState_rl_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::rl_l2gar::LowState_rl_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.imu);
      stream.next(m.motorState);
      stream.next(m.userValue);
      stream.next(m.userCmd);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct LowState_rl_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::rl_l2gar::LowState_rl_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::rl_l2gar::LowState_rl_<ContainerAllocator>& v)
  {
    s << indent << "imu: ";
    s << std::endl;
    Printer< ::unitree_legged_msgs::IMU_<ContainerAllocator> >::stream(s, indent + "  ", v.imu);
    s << indent << "motorState[]" << std::endl;
    for (size_t i = 0; i < v.motorState.size(); ++i)
    {
      s << indent << "  motorState[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::unitree_legged_msgs::MotorState_<ContainerAllocator> >::stream(s, indent + "    ", v.motorState[i]);
    }
    s << indent << "userValue: ";
    s << std::endl;
    Printer< ::rl_l2gar::userValue_msg_<ContainerAllocator> >::stream(s, indent + "  ", v.userValue);
    s << indent << "userCmd: ";
    Printer<int32_t>::stream(s, indent + "  ", v.userCmd);
  }
};

} // namespace message_operations
} // namespace ros

#endif // RL_L2GAR_MESSAGE_LOWSTATE_RL_H