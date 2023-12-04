/* Copyright Legacy Robotics 2023*/
#ifndef TAPROOT_ROS_HPP_
#define TAPROOT_ROS_HPP_

#include "tap/util_macros.hpp"

#include <geometry_msgs/Twist.h>
#include <ros/geometry_msgs/Twist.h>
#include <modm/communication/ros.hpp>

namespace tap
{
class Drivers;
}

namespace tap::communication::serial
{
using modmHardware = ModmHardware<modm::platform::Uart7>;
using ModmNodeHandle = NodeHandle_<modmHardware>;
/**
 * A class designed to handle communication between the STM32 and ROS master on the Jetson Nano
 */
class Ros  
{
public:
    Ros(Drivers* drivers);
    DISALLOW_COPY_AND_ASSIGN(Ros)
    mockable ~Ros() = default;

private:
    geometry_msgs::Twist encoder_msg;
    void twist_callback(const geometry_msgs::Twist &msg);
    ModmNodeHandle nh;
    Drivers *drivers;
    arch::MilliTimeout ROSTimeout;
};

}  // namespace tap::communication::serial

#endif  // TAPROOT_REF_SERIAL_HPP_
