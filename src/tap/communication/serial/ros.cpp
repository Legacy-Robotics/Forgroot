#include "ros.hpp"

#include "tap/drivers.hpp"

namespace tap::communication::serial
{
Ros::Ros(Drivers* drivers) : drivers(drivers)
{

}

Ros::init()
{
    drivers->uart.init<Uart::UartPort::Uart7, 115200>();
}

Ros::update()
{

}


}  // namespace tap::communication::serial
