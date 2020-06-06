#include "aruwlib/Drivers.hpp"

#include "can_rx_listener.hpp"

namespace aruwlib
{
namespace can
{
CanRxListner::~CanRxListner() { Drivers::canRxHandler.removeReceiveHandler(*this); }

CanRxListner::CanRxListner(uint32_t id, CanBus cB) : canIdentifier(id), canBus(cB)
{
    Drivers::canRxHandler.attachReceiveHandler(this);
}
}  // namespace can

}  // namespace aruwlib
