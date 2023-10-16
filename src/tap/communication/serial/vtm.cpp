/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of Taproot.
 *
 * Taproot is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Taproot is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Taproot.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "vtm.hpp"

#include "tap/algorithms/crc.hpp"
#include "tap/architecture/clock.hpp"
#include "tap/architecture/endianness_wrappers.hpp"
#include "tap/communication/serial/vtm_constants.hpp"
#include "tap/drivers.hpp"
#include "tap/errors/create_errors.hpp"

using namespace tap::arch;

namespace tap::communication::serial
{
VTM::VTM(Drivers* drivers)
    : DJISerial(drivers, bound_ports::VTM_UART_PORT, false),
      vtm(),
      transmissionSemaphore(1)
{
    VTMOfflineTimeout.stop();
}

bool VTM::getVTMReceivingData() const
{
    return !(VTMOfflineTimeout.isStopped() || VTMOfflineTimeout.isExpired());
}

//Todo: rename function
void VTM::messageReceiveCallback(const ReceivedSerialMessage& completeMessage)
{
    VTMOfflineTimeout.restart(10000);

    decodeVTMControl(completeMessage);
}

//decode ref serial messages containing keyboard control data
bool VTM::decodeVTMControl(const ReceivedSerialMessage& message)
{
    //parse incoming serial data
    if (message.header.dataLength != 12) return false;

    convertFromLittleEndian(&vtm.mouse.x, message.data);
    convertFromLittleEndian(&vtm.mouse.y, message.data + 2);
    convertFromLittleEndian(&vtm.mouse.wheel., message.data + 4);
    VTMControlData.mouseL = message.data[6];
    VTMControlData.mouseR = message.data[7];
    convertFromLittleEndian(&VTMControlData.keys, message.data + 8);

    //ensure that disabled state is only toggled on keyup, so it isn't continually changed while holding
    //Todo: fix this
    /*
    if (getKey(Rx::Key::X) && !VTMControlData.disableKeyPressed) {
        VTMControlData.disableKeyPressed = true;
    }
    if (!getKey(Rx::Key::X) && VTMControlData.disableKeyPressed) {
        VTMControlData.disableKeyPressed = false;
        VTMControlData.controlDisabled = !VTMControlData.controlDisabled;
    }
    */

    //update command scheduler key states
    drivers->commandMapper.handleKeyStateChange(vtm.keys,
                                                tap::communication::serial::Remote::SwitchState::UNKNOWN, 
                                                tap::communication::serial::Remote::SwitchState::UNKNOWN,
                                                vtm.mouse.l, vtm.mouse.r);

    return true;
}

//return true if the key (k) is currently pressed
bool VTM::getKey(Rx::Key k) {
    return static_cast<uint16_t>(k) & vtm.keys; 
}

//clears keyboard state and disables the robot
void VTM::resetKeys() {
    vtm.keys = 0;
    vtm.mouse.x = 0;
    vtm.mouse.y = 0;
    vtm.mouse.wheel = 0;
    vtm.mouse.left = 0;
    vtm.mouse.right = 0;
    drivers->commandMapper.handleKeyStateChange(0, tap::communication::serial::Remote::SwitchState::UNKNOWN,
        tap::communication::serial::Remote::SwitchState::UNKNOWN, false, false);
}
}  // namespace tap::communication::serial
