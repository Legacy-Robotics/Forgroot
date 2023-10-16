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

#include "dji_motor.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/drivers.hpp"

#ifdef PLATFORM_HOSTED
#include <iostream>

#include "tap/communication/tcp-server/json_messages.hpp"
#include "tap/communication/tcp-server/tcp_server.hpp"

#include "modm/architecture/interface/can_message.hpp"
#endif

namespace tap
{
namespace motor
{
DjiMotor::~DjiMotor() { drivers->djiMotorTxHandler.removeFromMotorManager(*this); }

DjiMotor::DjiMotor(
    Drivers* drivers,
    MotorId desMotorIdentifier,
    bool isInverted,
    const char* name,
    uint16_t encoderWrapped,
    int64_t encoderRevolutions,
    MotorType motorType)
    : CanRxListener(drivers, static_cast<uint32_t>(desMotorIdentifier), 
        (motorType == MotorType::GM6020) ? tap::can::CanBus::CAN_BUS2 : tap::can::CanBus::CAN_BUS1),
      motorName(name),
      motorType(motorType),
      drivers(drivers),
      motorIdentifier(desMotorIdentifier),
      motorCanBus((motorType == MotorType::GM6020) ? tap::can::CanBus::CAN_BUS2 : tap::can::CanBus::CAN_BUS1),
      desiredOutput(0),
      shaftRPM(0),
      temperature(0),
      torque(0),
      motorInverted(isInverted),
      encoderWrapped(encoderWrapped),
      encoderRevolutions(encoderRevolutions)
{
    motorDisconnectTimeout.stop();
}

void DjiMotor::initialize()
{
    drivers->djiMotorTxHandler.addMotorToManager(this);
    attachSelfToRxHandler();
}

void DjiMotor::processMessage(const modm::can::Message& message)
{
    if (message.getIdentifier() != DjiMotor::getMotorIdentifier())
    {
        return;
    }
    uint16_t encoderActual =
        static_cast<uint16_t>(message.data[0] << 8 | message.data[1]);        // encoder value
    shaftRPM = static_cast<int16_t>(message.data[2] << 8 | message.data[3]);  // rpm
    shaftRPM = motorInverted ? -shaftRPM : shaftRPM;
    torque = static_cast<int16_t>(message.data[4] << 8 | message.data[5]);  // torque
    torque = motorInverted ? -torque : torque;
    if (motorType == MotorType::GM6020)
    {
        temperature = static_cast<int8_t>(message.data[6]);  // temperature
    }
    //no temperature on M2006
    else
    {
        temperature = 0;
    }

    // restart disconnect timer, since you just received a message from the motor
    motorDisconnectTimeout.restart(MOTOR_DISCONNECT_TIME);

    // invert motor if necessary
    encoderActual = motorInverted ? ENC_RESOLUTION - 1 - encoderActual : encoderActual;
    updateEncoderValue(encoderActual);
}

void DjiMotor::setDesiredOutput(int32_t desiredOutput)
{
    int16_t desOutputNotInverted =
        static_cast<int16_t>(tap::algorithms::limitVal<int32_t>(desiredOutput, SHRT_MIN, SHRT_MAX));
    this->desiredOutput = motorInverted ? -desOutputNotInverted : desOutputNotInverted;
}

bool DjiMotor::isMotorOnline() const
{
    /*
     * motor online if the disconnect timout has not expired (if it received message but
     * somehow got disconnected) and the timeout hasn't been stopped (initially, the timeout
     * is stopped)
     */
    return !motorDisconnectTimeout.isExpired() && !motorDisconnectTimeout.isStopped();
}

void DjiMotor::serializeCanSendData(modm::can::Message* txMessage) const
{
    int id = this->getMotorIdentifierNum() - 1;  // number between 0 and 7
    // this method assumes you have choosen the correct message
    // to send the data in. Is blind to message type and is a private method
    // that I use accordingly.
    id %= 4;
    txMessage->data[2 * id] = this->getOutputDesired() >> 8;
    txMessage->data[2 * id + 1] = this->getOutputDesired() & 0xFF;
}

// getter functions
int16_t DjiMotor::getOutputDesired() const { return desiredOutput; }

uint32_t DjiMotor::getMotorIdentifier() const { 
    if (motorType == MotorType::GM6020)
    {
        return motorIdentifier + GM6020_BASE_ID;
    }
    //M2006 + others
    else 
    {
        return motorIdentifier + M2006_BASE_ID; 
    }
}

uint8_t DjiMotor::getMotorIdentifierNum() const
{
    return motorIdentifier;
}

uint8_t DjiMotor::getMaxMotorID() const
{
    if (motorType == MotorType::GM6020)
    {
        return GM6020_MAX_MOTORS;
    }
    //M2006 + others
    else 
    {
        return M2006_MAX_MOTORS;
    }
}

int8_t DjiMotor::getTemperature() const { return temperature; }

int16_t DjiMotor::getTorque() const { return torque; }

int16_t DjiMotor::getShaftRPM() const { return shaftRPM; }

bool DjiMotor::isMotorInverted() const { return motorInverted; }

tap::can::CanBus DjiMotor::getCanBus() const { return motorCanBus; }

const char* DjiMotor::getName() const { return motorName; }

int64_t DjiMotor::getEncoderUnwrapped() const
{
    return static_cast<int64_t>(encoderWrapped) +
           static_cast<int64_t>(ENC_RESOLUTION) * encoderRevolutions;
}

uint16_t DjiMotor::getEncoderWrapped() const { return encoderWrapped; }

void DjiMotor::updateEncoderValue(uint16_t newEncWrapped)
{
    int16_t enc_dif = newEncWrapped - encoderWrapped;
    if (enc_dif < -ENC_RESOLUTION / 2)
    {
        encoderRevolutions++;
    }
    else if (enc_dif > ENC_RESOLUTION / 2)
    {
        encoderRevolutions--;
    }
    encoderWrapped = newEncWrapped;
}
}  // namespace motor

}  // namespace tap
