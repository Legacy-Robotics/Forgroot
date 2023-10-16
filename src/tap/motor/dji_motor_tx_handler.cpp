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

#include "dji_motor_tx_handler.hpp"

#include <cassert>

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/drivers.hpp"
#include "tap/errors/create_errors.hpp"

#include "modm/architecture/interface/assert.h"
#include "modm/architecture/interface/can_message.hpp"

namespace tap::motor
{
void DjiMotorTxHandler::addMotorToManager(DjiMotor** canMotorStore, DjiMotor* const motor)
{
    assert(motor != nullptr);
    uint32_t idIndex = motor->getMotorIdentifierNum() - 1;
    bool motorOverloaded = canMotorStore[idIndex] != nullptr;
    bool motorOutOfBounds = idIndex >= motor->getMaxMotorID() - 1;
    modm_assert(!motorOverloaded && !motorOutOfBounds, "DjiMotorTxHandler", "overloading");
    canMotorStore[idIndex] = motor;
}

void DjiMotorTxHandler::addMotorToManager(DjiMotor* motor)
{
    // add new motor to either the can1 or can2 motor store
    // because we checked to see if the motor is overloaded, we will
    // never have to worry about overfilling the CanxMotorStore array
    if (motor->getCanBus() == tap::can::CanBus::CAN_BUS1)
    {
        addMotorToManager(can1MotorStore, motor);
    }
    else
    {
        addMotorToManager(can2MotorStore, motor);
    }
}

void DjiMotorTxHandler::encodeAndSendCanData()
{
    // set up new can messages to be sent via CAN bus 1 and 2
    modm::can::Message can1MessageLow(
        M2006_CAN_DJI_LOW_IDENTIFIER,
        CAN_DJI_MESSAGE_SEND_LENGTH,
        0,
        false);
    modm::can::Message can1MessageHigh(
        M2006_CAN_DJI_HIGH_IDENTIFIER,
        CAN_DJI_MESSAGE_SEND_LENGTH,
        0,
        false);
    //GM6020 on CAN2 Alone
    modm::can::Message can2MessageLow(
        GM6020_CAN_DJI_LOW_IDENTIFIER,
        CAN_DJI_MESSAGE_SEND_LENGTH,
        0,
        false);
    modm::can::Message can2MessageHigh(
        GM6020_CAN_DJI_HIGH_IDENTIFIER,
        CAN_DJI_MESSAGE_SEND_LENGTH,
        0,
        false);

    bool can1ValidMotorMessageLow = false;
    bool can1ValidMotorMessageHigh = false;
    bool can2ValidMotorMessageLow = false;
    bool can2ValidMotorMessageHigh = false;

    serializeMotorStoreSendData(
        can1MotorStore,
        &can1MessageLow,
        &can1MessageHigh,
        &can1ValidMotorMessageLow,
        &can1ValidMotorMessageHigh,
        MotorType::M2006);

    serializeMotorStoreSendData(
        can2MotorStore,
        &can2MessageLow,
        &can2MessageHigh,
        &can2ValidMotorMessageLow,
        &can2ValidMotorMessageHigh,
        MotorType::GM6020);

    bool messageSuccess = true;

    if (drivers->can.isReadyToSend(can::CanBus::CAN_BUS1))
    {
        if (can1ValidMotorMessageLow)
        {
            messageSuccess &= drivers->can.sendMessage(can::CanBus::CAN_BUS1, can1MessageLow);
        }
        if (can1ValidMotorMessageHigh)
        {
            messageSuccess &= drivers->can.sendMessage(can::CanBus::CAN_BUS1, can1MessageHigh);
        }
    }
    if (drivers->can.isReadyToSend(can::CanBus::CAN_BUS2))
    {
        if (can2ValidMotorMessageLow)
        {
            messageSuccess &= drivers->can.sendMessage(can::CanBus::CAN_BUS2, can2MessageLow);
        }
        if (can2ValidMotorMessageHigh)
        {
            messageSuccess &= drivers->can.sendMessage(can::CanBus::CAN_BUS2, can2MessageHigh);
        }
    }

    if (!messageSuccess)
    {
        RAISE_ERROR(drivers, "sendMessage failure");
    }
}

void DjiMotorTxHandler::serializeMotorStoreSendData(
    DjiMotor** canMotorStore,
    modm::can::Message* messageLow,
    modm::can::Message* messageHigh,
    bool* validMotorMessageLow,
    bool* validMotorMessageHigh,
    MotorType motorType
    )
{
    uint8_t maxMotorID = (motorType == MotorType::GM6020) ? GM6020_MAX_MOTORS : M2006_MAX_MOTORS;
    maxMotorID--; //decrememnt because we are zero indexing
    for (uint8_t i = 0; i < maxMotorID; i++)
    {
        const DjiMotor* const motor = canMotorStore[i];
        if (motor != nullptr)
        {

            if (motor->getMotorIdentifierNum() <= 4)
            {
                motor->serializeCanSendData(messageLow);
                *validMotorMessageLow = true;
            }
            else
            {
                motor->serializeCanSendData(messageHigh);
                *validMotorMessageHigh = true;
            }
        }
    }
}

void DjiMotorTxHandler::removeFromMotorManager(const DjiMotor& motor)
{
    if (motor.getCanBus() == tap::can::CanBus::CAN_BUS1)
    {
        removeFromMotorManager(motor, can1MotorStore);
    }
    else
    {
        removeFromMotorManager(motor, can2MotorStore);
    }
}

void DjiMotorTxHandler::removeFromMotorManager(const DjiMotor& motor, DjiMotor** motorStore)
{
    //want to index at 0, so motor ID is offset by 1
    uint32_t id = motor.getMotorIdentifierNum() - 1;
    if (id > (motor.getMaxMotorID() - 1) || motorStore[id] == nullptr)
    {
        RAISE_ERROR(drivers, "invalid motor id");
        return;
    }
    motorStore[id] = nullptr;
}

DjiMotor const* DjiMotorTxHandler::getCan1Motor(MotorId motorId)
{
    uint32_t index = motorId - 1;
    return index > M2006_MAX_MOTORS ? nullptr : can1MotorStore[index];
}

DjiMotor const* DjiMotorTxHandler::getCan2Motor(MotorId motorId)
{
    uint32_t index = motorId - 1;
    return index > GM6020_MAX_MOTORS ? nullptr : can2MotorStore[index];
}
}  // namespace tap::motor
