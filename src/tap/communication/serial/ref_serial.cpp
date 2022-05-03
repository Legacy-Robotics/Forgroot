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

#include "ref_serial.hpp"

#include "tap/algorithms/crc.hpp"
#include "tap/architecture/clock.hpp"
#include "tap/architecture/endianness_wrappers.hpp"
#include "tap/communication/serial/ref_serial_constants.hpp"
#include "tap/drivers.hpp"
#include "tap/errors/create_errors.hpp"

using namespace tap::arch;

namespace tap::communication::serial
{
RefSerial::RefSerial(Drivers* drivers)
    : DJISerial(drivers, bound_ports::REF_SERIAL_UART_PORT),
      robotData(),
      gameData(),
      receivedDpsTracker(),
      transmissionSemaphore(1)
{
    refSerialOfflineTimeout.stop();
}

bool RefSerial::getRefSerialReceivingData() const
{
    return !(refSerialOfflineTimeout.isStopped() || refSerialOfflineTimeout.isExpired());
}

void RefSerial::messageReceiveCallback(const ReceivedSerialMessage& completeMessage)
{
    refSerialOfflineTimeout.restart(TIME_OFFLINE_REF_DATA_MS);

    updateReceivedDamage();
    switch (completeMessage.messageType)
    {
        case REF_MESSAGE_TYPE_GAME_STATUS:
        {
            decodeToGameStatus(completeMessage);
            break;
        }
        case REF_MESSAGE_TYPE_GAME_RESULT:
        {
            decodeToGameResult(completeMessage);
            break;
        }
        case REF_MESSAGE_TYPE_ALL_ROBOT_HP:
        {
            decodeToAllRobotHP(completeMessage);
            break;
        }
        case REF_MESSAGE_TYPE_WARNING_DATA:
        {
            decodeToWarningData(completeMessage);
            break;
        }
        case REF_MESSAGE_TYPE_ROBOT_STATUS:
        {
            decodeToRobotStatus(completeMessage);
            break;
        }
        case REF_MESSAGE_TYPE_POWER_AND_HEAT:
        {
            decodeToPowerAndHeat(completeMessage);
            break;
        }
        case REF_MESSAGE_TYPE_ROBOT_POSITION:
        {
            decodeToRobotPosition(completeMessage);
            break;
        }
        case REF_MESSAGE_TYPE_ROBOT_BUFF_STATUS:
        {
            decodeToRobotBuffs(completeMessage);
            break;
        }
        case REF_MESSAGE_TYPE_AERIAL_ENERGY_STATUS:
        {
            decodeToAerialEnergyStatus(completeMessage);
            break;
        }
        case REF_MESSAGE_TYPE_RECEIVE_DAMAGE:
        {
            decodeToDamageStatus(completeMessage);
            break;
        }
        case REF_MESSAGE_TYPE_PROJECTILE_LAUNCH:
        {
            decodeToProjectileLaunch(completeMessage);
            break;
        }
        case REF_MESSAGE_TYPE_BULLETS_REMAIN:
        {
            decodeToBulletsRemain(completeMessage);
            break;
        }
        case REF_MESSAGE_TYPE_RFID_STATUS:
        {
            decodeToRFIDStatus(completeMessage);
            break;
        }
        case REF_MESSAGE_TYPE_CUSTOM_DATA:
        {
            handleRobotToRobotCommunication(completeMessage);
            break;
        }
        default:
            break;
    }
}

const RefSerialData::Rx::RobotData& RefSerial::getRobotData() const { return robotData; }

const RefSerialData::Rx::GameData& RefSerial::getGameData() const { return gameData; }

bool RefSerial::decodeToGameStatus(const ReceivedSerialMessage& message)
{
    if (message.header.dataLength != 11)
    {
        return false;
    }
    gameData.gameType = static_cast<Rx::GameType>(0xf & message.data[0]);
    gameData.gameStage = static_cast<Rx::GameStage>(0xf & (message.data[0] >> 4));
    convertFromLittleEndian(&gameData.stageTimeRemaining, message.data + 1);
    // reinterpreting as a uint64_t doesn't work, so do this instead
    gameData.unixTime = static_cast<uint64_t>(message.data[10]) << 56 |
                        static_cast<uint64_t>(message.data[9]) << 48 |
                        static_cast<uint64_t>(message.data[8]) << 40 |
                        static_cast<uint64_t>(message.data[7]) << 32 |
                        static_cast<uint64_t>(message.data[6]) << 24 |
                        static_cast<uint64_t>(message.data[5]) << 16 |
                        static_cast<uint64_t>(message.data[4]) << 8 |
                        static_cast<uint64_t>(message.data[3]);
    return true;
}

bool RefSerial::decodeToGameResult(const ReceivedSerialMessage& message)
{
    if (message.header.dataLength != 1)
    {
        return false;
    }
    gameData.gameWinner = static_cast<Rx::GameWinner>(message.data[0]);
    return true;
}

bool RefSerial::decodeToAllRobotHP(const ReceivedSerialMessage& message)
{
    if (message.header.dataLength != 32)
    {
        return false;
    }
    convertFromLittleEndian(&robotData.allRobotHp.red.hero1, message.data);
    convertFromLittleEndian(&robotData.allRobotHp.red.engineer2, message.data + 2);
    convertFromLittleEndian(&robotData.allRobotHp.red.standard3, message.data + 4);
    convertFromLittleEndian(&robotData.allRobotHp.red.standard4, message.data + 6);
    convertFromLittleEndian(&robotData.allRobotHp.red.standard5, message.data + 8);
    convertFromLittleEndian(&robotData.allRobotHp.red.sentry7, message.data + 10);
    convertFromLittleEndian(&robotData.allRobotHp.red.outpost, message.data + 12);
    convertFromLittleEndian(&robotData.allRobotHp.red.base, message.data + 14);

    convertFromLittleEndian(&robotData.allRobotHp.blue.hero1, message.data + 16);
    convertFromLittleEndian(&robotData.allRobotHp.blue.engineer2, message.data + 18);
    convertFromLittleEndian(&robotData.allRobotHp.blue.standard3, message.data + 20);
    convertFromLittleEndian(&robotData.allRobotHp.blue.standard4, message.data + 22);
    convertFromLittleEndian(&robotData.allRobotHp.blue.standard5, message.data + 24);
    convertFromLittleEndian(&robotData.allRobotHp.blue.sentry7, message.data + 26);
    convertFromLittleEndian(&robotData.allRobotHp.blue.outpost, message.data + 28);
    convertFromLittleEndian(&robotData.allRobotHp.blue.base, message.data + 30);

    return true;
}

bool RefSerial::decodeToSiteEventData(const ReceivedSerialMessage&) { return false; }

bool RefSerial::decodeToWarningData(const ReceivedSerialMessage& message)
{
    if (message.header.dataLength != 2)
    {
        return false;
    }
    robotData.refereeWarningData.level = message.data[0];
    robotData.refereeWarningData.foulRobotID = static_cast<RobotId>(message.data[1]);
    robotData.refereeWarningData.lastReceivedWarningRobotTime = clock::getTimeMilliseconds();
    return true;
}

bool RefSerial::decodeToRobotStatus(const ReceivedSerialMessage& message)
{
    if (message.header.dataLength != 27)
    {
        return false;
    }
    robotData.robotId = static_cast<RobotId>(message.data[0]);
    robotData.robotLevel = message.data[1];
    convertFromLittleEndian(&robotData.currentHp, message.data + 2);
    convertFromLittleEndian(&robotData.maxHp, message.data + 4);
    convertFromLittleEndian(&robotData.turret.heatCoolingRate17ID1, message.data + 6);
    convertFromLittleEndian(&robotData.turret.heatLimit17ID1, message.data + 8);
    convertFromLittleEndian(&robotData.turret.barrelSpeedLimit17ID1, message.data + 10);
    convertFromLittleEndian(&robotData.turret.heatCoolingRate17ID2, message.data + 12);
    convertFromLittleEndian(&robotData.turret.heatLimit17ID2, message.data + 14);
    convertFromLittleEndian(&robotData.turret.barrelSpeedLimit17ID2, message.data + 16);
    convertFromLittleEndian(&robotData.turret.heatCoolingRate42, message.data + 18);
    convertFromLittleEndian(&robotData.turret.heatLimit42, message.data + 20);
    convertFromLittleEndian(&robotData.turret.barrelSpeedLimit42, message.data + 22);
    convertFromLittleEndian(&robotData.chassis.powerConsumptionLimit, message.data + 24);
    robotData.robotPower.value = message.data[26] & 0b111;
    robotData.robotDataReceivedTimestamp = clock::getTimeMilliseconds();

    processReceivedDamage(
        robotData.robotDataReceivedTimestamp,
        static_cast<int>(robotData.previousHp) - static_cast<int>(robotData.currentHp));
    robotData.previousHp = robotData.currentHp;

    return true;
}

bool RefSerial::decodeToPowerAndHeat(const ReceivedSerialMessage& message)
{
    if (message.header.dataLength != 16)
    {
        return false;
    }
    convertFromLittleEndian(&robotData.chassis.volt, message.data);
    convertFromLittleEndian(&robotData.chassis.current, message.data + 2);
    convertFromLittleEndian(&robotData.chassis.power, message.data + 4);
    convertFromLittleEndian(&robotData.chassis.powerBuffer, message.data + 8);
    convertFromLittleEndian(&robotData.turret.heat17ID1, message.data + 10);
    convertFromLittleEndian(&robotData.turret.heat17ID2, message.data + 12);
    convertFromLittleEndian(&robotData.turret.heat42, message.data + 14);
    return true;
}

bool RefSerial::decodeToRobotPosition(const ReceivedSerialMessage& message)
{
    if (message.header.dataLength != 16)
    {
        return false;
    }
    convertFromLittleEndian(&robotData.chassis.x, message.data);
    convertFromLittleEndian(&robotData.chassis.y, message.data + 4);
    convertFromLittleEndian(&robotData.chassis.z, message.data + 8);
    convertFromLittleEndian(&robotData.turret.yaw, message.data + 12);
    return true;
}

bool RefSerial::decodeToRobotBuffs(const ReceivedSerialMessage& message)
{
    if (message.header.dataLength != 1)
    {
        return false;
    }
    robotData.robotBuffStatus.value = message.data[0] & 0b1111;
    return true;
}

bool RefSerial::decodeToAerialEnergyStatus(const ReceivedSerialMessage& message)
{
    if (message.header.dataLength != 2)
    {
        return false;
    }
    convertFromLittleEndian(&robotData.aerialEnergyStatus, message.data);
    return true;
}

bool RefSerial::decodeToDamageStatus(const ReceivedSerialMessage& message)
{
    if (message.header.dataLength != 1)
    {
        return false;
    }
    robotData.damagedArmorId = static_cast<Rx::ArmorId>((message.data[0]) & 0xf);
    robotData.damageType = static_cast<Rx::DamageType>((message.data[0] >> 4) & 0xf);
    return true;
}

bool RefSerial::decodeToProjectileLaunch(const ReceivedSerialMessage& message)
{
    if (message.header.dataLength != 7)
    {
        return false;
    }
    robotData.turret.bulletType = static_cast<Rx::BulletType>(message.data[0]);
    robotData.turret.launchMechanismID = static_cast<Rx::MechanismID>(message.data[1]);
    robotData.turret.firingFreq = message.data[2];
    robotData.turret.lastReceivedLaunchingInfoTimestamp = clock::getTimeMilliseconds();
    convertFromLittleEndian(&robotData.turret.bulletSpeed, message.data + 3);
    return true;
}

bool RefSerial::decodeToBulletsRemain(const ReceivedSerialMessage& message)
{
    if (message.header.dataLength != 6)
    {
        return false;
    }
    convertFromLittleEndian(&robotData.turret.bulletsRemaining17, message.data);
    convertFromLittleEndian(&robotData.turret.bulletsRemaining42, message.data + 2);
    convertFromLittleEndian(&robotData.remainingCoins, message.data + 4);
    return true;
}

bool RefSerial::decodeToRFIDStatus(const ReceivedSerialMessage& message)
{
    if (message.header.dataLength != 4)
    {
        return false;
    }
    robotData.rfidStatus.value = message.data[0];
    return true;
}

bool RefSerial::handleRobotToRobotCommunication(const ReceivedSerialMessage& message)
{
    if (message.header.dataLength < sizeof(Tx::RobotToRobotMessage::interactiveHeader))
    {
        return false;
    }

    if (msgIdToRobotToRobotHandlerMap.size() == 0)
    {
        return true;
    }

    const Tx::InteractiveHeader* interactiveHeader =
        reinterpret_cast<const Tx::InteractiveHeader*>(message.data);

    if (msgIdToRobotToRobotHandlerMap.count(interactiveHeader->dataCmdId))
    {
        (*msgIdToRobotToRobotHandlerMap[interactiveHeader->dataCmdId])(message);
    }

    return true;
}

void RefSerial::processReceivedDamage(uint32_t timestamp, int32_t damageTaken)
{
    if (damageTaken > 0)
    {
        // create a new DamageEvent with the damage_taken, and current time
        Rx::DamageEvent damageEvent = {static_cast<uint16_t>(damageTaken), timestamp};

        if (receivedDpsTracker.getSize() == DPS_TRACKER_DEQUE_SIZE)
        {
            receivedDpsTracker.removeBack();
        }
        robotData.receivedDps += damageTaken;

        receivedDpsTracker.append(damageEvent);
    }
}

void RefSerial::updateReceivedDamage()
{
    // if current damage at head of circular array occurred more than a second ago,
    // decrease receivedDps by that amount of damage and increment head index
    while (receivedDpsTracker.getSize() > 0 &&
           clock::getTimeMilliseconds() > receivedDpsTracker.getFront().timestampMs + 1000)
    {
        robotData.receivedDps -= receivedDpsTracker.getFront().damageAmount;
        receivedDpsTracker.removeFront();
    }
}

RefSerial::RobotId RefSerial::getRobotIdBasedOnCurrentRobotTeam(RobotId id)
{
    if (id == RobotId::INVALID || robotData.robotId == RobotId::INVALID)
    {
        return id;
    }

    if (!isBlueTeam(robotData.robotId) && isBlueTeam(id))
    {
        id = id - RobotId::BLUE_HERO + RobotId::RED_HERO;
    }
    else if (isBlueTeam(robotData.robotId) && !isBlueTeam(id))
    {
        id = id - RobotId::RED_HERO + RobotId::BLUE_HERO;
    }
    return id;
}

void RefSerial::attachRobotToRobotMessageHandler(
    uint16_t msgId,
    RobotToRobotMessageHandler* handler)
{
    if (msgIdToRobotToRobotHandlerMap.count(msgId) != 0 || msgId < 0x0200 || msgId > 0x02FF)
    {
        RAISE_ERROR(drivers, "error adding msg handler");
        return;
    }

    msgIdToRobotToRobotHandlerMap[msgId] = handler;
}

bool RefSerial::operatorBlinded() const
{
    const uint32_t blindTime = (robotData.refereeWarningData.foulRobotID == robotData.robotId)
                                   ? Rx::RefereeWarningData::OFFENDING_OPERATOR_BLIND_TIME
                                   : Rx::RefereeWarningData::NONOFFENDING_OPERATOR_BLIND_TIME;

    const uint32_t lastReceivedWarningRobotTime =
        robotData.refereeWarningData.lastReceivedWarningRobotTime;

    return getRefSerialReceivingData() && (lastReceivedWarningRobotTime != 0) &&
           (arch::clock::getTimeMilliseconds() - lastReceivedWarningRobotTime <= blindTime);
}

}  // namespace tap::communication::serial
