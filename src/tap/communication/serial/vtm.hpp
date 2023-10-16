/* Copyright Legacy Robotics 2023*/
#ifndef TAPROOT_VTM_HPP_
#define TAPROOT_VTM_HPP_

#include <cstdint>
#include <unordered_map>

#include "tap/architecture/timeout.hpp"
#include "tap/util_macros.hpp"

#include "modm/container/deque.hpp"
#include "modm/processing/protothread/semaphore.hpp"

#include "dji_serial.hpp"

namespace tap
{
class Drivers;
}

namespace tap::communication::serial
{
/**
 * A class designed to communicate with the 2021 version of the RoboMaster
 * referee system. Supports decoding various referee serial message types. Also supports sending
 * custom UI messages to the referee serial and sending custom robot to robot communication.
 *
 * For information about the protocol that this serial parser/decoder uses,
 * view RoboMaster's ref serial website:
 * https://www.robomaster.com/en-US/products/components/referee (in the Document Download tab).
 *
 * @note use the instance stored in the `Drivers` to interact with this class
 *      (you shouldn't be declaring your own `RefSerial` object).
 *
 * Receive information from the referee serial by continuously calling `messageReceiveCallback`.
 * Access data sent by the referee serial by calling `getRobotData` or `getGameData`.
 */
class Vtm : public DJISerial
{
private:
    /**
     * Time since last message is received before we deem the referee serial port offline
     */
    static constexpr uint32_t TIME_OFFLINE_REF_DATA_MS = 1000;

public:
    /**
     * RX message type defines, referred to as "Command ID"s in the RoboMaster Ref System
     * Protocol Appendix. Ignored message types commented out because they are not handled by this
     * parser yet. They are values that are used in message headers to indicate the type of message
     * we have received.
     */
    enum MessageType
    {
        REF_MESSAGE_TYPE_VTM_CONTROL = 0x304 // message containing VTM data
    };

    /**
     * Constructs a VTM class connected to `bound_ports::VTM_SERIAL_UART_PORT` with
     * CRC enforcement enabled.
     *
     * @see `DjiSerial`
     */
    Vtm(Drivers* drivers);
    DISALLOW_COPY_AND_ASSIGN(Vtm)
    mockable ~Vtm() = default;

    enum class Key
    {
        W = 0,
        S,
        A,
        D,
        SHIFT,
        CTRL,
        Q,
        E,
        R,
        F,
        G,
        Z,
        X,
        C,
        V,
        B
    };

    enum class Key : uint16_t 
    {
        W = modm::Bit0,
        S = modm::Bit1,
        A = modm::Bit2,
        D = modm::Bit3,
        SHIFT = modm::Bit4,
        CTRL = modm::Bit5,
        Q = modm::Bit6,
        E = modm::Bit7,
        R = modm::Bit8,
        F = modm::Bit9,
        G = modm::Bit10,
        Z = modm::Bit11,
        X = modm::Bit12,
        C = modm::Bit13,
        V = modm::Bit14,
        B = modm::Bit15
    };

    /**
     * Handles the types of messages defined above in the RX message handlers section.
     */
    void messageReceiveCallback(const ReceivedSerialMessage& completeMessage) override;

    mockable bool getRefSerialReceivingData() const;

    /**
     * Used by `RefSerialTransmitter`. It is necessary to acquire this lock to coordinate sending
     * ref serial data from different protothreads.
     */
    mockable void acquireTransmissionSemaphore() { transmissionSemaphore.acquire(); }

    mockable void releaseTransmissionSemaphore() { transmissionSemaphore.release(); }

    /**
     * @return `true` if the remote is connected, `false` otherwise.
     * @note A timer is used to determine if the remote is disconnected, so expect a
     *      second or so of delay from disconnecting the remote to this function saying
     *      the remote is disconnected.
     */
    mockable bool isConnected() const;

    /**
     * @return The current mouse x value.
     */
    mockable inline int16_t getMouseX() const { return vtm.mouse.x; }

    /**
     * @return The current mouse y value.
     */
    mockable inline int16_t getMouseY() const { return vtm.mouse.y; }

    /**
     * @return The current mouse z value.
     */
    mockable inline int16_t getMouseZ() const { return vtm.mouse.z; }

    /**
     * @return The current mouse l value.
     */
    mockable inline bool getMouseL() const { return vtm.mouse.l; }

    /**
     * @return The current mouse r value.
     */
    mockable inline bool getMouseR() const { return vtm.mouse.r; }

    /**
     * @return `true` if the given `key` is pressed, `false` otherwise.
     */
    mockable inline bool keyPressed(Key key) const
    {
        return (vtm.key & (1 << static_cast<uint8_t>(key))) != 0;
    }
private:
    Drivers *drivers;
    arch::MilliTimeout VTMOfflineTimeout;
    modm::pt::Semaphore transmissionSemaphore;

    bool decodeVTMControl(const ReceivedSerialMessage& message);

    struct VTMInfo
    {
        uint32_t updateCounter = 0;
        /// Mouse information
        struct
        {
            int16_t x = 0;
            int16_t y = 0;
            int16_t wheel = 0;
            bool l = false;
            bool r = false;
        } mouse;
        uint16_t key = 0;   ///< Keyboard information
    };

    VTMInfo vtm;
};

}  // namespace tap::communication::serial

#endif  // TAPROOT_REF_SERIAL_HPP_
