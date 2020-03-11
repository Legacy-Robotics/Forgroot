#include "sentinel_drive_subsystem.hpp"
#include "src/aruwlib/algorithms/math_user_utils.hpp"
#include "src/aruwlib/motor/dji_motor.hpp"
#include "src/aruwlib/errors/create_errors.hpp"

namespace aruwsrc
{

namespace control
{
    void SentinelDriveSubsystem::initLimitSwitches()
    {
        leftLimitSwitch::setInputTrigger(modm::platform::Gpio::InputTrigger::RisingEdge);
        leftLimitSwitch::configure(modm::platform::Gpio::InputType::PullDown);

        rightLimitSwitch::setInputTrigger(modm::platform::Gpio::InputTrigger::RisingEdge);
        rightLimitSwitch::configure(modm::platform::Gpio::InputType::PullDown);
    }

    void SentinelDriveSubsystem::setDesiredRpm(float desRpm)
    {
        desiredRpm = desRpm;
    }

    void SentinelDriveSubsystem::refresh()
    {
        velocityPidLeftWheel.update(desiredRpm - leftWheel.getShaftRPM());
        leftWheel.setDesiredOutput(velocityPidLeftWheel.getValue());
        velocityPidRightWheel.update(desiredRpm - rightWheel.getShaftRPM());
        rightWheel.setDesiredOutput(velocityPidRightWheel.getValue());
        // constantly poll the limit switches, resetting offset if needed
        resetOffsetFromLimitSwitch();
    }

    float SentinelDriveSubsystem::absolutePosition ()
    {
        float leftPosition = distanceFromEncoder(&leftWheel) - leftZeroRailOffset;
        float rightPosition = distanceFromEncoder(&rightWheel) - rightZeroRailOffset;
        float average = 0.0f;
        if (leftWheel.isMotorOnline() && rightWheel.isMotorOnline())
        {
            average = (leftPosition + rightPosition) / 2.0f;
        }
        else if (leftWheel.isMotorOnline())
        {
            RAISE_ERROR("right sentinel drive motor offline", aruwlib::errors::Location::SUBSYSTEM,
                    aruwlib::errors::ErrorType::MOTOR_OFFLINE);
            average = leftPosition;
        }
        else if (rightWheel.isMotorOnline())
        {
            RAISE_ERROR("left sentinel drive motor offline", aruwlib::errors::Location::SUBSYSTEM,
                    aruwlib::errors::ErrorType::MOTOR_OFFLINE);
            average = rightPosition;
        }
        else
        {
            RAISE_ERROR("both sentinel drive motors offline", aruwlib::errors::Location::SUBSYSTEM,
                    aruwlib::errors::ErrorType::MOTOR_OFFLINE);
            average = 0.0f;
        }
        return aruwlib::algorithms::limitVal<float>(average, 0.0f,
                SentinelDriveSubsystem::RAIL_LENGTH);
    }

    // Resets the encoder offset used to determine position of the sentinel on the rail depending on
    // which limit switch is hit. If neither limit switch is hit, no-op. Left limit switch indicates
    // being at the start of the rail, right limit switch indicates end of rail.
    void SentinelDriveSubsystem::resetOffsetFromLimitSwitch ()
    {
        if (leftLimitSwitch::read()) {  // DigitalPin where limit switch is placed
            leftZeroRailOffset = distanceFromEncoder(&leftWheel);
            rightZeroRailOffset = distanceFromEncoder(&rightWheel);
        } else if (rightLimitSwitch::read()) {
            leftZeroRailOffset = RAIL_LENGTH - distanceFromEncoder(&leftWheel);
            rightZeroRailOffset = RAIL_LENGTH - distanceFromEncoder(&rightWheel);
        }
    }

    // Returns the distance covered by the sentinel wheel on the rail
    // with respect to the encoders
    // Equation used: Arc Length = Angle * numberOfRotations * radius
    // Here we get the radius from the getEncoderUnwrapped function
    float SentinelDriveSubsystem::distanceFromEncoder(aruwlib::motor::DjiMotor* motor){
        float unwrappedAngle = motor->encStore.getEncoderUnwrapped();
        float numberOfRotations = unwrappedAngle / (aruwlib::motor::DjiMotor::ENC_RESOLUTION);
        return numberOfRotations * 2.0f * aruwlib::algorithms::PI * WHEEL_RADIUS / GEAR_RATIO;
    }
}  // namespace control

}  // namespace aruwsrc