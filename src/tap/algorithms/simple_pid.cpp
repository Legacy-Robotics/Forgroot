#include "simple_pid.hpp"

#include "tap/algorithms/math_user_utils.hpp"

using namespace tap::algorithms;

namespace tap
{
namespace algorithms
{
SimplePid::SimplePid(const SimplePidConfig &pidConfig)
    : config(pidConfig)
{
}

float SimplePid::runController(float error, float errorDerivative, float dt)
{
    if (abs(error) < config.errDeadzone)
    {
        error = 0.0f;
    }

    // p
    currErrorP = config.kp * error;
    // i
    currErrorI = limitVal<float>(
        currErrorI + config.ki * error * dt,
        -config.maxICumulative,
        config.maxICumulative);
    // d
    currErrorD = -config.kd * errorDerivative;
    if (fabs(error) < config.errorDerivativeFloor)
    {
        // the error is less than some amount, so round derivative output to 0
        // done to avoid high frequency control oscilations in some systems
        currErrorD = 0.0f;
    }
    // total
    output =
        limitVal<float>(currErrorP + currErrorI + currErrorD, -config.maxOutput, config.maxOutput);
    return output;
}

float SimplePid::runControllerDerivateError(float error, float dt)
{
    if (compareFloatClose(dt, 0.0f, 1E-5))
    {
        dt = 1.0f;
    }
    float errorDerivative = (error - prevError) / dt;
    prevError = error;
    return runController(error, errorDerivative, dt);
}

float SimplePid::getOutput() { return output; }

void SimplePid::reset()
{
    this->output = 0.0f;
    this->currErrorP = 0.0f;
    this->currErrorI = 0.0f;
    this->currErrorD = 0.0f;
    this->prevError = 0.0f;
}

}  // namespace algorithms

}  // namespace tap
