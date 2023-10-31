#ifndef TAPROOT_SIMPLE_PID_HPP_
#define TAPROOT_SIMPLE_PID_HPP_

#include <cstdint>

namespace tap
{
namespace algorithms
{
class SimplePIDConfig
{
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    float maxICumulative = 0.0f;
    float maxOutput = 0.0f;
    float errDeadzone = 0.0f;          /**< Within [-errDeadzone, errDeadzone], the PID controller
                                        * error will be set to 0. */
    float errorDerivativeFloor = 0.0f; /**< Minimum error value at which the PID controller will
                                        * compute and apply the derivative term. */
}
class SimplePID
{
    SimplePID(const SimplePIDConfig &pidConfig);

    /**
     * Runs the PID controller. Should be called frequently for best results.
     *
     * @param[in] error The error (in user-defined units) between some target and measured value.
     * @param[in] errorDerivative The derivative of the error passed in above (in user-defined units
     * / time).
     * @param[in] dt The difference in time between the time this function is being called and the
     * last time this function was called.
     */
    virtual float runController(float error, float errorDerivative, float dt);

    float runControllerDerivateError(float error, float dt);

    float getOutput();

    void reset();

    inline void setP(float p) { config.kp = p; }
    inline void setI(float i) { config.ki = i; }
    inline void setD(float d) { config.kd = d; }
    inline void setMaxICumulative(float maxICumulative) { config.maxICumulative = maxICumulative; }
    inline void setMaxOutput(float maxOutput) { config.maxOutput = maxOutput; }
    inline void setErrDeadzone(float errDeadzone) { config.errDeadzone = errDeadzone; }

private:
    // gains and constants, to be set by the user
    SimplePIDConfig config;

    // while these could be local, debugging pid is much easier if they are not
    float currErrorP = 0.0f;
    float currErrorI = 0.0f;
    float currErrorD = 0.0f;
    float output = 0.0f;
    float prevError = 0.0f;
}
} //namespace algorithms
} //namespace tap