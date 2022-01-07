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

#ifndef CURRENT_SENSOR_INTERFACE_HPP_
#define CURRENT_SENSOR_INTERFACE_HPP_

#include "tap/communication/sensors/sensor_interface.hpp"

namespace tap::communication::sensors::current
{
/**
 * Interface for a generic current sensor.
 */
class CurrentSensorInterface : public tap::communication::sensors::SensorInterface
{
public:
    /**
     * @return The current read by the current sensor, in milliamps.
     */
    virtual float getCurrentMa() const = 0;
};
}  // namespace tap::communication::sensors::current

#endif  // CURRENT_SENSOR_INTERFACE_HPP_
