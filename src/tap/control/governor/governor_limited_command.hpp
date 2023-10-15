/*
 * Copyright (c) 2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef TAPROOT_GOVERNOR_LIMITED_COMMAND_HPP_
#define TAPROOT_GOVERNOR_LIMITED_COMMAND_HPP_

#include <array>
#include <cassert>
#include <vector>

#include "../command.hpp"

#include "command_governor_interface.hpp"

namespace tap::control::governor
{
/**
 * A command that runs another command, but will only execute the command when some list of
 * governors all allow it to. All governors also have control over ending the command. If one of the
 * governors believes the command should be finished, the command will be finished.
 *
 * @tparam NUM_CONDITIONS The number of governors in the governor list.
 */
template <size_t NUM_CONDITIONS>
class GovernorLimitedCommand : public Command
{
public:
    GovernorLimitedCommand(
        const std::vector<Subsystem *> &subRequirements,
        Command &command,
        const std::array<CommandGovernorInterface *, NUM_CONDITIONS> &commandGovernorList)
        : command(command),
          commandGovernorList(commandGovernorList)
    {
        std::for_each(subRequirements.begin(), subRequirements.end(), [&](auto sub) {
            addSubsystemRequirement(sub);
        });
        assert(command.getRequirementsBitwise() == this->getRequirementsBitwise());
    }

    const char *getName() const override { return command.getName(); }

    bool isReady() override
    {
        return std::all_of(
                   commandGovernorList.begin(),
                   commandGovernorList.end(),
                   [](auto governor) { return governor->isReady(); }) &&
               command.isReady();
    }

    void initialize() override
    {
        command.initialize();

        for (auto governor : commandGovernorList)
        {
            governor->onGovernedCommandInitialized();
        }
    }

    void execute() override { command.execute(); }

    void end(bool interrupted) override { command.end(interrupted); }

    bool isFinished() const override
    {
        return std::any_of(
                   commandGovernorList.begin(),
                   commandGovernorList.end(),
                   [](auto governor) { return governor->isFinished(); }) ||
               command.isFinished();
    }

private:
    Command &command;

    std::array<CommandGovernorInterface *, NUM_CONDITIONS> commandGovernorList;
};
}  // namespace tap::control::governor

#endif  // TAPROOT_GOVERNOR_LIMITED_COMMAND_HPP_