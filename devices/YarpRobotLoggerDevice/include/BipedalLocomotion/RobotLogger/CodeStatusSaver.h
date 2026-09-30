/**
 * @copyright 2026 Generative Bionics S.R.L. This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_ROBOT_LOGGER_CODE_STATUS_SAVER_H
#define BIPEDAL_LOCOMOTION_ROBOT_LOGGER_CODE_STATUS_SAVER_H

#include <string>
#include <vector>

namespace BipedalLocomotion
{
namespace RobotLogger
{

/**
 * CodeStatusSaver runs a set of commands every time a file is saved, e.g., to store the status
 * of the code used to generate the data. The `{filename}` placeholder in a command is replaced by
 * the path of the saved file without extension.
 */
class CodeStatusSaver
{
public:
    explicit CodeStatusSaver(std::vector<std::string> commands);

    void save(const std::string& fileName) const;

private:
    std::vector<std::string> m_commands;
};

} // namespace RobotLogger
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_ROBOT_LOGGER_CODE_STATUS_SAVER_H
