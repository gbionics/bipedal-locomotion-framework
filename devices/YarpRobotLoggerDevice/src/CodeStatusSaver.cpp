/**
 * @copyright 2026 Generative Bionics S.R.L. This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <chrono>
#include <sstream>

#include <process.hpp>

#include <BipedalLocomotion/TextLogging/Logger.h>

#include <BipedalLocomotion/RobotLogger/CodeStatusSaver.h>

using namespace BipedalLocomotion::RobotLogger;

namespace
{
void findAndReplaceAll(std::string& data, const std::string& toSearch, const std::string& replace)
{
    std::size_t position = data.find(toSearch);
    while (position != std::string::npos)
    {
        data.replace(position, toSearch.size(), replace);
        position = data.find(toSearch, position + replace.size());
    }
}
} // namespace

CodeStatusSaver::CodeStatusSaver(std::vector<std::string> commands)
    : m_commands(std::move(commands))
{
}

void CodeStatusSaver::save(const std::string& fileName) const
{
    constexpr auto logPrefix = "[CodeStatusSaver::save]";

    if (m_commands.empty())
    {
        return;
    }

    const auto start = std::chrono::steady_clock::now();
    for (const auto& commandTemplate : m_commands)
    {
        std::string command = commandTemplate;
        findAndReplaceAll(command, "{filename}", fileName);

        log()->info("{} Running the code status command: {}", logPrefix, command);

        std::stringstream output;
        TinyProcessLib::Process process(command, "", [&output](const char* bytes, size_t n) {
            output << std::string(bytes, n);
        });
        const int exitStatus = process.get_exit_status();
        if (exitStatus != 0)
        {
            log()->warn("{} The command '{}' exited with status {}. Output: {}",
                        logPrefix,
                        command,
                        exitStatus,
                        output.str());
        }
    }

    log()->info("{} Status of the code saved in {}.",
                logPrefix,
                std::chrono::duration<double>(std::chrono::steady_clock::now() - start));
}
