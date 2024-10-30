#include <iostream>
#include <vector>
#include <string>
#include <thread>
#include <chrono>
#include <memory>
#include <future>
#include <cmath>
#include <iostream>
#include <vector>
#include <string>
#include <thread>
#include <sstream>

#include "radar.h"
#include "angular_speed_estimator.h"

// Define custom data structures to replace ROS message types
struct RadarEcho
{
    std::vector<float> echoes;
};

struct RadarSector
{
    std::chrono::steady_clock::time_point stamp;
    std::string frame_id;
    double angle_start;
    double angle_increment;
    double range_min;
    double range_max;
    std::vector<RadarEcho> intensities;
    std::chrono::duration<double> scan_time;
    std::chrono::duration<double> time_increment;
};

enum ControlType
{
    CONTROL_TYPE_ENUM,
    CONTROL_TYPE_FLOAT,
    CONTROL_TYPE_FLOAT_WITH_AUTO
};

struct RadarControlItem
{
    std::string name;
    std::string value;
    std::string label;
    ControlType type;
    float min_value;
    float max_value;
    std::vector<std::string> enums;
};

struct RadarControlSet
{
    std::vector<RadarControlItem> items;
};

struct RadarControlValue
{
    std::string key;
    std::string value;
};

class HaloRadar : public halo_radar::Radar
{
public:
    HaloRadar(halo_radar::AddressSet const &addresses) : halo_radar::Radar(addresses)
    {
        m_rangeCorrectionFactor = 1.024; // Default value
        m_frame_id = "radar";            // Default frame ID
        startHeartbeatTimer();
        startThreads();
    }

    void stopHeartbeatTimer()
    {
        m_running = false;
        if (m_heartbeatThread.joinable())
            m_heartbeatThread.join();
    }

    ~HaloRadar()
    {
        stopHeartbeatTimer();
    }

protected:
    void processData(std::vector<halo_radar::Scanline> const &scanlines) override
    {
        if (scanlines.empty())
            return;

        RadarSector rs;
        rs.stamp = std::chrono::steady_clock::now();
        rs.frame_id = m_frame_id;
        rs.angle_start = 2.0 * M_PI * (360 - scanlines.front().angle) / 360.0;
        double angle_max = 2.0 * M_PI * (360 - scanlines.back().angle) / 360.0;
        if (scanlines.size() > 1)
        {
            if (angle_max > rs.angle_start && angle_max - rs.angle_start > M_PI)
                angle_max -= 2.0 * M_PI;
            rs.angle_increment = (angle_max - rs.angle_start) / double(scanlines.size() - 1);
        }
        rs.range_min = 0.0;
        rs.range_max = scanlines.front().range;
        for (auto sl : scanlines)
        {
            RadarEcho echo;
            for (auto i : sl.intensities)
                echo.echoes.push_back(i / 15.0); // 4-bit int to float
            rs.intensities.push_back(echo);
        }

        auto angular_speed = m_estimator.update(rs.stamp, rs.angle_start);
        double scan_time = 0.0;
        if (angular_speed != 0.0)
            scan_time = 2 * M_PI / fabs(angular_speed);

        rs.scan_time = std::chrono::duration<double>(scan_time);

        double time_increment = 0.0;
        if (scan_time > 0)
            time_increment = std::abs(rs.angle_increment) / scan_time;
        rs.time_increment = std::chrono::duration<double>(time_increment);

        publishData(rs);
    }

    void stateUpdated() override
    {
        RadarControlSet rcs;

        std::string statusEnums[] = {"standby", "transmit", ""};

        createEnumControl("status", "Status", statusEnums, rcs);
        createFloatControl("range", "Range", 25, 75000, rcs);

        std::string modeEnums[] = {"custom", "harbor", "offshore", "weather", "bird", ""};

        createEnumControl("mode", "Mode", modeEnums, rcs);
        createFloatWithAutoControl("gain", "gain_mode", "Gain", 0, 100, rcs);
        createFloatWithAutoControl("sea_clutter", "sea_clutter_mode", "Sea clutter", 0, 100, rcs);
        createFloatControl("auto_sea_clutter_nudge", "Auto sea clut adj", -50, 50, rcs);

        std::string seaStateEnums[] = {"calm", "moderate", "rough", ""};

        createEnumControl("sea_state", "Sea state", seaStateEnums, rcs);
        createFloatControl("rain_clutter", "Rain clutter", 0, 100, rcs);

        std::string lowMedHighEnums[] = {"off", "low", "medium", "high", ""};

        createEnumControl("noise_rejection", "Noise rejection", lowMedHighEnums, rcs);
        createEnumControl("target_expansion", "Target expansion", lowMedHighEnums, rcs);
        createEnumControl("interference_rejection", "Interf. rej", lowMedHighEnums, rcs);
        createEnumControl("target_separation", "Target separation", lowMedHighEnums, rcs);

        std::string scanSpeedEnums[] = {"off", "medium", "high", ""};

        createEnumControl("scan_speed", "Fast scan", scanSpeedEnums, rcs);

        std::string dopplerModeEnums[] = {"off", "normal", "approaching_only", ""};

        createEnumControl("doppler_mode", "VelocityTrack", dopplerModeEnums, rcs);
        createFloatControl("doppler_speed", "Speed threshold", 0.05, 15.95, rcs);
        createFloatControl("antenna_height", "Antenna height", 0.0, 30.175, rcs);
        createFloatControl("bearing_alignment", "Bearing alignment", 0, 360, rcs);
        createFloatWithAutoControl("sidelobe_suppression", "sidelobe_suppression_mode", "Sidelobe sup.", 0, 100, rcs);
        createEnumControl("lights", "Halo light", lowMedHighEnums, rcs);

        publishState(rcs);
    }

private:
    void onStateChange(const RadarControlValue &cv)
    {
        sendCommand(cv.key, cv.value);
    }

    void hbTimerCallback()
    {
        if (checkHeartbeat())
            stateUpdated();
    }

    void createEnumControl(const std::string &name, const std::string &label, const std::string enums[],
                           RadarControlSet &rcs)
    {
        if (m_state.find(name) != m_state.end())
        {
            RadarControlItem rci;
            rci.name = name;
            rci.value = m_state[name];
            rci.label = label;
            rci.type = CONTROL_TYPE_ENUM;
            for (int i = 0; !enums[i].empty(); i++)
                rci.enums.push_back(enums[i]);
            rcs.items.push_back(rci);
        }
    }

    void createFloatControl(const std::string &name, const std::string &label, float min_value, float max_value,
                            RadarControlSet &rcs)
    {
        if (m_state.find(name) != m_state.end())
        {
            RadarControlItem rci;
            rci.name = name;
            rci.value = m_state[name];
            rci.label = label;
            rci.type = CONTROL_TYPE_FLOAT;
            rci.min_value = min_value;
            rci.max_value = max_value;
            rcs.items.push_back(rci);
        }
    }

    void createFloatWithAutoControl(const std::string &name, const std::string &auto_name, const std::string &label,
                                    float min_value, float max_value, RadarControlSet &rcs)
    {
        if (m_state.find(name) != m_state.end() && m_state.find(auto_name) != m_state.end())
        {
            RadarControlItem rci;
            rci.name = name;
            std::string value = m_state[name];
            if (m_state[auto_name] == "auto")
                value = "auto";
            rci.value = value;
            rci.label = label;
            rci.type = CONTROL_TYPE_FLOAT_WITH_AUTO;
            rci.min_value = min_value;
            rci.max_value = max_value;
            rcs.items.push_back(rci);
        }
    }

    void publishData(const RadarSector &rs)
    {
        // Implement handling of radar data
        // For example, print data or process it further
        std::cout << "Radar data received at time: " << std::chrono::duration_cast<std::chrono::milliseconds>(rs.stamp.time_since_epoch()).count() << " ms" << std::endl;
    }

    void publishState(const RadarControlSet &rcs)
    {
        // Implement handling of radar state
        // For example, update UI, log state, etc.
        std::cout << "Radar state updated with " << rcs.items.size() << " items." << std::endl;
    }

    void startHeartbeatTimer()
    {
        m_running = true;
        m_heartbeatThread = std::thread([this]() {
            while (m_running)
            {
                std::this_thread::sleep_for(std::chrono::seconds(1));
                hbTimerCallback();
            }
        });
    }

    double m_rangeCorrectionFactor = 1.024;
    std::string m_frame_id = "radar";
    AngularSpeedEstimator m_estimator;
    std::thread m_heartbeatThread;
    bool m_running = true;
};

std::shared_ptr<halo_radar::HeadingSender> headingSender;

// Function to handle user commands
void commandHandler(std::shared_ptr<HaloRadar> radar)
{
    std::string line;
    std::cout << "Enter commands (type 'help' for a list of commands, 'exit' to quit):" << std::endl;
    while (true)
    {
        std::cout << "> ";
        if (!std::getline(std::cin, line))
            break; // EOF or error

        std::istringstream iss(line);
        std::string command;
        iss >> command;

        if (command.empty())
            continue;

        if (command == "exit")
            break;

        if (command == "help")
        {
            std::cout << "Supported commands:" << std::endl;
            std::cout << "  status [standby|transmit]" << std::endl;
            std::cout << "  range [value]" << std::endl;
            std::cout << "  gain [value|auto]" << std::endl;
            std::cout << "  sea_clutter [value|auto]" << std::endl;
            std::cout << "  rain_clutter [value]" << std::endl;
            std::cout << "  sidelobe_suppression [value|auto]" << std::endl;
            // Add more commands as needed
            continue;
        }

        // Parse command and arguments
        std::string arg1;
        iss >> arg1;

        if (command == "status" && (arg1 == "standby" || arg1 == "transmit" || arg1 == "spinning_up" || arg1 == "unknown"))
        {
            radar->sendCommand(command, arg1);
        }
        else if (command == "range")
        {
            try
            {
                float range = std::stof(arg1);
                radar->sendCommand(command, std::to_string(range));
            }
            catch (const std::invalid_argument &)
            {
                std::cerr << "Invalid range value." << std::endl;
            }
        }
        else if (command == "gain")
        {
            if (arg1 == "auto" || arg1 == "manual")
            {
                radar->sendCommand("gain_mode", arg1 == "auto" ? "auto" : "manual");
                if (arg1 != "auto")
                {
                    try
                    {
                        float gain = std::stof(arg1);
                        radar->sendCommand(command, std::to_string(gain));
                    }
                    catch (const std::invalid_argument &)
                    {
                        std::cerr << "Invalid gain value." << std::endl;
                    }
                }
            }
            else
            {
                try
                {
                    float gain = std::stof(arg1);
                    radar->sendCommand(command, std::to_string(gain));
                }
                catch (const std::invalid_argument &)
                {
                    std::cerr << "Invalid gain value." << std::endl;
                }
            }
        }
        else if (command == "sea_clutter")
        {
            if (arg1 == "auto" || arg1 == "manual")
            {
                radar->sendCommand("sea_clutter_mode", arg1 == "auto" ? "auto" : "manual");
                if (arg1 != "auto")
                {
                    try
                    {
                        float sea_clutter = std::stof(arg1);
                        radar->sendCommand(command, std::to_string(sea_clutter));
                    }
                    catch (const std::invalid_argument &)
                    {
                        std::cerr << "Invalid sea clutter value." << std::endl;
                    }
                }
            }
            else
            {
                try
                {
                    float sea_clutter = std::stof(arg1);
                    radar->sendCommand(command, std::to_string(sea_clutter));
                }
                catch (const std::invalid_argument &)
                {
                    std::cerr << "Invalid sea clutter value." << std::endl;
                }
            }
        }
        else if (command == "rain_clutter")
        {
            try
            {
                float rain_clutter = std::stof(arg1);
                radar->sendCommand(command, std::to_string(rain_clutter));
            }
            catch (const std::invalid_argument &)
            {
                std::cerr << "Invalid rain clutter value." << std::endl;
            }
        }
        else if (command == "sidelobe_suppression")
        {
            if (arg1 == "auto" || arg1 == "manual")
            {
                radar->sendCommand("sidelobe_suppression_mode", arg1 == "auto" ? "auto" : "manual");
                if (arg1 != "auto")
                {
                    try
                    {
                        float sidelobe_suppression = std::stof(arg1);
                        radar->sendCommand(command, std::to_string(sidelobe_suppression));
                    }
                    catch (const std::invalid_argument &)
                    {
                        std::cerr << "Invalid sidelobe suppression value." << std::endl;
                    }
                }
            }
            else
            {
                try
                {
                    float sidelobe_suppression = std::stof(arg1);
                    radar->sendCommand(command, std::to_string(sidelobe_suppression));
                }
                catch (const std::invalid_argument &)
                {
                    std::cerr << "Invalid sidelobe suppression value." << std::endl;
                }
            }
        }
        else
        {
            std::cerr << "Unknown or incomplete command. Type 'help' for a list of commands." << std::endl;
        }
    }
}

int main(int argc, char **argv)
{
    std::vector<std::shared_ptr<HaloRadar>> radars;
    std::vector<uint32_t> hostIPs;
    // Optionally populate hostIPs from command-line arguments or configuration

    // Start the scanning thread
    std::future<void> scanResult = std::async(std::launch::async, [&]()
                                              {
        while (radars.empty())
        {
            std::vector<halo_radar::AddressSet> as;
            if (hostIPs.empty())
                as = halo_radar::scan();
            else
                as = halo_radar::scan(hostIPs);
            if (as.empty())
                std::cerr << "No radars found!" << std::endl;
            for (auto a : as)
            {
                radars.push_back(std::make_shared<HaloRadar>(a));
                if (!headingSender)
                    headingSender = std::make_shared<halo_radar::HeadingSender>(a.interface);
            }
            if (radars.empty())
                std::this_thread::sleep_for(std::chrono::seconds(1));
        } });

    // Wait until at least one radar is found
    scanResult.wait();

    if (radars.empty())
    {
        std::cerr << "Failed to find any radars. Exiting." << std::endl;
        return -1;
    }

    // Start the command handler thread
    std::thread cmdThread(commandHandler, radars[0]); // Assuming single radar

    // Optionally, perform other tasks or enter a main loop here

    // Wait for the command thread to finish (user types 'exit')
    cmdThread.join();

    // Clean up resources
    for (auto radar : radars)
    {
        radar->stopHeartbeatTimer();
    }

    // Optionally, perform additional cleanup or logging here

    std::cout << "Exiting Radar Application." << std::endl;

    return 0;
}