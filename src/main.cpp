// src/main.cpp

#include <iostream>
#include <memory>
#include <thread>
#include <chrono>
#include "radar.h"
#include "logger.h"

// Since Radar is an abstract class, we need to implement the pure virtual functions
class MyRadar : public halo_radar::Radar {
public:
    MyRadar(const halo_radar::AddressSet& addresses, quill::Logger* logger)
        : halo_radar::Radar(addresses), m_logger(logger)
    {
        startThreads();
    }

protected:
    void processData(const std::vector<halo_radar::Scanline>& scanlines) override {
        // Log the number of scanlines received
        LOG_INFO(m_logger, "Received {} scanlines.", scanlines.size());
    }

    void stateUpdated() override {
        // Log that the state has been updated
        LOG_INFO(m_logger, "Radar state updated.");
    }

private:
    quill::Logger* m_logger;
};

int main(int argc, char** argv)
{
    // Initialize the logger using your custom function
    quill::Logger* logger = initialize_logger();

    // Check if logger was initialized successfully
    if (!logger)
    {
        std::cerr << "Failed to initialize logger." << std::endl;
        return -1;
    }

    // Start scanning for radars
    std::vector<halo_radar::AddressSet> addressSets = halo_radar::scan(logger);

    if (addressSets.empty()) {
        LOG_CRITICAL(logger, "No radars found! Exiting.");
        return -1;
    }

    // Use the first radar found
    halo_radar::AddressSet radarAddress = addressSets[0];
    LOG_INFO(logger, "Radar found at: {}", radarAddress.str());

    // Create a MyRadar instance
    std::shared_ptr<MyRadar> radar = std::make_shared<MyRadar>(radarAddress, logger);

    // Send a command to the radar
    LOG_INFO(logger, "Sending command: status = transmit");
    radar->sendCommand("status", "transmit");

    // Wait for a few seconds to receive data
    std::this_thread::sleep_for(std::chrono::seconds(5));

    // Clean up
    LOG_INFO(logger, "Shutting down radar communication.");
    // Radar destructor will handle stopping threads

    return 0;
}

