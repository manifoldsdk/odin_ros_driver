#ifndef ODIN_DRIVER_LIFECYCLE_H
#define ODIN_DRIVER_LIFECYCLE_H

#include "lidar_api_type.h"
#include <yaml-cpp/yaml.h>
#include <array>
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>

namespace odin_ros_driver {

struct DeviceEvent {
    lidar_device_info_t deviceInfo{};
    bool attach = false;
    uint64_t generation = 0;
};

class DriverLifecycle {
public:
    std::atomic<bool> connected{false};

    void Submit(const lidar_device_info_t& deviceInfo, bool attach) {
        std::lock_guard<std::mutex> lock(eventMutex);
        if (stopped) return;
        connected.store(false);
        const auto nextGeneration = generation.fetch_add(1) + 1;
        pendingEvent = DeviceEvent{deviceInfo, attach, nextGeneration};
    }

    std::optional<DeviceEvent> TakeLatest() {
        std::lock_guard<std::mutex> lock(eventMutex);
        auto event = pendingEvent;
        pendingEvent.reset();
        return event;
    }

    bool IsCurrent(uint64_t eventGeneration) const {
        return eventGeneration == generation.load();
    }

    bool MarkReady(uint64_t eventGeneration) {
        std::lock_guard<std::mutex> lock(eventMutex);
        if (stopped || !IsCurrent(eventGeneration)) return false;
        connected.store(true);
        return true;
    }

    void Stop() {
        std::lock_guard<std::mutex> lock(eventMutex);
        stopped = true;
        connected.store(false);
        generation.fetch_add(1);
        pendingEvent.reset();
    }

private:
    mutable std::mutex eventMutex;
    std::atomic<uint64_t> generation{0};
    std::optional<DeviceEvent> pendingEvent;
    bool stopped = false;
};

inline void StopWorker(std::atomic<bool>& running, std::mutex& queueMutex, std::condition_variable& queueCondition) {
    {
        std::lock_guard<std::mutex> lock(queueMutex);
        running = false;
    }
    queueCondition.notify_all();
}

inline bool ValidateCalibration(const YAML::Node& config) {
    try {
        const auto camera = config["cam_0"];
        const auto transform = config["Tcl_0"];
        if (!camera.IsMap() || !transform.IsSequence() || transform.size() != 16) return false;
        if (camera["image_width"].as<int>() <= 0 || camera["image_height"].as<int>() <= 0) return false;
        for (const auto* key : {"A11", "A12", "A22", "u0", "v0", "k2", "k3", "k4", "k5", "k6", "k7", "p1", "p2"}) {
            if (!std::isfinite(camera[key].as<double>())) return false;
        }
        if (camera["A11"].as<double>() <= 0 || camera["A22"].as<double>() <= 0) return false;
        std::array<double, 16> matrix{};
        for (size_t i = 0; i < matrix.size(); ++i) {
            matrix[i] = transform[i].as<double>();
            if (!std::isfinite(matrix[i])) return false;
        }
        const double determinant = matrix[0] * (matrix[5] * matrix[10] - matrix[6] * matrix[9])
            - matrix[1] * (matrix[4] * matrix[10] - matrix[6] * matrix[8])
            + matrix[2] * (matrix[4] * matrix[9] - matrix[5] * matrix[8]);
        return std::abs(determinant) > 1e-9 && std::abs(matrix[12]) < 1e-6
            && std::abs(matrix[13]) < 1e-6 && std::abs(matrix[14]) < 1e-6
            && std::abs(matrix[15] - 1.0) < 1e-6;
    } catch (const YAML::Exception&) {
        return false;
    }
}

inline bool ValidateCalibrationFile(const std::string& path) {
    try {
        return ValidateCalibration(YAML::LoadFile(path));
    } catch (const YAML::Exception&) {
        return false;
    }
}

}

#endif
