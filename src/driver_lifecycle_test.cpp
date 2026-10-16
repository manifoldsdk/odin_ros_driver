#include "driver_lifecycle.h"
#include <iostream>
#include <stdexcept>
#include <thread>
#include <vector>

static void Require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

static YAML::Node MakeCalibration() {
    return YAML::Load(R"(
cam_0:
  image_width: 1600
  image_height: 1296
  A11: 732.37
  A12: 0.08
  A22: 732.0
  u0: 816.896
  v0: 640.986
  k2: -0.008
  k3: 0.018
  k4: -0.069
  k5: 0.068
  k6: -0.035
  k7: 0.005
  p1: 0.0
  p2: 0.0
Tcl_0: [1, 0, 0, 0.03, 0, 1, 0, 0.01, 0, 0, 1, -0.01, 0, 0, 0, 1]
)");
}

static void TestReconnectLifecycle() {
    odin_ros_driver::DriverLifecycle lifecycle;
    lidar_device_info_t deviceInfo{};
    deviceInfo.initial_state = LIDAR_DEVICE_STREAMING;
    deviceInfo.serial[0] = 'A';
    lifecycle.Submit(deviceInfo, true);
    deviceInfo.serial[0] = 'B';
    const auto firstEvent = lifecycle.TakeLatest();
    Require(firstEvent && firstEvent->attach, "Attach event was lost");
    Require(firstEvent->deviceInfo.serial[0] == 'A', "Device metadata was not copied");
    Require(firstEvent->deviceInfo.initial_state == LIDAR_DEVICE_STREAMING, "Streaming state was lost");
    Require(!lifecycle.connected, "Device became ready before initialization");
    Require(lifecycle.MarkReady(firstEvent->generation), "Current initialization was rejected");
    lifecycle.Submit(deviceInfo, false);
    Require(!lifecycle.connected, "Detach did not immediately stop publication");
    Require(!lifecycle.MarkReady(firstEvent->generation), "Stale initialization re-enabled publication");
    lifecycle.Submit(deviceInfo, true);
    const auto latestEvent = lifecycle.TakeLatest();
    Require(latestEvent && latestEvent->attach, "Latest reconnect was not retained");
    Require(latestEvent->generation > firstEvent->generation, "Connection generation did not advance");
    Require(!lifecycle.IsCurrent(firstEvent->generation), "Old callback generation remains valid");
    Require(lifecycle.MarkReady(latestEvent->generation), "Reconnect did not become ready");
    lifecycle.Stop();
    Require(!lifecycle.connected, "Shutdown did not stop publication");
    Require(!lifecycle.MarkReady(latestEvent->generation), "Shutdown accepted a late initialization");
    lifecycle.Submit(deviceInfo, true);
    Require(!lifecycle.TakeLatest(), "Shutdown accepted a late hotplug event");
}

static void TestConcurrentHotplug() {
    odin_ros_driver::DriverLifecycle lifecycle;
    std::atomic<int> running{4};
    std::vector<std::thread> producers;
    for (int i = 0; i < 4; ++i) {
        producers.emplace_back([&lifecycle, &running]() {
            lidar_device_info_t deviceInfo{};
            for (int j = 0; j < 5000; ++j) lifecycle.Submit(deviceInfo, j % 2 == 0);
            --running;
        });
    }
    while (running.load() > 0) {
        if (const auto event = lifecycle.TakeLatest()) lifecycle.MarkReady(event->generation);
        std::this_thread::yield();
    }
    for (auto& producer : producers) producer.join();
    lifecycle.Stop();
    Require(!lifecycle.connected && !lifecycle.TakeLatest(), "Concurrent hotplug survived shutdown");
}

static void TestWorkerStop() {
    for (int i = 0; i < 1000; ++i) {
        std::atomic<bool> running{true};
        std::mutex queueMutex;
        std::condition_variable queueCondition;
        std::thread worker([&]() {
            std::unique_lock<std::mutex> lock(queueMutex);
            queueCondition.wait(lock, [&]() { return !running; });
        });
        odin_ros_driver::StopWorker(running, queueMutex, queueCondition);
        worker.join();
        Require(!running, "Worker stop predicate remains set");
    }
}

static void TestCalibrationValidation() {
    Require(odin_ros_driver::ValidateCalibration(MakeCalibration()), "Valid calibration was rejected");
    Require(!odin_ros_driver::ValidateCalibration(YAML::Load("")), "Empty calibration was accepted");
    Require(!odin_ros_driver::ValidateCalibration(YAML::Load("cam_0: {}")), "Incomplete calibration was accepted");
    auto calibration = MakeCalibration();
    calibration.remove("cam_0");
    Require(!odin_ros_driver::ValidateCalibration(calibration), "Missing camera was accepted");
    calibration = MakeCalibration();
    calibration["cam_0"]["image_width"] = 0;
    Require(!odin_ros_driver::ValidateCalibration(calibration), "Zero image width was accepted");
    calibration = MakeCalibration();
    calibration["cam_0"]["A11"] = "invalid";
    Require(!odin_ros_driver::ValidateCalibration(calibration), "Invalid scalar was accepted");
    calibration = MakeCalibration();
    calibration["cam_0"]["k2"] = ".nan";
    Require(!odin_ros_driver::ValidateCalibration(calibration), "Non-finite intrinsic was accepted");
    calibration = MakeCalibration();
    calibration["cam_0"]["A22"] = -1;
    Require(!odin_ros_driver::ValidateCalibration(calibration), "Negative focal length was accepted");
    calibration = MakeCalibration();
    calibration["Tcl_0"] = YAML::Load("[1, 2, 3]");
    Require(!odin_ros_driver::ValidateCalibration(calibration), "Truncated extrinsic was accepted");
    calibration = MakeCalibration();
    calibration["Tcl_0"][0] = 0;
    Require(!odin_ros_driver::ValidateCalibration(calibration), "Singular extrinsic was accepted");
    calibration = MakeCalibration();
    calibration["Tcl_0"][15] = 0;
    Require(!odin_ros_driver::ValidateCalibration(calibration), "Invalid homogeneous transform was accepted");
    Require(!odin_ros_driver::ValidateCalibrationFile("/nonexistent/odin_calibration.yaml"), "Missing file was accepted");
}

int main(int argc, char* argv[]) {
    try {
        if (argc > 1) Require(odin_ros_driver::ValidateCalibrationFile(argv[1]), "Runtime calibration was rejected");
        TestReconnectLifecycle();
        TestConcurrentHotplug();
        TestWorkerStop();
        TestCalibrationValidation();
        std::cout << "Driver lifecycle and calibration regression tests passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
