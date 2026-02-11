/*
Copyright 2025 Manifold Tech Ltd.(www.manifoldtech.com.co)
Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at
   http://www.apache.org/licenses/LICENSE-2.0
Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/
#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <deque>
#include <filesystem>
#include <fstream>
#include <chrono>
#include <memory>
#include <cstdlib>
#include <ctime>

class BinaryDataLogger {
public:
    struct Options {
    public:
        // Number of frames per batch before flushing to disk
        size_t batch_size = 20; 
        // Base directory to place time-stamped run directory under
        // If empty, will use $ODIN_DATA_DIR or default to $HOME/OdinData
        std::filesystem::path base_dir;
    };

    explicit BinaryDataLogger(const Options& opts) {
        namespace fs = std::filesystem;
        // Determine base dir
        fs::path base = opts.base_dir;
        if (base.empty()) {
            const char* env_dir = std::getenv("ODIN_DATA_DIR");
            if (env_dir && *env_dir) {
                base = fs::path(env_dir);
            } else {
                const char* home = std::getenv("HOME");
                base = home && *home ? fs::path(home) / "OdinData" : fs::current_path() / "OdinData";
            }
        }
        // Make timestamp folder name: YYYYMMDD_HHMMSS
        auto now = std::chrono::system_clock::now();
        std::time_t t = std::chrono::system_clock::to_time_t(now);
        std::tm tm{};
        #ifdef _WIN32
            localtime_s(&tm, &t);
        #else
            localtime_r(&t, &tm);
        #endif
        char buf[32];
        std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &tm);
        root_dir_ = base / buf;
        fs::create_directories(root_dir_);
        fs::create_directories(root_dir_ / "image");
        // 转成 std::string
        std::string timestamp(buf);
        // 拼接文件名
        std::filesystem::path pcFile = root_dir_ / ("MT" + timestamp + ".olx");
        // Create placeholder files
        write_text_file(root_dir_ / "image" / "info.txt", "device=OdinOne\npointcloud=xyzrgbi\ncreated_at=" + std::string(buf) + "\n");
        write_text_file(root_dir_ / "image" / "cam_in_ex.txt", "# camera intrinsics/extrinsics TBD\n");

        // Init writers
        pose_writer_ = std::make_unique<Writer>(root_dir_ / "OdinPose.bin", opts.batch_size);
        cloud_writer_ = std::make_unique<Writer>(pcFile, opts.batch_size);
        //cloud_writer_ = std::make_unique<Writer>(root_dir_ / "OdinPointCloud.olx", opts.batch_size);
        image_writer_ = std::make_unique<Writer>(root_dir_ / "OdinImage.bin", opts.batch_size);
        roatation_writer_ = std::make_unique<Writer>(root_dir_ / "OdinRotate.bin", opts.batch_size);
        imu_writer_ = std::make_unique<Writer>(root_dir_ / "OdinIMU.bin", opts.batch_size);
    }

    ~BinaryDataLogger() {
        // Ensure all writers flush on destruction
        if (pose_writer_) pose_writer_->shutdown();
        if (cloud_writer_) cloud_writer_->shutdown();
        if (image_writer_) image_writer_->shutdown();
        if (roatation_writer_) roatation_writer_->shutdown();
        if (imu_writer_) imu_writer_->shutdown();
    }

    const std::filesystem::path& root_dir() const { return root_dir_; }

    // Enqueue ready-to-write frame blobs (already formatted as per spec)
    void enqueuePoseFrame(std::vector<uint8_t>&& blob) {
        if (pose_writer_) pose_writer_->enqueue(std::move(blob));
    }
    void enqueuePointCloudFrame(std::vector<uint8_t>&& blob) {
        if (cloud_writer_) cloud_writer_->enqueue(std::move(blob));
    }
    void enqueueImageFrame(std::vector<uint8_t>&& blob) {
        if (image_writer_) image_writer_->enqueue(std::move(blob));
    }
    void enqueueRotateFrame(std::vector<uint8_t>&& blob) {
        if (roatation_writer_) roatation_writer_->enqueue(std::move(blob));
    }
    void enqueueIMUFrame(std::vector<uint8_t>&& blob) {
        if (imu_writer_) imu_writer_->enqueue(std::move(blob));
    }

private:
    struct Writer {
        explicit Writer(const std::filesystem::path& filepath, size_t batch)
            : file_path(filepath), batch_size(batch), stop(false) {
            worker = std::thread([this]() { this->run(); });
        }
        ~Writer() {
            shutdown();
        }
        void enqueue(std::vector<uint8_t>&& frame) {
            {
                std::lock_guard<std::mutex> lk(mtx);
                pending.emplace_back(std::move(frame));
                if (pending.size() >= batch_size) {
                    swap_pending_unlocked();
                }
            }
            cv.notify_one();
        }
        void shutdown() {
            {
                std::lock_guard<std::mutex> lk(mtx);
                if (stop) return;
                // Move leftovers to write buffer
                if (!pending.empty()) {
                    swap_pending_unlocked();
                }
                stop = true;
            }
            cv.notify_one();
            if (worker.joinable()) worker.join();
        }
    private:
        void swap_pending_unlocked() {
            if (!pending.empty()) {
                write_queue.emplace_back(std::move(pending));
                pending.clear();
            }
        }
        void run() {
            std::ofstream out(file_path, std::ios::binary | std::ios::app);
            if (!out.is_open()) {
                // If file can't be opened, silently drop (or could add logging hook)
                return;
            }
            for (;;) {
                std::vector<std::vector<uint8_t>> batch;
                {
                    std::unique_lock<std::mutex> lk(mtx);
                    cv.wait(lk, [&]{ return stop || !write_queue.empty(); });
                    if (!write_queue.empty()) {
                        batch = std::move(write_queue.front());
                        write_queue.pop_front();
                    } else if (stop) {
                        break;
                    }
                }
                if (!batch.empty()) {
                    for (auto& frame : batch) {
                        if (!frame.empty()) {
                            out.write(reinterpret_cast<const char*>(frame.data()), static_cast<std::streamsize>(frame.size()));
                        }
                    }
                    out.flush();
                }
            }
        }
        std::filesystem::path file_path;
        size_t batch_size;
        std::mutex mtx;
        std::condition_variable cv;
        std::vector<std::vector<uint8_t>> pending; // accumulate frames
        std::deque<std::vector<std::vector<uint8_t>>> write_queue; // queued batches
        std::thread worker;
        bool stop;
    };

    static void write_text_file(const std::filesystem::path& p, const std::string& content) {
        std::ofstream f(p, std::ios::out | std::ios::trunc);
        if (f.is_open()) {
            f << content;
        }
    }

    std::filesystem::path root_dir_;
    std::unique_ptr<Writer> pose_writer_;
    std::unique_ptr<Writer> cloud_writer_;
    std::unique_ptr<Writer> image_writer_;
    std::unique_ptr<Writer> roatation_writer_;
    std::unique_ptr<Writer> imu_writer_;
};
