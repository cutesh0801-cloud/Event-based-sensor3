/**********************************************************************************************************************
 * Copyright (c) Prophesee S.A.                                                                                       *
 *                                                                                                                    *
 * Licensed under the Apache License, Version 2.0 (the "License");                                                    *
 * you may not use this file except in compliance with the License.                                                   *
 * You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0                                 *
 * Unless required by applicable law or agreed to in writing, software distributed under the License is distributed   *
 * on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.                      *
 * See the License for the specific language governing permissions and limitations under the License.                 *
 **********************************************************************************************************************/

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <csignal>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <metavision/sdk/base/events/event_cd.h>
#include <metavision/sdk/stream/camera.h>

namespace {
constexpr Metavision::timestamp kWindowUs = 2000;
constexpr std::size_t kMaxQueueSize = 200;
constexpr int kDisplayDelayMs = 1;
const char kWindowName[] = "EVS 2ms Accumulation";
std::atomic<bool> *g_running = nullptr;

struct ChunkQueue {
    std::mutex mutex;
    std::condition_variable cv;
    std::deque<std::vector<Metavision::EventCD>> queue;
};

std::string make_timestamped_output_dir() {
    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    std::tm local_tm{};
#if defined(_WIN32)
    localtime_s(&local_tm, &now_time);
#else
    localtime_r(&now_time, &local_tm);
#endif
    std::ostringstream oss;
    oss << "output/run_" << std::put_time(&local_tm, "%Y%m%d_%H%M%S");
    return oss.str();
}

void handle_command(char cmd,
                    std::unique_ptr<Metavision::Camera> &camera,
                    std::atomic<bool> &camera_on,
                    std::atomic<bool> &recording_enabled,
                    std::atomic<bool> &running,
                    std::atomic<bool> &reset_requested,
                    std::atomic<bool> &recording_reset_requested,
                    std::mutex &output_mutex,
                    std::string &output_dir,
                    std::atomic<int> &camera_width,
                    std::atomic<int> &camera_height,
                    ChunkQueue &chunk_queue) {
    switch (cmd) {
    case 'o':
    case 'O': {
        if (camera_on.load()) {
            std::cout << "Camera already ON." << std::endl;
            return;
        }
        try {
            camera = std::make_unique<Metavision::Camera>(Metavision::Camera::from_first_available());
        } catch (const std::exception &e) {
            std::cerr << "Failed to open camera: " << e.what() << std::endl;
            return;
        }
        camera_width.store(camera->geometry().get_width());
        camera_height.store(camera->geometry().get_height());
        camera_on.store(true);
        reset_requested.store(true);
        camera->cd().add_callback([&chunk_queue, &camera_on, &running](const Metavision::EventCD *begin,
                                                                       const Metavision::EventCD *end) {
            if (!camera_on.load() || !running.load()) {
                return;
            }
            std::vector<Metavision::EventCD> chunk(begin, end);
            if (chunk.empty()) {
                return;
            }
            std::unique_lock<std::mutex> lock(chunk_queue.mutex);
            chunk_queue.cv.wait(lock, [&]() {
                return !running.load() || !camera_on.load() || chunk_queue.queue.size() < kMaxQueueSize;
            });
            if (!running.load() || !camera_on.load()) {
                return;
            }
            chunk_queue.queue.push_back(std::move(chunk));
            lock.unlock();
            chunk_queue.cv.notify_one();
        });
        camera->start();
        std::cout << "Camera ON. Resolution: " << camera_width.load() << "x" << camera_height.load() << std::endl;
        return;
    }
    case 'f':
    case 'F': {
        if (!camera_on.load()) {
            std::cout << "Camera already OFF." << std::endl;
            return;
        }
        camera_on.store(false);
        if (camera) {
            camera->stop();
            camera.reset();
        }
        {
            std::lock_guard<std::mutex> lock(chunk_queue.mutex);
            chunk_queue.queue.clear();
        }
        chunk_queue.cv.notify_all();
        reset_requested.store(true);
        std::cout << "Camera OFF." << std::endl;
        return;
    }
    case 's':
    case 'S': {
        if (recording_enabled.load()) {
            std::cout << "Recording already ON." << std::endl;
            return;
        }
        std::string new_dir = make_timestamped_output_dir();
        try {
            std::filesystem::create_directories(new_dir);
        } catch (const std::exception &e) {
            std::cerr << "Failed to create output directory: " << e.what() << std::endl;
            return;
        }
        {
            std::lock_guard<std::mutex> lock(output_mutex);
            output_dir = std::move(new_dir);
        }
        recording_reset_requested.store(true);
        recording_enabled.store(true);
        std::cout << "Recording ON. Output dir: " << output_dir << std::endl;
        return;
    }
    case 'e':
    case 'E': {
        if (!recording_enabled.load()) {
            std::cout << "Recording already OFF." << std::endl;
            return;
        }
        recording_enabled.store(false);
        std::cout << "Recording OFF." << std::endl;
        return;
    }
    case 'q':
    case 'Q': {
        running.store(false);
        chunk_queue.cv.notify_all();
        std::cout << "Exit requested." << std::endl;
        return;
    }
    default:
        return;
    }
}

} // namespace

int main() {
    std::atomic<bool> running{true};
    std::atomic<bool> camera_on{false};
    std::atomic<bool> recording_enabled{false};
    std::atomic<bool> reset_requested{false};
    std::atomic<bool> recording_reset_requested{false};
    std::atomic<int> camera_width{0};
    std::atomic<int> camera_height{0};

    std::mutex output_mutex;
    std::string output_dir;

    ChunkQueue chunk_queue;

    std::mutex frame_mutex;
    cv::Mat latest_frame;

    g_running = &running;
    std::signal(SIGINT, [](int) {
        if (g_running) {
            g_running->store(false);
        }
    });

    std::unique_ptr<Metavision::Camera> camera;

    std::thread consumer_thread([&]() {
        std::optional<Metavision::timestamp> window_start;
        Metavision::timestamp window_end = 0;
        std::vector<Metavision::EventCD> window_events;
        cv::Mat current_frame;
        std::size_t frame_index = 0;
        std::size_t recording_frame_index = 0;
        bool window_recording = false;

        while (running.load()) {
            if (reset_requested.exchange(false)) {
                window_start.reset();
                window_events.clear();
                current_frame.release();
                frame_index = 0;
                recording_frame_index = 0;
            }
            if (recording_reset_requested.exchange(false)) {
                recording_frame_index = 0;
            }

            std::vector<Metavision::EventCD> chunk;
            {
                std::unique_lock<std::mutex> lock(chunk_queue.mutex);
                chunk_queue.cv.wait(lock, [&]() { return !running.load() || !chunk_queue.queue.empty(); });
                if (!running.load()) {
                    break;
                }
                if (chunk_queue.queue.empty()) {
                    continue;
                }
                chunk = std::move(chunk_queue.queue.front());
                chunk_queue.queue.pop_front();
            }
            chunk_queue.cv.notify_one();

            if (chunk.empty()) {
                continue;
            }

            for (const auto &ev : chunk) {
                if (!window_start) {
                    int width = camera_width.load();
                    int height = camera_height.load();
                    if (width <= 0 || height <= 0) {
                        continue;
                    }
                    current_frame = cv::Mat(height, width, CV_8UC1, cv::Scalar(0));
                    window_events.clear();
                    window_start = ev.t;
                    window_end = *window_start + kWindowUs;
                    window_recording = recording_enabled.load();
                }

                while (ev.t >= window_end) {
                    if (window_start) {
                        if (window_recording) {
                            std::string dir_copy;
                            {
                                std::lock_guard<std::mutex> lock(output_mutex);
                                dir_copy = output_dir;
                            }
                            if (!dir_copy.empty()) {
                                ++recording_frame_index;
                                std::ostringstream filename;
                                filename << dir_copy << "/frame_" << std::setw(6) << std::setfill('0')
                                         << recording_frame_index << "_t0_" << *window_start << "us.txt";
                                std::ofstream out(filename.str());
                                for (const auto &evt : window_events) {
                                    out << evt.x << " " << evt.y << "\n";
                                }
                            }
                        }

                        {
                            std::lock_guard<std::mutex> lock(frame_mutex);
                            latest_frame = current_frame.clone();
                        }

                        std::size_t queue_size = 0;
                        {
                            std::lock_guard<std::mutex> lock(chunk_queue.mutex);
                            queue_size = chunk_queue.queue.size();
                        }

                        std::cout << "Frame " << frame_index << " t0=" << *window_start << "us | queue="
                                  << queue_size << " | recording=" << (window_recording ? "ON" : "OFF")
                                  << std::endl;
                    }

                    ++frame_index;
                    window_start = window_end;
                    window_end = *window_start + kWindowUs;
                    current_frame.setTo(0);
                    window_events.clear();
                    window_recording = recording_enabled.load();
                }

                if (ev.x < current_frame.cols && ev.y < current_frame.rows) {
                    current_frame.at<std::uint8_t>(ev.y, ev.x) = 255;
                }
                if (window_recording) {
                    window_events.push_back(ev);
                }
            }
        }
    });

    cv::namedWindow(kWindowName, cv::WINDOW_NORMAL);
    std::cout << "Commands: o(Camera ON), f(Camera OFF), s(Record START), e(Record END), q(Quit)" << std::endl;

    while (running.load()) {
        cv::Mat frame_copy;
        {
            std::lock_guard<std::mutex> lock(frame_mutex);
            if (!latest_frame.empty()) {
                frame_copy = latest_frame.clone();
            }
        }
        if (!frame_copy.empty()) {
            cv::imshow(kWindowName, frame_copy);
        }

        int key = cv::waitKey(kDisplayDelayMs);
        if (key == 'q' || key == 'Q') {
            handle_command('q', camera, camera_on, recording_enabled, running, reset_requested,
                           recording_reset_requested, output_mutex, output_dir, camera_width, camera_height,
                           chunk_queue);
        } else if (key > 0) {
            handle_command(static_cast<char>(key), camera, camera_on, recording_enabled, running, reset_requested,
                           recording_reset_requested, output_mutex, output_dir, camera_width, camera_height,
                           chunk_queue);
        }

        if (std::cin.rdbuf()->in_avail() > 0) {
            char cmd = static_cast<char>(std::cin.get());
            if (cmd == '\n' || cmd == '\r') {
                continue;
            }
            handle_command(cmd, camera, camera_on, recording_enabled, running, reset_requested,
                           recording_reset_requested, output_mutex, output_dir, camera_width, camera_height,
                           chunk_queue);
        }
    }

    if (camera_on.load() && camera) {
        camera->stop();
        camera.reset();
    }

    chunk_queue.cv.notify_all();
    if (consumer_thread.joinable()) {
        consumer_thread.join();
    }

    cv::destroyAllWindows();

    return 0;
}