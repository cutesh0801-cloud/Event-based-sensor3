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
#include <utility>
#include <vector>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

#if defined(_WIN32)
#include <conio.h>
#endif

#include <metavision/hal/facilities/i_ll_biases.h>
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

struct BiasCliOptions {
    std::optional<int> bias_diff;
    std::optional<int> bias_diff_on;
    std::optional<int> bias_diff_off;
    std::optional<int> bias_fo;
    std::optional<int> bias_hpf;
    bool print_bias_on_open = false;

    bool has_bias_values() const {
        return bias_diff || bias_diff_on || bias_diff_off || bias_fo || bias_hpf;
    }
};

void print_usage(const char *app_name) {
    std::cout << "Usage: " << app_name << " [options]\n"
              << "Options:\n"
              << "  --bias-diff <int>       Set bias_diff before starting the camera\n"
              << "  --bias-diff-on <int>    Set bias_diff_on before starting the camera\n"
              << "  --bias-diff-off <int>   Set bias_diff_off before starting the camera\n"
              << "  --bias-fo <int>         Set bias_fo before starting the camera\n"
              << "  --bias-hpf <int>        Set bias_hpf before starting the camera\n"
              << "  --print-bias            Print current bias values when the camera is opened\n"
              << "  --help                  Show this help message\n";
}

bool parse_int_arg(const std::string &arg, int &value) {
    try {
        std::size_t idx = 0;
        value = std::stoi(arg, &idx, 10);
        return idx == arg.size();
    } catch (const std::exception &) {
        return false;
    }
}

bool parse_cli_options(int argc, char **argv, BiasCliOptions &options, bool &show_help) {
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--help") {
            print_usage(argv[0]);
            show_help = true;
            return false;
        }
        auto parse_option_with_value = [&](std::optional<int> &target) -> bool {
            if (i + 1 >= argc) {
                std::cerr << "Missing value for " << arg << std::endl;
                return false;
            }
            int value = 0;
            if (!parse_int_arg(argv[i + 1], value)) {
                std::cerr << "Invalid integer for " << arg << ": " << argv[i + 1] << std::endl;
                return false;
            }
            target = value;
            ++i;
            return true;
        };

        if (arg == "--bias-diff") {
            if (!parse_option_with_value(options.bias_diff)) {
                return false;
            }
        } else if (arg == "--bias-diff-on") {
            if (!parse_option_with_value(options.bias_diff_on)) {
                return false;
            }
        } else if (arg == "--bias-diff-off") {
            if (!parse_option_with_value(options.bias_diff_off)) {
                return false;
            }
        } else if (arg == "--bias-fo") {
            if (!parse_option_with_value(options.bias_fo)) {
                return false;
            }
        } else if (arg == "--bias-hpf") {
            if (!parse_option_with_value(options.bias_hpf)) {
                return false;
            }
        } else if (arg == "--print-bias") {
            options.print_bias_on_open = true;
        } else {
            std::cerr << "Unknown option: " << arg << std::endl;
            return false;
        }
    }
    return true;
}

std::optional<char> poll_console_command() {
#if defined(_WIN32)
    if (_kbhit()) {
        int ch = _getch();
        if (ch == 0 || ch == 224) {
            if (_kbhit()) {
                static_cast<void>(_getch());
            }
            return std::nullopt;
        }
        return static_cast<char>(ch);
    }
    return std::nullopt;
#else
    if (std::cin.rdbuf()->in_avail() > 0) {
        char cmd = static_cast<char>(std::cin.get());
        if (cmd == '\n' || cmd == '\r') {
            return std::nullopt;
        }
        return cmd;
    }
    return std::nullopt;
#endif
}

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

void print_bias_values(Metavision::I_LL_Biases *biases) {
    if (!biases) {
        std::cout << "이 디바이스는 bias 조절 미지원 (I_LL_Biases facility unavailable)." << std::endl;
        return;
    }

    auto all_biases = biases->get_all_biases();
    if (all_biases.empty()) {
        std::cout << "No biases reported by the camera." << std::endl;
        return;
    }

    std::cout << "Current biases:" << std::endl;
    for (const auto &entry : all_biases) {
        std::cout << "  - " << entry.first << " = " << entry.second << std::endl;
    }
}

bool camera_biases_ready(const std::atomic<bool> &camera_on, Metavision::I_LL_Biases *biases) {
    if (!camera_on.load()) {
        std::cout << "Camera must be ON before using bias commands." << std::endl;
        return false;
    }
    if (!biases) {
        std::cout << "이 디바이스는 bias 조절 미지원 (I_LL_Biases facility unavailable)." << std::endl;
        return false;
    }
    return true;
}

std::string trim_bias_name(const std::string &name) {
    auto start = name.find_first_not_of(" \t\r\n");
    if (start == std::string::npos) {
        return {};
    }
    auto end = name.find_last_not_of(" \t\r\n");
    return name.substr(start, end - start + 1);
}

std::pair<int, int> bias_range_or_default(Metavision::I_LL_Biases *biases, const std::string &bias_name) {
    if (!biases) {
        return {0, 255};
    }
    Metavision::LL_Bias_Info info;
    if (biases->get_bias_info(bias_name, info)) {
        auto range = info.get_bias_range();
        if (range.first > range.second) {
            std::swap(range.first, range.second);
        }
        return range;
    }
    return {0, 255};
}

int clamp_bias_value(int value, const std::pair<int, int> &range) {
    if (value < range.first) {
        return range.first;
    }
    if (value > range.second) {
        return range.second;
    }
    return value;
}

void list_biases(Metavision::I_LL_Biases *biases,
                 bool verbose,
                 std::string &selected_bias,
                 int step,
                 const std::vector<int> &step_options) {
    if (!biases) {
        std::cout << "이 디바이스는 bias 조절 미지원 (I_LL_Biases facility unavailable)." << std::endl;
        return;
    }

    auto all_biases = biases->get_all_biases();
    if (all_biases.empty()) {
        std::cout << "No biases reported by the camera." << std::endl;
        return;
    }

    std::cout << "Available biases:" << std::endl;
    for (const auto &entry : all_biases) {
        const auto &name = entry.first;
        int value = entry.second;
        std::cout << "  - " << name << " = " << value;
        if (verbose) {
            Metavision::LL_Bias_Info info;
            if (biases->get_bias_info(name, info)) {
                auto recommended = info.get_bias_range();
                auto allowed = info.get_bias_allowed_range();
                auto desc = info.get_description();
                auto category = info.get_category();
                std::cout << " | range=" << recommended.first << ".." << recommended.second;
                if (recommended != allowed) {
                    std::cout << " (allowed " << allowed.first << ".." << allowed.second << ")";
                }
                if (!desc.empty()) {
                    std::cout << " | desc=" << desc;
                }
                if (!category.empty()) {
                    std::cout << " | category=" << category;
                }
                std::cout << " | modifiable=" << (info.is_modifiable() ? "yes" : "no");
            } else {
                std::cout << " | info=unavailable";
            }
        }
        std::cout << std::endl;
    }

    if (selected_bias.empty()) {
        selected_bias = all_biases.begin()->first;
    }

    std::cout << "Selected bias: " << selected_bias << " | step=" << step << " (";
    for (std::size_t i = 0; i < step_options.size(); ++i) {
        std::cout << step_options[i];
        if (i + 1 < step_options.size()) {
            std::cout << "/";
        }
    }
    std::cout << ")" << std::endl;
}

bool apply_single_bias(Metavision::I_LL_Biases *biases, const std::string &bias_name, int value) {
    if (!biases) {
        std::cerr << "Bias facility not available; cannot set " << bias_name << "." << std::endl;
        return false;
    }

    Metavision::LL_Bias_Info info;
    if (!biases->get_bias_info(bias_name, info)) {
        std::cerr << "Bias \"" << bias_name << "\" is not available on this camera." << std::endl;
        return false;
    }
    if (!info.is_modifiable()) {
        std::cerr << "Bias \"" << bias_name << "\" is read-only and cannot be modified." << std::endl;
        return false;
    }

    auto range = bias_range_or_default(biases, bias_name);
    int clamped = clamp_bias_value(value, range);
    if (clamped != value) {
        std::cout << "Bias \"" << bias_name << "\" clamped to " << clamped << " within range [" << range.first << ", "
                  << range.second << "]." << std::endl;
    }

    if (!biases->set(bias_name, clamped)) {
        std::cerr << "Failed to set bias \"" << bias_name << "\" to " << clamped << "." << std::endl;
        return false;
    }

    std::cout << "Applied bias \"" << bias_name << "\" = " << clamped << std::endl;
    return true;
}

void apply_bias_settings(Metavision::I_LL_Biases *biases, const BiasCliOptions &options) {
    if (!options.has_bias_values()) {
        return;
    }

    const std::pair<std::string, std::optional<int>> bias_entries[] = {
        {"bias_diff", options.bias_diff},
        {"bias_diff_on", options.bias_diff_on},
        {"bias_diff_off", options.bias_diff_off},
        {"bias_fo", options.bias_fo},
        {"bias_hpf", options.bias_hpf},
    };

    for (const auto &entry : bias_entries) {
        if (!entry.second) {
            continue;
        }
        try {
            apply_single_bias(biases, entry.first, *entry.second);
        } catch (const std::exception &e) {
            std::cerr << "Exception while setting bias \"" << entry.first << "\": " << e.what() << std::endl;
        }
    }
}

void adjust_bias(Metavision::I_LL_Biases *biases, const std::string &bias_name, int delta) {
    if (bias_name.empty()) {
        std::cout << "No bias selected. Use 'n' to set a bias name." << std::endl;
        return;
    }

    auto all_biases = biases->get_all_biases();
    auto it = all_biases.find(bias_name);
    if (it == all_biases.end()) {
        std::cout << "Bias \"" << bias_name << "\" is not available on this camera." << std::endl;
        return;
    }

    int current_value = it->second;
    int requested_value = current_value + delta;

    Metavision::LL_Bias_Info info;
    bool has_info = biases->get_bias_info(bias_name, info);
    if (has_info && !info.is_modifiable()) {
        std::cout << "Bias \"" << bias_name << "\" is read-only and cannot be modified." << std::endl;
        return;
    }

    try {
        auto range = bias_range_or_default(biases, bias_name);
        int clamped_value = clamp_bias_value(requested_value, range);
        if (!biases->set(bias_name, clamped_value)) {
            std::cout << "Failed to update bias \"" << bias_name << "\" to " << clamped_value << "." << std::endl;
            return;
        }
        int updated_value = biases->get(bias_name);
        std::cout << "Bias \"" << bias_name << "\" updated: " << current_value << " -> " << updated_value;
        if (clamped_value != requested_value) {
            std::cout << " (requested " << requested_value << ", clamped to " << clamped_value << ")";
        }
        std::cout << std::endl;
    } catch (const std::exception &e) {
        std::cout << "Failed to update bias \"" << bias_name << "\": " << e.what() << std::endl;
    }
}

void print_selected_bias(Metavision::I_LL_Biases *biases, const std::string &bias_name, int step) {
    if (!biases) {
        std::cout << "이 디바이스는 bias 조절 미지원 (I_LL_Biases facility unavailable)." << std::endl;
        return;
    }
    if (bias_name.empty()) {
        std::cout << "No bias selected. Use 'n' to set a bias name." << std::endl;
        return;
    }
    auto all_biases = biases->get_all_biases();
    auto it = all_biases.find(bias_name);
    if (it == all_biases.end()) {
        std::cout << "Bias \"" << bias_name << "\" is not available on this camera." << std::endl;
        return;
    }
    std::cout << "Selected bias: " << bias_name << " = " << it->second << " | step=" << step << std::endl;
}

void prompt_bias_name(Metavision::I_LL_Biases *biases, std::string &selected_bias) {
    if (!biases) {
        std::cout << "이 디바이스는 bias 조절 미지원 (I_LL_Biases facility unavailable)." << std::endl;
        return;
    }
    std::cout << "Enter bias name: " << std::flush;
    std::string input;
    if (!std::getline(std::cin, input)) {
        std::cin.clear();
        return;
    }
    input = trim_bias_name(input);
    if (input.empty()) {
        std::cout << "Bias name not changed (empty input)." << std::endl;
        return;
    }
    auto all_biases = biases->get_all_biases();
    if (all_biases.find(input) == all_biases.end()) {
        std::cout << "Bias \"" << input << "\" not found. Use 'b' to list available biases." << std::endl;
        return;
    }
    selected_bias = input;
    std::cout << "Selected bias set to \"" << selected_bias << "\"." << std::endl;
}

void adjust_step(int direction, int &step_index, const std::vector<int> &step_options) {
    if (step_options.empty()) {
        return;
    }
    int next = step_index + direction;
    if (next < 0) {
        next = 0;
    } else if (next >= static_cast<int>(step_options.size())) {
        next = static_cast<int>(step_options.size()) - 1;
    }
    step_index = next;
    std::cout << "Bias step set to " << step_options[step_index] << std::endl;
}

void handle_command(char cmd,
                    std::unique_ptr<Metavision::Camera> &camera,
                    Metavision::I_LL_Biases *&biases,
                    std::atomic<bool> &camera_on,
                    std::atomic<bool> &recording_enabled,
                    std::atomic<bool> &running,
                    std::atomic<bool> &reset_requested,
                    std::atomic<bool> &recording_reset_requested,
                    std::mutex &output_mutex,
                    std::string &output_dir,
                    std::atomic<int> &camera_width,
                    std::atomic<int> &camera_height,
                    std::string &selected_bias,
                    int &bias_step_index,
                    const std::vector<int> &step_options,
                    ChunkQueue &chunk_queue,
                    BiasCliOptions &bias_options) {
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
        biases = camera->get_device().get_facility<Metavision::I_LL_Biases>();
        selected_bias.clear();
        if (!biases) {
            std::cout << "이 디바이스는 bias 조절 미지원 (I_LL_Biases facility unavailable)." << std::endl;
        }
        apply_bias_settings(biases, bias_options);
        if (bias_options.print_bias_on_open) {
            print_bias_values(biases);
            bias_options.print_bias_on_open = false;
        }
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
        biases = nullptr;
        selected_bias.clear();
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
    case 'b':
    case 'B': {
        if (!camera_biases_ready(camera_on, biases)) {
            return;
        }
        list_biases(biases, cmd == 'B', selected_bias, step_options[bias_step_index], step_options);
        return;
    }
    case 'n':
    case 'N': {
        if (!camera_biases_ready(camera_on, biases)) {
            return;
        }
        prompt_bias_name(biases, selected_bias);
        return;
    }
    case '+': {
        if (!camera_biases_ready(camera_on, biases)) {
            return;
        }
        adjust_bias(biases, selected_bias, step_options[bias_step_index]);
        return;
    }
    case '-': {
        if (!camera_biases_ready(camera_on, biases)) {
            return;
        }
        adjust_bias(biases, selected_bias, -step_options[bias_step_index]);
        return;
    }
    case ']': {
        adjust_step(1, bias_step_index, step_options);
        return;
    }
    case '[': {
        adjust_step(-1, bias_step_index, step_options);
        return;
    }
    case 'p':
    case 'P': {
        if (!camera_biases_ready(camera_on, biases)) {
            return;
        }
        print_selected_bias(biases, selected_bias, step_options[bias_step_index]);
        return;
    }
    default:
        return;
    }
}

} // namespace

int main(int argc, char **argv) {
    BiasCliOptions bias_options;
    bool show_help = false;
    if (!parse_cli_options(argc, argv, bias_options, show_help)) {
        if (show_help) {
            return 0;
        }
        if (argc > 1) {
            print_usage(argv[0]);
            return 1;
        }
    }

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
    Metavision::I_LL_Biases *biases = nullptr;
    std::string selected_bias;
    int bias_step_index = 0;
    std::vector<int> step_options = {1, 5, 10, 20, 50};

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
    std::cout
        << "Commands: o(Camera ON), f(Camera OFF), s(Record START), e(Record END), "
           "b(List biases), B(Verbose bias info), n(Select bias), +/-(Bias +/-), [ ](Step), p(Print selection), "
           "q(Quit)"
        << std::endl;

    if (bias_options.has_bias_values() || bias_options.print_bias_on_open) {
        handle_command('o', camera, biases, camera_on, recording_enabled, running, reset_requested,
                       recording_reset_requested, output_mutex, output_dir, camera_width, camera_height, selected_bias,
                       bias_step_index, step_options, chunk_queue, bias_options);
    }

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
            handle_command('q', camera, biases, camera_on, recording_enabled, running, reset_requested,
                           recording_reset_requested, output_mutex, output_dir, camera_width, camera_height,
                           selected_bias, bias_step_index, step_options, chunk_queue, bias_options);
        } else if (key > 0) {
            handle_command(static_cast<char>(key), camera, biases, camera_on, recording_enabled, running,
                           reset_requested, recording_reset_requested, output_mutex, output_dir, camera_width,
                           camera_height, selected_bias, bias_step_index, step_options, chunk_queue, bias_options);
        }

        if (auto cmd = poll_console_command()) {
            handle_command(*cmd, camera, biases, camera_on, recording_enabled, running, reset_requested,
                           recording_reset_requested, output_mutex, output_dir, camera_width, camera_height,
                           selected_bias, bias_step_index, step_options, chunk_queue, bias_options);
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
