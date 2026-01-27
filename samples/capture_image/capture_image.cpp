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
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#if defined(_WIN32)
#include <windows.h>
#endif

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <metavision/hal/facilities/i_ll_biases.h>
#include <metavision/sdk/core/algorithms/event_buffer_reslicer_algorithm.h>
#include <metavision/sdk/core/algorithms/on_demand_frame_generation_algorithm.h>
#include <metavision/sdk/stream/camera.h>
#include <metavision/sdk/ui/utils/event_loop.h>

namespace {

struct BiasControl {
    std::string name;
    int min_value = 0;
    int max_value = 255;
    int trackbar_value = 0;
    bool supported = false;
    bool modifiable = false;
    bool warned_unsupported = false;
};

struct AppState {
    std::unique_ptr<Metavision::Camera> camera;
    std::unique_ptr<Metavision::OnDemandFrameGenerationAlgorithm> frame_generator;
    std::unique_ptr<Metavision::EventBufferReslicerAlgorithm> reslicer;
    Metavision::I_LL_Biases *biases = nullptr;

    std::mutex frame_mutex;
    cv::Mat latest_frame;

    std::atomic<bool> desired_camera_on{false};
    std::atomic<bool> capture_requested{false};
    bool suppress_bias_callbacks = false;
};

struct BiasCallbackData {
    AppState *app = nullptr;
    BiasControl *control = nullptr;
};

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
        std::cout << "Bias \"" << bias_name << "\" clamped to " << clamped << " within range [" << range.first
                  << ", " << range.second << "]." << std::endl;
    }

    if (!biases->set(bias_name, clamped)) {
        std::cerr << "Failed to set bias \"" << bias_name << "\" to " << clamped << "." << std::endl;
        return false;
    }

    std::cout << "Applied bias \"" << bias_name << "\" = " << clamped << std::endl;
    return true;
}

std::string make_capture_filename() {
    auto now = std::chrono::system_clock::now();
    auto now_time = std::chrono::system_clock::to_time_t(now);
    std::tm local_tm{};
#if defined(_WIN32)
    localtime_s(&local_tm, &now_time);
#else
    localtime_r(&now_time, &local_tm);
#endif
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

    std::ostringstream oss;
    oss << "capture_" << std::put_time(&local_tm, "%Y%m%d_%H%M%S") << "_" << std::setw(3) << std::setfill('0')
        << ms.count() << ".bmp";
    return oss.str();
}

void on_bias_trackbar(int position, void *userdata) {
    auto *data = static_cast<BiasCallbackData *>(userdata);
    if (!data || !data->app || !data->control) {
        return;
    }
    auto &app = *data->app;
    auto &control = *data->control;
    if (app.suppress_bias_callbacks) {
        return;
    }
    if (!app.biases || !control.supported || !control.modifiable) {
        if (!control.warned_unsupported) {
            std::cout << "Bias \"" << control.name << "\" is not supported or modifiable on this camera." << std::endl;
            control.warned_unsupported = true;
        }
        return;
    }
    int actual_value = position + control.min_value;
    apply_single_bias(app.biases, control.name, actual_value);
}

void on_camera_toggle(int value, void *userdata) {
    auto *app = static_cast<AppState *>(userdata);
    if (!app) {
        return;
    }
    app->desired_camera_on.store(value != 0);
}

struct CaptureCallbackData {
    AppState *app = nullptr;
    std::string window_name;
};

void on_capture_toggle(int value, void *userdata) {
    auto *data = static_cast<CaptureCallbackData *>(userdata);
    if (!data || !data->app) {
        return;
    }
    if (value != 0) {
        data->app->capture_requested.store(true);
        cv::setTrackbarPos("Capture", data->window_name, 0);
    }
}

void update_bias_controls(AppState &app, std::vector<BiasControl> &controls, const std::string &window_name) {
    app.suppress_bias_callbacks = true;
    for (auto &control : controls) {
        control.supported = false;
        control.modifiable = false;
        control.warned_unsupported = false;
        if (!app.biases) {
            continue;
        }
        Metavision::LL_Bias_Info info;
        if (!app.biases->get_bias_info(control.name, info)) {
            std::cout << "Bias \"" << control.name << "\" is not available on this camera." << std::endl;
            continue;
        }
        control.supported = true;
        control.modifiable = info.is_modifiable();
        if (!control.modifiable) {
            std::cout << "Bias \"" << control.name << "\" is read-only and cannot be modified." << std::endl;
            continue;
        }
        auto range = bias_range_or_default(app.biases, control.name);
        control.min_value = range.first;
        control.max_value = range.second;
        int current = app.biases->get(control.name);
        control.trackbar_value = clamp_bias_value(current, range) - control.min_value;
        cv::setTrackbarMax(control.name, window_name, std::max(0, control.max_value - control.min_value));
        cv::setTrackbarPos(control.name, window_name, control.trackbar_value);
        std::cout << "Bias \"" << control.name << "\" initialized to " << current << " (range "
                  << control.min_value << ".." << control.max_value << ")." << std::endl;
    }
    app.suppress_bias_callbacks = false;
}

bool start_camera(AppState &app, std::vector<BiasControl> &controls, const std::string &controls_window) {
    try {
        app.camera = std::make_unique<Metavision::Camera>(Metavision::Camera::from_first_available());
    } catch (const Metavision::CameraException &e) {
        std::cerr << "Failed to open camera: " << e.what() << std::endl;
        return false;
    }

    const int width = app.camera->geometry().get_width();
    const int height = app.camera->geometry().get_height();
    app.latest_frame = cv::Mat(height, width, CV_8UC3, cv::Scalar(0, 0, 0));

    constexpr uint32_t kAccumulationTimeUs = 50000;
    constexpr int kSlicePeriodUs = 30000;

    app.frame_generator = std::make_unique<Metavision::OnDemandFrameGenerationAlgorithm>(
        width, height, kAccumulationTimeUs, Metavision::ColorPalette::Dark);

    app.reslicer = std::make_unique<Metavision::EventBufferReslicerAlgorithm>(
        nullptr, Metavision::EventBufferReslicerAlgorithm::Condition::make_n_us(kSlicePeriodUs));

    app.reslicer->set_on_new_slice_callback([&app](
                                               Metavision::EventBufferReslicerAlgorithm::ConditionStatus,
                                               Metavision::timestamp ts, std::size_t) {
        std::lock_guard<std::mutex> lock(app.frame_mutex);
        if (!app.frame_generator) {
            return;
        }
        app.frame_generator->generate(ts, app.latest_frame);
    });

    auto reslicer_ev_callback = [&app](const Metavision::EventCD *begin, const Metavision::EventCD *end) {
        if (app.frame_generator) {
            app.frame_generator->process_events(begin, end);
        }
    };

    app.camera->cd().add_callback([&app, reslicer_ev_callback](const Metavision::EventCD *begin,
                                                              const Metavision::EventCD *end) {
        if (app.reslicer) {
            app.reslicer->process_events(begin, end, reslicer_ev_callback);
        }
    });

    app.camera->start();
    std::cout << "Camera started." << std::endl;

    app.biases = app.camera->get_device().get_facility<Metavision::I_LL_Biases>();
    if (!app.biases) {
        std::cout << "This camera does not expose I_LL_Biases; bias controls will be ignored." << std::endl;
    }

    update_bias_controls(app, controls, controls_window);
    return true;
}

void stop_camera(AppState &app) {
    if (!app.camera) {
        return;
    }
    if (app.camera->is_running()) {
        app.camera->stop();
    }
    app.camera.reset();
    app.frame_generator.reset();
    app.reslicer.reset();
    app.biases = nullptr;
    std::cout << "Camera stopped." << std::endl;
}

void capture_image(AppState &app) {
    cv::Mat frame_copy;
    {
        std::lock_guard<std::mutex> lock(app.frame_mutex);
        if (app.latest_frame.empty()) {
            std::cout << "No frame available to capture." << std::endl;
            return;
        }
        frame_copy = app.latest_frame.clone();
    }

    std::filesystem::path capture_dir = std::filesystem::path(".") / "captures";
    std::error_code ec;
    std::filesystem::create_directories(capture_dir, ec);
    if (ec) {
        std::cerr << "Failed to create capture directory: " << capture_dir.string() << std::endl;
        return;
    }

    auto filename = make_capture_filename();
    auto output_path = capture_dir / filename;
    if (!cv::imwrite(output_path.string(), frame_copy)) {
        std::cerr << "Failed to save capture to " << output_path.string() << std::endl;
        return;
    }

    std::cout << "Captured image saved to: " << std::filesystem::absolute(output_path).string() << std::endl;
}

#if defined(_WIN32)
std::wstring get_executable_directory() {
    std::wstring buffer(MAX_PATH, L'\0');
    while (true) {
        DWORD size = static_cast<DWORD>(buffer.size());
        DWORD length = GetModuleFileNameW(nullptr, buffer.data(), size);
        if (length == 0) {
            return L"";
        }
        if (length < size - 1) {
            buffer.resize(length);
            break;
        }
        buffer.resize(buffer.size() * 2);
    }
    std::filesystem::path exe_path(buffer);
    return exe_path.parent_path().wstring();
}

void setup_windows_runtime_paths() {
    const std::wstring exe_dir = get_executable_directory();
    if (exe_dir.empty()) {
        return;
    }

    std::filesystem::path plugin_dir = std::filesystem::path(exe_dir) / L".." / L".." / L"lib" / L"metavision" /
                                       L"hal" / L"plugins";
    plugin_dir = plugin_dir.lexically_normal();

    const char *current_plugin_path = std::getenv("MV_HAL_PLUGIN_PATH");
    if (!current_plugin_path || std::string(current_plugin_path).empty()) {
        _wputenv_s(L"MV_HAL_PLUGIN_PATH", plugin_dir.wstring().c_str());
    }

    HMODULE kernel = GetModuleHandleW(L"kernel32.dll");
    if (kernel) {
        using SetDefaultDllDirectoriesFn = BOOL(WINAPI *)(DWORD);
        using AddDllDirectoryFn = DLL_DIRECTORY_COOKIE(WINAPI *)(PCWSTR);
        auto set_default =
            reinterpret_cast<SetDefaultDllDirectoriesFn>(GetProcAddress(kernel, "SetDefaultDllDirectories"));
        auto add_dir = reinterpret_cast<AddDllDirectoryFn>(GetProcAddress(kernel, "AddDllDirectory"));
        if (set_default && add_dir) {
            set_default(LOAD_LIBRARY_SEARCH_DEFAULT_DIRS | LOAD_LIBRARY_SEARCH_USER_DIRS);
            add_dir(exe_dir.c_str());
            add_dir(plugin_dir.wstring().c_str());
            return;
        }
    }

    SetDllDirectoryW(exe_dir.c_str());
}
#endif

} // namespace

int main() {
#if defined(_WIN32)
    setup_windows_runtime_paths();
#endif

    const auto available_sources = Metavision::Camera::list_online_sources();
    if (available_sources.empty()) {
        std::cerr << "No camera detected. Please connect a Prophesee camera and try again." << std::endl;
        return 1;
    }

    const std::string display_window = "Capture image";
    const std::string controls_window = "Capture image controls";

    cv::namedWindow(display_window, cv::WINDOW_NORMAL);
    cv::namedWindow(controls_window, cv::WINDOW_NORMAL);

    AppState app;

    int camera_toggle = 0;
    cv::createTrackbar("Camera", controls_window, &camera_toggle, 1, on_camera_toggle, &app);
    cv::setTrackbarPos("Camera", controls_window, 0);

    int capture_toggle = 0;
    CaptureCallbackData capture_data{&app, controls_window};
    cv::createTrackbar("Capture", controls_window, &capture_toggle, 1, on_capture_toggle, &capture_data);

    std::vector<BiasControl> bias_controls = {
        {"bias_diff"},
        {"bias_diff_off"},
        {"bias_diff_on"},
        {"bias_hpf"},
        {"bias_fo"},
    };

    std::vector<BiasCallbackData> bias_callbacks;
    bias_callbacks.reserve(bias_controls.size());
    for (auto &control : bias_controls) {
        bias_callbacks.push_back({&app, &control});
        cv::createTrackbar(control.name, controls_window, &control.trackbar_value, control.max_value,
                           on_bias_trackbar, &bias_callbacks.back());
    }

    bool running = true;
    while (running) {
        const bool want_on = app.desired_camera_on.load();
        if (want_on && !app.camera) {
            std::cout << "Starting camera..." << std::endl;
            if (!start_camera(app, bias_controls, controls_window)) {
                return 1;
            }
        } else if (!want_on && app.camera) {
            std::cout << "Stopping camera..." << std::endl;
            stop_camera(app);
        }

        if (app.capture_requested.exchange(false)) {
            capture_image(app);
        }

        if (!app.latest_frame.empty()) {
            std::lock_guard<std::mutex> lock(app.frame_mutex);
            cv::imshow(display_window, app.latest_frame);
        }

        int key = cv::waitKey(1);
        if ((key & 0xff) == 27 || (key & 0xff) == 'q') {
            running = false;
        }

        if (cv::getWindowProperty(display_window, cv::WND_PROP_VISIBLE) < 1) {
            running = false;
        }

        Metavision::EventLoop::poll_and_dispatch(1);
    }

    stop_camera(app);
    return 0;
}
