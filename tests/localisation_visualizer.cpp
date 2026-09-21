/*
 * Live RPLidar MCL particle-cloud visualizer.
 *
 * Build from the project root:
 *   g++ -std=c++11 -O2 -o tests/localisation_visualizer \
 *     tests/localisation_visualizer.cpp lib/localisation.cpp \
 *     -I./lib -I./rplidar_sdk/sdk/include -I./rplidar_sdk/sdk/src \
 *     -L./rplidar_sdk/output/Linux/Release \
 *     -lsl_lidar_sdk -lSDL2 -lpthread -lrt
 *
 * Run:
 *   ./tests/localisation_visualizer [--port /dev/ttyUSB0] [--yaw 0]
 *
 * The laptop visualizer assumes the LIDAR is stationary unless odometry is
 * added. --yaw supplies a startup-relative absolute yaw prior, equivalent to
 * the robot's IMU yaw. R resets global localization; Esc/Q quits.
 */

#include <SDL2/SDL.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <signal.h>
#include <string>
#include <vector>

#include "lidar_config.h"
#include "localisation.h"
#include "sl_lidar_driver.h"

constexpr bool USE_PCB = false;

#ifndef _countof
#define _countof(array) (int)(sizeof(array) / sizeof((array)[0]))
#endif

using namespace sl;

static constexpr float PITCH_WIDTH_MM = 2430.0f;
static constexpr float PITCH_HEIGHT_MM = 1820.0f;
static constexpr float MIN_RANGE_MM = 80.0f;
static constexpr float MAX_RANGE_MM = 6000.0f;
static constexpr int MIN_QUALITY = 5;
static constexpr int WINDOW_WIDTH = 1100;
static constexpr int WINDOW_HEIGHT = 820;
static constexpr int MARGIN = 55;

static volatile sig_atomic_t g_stop_requested = 0;

static void handle_signal(int) {
    g_stop_requested = 1;
}

static double monotonic_time_s() {
    return std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

struct ViewTransform {
    float scale;
    float offset_x;
    float offset_y;

    ViewTransform() {
        const float available_x = WINDOW_WIDTH - 2.0f * MARGIN;
        const float available_y = WINDOW_HEIGHT - 2.0f * MARGIN;
        scale = std::min(
            available_x / PITCH_WIDTH_MM,
            available_y / PITCH_HEIGHT_MM);
        offset_x = 0.5f * (WINDOW_WIDTH - PITCH_WIDTH_MM * scale);
        offset_y = 0.5f * (WINDOW_HEIGHT - PITCH_HEIGHT_MM * scale);
    }

    SDL_Point point(float x, float y) const {
        return {
            (int)std::lround(offset_x + x * scale),
            (int)std::lround(offset_y + y * scale)
        };
    }
};

static void draw_filled_circle(
    SDL_Renderer* renderer, int cx, int cy, int radius
) {
    for (int y = -radius; y <= radius; ++y) {
        const int half_width =
            (int)std::sqrt((float)(radius * radius - y * y));
        SDL_RenderDrawLine(
            renderer, cx - half_width, cy + y, cx + half_width, cy + y);
    }
}

static void draw_world_line(
    SDL_Renderer* renderer, const ViewTransform& view,
    float x1, float y1, float x2, float y2
) {
    const SDL_Point a = view.point(x1, y1);
    const SDL_Point b = view.point(x2, y2);
    SDL_RenderDrawLine(renderer, a.x, a.y, b.x, b.y);
}

static void draw_pitch(SDL_Renderer* renderer, const ViewTransform& view) {
    SDL_SetRenderDrawColor(renderer, 225, 225, 235, 255);
    draw_world_line(renderer, view, 0, 0, PITCH_WIDTH_MM, 0);
    draw_world_line(
        renderer, view, PITCH_WIDTH_MM, 0,
        PITCH_WIDTH_MM, PITCH_HEIGHT_MM);
    draw_world_line(
        renderer, view, PITCH_WIDTH_MM, PITCH_HEIGHT_MM,
        0, PITCH_HEIGHT_MM);
    draw_world_line(renderer, view, 0, PITCH_HEIGHT_MM, 0, 0);

    SDL_SetRenderDrawColor(renderer, 100, 110, 130, 255);
    draw_world_line(
        renderer, view, PITCH_WIDTH_MM * 0.5f, 0,
        PITCH_WIDTH_MM * 0.5f, PITCH_HEIGHT_MM);

    // These are the same physical goal segments used by localisation.cpp.
    SDL_SetRenderDrawColor(renderer, 255, 190, 50, 255);
    draw_world_line(renderer, view, 300, 685, 0, 685);
    draw_world_line(renderer, view, 226, 685, 226, 1140);
    draw_world_line(renderer, view, 0, 1135, 300, 1135);

    SDL_SetRenderDrawColor(renderer, 50, 210, 235, 255);
    draw_world_line(renderer, view, 2130, 685, PITCH_WIDTH_MM, 685);
    draw_world_line(renderer, view, 2204, 685, 2204, 1140);
    draw_world_line(
        renderer, view, PITCH_WIDTH_MM, 1135, 2130, 1135);
}

static void draw_particles(
    SDL_Renderer* renderer, const ViewTransform& view,
    const std::vector<LocParticle>& particles
) {
    float max_weight = 0.0f;
    for (const auto& particle : particles) {
        max_weight = std::max(max_weight, particle.weight);
    }

    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);
    for (const auto& particle : particles) {
        const float relative_weight =
            max_weight > 0.0f ? particle.weight / max_weight : 0.0f;
        const Uint8 alpha =
            (Uint8)std::lround(35.0f + 190.0f * std::sqrt(relative_weight));
        SDL_SetRenderDrawColor(renderer, 180, 90, 255, alpha);
        const SDL_Point p = view.point(particle.x, particle.y);
        SDL_RenderDrawPoint(renderer, p.x, p.y);

        // A faint heading tick makes multimodal yaw hypotheses visible.
        const float yaw_rad = particle.yaw_deg * (float)M_PI / 180.0f;
        const int hx = p.x + (int)std::lround(3.0f * std::cos(yaw_rad));
        const int hy = p.y + (int)std::lround(3.0f * std::sin(yaw_rad));
        SDL_RenderDrawLine(renderer, p.x, p.y, hx, hy);
    }
}

static void draw_scan(
    SDL_Renderer* renderer, const ViewTransform& view,
    const std::vector<LocScanPoint>& scan, const LocPose& pose
) {
    if (!pose.ok) {
        return;
    }

    SDL_SetRenderDrawColor(renderer, 30, 230, 130, 150);
    const float yaw_rad = pose.yaw_deg * (float)M_PI / 180.0f;
    for (const auto& reading : scan) {
        if (!reading.hit) {
            continue;
        }
        const float angle =
            yaw_rad + reading.angle_deg * (float)M_PI / 180.0f;
        const float x = pose.x + reading.distance_mm * std::cos(angle);
        const float y = pose.y + reading.distance_mm * std::sin(angle);
        const SDL_Point p = view.point(x, y);
        SDL_RenderDrawPoint(renderer, p.x, p.y);
    }
}

static void draw_pose(
    SDL_Renderer* renderer, const ViewTransform& view, const LocPose& pose
) {
    if (!pose.ok) {
        return;
    }

    const SDL_Point center = view.point(pose.x, pose.y);
    SDL_SetRenderDrawColor(renderer, 255, 70, 70, 255);
    draw_filled_circle(renderer, center.x, center.y, 6);

    const float yaw_rad = pose.yaw_deg * (float)M_PI / 180.0f;
    const SDL_Point heading = view.point(
        pose.x + 130.0f * std::cos(yaw_rad),
        pose.y + 130.0f * std::sin(yaw_rad));
    SDL_RenderDrawLine(
        renderer, center.x, center.y, heading.x, heading.y);
}

static void render(
    SDL_Window* window, SDL_Renderer* renderer,
    const std::vector<LocScanPoint>& scan
) {
    const ViewTransform view;
    const LocPose pose = loc_get_pose();
    const LocRecoveryStatus recovery = loc_get_recovery_status();
    const std::vector<LocParticle> particles = loc_get_particles();

    SDL_SetRenderDrawColor(renderer, 17, 20, 31, 255);
    SDL_RenderClear(renderer);
    draw_pitch(renderer, view);
    draw_particles(renderer, view, particles);
    draw_scan(renderer, view, scan, pose);
    draw_pose(renderer, view, pose);
    SDL_RenderPresent(renderer);

    char title[256];
    if (pose.ok) {
        std::snprintf(
            title, sizeof(title),
            "MCL | pose=(%.0f, %.0f) yaw=%.1f conf=%.2f "
            "quality=%.2f/%.2f bad=%d global=%.0f%%",
            pose.x, pose.y, pose.yaw_deg, pose.confidence,
            recovery.scan_quality, recovery.quality_baseline,
            recovery.bad_scan_count,
            100.0f * recovery.global_particle_fraction);
    } else {
        std::snprintf(
            title, sizeof(title),
            "MCL global | conf=%.2f quality=%.2f/%.2f bad=%d global=%.0f%%",
            pose.confidence, recovery.scan_quality, recovery.quality_baseline,
            recovery.bad_scan_count,
            100.0f * recovery.global_particle_fraction);
    }
    SDL_SetWindowTitle(window, title);
}

static bool check_lidar_health(ILidarDriver* driver) {
    sl_lidar_response_device_health_t health = {};
    const sl_result result = driver->getHealth(health);
    if (!SL_IS_OK(result)) {
        std::fprintf(stderr, "Could not read LIDAR health: %x\n", result);
        return false;
    }
    if (health.status == SL_LIDAR_STATUS_ERROR) {
        std::fprintf(stderr, "LIDAR reports an internal error.\n");
        return false;
    }
    return true;
}

static void print_usage(const char* program) {
    std::printf(
        "Usage: %s [--port DEVICE] [--baud RATE] [--yaw DEGREES]\n"
        "  --port DEVICE  Serial device (default: lidar_port in config.txt; run from project root)\n"
        "  --baud RATE    Serial baud rate (default 460800)\n"
        "  --yaw DEGREES  Accurate absolute startup-relative yaw prior\n"
        "Controls: R reset global localization, Esc/Q quit\n",
        program);
}

int main(int argc, char** argv) {
    std::string port;
    bool port_supplied = false;
    int baudrate = 460800;
    bool yaw_supplied = false;
    float imu_yaw_deg = 0.0f;

    for (int i = 1; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--port") && i + 1 < argc) {
            port = argv[++i];
            port_supplied = true;
        } else if (!std::strcmp(argv[i], "--baud") && i + 1 < argc) {
            baudrate = std::atoi(argv[++i]);
        } else if (!std::strcmp(argv[i], "--yaw") && i + 1 < argc) {
            imu_yaw_deg = std::strtof(argv[++i], nullptr);
            yaw_supplied = true;
        } else if (!std::strcmp(argv[i], "--help")
                   || !std::strcmp(argv[i], "-h")) {
            print_usage(argv[0]);
            return 0;
        } else {
            print_usage(argv[0]);
            return 1;
        }
    }

    if (!port_supplied) {
        try {
            port = configured_lidar_port();
        } catch (const std::exception& error) {
            std::fprintf(stderr, "%s\n", error.what());
            return 1;
        }
    }

    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        std::fprintf(stderr, "SDL initialization failed: %s\n", SDL_GetError());
        return 2;
    }
    SDL_Window* window = SDL_CreateWindow(
        "MCL particles", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        WINDOW_WIDTH, WINDOW_HEIGHT, SDL_WINDOW_SHOWN);
    SDL_Renderer* renderer = window
        ? SDL_CreateRenderer(
              window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC)
        : nullptr;
    if (!renderer && window) {
        renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_SOFTWARE);
    }
    if (!window || !renderer) {
        std::fprintf(stderr, "SDL window creation failed: %s\n", SDL_GetError());
        if (renderer) SDL_DestroyRenderer(renderer);
        if (window) SDL_DestroyWindow(window);
        SDL_Quit();
        return 2;
    }

    ILidarDriver* driver = *createLidarDriver();
    IChannel* channel = *createSerialPortChannel(port.c_str(), baudrate);
    if (!driver || !channel || !SL_IS_OK(driver->connect(channel))
        || !check_lidar_health(driver)) {
        std::fprintf(
            stderr, "Could not connect to a healthy LIDAR at %s:%d\n",
            port.c_str(), baudrate);
        delete driver;
        delete channel;
        SDL_DestroyRenderer(renderer);
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 3;
    }

    signal(SIGINT, handle_signal);
    driver->setMotorSpeed();
    if (!SL_IS_OK(driver->startScan(0, 1))) {
        std::fprintf(stderr, "Could not start LIDAR scanning.\n");
        delete driver;
        delete channel;
        SDL_DestroyRenderer(renderer);
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 4;
    }

    loc_init_map(PITCH_WIDTH_MM, PITCH_HEIGHT_MM);
    if (yaw_supplied) {
        loc_set_imu_yaw(imu_yaw_deg);
    }
    loc_start(USE_PCB);

    bool running = true;
    double last_predict_time_s = monotonic_time_s();
    sl_lidar_response_measurement_node_hq_t nodes[8192];
    std::vector<LocScanPoint> scan;

    while (running && !g_stop_requested) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) {
                running = false;
            } else if (event.type == SDL_KEYDOWN) {
                if (event.key.keysym.sym == SDLK_ESCAPE
                    || event.key.keysym.sym == SDLK_q) {
                    running = false;
                } else if (event.key.keysym.sym == SDLK_r) {
                    loc_reset();
                }
            }
        }

        size_t count = _countof(nodes);
        const double scan_start_s = monotonic_time_s();
        const sl_result result = driver->grabScanDataHq(nodes, count, 0);
        const double scan_end_s = monotonic_time_s();
        if (!SL_IS_OK(result)) {
            SDL_Delay(5);
            continue;
        }
        driver->ascendScanData(nodes, count);

        const double now_s = monotonic_time_s();
        const float dt_s = (float)(now_s - last_predict_time_s);
        last_predict_time_s = now_s;
        loc_predict_odometry(0.0f, 0.0f, 0.0f, dt_s);

        scan.clear();
        scan.reserve(count);
        for (size_t i = 0; i < count; ++i) {
            LocScanPoint point = {};
            point.angle_deg =
                (nodes[i].angle_z_q14 * 90.0f) / 16384.0f;
            point.distance_mm = nodes[i].dist_mm_q2 / 4.0f;
            point.quality =
                nodes[i].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT;
            point.hit = nodes[i].dist_mm_q2 != 0
                        && point.quality >= MIN_QUALITY;
            if (!point.hit) {
                point.distance_mm = 0.0f;
                point.quality = 0;
            }
            scan.push_back(point);
        }

        loc_update_scan(
            scan.data(), (int)scan.size(),
            MIN_RANGE_MM, MAX_RANGE_MM, MIN_QUALITY,
            0.5 * (scan_start_s + scan_end_s));
        render(window, renderer, scan);
    }

    loc_stop();
    driver->stop();
    SDL_Delay(200);
    driver->setMotorSpeed(0);
    delete driver;
    delete channel;
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}
