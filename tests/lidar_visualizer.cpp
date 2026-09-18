/*
 * RPLidar C1 Bird's Eye View Visualizer
 * 
 * Connects to RPLidar C1 sensor and renders a real-time bird's eye view
 * visualization using SDL2, with dots representing detected objects.
 * 
 * Usage:
 *   ./tests/lidar_visualizer
 *
 * Build (from project root on Linux):
 *   g++ -std=c++11 -o tests/lidar_visualizer tests/lidar_visualizer.cpp \
 *     -I./rplidar_sdk/sdk/include -I./rplidar_sdk/sdk/src \
 *     -L./rplidar_sdk/output/Linux/Release -lsl_lidar_sdk -lSDL2 -lpthread -lrt
 *
 * Serial Port: lidar_port in config.txt (run from the project root)
 * Baudrate: 460800
 */

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <math.h>
#include <time.h>

#include <SDL2/SDL.h>

#include "lidar_config.h"
#include "sl_lidar.h"
#include "sl_lidar_driver.h"

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

// Configuration
#define BAUDRATE 460800

// Window settings
#define WINDOW_WIDTH 800
#define WINDOW_HEIGHT 800
#define CENTER_X (WINDOW_WIDTH / 2)
#define CENTER_Y (WINDOW_HEIGHT / 2)

// Visualization settings
#define MAX_DISTANCE_MM 12000.0f  // Maximum distance to display (12 meters)
#define SCALE_FACTOR ((float)(WINDOW_WIDTH / 2 - 50) / MAX_DISTANCE_MM)

// Colors (RGBA)
#define COLOR_BACKGROUND   0x1a, 0x1a, 0x2e, 0xff  // Dark blue-gray
#define COLOR_LIDAR_CENTER 0xff, 0x00, 0x00, 0xff  // Red for LIDAR position
#define COLOR_SCAN_POINT   0x00, 0xff, 0x88, 0xff  // Cyan-green for scan points
#define COLOR_GRID         0x33, 0x33, 0x44, 0xff  // Dim grid lines
#define COLOR_SCALE_TEXT   0xaa, 0xaa, 0xaa, 0xff  // Gray for scale text

using namespace sl;

static inline void delay(sl_word_size_t ms) {
    while (ms >= 1000) {
        usleep(1000 * 1000);
        ms -= 1000;
    }
    if (ms != 0)
        usleep(ms * 1000);
}

// Global control flag
bool ctrl_c_pressed = false;
void ctrlc(int) {
    ctrl_c_pressed = true;
}

// Check LIDAR health status
bool checkLidarHealth(ILidarDriver* drv) {
    sl_result op_result;
    sl_lidar_response_device_health_t healthinfo;

    op_result = drv->getHealth(healthinfo);
    if (SL_IS_OK(op_result)) {
        printf("LIDAR health status: %d\n", healthinfo.status);
        if (healthinfo.status == SL_LIDAR_STATUS_ERROR) {
            fprintf(stderr, "Error: LIDAR internal error detected. Please reboot the device.\n");
            return false;
        }
        return true;
    } else {
        fprintf(stderr, "Error: Cannot retrieve LIDAR health code: %x\n", op_result);
        return false;
    }
}

// Draw a filled circle
void drawFilledCircle(SDL_Renderer* renderer, int cx, int cy, int radius) {
    for (int y = -radius; y <= radius; y++) {
        for (int x = -radius; x <= radius; x++) {
            if (x * x + y * y <= radius * radius) {
                SDL_RenderDrawPoint(renderer, cx + x, cy + y);
            }
        }
    }
}

// Draw grid circles for distance reference
void drawGrid(SDL_Renderer* renderer) {
    SDL_SetRenderDrawColor(renderer, COLOR_GRID);
    
    // Draw concentric circles at 2m, 4m, 6m, 8m, 10m, 12m
    float distances[] = {2000.0f, 4000.0f, 6000.0f, 8000.0f, 10000.0f, 12000.0f};
    
    for (int d = 0; d < 6; d++) {
        int radius = (int)(distances[d] * SCALE_FACTOR);
        
        // Draw circle using points
        for (int angle = 0; angle < 360; angle++) {
            float rad = angle * M_PI / 180.0f;
            int x = CENTER_X + (int)(radius * cos(rad));
            int y = CENTER_Y + (int)(radius * sin(rad));
            SDL_RenderDrawPoint(renderer, x, y);
        }
    }
    
    // Draw cross-hairs
    SDL_RenderDrawLine(renderer, CENTER_X, 0, CENTER_X, WINDOW_HEIGHT);
    SDL_RenderDrawLine(renderer, 0, CENTER_Y, WINDOW_WIDTH, CENTER_Y);
}

// Render the scan data
void renderScan(SDL_Renderer* renderer, sl_lidar_response_measurement_node_hq_t* nodes, size_t count) {
    // Clear screen with background color
    SDL_SetRenderDrawColor(renderer, COLOR_BACKGROUND);
    SDL_RenderClear(renderer);
    
    // Draw grid
    drawGrid(renderer);
    
    // Draw scan points
    SDL_SetRenderDrawColor(renderer, COLOR_SCAN_POINT);
    
    for (size_t i = 0; i < count; i++) {
        // Skip invalid measurements (distance = 0 means no valid reading)
        if (nodes[i].dist_mm_q2 == 0) continue;
        
        // Get quality - skip low quality measurements
        int quality = nodes[i].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT;
        if (quality < 10) continue;
        
        // Convert angle from fixed-point to degrees, then to radians
        float angle_deg = (nodes[i].angle_z_q14 * 90.0f) / 16384.0f;
        float angle_rad = angle_deg * M_PI / 180.0f;
        
        // Convert distance from fixed-point to mm
        float distance_mm = nodes[i].dist_mm_q2 / 4.0f;
        
        // Skip if beyond max distance
        if (distance_mm > MAX_DISTANCE_MM) continue;
        
        // Convert polar to Cartesian coordinates
        // Note: In LIDAR coordinate system, 0 degrees is forward, 
        // increasing clockwise. We adjust for screen coordinates.
        float scaled_dist = distance_mm * SCALE_FACTOR;
        int x = CENTER_X + (int)(scaled_dist * sin(angle_rad));
        int y = CENTER_Y - (int)(scaled_dist * cos(angle_rad));  // Negative because screen Y is inverted
        
        // Color intensity based on quality
        int intensity = 128 + (quality * 127 / 255);
        if (intensity > 255) intensity = 255;
        SDL_SetRenderDrawColor(renderer, 0x00, intensity, 0x88, 0xff);
        
        // Draw point (small filled circle for visibility)
        drawFilledCircle(renderer, x, y, 2);
    }
    
    // Draw LIDAR center position (red dot)
    SDL_SetRenderDrawColor(renderer, COLOR_LIDAR_CENTER);
    drawFilledCircle(renderer, CENTER_X, CENTER_Y, 5);
    
    // Present the rendered frame
    SDL_RenderPresent(renderer);
}

void printUsage(const char* progname) {
    printf("RPLidar C1 Bird's Eye View Visualizer\n");
    printf("Usage: %s\n", progname);
    printf("\nSerial Port: lidar_port in config.txt (run from the project root)\n");
    printf("Baudrate: %d\n", BAUDRATE);
    printf("\nControls:\n");
    printf("  ESC or Q - Quit\n");
    printf("  Ctrl+C   - Quit\n");
}

int main(int argc, char* argv[]) {
    // Show usage if requested
    if (argc > 1 && (strcmp(argv[1], "--help") == 0 || strcmp(argv[1], "-h") == 0)) {
        printUsage(argv[0]);
        return 0;
    }
    
    printf("RPLidar C1 Bird's Eye View Visualizer\n");
    printf("SDK Version: %s\n", SL_LIDAR_SDK_VERSION);
    std::string port;
    try {
        port = configured_lidar_port();
    } catch (const std::exception& error) {
        fprintf(stderr, "%s\n", error.what());
        return 1;
    }
    
    // Initialize SDL
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        fprintf(stderr, "SDL initialization failed: %s\n", SDL_GetError());
        return -1;
    }
    
    // Create window
    SDL_Window* window = SDL_CreateWindow(
        "RPLidar C1 - Bird's Eye View",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        WINDOW_WIDTH, WINDOW_HEIGHT,
        SDL_WINDOW_SHOWN
    );
    
    if (!window) {
        fprintf(stderr, "Window creation failed: %s\n", SDL_GetError());
        SDL_Quit();
        return -1;
    }
    
    // Create renderer
    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    if (!renderer) {
        fprintf(stderr, "Renderer creation failed: %s\n", SDL_GetError());
        SDL_DestroyWindow(window);
        SDL_Quit();
        return -1;
    }
    
    // Create LIDAR driver
    ILidarDriver* drv = *createLidarDriver();
    if (!drv) {
        fprintf(stderr, "Error: Insufficient memory for LIDAR driver\n");
        SDL_DestroyRenderer(renderer);
        SDL_DestroyWindow(window);
        SDL_Quit();
        return -2;
    }
    
    // Create serial channel and connect
    printf("Connecting to LIDAR at %s (baudrate: %d)...\n", port.c_str(), BAUDRATE);
    IChannel* channel = *createSerialPortChannel(port.c_str(), BAUDRATE);
    if (!channel) {
        fprintf(stderr, "Error: Unable to create serial channel for %s\n", port.c_str());
        delete drv;
        SDL_DestroyRenderer(renderer);
        SDL_DestroyWindow(window);
        SDL_Quit();
        return -3;
    }
    
    sl_result op_result;
    sl_lidar_response_device_info_t devinfo;
    bool connected = false;
    
    if (SL_IS_OK(drv->connect(channel))) {
        op_result = drv->getDeviceInfo(devinfo);
        if (SL_IS_OK(op_result)) {
            connected = true;
        }
    }
    
    if (!connected) {
        fprintf(stderr, "Error: Cannot connect to LIDAR at %s\n", port.c_str());
        fprintf(stderr, "Make sure:\n");
        fprintf(stderr, "  1. The LIDAR is connected\n");
        fprintf(stderr, "  2. You have permission to access the serial port\n");
        fprintf(stderr, "     (try: sudo chmod 666 %s)\n", port.c_str());
        delete drv;
        delete channel;
        SDL_DestroyRenderer(renderer);
        SDL_DestroyWindow(window);
        SDL_Quit();
        return -3;
    }
    
    // Print device info
    printf("LIDAR Connected!\n");
    printf("  Serial Number: ");
    for (int i = 0; i < 16; i++) {
        printf("%02X", devinfo.serialnum[i]);
    }
    printf("\n");
    printf("  Firmware Version: %d.%02d\n", devinfo.firmware_version >> 8, devinfo.firmware_version & 0xFF);
    printf("  Hardware Version: %d\n", (int)devinfo.hardware_version);
    
    // Check health
    if (!checkLidarHealth(drv)) {
        delete drv;
        delete channel;
        SDL_DestroyRenderer(renderer);
        SDL_DestroyWindow(window);
        SDL_Quit();
        return -4;
    }
    
    // Set up signal handler
    signal(SIGINT, ctrlc);
    
    // Start motor and scanning
    printf("Starting motor and scan...\n");
    drv->setMotorSpeed();
    drv->startScan(0, 1);
    
    // Wait a moment for the scan to start
    delay(500);
    
    // Main loop
    printf("Visualization running. Press ESC or Q to quit.\n");
    
    bool running = true;
    SDL_Event event;
    
    sl_lidar_response_measurement_node_hq_t nodes[8192];
    
    while (running && !ctrl_c_pressed) {
        // Handle SDL events
        while (SDL_PollEvent(&event)) {
            switch (event.type) {
                case SDL_QUIT:
                    running = false;
                    break;
                case SDL_KEYDOWN:
                    if (event.key.keysym.sym == SDLK_ESCAPE || 
                        event.key.keysym.sym == SDLK_q) {
                        running = false;
                    }
                    break;
            }
        }
        
        // Grab scan data
        size_t count = _countof(nodes);
        op_result = drv->grabScanDataHq(nodes, count, 0);
        
        if (SL_IS_OK(op_result)) {
            // Sort by angle
            drv->ascendScanData(nodes, count);
            
            // Render the scan
            renderScan(renderer, nodes, count);
        }
        
        // Small delay to prevent CPU spinning
        delay(10);
    }
    
    // Cleanup
    printf("Shutting down...\n");
    drv->stop();
    delay(200);
    drv->setMotorSpeed(0);
    
    delete drv;
    delete channel;
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    
    printf("Done.\n");
    return 0;
}

