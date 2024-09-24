#pragma once

#include <math.h>
#include <sched.h>

#include <algorithm>
#include <functional>
#include <opencv2/opencv.hpp>

#include "Logger.h"

#if defined(_MSC_VER)
#ifdef DLL_EXPORTS
#define EXPORT_API __declspec(dllexport)
#else
#define EXPORT_API __declspec(dllimport)
#endif
#elif __GNUC__ >= 4
#define EXPORT_API __attribute__((visibility("default")))
#else
#define EXPORT_API
#endif

#define WIDTH 640
#define HEIGHT 480

// Inference hardware runtime.
typedef enum runtime {
  CPU = 0,
  GPU,
  GPU_FLOAT16,
  DSP,
  DSP_FIXED8,
  AIP
} runtime_t;

struct ObjectData {
  // Bounding box information: top-left coordinate and width, height
  cv::Rect bbox;
  // Confidence of this bounding box
  float confidence = -1.0f;
  // The label of this Bounding box
  int label = -1;
  // Time cost of detecting this frame
  int64_t time_cost = 0;
};

/**
 * @brief: ObjectData detection config info.
 */
struct ObjectDetectionConfig {
  std::string model_path;
  runtime_t runtime;
  int labels = 85;
  int grids = 25200;
  std::vector<std::string> inputLayers;
  std::vector<std::string> outputLayers;
  std::vector<std::string> outputTensors;
};

static float calcIoU(const cv::Rect& a, const cv::Rect& b) {
  float xOverlap = std::max(
      0., std::min(a.x + a.width, b.x + b.width) - std::max(a.x, b.x) + 1.);
  float yOverlap = std::max(
      0., std::min(a.y + a.height, b.y + b.height) - std::max(a.y, b.y) + 1.);
  float intersection = xOverlap * yOverlap;
  float unio = (a.width + 1.) * (a.height + 1.) +
               (b.width + 1.) * (b.height + 1.) - intersection;
  return intersection / unio;
}

static int64_t GetTimeStamp_ms() {
  std::chrono::time_point<std::chrono::system_clock, std::chrono::milliseconds>
      tp = std::chrono::time_point_cast<std::chrono::milliseconds>(
          std::chrono::system_clock::now());
  std::time_t timestamp = tp.time_since_epoch().count();
  return timestamp;
}

// static double clock_ms() {
//   double clockbase_ms = 0;
//   struct timespec ts;
//   clock_gettime(CLOCK_MONOTONIC, &ts);
//   return (double)ts.tv_sec * 1000.0 + (double)ts.tv_nsec / 1000000.0 -
//          clockbase_ms;
// }

// static int getNumberCPUs() {
//   int cores = 0;
//   DIR* dir;
//   struct dirent* ent;
//   if ((dir = opendir("/sys/devices/system/cpu/") != NULL)) {
//     while (ent = readdir(dir) != NULL) {
//       std::string path = ent->d_name;
//       if (path.find("cpu") == 0) {
//         bool isCore = true;
//         for (int i = 3; i < path.length(); i++) {
//           if (path[i] < '0' || path[i] > '9') {
//             isCore = false;
//             break;
//           }
//         }
//         if (isCore) {
//           cores++;
//         }
//       }
//     }
//     closedir(dir);
//   }
//   return cores;
// }

// void bindCore(int coreNum) {
//   cpu_set_t mask;
//   CPU_ZERO(&mask);
//   CPU_SET(coreNum, &mask);
//   if (sched_setaffinity(0, sizeof(mask), &mask) == -1) {
//     LOG_ERROR("Failed to bind CPU core: {}", coreNum);
//   }
// }