/*
 * @Description: Inference SDK based on SNPE.
 * @version: 1.1
 * @Author: chunhu.chen@aeroratech.com
 * @Date: 2024.07.15
 * @LastEditors: chunhu.chen
 * @LastEditTime:
 */

#ifndef __YOLOV5S_IMPL_H__
#define __YOLOV5S_IMPL_H__

#include <arm_neon.h>

#include <string>
#include <vector>
#if defined(WIN32) || defined(_WIN32)
#include <time.h>
#else
#include <unistd.h>
#endif
#include <memory>

#include "../mavlink/include/mavlink.hpp"
#include "../mavlink/include/mavlink_gcs.hpp"
#include "SNPEPipeline.hpp"
#include "yolo.hpp"

namespace yolov5 {

class ObjectDetectionImpl {
 public:
  ObjectDetectionImpl();
  ~ObjectDetectionImpl();
  bool Detect(cv::Mat& image, std::vector<ObjectData>& results);
  bool Initialize(const ObjectDetectionConfig& config);
  bool DeInitialize();

  bool SetScoreThresh(const float& conf_thresh,
                      const float& nms_thresh = 0.5) noexcept {
    this->m_nmsThresh = nms_thresh;
    this->m_confThresh = conf_thresh;
    return true;
  }

  bool SetROI(const cv::Rect& roi) {
    this->m_roi = roi;
    return true;
  }

  bool RegisterPreProcess(pre_process_t func) {
    this->m_preProcess = func;
    m_isRegisteredPreProcess = true;
    return true;
  }

  bool RegisterPreProcess(post_process_t func) {
    this->m_postProcess = func;
    m_isRegisteredPostProcess = true;
    return true;
  }

  bool IsInitialized() const { return m_isInit; }

  static std::vector<ObjectData> nms(std::vector<ObjectData> winList,
                                     const float& nms_thresh) {
    if (winList.empty()) {
      return winList;
    }

    std::sort(winList.begin(), winList.end(),
              [](const ObjectData& left, const ObjectData& right) {
                if (left.confidence > right.confidence) {
                  return true;
                } else {
                  return false;
                }
              });

    std::vector<bool> flag(winList.size(), false);
    for (int i = 0; i < winList.size(); i++) {
      if (flag[i]) {
        continue;
      }

      for (int j = i + 1; j < winList.size(); j++) {
        if (calcIoU(winList[i].bbox, winList[j].bbox) > nms_thresh) {
          flag[j] = true;
        }
      }
    }

    std::vector<ObjectData> ret;
    for (int i = 0; i < winList.size(); i++) {
      if (!flag[i]) ret.push_back(winList[i]);
    }

    return std::move(ret);
  }

 private:
  // void sendMessage();
  bool m_isInit = false;
  bool m_isRegisteredPreProcess = false;
  bool m_isRegisteredPostProcess = false;

  bool PreProcess(cv::Mat& frame);
  bool PostProcess(std::vector<ObjectData>& results, int64_t time);
  // ObjectData& getLockedObj(std::vector<ObjectData>& results) {
  //   for (auto it = results.begin(); it != results.end(); it++) {
  //     // if (it.bbox.contains(point)) {
  //     //   return *it;
  //     // }
  //   }
  // }

  pre_process_t m_preProcess;
  post_process_t m_postProcess;

  std::unique_ptr<snpetask::SNPETask> m_task;
  std::vector<std::string> m_inputLayers;
  std::vector<std::string> m_outputLayers;
  std::vector<std::string> m_outputTensors;

  std::vector<int> boxIndexs;
  std::vector<float> boxConfidences;
  std::vector<ObjectData> winList;

  int m_labels;
  int m_grids;
  float* m_output;

  cv::Rect m_roi = {0, 0, 0, 0};
  uint32_t m_minBoxBorder = 16;
  float m_nmsThresh = 0.5f;
  float m_confThresh = 0.5f;
  float m_scale;
  int m_xOffset, m_yOffset;
  bool locked = false;
  int id = 0;         //锁定目标的id
  cv::Point2f point;  //用户点击的坐标
};

}  // namespace yolov5

#endif  // __YOLOV5S_IMPL_H__
