#if defined(WIN32) || defined(_WIN32)
#include <time.h>
#else
#include <unistd.h>
#endif

#include "ObjectDetector.hpp"

namespace yolov5 {

ObjectDetection::ObjectDetection() {
  impl = new ObjectDetectionImpl();
  tracker_ = std::make_shared<BYTETracker>();
}

ObjectDetection::~ObjectDetection() {
  if (nullptr != impl) {
    delete static_cast<ObjectDetectionImpl*>(impl);
    impl = nullptr;
  }

  if (tracker_ == nullptr) {
    tracker_ = std::make_shared<BYTETracker>(20, 30);
  }
}

bool ObjectDetection::Init(const ObjectDetectionConfig& config) {
  if (IsInitialized()) {
    return static_cast<ObjectDetectionImpl*>(impl)->DeInitialize() &&
           static_cast<ObjectDetectionImpl*>(impl)->Initialize(config);
  } else {
    return static_cast<ObjectDetectionImpl*>(impl)->Initialize(config);
  }
}

bool ObjectDetection::Deinit() {
  if (nullptr != impl && IsInitialized()) {
    return static_cast<ObjectDetectionImpl*>(impl)->DeInitialize();
  } else {
    LOG_ERROR("ObjectDetection: deinit failed!");
    return false;
  }
}

bool ObjectDetection::IsInitialized() {
  return static_cast<ObjectDetectionImpl*>(impl)->IsInitialized();
}

bool ObjectDetection::Detect(cv::Mat& image, std::vector<ObjectData>& results) {
  if (nullptr != impl && IsInitialized()) {
    auto ret = static_cast<ObjectDetectionImpl*>(impl)->Detect(image, results);
    return ret;
  } else {
    LOG_ERROR(
        "ObjectDetection::Detect failed caused by incompleted initialization!");
    return false;
  }
}

bool ObjectDetection::SetScoreThreshold(const float& conf_thresh,
                                        const float& nms_thresh) {
  if (nullptr != impl) {
    return static_cast<ObjectDetectionImpl*>(impl)->SetScoreThresh(conf_thresh,
                                                                   nms_thresh);
  } else {
    LOG_ERROR(
        "ObjectDetection::SetScoreThreshold failed because incompleted "
        "initialization!");
    return false;
  }
}

bool ObjectDetection::SetROI(const cv::Rect& roi) {
  if (nullptr != impl) {
    return static_cast<ObjectDetectionImpl*>(impl)->SetROI(roi);
  } else {
    LOG_ERROR(
        "ObjectDetection::SetROI failed because incompleted initialization!");
    return false;
  }
}

bool ObjectDetection::RegisterPreProcess(pre_process_t func) {
  if (nullptr != impl) {
    return static_cast<ObjectDetectionImpl*>(impl)->RegisterPreProcess(func);
  } else {
    LOG_ERROR(
        "ObjectDetection::RegisterPreProcess failed because incompleted "
        "initialization!");
    return false;
  }
}

bool ObjectDetection::RegisterPreProcess(post_process_t func) {
  if (nullptr != impl) {
    return static_cast<ObjectDetectionImpl*>(impl)->RegisterPreProcess(func);
  } else {
    LOG_ERROR(
        "ObjectDetection::RegisterPreProcess failed because incompleted "
        "initialization!");
    return false;
  }
}

}  // namespace yolov5