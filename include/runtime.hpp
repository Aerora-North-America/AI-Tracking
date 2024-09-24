#pragma once
#include <jsoncpp/json/json.h>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <stdexcept>
#include <vector>

#include "ObjectDetector.hpp"
#include "safeQueue.hpp"
#include "udp_receive.hpp"
// #include "fast-reid.h"
// #include "tracker.h"

class JsonReader {
 public:
  JsonReader(const std::string& filePath) : input(filePath) {
    if (!input.is_open()) {
      throw std::runtime_error("Failed to open configure file");
    }
    contents = {std::istreambuf_iterator<char>(input),
                std::istreambuf_iterator<char>()};

    parseJson();
  }

  void parseJson() {
    Json::Reader reader;

    if (!reader.parse(contents, value)) {
      throw std::runtime_error("Failed parsing json");
    }
    // return value;
  }

  static runtime_t device2runtime(std::string& device) {
    std::transform(device.begin(), device.end(), device.begin(),
                   [](unsigned char ch) { return tolower(ch); });

    if (0 == device.compare("cpu")) {
      return CPU;
    } else if (0 == device.compare("gpu")) {
      return GPU;
    } else if (0 == device.compare("gpu_float16")) {
      return GPU_FLOAT16;
    } else if (0 == device.compare("dsp")) {
      return DSP;
    } else if (0 == device.compare("dsp_fixed8")) {
      return DSP_FIXED8;
    } else if (0 == device.compare("aip")) {
      return AIP;
    } else {
      return CPU;
    }
  }

  // void parse_frome_json(yolov5::ObjectDetectionConfig& objconfig,
  //                       fastReidConfig& reidconfig, TrackerConfig& tkconfig,
  //                       const Json::Value& value)
  void parse_frome_json(ObjectDetectionConfig& config) {
    if (value["tracker"]["output-layers"].isArray()) {
      for (Json::ArrayIndex i = 0; i < value["tracker"]["output-layers"].size();
           i++) {
        auto element = value["tracker"]["output-layers"][i];

        if (element.isString()) {
          config.outputLayers.push_back(element.asString());
        }
      }
    }

    if (value["tracker"]["output-tensors"].isArray()) {
      for (Json::ArrayIndex i = 0;
           i < value["tracker"]["output-tensors"].size(); i++) {
        auto element = value["tracker"]["output-tensors"][i];

        if (element.isString()) {
          config.outputTensors.push_back(element.asString());
        }
      }
    }

    if (value["tracker"]["input-layers"].isArray()) {
      for (Json::ArrayIndex i = 0;
           i < value["tracker"]["output-tensors"].size(); i++) {
        auto element = value["tracker"]["input-layers"][i];
        if (element.isString()) {
          config.inputLayers.push_back(element.asString());
        }
      }
    }

    config.grids = value["tracker"]["grids"].asInt();
    config.labels = value["tracker"]["labels"].asInt();
    // config.runtime = runtime_t::CPU;
    config.model_path = value["tracker"]["model-path"].asString();

    std::string runtime = (value["tracker"]["runtime"].asString());

    config.runtime = device2runtime(runtime);
    // config.runtime = device2runtime(value["tracker"]["runtime"].asString());
  }

 private:
  Json::Value value;
  std::ifstream input;
  std::string contents;
};
