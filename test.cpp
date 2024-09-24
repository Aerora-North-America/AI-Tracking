#include <gflags/gflags.h>
#include <jsoncpp/json/json.h>

#include <fstream>
#include <opencv2/opencv.hpp>

#include "ObjectDetector.hpp"
#include "runtime.hpp"
#include "yolo.hpp"

static void drawResults(cv::Mat& img,
                        const std::vector<yolov5::ObjectData>& results,
                        const std::vector<std::string>& labels) {
  cv::cvtColor(img, img, cv::COLOR_BGR2RGB);
  // LOG_INFO("results size:", results.size());
  std::cout << "results size: " << results.size() << std::endl;
  for (const auto& result : results) {
    char t[256];
    cv::rectangle(img,
                  cv::Rect(result.bbox.x, result.bbox.y, result.bbox.width,
                           result.bbox.height),
                  cv::Scalar(0, 255, 0));

    cv::Point position = cv::Point(result.bbox.x, result.bbox.y - 10);
    // ToDo: label数量为0
    // cv::putText(img, labels[result.label], position,
    // cv::FONT_HERSHEY_COMPLEX,
    //             0.8, cv::Scalar(0, 255, 0), 2, 0.3);
  }
}

int main(int argc, char** argv) {
  google::ParseCommandLineFlags(&argc, &argv, true);

  if (argc < 2) {
    LOG_ERROR("Usage:./yolov5_inference <model_path>");
    return -1;
  }

  std::string file_path(argv[1]);

  // std::string video_name(argv[2]);

  if (!file_path.compare("")) {
    LOG_ERROR("Please provide a correct file");
    return -1;
  }

  //读取标签
  //   DEFINE_string(labels,
  //                 "/home/src/snpe-yolov7-interfence/cfg/yolov5s_labels.txt.txt",
  //                 "Labels file for the yolov5 model");

  std::vector<std::string> labels;
  //   std::ifstream in(FLAGS_labels);
  //   std::string line;

  //   while (getline(in, line)) {
  //     labels.push_back(line);
  //   }

  //   // LOG_INFO("Labels:", labels.size());
  //   std::cout << "labels: " << labels.size() << std::endl;
  std::shared_ptr<JsonReader> jsRead = std::make_shared<JsonReader>(file_path);

  //读取参数,涉及到数组
  yolov5::ObjectDetectionConfig config;
  // Json::Value value;
  jsRead->parse_frome_json(config);

  std::shared_ptr<UdpGetData> udp_get_data_ = std::make_shared<UdpGetData>();

  std::shared_ptr<yolov5::ObjectDetection> dsp_detect =
      std::make_shared<yolov5::ObjectDetection>();

  dsp_detect->Init(config);
  dsp_detect->SetScoreThreshold(0.7f, 0.5f);

  udp_get_data_->init();
  cv::Mat img;

  cv::VideoWriter video_writer(
      "result.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 15,
      cv::Size(cv::CAP_PROP_FRAME_WIDTH, cv::CAP_PROP_FRAME_HEIGHT));

  std::vector<yolov5::ObjectData> obj_vec(100);
  while (true) {
    udp_get_data_->product(img);

    if (img.empty()) {
      continue;
    } else {
      dsp_detect->Detect(img, obj_vec);
      drawResults(img, obj_vec, labels);
      //   video_writer << img;
    }

    obj_vec.clear();
  }
}