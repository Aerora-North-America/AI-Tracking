#include "runtime.hpp"

enum queueCtrl {
  CACHE_1 = 1,
  CACHE_2,
  CACHE_3,
  CACHE_4,
  CACHE_5,
  CACHE_6,
};

enum queueCtrl QueueCtrl = CACHE_1;

Runtime::~Runtime() {
  if (detect_1_.joinable()) {
    detect_1_.join();
  }

  if (detect_2_.joinable()) {
    detect_2_.join();
  }

  if (detect_3_.joinable()) {
    detect_3_.join();
  }

  if (detect_4_.joinable()) {
    detect_4_.join();
  }

  if (detect_5_.joinable()) {
    detect_5_.join();
  }

  if (detect_6_.joinable()) {
    detect_6_.join();
  }

  if (getdata_.joinable()) {
    getdata_.join();
  }
}

Runtime::Runtime() {
  // std::cout << json_->value << std::endl;

  // Json::StreamWriterBuilder writer;
  // std::string output = Json::writeString(writer, json->value);

  // // 打印 JSON 字符串
  // std::cout << output << std::endl;

  // yolov5::ObjectDetectionConfig config;

  if (udp_get_data_) {
    udp_get_data_ = std::make_shared<UdpGetData>();
  }

  if (dsp_detector1_) {
    dsp_detector1_ = std::make_shared<yolov5::ObjectDetectionImpl>();
  }

  if (dsp_detector2_) {
    dsp_detector2_ = std::make_shared<yolov5::ObjectDetectionImpl>();
  }

  if (dsp_detector3_) {
    dsp_detector3_ = std::make_shared<yolov5::ObjectDetectionImpl>();
  }

  if (dsp_detector4_) {
    dsp_detector4_ = std::make_shared<yolov5::ObjectDetectionImpl>();
  }

  if (dsp_detector5_) {
    dsp_detector5_ = std::make_shared<yolov5::ObjectDetectionImpl>();
  }

  if (dsp_detector6_) {
    dsp_detector6_ = std::make_shared<yolov5::ObjectDetectionImpl>();
  }

  // ToDo: 段错误

  dsp_detector1_->Initialize(objconfig_);
  dsp_detector1_->SetScoreThresh(0.7, 0.7);

  dsp_detector2_->Initialize(objconfig_);
  dsp_detector2_->SetScoreThresh(0.7, 0.7);

  dsp_detector3_->Initialize(objconfig_);
  dsp_detector3_->SetScoreThresh(0.7, 0.7);

  dsp_detector4_->Initialize(objconfig_);
  dsp_detector4_->SetScoreThresh(0.7, 0.7);

  dsp_detector5_->Initialize(objconfig_);
  dsp_detector5_->SetScoreThresh(0.7, 0.7);

  dsp_detector6_->Initialize(objconfig_);
  dsp_detector6_->SetScoreThresh(0.7, 0.7);

  LOG_INFO("construct is complete");
}

void Runtime::init() {
  udp_get_data_->init();

  LOG_INFO("init is complete");
}

void Runtime::cbData() {
  cv::Mat tmpdata;
  //生产数据
  // udp_get_data_->process_frame(tmpdata);

  while (true) {
    tmpdata = udp_get_data_->get_data();
    switch (QueueCtrl) {
      case CACHE_1:
        frame_cache_1.product(std::make_shared<cv::Mat>(tmpdata));
        LOG_INFO("push date in queue 1");
        QueueCtrl = CACHE_2;
        break;

      case CACHE_2:
        frame_cache_2.product(std::make_shared<cv::Mat>(tmpdata));
        LOG_INFO("push date in queue 2");
        QueueCtrl = CACHE_3;

        break;

      case CACHE_3:
        frame_cache_3.product(std::make_shared<cv::Mat>(tmpdata));
        LOG_INFO("push date in queue 3");
        QueueCtrl = CACHE_4;
        break;

      case CACHE_4:
        frame_cache_4.product(std::make_shared<cv::Mat>(tmpdata));
        LOG_INFO("push date in queue 4");
        QueueCtrl = CACHE_5;
        break;

      case CACHE_5:
        frame_cache_5.product(std::make_shared<cv::Mat>(tmpdata));
        LOG_INFO("push date in queue 5");
        QueueCtrl = CACHE_6;
        break;

      case CACHE_6:
        frame_cache_6.product(std::make_shared<cv::Mat>(tmpdata));
        LOG_INFO("push date in queue 6");
        QueueCtrl = CACHE_1;
        break;

      default:
        break;
    }
  }
}

void Runtime::create() {
  //创建线程
  // std::thread producer([&]() { cbData(); });
  // producer.detach();
  getdata_ = std::thread(&Runtime::cbData, this);
  detect_1_ = std::thread(&Runtime::thread_detect_1, this);
  detect_2_ = std::thread(&Runtime::thread_detect_2, this);
  detect_3_ = std::thread(&Runtime::thread_detect_3, this);
  detect_4_ = std::thread(&Runtime::thread_detect_4, this);
  detect_5_ = std::thread(&Runtime::thread_detect_5, this);
  detect_6_ = std::thread(&Runtime::thread_detect_6, this);
}

void Runtime::thread_detect_1() {
  while (true) {
    std::shared_ptr<cv::Mat> imgframe;
    std::vector<yolov5::ObjectData> obj_vec;

    frame_cache_1.consumption(imgframe);

    // detect
    dsp_detector1_->Detect(*imgframe.get(), obj_vec);
    LOG_INFO("detect output size = %f", obj_vec.size());
    //要不要写入视频？
  }
}

void Runtime::thread_detect_2() {
  while (true) {
    std::shared_ptr<cv::Mat> imgframe;
    std::vector<yolov5::ObjectData> obj_vec;

    frame_cache_2.consumption(imgframe);

    // detect
    dsp_detector2_->Detect(*imgframe.get(), obj_vec);
    LOG_INFO("detect output size = %f", obj_vec.size());
    //要不要写入视频？
  }
}

void Runtime::thread_detect_3() {
  while (true) {
    std::shared_ptr<cv::Mat> imgframe;
    std::vector<yolov5::ObjectData> obj_vec;

    frame_cache_3.consumption(imgframe);

    // detect
    dsp_detector3_->Detect(*imgframe.get(), obj_vec);
    LOG_INFO("detect output size = %f", obj_vec.size());
    //要不要写入视频？
  }
}

void Runtime::thread_detect_4() {
  while (true) {
    std::shared_ptr<cv::Mat> imgframe;
    std::vector<yolov5::ObjectData> obj_vec;

    frame_cache_4.consumption(imgframe);

    // detect
    dsp_detector4_->Detect(*imgframe.get(), obj_vec);
    LOG_INFO("detect output size = %f", obj_vec.size());
    //要不要写入视频？
  }
}

void Runtime::thread_detect_5() {
  while (true) {
    std::shared_ptr<cv::Mat> imgframe;
    std::vector<yolov5::ObjectData> obj_vec;

    frame_cache_5.consumption(imgframe);

    // detect
    dsp_detector5_->Detect(*imgframe.get(), obj_vec);
    LOG_INFO("detect output size = %f", obj_vec.size());
    //要不要写入视频？
  }
}

void Runtime::thread_detect_6() {
  while (true) {
    std::shared_ptr<cv::Mat> imgframe;
    std::vector<yolov5::ObjectData> obj_vec;

    frame_cache_6.consumption(imgframe);

    // detect
    dsp_detector6_->Detect(*imgframe.get(), obj_vec);
    LOG_INFO("detect output size = %f", obj_vec.size());
    //要不要写入视频？
  }
}
