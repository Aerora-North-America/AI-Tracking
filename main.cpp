#include <arm_neon.h>
#include <gflags/gflags.h>
#include <jsoncpp/json/json.h>

#include <fstream>
#include <opencv2/opencv.hpp>

#include "ObjectDetector.hpp"
#include "runtime.hpp"
#include "yolo.hpp"

#include <arpa/inet.h>
#include <unistd.h>

#include <semaphore.h>

bool mavlink_run = false;
GCS_MAVLINK gcs_data_;
std::mutex videoWriting;
char mavlinkData[64];
int mavlinkDataLen =0;
sem_t getMavlinkDataSem;

bool detect_1_finished = true;
bool detect_2_finished = true;
bool detect_3_finished = true;
bool detect_4_finished = true;

void mavlinkRouter_interface() {
    // 创建 UDP 套接字
    int udpSocket = socket(AF_INET, SOCK_DGRAM, 0);
    if (udpSocket < 0) {
        std::cerr << "Failed to create socket." << std::endl;
    }

    // 设置目标地址和端口
    sockaddr_in targetAddr;
    targetAddr.sin_family = AF_INET;
    targetAddr.sin_port = htons(14550);
    targetAddr.sin_addr.s_addr = inet_addr("0.0.0.0"); // 本机

    while (true) {
      // 要发送的数据
      //const char data[16] = "Hello,UDP World";
      //sem_wait(&getMavlinkDataSem);
      // 发送数据
      ssize_t sendResult = sendto(udpSocket, mavlinkData, mavlinkDataLen, 0, 
                                  (sockaddr*)&targetAddr, sizeof(targetAddr));
      if (sendResult < 0) {
          std::cerr << "Failed to send data." << std::endl;
          //close(udpSocket);
      }
      else{
          //printf("data: ");
	  //for(int i=0 ; i<mavlinkDataLen; i++){
	    //printf("%x ",mavlinkData[i]);
	  //}
	  //printf("\n");
	  //std::cout << "Send data success." << std::endl;
      }
      usleep(20000);
    }
}


void mavlink_main() {
  mavlink_run = true;
  //detect_tty();
  gcs_data_.init(GCS_MAVLINK_DEFINE_COM, GCS_MAVLINK_DEFINE_VERSION);
  LOG_INFO("start mavlink main thread");

  while (mavlink_run) {
    //detect_tty();
    // sendMessage()
    gcs_data_.update();
    usleep(1000);
  }
}

enum queueCtrl {
  CACHE_1 = 1,
  CACHE_2,
  CACHE_3,
  CACHE_4,
  CACHE_5,
  CACHE_6,
};

enum queueCtrl QueueCtrl = CACHE_1;

yolov5::ObjectDetection dsp_detector1_;
yolov5::ObjectDetection dsp_detector2_;
yolov5::ObjectDetection dsp_detector3_;
yolov5::ObjectDetection dsp_detector4_;
yolov5::ObjectDetection dsp_detector5_;
yolov5::ObjectDetection dsp_detector6_;

UdpGetData* udp_get_data_;

TsProAndCon<cv::Mat> frame_cache_1;
TsProAndCon<cv::Mat> frame_cache_2;
TsProAndCon<cv::Mat> frame_cache_3;
TsProAndCon<cv::Mat> frame_cache_4;
TsProAndCon<cv::Mat> frame_cache_5;
TsProAndCon<cv::Mat> frame_cache_6;

// std::thread getdata_;
std::thread detect_1_;
std::thread detect_2_;
std::thread detect_3_;
std::thread detect_4_;
std::thread detect_5_;
std::thread detect_6_;
std::thread mavlink_thread_;

static int locked_frame_id = -1;

std::vector<std::string> labels;

static bool locked = false;
static void sendMessage(const ObjectData track) {
  // ToDo: send message to GCS
  // for (const auto& track : track) {
  //   // ToDo: send data to GCS
  // }

  mavlink_camera_tracking_image_status_t data;

  // //没锁定只发状态
  // if (!locked) {

    auto iter =  track;
    //printf("x: %d  y: %d \n ",iter.bbox.x,iter.bbox.y);
    cv::Point2i center;
    //result.bbox.x, result.bbox.y, result.bbox.width,result.bbox.height

    //归一化目标中心点
    center.x = iter.bbox.x + iter.bbox.width / 2;
    center.y = iter.bbox.y + iter.bbox.height / 2;

    float point_x = (float)center.x / WIDTH;
    float point_y = (float)center.y / HEIGHT;
    printf("px: %f  py: %f \n ",point_x,point_y);

    //归一化目标矩形框
    float rec_top_x = (float)iter.bbox.x / WIDTH;
    float rec_top_y = (float)iter.bbox.y / HEIGHT;
    float rec_width = (float)iter.bbox.width / WIDTH;
    float rec_height = (float)iter.bbox.height / HEIGHT;

    data.point_x = point_x;
    data.point_y = point_y;

    data.rec_top_x = rec_top_x;
    data.rec_top_y = rec_top_y;
    data.rec_bottom_x = rec_width;
    data.rec_bottom_y = rec_height;

    //状态更新
    data.tracking_mode = 1;
    data.tracking_status = 1;
    //然后将目标发送出去
    gcs_data_.push_data(data);
  // } else {
  //     //先确定锁定目标id
  //     cv::Point2i touch_position;
  //     for (auto iter : track) {
  //       cv::Rect rect(iter.tlwh[0], iter.tlwh[1], iter.tlwh[2], iter.tlwh[3]);
  //       if (rect.contains(touch_position)) {
  //         locked_frame_id = iter.track_id;
  //         cv::Point2i center;

  //         //归一化目标中心点
  //         // center.x = lock_obj.bbox.x + lock_obj.bbox.width / 2;
  //         // center.y = lock_obj.bbox.y + lock_obj.bbox.height / 2;
  //         center.x = iter.tlwh[0] + iter.tlwh[2] / 2;
  //         center.y = iter.tlwh[1] + iter.tlwh[3] / 2;

  //         auto point_x = center.x / WIDTH;
  //         auto point_y = center.y / HEIGHT;

  //         //归一化目标矩形框
  //         auto rec_top_x = iter.tlwh[0] / WIDTH;
  //         auto rec_top_y = iter.tlwh[1] / HEIGHT;
  //         auto rec_width = iter.tlwh[2] / WIDTH;
  //         auto rec_height = iter.tlwh[3] / HEIGHT;

  //         data.point_x = point_x;
  //         data.point_y = point_y;

  //         data.rec_top_x = rec_top_x;
  //         data.rec_top_y = rec_top_y;
  //         data.rec_bottom_x = rec_width;
  //         data.rec_bottom_y = rec_height;

  //         //状态更新
  //         data.tracking_mode = 1;
  //         data.tracking_status = 1;
  //         //然后将目标发送出去
  //         gcs_data_.push_data(data);
  //         // gcs_data_.update();
  //         break;
  //       }
  //       continue;
  //     }
  //     locked = true;
  // }
}

cv::VideoWriter video_writer("result.avi",
                             cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 15,
                             cv::Size(640, 480));

static void drawResults(cv::Mat& img, const std::vector<ObjectData>& results,
                        const std::vector<std::string>& labels) {
  cv::cvtColor(img, img, cv::COLOR_BGR2RGB);
  // LOG_INFO("results size:", results.size());
  // std::cout << "results size: " << results.size() << std::endl;
  for (const auto& result : results) {
    char t[256];
    cv::rectangle(img,
                  cv::Rect(result.bbox.x, result.bbox.y, result.bbox.width,
                           result.bbox.height),
                  cv::Scalar(0, 255, 0));

    cv::Point position = cv::Point(result.bbox.x, result.bbox.y - 10);

    cv::putText(img, labels[result.label], position,
    cv::FONT_HERSHEY_COMPLEX,
                0.8, cv::Scalar(0, 255, 0), 2, 0.3);
  }
}


// static void drawResults(cv::Mat& img, const std::vector<ObjectData>& results)
// {
//   // cv::cvtColor(img, img, cv::COLOR_BGR2RGB);
//   // LOG_INFO("results size:", results.size());
//   std::cout << "results size: " << results.size() << std::endl;
//   for (const auto& result : results) {
//     char t[256];
//     cv::rectangle(img,
//                   cv::Rect(result.bbox.x, result.bbox.y, result.bbox.width,
//                            result.bbox.height),
//                   cv::Scalar(0, 255, 0));

//     cv::Point position = cv::Point(result.bbox.x, result.bbox.y - 10);
//     // ToDo: label数量为0
//     // cv::putText(img, labels[result.label], position,
//     // cv::FONT_HERSHEY_COMPLEX,
//     //             0.8, cv::Scalar(0, 255, 0), 2, 0.3);
//   }
// }

void cbData() {
  cv::Mat tmpdata;
  //生产数据

  while (true) {
    //缓存为0则跳过
    // if (udp_get_data_->get_size() <= 0) {
    //   continue;
    // }
    //取出错误数据也跳过
    udp_get_data_->product(tmpdata);
    if (tmpdata.empty()) {
      LOG_ERROR("get data from udp is empty, continue");
      continue;
    }

    if(detect_1_finished==true){
      frame_cache_1.product(std::make_shared<cv::Mat>(tmpdata));
    }
    else if(detect_1_finished==false && detect_2_finished == true){
      frame_cache_2.product(std::make_shared<cv::Mat>(tmpdata));
    }
    else if(detect_1_finished==false && detect_2_finished == false && detect_3_finished == true){
      frame_cache_3.product(std::make_shared<cv::Mat>(tmpdata));
    }
    else if(detect_1_finished==false && detect_2_finished == false && detect_3_finished == false && detect_4_finished == true){
      frame_cache_4.product(std::make_shared<cv::Mat>(tmpdata));
    }
    usleep(10000);
  }
}

void thread_detect_1() {
  std::shared_ptr<cv::Mat> imgframe;
  std::vector<ObjectData> obj_vec;
  while (true) {
    imgframe = nullptr;
    obj_vec.clear();

    frame_cache_1.consumption(imgframe);
    // detect
    if (imgframe.get() == nullptr) {
      continue;
    }
    detect_1_finished = false;
    // LOG_INFO("start detect");
    cv::Mat tmp = *imgframe.get();

    auto start = std::chrono::high_resolution_clock::now();
    auto end = std::chrono::high_resolution_clock::now();

    dsp_detector1_.Detect(tmp, obj_vec);

    if (obj_vec.empty()) {
      detect_1_finished = true;
      continue;
    }

    auto track_results = dsp_detector1_.tracker_->update(obj_vec);

    for (const auto& result : obj_vec) {
      //测试只追人，而且只能有一个人
      printf("frame_id: %d \n",result.label);
      if(result.label == 0){
        //printf("x: %d  y: %d \n ",result.bbox.x,result.bbox.y);
	      sendMessage(result);
      }
    }



    auto time_used =
        std::chrono::duration_cast<std::chrono::duration<double>>(end - start);
    //LOG_INFO("detect output size = %f", obj_vec.size());
    // std::cout << "detect output size =" << obj_vec.size() << std::endl;
    // std::cout << "time_used = " << time_used.count() << std::endl;

    // drawResults(tmp, obj_vec,labels);
    // videoWriting.lock();
    // video_writer << tmp;
    // videoWriting.unlock();
    detect_1_finished = true;
    usleep(100);
  }
}

void thread_detect_2() {
  std::shared_ptr<cv::Mat> imgframe;
  std::vector<ObjectData> obj_vec;
  while (true) {
    imgframe = nullptr;
    obj_vec.clear();

    frame_cache_2.consumption(imgframe);

    // detect
    // LOG_INFO("start detect");

    if (imgframe.get() == nullptr) {
      continue;
    }
    detect_2_finished = false;
    cv::Mat tmp = *imgframe.get();

    dsp_detector2_.Detect(tmp, obj_vec);

    if (obj_vec.empty()) {
      detect_2_finished = true;
      continue;
    }

    auto track_results = dsp_detector2_.tracker_->update(obj_vec);

    for (const auto& result : obj_vec) {
      //测试只追人，而且只能有一个人
      printf("frame_id: %d \n",result.label);
      if(result.label == 0){
        sendMessage(result);
      }
    }

    // std::cout << "detect output size =" << obj_vec.size() << std::endl;
    // std::cout << "time_used = " << time_used.count() << std::endl;

    // drawResults(tmp, obj_vec,labels);
    // videoWriting.lock();
    // video_writer << tmp;
    // videoWriting.unlock();
    detect_2_finished = true;
    usleep(100);
  }
}

void thread_detect_3() {
  std::shared_ptr<cv::Mat> imgframe;
  std::vector<ObjectData> obj_vec;
  while (true) {
    imgframe = nullptr;
    obj_vec.clear();

    frame_cache_3.consumption(imgframe);
    if (imgframe.get() == nullptr) {
      continue;
    }
    // detect
    // LOG_INFO("start detect");
    detect_3_finished = false;
    cv::Mat tmp = *imgframe.get();

    dsp_detector3_.Detect(tmp, obj_vec);

    if (obj_vec.empty()) {
      detect_3_finished = true;      
      continue;
    }

    auto track_results = dsp_detector3_.tracker_->update(obj_vec);

    for (const auto& result : obj_vec) {
      //测试只追人，而且只能有一个人
      printf("frame_id: %d \n",result.label);
      if(result.label == 0){
        sendMessage(result);
      }
    }

    // std::cout << "detect output size =" << obj_vec.size() << std::endl;

    // drawResults(tmp, obj_vec,labels);
    // videoWriting.lock();
    // video_writer << tmp;
    // videoWriting.unlock();
    detect_3_finished = true;
    usleep(100);
  }
}

void thread_detect_4() {
  std::shared_ptr<cv::Mat> imgframe;
  std::vector<ObjectData> obj_vec;
  while (true) {
    imgframe = nullptr;
    obj_vec.clear();

    frame_cache_4.consumption(imgframe);
    if (imgframe.get() == nullptr) {
      continue;
    }
    // detect
    // LOG_INFO("start detect");
    detect_4_finished = false;
    cv::Mat tmp = *imgframe.get();

    dsp_detector4_.Detect(tmp, obj_vec);

    if (obj_vec.empty()) {
      detect_4_finished = true;
      continue;
    }

    auto track_results = dsp_detector4_.tracker_->update(obj_vec);

    for (const auto& result : obj_vec) {
      //测试只追人，而且只能有一个人
      printf("frame_id: %d \n",result.label);
      if(result.label == 0){
        sendMessage(result);
      }
    }

    // std::cout << "detect output size =" << obj_vec.size() << std::endl;

    // drawResults(tmp, obj_vec,labels);
    // videoWriting.lock();
    // video_writer << tmp;
    // videoWriting.unlock();
    detect_4_finished = true;
    usleep(100);

  }
}

// void thread_detect_5() {
//   while (true) {
//     std::shared_ptr<cv::Mat> imgframe;
//     std::vector<ObjectData> obj_vec;

//     frame_cache_5.consumption(imgframe);
//     if (imgframe.get() == nullptr) {
//       continue;
//     }
//     // detect
//     LOG_INFO("start detect");

//     dsp_detector5_.Detect(*imgframe.get(), obj_vec);
//     std::cout << "detect output size =" << obj_vec.size() << std::endl;
//     //要不要写入视频？
//   }
// }

// void thread_detect_6() {
//   while (true) {
//     std::shared_ptr<cv::Mat> imgframe;
//     std::vector<ObjectData> obj_vec;

//     frame_cache_6.consumption(imgframe);
//     if (imgframe.get() == nullptr) {
//       continue;
//     }
//     LOG_INFO("start detect");
//     // detect
//     dsp_detector6_.Detect(*imgframe.get(), obj_vec);
//     std::cout << "detect output size =" << obj_vec.size() << std::endl;
//     //要不要写入视频？
//   }
// }

void Deinit() {
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

  if (mavlink_thread_.joinable()) {
    mavlink_thread_.join();
  }
}

// static void drawResults(cv::Mat& img, const std::vector<ObjectData>& results,
//                         const std::vector<std::string>& labels) {
//   cv::cvtColor(img, img, cv::COLOR_BGR2RGB);
//   // LOG_INFO("results size:", results.size());
//   std::cout << "results size: " << results.size() << std::endl;
//   for (const auto& result : results) {
//     char t[256];
//     cv::rectangle(img,
//                   cv::Rect(result.bbox.x, result.bbox.y, result.bbox.width,
//                            result.bbox.height),
//                   cv::Scalar(0, 255, 0));

//     cv::Point position = cv::Point(result.bbox.x, result.bbox.y - 10);
//     // ToDo: label数量为0
//     // cv::putText(img, labels[result.label], position,
//     // cv::FONT_HERSHEY_COMPLEX,
//     //             0.8, cv::Scalar(0, 255, 0), 2, 0.3);
//   }
// }

static bool validateLabels(const char* name, const std::string& value) {
  if (!value.compare("")) {
    LOG_ERROR("you must provide label file");
    return false;
  }

  struct stat statbuf;
  if (0 == stat(value.c_str(), &statbuf)) {
    return true;
  }
}

DEFINE_string(labels,
              "/code/src/snpe-yolov7-inference/cfg/yolov5s_labels.txt",
              "Labels file for the yolov5 model");
DEFINE_validator(labels, &validateLabels);

DEFINE_double(confidence, 0.7, "confidence Threshold");
DEFINE_double(nms, 0.5, "nms Threshold");

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

  // std::vector<std::string> labels;
  std::ifstream in(FLAGS_labels);
  std::string line;

  if (!in.is_open()) {
    std::cerr << "Failed to open file: " << FLAGS_labels << std::endl;
    return 1;
  }

  while (getline(in, line)) {
    labels.push_back(line);
  }
  in.close();

  // LOG_INFO("Labels:", labels.size());
  std::cout << "labels: " << labels.size() << std::endl;
  std::shared_ptr<JsonReader> jsRead = std::make_shared<JsonReader>(file_path);

    //初始化信号量
  sem_init(&getMavlinkDataSem,0,0);

  //读取参数,涉及到数组
  ObjectDetectionConfig config;
  // Json::Value value;
  jsRead->parse_frome_json(config);

  std::shared_ptr<UdpGetData> data = std::make_shared<UdpGetData>();
  udp_get_data_ = data.get();

  if (!udp_get_data_->init()) {
    LOG_ERROR("Init UdpGetData failed");
    return -1;
  }

  dsp_detector1_.Init(config);
  dsp_detector1_.SetScoreThreshold(0.7f, 0.5f);

  // dsp_detector2_ = dsp.get();
  dsp_detector2_.Init(config);
  dsp_detector2_.SetScoreThreshold(0.7f, 0.5f);

  // dsp_detector3_ = dsp.get();
  dsp_detector3_.SetScoreThreshold(0.7f, 0.5f);
  dsp_detector3_.Init(config);

  // dsp_detector4_ = dsp.get();
  dsp_detector4_.SetScoreThreshold(0.7f, 0.5f);
  dsp_detector4_.Init(config);

  // dsp_detector5_ = dsp.get();
  dsp_detector5_.SetScoreThreshold(0.7f, 0.5f);
  dsp_detector5_.Init(config);

  // dsp_detector6_ = dsp.get();
  dsp_detector6_.SetScoreThreshold(0.7f, 0.5f);
  dsp_detector6_.Init(config);

  std::thread getdata_ = std::thread(&cbData);
  std::thread mavlinkRouter_ = std::thread(&mavlinkRouter_interface);
  detect_1_ = std::thread(&thread_detect_1);
  detect_2_ = std::thread(&thread_detect_2);
  detect_3_ = std::thread(&thread_detect_3);
  detect_4_ = std::thread(&thread_detect_4);
  mavlink_thread_ = std::thread(&mavlink_main);

  mavlinkRouter_.join();
  getdata_.join();
  Deinit();

  return 1;

  // std::shared_ptr<yolov5::ObjectDetection> detect1 =
  //     std::make_shared<yolov5::ObjectDetection>();

  // std::shared_ptr<yolov5::ObjectDetection> detect2 =
  //     std::make_shared<yolov5::ObjectDetection>();

  // //配置参数
  // detect1->Init(config);
  // detect1->SetScoreThreshold(FLAGS_confidence, FLAGS_nms);

  // detect2->Init(config);
  // detect2->SetScoreThreshold(FLAGS_confidence, FLAGS_nms);

  // cv::VideoCapture cap(video_name);
  // cv::Size sSize = cv::Size((int)cap.get(cv::CAP_PROP_FRAME_WIDTH),
  //                           (int)cap.get(cv::CAP_PROP_FRAME_HEIGHT));

  // auto fFps = (float)cap.get(cv::CAP_PROP_FPS);
  // cv::VideoWriter video_writer(
  //     "result.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), fFps,
  //     sSize);

  // cv::Mat img;

  // std::vector<yolov5::ObjectData> vec_res;
  // vec_res.reserve(100);

  // while (cap.read(img)) {
  //   cv::cvtColor(img, img, cv::COLOR_BGR2RGB);
  //   vec_res.clear();
  //   auto start = std::chrono::high_resolution_clock::now();
  //   detect1->Detect(img, vec_res);
  //   auto end = std::chrono::high_resolution_clock::now();
  //   auto time_used =
  //       std::chrono::duration_cast<std::chrono::duration<double>>(end -
  //       start);

  //   // LOG_INFO("time_used", time_used);
  //   std::cout << "time_used: " << time_used.count() << "s" << std::endl;

  //   drawResults(img, vec_res, labels);
  //   video_writer << img;
  // }
}
