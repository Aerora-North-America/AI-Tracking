#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <condition_variable>
#include <cstring>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>

#include "Logger.h"
#include "utils.h"

#define MAX_UDP_PACKET_SIZE 65507
#define HEADER_SIZE 2

// int save_to_file = 0;

class UdpGetData {
 public:
  ~UdpGetData() {
    if (data_thread_.joinable()) {
      data_thread_.join();
    }
    close(sock);
  }

  bool init() {
    sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
      LOG_ERROR("failed to create socket: %s", strerror(errno));
      return false;
    }
    LOG_INFO("Socket created successfully. Socket descriptor : %f", sock);

    addr.sin_family = AF_INET;
    addr.sin_port = htons(9001);
    addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
      //   std::cerr << "Failed to bind socket: " << strerror(errno) <<
      //   std::endl;
      LOG_ERROR("failed to bind socket: %s", strerror(errno));
      close(sock);
      return false;
    }

    data_thread_ = std::thread(&UdpGetData::get_data, this);

    return true;
  }

  void get_data() {
    std::vector<uint8_t> y_plane(WIDTH * HEIGHT);
    std::vector<uint8_t> uv_plane(WIDTH * HEIGHT / 2);
    size_t y_offset = 0;
    size_t uv_offset = 0;

    std::vector<uint8_t> buffer(MAX_UDP_PACKET_SIZE);

    while (true) {
      ssize_t recv_size;
      try {
        recv_size =
            recvfrom(sock, buffer.data(), buffer.size(), 0, nullptr, nullptr);
        if (recv_size < 0) {
          std::cerr << "Failed to receive packet: " << strerror(errno)
                    << std::endl;
        }
      } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
      }

      char data_type = buffer[0];
      bool is_end = buffer[1];
      cv::Mat img(HEIGHT, WIDTH, CV_8UC3);
      //   cv::Mat img;
      size_t data_size = recv_size - HEADER_SIZE;

      //内存越界

      if (data_type == 'Y') {
        std::memcpy(y_plane.data() + y_offset, buffer.data() + HEADER_SIZE,
                    data_size);
        y_offset += data_size;

        // std::cout << "y_offset: " << y_offset << std::endl;
      } else if (data_type == 'U') {
        std::memcpy(uv_plane.data() + uv_offset, buffer.data() + HEADER_SIZE,
                    data_size);
        uv_offset += data_size;
      }

      if (is_end && data_type == 'Y') {
        y_offset = 0;
      }
      if (is_end && data_type == 'U') {
        uv_offset = 0;
        y_offset = 0;
        process_frame(y_plane, uv_plane, img);
        produce(img);
        usleep(100);
      }

      //   return img;
    }
  }

  void process_frame(std::vector<uint8_t> &y_plane,
                     std::vector<uint8_t> &uv_plane, cv::Mat &bgr) {
    //将uv_plane拆分出来

    cv::Mat yPlane(HEIGHT, WIDTH, CV_8UC1, y_plane.data());

    cv::Mat uvPlane(HEIGHT / 2, WIDTH / 2, CV_8UC2, uv_plane.data());

    // std::vector<cv::Mat> yuvplane = {yPlane, uPlane, vPlane};
    // // cv::Mat yuvMat;
    // cv::merge(yuvplane, bgr);
    // cv::cvtColor(bgr, bgr, cv::COLOR_YUV2RGB_NV12);
    // auto start = std::chrono::system_clock::now();

    // ToDo这个函数会把cpu占用拉高
    cv::cvtColorTwoPlane(yPlane, uvPlane, bgr, cv::COLOR_YUV2RGB_NV12);
  }

  void produce(const cv::Mat &img) {
    // control cache buffer size
    while (date_queue.size() > 10) {
      date_queue.pop();
    }

    mtx.lock();

    date_queue.push(img);
    mtx.unlock();
  }

  void product(cv::Mat &img) {
    if (date_queue.empty()) {
      // LOG_ERROR("data buffer is empty");
      return;
    }

    // std::cout << "date queue size:" << date_queue.size() << std::endl;
    mtx.lock();
    img = date_queue.front().clone();
    date_queue.pop();
    mtx.unlock();
  }

  int get_size() { return date_queue.size(); }

 private:
  void Separate(const std::vector<uint8_t> uv_plane,
                std::vector<uint8_t> &u_plane, std::vector<uint8_t> v_plane) {
    for (int i = 0; i < HEIGHT / 2; i++) {
      for (int j = 0; j < WIDTH / 2; j++) {
        u_plane[i * (WIDTH / 2) + j] = uv_plane[i * WIDTH + 2 * j];
        v_plane[i * (WIDTH / 2) + j] = uv_plane[i * WIDTH + 2 * j + 1];
      }
    }
  }

  int sock = -1;
  struct sockaddr_in addr;

  //   TsProAndCon<cv::Mat> date_queue;
  std::queue<cv::Mat> date_queue;
  std::mutex mtx;

  std::thread data_thread_;
};
