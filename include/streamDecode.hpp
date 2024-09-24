#pragma once

/**
 * @author Chunhu.Chen
 */

#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <gst/gst.h>
#include <opencv2/imgproc/types_c.h>

#include <iostream>
#include <map>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>

#include "safeQueue.hpp"
#include "yolo.hpp"

typedef struct _PipelineData {
  std::shared_ptr<GstElement> pipeline;
  std::shared_ptr<GstElement> source;
  std::shared_ptr<GstElement> main_capsfilter;

  std::shared_ptr<GstElement> videoDepay;
  std::shared_ptr<GstElement> videoParse;
  std::shared_ptr<GstElement> h264dec;
  std::shared_ptr<GstElement> transform;
  std::shared_ptr<GstElement> sink;
} PipelineData;

// typedef struct _FrameProcessData {
//   uint32_t frameId;
//   int interval = 25;
//   //    shared_ptr<ModelInference> inference;
//   shared_ptr<DecodeQueue> blockQueue;
//   string streamName;
//   int StreamId;
// } FrameProcessData;

class StreamDecode {
 public:
  StreamDecode(std::string streamtype, std::string rtspUrl);
  ~StreamDecode();
  int Initialization(std::shared_ptr<DecodeQueue> &queue);
  void UnInitialization();
  void DecodeAndInference();
  void SetRtspUrl(std::string url);

  void SetSkipFrame(int interval);
  void SetStreamName(std::string name);
  void SetStreamId(int uuid);

  static void OnPadAdd(GstElement *element, GstPad *pad, gpointer data);
  static GstFlowReturn OnAppsinkNewSample(GstElement *appsink,
                                          gpointer user_data);
  void Stop();

 protected:
  static void UnRefElement(GstElement *elem);

 private:
  PipelineData data_;
  std::shared_ptr<GstBus> bus_ = nullptr;

  std::shared_ptr<yolov5::ObjectDetection> detect_{nullptr};

  bool terminate_ = FALSE;
  std::string rtspUrl_;
  std::string StreamType;
  //   FrameProcessData *frameProcess_ = nullptr;

  //本地相机流
  int gst_camera_pipeline_init();
  // rtsp相机流
  int gst_rtsp_pipeline_init();

  TsProAndCon<cv::Mat> frame_cache_1;
  TsProAndCon<cv::Mat> frame_cache_2;
  TsProAndCon<cv::Mat> frame_cache_3;
  TsProAndCon<cv::Mat> frame_cache_4;
  TsProAndCon<cv::Mat> frame_cache_5;
  TsProAndCon<cv::Mat> frame_cache_6;

  TsProAndCon<cv::Mat> show_frame_cache_1;
  TsProAndCon<cv::Mat> show_frame_cache_2;
  TsProAndCon<cv::Mat> show_frame_cache_3;
  TsProAndCon<cv::Mat> show_frame_cache_4;
  TsProAndCon<cv::Mat> show_frame_cache_5;
  TsProAndCon<cv::Mat> show_frame_cache_6;
};
