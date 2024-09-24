#pragma once
#include <mutex>
#include <queue>

#include "fctty.hpp"
#include "mavlink.hpp"
// #include "utils.h"
#include "v2.0/common/mavlink_msg_camera_tracking_image_status.h"
#include "v2.0/mavlink_types.h"
#include "v2.0/standard/standard.h"
#include "v2.0/standard/version.h"

#define GCS_MAVLINK_DEFINE_COM MAVLINK_COMM_0
#define GCS_MAVLINK_DEFINE_VERSION MAVLINK_VERSION_2_0

#define FCTTYRECV_BUFFER_SIZE 2048
#define RECEIVE_NONBLOCK_WAITTIME_US 100
#define SERIAL_PORT_BUFFSIZE FCTTYRECV_BUFFER_SIZE

class GCS_MAVLINK {
 public:
  GCS_MAVLINK();

  void update();
  bool init(mavlink_channel_t mav_chan, uint8_t version) {
    init_mavlink_version_ = version;
    mav_chan_ = mav_chan;

    mavlink_status_t *status = mavlink_get_channel_status(mav_chan_);

    if (status) {
      if (init_mavlink_version_ == MAVLINK_VERSION_1_0) {
        status->flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
      }
    } else {
      init_ = false;
      return init_;
    }

    init_ = true;
    return init_;
  }
  int setup_uart(void);

  //   void send_message();
  void sendMessage();

  void receiveMessage() {
    mavlink_message_t msg;
    mavlink_status_t status;

    status.packet_rx_drop_count = 0;

    if (fd_ == -1) {
      if (mavlink_fd_ == -1) {
        mavlink_rcvlen_ = 0;
      } else {
        mavlink_rcvlen_ =
            FcTty_Recv(mavlink_fd_, mavlink_rcvbuf, FCTTYRECV_BUFFER_SIZE);
        // use mavlink fd when check if the serial port is mavlink port
      }
    } else {
      mavlink_rcvlen_ = (fd_, mavlink_rcvbuf, FCTTYRECV_BUFFER_SIZE);
    }

    if (mavlink_rcvlen_ > 0) {
      if (mavlink_rcvlen_ > SERIAL_PORT_BUFFSIZE) {
        mavlink_rcvlen_ = SERIAL_PORT_BUFFSIZE;
      }

      for (uint16_t i = 0; i < mavlink_rcvlen_; i++) {
        if (mavlink_parse_char(mav_chan_, mavlink_rcvbuf[i], &msg, &status)) {
          packetReceived(status, msg);
        }
      }
    }
  }

  uint8_t get_mavlink_active();

  uint8_t get_stateflag();

  void mavlink_lock_detect();

  void push_data(const mavlink_camera_tracking_image_status_t &msg) {
    std::lock_guard<std::mutex> lock(mtx_);
    msgs.push(msg);
  }

  void get_data(mavlink_camera_tracking_image_status_t &msg) {
    std::lock_guard<std::mutex> lock(mtx_);
    msg = msgs.front();
    msgs.pop();
  }

 private:
  void packetReceived(const mavlink_status_t &status, mavlink_message_t &msg);

  void handleMessage(mavlink_message_t *msg);

  std::mutex mtx_;
  std::queue<mavlink_camera_tracking_image_status_t> msgs;

  mavlink_channel_t mav_chan_;

  uint8_t mavlink_active_;
  uint8_t init_mavlink_version_;

  int fd_ = -1;
  bool init_ = false;
};