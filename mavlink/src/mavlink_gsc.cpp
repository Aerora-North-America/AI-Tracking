#include "../include/mavlink_gcs.hpp"

#ifdef MAVLINK_SEPARATE_HELPERS
#include "v2.0/mavlink_helpers.h"
#endif

bool mavlink_port_locked_ = false;
int mavlink_fd_ = -1;

uint8_t mavlink_rcvbuf[FCTTYRECV_BUFFER_SIZE] = {0};
int16_t mavlink_rcvlen_ = 0;

mavlink_system_t mavlink_system = {29, 1};

GCS_MAVLINK::GCS_MAVLINK() {
  mavlink_active_ = 0;
  init_mavlink_version_ = MAVLINK_VERSION_2_0;
  fd_ = -1;
  init_ = false;
}

void GCS_MAVLINK::update() {
  if (!init_) {
    init(GCS_MAVLINK_DEFINE_COM, GCS_MAVLINK_DEFINE_VERSION);
  }
  mavlink_lock_detect();

  receiveMessage();

  if (mavlink_port_locked_) {
    sendMessage();
  }
}

void GCS_MAVLINK::sendMessage() {
  if (!msgs.empty()) {
    auto msg = msgs.front();
    msgs.pop();
    mavlink_msg_camera_tracking_image_status_send_struct(GCS_MAVLINK_DEFINE_COM,
                                                         &msg);
  }
}

// bool GCS_MAVLINK::init(mavlink_channel_t mav_chan, uint8_t version) {
//   init_mavlink_version_ = version;
//   mav_chan_ = mav_chan;

//   mavlink_status_t *status = mavlink_get_channel_status(mav_chan_);

//   if (status) {
//     if (init_mavlink_version_ == MAVLINK_VERSION_1_0) {
//       status->flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
//     }
//   } else {
//     init_ = false;
//     return init_;
//   }

//   init_ = true;
//   return init_;
// }

// void GCS_MAVLINK::receiveMessage() {
//   mavlink_message_t msg;
//   mavlink_status_t status;

//   status.packet_rx_drop_count = 0;

//   if (fd_ == -1) {
//     if (mavlink_fd_ == -1) {
//       mavlink_rcvlen_ = 0;
//     } else {
//       mavlink_rcvlen_ =
//           FcttyRecv(mavlink_fd_, mavlink_rcvbuf, FCTTYRECV_BUFFER_SIZE);
//       // use mavlink fd when check if the serial port is mavlink port
//     }
//   } else {
//     mavlink_rcvlen_ = (fd_, mavlink_rcvbuf, FCTTYRECV_BUFFER_SIZE);
//   }

//   if (mavlink_rcvlen_ > 0) {
//     if (mavlink_rcvlen_ > SERIAL_PORT_BUFFSIZE) {
//       mavlink_rcvlen_ = SERIAL_PORT_BUFFSIZE;
//     }

//     for (uint16_t i = 0; i < mavlink_rcvlen_; i++) {
//       if (mavlink_parse_char(mav_chan_, mavlink_rcvbuf[i], &msg, &status)) {
//         packetReceived(status, msg);
//       }
//     }
//   }
// }

void GCS_MAVLINK::packetReceived(const mavlink_status_t &status,
                                 mavlink_message_t &msg) {
  if (msg.msgid != MAVLINK_MSG_ID_RADIO_STATUS) {
    mavlink_active_ |= (1U << (mav_chan_ - MAVLINK_COMM_0));
  }

  if (!(status.flags & MAVLINK_STATUS_FLAG_IN_MAVLINK1) &&
      (status.flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) &&
      init_mavlink_version_ == MAVLINK_VERSION_2_0) {
    // MAVLINK2
    mavlink_status_t *cstatus = mavlink_get_channel_status(mav_chan_);
    if (cstatus != nullptr) {
      cstatus->flags &= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
    }
  }
  handleMessage(&msg);
}

void GCS_MAVLINK::handleMessage(mavlink_message_t *msg) {
  if (msg->msgid == MAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS) {
    mavlink_camera_tracking_image_status_t Rapacket;
    mavlink_msg_camera_tracking_image_status_decode(msg, &Rapacket);

    auto status = Rapacket.tracking_status;
    auto mode = Rapacket.tracking_mode;
  }
}

void GCS_MAVLINK::mavlink_lock_detect() {
  if (!mavlink_port_locked_) {
    detect_tty();
  }

  //这是正确的mavlink 串口
  fd_ = mavlink_fd_;
  mavlink_port_locked_ = true;
}

void comm_send_ch(mavlink_channel_t chan, uint8_t ch) {
  if (!valid_channel(chan)) {
    return;
  }
  if (mavlink_fd_) {
    FcTty_Send(mavlink_fd_, &ch, 1);
  }
}

void comm_send_buffer(mavlink_channel_t chan, const uint8_t *buf, uint8_t len) {
  if (!valid_channel(chan)) {
    return;
  }

  if (mavlink_fd_) {
    FcTty_Send(mavlink_fd_, buf, len);
  }
}
