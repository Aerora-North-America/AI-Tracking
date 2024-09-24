
#pragma once

#define MAVLINK_SEPARATE_HELPERS
#define MAVLINK_SEND_UART_BYTES(chan, buf, len) comm_send_buffer(chan, buf, len)
// allow five telemetry ports
#define MAVLINK_COMM_NUM_BUFFERS 5

/*
  The MAVLink protocol code generator does its own alignment, so
  alignment cast warnings can be ignored
 */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-align"
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"

#include "v2.0/standard/version.h"
#define MAVLINK_MAX_PAYLOAD_LEN 255
// #include "v2.0/mavlink_helpers.h"
#include "v2.0/mavlink_types.h"

extern mavlink_system_t mavlink_system;

static inline bool valid_channel(mavlink_channel_t chan) {
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wtautological-constant-out-of-range-compare"
  return chan < MAVLINK_COMM_NUM_BUFFERS;
#pragma clang diagnostic pop
}

void comm_send_ch(mavlink_channel_t chan, uint8_t ch);

void comm_send_buffer(mavlink_channel_t chan, const uint8_t *buf, uint8_t len);
uint8_t comm_receive_ch(mavlink_channel_t chan);

uint16_t comm_get_available(mavlink_channel_t chan);
uint16_t comm_get_txspace(mavlink_channel_t chan);
bool comm_is_idle(mavlink_channel_t chan);

#define MAVLINK_USE_CONVENIENCE_FUNCTIONS
#include "v2.0/standard/mavlink.h"

enum {
  MAVLINK_VERSION_1_0 = 1,
  MAVLINK_VERSION_2_0,
};

enum {
  FMU_1_NODE_ID = 1,
  FMU_2_NODE_ID,

  GEM_1_NODE_ID = 10,

  ESC_1_NODE_ID = 21,
  ESC_2_NODE_ID,
  ESC_3_NODE_ID,
  ESC_4_NODE_ID,
  ESC_5_NODE_ID,
  ESC_6_NODE_ID,

  GIMBAL_1_NODE_ID = 40,
  GIMBAL_2_NODE_ID,

  MOUNT_1_NODE_ID = 50,
  PLND_NODE_ID = 51,
};

// static double clockbase_ms = 0;

#pragma GCC diagnostic pop