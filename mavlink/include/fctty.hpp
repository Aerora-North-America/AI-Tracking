#pragma once
#include <errno.h> /*错误号定义*/
#include <fcntl.h> /*文件控制定义*/
#include <stdint.h>
#include <stdio.h>  /*标准输入输出的定义*/
#include <stdlib.h> /*标准函数库定义*/
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h> /*PPSIX 终端控制定义*/
#include <unistd.h>  /*UNIX 标准函数定义*/

#include <fstream>
#include <iomanip>
#include <iostream>
#include <queue>
#include <sstream>

#include "Logger.h"

#define REMINDER_INFO_ENABLED 1
#define FILE_SETW std::setw(16)

#define RECEIVE_NONBLOCK_WAITTIME_US 100
#define FCTTYRECV_BUFFER_SIZE 2048

static double clockbase_ms = 0;
// extern int temp_cali_start, temp_cali_stop, temp_cali_count;

extern uint8_t mavlink_rcvbuf[FCTTYRECV_BUFFER_SIZE];
extern int16_t mavlink_rcvlen_;
extern int mavlink_fd_;
extern bool mavlink_port_locked_;

#define DEV_DEFINE_NUM 13

static char dev_name[13][35] = {
    "/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyACM2",  "/dev/ttyACM3",
    "/dev/ttyACM4", "/dev/ttyACM5", "/dev/ttyACM6",  "/dev/ttyACM7",
    "/dev/ttyACM8", "/dev/ttyACM9", "/dev/ttyACM10", "/dev/ttyACM11",
    "/dev/ttyACM12"

};

static char dev_read[DEV_DEFINE_NUM][35] = {0};

static uint8_t dev_readed_line = 0;

static uint8_t dev_num = 0;

static double detect_startms, detect_lastms;

static bool detect_runing = false;

int FcTty_Init(const char *devname);
int FcTty_Recv(int fd, uint8_t *rcv_buf, int buf_size);
int FcTty_Send(int fd, const uint8_t *send_buf, int buf_size);
void FcTty_Close(int fd);
void Fctty_Flush(int fd);

void detect_tty(void);

static double clock_ms() {
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  double result_ms = (t.tv_sec * 1000) + (t.tv_nsec * 1e-6);
  return result_ms - clockbase_ms;
}