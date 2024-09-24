#include "fctty.hpp"

int FcttyOpen(const char *dev) {
  int fd = open(dev, O_RDWR | O_NOCTTY | O_NDELAY);
  if (fd < 0) {
    close(fd);
    return -1;
  }
  return fd;
}

int FcttySetup(int fd, int speed, int flow_ctrl, int databits, int stopbits,
               int parity) {
  //设置串口数据帧格式
  int speed_arr[] = {B115200, B19200, B9600, B4800, B2400, B1200, B300};
  int name_arr[] = {115200, 19200, 9600, 4800, 2400, 1200, 300};

  struct termios options = {0};

  // tcgetattr(fd,
  // &options)得到与fd指向对象的相关参数，并将它们保存于options,
  // 该函数还可以测试配置是否正确，该串口是否可用等。
  // 若调用成功，函数返回值为0，若调用失败，函数返回值为1.
  if (tcgetattr(fd, &options) != 0) {
#if REMINDER_INFO_ENABLED
    // fout << "Tcgetattr failed!" << std::endl;
    LOG_ERROR("Tcgetattr failed!");
#endif
    return -1;
  }

  //设置串口输入波特率和输出波特率
  for (int i = 0; i < sizeof(speed_arr) / sizeof(int); i++) {
    if (speed == name_arr[i]) {
      cfsetispeed(&options, speed_arr[i]);  // 设置输入波特率
      cfsetospeed(&options, speed_arr[i]);  // 设置输出波特率
      break;
    }
  }

#if 0
    //设置数据流控制
    switch(flow_ctrl)
    {
        case 0 ://不使用流控制  
            options.c_cflag &= ~CRTSCTS;  
            break;     
        
        case 1 ://使用硬件流控制  
            options.c_cflag |= CRTSCTS;  
            break;  
        case 2 ://使用软件流控制  
            options.c_cflag |= IXON|IXOFF|IXANY;  
            break;
        default:
            options.c_cflag &= ~CRTSCTS;  
            break; 
    }
#endif

  //设置数据位, 屏蔽其他标志位
  options.c_cflag &= ~CSIZE;
  switch (databits) {
    case 5:
      options.c_cflag |= CS5;
      break;
    case 6:
      options.c_cflag |= CS6;
      break;
    case 7:
      options.c_cflag |= CS7;
      break;
    case 8:
      options.c_cflag |= CS8;
      break;
    default:
#if REMINDER_INFO_ENABLED
      fprintf(stderr, "Unsupported data size\n");
#endif
      return (-1);
  }

  //设置校验位
  switch (parity) {
    case 'n':
    case 'N':  //无奇偶校验位.
    {
      options.c_cflag &= ~PARENB;
      options.c_iflag &= ~INPCK;
      break;
    }
    case 'o':
    case 'O':  //设置为奇校验
    {
      options.c_cflag |= (PARODD | PARENB);
      options.c_iflag |= INPCK;
      break;
    }
    case 'e':
    case 'E':  //设置为偶校验
    {
      options.c_cflag |= PARENB;
      options.c_cflag &= ~PARODD;
      options.c_iflag |= INPCK;
      break;
    }
    case 's':
    case 'S':  //设置为空格
    {
      options.c_cflag &= ~PARENB;
      options.c_cflag &= ~CSTOPB;
      break;
    }
    default:
#if REMINDER_INFO_ENABLED
      fprintf(stderr, "Unsupported parity\n");
#endif
      return (-1);
  }

  // 设置停止位
  switch (stopbits) {
    case 1:
      options.c_cflag &= ~CSTOPB;
      break;
    case 2:
      options.c_cflag |= CSTOPB;
      break;
    default:
#if REMINDER_INFO_ENABLED
      fprintf(stderr, "Unsupported stop bits\n");
#endif
      return (-1);
  }

#if 0
        //修改控制模式，保证程序不会占用串口
        options.c_cflag |= CLOCAL;
        //修改控制模式，使得能够从串口中读取输入数据  
        options.c_cflag |= CREAD;
        //修改输出模式，原始数据输出  
        options.c_oflag &= ~(OPOST);  
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);  
        //设置等待时间和最小接收字符  
        options.c_cc[VTIME] = 1; /* 读取一个字符等待1*(1/10)s */    
        options.c_cc[VMIN] = 1; /* 读取字符的最少个数为1 */
#else
  // 一般必设置的标志
  options.c_cflag |= (CLOCAL | CREAD);
  options.c_oflag &= ~(OPOST);
  options.c_lflag &= ~(ECHO | ICANON | IEXTEN | ISIG);
  options.c_iflag &= ~(ICRNL | INLCR | IGNCR | IXON | IXOFF | IXANY);
#endif
  //如果发生数据溢出，接收数据，但是不再读取 刷新收到的数据但是不读
  tcflush(fd, TCIFLUSH);

  //激活配置 (将修改后的termios数据设置到串口中）
  if (tcsetattr(fd, TCSANOW, &options) != 0) {
#if REMINDER_INFO_ENABLED
    // fout << "com set error!" << std::endl;
    LOG_ERROR("com set error!");
#endif
    return (-1);
  }

  return (0);
}

int FcTty_Init(const char *dev) {
  int fd = FcttyOpen(dev);
  if (fd <= 0) {
    LOG_ERROR("cannot open dev failed: %s", strerror(errno));
    return -1;
  } else {
    LOG_INFO("open dev success");
  }

  if (FcttySetup(fd, 115200, 0, 8, 1, 'N') != 0) {
    FcTty_Close(fd);
    return -1;
  } else {
    Fctty_Flush(fd);
    return fd;
  }
};

int FcTty_Recv(int fd, uint8_t *rev_buf, int bufsize) {
  fd_set fs_read;
  FD_ZERO(&fs_read);
  FD_SET(fd, &fs_read);

  struct timeval time;
  time.tv_sec = 0;
  time.tv_usec = RECEIVE_NONBLOCK_WAITTIME_US;

  // 使用select实现串口的多路通信
  if (select(fd + 1, &fs_read, NULL, NULL, &time) > 0) {
    return read(fd, rev_buf, bufsize);
  } else {
    return -1;
  }
};

int FcTty_Send(int fd, const uint8_t *sendbuf, int bufsize) {
  if (write(fd, sendbuf, bufsize) == bufsize) {
    return bufsize;
  } else {
    tcflush(fd, TCOFLUSH);
    return -1;
  }
};

void FcTty_Close(int fd) {
  if (fd > 0) close(fd);
};

void Fctty_Flush(int fd) { tcflush(fd, TCIOFLUSH); };

void detect_tty() {
  uint8_t dev_num = 0;
  uint8_t dev_readed_line = 0;
  double detect_startms, detect_lastms;

  if (mavlink_port_locked_ || detect_runing ||
      (clock_ms() - detect_lastms <= 100)) {
    return;
  }

  //   detect_lastms = clock_ms();
  detect_runing = true;

  int fd = -1;

  if (dev_readed_line == 0) {
    std::ifstream acmfp("ttyACM.txt");
    std::string line;
    if (acmfp) {
      std::string line;
      while (std::getline(acmfp, line))  // line中不包括每行的换行符
      {
        int cpylen = sizeof(line);
        if (cpylen > 35) {
          cpylen = 35;
        }
        line.copy(dev_read[dev_readed_line], cpylen);
        std::cout << dev_read[dev_readed_line] << std::endl;
        dev_readed_line++;
      }
    }
  }

  if (mavlink_fd_ == -1) {
    if (dev_readed_line > 0) {
      uint8_t index = dev_num % dev_readed_line;
      fd = FcTty_Init(dev_read[index]);

      if (fd > 0) {
        mavlink_fd_ = fd;
        detect_startms = clock_ms();
        // fout << "opening: ";
        // fout << dev_read[index];
        // fout << " successed." << std::endl;
        LOG_INFO("opening device successed");
        // std::cout << dev_read[index] << ";  ";
      } else {
        dev_num++;
      }
    } else {
      uint8_t index = dev_num % 13;
      fd = FcTty_Init(dev_name[index]);

      if (fd > 0) {
        mavlink_fd_ = fd;
        detect_startms = clock_ms();
        // fout << "opening: ";
        // fout << dev_name[index];
        // fout << " successed." << std::endl;
        LOG_INFO("opening device successed");
        // std::cout << dev_name[index] << ", ";
      } else {
        dev_num++;
      }
    }
  } else {
    if (clock_ms() - detect_startms >= 500) {
      // detect time out, redetect.
      FcTty_Close(mavlink_fd_);
      mavlink_fd_ = -1;
      dev_num++;
    }
  }

  detect_runing = false;
}
