/**
 *  @file       SerialPort.h
 *  @author     李敏
 *  @created    2020/9/16
 *  @brief      A SerialPort.h header file.
 **/

#ifndef _SERIAL_PORT_H_
#define _SERIAL_PORT_H_

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <cerrno>
#include <string>

#ifdef __ANDROID__

#include <android/log.h>
#define LOGV(fmt, ...) __android_log_print(ANDROID_LOG_VERBOSE, "Serial-Port", fmt, ##__VA_ARGS__)
#define LOGD(fmt, ...) __android_log_print(ANDROID_LOG_DEBUG,   "Serial-Port", fmt, ##__VA_ARGS__)
#define LOGI(fmt, ...) __android_log_print(ANDROID_LOG_INFO,    "Serial-Port", fmt, ##__VA_ARGS__)
#define LOGW(fmt, ...) __android_log_print(ANDROID_LOG_WARN,    "Serial-Port", fmt, ##__VA_ARGS__)
#define LOGE(fmt, ...) __android_log_print(ANDROID_LOG_ERROR,   "Serial-Port", fmt, ##__VA_ARGS__)

#else

#include <cstdio>
#define LOGV(fmt, ...) printf("Jni [V] Serial-Port: " fmt "\n", ##__VA_ARGS__)
#define LOGD(fmt, ...) printf("Jni [D] Serial-Port: " fmt "\n", ##__VA_ARGS__)
#define LOGI(fmt, ...) printf("Jni [I] Serial-Port: " fmt "\n", ##__VA_ARGS__)
#define LOGW(fmt, ...) printf("Jni [W] Serial-Port: " fmt "\n", ##__VA_ARGS__)
#define LOGE(fmt, ...) printf("Jni [E] Serial-Port: " fmt "\n", ##__VA_ARGS__)

#endif // __ANDROID__

constexpr speed_t kBaudRateSpeedSet[] = {
#if defined(B50)
    B50,
#endif
#if defined(B75)
    B75,
#endif
#if defined(B110)
    B110,
#endif
#if defined(B134)
    B134,
#endif
#if defined(B150)
    B150,
#endif
#if defined(B200)
    B200,
#endif
#if defined(B300)
    B300,
#endif
#if defined(B600)
    B600,
#endif
#if defined(B1200)
    B1200,
#endif
#if defined(B1800)
    B1800,
#endif
#if defined(B2400)
    B2400,
#endif
#if defined(B4800)
    B4800,
#endif
#if defined(B9600)
    B9600,
#endif
#if defined(B19200)
    B19200,
#endif
#if defined(B38400)
    B38400,
#endif
#if defined(B57600)
    B57600,
#endif
#if defined(B115200)
    B115200,
#endif
#if defined(B230400)
    B230400,
#endif
#if defined(B460800)
    B460800,
#endif
#if defined(B500000)
    B500000,
#endif
#if defined(B576000)
    B576000,
#endif
#if defined(B921600)
    B921600,
#endif
#if defined(B1000000)
    B1000000,
#endif
#if defined(B1152000)
    B1152000,
#endif
#if defined(B1500000)
    B1500000,
#endif
#if defined(B2000000)
    B2000000,
#endif
#if defined(B2500000)
    B2500000,
#endif
#if defined(B3000000)
    B3000000,
#endif
#if defined(B3500000)
    B3500000,
#endif
#if defined(B4000000)
    B4000000,
#endif
};

constexpr long kBaudRateSet[] = {
#if defined(B50)
    50,
#endif
#if defined(B75)
    75,
#endif
#if defined(B110)
    110,
#endif
#if defined(B134)
    134,
#endif
#if defined(B150)
    150,
#endif
#if defined(B200)
    200,
#endif
#if defined(B300)
    300,
#endif
#if defined(B600)
    600,
#endif
#if defined(B1200)
    1200,
#endif
#if defined(B1800)
    1800,
#endif
#if defined(B2400)
    2400,
#endif
#if defined(B4800)
    4800,
#endif
#if defined(B9600)
    9600,
#endif
#if defined(B19200)
    19200,
#endif
#if defined(B38400)
    38400,
#endif
#if defined(B57600)
    57600,
#endif
#if defined(B115200)
    115200,
#endif
#if defined(B230400)
    230400,
#endif
#if defined(B460800)
    460800,
#endif
#if defined(B500000)
    500000,
#endif
#if defined(B576000)
    576000,
#endif
#if defined(B921600)
    921600,
#endif
#if defined(B1000000)
    1000000,
#endif
#if defined(B1152000)
    1152000,
#endif
#if defined(B1500000)
    1500000,
#endif
#if defined(B2000000)
    2000000,
#endif
#if defined(B2500000)
    2500000,
#endif
#if defined(B3000000)
    3000000,
#endif
#if defined(B3500000)
    3500000,
#endif
#if defined(B4000000)
    4000000,
#endif
};

constexpr int  kBaudRateSpeedSize = (sizeof(kBaudRateSpeedSet) / sizeof(speed_t));
constexpr long kMaxBaudRate = kBaudRateSet[kBaudRateSpeedSize - 1];

constexpr int kBit5 = 0;
constexpr int kBit6 = 1;
constexpr int kBit7 = 2;
constexpr int kBit8 = 3;

constexpr int kParityDisable = 0;
constexpr int kParityOdd = 1;
constexpr int kParityEven = 2;

#endif //_SERIAL_PORT_H_
