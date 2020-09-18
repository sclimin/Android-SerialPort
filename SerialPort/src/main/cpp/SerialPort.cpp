/**
 *  @file       SerialPort.cpp
 *  @author     李敏
 *  @created    2020/9/15
 *  @brief      A SerialPort.cpp source file.
 **/
#include "SerialPort.h"
#include <jni.h>

static struct SerialPortOffsets {
    jclass mClass;
    jfieldID mNativeInstance;
} gSerialPortOffsets;

struct SerialPortContext {
    int mFd;
    std::string mDevPath;
    long mBaudRate;
    int mParity;
    int mBits;
    bool mSendStopBits;
    bool mHardwareFlow;
    bool mSoftwareFlow;
    bool mNonBlocking;

    SerialPortContext(const char *dev, long baudRate, int parity, int bits, bool sendStopBits, bool hardwareFlow,
                      bool softwareFlow, bool nonBlocking);

    ~SerialPortContext();

    bool open(std::string &err);

    void close();

    bool isOpened() const;
};

SerialPortContext::SerialPortContext(const char *dev, long baudRate, int parity, int bits, bool sendStopBits,
                                     bool hardwareFlow,
                                     bool softwareFlow, bool nonBlocking) :
    mFd(-1),
    mDevPath(dev),
    mBaudRate(baudRate),
    mParity(parity),
    mBits(bits),
    mSendStopBits(sendStopBits),
    mHardwareFlow(hardwareFlow),
    mSoftwareFlow(softwareFlow),
    mNonBlocking(nonBlocking) {
}

SerialPortContext::~SerialPortContext() {
    if (mFd > 0) {
        ::close(mFd);
        mFd = 0;
    }
}

bool SerialPortContext::open(std::string &err) {
    struct termios info {0};

    int fd = ::open(mDevPath.c_str(), O_RDWR | O_NONBLOCK | O_NOCTTY);
    if (fd >= 0) {
        int flags = fcntl(fd, F_GETFL);
        if (flags < 0) {
            LOGW("Get Flags Error (%d).", errno);
        }

        int newFlags = flags;
        newFlags &= ~O_NONBLOCK;
        if (mNonBlocking) {
            newFlags |= O_NONBLOCK;
        }

        if (newFlags != flags && ::fcntl(fd, F_SETFL, newFlags) < 0) {
            LOGW("Set Flags Error (%d).", errno);
        }
    }
    else {
        auto errStr = strerror(errno);
        if (errStr) {
            err = errStr;
        }

        if (err.empty()) {
            err = std::to_string(errno);
        }

        LOGE("Open Serial Port (%s) Failed: %s", mDevPath.c_str(), err.c_str());
        return false;
    }

    if (::tcgetattr(fd, &info) != 0) {
        LOGE("Get Attr Error (%d).", errno);
        ::close(fd);
        return false;
    }

    info.c_cflag = CS8 | CLOCAL | CREAD;
    // Disable output processing, including messing with end-of-line characters.
    info.c_oflag &= ~OPOST;
    info.c_iflag = IGNPAR;
    info.c_lflag = 0; /* turn of CANON, ECHO*, etc */
    /* no timeout but request at least one character per read */
    info.c_cc[VTIME] = 0;
    info.c_cc[VMIN] = 1;

    // Set speed.
    for (int i = kBaudRateSpeedSize - 1; i >= 0; i--) {
        if (mBaudRate >= kBaudRateSet[i]) {
            cfsetspeed(&info, kBaudRateSpeedSet[i]);
            break;
        }
    }

    // Set Parity Mode
    info.c_cflag &= ~(PARENB | PARODD);
    if (mParity == kParityEven || mParity == kParityOdd) {
        info.c_cflag |= (mParity == kParityOdd ? (PARENB | PARODD) : PARENB);
    }

    // Set Bits
    switch (mBits) {
        case kBit5:
            info.c_cflag = (info.c_cflag & ~CSIZE) | CS5;
            break;
        case kBit6:
            info.c_cflag = (info.c_cflag & ~CSIZE) | CS6;
            break;
        case kBit7:
            info.c_cflag = (info.c_cflag & ~CSIZE) | CS7;
            break;
        case kBit8:
        default:
            info.c_cflag = (info.c_cflag & ~CSIZE) | CS8;
            break;
    }

    // Set Stop Bits
    if (mSendStopBits) {
        info.c_cflag |= CSTOPB;
    }
    else {
        info.c_cflag &= ~CSTOPB;
    }

    // Set Hardware Flow
    if (mHardwareFlow) {
        info.c_cflag |= CRTSCTS;
    }
    else {
        info.c_cflag &= ~CRTSCTS;
    }

    // Set Software Flow
    if (mSoftwareFlow) {
        info.c_iflag |= IXON | IXOFF;
    }
    else {
        info.c_iflag &= ~(IXON | IXOFF | IXANY);
    }

    if (tcsetattr(fd, TCSANOW, &info) != 0) {
        LOGE("Set Attr Error (%d).", errno);
        ::close(fd);
        return false;
    }

    ::tcflush(fd, TCIOFLUSH);
    mFd = fd;
    LOGI("Serial Port Opened(fd=%d): %s", mFd, mDevPath.c_str());
    return true;
}

void SerialPortContext::close() {
    if (mFd > 0) {
        ::close(mFd);
        mFd = -1;
    }
}

bool SerialPortContext::isOpened() const {
    return mFd > 0;
}

// -------------------------------------------------------------------------------------
//                                  JNI
// -------------------------------------------------------------------------------------
static bool getExceptionSummary(JNIEnv *env, jthrowable exception, std::string &result) {

    jclass exceptionClass = env->GetObjectClass(exception);

    jmethodID classGetNameMethod = env->GetMethodID(exceptionClass, "getName", "()Ljava/lang/String;");

    auto classNameStr = (jstring) env->CallObjectMethod(exception, classGetNameMethod);

    if (classNameStr == nullptr) {
        env->ExceptionClear();
        result = "<error getting class name>";
        return false;
    }

    auto classNameChars = env->GetStringUTFChars(classNameStr, nullptr);

    if (classNameChars == nullptr) {
        env->ExceptionClear();
        result = "<error getting class name UTF-8>";
        return false;
    }

    result += classNameChars;

    env->ReleaseStringUTFChars(classNameStr, classNameChars);

    jmethodID getMessage = env->GetMethodID(exceptionClass, "getMessage", "()Ljava/lang/String;");
    auto messageStr = (jstring) env->CallObjectMethod(exception, getMessage);

    if (messageStr == nullptr) {
        return true;
    }

    result += ": ";

    auto messageChars = env->GetStringUTFChars(messageStr, nullptr);
    if (messageChars != nullptr) {
        result += messageChars;
        env->ReleaseStringUTFChars(messageStr, messageChars);
    }
    else {
        result += "<error getting message>";
        env->ExceptionClear();
    }

    return true;
}

static bool jniThrowException(JNIEnv *env, const char *className, const char *msg) {
    if (env->ExceptionCheck()) {
        jthrowable exception = env->ExceptionOccurred();

        if (exception != nullptr) {
            std::string text;
            getExceptionSummary(env, exception, text);
            LOGW("Discarding pending exception (%s) to throw %s", text.c_str(), className);
        }
    }

    jclass exceptionClass = env->FindClass(className);
    if (exceptionClass == nullptr) {
        LOGE("Unable to find exception class %s", className);
        return false;
    }

    if (env->ThrowNew(exceptionClass, msg) != JNI_OK) {
        LOGE("Failed throwing '%s' '%s'", className, msg);
        return false;
    }
    return true;
}

static jlong SerialPort_create(JNIEnv *env, jclass cls, jstring dev, jlong baudRate, jint parity, jint bits,
                               jboolean sendStopBits, jboolean hardwareFlow, jboolean softwareFlow,
                               jboolean nonBlocking) {
    LOGD("SerialPort.create: %p", cls);
    auto devPath = env->GetStringUTFChars(dev, nullptr);
    auto ctx = new SerialPortContext(
        devPath, baudRate, parity, bits, sendStopBits == JNI_TRUE, hardwareFlow == JNI_TRUE,
        softwareFlow == JNI_TRUE, nonBlocking == JNI_TRUE);
    env->ReleaseStringUTFChars(dev, devPath);
    return reinterpret_cast<jlong>(ctx);
}

static void SerialPort_destroy(JNIEnv *env, jclass cls, jlong instance) {
    LOGD("SerialPort.destroy: %p", cls);
    delete reinterpret_cast<SerialPortContext *>(instance);
}

static void SerialPort_open(JNIEnv *env, jobject obj) {
    LOGD("SerialPort.open");
    auto ctx = reinterpret_cast<SerialPortContext *>(
        env->GetLongField(obj, gSerialPortOffsets.mNativeInstance));
    std::string openErr;
    if (!ctx->open(openErr)) {
        std::string err = "Invalid Serial Port: ";
        err += ctx->mDevPath;
        err += ", Error Msg or Errno: ";
        err += openErr;
        jniThrowException(env, "java/io/IOException", err.c_str());
    }
}

static jboolean SerialPort_isOpened(JNIEnv *env, jobject obj) {
    LOGD("SerialPort.isOpened");
    auto ctx = reinterpret_cast<SerialPortContext *>(
        env->GetLongField(obj, gSerialPortOffsets.mNativeInstance));
    return ctx->isOpened() ? JNI_TRUE : JNI_FALSE;
}

static void SerialPort_close(JNIEnv *env, jobject obj) {
    LOGD("SerialPort.close");
    auto ctx = reinterpret_cast<SerialPortContext *>(
        env->GetLongField(obj, gSerialPortOffsets.mNativeInstance));
    ctx->close();
}

inline SerialPortContext *checkContextAndThrowException(JNIEnv *env, jobject obj) {
    auto ctx = reinterpret_cast<SerialPortContext *>(
        env->GetLongField(obj, gSerialPortOffsets.mNativeInstance));
    if (ctx->isOpened()) {
        return ctx;
    }
    std::string err = "Serial port (";
    err += ctx->mDevPath;
    err += ") is not open.";
    jniThrowException(env, "java/io/IOException", err.c_str());
    return nullptr;
}

inline int checkResultCodeOfOperateFd(JNIEnv *env, int ret) {
    if (ret > 0) {
        return ret;
    }
    if (errno == EAGAIN || errno == EINTR) {
        return 0;
    }
    jniThrowException(env, "java/io/IOException", nullptr);
    return -1;
}

static jint SerialPort_readArray(JNIEnv *env, jobject obj, jbyteArray buffer, jint offset, jint size) {
    auto ctx = checkContextAndThrowException(env, obj);
    if (ctx == nullptr) {
        return -1;
    }

    auto buf = (jbyte *) malloc(size);
    auto ret = ::read(ctx->mFd, buf, size);
    if (ret > 0) {
        env->SetByteArrayRegion(buffer, offset, ret, buf);
    }

    free(buf);
    return checkResultCodeOfOperateFd(env, ret);
}

static jint SerialPort_readDirect(JNIEnv *env, jobject obj, jobject buffer, jint offset, jint size) {
    auto ctx = checkContextAndThrowException(env, obj);
    if (ctx == nullptr) {
        return -1;
    }
    auto buf = (jbyte *) env->GetDirectBufferAddress(buffer);
    return checkResultCodeOfOperateFd(env, ::read(ctx->mFd, buf + offset, size));
}

static jint SerialPort_writeArray(JNIEnv *env, jobject obj, jbyteArray buffer, jint offset, jint size) {
    auto ctx = checkContextAndThrowException(env, obj);
    if (ctx == nullptr) {
        return -1;
    }

    auto buf = (jbyte *) malloc(size);
    env->GetByteArrayRegion(buffer, offset, size, buf);
    int ret = ::write(ctx->mFd, buf, size);
    free(buf);
    return checkResultCodeOfOperateFd(env, ret);
}

static jint SerialPort_writeDirect(JNIEnv *env, jobject obj, jobject buffer, jint offset, jint size) {
    auto ctx = checkContextAndThrowException(env, obj);
    if (ctx == nullptr) {
        return -1;
    }
    auto buf = (jbyte *) env->GetDirectBufferAddress(buffer);
    return checkResultCodeOfOperateFd(env, ::write(ctx->mFd, buf + offset, size));
}

static void SerialPort_sendBreak(JNIEnv *env, jobject obj) {
    LOGD("SerialPort.sendBreak is call");
    auto ctx = reinterpret_cast<SerialPortContext *>(
        env->GetLongField(obj, gSerialPortOffsets.mNativeInstance));
    if (ctx->isOpened()) {
        ::tcsendbreak(ctx->mFd, 0);
    }
}

constexpr char *kClassName = const_cast<char *>("com/sclimin/hardware/SerialPort");
constexpr char *kNativeInstanceName = const_cast<char *>("mNativeInstance");

constexpr JNINativeMethod kMethods[] = {
    {const_cast<char *>("nativeCreate"),      const_cast<char *>("(Ljava/lang/String;JIIZZZZ)J"), (void *) SerialPort_create},
    {const_cast<char *>("nativeDestroy"),     const_cast<char *>("(J)V"),                         (void *) SerialPort_destroy},
    {const_cast<char *>("nativeOpen"),        const_cast<char *>("()V"),                          (void *) SerialPort_open},
    {const_cast<char *>("nativeIsOpened"),    const_cast<char *>("()Z"),                          (void *) &SerialPort_isOpened},
    {const_cast<char *>("nativeClose"),       const_cast<char *>("()V"),                          (void *) &SerialPort_close},
    {const_cast<char *>("nativeReadArray"),   const_cast<char *>("([BII)I"),                      (void *) &SerialPort_readArray},
    {const_cast<char *>("nativeReadDirect"),  const_cast<char *>("(Ljava/nio/ByteBuffer;II)I"),   (void *) &SerialPort_readDirect},
    {const_cast<char *>("nativeWriteArray"),  const_cast<char *>("([BII)I"),                      (void *) &SerialPort_writeArray},
    {const_cast<char *>("nativeWriteDirect"), const_cast<char *>("(Ljava/nio/ByteBuffer;II)I"),   (void *) &SerialPort_writeDirect},
    {const_cast<char *>("nativeSendBreak"),   const_cast<char *>("()V"),                          (void *) &SerialPort_sendBreak},
};

constexpr int kMethodSize = sizeof(kMethods) / sizeof(JNINativeMethod);

JNIEXPORT jint JNI_OnLoad(JavaVM *vm, void *reserved) {
    JNIEnv *env = nullptr;
    if (vm->GetEnv((void **) &env, JNI_VERSION_1_6) != JNI_OK || !env) {
        return JNI_ERR;
    }

    gSerialPortOffsets.mClass = env->FindClass(kClassName);

    if (gSerialPortOffsets.mClass == nullptr) {
        LOGE("Not found class: %s", kClassName);
        return JNI_ERR;
    }

    auto ret = env->RegisterNatives(gSerialPortOffsets.mClass, kMethods, kMethodSize);
    if (ret != JNI_OK) {
        LOGE("RegisterNatives faild(%d).", ret);
        return JNI_ERR;
    }

    gSerialPortOffsets.mNativeInstance = env->GetFieldID(gSerialPortOffsets.mClass, kNativeInstanceName, "J");
    LOGI("::JNI_OnLoad::");
    return JNI_VERSION_1_6;
}
