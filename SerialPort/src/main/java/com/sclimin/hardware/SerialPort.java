package com.sclimin.hardware;

import androidx.annotation.IntDef;

import java.io.IOException;
import java.nio.ByteBuffer;

public final class SerialPort {
    public final static int BIT5 = 0;
    public final static int BIT6 = 1;
    public final static int BIT7 = 2;
    public final static int BIT8 = 3;

    @IntDef({BIT5, BIT6, BIT7, BIT8})
    public @interface Bits { }

    public final static int PARITY_DISABLE  = 0;
    public final static int PARITY_ODD      = 1;
    public final static int PARITY_EVEN     = 2;

    @IntDef({PARITY_DISABLE, PARITY_ODD, PARITY_EVEN})
    public @interface Parity { }

    private final long mNativeInstance;

    private SerialPort(long nativeInstance) {
        mNativeInstance = nativeInstance;
    }

    @Override
    protected void finalize() throws Throwable {
        try {
            super.finalize();
        }
        finally {
            nativeDestroy(mNativeInstance);
        }
    }

    public static Builder newBuilder(String dev, boolean nonBlocking) {
        return new Builder(dev, nonBlocking);
    }

    public static Builder newBuilder(String dev) {
        return newBuilder(dev, false);
    }

    public final static class Builder {
        private final String mDev;
        private final boolean mNonBlocking;
        private long mBaudRate = 115200;
        private int mParity = PARITY_ODD;
        private int mBits = BIT8;
        private boolean mStopBitsOn = true;
        private boolean mSoftwareFlowOn = true;
        private boolean mHardwareFlowOn = false;

        private Builder(String dev, boolean nonBlocking) {
            mDev = dev;
            mNonBlocking = nonBlocking;
        }

        /**
         * SerialPort speed must be one of 50, 75, 110, 134, 150, 200, 300, 600, 1200, 1800, 2400, 4800, 9600,
         * 19200, 38400, 57600, 115200, 230400, 460800, 500000, 576000, 921600, 1000000, 1152000,
         * 1500000, 2000000, 2500000, 3000000, 3500000, 4000000
         */
        public Builder setBaudRate(long baudRate) {
            mBaudRate = baudRate;
            return this;
        }

        public Builder setParity(@Parity int parity) {
            mParity = parity;
            return this;
        }

        public Builder setBits(@Bits int bits) {
            mBits = bits;
            return this;
        }

        public Builder setStopBits(boolean on) {
            mStopBitsOn = on;
            return this;
        }

        public Builder setHardwareFlow(boolean on) {
            mHardwareFlowOn = on;
            return this;
        }

        public Builder setSoftwareFlow(boolean on) {
            mSoftwareFlowOn = on;
            return this;
        }

        public SerialPort build() {
            return new SerialPort(nativeCreate(mDev, mBaudRate, mParity, mBits, mStopBitsOn,
                    mHardwareFlowOn, mSoftwareFlowOn, mNonBlocking));
        }
    }

    private static void checkBufferNull(Object buffer) {
        if (buffer == null) {
            throw new IllegalArgumentException("buffer is null.");
        }
    }

    private static void checkBuffer(byte[] buffer, int offset, int size) {
        checkBufferNull(buffer);
        if ((offset | size | (buffer.length - (offset + size))) < 0) {
            throw new IndexOutOfBoundsException(
                    "offset=" + offset + ", size=" + size + " out of bounds (length=" + buffer.length + ")");
        }
    }

    private static void checkBuffer(ByteBuffer buffer, int size) {
        checkBufferNull(buffer);
        if (size <= 0) {
            throw new IllegalArgumentException("ByteBuffer size <= 0");
        }
    }

    public void open() throws IOException {
        nativeOpen();
    }

    public void close() {
        nativeClose();
    }

    public boolean isOpened() {
        return nativeIsOpened();
    }

    public void sendBreak() {
        nativeSendBreak();
    }

    public int read(byte[] buffer, int offset, int size) throws IOException {
        checkBuffer(buffer, offset, size);
        return nativeReadArray(buffer, offset, size);
    }

    public int read(ByteBuffer buffer) throws IOException {
        checkBufferNull(buffer);

        if (buffer.isDirect()) {
            return nativeReadDirect(buffer, buffer.position(), buffer.remaining());
        }
        else if (buffer.hasArray()) {
            return nativeReadArray(buffer.array(), buffer.position(), buffer.remaining());
        }
        else {
            throw new IllegalArgumentException("Buffer is not direct and has no array!");
        }
    }

    public int write(byte[] buffer, int offset, int size) throws IOException {
        checkBuffer(buffer, offset, size);
        return nativeWriteArray(buffer, offset, size);
    }

    public int write(ByteBuffer buffer, int size) throws IOException {
        checkBuffer(buffer, size);

        if (buffer.isDirect()) {
            return nativeWriteDirect(buffer, buffer.position(), Math.min(size, buffer.remaining()));
        }
        else if (buffer.hasArray()) {
            return nativeWriteArray(buffer.array(), buffer.position(), Math.min(size, buffer.remaining()));
        }
        else {
            throw new IllegalArgumentException("Buffer is not direct and has no array!");
        }
    }

    private static native long nativeCreate(String path, long baudRate, int parity, int bits,
                                            boolean sendStopBits, boolean hardwareFlow, boolean softwareFlow,
                                            boolean nonBlocking);
    private static native void nativeDestroy(long nativeInstance);

    private native void nativeOpen() throws IOException;
    private native boolean nativeIsOpened();
    private native void nativeClose();

    private native int nativeReadArray(byte[] buffer, int offset, int size) throws IOException;
    private native int nativeReadDirect(ByteBuffer buffer, int offset, int size) throws IOException;

    private native int nativeWriteArray(byte[] buffer, int offset, int size) throws IOException;
    private native int nativeWriteDirect(ByteBuffer buffer, int offset, int size) throws IOException;

    private native void nativeSendBreak();

    static {
        System.loadLibrary("serial-port");
    }
}
