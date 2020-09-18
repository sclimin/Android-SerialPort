package com.sclimin.hardware.sample;

import androidx.appcompat.app.AppCompatActivity;

import android.os.Bundle;

import com.sclimin.hardware.SerialPort;

import java.io.IOException;

public class MainActivity extends AppCompatActivity {

    SerialPort mSerialPort;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        mSerialPort = SerialPort.newBuilder("/dev/ttyS0")
                .setBaudRate(115200)
                .setBits(SerialPort.BIT8)
                .setParity(SerialPort.PARITY_ODD)
                .setHardwareFlow(false)
                .setSoftwareFlow(true)
                .build();
        try {
            mSerialPort.open();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
