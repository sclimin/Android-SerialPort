# Android-SerialPort

[![License](https://img.shields.io/badge/license-Apache%202-green.svg)](https://www.apache.org/licenses/LICENSE-2.0)

## 介绍

Android端串口工具，借鉴了``minicom``、``Android源码``。( 正常操作，没做权限处理)

### 使用

```Java
// 默认阻塞读写，可设置非阻塞
SerialPort serialPort = SerialPort.newBuilder("/dev/ttyS0", true);
// or
SerialPort serialPort = SerialPort.newBuilder("/dev/ttyS0")
        .setBaudRate(115200)
        .setBits(SerialPort.BIT8)
        .setParity(SerialPort.PARITY_ODD)
        .setHardwareFlow(false)
        .setSoftwareFlow(true)
        .build();
try {
    serialPort.open();
    // 非阻塞模式，没数据立即返回0
    serialPort.read(...);
    serialPort.write(...);
    serialPort.close();
} catch (IOException e) {
    e.printStackTrace();
}
```

## License

```
   Copyright 2020 sclimin

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
```