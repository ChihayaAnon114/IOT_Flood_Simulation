物理层已由前面的assignment基本实现，例如信道、multiplex、编码解码等。在这个部分中，我们不做MIMO，也不做跳频寻址CDMA等高级实现，只保留一个信道、莱利衰减、自由空间衰减和5条多径。目标是实现一个BPSK/4QAM编码的编码-解码模拟并在不同情况下进行15-40分贝的SNR-BER蒙特卡洛扫频。这个模拟的重点在于链路层协议的仿真，**集中分析编码方式（BPSK/4QAM)、前向纠错与SNR的影响。**

编码与信道传输仿真方法：

## 1，报文结构：

```
[Message Type]-[Sensor ID]-[Payload(Timestamp-Water Depth-Flow Speed-Sensor Status)]
例如：
Message Type = 0x01      // 普通监测数据
Sensor ID    = 0x05      // 第5号传感器
Timestamp    = 1715000000
Water Depth  = 235       // 例如表示235 cm
Flow Speed   = 120       // 例如表示120 cm/s
Status       = 0x00      // 正常
```

## 2，数据链：

其实这里应该使用慢启动机制来适应高SNR情况，但是这就需要有连接的协议了。理论上来说，如果允许无限重传，几乎任何消息都能正确接收，所以我没有加入有连接与允许重传的机制，在报告里写就可以了。

加入了前向纠错，可选是否使用Hamming Code/简易LDPC进行前向纠错。没有循环冗余验证（因为我们不做重传，所以后向纠错没有意义）

**帧结构：**

[Start Flag]-[Message ID]-[Fragment Index]-[Total Fragments]-**[Frame Payload]**-[End Flag]----->Hamming Code/简易LDPC

## 3，实验结构：

随机模拟报文->编码->数据链层加入纠错码、成帧传输->接收端解码->对比接收到的报文并对比计算BNR->绘制频率-功率谱密度（已正则化）

日志中包含了接收到的错误报文、编解码过程及其错误原因

控制变量与实验设计你们写报告的拿到代码自己整吧

<img src="C:\Users\user\AppData\Roaming\Typora\typora-user-images\image-20260508170903986.png" alt="image-20260508170903986" style="zoom:30%;" />

## 4，运行样图

<img src="C:\Users\user\AppData\Roaming\Typora\typora-user-images\image-20260509033912670.png" alt="image-20260509033912670" style="zoom:50%;" />

<img src="C:\Users\user\AppData\Roaming\Typora\typora-user-images\image-20260509034017013.png" alt="image-20260509034017013" style="zoom:60%;" />

<img src="C:\Users\user\AppData\Roaming\Typora\typora-user-images\image-20260509034055440.png" alt="image-20260509034055440" style="zoom:50%;" />

<img src="C:\Users\user\AppData\Roaming\Typora\typora-user-images\image-20260509035002039.png" alt="image-20260509035002039" style="zoom:50%;" />