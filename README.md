# rc_car
遥控车

---

驱动板 V1.3

**开源协议：CC-BY-NC-SA 3.0，知识共享许可协议-署名-非商业使用-相同方式共享。**

Copyright (c) 2020 OpticalMoe .

---

![image](https://user-images.githubusercontent.com/47512823/192125091-5a1d4757-944f-4866-a91b-ce74e08a733a.png)

![IMG_20200508_203018](https://user-images.githubusercontent.com/47512823/197117580-50d6c7cc-a59f-4f48-a71f-fd65d114aa64.jpg)

![IMG_20200302_162350](https://user-images.githubusercontent.com/47512823/197117618-8f2e60c5-5585-4372-b07b-26e0a18146fc.jpg)


* MCU：STM32F103C8T6

* MOS：LR7843

* 半桥驱动芯片：IR2104

* 降压：LM2596-5.0 *2

* 电机： 
    * 类型：带霍尔反馈的直流有刷电机

    * 霍尔极数：17

* DBUS ：        
    * 波特率：100K
    * 停止位：1
    * 数据位：8
    * 校验    ：偶校验
    * 数据格式：参照DJI发布的DT7&DR16遥控器【未公开发布，已删除解码部分】

* UART ：        
    * 波特率：115200
    * 停止位：1
    * 数据位：8
    * 校验    ：无

```text
控制 数据格式：4D 6F 65 03 E8 03 E8 01 01 01 00 00 00 00 00 00
    帧头：4D 6F 65
    速度：03 E8 大端模式【速度m/s * 100 + 1000】
    方向：03 E8 大端模式【方向（舵机脉宽us） + 1000】
    大灯：01【00 关；01 开】
    喇叭：01【00 关；01 开】
    彩灯：01【00 关；01 开】
    预留：00 00 00 00 00 00

回传 数据格式：4D 6F 65 03 E8 03 E8 77 0C 4C 00 00 00 00 00 00
    帧头：4D 6F 65
    速度：03 E8 大端模式【速度m/s * 100 + 1000】
    方向：03 E8 大端模式【方向（舵机脉宽us） + 1000】
    电压：77 【电压V * 10】
    电流：0C （预留）【电流A * 10】
    状态：4D
    预留：00 00 00 00 00 00
```

* 拨码开关：           
    * SW1：UART控制 开关
    * SW2：DBUS控制 开关
    * SW3：预留
    * SW4：车身彩灯（WS2812B）控制 开关

* USB-A输出能力：5V@3A

* Sign灯：        
    * 刷新车速&方向时，翻转状态；
    * 失控时，闪烁（1Hz）。

* 车身彩灯：WS2812B
    * 侧灯：
        * 绿色：DBUS遥控 控制，低速模式，限速2.5m/s；
        * 蓝色：DBUS遥控 控制，高速模式，最大5.0m/s；
        * 黄色：UART 控制，高速模式，最大5.0m/s；
        * 红色；全亮 -> 失控！
    * 尾灯：高亮->刹车；低亮->行车。

* 失控保护：           
    * 动作：速度 -> 0m/s；方向 -> 回中；
    * 时间：500ms 无刷新车速&方向数据流入 则触发。

* 电池：      
    * 3S 锂电池。标压11.1V，满电12.6V；
    * 接口：XT60。
    * 报警电压：<10.5V，持续 1s 触发；
    * 低电压报警动作：喇叭 急促 “滴”。

* FreeRTOS：
    * 任务：
    
          1. PID
          2. 遥控器解码
          3. USART1 解码
          4. ADC&电量检测
          5. WS2812B灯带
          6. 拨码开关 设置
          7. 舵机云台测试【无意义】
          
    * 队列：
        速度、方向等信息
    * 信号量：
        遥控器、USART1 空闲中断接收完成；
  
