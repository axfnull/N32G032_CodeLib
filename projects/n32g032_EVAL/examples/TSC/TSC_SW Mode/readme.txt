1、功能说明
    轮询24个触摸按键，有按键输入时通过调试串口输出相关信息

2、使用环境
    软件开发环境：KEIL MDK-ARM V5.26
    硬件环境：基于开发板N32G032R8L7_STB V1.0开发

3、使用说明

    系统配置；
      1、时钟源：HSI+PLL
      2、系统时钟：16MHz
      3、TSC端口： 
        PA0:CH0   PA1:CH1   PA2:CH2   PA3:CH3
        PA4:CH4   PA5:CH5   PA6:CH6   PA7:CH7
        PB0:CH8   PB1:CH9   PB2:CH10  PB11:CH11
        PB12:CH12 PB14:CH13 PC6:CH14  PC7:CH15
        PC8:CH16  PC9:CH17  PA11:CH18 PA12:CH19
        PB6:CH20  PB8:CH21  PF0:CH22  PF1:CH23
      4、LED控制端口：
        PC0:LED1  PC1:LED2  PC2:LED3
      5、定时器：TIM4、TIM3
      6、调试端口：LPUART1
        TX:PC10    RX:PC11

    使用方法：
        1、在KEIL下编译后烧录到测试板，通过USB端口连接PC,通电，LED3闪烁
        2、在PC端打开串口调试软件，按下任意触摸按键，可看到相应信息

4、注意事项
无