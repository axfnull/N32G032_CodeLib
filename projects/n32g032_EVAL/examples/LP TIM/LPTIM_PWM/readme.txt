1、功能说明
    1、LPTIM 输出PWM信号
2、使用环境
    软件开发环境：  KEIL MDK-ARM V5.26.2.0
    硬件环境：      基于N32G032R8L7_STB开发
3、使用说明
    系统配置；
        1、时钟源：
                    HSE=8M,PLL=48M,AHB=48M,APB1=48M,APB2=48M,LPTIM CLK=LSI
        2、端口配置：
                    PA9选择为LPTIM输出
        3、LPTIM：
                    LPTIM CLK不分频，LPTIM输出PWM信号,频率50HZ（ARR设为600）。
    使用方法：
        1、编译后打开调试模式，可观察到PA9引脚PWM信号
4、注意事项
    无