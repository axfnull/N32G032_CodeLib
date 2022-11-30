1、功能说明
    1、LPTIM 计数IN1 IN2正交编码的上升沿个数
2、使用环境
    软件开发环境：  KEIL MDK-ARM V5.26.2.0
    硬件环境：      基于N32G032R8L7_STB开发
3、使用说明
    系统配置；
        1、时钟源：
                    HSE=8M,PLL=48M,AHB=48M,APB1=48M,APB2=48M,LPTIM CLK=LSI
        2、端口配置：
                    PA4选择为LPIME IN1输入
                    PA5选择为LPIME IN2输入
                    PB1选择为IO输出
                    PB6选择为IO输出
        3、LPTIM：
                    LPTIM正交编码器模式，利用内部LSI时钟连续计数IN1 IN2上升沿个数
    使用方法：
        1、编译后打开调试模式，连接PB1和PA4，PB6和PA5，将变量encCNT添加到watch窗口
        2、程序运行后，PB1 PB6输出20个脉冲周期，encCNT等于40
4、注意事项
    无