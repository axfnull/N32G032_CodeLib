1、功能说明
    1、LPTIM 计数IN1 IN2非正交编码的下升沿个数
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
                    PB0选择为IO输出
                    PB1选择为IO输出
        3、LPTIM：
                    LPTIM非正交编码器模式，利用内部LSI时钟连续计数IN1 IN2下升沿个数
    使用方法：
        1、编译后打开调试模式，连接PB0和PA4，PB1和PA5，将变量NencCNT添加到watch窗口
        2、程序运行后，PB0 PB1输出20个脉冲周期，NencCNT等于20
4、注意事项
    无