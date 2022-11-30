1、功能说明
    1、COMP1的输出刹车TIM1 的互补信号，COMP OUT变低后恢复TIM1 波形
2、使用环境
    软件开发环境：  KEIL MDK-ARM V5.26.2.0
    硬件环境：      基于N32G032R8L7_STB开发
3、使用说明
    系统配置；
        1、时钟源：
                    HSE=8M,PLL=48M,AHB=48M,APB1=48M,APB2=48M,COMP CLK=48M,TIM1 CLK=48M
        2、端口配置：
                    PA1选择为模拟功能COMP INP
                    PA0选择为模拟功能COMP INM
                    PA6选择为模拟功能COMP OUT
                    PA2选择为IO输出
                    PA3选择为IO输出
                    PA8选择为TIM1 CH1输出
                    PA9选择为TIM1 CH2输出
                    PA10选择为TIM1 CH3输出
                    PB13选择为TIM1 CH1N输出
                    PB14选择为TIM1 CH2N输出
                    PB15选择为TIM1 CH3N输出
        3、TIM：
                    TIM1开启CH1 CH2 CH3 CH1N CH2N CH3N输出,COMP作为刹车输入
        4、COMP：
                    COMP1输出触发TIM1 刹车，无输出时恢复TIM1 输出
    使用方法：
        1、编译后打开调试模式，将PA2连接到PA1，PA3连接到PA0，利用示波器或者逻辑分析仪观察TIM1输出波形
        2、当软件输出PA2电平大于PA3时，TIM波形消失，相反时，波形正常输出
4、注意事项
    无