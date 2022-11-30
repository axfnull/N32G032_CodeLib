1、功能说明
    1、TIM1和TIM8输出3路互补波形和一路CH4波形
2、使用环境
    软件开发环境：  KEIL MDK-ARM V5.26.2.0
    硬件环境：      基于N32G032R8L7_STB开发
3、使用说明
    系统配置；
        1、时钟源：
                    HSE=8M,PLL=48M,AHB=48M,APB1=48M,APB2=48M,TIM1 CLK=48M,TIM8 CLK=48M
        2、端口配置：
                    PA8选择为TIM1 CH1输出
                    PA9选择为TIM1 CH2输出
                    PA10选择为TIM1 CH3输出
                    PB13选择为TIM1 CH1N输出
                    PB14选择为TIM1 CH2N输出
                    PB15选择为TIM1 CH3N输出
                    PA11选择为TIM1 CH4输出
                    PA0选择为TIM8 CH1输出
                    PA1选择为TIM8 CH2输出
                    PA2选择为TIM8 CH3输出
                    PB6选择为TIM8 CH1N输出
                    PB7选择为TIM8 CH2N输出
                    PB5选择为TIM8 CH3N输出
                    PA3选择为TIM8 CH4输出
        3、TIM：
                    TIM1 6路互补输出,CH4输出
                    TIM8 6路互补输出,CH4输出
    使用方法：
        1、编译后打开调试模式，用示波器或者逻辑分析仪观察TIM1 TIM8的输出波形
        2、输出波形TIM1 3路互补加一路CH4，TIM8 3路互补加一路CH4
4、注意事项
    无