1、功能说明
    1、TIM3 TIM4在TIM1周期下计数
2、使用环境
    软件开发环境：  KEIL MDK-ARM V5.26.2.0
    硬件环境：      基于N32G032R8L7_STB开发
3、使用说明
    系统配置；
        1、时钟源：
                    HSE=8M,PLL=48M,AHB=48M,APB1=48M,APB2=48M,TIM1 CLK=48M,TIM3 CLK=48M,TIM4 CLK=48M
        2、端口配置：
                    PA6选择为TIM3的CH1输出
                    PA4选择为TIM4的CH1输出
                    PA8选择为TIM1的CH1输出
        3、TIM：
                    TIM1 CH1 周期触发TIM3 TIM4的门控
    使用方法：
        1、编译后打开调试模式，用示波器或者逻辑分析仪观察TIM1 CH1、TIM3 CH1、TIM4 CH1的波形
        2、程序运行后，TIM3 15倍周期TIM1，TIM4 10倍周期TIM1
4、注意事项
    无