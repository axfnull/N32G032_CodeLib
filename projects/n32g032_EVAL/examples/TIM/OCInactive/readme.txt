1、功能说明
    1、TIM3 CH1 CH2 CH3 CH4 达到CC值后，对应拉低PC6 PC7 PC8 PC9的IO电平
2、使用环境
    软件开发环境：  KEIL MDK-ARM V5.26.2.0
    硬件环境：      基于N32G032R8L7_STB开发
3、使用说明
    系统配置；
        1、时钟源：
                    HSE=8M,PLL=48M,AHB=48M,APB1=48M,APB2=48M,TIM3 CLK=48M
        2、中断：
                    TIM3 比较中断打开，抢断优先级0，子优先级1
        3、端口配置：
                    PA6选择为IO 输出
                    PA7选择为IO 输出
                    PA8选择为IO 输出
                    PA9选择为IO 输出
        4、TIM：
                    TIM3 配置好CH1 CH2 CH3 CH4的比较值，并打开比较中断
    使用方法：
        1、编译后打开调试模式，用示波器或者逻辑分析仪观察PA6 PA7 PA8 PA9的波形
        2、定时器运进入CC1 CC2 CC3 CC4中断后,对应拉低PA6 PA7 PA8 PA9的IO
4、注意事项
    无