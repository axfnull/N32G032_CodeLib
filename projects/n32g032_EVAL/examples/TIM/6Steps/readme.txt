1、功能说明
    1、systick 100ms触发TIM1输出6步换相波形
2、使用环境
    软件开发环境：  KEIL MDK-ARM V5.26.2.0
    硬件环境：      基于N32G032R8L7_STB开发
3、使用说明
    系统配置；
        1、时钟源：
                    HSE=8M,PLL=48M,AHB=48M,APB1=48M,APB2=48M,TIM1 CLK=48M
        2、中断：
                    TIM1 COM事件中断打开，优先级1
                    Systick 100ms中断，优先级0
        3、端口配置：
                    PA8选择为TIM1 CH1输出
                    PA9选择为TIM1 CH2输出
                    PA10选择为TIM1 CH3输出
                    PB13选择为TIM1 CH1N输出
                    PB14选择为TIM1 CH2N输出
                    PB15选择为TIM1 CH3N输出
                    PB12选择为TIM1 Breakin输入
        4、TIM：
                    TIM1 6路互补冻结输出模式，无刹车，开COM中断
    使用方法：
        1、编译后打开调试模式，用示波器或者逻辑分析仪观察TIM1的输出波形
        2、每隔100ms systick触发COM中断，在TIM的COM中断里面输出AB AC BC BA CA CB的6步换相波形
4、注意事项
       开发板默认PA9,PA10跳帽接到NSLINK的虚拟串口，若工程中PA9，PA10不作串口，用作其他用途，须拔掉串口跳帽。