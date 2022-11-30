1、功能说明
    1、TIM4 CH2上升沿触发CH1输出一个单脉冲
2、使用环境
    软件开发环境：  KEIL MDK-ARM V5.26.2.0
    硬件环境：      基于N32G032R8L7_STB开发
3、使用说明
    系统配置；
    1、时钟源：
        HSE=8M,PLL=48M,AHB=48M,APB1=48M,APB2=48M,TIM4 CLK=48M
    2、端口配置：
        PA4选择为TIM4的CH1输出
        PA7选择为TIM4的CH2输入
        PA3选择为IO输出
    3、TIM：
        TIM4 配置CH2上升沿触发CH1输出一个单脉冲
    使用方法：
        1、编译后打开调试模式，PA3连接PA7，用示波器或者逻辑分析仪观察TIM4 的CH1 的波形
        2、将变量gSendTrigEn添加到watch窗口，默认gSendTrigEn=0，每次手动修改gSendTrigEn=1,将会看到在TIM4 CH1端口输出一个单脉冲
4、注意事项
    无