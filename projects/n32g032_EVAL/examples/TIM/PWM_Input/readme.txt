1、功能说明
    1、TIM3 CH2捕获引脚通过CH1下降沿和CH2上升沿计算占空比和频率
2、使用环境
    软件开发环境：  KEIL MDK-ARM V5.26.2.0
    硬件环境：      基于N32G032R8L7_STB开发
3、使用说明
    系统配置；
        1、时钟源：
                    HSE=8M,PLL=48M,AHB=48M,APB1=48M,APB2=48M,TIM3 CLK=48M
        2、中断：
                    TIM3 CC2比较中断打开，优先级1
        3、端口配置：
                    PA7选择为TIM3的CH2输入
                    PA3选择为IO输出
        4、TIM：
                    TIM3 CH1捕获CH2信号的下降沿。TIM3 CH2捕获CH2信号的上升沿
    使用方法：
        1、编译后打开调试模式，连接PA3和PA7，将Frequency、DutyCycle、gOnePulsEn添加到watch窗口
        2、程序运行后，默认gOnePulsEn=0，每次手动给gOnePulsEn=1，此时可以看到被捕获到的占空比和频率的值分别存储在DutyCycle和Frequency变量
4、注意事项
    无