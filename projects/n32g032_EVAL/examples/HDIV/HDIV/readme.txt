1、功能说明
    1、HDIV执行一次除法，在完成中断里面通过串口发送商和余数
2、使用环境
    软件开发环境：KEIL MDK-ARM V5.34.0.0
    硬件环境：基于评估板N32G032R8L7-STBV1.0开发
3、使用说明
    系统配置；
        1、时钟源：
                    HSE=8M,PLL=48M,AHB=48M,APB1=48M,APB2=48M,HDIV CLK=48M
        2、端口配置：
                            PA9选择为串口输出
                    PA10选择为串口输入
        3、HDIV：
                    HIDV中断打开优先级0，计算 0x1E240除以 0x7B的结果
    使用方法：
        1、编译后打开调试模式，将变量HDIV的寄存器打开
        2、程序执行后，可以看到商的结果为1003，余数的结果为87，同时串口也会打印相关信息
4、注意事项
    无