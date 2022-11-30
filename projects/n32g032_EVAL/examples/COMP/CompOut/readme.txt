1、功能说明
    1、COMP1的输出PA6受输入INP PA1和INM PA0的影响
2、使用环境
    软件开发环境：  KEIL MDK-ARM V5.26.2.0
    硬件环境：      基于N32G032R8L7_STB开发
3、使用说明
    系统配置；
        1、时钟源：
                    HSE=8M,PLL=48M,AHB=48M,APB1=48M,APB2=48M,COMP CLK=48M
        2、端口配置：
                    PA1选择为模拟功能COMP INP
                    PA0选择为模拟功能COMP INM
                    PA6选择为模拟功能COMP OUT
                    PA2选择为IO输出
                    PA3选择为IO输出
        3、COMP：
                    COMP1输入PA1，PA0，输出PA6
    使用方法：
        1、编译后打开调试模式，将PA2连接到PA1，PA3连接到PA0，利用示波器或者逻辑分析仪观察PA6输出波形
        2、当软件输出PA2电平大于PA3时，PA6输出高电平，相反时，输出低电平
4、注意事项
    无