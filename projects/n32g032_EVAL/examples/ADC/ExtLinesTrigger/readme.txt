1、功能说明
    1、ADC规则通道采样PA4、PA5引脚的模拟电压，注入通道采样PA0、PA1引脚的模拟电压
    2、其中规则转换结果通过DMA_CH1通道读取到变量ADC_RegularConvertedValueTab[64]数组
           注入转换结果通过转换结束中断读取到变量ADC_InjectedConvertedValueTab[32]数组
2、使用环境
    软件开发环境：  KEIL MDK-ARM V5.26.2.0
    硬件环境：      基于于N32G032R8L7_STB开发
3、使用说明
    系统配置；
        1、时钟源：
                    HSE=8M,PLL=48M,AHB=48M,APB1=48M,APB2=48M,ADC CLK=48M/16,ADC 1M CLK=HSI/8,DMA CLK=48M
        2、中断：
                    ADC注入转换结果完成中断打开，优先级0
                    中断处理接收注入转换结果到ADC_InjectedConvertedValueTab[32]数组
        3、端口配置：
                    PA4选择为模拟功能AD转换通道
                    PA5选择为模拟功能ADC转换通道
                    PA0选择为模拟功能ADC转换通道
                    PA1选择为模拟功能ADC转换通道
                    PA9选择为外部EXTI事件上升沿触发
                    PA10选择为外部EXTI事件上升沿触发
        4、DMA：
                    DMA_CH1通道回环模式搬运64个半字的ADC规则通道转换结果到ADC_RegularConvertedValueTab[64]数组
        5、ADC：
                    ADC规则通道扫描间断模式、EXTI9触发、12位数据右对齐，转换通道PA4和PA5的模拟电压数据
                    ADC注入通道扫描模式、EXTI10触发、12位数据右对齐，转换通道PA0和PA1的模拟电压数据
    使用方法：
        1、编译后打开调试模式，将变量ADC_RegularConvertedValueTab[64],ADC_InjectedConvertedValueTab[32]添加到watch窗口观察
        2、通过PA9给上升沿可以触发规则通道数据采样，PA10给上升沿可以触发注入通道数据采样
4、注意事项
    当系统采用HSE时钟时（一般HSI也是打开的），RCC_ConfigAdc1mClk(RCC_ADC1MCLK_SRC_HSE, RCC_ADC1MCLK_DIV8)可以配置为HSE或者HSI
    当系统采样HSI时钟时（一般HSE是关闭的），RCC_ConfigAdc1mClk(RCC_ADC1MCLK_SRC_HSI, RCC_ADC1MCLK_DIV8)只能配置为HSI
    开发板默认PA9,PA10跳帽接到NSLINK的虚拟串口，若工程中PA9，PA10不作串口，用作其他用途，须拔掉串口跳帽。