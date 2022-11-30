1、功能说明
    1、ADC采样内部通道Vrefint
2、使用环境
    软件开发环境：  KEIL MDK-ARM V5.26.2.0
    硬件环境：      基于N32G032R8L7_STB开发
3、使用说明
    系统配置；
        1、时钟源：
                    HSE=8M,PLL=48M,AHB=48M,APB1=48M,APB2=48M,ADC CLK=48M/16,ADC 1M CLK=HSI/8,DMA CLK=48M
       2、DMA：
                    DMA_CH1通道循环模式搬运一个半字的ADC转换结果到ADCConvertedValue变量
        3、ADC：
                    ADC连续转换、软件触发、12位数据右对齐，转换通道16即内部温度传感器的模拟电压数据
    使用方法：
        1、编译后打开调试模式，将变量ADCConvertedValue,添加到watch窗口观察，这个值就是Vrefint的采样值
4、注意事项
    当系统采用HSE时钟时（一般HSI也是打开的），RCC_ConfigAdc1mClk(RCC_ADC1MCLK_SRC_HSE, RCC_ADC1MCLK_DIV8)可以配置为HSE或者HSI
    当系统采样HSI时钟时（一般HSE是关闭的），RCC_ConfigAdc1mClk(RCC_ADC1MCLK_SRC_HSI, RCC_ADC1MCLK_DIV8)只能配置为HSI
