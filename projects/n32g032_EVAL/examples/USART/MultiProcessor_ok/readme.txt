1、功能说明

	该测例演示了如何使用USART多处理器模式。USARTy和USARTz可以是USART1
和USART2、USART3和UART4或UART4和UART5。
    首先，分别设置USARTy和USARTz的地址为0x1和0x2。USARTy连续给USARTz
发送字符0x33。USARTz收到0x33，便翻转LED1、LED2和LED3的引脚。
    一旦KEY1_INT_EXTI_LINE线检测到上升沿，则产生EXTI0中断，在
EXTI0_IRQHandler中断处理函数中(the ControlFlag = 0)，USARTz进入静默
模式，在静默模式中，LED引脚停止翻转，直到KEY1_INT_EXTI_LINE线检测到
上升沿(the ControlFlag = 1)。在EXTI0_IRQHandler中断处理函数中，USARTy
发送地址0x102唤醒USARTz。LED引脚重新启动翻转。


2、使用环境

	软件开发环境：KEIL MDK-ARM Professional Version 5.26.2.0

        硬件环境：最小系统板N32G032C8L7-STB_V1.1


3、使用说明
	
	系统时钟配置如下：
    - 时钟源 = HSI + PLL
    - 系统时钟 = 48MHz
    
    USARTy配置如下：
    - 波特率 = 115200 baud
    - 字长 = 9数据位
    - 1停止位
    - 校验控制禁用
    - 硬件流控制禁用（RTS和CTS信号）
    - 接收器和发送器使能  
    
    
    USART引脚连接如下：    
    - USART1_Tx.PA9    <------->   UART6_Rx.PA5
    - USART1_Rx.PA10   <------->   UART6_Tx.PA4    
    
    KEY1_INT_EXTI_LINE.PA6    <------->   KEY3
    
    LED1    <------->   PB1
    LED2    <------->   PB6
    LED3    <------->   PB7

    
    测试步骤与现象：
    - Demo在KEIL环境下编译后，下载至MCU
    - 复位运行，观察LED1~3是否处于闪烁状态
    - 按按键KEY，观察LED1~3是否停止闪烁
    - 再次按按键KEY，观察LED1~3是否恢复闪烁


4、注意事项