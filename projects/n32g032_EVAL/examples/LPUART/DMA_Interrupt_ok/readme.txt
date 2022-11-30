1、功能说明

    该测例演示了LPUART与USARTz间通过DMA实现的基础通信。
    首先，DMA传输TxBuffer1数据至LPUART发送数据寄存器，随后数据发送至
USARTz。USARTz利用中断接收数据，存至RxBuffer2。
    同时，DMA传输TxBuffer2数据至USARTz发送数据寄存器，随后数据发送至
USARTy。LPUART通过查询检查标志接收数据，存至RxBuffer1。
    最后，分别比较两组收、发数据，比较结果存入TransferStatus1变量
和TransferStatus2变量。


2、使用环境

	软件开发环境：KEIL MDK-ARM Professional Version 5.26.2.0

        硬件环境：最小系统板N32G032C8L7-STB_V1.1


3、使用说明
	
    系统时钟配置如下：
    - 时钟源 = HSI + PLL
    - 系统时钟 = 48MHz
    
    LPUART配置如下：
    - 波特率 = 115200 baud
    - 字长 = 8数据位（固定）
    - 1停止位（固定）
    - 校验控制禁用
    - 硬件流控制禁用（RTS和CTS信号）
    - 接收器和发送器使能
    - DMA发送模式和DMA接收模式使能
    
    LPUART配置如下：
    - 波特率 = 115200 baud
    - 字长 = 8数据位（固定）
    - 1停止位（固定）
    - 校验控制禁用
    - 硬件流控制禁用（RTS和CTS信号）
    - 接收器和发送器使能
    - DMA发送模式和DMA接收模式使能
    
    USART引脚连接如下：
    - LPUART_Tx.PA1   <------->   USART1_Rx.PA10
    - LPUART_Rx.PA0   <------->   USART1_Tx.PA9

    
    测试步骤与现象：
    - Demo在KEIL环境下编译后，下载至MCU
    - 复位运行，依次查看变量TransferStatus1和TransferStatus2，其中，
      PASSED为测试通过，FAILED为测试异常


4、注意事项