1、功能说明

	该测例演示了Board1_USARTy与Board2_USARTz间使用硬件流控制的基础通信。
USARTy和USARTz可以是USART1和USART2。
    首先，USARTy利用CTS发送TxBuffer1数据，USARTz利用RTS接收数据，存至
RxBuffer1。
    随后，比较接收数据与发送数据，比较结果存入TransferStatus变量。


2、使用环境

	软件开发环境：KEIL MDK-ARM Professional Version 5.26.2.0

        硬件环境：最小系统板N32G032C8L7-STB_V1.1


3、使用说明
	
	系统时钟配置如下：
    - 时钟源 = HSI + PLL
    - 系统时钟 = 48MHz
    
    USARTy配置如下：
    - 波特率 = 115200 baud
    - 字长 = 8数据位
    - 1停止位
    - 校验控制禁用
    - CTS硬件流控制使能
    - 发送器使能   
    
    USARTz配置如下：
    - 波特率 = 115200 baud
    - 字长 = 8数据位
    - 1停止位
    - 校验控制禁用
    - RTS硬件流控制使能
    - 接收器使能   
    
    
    USART引脚连接如下：    
    - Board1 USART1_Tx.PA9    <------->   Board2 USART2_Rx.PB7
    - Board1 USART1_CTS.PA0   <------->   Board2 USART2_RTS.PA1 

    
    测试步骤与现象：
    - Demo在KEIL环境下编译后，下载至MCU
    - 先复位Board2再复位Board1运行，查看Board2变量TransferStatus，其中，PASSED为测试通过，FAILED
      为测试异常


4、注意事项