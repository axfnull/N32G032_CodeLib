1、功能说明
	通过PC13来检测入侵。



2、使用环境

	软件开发环境：KEIL MDK-ARM V5.25

        硬件环境：
		1、基于评估板N32G032R8L7-STB V1.1开发
		2、MCU：N32G032R8L7


3、使用说明
	
	系统配置；
		1、时钟：LSI
                2、检测口：PC13
		3、串口配置：
                            - 串口为USART1（TX：PA9  RX：PA10）:
                            - 数据位：8
                            - 停止位：1
                            - 奇偶校验：无
                            - 波特率： 115200 


	使用方法：
		1、在KEIL下编译后烧录到评估板，上电后，PC13外接上拉，再给PC13传入低电平，串口打印输出Tamper interrupt。
                
                


4、注意事项
	PC13需要外部上拉