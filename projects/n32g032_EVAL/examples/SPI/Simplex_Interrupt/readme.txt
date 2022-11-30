1、功能说明

	1、SPI 单线中断发送和接收数据

2、使用环境

	软件开发环境： KEIL MDK-ARM V5.34.0.0
	硬件环境：    基于N32G032R8L7_STB V1.0开发

3、使用说明
	
	/* 描述相关模块配置方法；例如:时钟，I/O等 */
        1、SystemClock：108MHz
        2、GPIO：SPI1: SCK--PA5、 MOSI--PA7,
		 SPI2: SCK--PB13、MISO--PB14
	3、中断：SPI1 中断入口函数 SPI1_IRQHandler，SPI2 中断入口函数 SPI2_3_IRQHandler

	/* 描述Demo的测试步骤和现象 */
        1.编译后下载程序复位运行；
        2.SPI1 有数据需要发送时进入 SPI1_IRQHandler 中断函数发送，SPI2 有数据需要接收时进入 SPI2_3_IRQHandler中断函数接收，
	  数据传输完成后，查看 TransferStatus 状态为 PASSED；


4、注意事项
	“单线”数据线在主设备端为MOSI引脚，在从设备端为MISO引脚