1、功能说明

	1、SPI 发送接收数据进行 CRC 校验

2、使用环境

	软件开发环境： KEIL MDK-ARM V5.34.0.0
	硬件环境：    基于N32G032R8L7_STB V1.0开发

3、使用说明
	
	/* 描述相关模块配置方法；例如:时钟，I/O等 */
	SystemClock：48MHz
	GPIO：SPI1: SCK--PA5、 MISO--PA6、MOSI--PA7,
	SPI2: SCK--PB13、MISO--PB14、MOSI--PB15，


	/* 描述Demo的测试步骤和现象 */
	1.编译后下载程序复位运行；
	2.SPI1、SPI2 同时收发数据，传输完成后，发送 CRC 数据，检查数据和 CRC 值，查看 TransferStatus1 和 TransferStatus2 状态为 PASSED，再查看 CRC 值；


4、注意事项
	无