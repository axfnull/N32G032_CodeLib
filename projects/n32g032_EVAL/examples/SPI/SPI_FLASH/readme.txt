1、功能说明

	1、SPI 读、写、擦除 W25Q128

2、使用环境

	软件开发环境： KEIL MDK-ARM V5.34.0.0
	硬件环境：    基于N32G032R8L7_STB V1.0开发

3、使用说明
	
	/* 描述相关模块配置方法；例如:时钟，I/O等 */
	1、SystemClock：48MHz
	2、SPI1: NSS--PA4、SCK--PA5、MISO--PA6、MOSI--PA7


	/* 描述Demo的测试步骤和现象 */
	1.编译后下载程序复位运行；
	2.通过 SPI1 读取 W25Q128 的 ID，然后写数据，再读出来，比较读写数据，查看 TransferStatus1 状态为 PASSED，然后擦除块，检查擦除块正常。

4、注意事项
	将 N32G032R8L7_STB V1.0 开发板上的 SPI1 接口接在 N32G457QE_EVB 开发板的 W25Q128 上，两个开发板需要共地