1、功能说明

	1、I2S 从模式接收数据完成以后切换成 SPI 从模式接收数据，再切换成 I2S 主模式接收数据


2、使用环境

	软件开发环境： KEIL MDK-ARM V5.34.0.0
	硬件环境：    基于N32G032R8L7_STB V1.0开发

3、使用说明
	
	/* 描述相关模块配置方法；例如:时钟，I/O等 */
	1、SystemClock：48MHz
	2、GPIO：（主模式 DEMO 板）I2S1: WS--PA4、CK--PA5、SD--PA7
			 （从模式 DEMO 板）I2S1: WS--PA4、CK--PA5、SD--PA7

	/* 描述Demo的测试步骤和现象 */
	1.编译后下载程序复位运行；
	2.从模式 DEMO 板进入调试模式，先按住主模式 DEMO 板复位键，然后从模式 DEMO 板全速跑，再松开主模式 DEMO 板复位键，查看 TransferStatus1、TransferStatus2、TransferStatus3 状态为 PASSED


4、注意事项
	需要两块 demo 板配合，一块板子烧录主模式程序，一块板子烧录从模式程序，两块板子需一起上电，连接两块板子的 VCC 和 GND