1、功能说明

	1、SPI 从模式全双工 DMA 发送和单线接收数据

2、使用环境

    软件开发环境： KEIL MDK-ARM V5.34.0.0
	硬件环境：    基于N32G032R8L7_STB V1.0开发

3、使用说明
	
	/* 描述相关模块配置方法；例如:时钟，I/O等 */
        SystemClock：48MHz
        GPIO：（主模式 DEMO 板）SPI1: SCK--PA5、 MISO--PA6、MOSI--PA7,
	          （从模式 DEMO 板）SPI1: SCK--PA5、 MISO--PA6、MOSI--PA7,
	
	日志打印：从模式 DEMO 板 PA9(TX)，波特率：115200
	

	/* 描述Demo的测试步骤和现象 */
        1.编译后下载程序复位运行；
	2.接好串口打印工具，上电，查看打印测试成功


4、注意事项
	需要两块 demo 板配合，一块板子烧录主模式程序，一块板子烧录从模式程序，两块板子需一起上电，连接两块板子的 VCC 和 GND