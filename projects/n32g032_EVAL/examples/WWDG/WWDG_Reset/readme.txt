1、功能说明
	1、WWDG复位功能。


2、使用环境

	软件开发环境：KEIL MDK-ARM V5.25

    硬件环境：
		1、基于评估板N32G032C8L7-STB V1.1开发
		2、MCU：N32G032C8L7


3、使用说明
	
	系统配置；
		1、WWDG时钟源：PCLK1
		2、窗口值：32.08ms < n <43.69ms
                3、指示灯：PA4(LED1)   PA5(LED2)
             



	使用方法：
		1、在KEIL下编译后烧录到评估板，上电后，指示灯LED2不停的闪烁。说明窗口值刷新正常，代码正常运行。
                2、当把Delay(38)函数参数小于33或者大于43时，整个系统将一直处于复位状态。LED1亮。


4、注意事项
	1、当窗口值很小时，系统处于频繁的复位状态，此时，容易引起程序无法正常下载。将BOOT0引脚拉高即可正常下载。 


