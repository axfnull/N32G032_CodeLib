1、功能说明
1、STOP进入和唤醒退出示例。


2、使用环境

	软件开发环境：KEIL MDK-ARM V5.25

    硬件环境：
		1、基于评估板N32G032R8L7-STB V1.1开发
		2、MCU：N32G032R8L7


3、使用说明
	
	系统配置；
		1、时钟源：HSE+PLL




	使用方法：
		在KEIL下编译后烧录到评估板，上电输出打印“PWR_STOP INIT”。过了一会输出打印“STOP ENTRY”，表明进入STOP了。
        按下WAKEUP按键后，串口又输出“STOP EXIT”，表明MCU被唤醒了.从停止的位置执行。


4、注意事项
	无

