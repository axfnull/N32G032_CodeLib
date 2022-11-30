1、功能说明

    此例程展示了通过I2C模块与外部EEPRON的通信。   

2、使用环境

   软件开发环境：
        IDE工具：KEIL MDK-ARM 5.21.1.0
    
    硬件环境：
        开发板 N32G032R8L7-STB


3、使用说明
	
    1、时钟源：HSE+PLL
    2、主时钟：48MHz
    3、I2C1 配置：
            SCL   -->  PB6          
            SDA   -->  PB7         
            CLOCK:400KHz
            
    4、USART1配置：
            TX  -->  PA9   
	    RX  -->  PA10           
            波特率：115200
    
    5、测试使用EEPROM型号:
	    AT24C04（容量4kb）
        

    6、测试步骤与现象
        a，检查EEPROM连接
        b，编译下载代码复位运行
        c，从串口看打印信息，验证结果

4、注意事项
   无