1������˵��

    �ò�����ʾ��LPUART��USARTz��ͨ��DMAʵ�ֵĻ���ͨ�š�
    ���ȣ�DMA����TxBuffer1������LPUART�������ݼĴ�����������ݷ�����
USARTz��USARTz���մ�DMA���������ݣ�����RxBuffer2���Ƚ��ա������ݣ��Ƚ�
�������TransferStatus2������
    ͬʱ��DMA����TxBuffer2������USARTz�������ݼĴ�����������ݷ�����
LPUART��LPUART���մ�DMA���������ݣ�����RxBuffer1���Ƚ��ա������ݣ��Ƚ�
�������TransferStatus1������   


2��ʹ�û���

	�������������KEIL MDK-ARM Professional Version 5.26.2.0

        Ӳ����������Сϵͳ��N32G032C8L7-STB_V1.1


3��ʹ��˵��
	
    ϵͳʱ���������£�
    - ʱ��Դ = HSI + PLL
    - ϵͳʱ�� = 48MHz
    
    LPUART�������£�
    - ������ = 115200 baud
    - �ֳ� = 8����λ���̶���
    - 1ֹͣλ���̶���
    - У����ƽ���
    - Ӳ�������ƽ��ã�RTS��CTS�źţ�
    - �������ͷ�����ʹ��
    - DMA����ģʽ��DMA����ģʽʹ��
    
    USART�������£�
    - ������ = 115200 baud
    - �ֳ� = 8����λ
    - 1ֹͣλ
    - У����ƽ���
    - Ӳ�������ƽ��ã�RTS��CTS�źţ�
    - �������ͷ�����ʹ��
    - DMA����ģʽ��DMA����ģʽʹ��
    
    LPUART��USART�����������£�
    - LPUART_Tx.PA1   <------->   USART1_Rx.PA10
    - LPUART_Rx.PA0   <------->   USART1_Tx.PA9

    
    ���Բ���������
    - Demo��KEIL�����±����������MCU
    - ��λ���У����β鿴����TransferStatus1��TransferStatus2�����У�
      PASSEDΪ����ͨ����FAILEDΪ�����쳣


4��ע������