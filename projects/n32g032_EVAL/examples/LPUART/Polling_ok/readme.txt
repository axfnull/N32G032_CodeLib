1������˵��

	�ò�����ʾ��LPUART��USARTz��ͨ����ѯ����ʶʵ�ֵĻ���ͨ�š�
    ���ȣ�LPUART����TxBuffer1������USARTz��USARTz�������ݴ���RxBuffer1��
�ȽϽ��������뷢�����ݣ��ȽϽ������TransferStatus������
    ���USARTz����TxBuffer2������LPUART��LPUART�������ݴ���RxBuffer2��
�ȽϽ��������뷢�����ݣ��ȽϽ������TransferStatus������


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
    
    USART�������£�
    - ������ = 115200 baud
    - �ֳ� = 8����λ
    - 1ֹͣλ
    - У����ƽ���
    - Ӳ�������ƽ��ã�RTS��CTS�źţ�
    - �������ͷ�����ʹ��
    
    LPUART��USART�����������£�
    - LPUART_Tx.PA1   <------->   USART1_Rx.PA10
    - LPUART_Rx.PA0   <------->   USART1_Tx.PA9

    
    ���Բ���������
    - Demo��KEIL�����±����������MCU
    - ��λ���У������β鿴����TransferStatus�����У�PASSEDΪ����ͨ����
      FAILEDΪ�����쳣


4��ע������