1������˵��

	�ò�����ʾ��USARTy��USARTz��ͨ����ѯ����ʶʵ�ֵĻ���ͨ�š�
    ���ȣ�USARTy����TxBuffer1������USARTz��USARTz�������ݴ���RxBuffer1��
�ȽϽ��������뷢�����ݣ��ȽϽ������TransferStatus������
    ���USARTz����TxBuffer2������USARTy��USARTy�������ݴ���RxBuffer2��
�ȽϽ��������뷢�����ݣ��ȽϽ������TransferStatus������
    USARTy��USARTz������USART1��USART2��


2��ʹ�û���

	�������������KEIL MDK-ARM Professional Version 5.26.2.0

        Ӳ����������Сϵͳ��N32G032C8L7-STB_V1.1


3��ʹ��˵��
	
    ϵͳʱ���������£�
    - ʱ��Դ = HSI + PLL
    - ϵͳʱ�� = 48MHz
    
    USART�������£�
    - ������ = 115200 baud
    - �ֳ� = 8����λ
    - 1ֹͣλ
    - У����ƽ���
    - Ӳ�������ƽ��ã�RTS��CTS�źţ�
    - �������ͷ�����ʹ��
    
    USART�����������£�
    - USART1_Tx.PA9    <------->   UART6_Rx.PA5
    - USART1_Rx.PA10   <------->   UART6_Tx.PA4 
    ��
    - UART5_Tx.PB0   <------->   USART2_Rx.PA10
    - UART5_Rx.PB1   <------->   USART2_Tx.PA9

    
    ���Բ���������
    - Demo��KEIL�����±����������MCU
    - ��λ���У������β鿴����TransferStatus�����У�PASSEDΪ����ͨ����
      FAILEDΪ�����쳣


4��ע������