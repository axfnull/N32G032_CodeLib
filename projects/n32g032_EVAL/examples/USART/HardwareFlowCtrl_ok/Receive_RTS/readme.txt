1������˵��

	�ò�����ʾ��Board1_USARTy��Board2_USARTz��ʹ��Ӳ�������ƵĻ���ͨ�š�
USARTy��USARTz������USART1��USART2��
    ���ȣ�USARTy����CTS����TxBuffer1���ݣ�USARTz����RTS�������ݣ�����
RxBuffer1��
    ��󣬱ȽϽ��������뷢�����ݣ��ȽϽ������TransferStatus������


2��ʹ�û���

	�������������KEIL MDK-ARM Professional Version 5.26.2.0

        Ӳ����������Сϵͳ��N32G032C8L7-STB_V1.1


3��ʹ��˵��
	
	ϵͳʱ���������£�
    - ʱ��Դ = HSI + PLL
    - ϵͳʱ�� = 48MHz
    
    USARTy�������£�
    - ������ = 115200 baud
    - �ֳ� = 8����λ
    - 1ֹͣλ
    - У����ƽ���
    - CTSӲ��������ʹ��
    - ������ʹ��   
    
    USARTz�������£�
    - ������ = 115200 baud
    - �ֳ� = 8����λ
    - 1ֹͣλ
    - У����ƽ���
    - RTSӲ��������ʹ��
    - ������ʹ��   
    
    
    USART�����������£�    
    - Board1 USART1_Tx.PA9    <------->   Board2 USART2_Rx.PB7
    - Board1 USART1_CTS.PA0   <------->   Board2 USART2_RTS.PA1 

    
    ���Բ���������
    - Demo��KEIL�����±����������MCU
    - �ȸ�λBoard2�ٸ�λBoard1���У��鿴Board2����TransferStatus�����У�PASSEDΪ����ͨ����FAILED
      Ϊ�����쳣


4��ע������