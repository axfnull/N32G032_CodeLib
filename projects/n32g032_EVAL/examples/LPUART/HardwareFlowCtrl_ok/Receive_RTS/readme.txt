1������˵��

	�ò�����ʾ��Board1_LPUART��Board2_LPUART��ʹ��Ӳ�������ƵĻ���ͨ�š�
    ���ȣ�Board1_LPUART����CTS����TxBuffer1���ݣ�Board2_LPUART����RTS����
���ݣ�����RxBuffer1��
    ��󣬱ȽϽ��������뷢�����ݣ��ȽϽ������TransferStatus������


2��ʹ�û���

	�������������KEIL MDK-ARM Professional Version 5.26.2.0

        Ӳ����������Сϵͳ��N32G032C8L7-STB_V1.1


3��ʹ��˵��
	
	ϵͳʱ���������£�
    - ʱ��Դ = HSI + PLL
    - ϵͳʱ�� = 48MHz
    
    Board1_LPUART�������£�
    - ������ = 9600 baud
    - �ֳ� = 8����λ���̶���
    - 1ֹͣλ���̶���
    - У����ƽ���
    - CTSӲ��������ʹ��
    - ������ʹ��
    
    Board2_LPUART�������£�
    - ������ = 9600 baud
    - �ֳ� = 8����λ���̶���
    - 1ֹͣλ���̶���
    - У����ƽ���
    RTSӲ��������ʹ��
    - ������ʹ��  
    
    
    LPUART�����������£�    
    - Board1_LPUART_Tx.PB6    <------->   Board2_LPUART_Rx.PB7
    - Board1_LPUART_CTS.PA6   <------->   Board2_LPUART_RTS.PB1  

    
    ���Բ���������
    - Demo��KEIL�����±����������MCU
    - ��λ���У��鿴����TransferStatus�����У�PASSEDΪ����ͨ����FAILED
      Ϊ�����쳣


4��ע������