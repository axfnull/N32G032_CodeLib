1������˵��

	1��SPI �����жϷ��ͺͽ�������

2��ʹ�û���

	������������� KEIL MDK-ARM V5.34.0.0
	Ӳ��������    ����N32G032R8L7_STB V1.0����

3��ʹ��˵��
	
	/* �������ģ�����÷���������:ʱ�ӣ�I/O�� */
        1��SystemClock��108MHz
        2��GPIO��SPI1: SCK--PA5�� MOSI--PA7,
		 SPI2: SCK--PB13��MISO--PB14
	3���жϣ�SPI1 �ж���ں��� SPI1_IRQHandler��SPI2 �ж���ں��� SPI2_3_IRQHandler

	/* ����Demo�Ĳ��Բ�������� */
        1.��������س���λ���У�
        2.SPI1 ��������Ҫ����ʱ���� SPI1_IRQHandler �жϺ������ͣ�SPI2 ��������Ҫ����ʱ���� SPI2_3_IRQHandler�жϺ������գ�
	  ���ݴ�����ɺ󣬲鿴 TransferStatus ״̬Ϊ PASSED��


4��ע������
	�����ߡ������������豸��ΪMOSI���ţ��ڴ��豸��ΪMISO����