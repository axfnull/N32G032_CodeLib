1������˵��
    1��TIM1 CH4����ADCע�����OPA�ĵ�ѹ���ݣ�����TIM1�����COMP1ɲ������
2��ʹ�û���
    �������������  KEIL MDK-ARM V5.26.2.0
    Ӳ��������      ����N32G032R8L7_STB����
3��ʹ��˵��
    ϵͳ���ã�
        1��ʱ��Դ��
                    HSE=8M,PLL=48M,AHB=48M,APB1=48M,APB2=48M,ADC CLK=48M/16,ADC 1M CLK=HSE/8,OPA CLK=48M,COMP CLK=48M,TIM1 CLK=48M,TIM8 CLK=48M
        2���жϣ�
                    ADCע���жϴ򿪣����ȼ�0
                    ��ȡת�����
        3���˿����ã�
                    PA0ѡ��Ϊģ�⹦��OPA VP
                    PA6ѡ��Ϊģ�⹦��OPA OUT
                    PF7ѡ��Ϊģ�⹦��COMP1 INP
                    PF6ѡ��Ϊģ�⹦��COMP1 INM
                    PA15ѡ��Ϊģ�⹦��COMP1 OUT    
                    PA8ѡ��ΪTIM1 CH1���
                    PA9ѡ��ΪTIM1 CH2���
                    PA10ѡ��ΪTIM1 CH3���
                    PB13ѡ��ΪTIM1 CH1N���
                    PB14ѡ��ΪTIM1 CH2N���
                    PB15ѡ��ΪTIM1 CH3N���
                    PB12ѡ��ΪTIM1 Breakin����
                    PA6ѡ��ΪADC_CH6ͨ������
        4��OPA��
                    OPA��ѹ���������ܣ�VM������
        5��ADC��
                    ADCע��ģʽ��ɨ��ת����TIM1 CC4������12λ�����Ҷ��룬ע��ת��OPA���ģ���ѹ����
        6��COMP��
                    COMP1 INPѡ��PF7��INMѡ��PF6���������TIM1��TIM8ɲ��
        7��TIM��
                    TIM1 6·�����򿪣�ɲ��ʹ���Ҵ��жϣ�CH4�������
    ʹ�÷�����
        1�������򿪵���ģʽ����ʾ���������߼������ǹ۲�TIM1��������κ�ADC��ȡ�Ĳɼ�����
        2���ı�COMP1 �����״̬����PB12���ŵ�ѹ����ɲ��TIM1���ı�OPA�������ѹ���Ըı�ADC��ȡ������
4��ע������
    ��