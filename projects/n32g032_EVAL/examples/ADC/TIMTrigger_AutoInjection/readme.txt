1������˵��
    1��ADC�������PA5���Զ�ע�����PA4���ŵ�ģ���ѹ����TIM1 CC2�¼��´�������
    2�����й���ת�����ͨ��DMA_CH1ͨ����ȡ������ADC_RegularConvertedValueTab[32]����
           ע��ת�����ͨ��ת�������ж϶�ȡ������ADC_InjectedConvertedValueTab[32]����
2��ʹ�û���
    �������������  KEIL MDK-ARM V5.26.2.0
    Ӳ��������      ����N32G032R8L7_STB����
3��ʹ��˵��
    ϵͳ���ã�
        1��ʱ��Դ��
                    HSE=8M,PLL=48M,AHB=48M,APB1=48M,APB2=48M,ADC CLK=48M/16,ADC 1M CLK=HSI/8,DMA CLK=48M
        2���жϣ�
                    ADCע��ת���������жϴ�, �����ȼ�0
                    �жϺ����н�ע������ȡ��ADC_InjectedConvertedValueTab[32]���飬����תPA6��ƽ
        3���˿����ã�
                    PA4ѡ��Ϊģ�⹦��ADCת��ͨ��
                    PA5ѡ��Ϊģ�⹦��ADCת��ͨ��
                    PA6ѡ��Ϊͨ��IO���
                    PA9ѡ��ΪTIM1 CH2 PWM���
        4��DMA��
                    DMA_CH1ͨ���ػ�ģʽ����32�����ֵ�ADC1ת�������ADC_RegularConvertedValueTab[32]����
        5��ADC��
                    ADC TIM1 CC2������12λ�����Ҷ��룬����ת��ͨ��PA5���Զ�ע��ת��ͨ��PA4��ģ���ѹ����
        6��TIM��
                    TIM1����CH2�����CH2��������ADCת��        
    ʹ�÷�����
        1�������򿪵���ģʽ��������ADC_RegularConvertedValueTab[32],ADC_InjectedConvertedValueTab[32]��ӵ�watch���ڹ۲�
        2��ͨ���ı�PA4 PA5���ŵĵ�ѹ����ÿ��CC2�¼�����ʱת��һ�ι����ע��ͨ�������������ڶ�Ӧ�����С�ͬʱ��PA9 ���Կ���TIM1 CH2�� PWM ����
4��ע������
    ��ϵͳ����HSEʱ��ʱ��һ��HSIҲ�Ǵ򿪵ģ���RCC_ConfigAdc1mClk(RCC_ADC1MCLK_SRC_HSE, RCC_ADC1MCLK_DIV8)��������ΪHSE����HSI
    ��ϵͳ����HSIʱ��ʱ��һ��HSE�ǹرյģ���RCC_ConfigAdc1mClk(RCC_ADC1MCLK_SRC_HSI, RCC_ADC1MCLK_DIV8)ֻ������ΪHSI