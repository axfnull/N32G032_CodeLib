1������˵��
    1��ADC����ͨ������PA4��PA5���ŵ�ģ���ѹ��ע��ͨ������PA0��PA1���ŵ�ģ���ѹ
    2�����й���ת�����ͨ��DMA_CH1ͨ����ȡ������ADC_RegularConvertedValueTab[64]����
           ע��ת�����ͨ��ת�������ж϶�ȡ������ADC_InjectedConvertedValueTab[32]����
2��ʹ�û���
    �������������  KEIL MDK-ARM V5.26.2.0
    Ӳ��������      ������N32G032R8L7_STB����
3��ʹ��˵��
    ϵͳ���ã�
        1��ʱ��Դ��
                    HSE=8M,PLL=48M,AHB=48M,APB1=48M,APB2=48M,ADC CLK=48M/16,ADC 1M CLK=HSI/8,DMA CLK=48M
        2���жϣ�
                    ADCע��ת���������жϴ򿪣����ȼ�0
                    �жϴ������ע��ת�������ADC_InjectedConvertedValueTab[32]����
        3���˿����ã�
                    PA4ѡ��Ϊģ�⹦��ADת��ͨ��
                    PA5ѡ��Ϊģ�⹦��ADCת��ͨ��
                    PA0ѡ��Ϊģ�⹦��ADCת��ͨ��
                    PA1ѡ��Ϊģ�⹦��ADCת��ͨ��
                    PA9ѡ��Ϊ�ⲿEXTI�¼������ش���
                    PA10ѡ��Ϊ�ⲿEXTI�¼������ش���
        4��DMA��
                    DMA_CH1ͨ���ػ�ģʽ����64�����ֵ�ADC����ͨ��ת�������ADC_RegularConvertedValueTab[64]����
        5��ADC��
                    ADC����ͨ��ɨ����ģʽ��EXTI9������12λ�����Ҷ��룬ת��ͨ��PA4��PA5��ģ���ѹ����
                    ADCע��ͨ��ɨ��ģʽ��EXTI10������12λ�����Ҷ��룬ת��ͨ��PA0��PA1��ģ���ѹ����
    ʹ�÷�����
        1�������򿪵���ģʽ��������ADC_RegularConvertedValueTab[64],ADC_InjectedConvertedValueTab[32]��ӵ�watch���ڹ۲�
        2��ͨ��PA9�������ؿ��Դ�������ͨ�����ݲ�����PA10�������ؿ��Դ���ע��ͨ�����ݲ���
4��ע������
    ��ϵͳ����HSEʱ��ʱ��һ��HSIҲ�Ǵ򿪵ģ���RCC_ConfigAdc1mClk(RCC_ADC1MCLK_SRC_HSE, RCC_ADC1MCLK_DIV8)��������ΪHSE����HSI
    ��ϵͳ����HSIʱ��ʱ��һ��HSE�ǹرյģ���RCC_ConfigAdc1mClk(RCC_ADC1MCLK_SRC_HSI, RCC_ADC1MCLK_DIV8)ֻ������ΪHSI
    ������Ĭ��PA9,PA10��ñ�ӵ�NSLINK�����⴮�ڣ���������PA9��PA10�������ڣ�����������;����ε�������ñ��