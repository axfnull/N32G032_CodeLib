1������˵��
    1��ADC������ת��PA1���ŵ�ģ���ѹ
    2������ADCת�����ͨ��DMA_CH1ͨ����ȡ������ADCConvertedValue
2��ʹ�û���
    �������������  KEIL MDK-ARM V5.26.2.0
    Ӳ��������      ����N32G032R8L7_STB����
3��ʹ��˵��
    ϵͳ���ã�
        1��ʱ��Դ��
                    HSE=8M,PLL=48M,AHB=48M,APB1=48M,APB2=48M,ADC CLK=48M/16,ADC 1M CLK=HSI/8,DMA CLK=48M
        2���˿����ã�
                    PA1ѡ��Ϊģ�⹦��ADCת��ͨ��1
        3��DMA��
                    DMA_CH1ͨ���ػ�ģʽ����һ�����ֵ�ADCת�������ADCConvertedValue����
        4��ADC��
                    ADC����ת����ɨ��ģʽ�����������12λ�����Ҷ��룬ת��ͨ��1��PA1��ģ���ѹ����
    ʹ�÷�����
        1�������򿪵���ģʽ��������ADCConvertedValue��ӵ�watch���ڹ۲�
        2��ͨ���ı�PA1���ŵĵ�ѹ�����Կ���ת���������ͬ���ı�
4��ע������
    ��ϵͳ����HSEʱ��ʱ��һ��HSIҲ�Ǵ򿪵ģ���RCC_ConfigAdc1mClk(RCC_ADC1MCLK_SRC_HSE, RCC_ADC1MCLK_DIV8)��������ΪHSE����HSI
    ��ϵͳ����HSIʱ��ʱ��һ��HSE�ǹرյģ���RCC_ConfigAdc1mClk(RCC_ADC1MCLK_SRC_HSI, RCC_ADC1MCLK_DIV8)ֻ������ΪHSI