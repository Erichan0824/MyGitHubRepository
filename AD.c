#define MY_AD_GLOBALS
#include "includes.h"


void adc_init(void)
{
  ADMUX = 1<<REFS0;
  ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1);//64��Ƶ 172.8K@11.0592M
  DIDR0 = 0x1F;
}

INT16U read_adc(INT8U adc_input)//��ѯ��ʽ��ȡADC����ͨ��
{
  INT8U index;
  INT16U adc_temp = 0;
  
  ADMUX &= 0xF8;
  ADMUX |= adc_input;
  
  for(index = 0; index < 5; index++)      //����5��ת����ƽ��ֵ
  {
    ADCSRA |= (1<<ADSC); //����ADת��
    while ((ADCSRA&(1<<ADIF))==0); 
    ADCSRA |= (1<<ADIF); //д1�����־λ
    adc_temp += ADC;
  }
  
  return (adc_temp/5); //ADC=ADCH:ADCL
}
