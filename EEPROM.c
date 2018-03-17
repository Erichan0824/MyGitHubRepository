#include <iom88p.h>
#include <inavr.h>
/*
#pragma vector = EE_RDY_vect
__interrupt void EEPROM_READY_interrupt(void)
{
}
*/
void writeChar(int position,char data)
{
  __disable_interrupt();
  EEAR = position;   //��ַ�Ĵ���
  EEDR = data;   //���ݼĴ���
  EECR |= (1<<EEMPE);  // EEPROM ���ƼĴ���  λ2 �C EEMWE: EEPROM дʹ�� EEMWE��������EEWEΪ"1���Ƿ����
  EECR |= (1<<EEPE);  // λ1 �C EEPE: EEPROM дʹ��  ��λEEWE ������д����E
  __enable_interrupt();
  SMCR = (1<<SE);
  __sleep();
  SMCR = 0;
}

void writeBlock(int position,char *buffer,char num)
{  
  char i;
  while (EECR & (1<<EEPE)); //�ȴ���һ��д��������
  EECR |= (1<<EERIE);  //λ3 �C EERIE: ʹ��EEPROM ׼�����ж�
  for(i=0;i<num;i++)
    writeChar(position+i,*(buffer+i));
  EECR &= ~(1<<EERIE);
}

char readChar(int position)
{
  EEAR = position; 
  EECR |= (1<<EERE); //λ 0 �C EERE: EEPROM ��ʹ��  ����EERE ������������ */
  return EEDR;
}

void readBlock(char *buffer,int position,char num)
{
  char i;
  while (EECR & (1<<EEPE)); //�ȴ���һ��д��������
  for(i=0;i<num;i++)  //
    *(buffer+i) = readChar(position+i);
}
                              

