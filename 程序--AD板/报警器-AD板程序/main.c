/******************************************
* Copyright (c) 2016.10.17 �ൺ�Ŀ�����
* All rights reserved.
* super 
��Ƭ�� mega88pa23474
����  11.0592M
����-485 ��������PD2 PD3
8·AD ��
·��Դ���ƣ� PD4


**************************************************/

#include <inavr.h>
#include <iom48p.h>
#include <stdlib.h>

#define SOH          0x01
#define STX          0x02  
#define ETX          0x03
#define EOT          0x04

#define IDLE 0
#define ID_ACCORD 1
#define COMMAND 2
#define WRITE_COMMAND 4
#define DATA_READY 8
#define ID_ID 0X10
   

#define BUFFER_SIZE 30                

#define stateBase 0x70
#define idBase 0x50
 
#define CPU_CRYSTAL (11.0592)           // 2016.11.19�޸�
#define delay_us(us)  __delay_cycles((unsigned long int)(us*CPU_CRYSTAL))
#define delay_ms(ms) __delay_cycles( (unsigned long)(ms) *(unsigned long)(11059.2))

__flash char commands[] = {0x92,0x58,0x57};  
__flash char sensorStateCode[] = {"S99S00S01S02S03S04S05" };
enum{sNone, sNormal, sLow, sHigh, sOpen, sShort,sRes};                                    

#define RE PORTD_Bit3
#define DE PORTD_Bit2
#define AD_PWRON  PORTD_Bit4 = 0
#define AD_PWROFF PORTD_Bit4 = 1

#define U485_OUT    DE=1, RE=1
#define U485_IN     DE=0, RE=0
#define U485_OFF    DE=0, RE=1

#define fstate GPIOR1
#define ptr    GPIOR2

char T1OV =0;
char generateCRC(char *data,char num);
int ASCIItoINT(char *data,char delimiter);
void readBlock(char *buffer,int address,char num);
void writeBlock(int address,char *buffer,char num);
char intToASCII(unsigned int data,char *buffer);

typedef struct
{
  char command;  
  char comBuffer[30];
  char txBuffer[30];
  char  sensorState[8];   //2016.11.19 �޸�
} sram;
sram global;
//char test[8]={"test!!!!" };
#pragma vector = INT0_vect    //External Interrupt Request 0
__interrupt void INT0_interrupt(void)
{
  EIMSK = 0;
}

#pragma vector = EE_RDY_vect   //EEPROM Ready
__interrupt void EEPROM_READY_interrupt(void)
{
}

#pragma vector = TIMER1_OVF_vect   //��ʱ����1���ж�
__interrupt void ISR_TOV1(void)
{
  TCCR1B = 0;
  if( fstate == IDLE )
    __watchdog_reset();
  else
     T1OV = 1;
}

#pragma vector = USART_RX_vect    //���ڽ����ж�
__interrupt void UART_RX_interrupt(void) 
{  
  sram *gv = &global;
  char p = ptr;
  gv->comBuffer[p] = UDR0; 
  ptr++;
}                                

#pragma vector = USART_UDRE_vect  //USART, Data Register Empty
__interrupt void UART_UDRE_interrupt(void)
{
}

#pragma vector = USART_TX_vect   //USART, Tx Complete 
__interrupt void USART_TX_interrupt(void)
{
}

char isValid(char value)   //�ж��Ƿ�����Ч������
{
  char flag = 0, i;
  char __flash *flashPointer;   //�������洢���������Ρ� ��ʾָ��ָ��ı�����Ҫ��������Flash�С�
  flashPointer = commands;
  for(i=0;i<20;i++)       
  {
    if(*(flashPointer+i)==value)
    {
      flag=1;
      break;
    }
  }
  return flag;  
}

void transmit(char *data,char num)   //���ڷ����Ӻ���
{
  char i;
  PRR |= (1<<PRTIM1);   //�ر�
  U485_OUT;   //184оƬΪ����״̬
  UCSR0B = (1<<UDRIE0)|(1<<TXEN0);  //λ5 �C UDRIEn: USART ���ݼĴ������ж�ʹ��  λ3 �C TXENn: ����ʹ��
  for(i=0;i<num;i++)
  {
    UDR0 = *data;
    data++;
    SMCR = (1<<SE);
    __sleep();
    SMCR = 0;
  }
  UCSR0B = (1<<TXCIE0)|(1<<TXEN0);  //λ6 �C TXCIEn: ���ͽ����ж�ʹ��
  SMCR = (1<<SE);
  __sleep();
  SMCR = 0;
  U485_IN;  //184оƬΪ����״̬ 
  UCSR0B = (1<<RXCIE0)|(1<<RXEN0);   //λ7 �C RXCIEn: ���ս����ж�ʹ��   λ4 �C RXENn: ����ʹ��
  PRR &= ~(1<<PRTIM1);  //
}

void WTDinit(void)
{
  __disable_interrupt();
  __watchdog_reset();
  WDTCSR |= (1<<WDCE)|(1<<WDE);
  WDTCSR = (1<<WDE)|(1<<WDP2)|(1<<WDP1)|(1<<WDP0);
  __enable_interrupt(); 
}

void USARTinit(void)
{
  U485_IN;
  UCSR0B = (1<<RXCIE0)|(1<<RXEN0);//����0����ʹ�� �� �����ж�ʹ��
  UCSR0C = (1<<UCSZ01)|(1<<UCSZ00);  //8λ 0 ֹͣλ
  UBRR0 = 143;   //4800               //2016.11.19�޸�  ���; 
}

void TIMER1init(void)
{
  TIMSK1 |= (1<<TOIE1);
  TCCR1A = 0;
  TCCR1B = 0;// (1<<CS11)|(1<<CS10);//CLK/64. MAX = 1.137S
  TCNT1 = 0;
}

void ADCinit(void)
{
  ADMUX = (1<<REFS0);// AVCC
  ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1);//(1<<ADEN)|(1<<ADPS2);//16PRESCALER 1.8432M/16=115.2KHZ   11.0592M/128= 86400=86.4K ����11.0592M/64= 172800=172.8K   
  ADCSRB = 0; 
}



void dataInit(void) 
{
  sram *gv = &global;
  readBlock(gv->comBuffer, idBase, 1);  //#define idBase 0x50
  GPIOR0 = ((gv->comBuffer[0]>=0xA0) ? gv->comBuffer[0] : 0xFF);  //ͨ��I/O�Ĵ���0 �洢̽��ĵ�ַ
  fstate = IDLE;
  readBlock(gv->sensorState, stateBase, 8);     //   2016.11.19�޸�
}
/*
PD4
L = POWER ON
*/
void portInit(void)
{
  PORTB = 0XFF;    // PORTB   1111.1111  DDRB 0000.0110   ���������    2016.11.19�޸�
  DDRB = 0X00;
  PORTC = 0xff;    //PORTC    1111.1111  DDRC 0000.0000
  DDRC = 0;
  PORTD = 0xF7;   //PORTD     1111.0111 DDRD 1100 1100  // PD2 PD3��485���ƣ�  PD4����Դ�� 0001 1100 2016.11.19�޸�
  DDRD = 0x1C;
}

void ADchannelInput()
{
  PORTC = 0XFF;
  DDRC = 0;
}

void ADchannelHighZ()
{
  PORTC = 0X00;
  DDRC = 0;
}

void PowerDownInit(void)
{
  MCUCR = 0;  
  ACSR = (1<<ACD);  //ģ��Ƚ�������
  ADCSRA &= ~(1<<ADEN);//PRR�ض�ADC֮ǰ��Ҫ��ʧ��ADC��
  PRR = (1<<PRTWI)|(1<<PRTIM2)|(1<<PRTIM0)|(1<<PRSPI)|(1<<PRADC);  //�ر�IIC ��ʱ��2 ��ʱ��0  spi��  ģ��Ƚ���
  DIDR0 = (1<<ADC5D)|(1<<ADC4D)|(1<<ADC3D)|(1<<ADC2D)|(1<<ADC1D)|(1<<ADC0D);  //���������ֹ�Ĵ���0
  DIDR1 = (1<<AIN1D)|(1<<AIN0D);  //���������ֹ�Ĵ���1
  ADchannelHighZ();
}

void systemInit(void)
{
  ACSR |= (1<<ACD); // ANALOG COMPARATOR IS SWITCHED OFF
  portInit(); 
  TIMER1init();
  ADCinit();
  PowerDownInit();
  dataInit();
  USARTinit();
  __enable_interrupt();
} 

unsigned int getADvalue(char channel) //channel = 0--7.ADC0---ADC7;
{
  char i=0;
  unsigned int temp = 0;
  ADMUX = (1<<REFS0);
  ADMUX |= channel;
  for(i = 0; i< 5; i++ )
  {
    ADCSRA |= (1<<ADSC);//AD START CONVERSION
    while( ( ADCSRA&(1<<ADIF) )== 0 );
    ADCSRA |= (1<<ADIF);//CLEAR FLAG
    temp += ADC;
  }
  return( temp/5);
}

void getSensorState(void)
{
  sram *gv = &global;
  char i = 0;
  unsigned int value = 0;
  unsigned int res = 0;
  AD_PWRON;
  delay_us(5);
  PRR &= ~(1<<PRADC);  //PRR��Ҫ����ADEN֮ǰ�򿪡�����������������
  ADCSRA |= (1<<ADEN); 
  ADchannelInput();
  for(i =0; i<8; i++)
  {
    if(gv->sensorState[i] == sNone)
    {
      continue;
    } 
    value = getADvalue(i);

    res = (unsigned long int)value*1000/(1024-value);
    if( (res>1220)&&(res<=1680) )
    {
      gv->sensorState[i] = sNormal;
    }
    else if( (res>=750)&&(res<=1220) )
    {
      gv->sensorState[i] = sHigh;
    }
    else if( (res>480) && (res <= 750) )
    {
      gv->sensorState[i] = sLow;
    }
    else if( res <= 480 )
    {
      gv->sensorState[i] = sShort;
    }
    else if( res>1680 )
    {
      gv->sensorState[i] = sOpen;
    }
   }
  AD_PWROFF;
  ADCSRA &= ~(1<<ADEN);//ADC must be disabled ,before pre shutdown adc
  PRR |= (1<<PRADC);
  ADchannelHighZ();
}


void writeID(void)
{
  int result; 
  sram *gv = &global;
  char tempBuffer[2];
  result = ASCIItoINT(&(gv->comBuffer[1]),EOT);
  tempBuffer[0] = result & 0xFF;
  if( (tempBuffer[0]>= 0xa0)&&(tempBuffer[0]<=0xaf) )
  {
    writeBlock(idBase, tempBuffer, 1);
    GPIOR0 = tempBuffer[0];
  }
}

void  processRead(char value)
{
  char index = 0, i=0;
  char crc=0;
  sram *gv = &global; 
  char __flash *flashPtr;   //�������洢���������Ρ� ��ʾָ��ָ��ı�����Ҫ��������Flash�С�
  char sIndex = 0;
  flashPtr = sensorStateCode;
  gv->txBuffer[index++] = STX;
  switch(value)
  {
    case 0x92:   //???
      getSensorState();
      for(i=0; i<8; i++)
      {
        sIndex = 3*gv->sensorState[i];
        gv->txBuffer[index++] = *(flashPtr+sIndex+0);
        gv->txBuffer[index++] = *(flashPtr+sIndex+1); 
        gv->txBuffer[index++] = *(flashPtr+sIndex+2);
        gv->txBuffer[index++] = ':';
      }
      index--;  
      break;
    default:   //
      break;
  }
  gv->txBuffer[index++] = ETX;
  crc = generateCRC(gv->txBuffer, index);
  gv->txBuffer[index++] = crc;
  PRR &= ~(1<<PRUSART0);  //�򿪴���
  transmit(gv->txBuffer,index); //����׼���õ���Ҫ���ص��ֽ�
  fstate = IDLE;
}
char readIDASCII(int address,char *buffer)   
{
  char index=0;  
  char tempBuffer[2];
  readBlock(tempBuffer,address,1);
  index+=intToASCII(tempBuffer[0], buffer+index);  
  return index;
}
void writeState()
{
  char i,result=0; 
  sram *gv = &global;
  for(i=0; i<4; i++)
  {
     result = gv->comBuffer[i*2+1];
     if( result==0x30 || result ==0x31 )
       gv->sensorState[i] = result-0x30;
  }
  writeBlock(stateBase,gv->sensorState, 4);
}
char readStateASCII(char *target)
{
  char index =0;
  int i=0;sram *gv = &global;
  readBlock(gv->sensorState,stateBase,  4);
  for(i=0; i<4; i++)
  {
    index=i*2;
    target[index] = (gv->sensorState[i]==0)?0x30:0x31;
    index++;
    target[index++] = ':';
  }
  index--;
  return index;
}

void processWrite(char value)
{
  char index = 0;
  char crc;
  sram *gv = &global;
  TCCR1B = 0;
  UCSR0B = 0;
  PRR |= (1<<PRUSART0);
  gv->txBuffer[index++]=STX;
  switch(value)
  {
  case 0x58:  //д��ַ
    writeID();
    index+=readIDASCII(idBase,&(gv->txBuffer[index]));
    break;
  case 0x57: //
    writeState();
    index += readStateASCII(gv->txBuffer+index);
    break;
  }
  gv->txBuffer[index++] = ETX;
  crc = generateCRC(gv->txBuffer, index);
  gv->txBuffer[index++] = crc;
  PRR &= ~(1<<PRUSART0);  //�򿪴���
  transmit(gv->txBuffer,index); //����׼���õ���Ҫ���ص��ֽ�
}

void echo(char address, char value)   //���ݷ���
{
  sram *gv=&global;
  gv->txBuffer[0] = GPIOR0;
  gv->txBuffer[1] = value;
  transmit(gv->txBuffer,2);
  if(value==0x58 || value==0x57 )   //�����д�������
  {   
    TCNT1=  22335;//36736;//51136;                 1s //0.5 s
    TCCR1B =(1<<CS12);   //256  PRE         //(1<<CS10)|(1<<CS11);    //64 PRE , max=2.27s
    TIMSK1 = (1<<TOIE1);   //��ʱ������ж�ʹ��
    fstate = WRITE_COMMAND;
    ptr  = 0;
  }
  else
  {
    UCSR0B = 0;
    U485_OFF;
    PRR |= (1<<PRUSART0);  //�رմ���0
    processRead(value);
    fstate = IDLE;
    ptr  = 0;
  }
}

__C_task void main( void )
{
  sram *gv=&global;
  systemInit();  
  WTDinit();
  ptr  = 0;
//  for(int i=0; i<10; i++)
//    gv->txBuffer[i] = 0x30+i;
//  gv->txBuffer[0] = GPIOR0;
//  transmit(gv->txBuffer,10);
  __watchdog_reset();
  
  
  for(;;)
  {
    switch( fstate )
    {
    case IDLE:
//      transmit(test,8);
      __watchdog_reset();
      if( ptr == 1 )
      {
        TCCR1B = 0;  //ʱ��ֹͣ
        TCNT1 = 10239;          //Max time between Address and command 5ms    65535-10239=55296 1/11.0592M*55296=5ms
        TCCR1B = (1<<CS10);     //  ��Ԥ��Ƶ         ,���                                
        TIMSK1 = (1<<TOIE1);    //λ0 �C TOIE1: T/C1 ����ж�ʹ��
        fstate = ID_ID;
        break;
      }
      __disable_interrupt();
      __watchdog_reset();
      __enable_interrupt();  
      TCNT1= 22335;   //36736;//51136;                 1s //0.5 s      65535-22335=43200  1/11.0592*43200*256=
    //  TCCR1B = (1<<CS10)|(1<<CS11);    //64 PRE     1/11.0592M*?=1s      
       TCCR1B = (1<<CS12);   //256��Ƶ
      TIMSK1 = (1<<TOIE1);   //��ʱ������ж�ʹ��     
      SMCR = (1<<SE);
      __sleep();
      SMCR = 0;
      break;
    case ID_ID:
      if( ptr  > 1 )
      {
        TCCR1B = 0;
        fstate = ID_ACCORD;
        break;
      }
      else if( T1OV >0 )
      {
        T1OV = 0;
        fstate = IDLE;
        ptr  = 0;
        break;
      }
      SMCR = (1<<SE);
      __sleep();
      SMCR = 0;
      break;
    case ID_ACCORD:
      if( (gv->comBuffer[0]==0xff)||(gv->comBuffer[0]==GPIOR0) )
      {
        gv->command = gv->comBuffer[1];
        fstate = COMMAND;
      }
      else
      {
        fstate = IDLE;
        ptr  = 0;
      }
      break;
    case COMMAND:
      if(isValid(gv->command))  //�ж������Ƿ���Ч������
      {
        echo(GPIOR0,gv->command);
      }
      else
      {
        fstate = IDLE;
        ptr  = 0;
      }
      break;
    case WRITE_COMMAND:
      if( ptr  >0 )
      {
        if((gv->comBuffer[0] == SOH)&&(gv->comBuffer[ptr -1]==EOT) )
        {
          TCCR1B = 0;
          fstate |= DATA_READY;
          U485_OFF;
          break;
        }
        else if( ptr  >= BUFFER_SIZE )
        {
          TCCR1B = 0;
          fstate = IDLE;
          ptr  = 0;
          break;
        }
      }
      if( T1OV > 0 )
      {
        T1OV = 0;
        ptr  = 0;
        fstate = IDLE;
        break;
      }
      SMCR = (1<<SE);
      __sleep();
      SMCR = 0;
      break;
    case (WRITE_COMMAND|DATA_READY):
      processWrite( gv->command );
      ptr  = 0;
      fstate = IDLE;
      break;
    default:
      break;
    }
  }
}
