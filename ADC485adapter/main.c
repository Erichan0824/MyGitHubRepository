/******************************************
* Copyright (c) 2016.10.17 青岛澳科仪器
* All rights reserved.
* super 
单片机 mega88pa23474
晶振  1.8432M
串口-485 控制引脚PD2.
4路AD ：AD0-3 = PC0-3
4路电源控制： PD4-7


**************************************************/

#include <inavr.h>
#include <iom88p.h>
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
#define ID_ID 0X10         // 0x10   0001.0000
   

#define BUFFER_SIZE 30                

#define stateBase 0x70
#define idBase 0x50
 
#define CPU_CRYSTAL (1.8432)
#define delay_us(us)  __delay_cycles((unsigned long int)(us*CPU_CRYSTAL))
#define delay_ms(ms) __delay_cycles( (unsigned long)(ms) *(unsigned long)(1843.2))

__flash char commands[] = {0x92,0x58,0x57};  
__flash char sensorStateCode[] = {"S99S00S01S02S03S04S05" };
enum{sNone, sNormal, sLow, sHigh, sOpen, sShort,sRes};                                    

#define RE PORTD_Bit2
#define DE PORTD_Bit3 
#define U485_OUT    DE=1, RE=1       // 高电平 发送  
#define U485_IN     DE=0, RE=0       // 低电平接收
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
  char command;       // 存的指令； 共用；
  char comBuffer[30];
  char txBuffer[30];
  char sensorState[4];  //传感器状态
} sram;
sram global;

#pragma vector = INT0_vect    //External Interrupt Request 0
__interrupt void INT0_interrupt(void)
{
  EIMSK = 0;
}

#pragma vector = EE_RDY_vect   //EEPROM Ready
__interrupt void EEPROM_READY_interrupt(void)
{
}

#pragma vector = TIMER1_OVF_vect   //定时器溢1出中断
__interrupt void ISR_TOV1(void)
{
  TCCR1B = 0;                    // 关时钟；  
  if( fstate == IDLE )
    __watchdog_reset();
  else
     T1OV = 1;
}

#pragma vector = USART_RX_vect    //串口接收中断
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

char isValid(char value)   //判断是否是有效的命令
{
  char flag = 0, i;
  char __flash *flashPointer;   //编译器存储器类型修饰。 表示指针指向的变量需要被保存在Flash中。
  flashPointer = commands;
  for(i=0;i<20;i++)       
  {
    if(*(flashPointer+i)==value)    // 为什么是20   2016.11.18   没必要20 
    {
      flag=1;
      break;
    }
  }
  return flag;  
}

void transmit(char *data,char num)   //串口发送子函数
{
  char i;
  PRR |= (1<<PRTIM1);   //关闭
  U485_OUT;   //184芯片为发送状态
  UCSR0B = (1<<UDRIE0)|(1<<TXEN0);  //位5 – UDRIEn: USART 数据寄存器空中断使能  位3 – TXENn: 发送使能
  for(i=0;i<num;i++)
  {
    UDR0 = *data;
    data++;
    SMCR = (1<<SE);    // IDLE 
    __sleep();
    SMCR = 0;
  }
  UCSR0B = (1<<TXCIE0)|(1<<TXEN0);  //位6 – TXCIEn: 发送结束中断使能
  SMCR = (1<<SE);
  __sleep();
  SMCR = 0;
  U485_IN;  //184芯片为接收状态 
  UCSR0B = (1<<RXCIE0)|(1<<RXEN0);   //位7 – RXCIEn: 接收结束中断使能   位4 – RXENn: 接收使能
  PRR &= ~(1<<PRTIM1);  //
}

void WTDinit(void)
{
  __disable_interrupt();
  __watchdog_reset();
  WDTCSR |= (1<<WDCE)|(1<<WDE);
  WDTCSR = (1<<WDE)|(1<<WDP2)|(1<<WDP1)|(1<<WDP0);    // 
  __enable_interrupt(); 
}

void USARTinit(void)
{
  U485_IN;
  UCSR0B = (1<<RXCIE0)|(1<<RXEN0);//串口0接受使能 和 接受中断使能
  UCSR0C = (1<<UCSZ01)|(1<<UCSZ00);
  UBRR0 = 23;      //4800
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
  ADMUX = (1<<REFS0);//    参考源 AVCC
  ADCSRA = (1<<ADEN)|(1<<ADPS2);//16PRESCALER 1.8432M/16=115.2KHZ
  ADCSRB = 0; 
}



void dataInit(void) 
{
  sram *gv = &global;
  readBlock(gv->comBuffer, idBase, 1);  //#define idBase 0x50
  GPIOR0 = ((gv->comBuffer[0]>=0xA0) ? gv->comBuffer[0] : 0xFF);  //通用I/O寄存器0 存储探班的地址   0xA0  160 
  fstate = IDLE;
  readBlock(gv->sensorState, stateBase, 4);
}
/*
PB1,PB2=ENpow1,2
PD6,PD7=Enpow3,4
L = POWER ON
*/
void portInit(void)
{
  PORTB = 0XFF;        //PORTB  0B 1111.1111       DDRB 0000.0110
  DDRB = 0X06;
  PORTC = 0xff;        //PORTC  0B  1111.1111       DDRC 0000.0000输入态
  DDRC = 0;             
  PORTD = 0xF7;        //PORTD   0B  1111.0111      DDRD 1100.1100    初始化 应该 既发送也不接收（3、4位处理）
  DDRD = 0xCC;
}

void ADchannelInput()
{
  PORTC = 0XFF;              // 1111.1111
  DDRC = 0;
}

void ADchannelHighZ()
{
  PORTC = 0XF0;        //1111.0000
  DDRC = 0;
}

void PowerDownInit(void)
{
  MCUCR = 0;  
  ACSR = (1<<ACD);                                                 //模拟比较器禁用
  ADCSRA &= ~(1<<ADEN);                                            //PRR关断ADC之前，要先关断ADC。
  PRR = (1<<PRTWI)|(1<<PRTIM2)|(1<<PRTIM0)|(1<<PRSPI)|(1<<PRADC);  //关闭IIC 定时器2 定时器0  spi口  模拟比较器
  DIDR0 = (1<<ADC5D)|(1<<ADC4D)|(1<<ADC3D)|(1<<ADC2D)|(1<<ADC1D)|(1<<ADC0D);  //数字输入禁止寄存器0     
  DIDR1 = (1<<AIN1D)|(1<<AIN0D);  //数字输入禁止寄存器1
  ADchannelHighZ();
}

void systemInit(void)
{
  ACSR |= (1<<ACD);    // ANALOG COMPARATOR IS SWITCHED OFF
  portInit(); 
  TIMER1init();       // 定时器1 开中断;
  ADCinit();
  PowerDownInit();
  dataInit();         //读取 地址；    
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


void powerOnAD(char channel)//0-3  AD0--AD3
{
  switch(channel)
  {
  case 0:
    PORTB_Bit1 = 0;
    break;
  case 1:
    PORTB_Bit2 = 0;
    break;
  case 2:
    PORTD_Bit6 = 0;
    break;
  case 3:
    PORTD_Bit7 = 0;
    break;
  default:
    break;
  }
}


void powerOffAD(char channel)
{
  switch(channel)
  {
  case 0:
    PORTB_Bit1 = 1;
    break;
  case 1:
    PORTB_Bit2 = 1;
    break;
  case 2:
    PORTD_Bit6 = 1;
    break;
  case 3:
    PORTD_Bit7 = 1;
    break;
  default:
    break;
  }
}

void getSensorState(void)
{
  sram *gv = &global;
  char i = 0;
  unsigned int value = 0;
  unsigned int res = 0;
  PRR &= ~(1<<PRADC);  //PRR需要放在ADEN之前打开。否则不能正常工作。
  ADCSRA |= (1<<ADEN); 
  ADchannelInput();
  for(i =0; i<4; i++)
  {
    if(gv->sensorState[i] == sNone)              // 如果是预留态 那就预留着；
    {
      continue;
    } 
    powerOnAD(i);
    delay_us(5);
    value = getADvalue(i);
    powerOffAD(i);

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
  tempBuffer[0] = result & 0xFF;     // 0B  1111.1111
  if( (tempBuffer[0]>= 0xa0)&&(tempBuffer[0]<=0xaf) )
  {
    writeBlock(idBase, tempBuffer, 1);
    GPIOR0 = tempBuffer[0];               //顺便把全局变量变了
  }
}

void  processRead(char value)
{
  char index = 0, i=0;
  char crc=0;
  sram *gv = &global; 
  char __flash *flashPtr;   //编译器存储器类型修饰。 表示指针指向的变量需要被保存在Flash中。
  char sIndex = 0;
  flashPtr = sensorStateCode;
  gv->txBuffer[index++] = STX;          
  switch(value)
  {
    case 0x92:   //???
      getSensorState();
      for(i=0; i<4; i++)
      {
        sIndex = 3*gv->sensorState[i]; 
        gv->txBuffer[index++] = *(flashPtr+sIndex+0);              // index 是个局部变量
        gv->txBuffer[index++] = *(flashPtr+sIndex+1); 
        gv->txBuffer[index++] = *(flashPtr+sIndex+2);
        gv->txBuffer[index++] = ':';                       
      }
      index--;                         // 最后一个
      break;
    default:   //
      break;
  }
  gv->txBuffer[index++] = ETX;
  crc = generateCRC(gv->txBuffer, index);
  gv->txBuffer[index++] = crc;
  PRR &= ~(1<<PRUSART0);  //打开串口
  transmit(gv->txBuffer,index); //发送准备好的需要返回的字节
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
  TCCR1B = 0;         //定时器关 
  UCSR0B = 0;         // 串口使能关
  PRR |= (1<<PRUSART0);
  gv->txBuffer[index++]=STX;
  switch(value)
  {
  case 0x58:  //写地址
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
  PRR &= ~(1<<PRUSART0);  //打开串口
  transmit(gv->txBuffer,index); //发送准备好的需要返回的字节
}

void echo(char address, char value)   //数据返回
{
  sram *gv=&global;
  gv->txBuffer[0] = GPIOR0;      // 返回 地址
  gv->txBuffer[1] = value;       //返回指令
  transmit(gv->txBuffer,2);
  if(value==0x58 || value==0x57 )   //如果是写命令。。。
  {   
    TCNT1= 36736;//51136;                 1s //0.5 s
    TCCR1B = (1<<CS10)|(1<<CS11);    //64 PRE , max=2.27s
    TIMSK1 = (1<<TOIE1);   //定时器溢出中断使能
    fstate = WRITE_COMMAND;
    ptr  = 0;
  }
  else                                //读指令的
  {
    UCSR0B = 0;
    U485_OFF;
    PRR |= (1<<PRUSART0);  //关闭串口0
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
      __watchdog_reset();       //如果我直接发5个字符呢？ ptr还能归零吗
      if( ptr == 1 )                          //不明白了?为什么是 ==1;  >0  类似BUG     如果改为>0呢
      {
        TCCR1B = 0;  //时钟停止 
        TCNT1 = 56320;          //Max time between Address and command 5ms   65535-56320=9215   1/1.8432M*9215=5ms
        TCCR1B = (1<<CS10);     //  无预分频      无预分频    ,最大35ms                                
        TIMSK1 = (1<<TOIE1);    //位0 – TOIE1: T/C1 溢出中断使能
        fstate = ID_ID;
        break;
      }
      __disable_interrupt();
      __watchdog_reset();
      __enable_interrupt();  
      TCNT1= 36736;//51136;                 1s  //0.5 s 65535-36736=28799       1/1.8432M*28799 *64=0.99s  1s就可以唤醒一次
      TCCR1B = (1<<CS10)|(1<<CS11);             //64 PRE , max=2.27s
      TIMSK1 = (1<<TOIE1);                      //定时器溢出中断使能  
      MCUCR = (1<<BODSE)|(1<<BODS);             // 2016.11.21 BOD sleep(加) 不顶用啊 
      MCUCR = (1<<BODS);
      MCUCR &=~(1<<BODSE);
      __no_operation();
      __no_operation();
      __no_operation();
      SMCR = (1<<SE);
      __sleep();
      SMCR = 0;
      break;
    case ID_ID:
      if( ptr  > 1 )                // 如果再多传一个字符；      
      {
        TCCR1B = 0;               // 时钟停止；
        fstate = ID_ACCORD;
        break;
      }
      else if( T1OV >0 )          //T1 OV >0;  如果还是两个字符
      {
        T1OV = 0;                //且 中断1 溢出
        fstate = IDLE;           //IDLE
        ptr  = 0;
        break;
      }
      SMCR = (1<<SE);
      __sleep();
      SMCR = 0;
      break;
    case ID_ACCORD:
      if( (gv->comBuffer[0]==0xff)||(gv->comBuffer[0]==GPIOR0) )  // 如果指令匹配；
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
      if(isValid(gv->command))  //判断命令是否有效。。。
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
      if( ptr  >0 )             // 如果
      {
        if((gv->comBuffer[0] == SOH)&&(gv->comBuffer[ptr -1]==EOT) )
        {
          TCCR1B = 0;
          fstate |= DATA_READY;            // DATA_READY
          U485_OFF;
          break;
        }
        else if( ptr  >= BUFFER_SIZE )    // 没必要30吧；
        {
          TCCR1B = 0;      //T1 时钟关 
          fstate = IDLE;
          ptr  = 0;
          break;
        }
      }
      if( T1OV > 0 )
      {
        T1OV = 0;         // 溢出标志位 清零。
        ptr  = 0;
        fstate = IDLE;
        break;
      }
      SMCR = (1<<SE);
      __sleep();
      SMCR = 0;
      break;
    case (WRITE_COMMAND|DATA_READY):              // 0B 0100  | 0B 1000
      processWrite( gv->command );
      ptr  = 0;
      fstate = IDLE;
      break;
    default:
      break;
    }
  }
}
