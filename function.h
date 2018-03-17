


#define delay_us(us) __delay_cycles(36864*us/10000)
#define delay_ms(ms) __delay_cycles(36864*ms/10)

char BCDtoChar(char bcd);
char BCDtoChar(char bcd);
char charToBCD(char data);


char intToASCII(unsigned int data,char *buffer);

char generateCRC(char *data,char num);
int ASCIItoINT(char *data,char delimiter);

/*
������
1�����̰���----------��������һ��
2������--------------�Ƶ�������������
3�����羲��----------�Ƶ�������������
4��й©--------------���������������
5��й©����----------�Ƶ�������������
6���ޱ�--------------����
*/