#ifndef _TCA9535_H
#define _TCA9535_H

#define   uchar unsigned char
#define   uint  unsigned int
typedef unsigned char BYTE;
typedef unsigned long   uint32;

#define TCA9535_IO_INT                     P0_0                //�ж�
#define TCA9535_IO_INT_PORT                0
#define TCA9535_IO_INT_PIN                 0

#define TCA9535_IC_SlaveAddress   0x24

void Init_TCA9535(void);
void TCA9535_ClrISR(void);

void TCA9535_Write(uint address,uchar REG_data);               //����д������
BYTE TCA9535_Read_One(uint address,unsigned int command);      //������ȡ�ڲ��Ĵ�������
uint32 TCA9535_Read_All(void);                    //�����Ķ�ȡ�ڲ��Ĵ�������
int TCA9535_Get_INT_Status(int d);

#endif /* _TCA9535_H */ 
