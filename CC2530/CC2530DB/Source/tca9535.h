#ifndef _TCA9535_H
#define _TCA9535_H



#define   uchar unsigned char
#define   uint  unsigned int
typedef unsigned char BYTE;
typedef unsigned long   uint32;

//ʹ�õĶ˿ڣ��밴�����½���
#define	IO_SCL                     P2_0                //IICʱ�����Ŷ���
#define	IO_SDA                     P0_1                //IIC�������Ŷ���
#define IO_INT                     P0_0                //�ж�

#define	TCA9535_IC_SlaveAddress           0x24	            //��������PCF8574PWR��IIC�����еĴӵ�ַ

#define IO_SA_WRITE(sa)             (sa<<1)
#define IO_SA_READ(sa)             ((sa<<1)|1)

void TCA9535_IO_SDADirIn(void);
void TCA9535_IO_SDADirOut(void);

//************
void TCA9535_delay(unsigned int k);
void TCA9535_Delayus(unsigned int usec);
void Init_TCA9535(void);

void TCA9535_Write(uchar SlaveAddress,uchar REG_data);                 //����д������
BYTE TCA9535_Read_One(uchar SlaveAddress, unsigned int command);      //������ȡ�ڲ��Ĵ�������
uint32 TCA9535_Read_All(void);                                        //�����Ķ�ȡ�ڲ��Ĵ�������
//������ģ��iicʹ�ú���-------------

void TCA9535_Start(void);
void TCA9535_Stop(void);
void TCA9535_SendACK(char ack);
char TCA9535_RecvACK(void);
BYTE TCA9535_SendByte(BYTE dat);
BYTE TCA9535_RecvByte(void);
void TCA9535_ReadPage(void);
void TCA9535_WritePage(void);

#endif /* _TCA9535_H */ 
