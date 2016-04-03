#ifndef _PCF8574PWR_H
#define _PCF8574PWR_H



#define   uchar unsigned char
#define   uint  unsigned int
typedef unsigned char BYTE;
typedef unsigned long   uint32;

//ʹ�õĶ˿ڣ��밴�����½���
#define	IO_SCL                     P2_0                //IICʱ�����Ŷ���
#define	IO_SDA                     P0_1                //IIC�������Ŷ���
#define IO_INT                     P0_0                //�ж�

#define	IO_SlaveAddress0           0x20	            //��������PCF8574PWR��IIC�����еĴӵ�ַ
#define	IO_SlaveAddress1           0x21	            //��������PCF8574PWR��IIC�����еĴӵ�ַ
#define	IO_SlaveAddress2           0x22	            //��������PCF8574PWR��IIC�����еĴӵ�ַ
#define	IO_SlaveAddress3           0x23	            //��������PCF8574PWR��IIC�����еĴӵ�ַ
#define	IO_SlaveAddress4           0x24	            //��������PCF8574PWR��IIC�����еĴӵ�ַ
#define	IO_SlaveAddress5           0x25	            //��������PCF8574PWR��IIC�����еĴӵ�ַ
#define	IO_SlaveAddress6           0x26	            //��������PCF8574PWR��IIC�����еĴӵ�ַ
#define	IO_SlaveAddress7           0x27	            //��������PCF8574PWR��IIC�����еĴӵ�ַ

#define IO_SA_WRITE(sa)             (sa<<1)
#define IO_SA_READ(sa)             ((sa<<1)|1)

void IO_SDADirIn(void);
void IO_SDADirOut(void);

//************
void delay(unsigned int k);
void Delayus(unsigned int usec);
void Init_PCF8574(void);

void PCF8574_Write(uchar SlaveAddress,uchar REG_data);   //����д������
BYTE PCF8574_Read_One(uchar SlaveAddress);                                  //�����Ķ�ȡ�ڲ��Ĵ�������
uint32 PCF8574_Read_All(void);                                  //�����Ķ�ȡ�ڲ��Ĵ�������
//������ģ��iicʹ�ú���-------------
void Delayus(unsigned int usec);
void PCF8574_Start(void);
void PCF8574_Stop(void);
void PCF8574_SendACK(char ack);
char PCF8574_RecvACK(void);
BYTE PCF8574_SendByte(BYTE dat);
BYTE PCF8574_RecvByte(void);
void PCF8574_ReadPage(void);
void PCF8574_WritePage(void);

#endif /* _PCF8574PWR_H */ 
