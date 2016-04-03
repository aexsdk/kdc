#ifndef _PCF8574PWR_H
#define _PCF8574PWR_H



#define   uchar unsigned char
#define   uint  unsigned int
typedef unsigned char BYTE;
typedef unsigned long   uint32;

//使用的端口，请按照以下接线
#define	IO_SCL                     P2_0                //IIC时钟引脚定义
#define	IO_SDA                     P0_1                //IIC数据引脚定义
#define IO_INT                     P0_0                //中断

#define	IO_SlaveAddress0           0x20	            //定义器件PCF8574PWR在IIC总线中的从地址
#define	IO_SlaveAddress1           0x21	            //定义器件PCF8574PWR在IIC总线中的从地址
#define	IO_SlaveAddress2           0x22	            //定义器件PCF8574PWR在IIC总线中的从地址
#define	IO_SlaveAddress3           0x23	            //定义器件PCF8574PWR在IIC总线中的从地址
#define	IO_SlaveAddress4           0x24	            //定义器件PCF8574PWR在IIC总线中的从地址
#define	IO_SlaveAddress5           0x25	            //定义器件PCF8574PWR在IIC总线中的从地址
#define	IO_SlaveAddress6           0x26	            //定义器件PCF8574PWR在IIC总线中的从地址
#define	IO_SlaveAddress7           0x27	            //定义器件PCF8574PWR在IIC总线中的从地址

#define IO_SA_WRITE(sa)             (sa<<1)
#define IO_SA_READ(sa)             ((sa<<1)|1)

void IO_SDADirIn(void);
void IO_SDADirOut(void);

//************
void delay(unsigned int k);
void Delayus(unsigned int usec);
void Init_PCF8574(void);

void PCF8574_Write(uchar SlaveAddress,uchar REG_data);   //单个写入数据
BYTE PCF8574_Read_One(uchar SlaveAddress);                                  //连续的读取内部寄存器数据
uint32 PCF8574_Read_All(void);                                  //连续的读取内部寄存器数据
//以下是模拟iic使用函数-------------
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
