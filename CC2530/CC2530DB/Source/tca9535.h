#ifndef _TCA9535_H
#define _TCA9535_H



#define   uchar unsigned char
#define   uint  unsigned int
typedef unsigned char BYTE;
typedef unsigned long   uint32;

//使用的端口，请按照以下接线
#define	IO_SCL                     P2_0                //IIC时钟引脚定义
#define	IO_SDA                     P0_1                //IIC数据引脚定义
#define IO_INT                     P0_0                //中断

#define	TCA9535_IC_SlaveAddress           0x24	            //定义器件PCF8574PWR在IIC总线中的从地址

#define IO_SA_WRITE(sa)             (sa<<1)
#define IO_SA_READ(sa)             ((sa<<1)|1)

void TCA9535_IO_SDADirIn(void);
void TCA9535_IO_SDADirOut(void);

//************
void TCA9535_delay(unsigned int k);
void TCA9535_Delayus(unsigned int usec);
void Init_TCA9535(void);

void TCA9535_Write(uchar SlaveAddress,uchar REG_data);                 //单个写入数据
BYTE TCA9535_Read_One(uchar SlaveAddress, unsigned int command);      //单个读取内部寄存器数据
uint32 TCA9535_Read_All(void);                                        //连续的读取内部寄存器数据
//以下是模拟iic使用函数-------------

void TCA9535_Start(void);
void TCA9535_Stop(void);
void TCA9535_SendACK(char ack);
char TCA9535_RecvACK(void);
BYTE TCA9535_SendByte(BYTE dat);
BYTE TCA9535_RecvByte(void);
void TCA9535_ReadPage(void);
void TCA9535_WritePage(void);

#endif /* _TCA9535_H */ 
