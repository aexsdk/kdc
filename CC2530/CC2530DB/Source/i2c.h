#ifndef _I2C_H
#define _I2C_H



#define   uchar unsigned char
#define   uint  unsigned int
typedef unsigned char BYTE;
typedef unsigned long   uint32;

//使用的端口，请按照以下接线
#define	IO_SCL                     P2_0                //IIC时钟引脚定义
#define IO_SCL_PORT                2
#define IO_SCL_PIN                 0
#define	IO_SDA                     P0_1                //IIC数据引脚定义
#define IO_SDA_PORT                0
#define IO_SDA_PIN                 1

#define IO_SA_WRITE(sa)             (sa<<1)
#define IO_SA_READ(sa)             ((sa<<1)|1)

void I2C_IO_SDADirIn(void);
void I2C_IO_SDADirOut(void);
void I2C_IO_SCLDirIn(void);
void I2C_IO_SCLDirOut(void);

int I2C_SetScl(BYTE v);
int I2C_GetScl(void);
int I2C_SetSda(BYTE v);
int I2C_GetSda(void);

//************
void I2C_delay(unsigned int k);
void I2C_Delayus(unsigned int usec);
void Init_I2C(void);

void I2C_Start(void);
void I2C_Stop(void);
void I2C_SendACK(char ack);
char I2C_RecvACK(void);
BYTE I2C_SendByte(BYTE dat);
BYTE I2C_RecvByte(BYTE ack);

#endif /* _I2C_H */ 
