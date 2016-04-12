#ifndef _RNS110_H
#define _RNS110_H

#define   uchar unsigned char
#define   uint  unsigned int
typedef unsigned char BYTE;
typedef unsigned long   uint32;

#define RNS110_IO_INT                     P0_6                //中断
#define RNS110_IO_VEN                     P0_7
#define RNS110_IO_UPGRADE                 P1_3

#define	RNS110_IC_SlaveAddress           0x28	            

void Init_RNS110(void);
int  RNS110_Write(char *buf,uint count);       //单个写入数据
BYTE RNS110_Read(char *buf,uint maxsize);      //单个读取内部寄存器数据
void RNS110_SetPower(BYTE onoff);
void RNS110_SetUpgrade(BYTE onoff);
int RNS110_Reset(void);

#endif /* _RNS110_H */ 
