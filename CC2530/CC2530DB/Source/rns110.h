#ifndef _RNS110_H
#define _RNS110_H

#include "i2c.h"

#define   uchar unsigned char
#define   uint  unsigned int
typedef unsigned char BYTE;
typedef unsigned long   uint32;

#define RNS110_IO_INT                     P1_5                //中断
#define RNS110_IO_VEN                     P0_7
#define RNS110_IO_UPGRADE                 P1_3

#define	RNS110_IC_SlaveAddress           0x28	            

int RNS110_Get_Can_Read(void);
void Init_RNS110(void);
void RNS110_Enable_irq(void);
void RNS110_Disable_irq(void);
int  RNS110_Write(char *buf,uint count);       //单个写入数据
BYTE RNS110_Read(char *buf,BYTE len);
void RNS110_SetPower(BYTE onoff);
void RNS110_SetUpgrade(BYTE onoff);
int RNS110_Reset(void);
int RNS110_Write_cmd1(void);
int RNS110_Get_INT_Status(int d);

#endif /* _RNS110_H */ 
