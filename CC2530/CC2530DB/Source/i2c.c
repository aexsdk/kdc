#include  <ioCC2530.h>  
#include "i2c.h"
#include "kkwAppHw.h"
#include <ioCC2530.h>
#include "OSAL.h"
#include "ZGlobals.h"
#include "OSAL_Nv.h"
#include "AF.h"
#include "aps_groups.h"
#include "ZDApp.h"
#include "ZComDef.h"


#define TRUE	1
#define FALSE	0
bool I2CErr = FALSE;

/**************************************
��ʱ1΢��
��ͬ�Ĺ�������,��Ҫ�����˺�����ע��ʱ�ӹ���ʱ��Ҫ�޸�
������1T��MCUʱ,���������ʱ����
**************************************/
//#pragma optimize=none
void I2C_Delayus(unsigned int usec)
{
    usec<<= 2;
    while(usec--)
    {
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
    }
}

//#pragma optimize=none
void I2C_delay(unsigned int n)	
{						
  uint i;
  
  for(i=0;i<n;i++);
  for(i=0;i<n;i++);
  for(i=0;i<n;i++);
  for(i=0;i<n;i++);
  for(i=0;i<n;i++);
  
  for(i=0;i<n;i++);
  for(i=0;i<n;i++);
  for(i=0;i<n;i++);
  for(i=0;i<n;i++);
  for(i=0;i<n;i++);
  
  
  for(i=0;i<n;i++);
  for(i=0;i<n;i++);
  for(i=0;i<n;i++);
  for(i=0;i<n;i++);
  for(i=0;i<n;i++);
  
  
  for(i=0;i<n;i++);
  for(i=0;i<n;i++);
  for(i=0;i<n;i++);
  for(i=0;i<n;i++);
  for(i=0;i<n;i++);
}

int I2C_SetScl(BYTE v)
{
  I2C_IO_SCLDirOut();
  I2C_Delayus(5);
  IO_SCL = v;
  I2C_Delayus(5);
  if(I2C_GetScl() == v)
    return TRUE;
  else
    return FALSE;
}

int I2C_GetScl(void)
{
  I2C_IO_SCLDirIn();
  I2C_Delayus(5);
  return IO_SCL;
}

int I2C_SetSda(BYTE v)
{
  I2C_IO_SDADirOut();
  I2C_Delayus(5);
  IO_SDA = v;
  I2C_Delayus(5);
  if(I2C_GetSda() == v)
    return TRUE;
  else
    return FALSE;
}

int I2C_GetSda(void)
{
  I2C_IO_SDADirIn();
  I2C_Delayus(5);
  return IO_SDA;
}

/**************************************
��ʼ�ź�
**************************************/
void I2C_Start(void)
{
  I2C_SetSda(1);                    //����������
  I2C_SetScl(1);                    //����ʱ����
  I2C_SetSda(0);                    //�����½���
  I2C_SetScl(0);                    //����ʱ����
  I2C_Delayus(5);  
}

/**************************************
ֹͣ�ź�
**************************************/
void I2C_Stop(void)
{   
  I2C_SetScl(0);
  I2C_SetSda(0);
  I2C_SetSda(1);    //����������
  I2C_Delayus(5);   //��ʱ
}

/**************************************
����Ӧ���ź�
��ڲ���:ack (0:ACK 1:NAK)
**************************************/
void I2C_SendACK(char ack)
{   
  if(ack != 0){
    I2C_SetSda(0);
  }
  I2C_SetScl(1);
  I2C_SetScl(0);
}

/**************************************
����Ӧ���ź�
**************************************/

char I2C_RecvACK()
{
  char ret = 0;

  I2C_SetSda(1);
  I2C_SetScl(1);
  ret = I2C_GetSda();
  I2C_SetScl(0);
  I2C_Delayus(5);
  return !ret;
}

/**************************************
��IIC���߷���һ���ֽ�����
**************************************/
BYTE I2C_SendByte(BYTE dat)
{
  BYTE i;
  for (i=0; i<8; i++)         //8λ������
  {
    I2C_SetScl(0);
    if(dat&0x80){
      I2C_SetSda(1);
    }else{
      I2C_SetSda(0); 
    }
    I2C_SetScl(1);
    dat <<=1;
    I2C_Delayus(5);
  }  
  return I2C_RecvACK();
}

/**************************************
��IIC���߽���һ���ֽ�����
**************************************/
BYTE I2C_RecvByte(BYTE ack)
{
  BYTE i;
  BYTE dat = 0;

  I2C_SetScl(0);
  I2C_SetSda(1);                    //ʹ���ڲ�����,׼����ȡ����,
  for (i=0; i<8; i++)         //8λ������
  {
    dat <<= 1;
    I2C_SetScl(1);                //����ʱ����
    dat |= I2C_GetSda();             //������               
    I2C_SetScl(0);                //����ʱ����
  }
  I2C_SendACK(!!ack);
  return 1;
}

void Init_I2C()
{
  //#define	IO_SCL                     P2_0                //IICʱ�����Ŷ���
  //#define	IO_SDA                     P0_1                //IIC�������Ŷ���
  KKW_IO_DIR_OUTPUT(2,0);    //Set port 2.0 to output
}

void I2C_IO_SDADirIn(void)
{
  //P0DIR &= 0xfc;
  KKW_IO_DIR_INPUT(0,1);    //Set port 0.1 to input
}

void I2C_IO_SDADirOut(void)
{
  //P0DIR |= 0x02; 
  KKW_IO_DIR_OUTPUT(0,1); //set port 0.1 to output
}

void I2C_IO_SCLDirIn(void)
{
  //P0DIR &= 0xfc;
  KKW_IO_DIR_INPUT(IO_SCL_PORT,IO_SCL_PIN);    //Set port 0.1 to input
}

void I2C_IO_SCLDirOut(void)
{
  //P0DIR |= 0x02; 
  KKW_IO_DIR_OUTPUT(IO_SCL_PORT,IO_SCL_PIN); //set port 0.1 to output
}