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
	

/**************************************
��ʼ�ź�
**************************************/
void I2C_Start(void)
{
    I2C_IO_SDADirOut();
    I2C_Delayus(5);  
    IO_SDA = 1;                    //����������
    IO_SCL = 1;                    //����ʱ����
    I2C_Delayus(5);                 //��ʱ
    IO_SDA = 0;                    //�����½���
    I2C_Delayus(5);                 //��ʱ
    IO_SCL = 0;                    //����ʱ����
    I2C_Delayus(5);  
}

/**************************************
ֹͣ�ź�
**************************************/
void I2C_Stop(void)
{   
    IO_SCL = 0;                       
    I2C_Delayus(1);  
    I2C_IO_SDADirOut();
    I2C_Delayus(1);  
    IO_SDA = 0;                    
    I2C_Delayus(5);                 //��ʱ
    IO_SDA = 1;                    //����������
    I2C_Delayus(5);                 //��ʱ
}

/**************************************
����Ӧ���ź�
��ڲ���:ack (0:ACK 1:NAK)
**************************************/
void I2C_SendACK(char ack)
{   
    IO_SCL = 0;
    I2C_Delayus(5);                
    I2C_IO_SDADirOut();
    I2C_Delayus(5);      
    IO_SDA = ack;                  //дӦ���ź�
    IO_SCL = 1;                    //����ʱ����
    I2C_Delayus(5);                 //��ʱ
    IO_SCL = 0;                    //����ʱ����
    I2C_Delayus(5);                 //��ʱ
}

/**************************************
����Ӧ���ź�
**************************************/

char I2C_RecvACK()
{
  uint8 times=255;			//������ϣ��趨�������

  IO_SCL = 0;
  I2C_Delayus(5);
  I2C_IO_SDADirOut();
  IO_SDA = 1;
  I2C_IO_SDADirIn();
  I2C_Delayus(5);          //�˴��Ƿ��б�ҪʹSDA�����ߣ�������
  IO_SCL = 1;
  I2C_Delayus(5);
  while(IO_SDA)
  { 
    times--;
    if(!times)				//��ʱֵΪ255
    {
      I2C_Stop();
      I2CErr=TRUE;			
      return FALSE;
    }
  }
  IO_SCL = 0; 
  I2CErr = FALSE;
  return TRUE;
}

/**************************************
��IIC���߷���һ���ֽ�����
**************************************/
BYTE I2C_SendByte(BYTE dat)
{
    BYTE i;
    I2C_IO_SDADirOut();
    
    for (i=0; i<8; i++)         //8λ������
    {
        IO_SCL = 0;
            I2C_Delayus(5);
        if(dat&0x80){
          IO_SDA = 1;
              I2C_Delayus(5);
        }else{
          IO_SDA = 0; 
              I2C_Delayus(5);
        }
        IO_SCL = 1;
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

    IO_SCL = 0;
    I2C_IO_SDADirOut();
    IO_SDA = 1;                    //ʹ���ڲ�����,׼����ȡ����,
    I2C_Delayus(5);
    I2C_IO_SDADirIn();
    for (i=0; i<8; i++)         //8λ������
    {
        dat <<= 1;
        IO_SCL = 1;                //����ʱ����
        I2C_Delayus(5);             //��ʱ
        dat |= IO_SDA;             //������               
        IO_SCL = 0;                //����ʱ����
        I2C_Delayus(5);             //��ʱ
    }
    I2C_SendACK(!!ack);
    return dat;
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