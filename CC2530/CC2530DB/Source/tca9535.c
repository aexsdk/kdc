#include  <ioCC2530.h>  
#include "tca9535.h"
#include "kkwAppHw.h"



/**************************************
��ʱ1΢��
��ͬ�Ĺ�������,��Ҫ�����˺�����ע��ʱ�ӹ���ʱ��Ҫ�޸�
������1T��MCUʱ,���������ʱ����
**************************************/
//#pragma optimize=none
void TCA9535_Delayus(unsigned int usec)
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
void TCA9535_delay(unsigned int n)	
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
void TCA9535_Start(void)
{
    TCA9535_IO_SDADirOut();
    TCA9535_Delayus(5);  
    IO_SDA = 1;                    //����������
    IO_SCL = 1;                    //����ʱ����
    TCA9535_Delayus(5);                 //��ʱ
    IO_SDA = 0;                    //�����½���
    TCA9535_Delayus(5);                 //��ʱ
    IO_SCL = 0;                    //����ʱ����
    TCA9535_Delayus(5);  
}

/**************************************
ֹͣ�ź�
**************************************/
void TCA9535_Stop(void)
{   
    IO_SCL = 0;                       
    TCA9535_Delayus(1);  
    TCA9535_IO_SDADirOut();
    TCA9535_Delayus(1);  
    IO_SDA = 0;                    
    TCA9535_Delayus(5);                 //��ʱ
    IO_SDA = 1;                    //����������
    TCA9535_Delayus(5);                 //��ʱ
}

/**************************************
����Ӧ���ź�
��ڲ���:ack (0:ACK 1:NAK)
**************************************/
void TCA9535_SendACK(char ack)
{   
    IO_SCL = 0;
    TCA9535_Delayus(5);                
    TCA9535_IO_SDADirOut();
    TCA9535_Delayus(5);      
    IO_SDA = ack;                  //дӦ���ź�
    IO_SCL = 1;                    //����ʱ����
    TCA9535_Delayus(5);                 //��ʱ
    IO_SCL = 0;                    //����ʱ����
    TCA9535_Delayus(5);                 //��ʱ
}

/**************************************
����Ӧ���ź�
**************************************/

char TCA9535_RecvACK()
{
    IO_SCL = 0;
    TCA9535_Delayus(5);
    TCA9535_IO_SDADirOut();
    IO_SDA = 1;
    TCA9535_IO_SDADirOut();
    TCA9535_Delayus(5);          //�˴��Ƿ��б�ҪʹSDA�����ߣ�������
    IO_SCL = 1;
    TCA9535_Delayus(5);
    if(IO_SDA == 1)
    {
      IO_SCL = 0;
      return 0;  //er
    }
    IO_SCL = 0;
    return 1;
}

/**************************************
��IIC���߷���һ���ֽ�����
**************************************/
BYTE TCA9535_SendByte(BYTE dat)
{
    BYTE i;
    TCA9535_IO_SDADirOut();
    
    for (i=0; i<8; i++)         //8λ������
    {
        IO_SCL = 0;
            TCA9535_Delayus(5);
        if(dat&0x80){
          IO_SDA = 1;
              TCA9535_Delayus(5);
        }else{
          IO_SDA = 0; 
              TCA9535_Delayus(5);
        }
        IO_SCL = 1;
        dat <<=1;
            TCA9535_Delayus(5);
    }  
    return TCA9535_RecvACK();
}

/**************************************
��IIC���߽���һ���ֽ�����
**************************************/
BYTE TCA9535_RecvByte()
{
    BYTE i;
    BYTE dat = 0;

    IO_SCL = 0;
    TCA9535_IO_SDADirOut();
    IO_SDA = 1;                    //ʹ���ڲ�����,׼����ȡ����,
    TCA9535_Delayus(5);
    TCA9535_IO_SDADirOut();
    for (i=0; i<8; i++)         //8λ������
    {
        dat <<= 1;
        IO_SCL = 1;                //����ʱ����
        TCA9535_Delayus(5);             //��ʱ
        dat |= IO_SDA;             //������               
        IO_SCL = 0;                //����ʱ����
        TCA9535_Delayus(5);             //��ʱ
    }
    return dat;
}

//***************************************************

void TCA9535_Write(uchar SlaveAddress,uchar REG_data)
{  
    TCA9535_Start();                  //��ʼ�ź�
    TCA9535_SendByte(IO_SA_WRITE(SlaveAddress));   //�����豸��ַ+д�ź�
    TCA9535_SendByte(REG_data);       //�ڲ��Ĵ������ݣ���ο�����pdf
    
    TCA9535_Stop();                   //����ֹͣ�ź�
    TCA9535_delay(10);
}

BYTE TCA9535_Read_One(uchar SlaveAddress, unsigned int command)
{
  BYTE r = 0;
    
  BYTE ret = 0;
  TCA9535_Start();                                            //��ʼ�ź�
  ret = TCA9535_SendByte(IO_SA_WRITE(SlaveAddress));           //�����豸��ַ+д�ź�
  if(ret == 0){
    //TCA9535_Stop(); 
    //return 0;
  }
  
  ret = TCA9535_SendByte(command);           //�����豸��ַ+д�ź�
  if(ret == 0){
    //TCA9535_Stop(); 
    //return 0;
  }
  
  TCA9535_Start(); 
  ret = TCA9535_SendByte(IO_SA_READ(SlaveAddress));           //�����豸��ַ+д�ź�
  if(ret == 0){
    //TCA9535_Stop(); 
    //return 0;
  }
  
  r = TCA9535_RecvByte();
  TCA9535_Stop();                                   //ֹͣ�ź�
  TCA9535_delay(100);
  return r;
}

void Init_TCA9535()
{
//#define	IO_SCL                     P2_0                //IICʱ�����Ŷ���
//#define	IO_SDA                     P0_1                //IIC�������Ŷ���
//#define       IO_INT                     P0_0                //�ж�  
       P0SEL = 0xfc;
       P2SEL = 0xfe;
  
       P2DIR= 0x01;
       P0DIR= 0xfe;
}

void TCA9535_IO_SDADirIn(void)
{
       P0DIR &= 0xfc;
}

void TCA9535_IO_SDADirOut(void)
{
       P0DIR |= 0x02; 
}