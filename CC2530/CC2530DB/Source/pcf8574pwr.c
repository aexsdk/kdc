#include  <ioCC2530.h>  
#include "pcf8574pwr.h"
#include "kkwAppHw.h"


/*******************************/
//#pragma optimize=none
void delay(unsigned int n)	
{						
  uint i;
  for(i=0;i<n;i++);
  for(i=0;i<n;i++);
  for(i=0;i<n;i++);
  for(i=0;i<n;i++);
  for(i=0;i<n;i++);					
}
	

/**************************************
��ʱ1΢��
��ͬ�Ĺ�������,��Ҫ�����˺�����ע��ʱ�ӹ���ʱ��Ҫ�޸�
������1T��MCUʱ,���������ʱ����
**************************************/
//#pragma optimize=none
void Delayus(unsigned int usec)
{
    usec>>= 1;
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

/**************************************
��ʼ�ź�
**************************************/
void PCF8574_Start(void)
{
    IO_SDADirOut();
    Delayus(5);  
    IO_SDA = 1;                    //����������
    IO_SCL = 1;                    //����ʱ����
    Delayus(5);                 //��ʱ
    IO_SDA = 0;                    //�����½���
    Delayus(5);                 //��ʱ
    IO_SCL = 0;                    //����ʱ����
    Delayus(5);  
}

/**************************************
ֹͣ�ź�
**************************************/
void PCF8574_Stop(void)
{   
    IO_SCL = 0;                       
    Delayus(1);  
    IO_SDADirOut();
    Delayus(1);  
    IO_SDA = 0;                    
    Delayus(5);                 //��ʱ
    IO_SDA = 1;                    //����������
    Delayus(5);                 //��ʱ
}

/**************************************
����Ӧ���ź�
��ڲ���:ack (0:ACK 1:NAK)
**************************************/
void PCF8574_SendACK(char ack)
{   
    IO_SCL = 0;
    Delayus(5);                
    IO_SDADirOut();
    Delayus(5);      
    IO_SDA = ack;                  //дӦ���ź�
    IO_SCL = 1;                    //����ʱ����
    Delayus(5);                 //��ʱ
    IO_SCL = 0;                    //����ʱ����
    Delayus(5);                 //��ʱ
}

/**************************************
����Ӧ���ź�
**************************************/
char PCF8574_RecvACK()
{
    IO_SCL = 0;
    Delayus(5);
    IO_SDADirOut();
    IO_SDA = 1;
    IO_SDADirIn();
    Delayus(5);          //�˴��Ƿ��б�ҪʹSDA�����ߣ�������
    IO_SCL = 1;
    Delayus(5);
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
BYTE PCF8574_SendByte(BYTE dat)
{
    BYTE i;
    IO_SDADirOut();
    
    for (i=0; i<8; i++)         //8λ������
    {
        IO_SCL = 0;
            Delayus(5);
        if(dat&0x80){
          IO_SDA = 1;
              Delayus(5);
        }else{
          IO_SDA = 0; 
              Delayus(5);
        }
        IO_SCL = 1;
        dat <<=1;
            Delayus(5);
    }
        
 //   IO_SCL = 0;
 //   Delayus(5);
//    IO_SDA = 1;  
//    Delayus(5);
//    IO_SCL = 1;
    
    return PCF8574_RecvACK();
}

/**************************************
��IIC���߽���һ���ֽ�����
**************************************/
BYTE PCF8574_RecvByte()
{
    BYTE i;
    BYTE dat = 0;

    IO_SCL = 0;
    IO_SDADirOut();
    IO_SDA = 1;                    //ʹ���ڲ�����,׼����ȡ����,
    Delayus(5);
    IO_SDADirIn();
    for (i=0; i<8; i++)         //8λ������
    {
        dat <<= 1;
        IO_SCL = 1;                //����ʱ����
        Delayus(5);             //��ʱ
        dat |= IO_SDA;             //������               
        IO_SCL = 0;                //����ʱ����
        Delayus(5);             //��ʱ
    }
    return dat;
}

//***************************************************

void PCF8574_Write(uchar SlaveAddress,uchar REG_data)
{  
    PCF8574_Start();                  //��ʼ�ź�
    PCF8574_SendByte(IO_SA_WRITE(SlaveAddress));   //�����豸��ַ+д�ź�
    PCF8574_SendByte(REG_data);       //�ڲ��Ĵ������ݣ���ο�����pdf
    
    PCF8574_Stop();                   //����ֹͣ�ź�
    delay(10);
}
//******************************************************
//
//����PCF8574
//
//******************************************************
BYTE PCF8574_Read_One(uchar SlaveAddress)
{
  BYTE r = 0;
  BYTE ret = 0;
  PCF8574_Start();                                      //��ʼ�ź�
  ret = PCF8574_SendByte(IO_SA_READ(SlaveAddress));     //�����豸��ַ+д�ź�
  if(ret == 0){
    //PCF8574_Stop(); 
    //return 0;
  }                                                    //���һ��������Ҫ��NOACK
  r = PCF8574_RecvByte();          
  PCF8574_Stop();                                      //ֹͣ�ź�
  delay(10);
  return r;
}

uint32 PCF8574_Read_All(void)
{
  uint32 r = 0;
  BYTE *pr = (BYTE *)&r;
  *pr++ = PCF8574_Read_One(IO_SlaveAddress0);
  *pr++ = PCF8574_Read_One(IO_SlaveAddress1);
  *pr++ = PCF8574_Read_One(IO_SlaveAddress2);
  *pr = PCF8574_Read_One(IO_SlaveAddress3);
  return r;
}

//��ʼ��PCF8574��������Ҫ��ο�pdf�����޸�****
//#define IO_SDADirOut()             KKW_IO_DIR_OUTPUT_PREP(0,1)    //P1DIR |= 0x02;      //xxxx1M01
//#define IO_SDADirIn()              KKW_IO_DIR_INPUT_PREP(0,1)     //P1DIR &= ~0x02;
void Init_PCF8574()
{
  
//#define	IO_SCL                     P2_0                //IICʱ�����Ŷ���
//#define	IO_SDA                     P0_1                //IIC�������Ŷ���
//#define       IO_INT                     P0_0                //�ж�
  
  

  
//    P0DIR |= 0x02;  //����write�����������
    P0DIR &= 0xFD;  //����read�����������
    P2DIR |= 0x01;  // read write 
    
    P0SEL &= 0xFD;
    P2SEL &= 0xFE;
}

void IO_SDADirIn(void)
{
       P0DIR &= 0xFD;  //����read����������� 
}

void IO_SDADirOut(void)
{
       P0DIR |= 0x02;  //����write����������� 
}