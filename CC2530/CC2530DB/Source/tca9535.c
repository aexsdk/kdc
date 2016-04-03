#include  <ioCC2530.h>  
#include "tca9535.h"
#include "kkwAppHw.h"



/**************************************
延时1微秒
不同的工作环境,需要调整此函数，注意时钟过快时需要修改
当改用1T的MCU时,请调整此延时函数
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
起始信号
**************************************/
void TCA9535_Start(void)
{
    TCA9535_IO_SDADirOut();
    TCA9535_Delayus(5);  
    IO_SDA = 1;                    //拉高数据线
    IO_SCL = 1;                    //拉高时钟线
    TCA9535_Delayus(5);                 //延时
    IO_SDA = 0;                    //产生下降沿
    TCA9535_Delayus(5);                 //延时
    IO_SCL = 0;                    //拉低时钟线
    TCA9535_Delayus(5);  
}

/**************************************
停止信号
**************************************/
void TCA9535_Stop(void)
{   
    IO_SCL = 0;                       
    TCA9535_Delayus(1);  
    TCA9535_IO_SDADirOut();
    TCA9535_Delayus(1);  
    IO_SDA = 0;                    
    TCA9535_Delayus(5);                 //延时
    IO_SDA = 1;                    //产生上升沿
    TCA9535_Delayus(5);                 //延时
}

/**************************************
发送应答信号
入口参数:ack (0:ACK 1:NAK)
**************************************/
void TCA9535_SendACK(char ack)
{   
    IO_SCL = 0;
    TCA9535_Delayus(5);                
    TCA9535_IO_SDADirOut();
    TCA9535_Delayus(5);      
    IO_SDA = ack;                  //写应答信号
    IO_SCL = 1;                    //拉高时钟线
    TCA9535_Delayus(5);                 //延时
    IO_SCL = 0;                    //拉低时钟线
    TCA9535_Delayus(5);                 //延时
}

/**************************************
接收应答信号
**************************************/

char TCA9535_RecvACK()
{
    IO_SCL = 0;
    TCA9535_Delayus(5);
    TCA9535_IO_SDADirOut();
    IO_SDA = 1;
    TCA9535_IO_SDADirOut();
    TCA9535_Delayus(5);          //此处是否有必要使SDA先拉高？！？！
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
向IIC总线发送一个字节数据
**************************************/
BYTE TCA9535_SendByte(BYTE dat)
{
    BYTE i;
    TCA9535_IO_SDADirOut();
    
    for (i=0; i<8; i++)         //8位计数器
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
从IIC总线接收一个字节数据
**************************************/
BYTE TCA9535_RecvByte()
{
    BYTE i;
    BYTE dat = 0;

    IO_SCL = 0;
    TCA9535_IO_SDADirOut();
    IO_SDA = 1;                    //使能内部上拉,准备读取数据,
    TCA9535_Delayus(5);
    TCA9535_IO_SDADirOut();
    for (i=0; i<8; i++)         //8位计数器
    {
        dat <<= 1;
        IO_SCL = 1;                //拉高时钟线
        TCA9535_Delayus(5);             //延时
        dat |= IO_SDA;             //读数据               
        IO_SCL = 0;                //拉低时钟线
        TCA9535_Delayus(5);             //延时
    }
    return dat;
}

//***************************************************

void TCA9535_Write(uchar SlaveAddress,uchar REG_data)
{  
    TCA9535_Start();                  //起始信号
    TCA9535_SendByte(IO_SA_WRITE(SlaveAddress));   //发送设备地址+写信号
    TCA9535_SendByte(REG_data);       //内部寄存器数据，请参考中文pdf
    
    TCA9535_Stop();                   //发送停止信号
    TCA9535_delay(10);
}

BYTE TCA9535_Read_One(uchar SlaveAddress, unsigned int command)
{
  BYTE r = 0;
    
  BYTE ret = 0;
  TCA9535_Start();                                            //起始信号
  ret = TCA9535_SendByte(IO_SA_WRITE(SlaveAddress));           //发送设备地址+写信号
  if(ret == 0){
    //TCA9535_Stop(); 
    //return 0;
  }
  
  ret = TCA9535_SendByte(command);           //发送设备地址+写信号
  if(ret == 0){
    //TCA9535_Stop(); 
    //return 0;
  }
  
  TCA9535_Start(); 
  ret = TCA9535_SendByte(IO_SA_READ(SlaveAddress));           //发送设备地址+写信号
  if(ret == 0){
    //TCA9535_Stop(); 
    //return 0;
  }
  
  r = TCA9535_RecvByte();
  TCA9535_Stop();                                   //停止信号
  TCA9535_delay(100);
  return r;
}

void Init_TCA9535()
{
//#define	IO_SCL                     P2_0                //IIC时钟引脚定义
//#define	IO_SDA                     P0_1                //IIC数据引脚定义
//#define       IO_INT                     P0_0                //中断  
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