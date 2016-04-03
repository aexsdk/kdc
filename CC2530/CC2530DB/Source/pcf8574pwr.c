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
延时1微秒
不同的工作环境,需要调整此函数，注意时钟过快时需要修改
当改用1T的MCU时,请调整此延时函数
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
起始信号
**************************************/
void PCF8574_Start(void)
{
    IO_SDADirOut();
    Delayus(5);  
    IO_SDA = 1;                    //拉高数据线
    IO_SCL = 1;                    //拉高时钟线
    Delayus(5);                 //延时
    IO_SDA = 0;                    //产生下降沿
    Delayus(5);                 //延时
    IO_SCL = 0;                    //拉低时钟线
    Delayus(5);  
}

/**************************************
停止信号
**************************************/
void PCF8574_Stop(void)
{   
    IO_SCL = 0;                       
    Delayus(1);  
    IO_SDADirOut();
    Delayus(1);  
    IO_SDA = 0;                    
    Delayus(5);                 //延时
    IO_SDA = 1;                    //产生上升沿
    Delayus(5);                 //延时
}

/**************************************
发送应答信号
入口参数:ack (0:ACK 1:NAK)
**************************************/
void PCF8574_SendACK(char ack)
{   
    IO_SCL = 0;
    Delayus(5);                
    IO_SDADirOut();
    Delayus(5);      
    IO_SDA = ack;                  //写应答信号
    IO_SCL = 1;                    //拉高时钟线
    Delayus(5);                 //延时
    IO_SCL = 0;                    //拉低时钟线
    Delayus(5);                 //延时
}

/**************************************
接收应答信号
**************************************/
char PCF8574_RecvACK()
{
    IO_SCL = 0;
    Delayus(5);
    IO_SDADirOut();
    IO_SDA = 1;
    IO_SDADirIn();
    Delayus(5);          //此处是否有必要使SDA先拉高？！？！
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
向IIC总线发送一个字节数据
**************************************/
BYTE PCF8574_SendByte(BYTE dat)
{
    BYTE i;
    IO_SDADirOut();
    
    for (i=0; i<8; i++)         //8位计数器
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
从IIC总线接收一个字节数据
**************************************/
BYTE PCF8574_RecvByte()
{
    BYTE i;
    BYTE dat = 0;

    IO_SCL = 0;
    IO_SDADirOut();
    IO_SDA = 1;                    //使能内部上拉,准备读取数据,
    Delayus(5);
    IO_SDADirIn();
    for (i=0; i<8; i++)         //8位计数器
    {
        dat <<= 1;
        IO_SCL = 1;                //拉高时钟线
        Delayus(5);             //延时
        dat |= IO_SDA;             //读数据               
        IO_SCL = 0;                //拉低时钟线
        Delayus(5);             //延时
    }
    return dat;
}

//***************************************************

void PCF8574_Write(uchar SlaveAddress,uchar REG_data)
{  
    PCF8574_Start();                  //起始信号
    PCF8574_SendByte(IO_SA_WRITE(SlaveAddress));   //发送设备地址+写信号
    PCF8574_SendByte(REG_data);       //内部寄存器数据，请参考中文pdf
    
    PCF8574_Stop();                   //发送停止信号
    delay(10);
}
//******************************************************
//
//读出PCF8574
//
//******************************************************
BYTE PCF8574_Read_One(uchar SlaveAddress)
{
  BYTE r = 0;
  BYTE ret = 0;
  PCF8574_Start();                                      //起始信号
  ret = PCF8574_SendByte(IO_SA_READ(SlaveAddress));     //发送设备地址+写信号
  if(ret == 0){
    //PCF8574_Stop(); 
    //return 0;
  }                                                    //最后一个数据需要回NOACK
  r = PCF8574_RecvByte();          
  PCF8574_Stop();                                      //停止信号
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

//初始化PCF8574，根据需要请参考pdf进行修改****
//#define IO_SDADirOut()             KKW_IO_DIR_OUTPUT_PREP(0,1)    //P1DIR |= 0x02;      //xxxx1M01
//#define IO_SDADirIn()              KKW_IO_DIR_INPUT_PREP(0,1)     //P1DIR &= ~0x02;
void Init_PCF8574()
{
  
//#define	IO_SCL                     P2_0                //IIC时钟引脚定义
//#define	IO_SDA                     P0_1                //IIC数据引脚定义
//#define       IO_INT                     P0_0                //中断
  
  

  
//    P0DIR |= 0x02;  //调用write函数必须调用
    P0DIR &= 0xFD;  //调用read函数必须调用
    P2DIR |= 0x01;  // read write 
    
    P0SEL &= 0xFD;
    P2SEL &= 0xFE;
}

void IO_SDADirIn(void)
{
       P0DIR &= 0xFD;  //调用read函数必须调用 
}

void IO_SDADirOut(void)
{
       P0DIR |= 0x02;  //调用write函数必须调用 
}