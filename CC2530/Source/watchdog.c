#include <ioCC2530.h>
#include "OSAL.h"
#include "ZGlobals.h"
#include "OSAL_Nv.h"
#include "AF.h"
#include "aps_groups.h"
#include "ZDApp.h"

#include "watchdog.h"

void Init_Watchdog(void) 
{ 
  //WDCTL = 0x00;
  //ʱ����һ�룬���Ź�ģʽ
  //WDCTL |= 0x08;	//�������Ź�
}

void SET_MAIN_CLOCK(int source)
{
  if(source){
    CLKCONCMD |= 0x40;/*RC*/
    while(!(CLKCONSTA&0X40));/*����*/
  }else{
    CLKCONCMD&=~0x47;/*����*/
    while((CLKCONSTA&0X40));/*����*/
  }
}

void FeetDog(void)
{
  WDCTL = 0xA0 |(WDCTL&0x0F);
  WDCTL = 0x50 |(WDCTL&0x0F);
}
