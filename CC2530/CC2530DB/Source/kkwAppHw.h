/**************************************************************************************************
  Filename:       SampleAppHw.h
  Revised:        $Date: 2007-10-27 17:22:23 -0700 (Sat, 27 Oct 2007) $
  Revision:       $Revision: 15795 $

  Description:    This file contains the Sample Application definitions.


  Copyright 2007 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED 揂S IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, 
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE, 
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com. 
**************************************************************************************************/

#ifndef PARKINGAPPHW_H
#define PARKINGAPPHW_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
 #include "ZComDef.h"

//#ifdef KKW_HAL_TEMP
//  #include "ds18b20.h"
//#endif
//#ifdef KKW_HAL_HUMI
//  #include "am2001.h"
//#endif
  
/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * MACROS
 */

#ifdef KKW_CC2530_BSP
  #define KKW_GPIO_OUTPUT(IDX,val)    KKW_IO_OUTPUT(GPIO_##IDX##_PORT, GPIO_##IDX##_PIN,val)

  #define KKW_GPIO_DIR_IN(IDX)        KKW_IO_DIR_INPUT(GPIO_##IDX##_PORT, GPIO_##IDX##_PIN)
  #define KKW_GPIO_DIR_OUT(IDX)       KKW_IO_DIR_OUTPUT(GPIO_##IDX##_PORT, GPIO_##IDX##_PIN)
  #define KKW_GPIO_OUTPUT(IDX,val)    KKW_IO_OUTPUT(GPIO_##IDX##_PORT, GPIO_##IDX##_PIN,val)
  #define KKW_GPIO_TRI(IDX)           KKW_IO_INPUT(GPIO_##IDX##_PORT, GPIO_##IDX##_PIN, KKW_IO_TRISTATE)
  #define KKW_GPIO_PULL_UP(IDX)       KKW_IO_INPUT(GPIO_##IDX##_PORT, GPIO_##IDX##_PIN, KKW_IO_PULLUP)
  #define KKW_GPIO_PULL_DN(IDX)       KKW_IO_INPUT(GPIO_##IDX##_PORT, GPIO_##IDX##_PIN, KKW_IO_PULLDOWN)
  #define KKW_GPIO_SET(IDX)           KKW_IO_SET_HIGH(GPIO_##IDX##_PORT, GPIO_##IDX##_PIN)
  #define KKW_GPIO_CLR(IDX)           KKW_IO_SET_LOW(GPIO_##IDX##_PORT, GPIO_##IDX##_PIN)
  #define KKW_GPIO_TOG(IDX)           KKW_IO_TGL(GPIO_##IDX##_PORT, GPIO_##IDX##_PIN)
  #define KKW_GPIO_GET(IDX)           KKW_IO_GET(GPIO_##IDX##_PORT, GPIO_##IDX##_PIN)
  #define KKW_GPIO_HiD_SET()          (PICTL |=  BV(7))  /* PADSC */
  #define KKW_GPIO_HiD_CLR()          (PICTL &= ~BV(7))  /* PADSC */

  // Used as "func" for the macros below
  #define KKW_IO_TRISTATE     1
  #define KKW_IO_PULLUP       2
  #define KKW_IO_PULLDOWN     3

  //-----------------------------------------------------------------------------
  //  Macros for simple configuration of IO pins on TI LPW SoCs
  //-----------------------------------------------------------------------------
  #define KKW_IO_PERIPHERAL(port, pin)   KKW_IO_PERIPHERAL_PREP(port, pin)
  #define KKW_IO_INPUT(port, pin, func)  KKW_IO_INPUT_PREP(port, pin, func)
  #define KKW_IO_OUTPUT(port, pin, val)  KKW_IO_OUTPUT_PREP(port, pin, val)
  #define KKW_IO_SET(port, pin, val)     KKW_IO_SET_PREP(port, pin, val)
  #define KKW_IO_SET_HIGH(port, pin)     KKW_IO_SET_HIGH_PREP(port, pin)
  #define KKW_IO_SET_LOW(port, pin)      KKW_IO_SET_LOW_PREP(port, pin)
  #define KKW_IO_TGL(port, pin)          KKW_IO_TGL_PREP(port, pin)
  #define KKW_IO_GET(port, pin)          KKW_IO_GET_PREP(port, pin)

  #define KKW_IO_DIR_INPUT(port, pin)    KKW_IO_DIR_INPUT_PREP(port, pin)
  #define KKW_IO_DIR_OUTPUT(port, pin)   KKW_IO_DIR_OUTPUT_PREP(port, pin)

  //----------------------------------------------------------------------------------
  //  Macros for internal use (the macros above need a new round in the preprocessor)
  //----------------------------------------------------------------------------------
  #define KKW_IO_PERIPHERAL_PREP(port, pin)   st( P##port##SEL |= BV(pin); )

  #define KKW_IO_INPUT_PREP(port, pin, func)  st( P##port##SEL &= ~BV(pin); \
                                                P##port##DIR &= ~BV(pin); \
                                                P##port##IEN |= BV(pin); \
                                                switch (func) { \
                                                case KKW_IO_PULLUP: \
                                                    P##port##INP &= ~BV(pin); \
                                                    P2INP &= ~BV(port + 5); \
                                                    break; \
                                                case KKW_IO_PULLDOWN: \
                                                    P##port##INP &= ~BV(pin); \
                                                    P2INP |= BV(port + 5); \
                                                    break; \
                                                default: \
                                                    P##port##INP |= BV(pin); \
                                                    break; } \
                                                )

  #define KKW_IO_OUTPUT_PREP(port, pin, val)  st( P##port##SEL &= ~BV(pin); \
                                                P##port##_##pin## = val; \
                                                P##port##DIR |= BV(pin); )

  #define KKW_IO_SET_HIGH_PREP(port, pin)     st( P##port##_##pin## = 1; )
  #define KKW_IO_SET_LOW_PREP(port, pin)      st( P##port##_##pin## = 0; )

  #define KKW_IO_SET_PREP(port, pin, val)     st( P##port##_##pin## = val; )
  #define KKW_IO_TGL_PREP(port, pin)          st( P##port##_##pin## ^= 1; )
  #define KKW_IO_GET_PREP(port, pin)          (P##port## & BV(pin))

  #define KKW_IO_DIR_INPUT_PREP(port, pin)    st( P##port##DIR &= ~BV(pin); )
  #define KKW_IO_DIR_OUTPUT_PREP(port, pin)   st( P##port##DIR |= BV(pin); )

/*
  GPIO input macro
*/
#define KKIO_IN_0         0     //P1.1
#define KKIO_IN_1         1     //P1.1
#define KKIO_IN_2         2     //P1.2
#define KKIO_IN_3         3     //P1.3
#define KKIO_IN_4         4     //P1.4
#define KKIO_IN_5         5     //P1.5
#define KKIO_IN_6         6     //P1.6
#define KKIO_IN_7         7     //P1.7
#define KKIO_IN_MIN       KKIO_IN_0
#define KKIO_IN_MAX       KKIO_IN_7

#define GPIO_KKIO_IN_0_PORT    1
#define GPIO_KKIO_IN_0_PIN     1
#define GPIO_KKIO_IN_1_PORT    1
#define GPIO_KKIO_IN_1_PIN     1
#define GPIO_KKIO_IN_2_PORT    1
#define GPIO_KKIO_IN_2_PIN     2
#define GPIO_KKIO_IN_3_PORT    1
#define GPIO_KKIO_IN_3_PIN     3
#define GPIO_KKIO_IN_4_PORT    1
#define GPIO_KKIO_IN_4_PIN     4
#define GPIO_KKIO_IN_5_PORT    1
#define GPIO_KKIO_IN_5_PIN     5
#define GPIO_KKIO_IN_6_PORT    1
#define GPIO_KKIO_IN_6_PIN     6
#define GPIO_KKIO_IN_7_PORT    1
#define GPIO_KKIO_IN_7_PIN     7
/*
  GPIO output macro
*/
#define KKIO_OUT_0         10      //P0.0
#define KKIO_OUT_1         11      //P0.1
#define KKIO_OUT_2         12      //P0.6
#define KKIO_OUT_3         13      //P0.7
#define KKIO_OUT_4         14      //P2.0    
#define KKIO_OUT_5         15      //P2.1    不调试时才可以用
#define KKIO_OUT_6         16      //P2.2    不调试时才可以用
#define KKIO_OUT_7         17      //P2.3    未引出
#define KKIO_OUT_MIN       KKIO_OUT_0
#define KKIO_OUT_MAX       KKIO_OUT_7

#define GPIO_KKIO_OUT_0_PORT    0
#define GPIO_KKIO_OUT_0_PIN     0
#define GPIO_KKIO_OUT_1_PORT    0
#define GPIO_KKIO_OUT_1_PIN     1
#define GPIO_KKIO_OUT_2_PORT    0
#define GPIO_KKIO_OUT_2_PIN     6
#define GPIO_KKIO_OUT_3_PORT    0
#define GPIO_KKIO_OUT_3_PIN     7
#define GPIO_KKIO_OUT_4_PORT    2
#define GPIO_KKIO_OUT_4_PIN     0
#define GPIO_KKIO_OUT_5_PORT    2
#define GPIO_KKIO_OUT_5_PIN     1
#define GPIO_KKIO_OUT_6_PORT    2
#define GPIO_KKIO_OUT_6_PIN     2
#define GPIO_KKIO_OUT_7_PORT    2
#define GPIO_KKIO_OUT_7_PIN     3

#endif
/*********************************************************************
 * FUNCTIONS
 */

/*
 * Read the Coordinator Jumper
 */
uint8 readCoordinatorJumper( void );

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* PARKINGAPPHW_H */
