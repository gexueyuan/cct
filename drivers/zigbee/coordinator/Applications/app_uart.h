/*
 * app_uart.h
 *
 *  Created on: 2012-8-22
 *      Author: Administrator
 */








#ifndef APP_UART_H_
#define APP_UART_H_

#ifdef STM32F051
#include "stm32f0xx.h"
#elif defined STM32F10X_MD
#include "stm32f10x.h"
#elif defined STM32F4XX
#include "stm32f4xx.h"
#endif

#define MAX_TAG_TABLE                                100
#define TAG_DATA_BUFFER_LENGH					     200

typedef struct tag_SLEEP_TIME_DESC
{
  unsigned short  byBank;
  unsigned char   byTime;
}SLEEP_TIME_DESC;
typedef struct tag_BACKGROUND_CMD_PROCE
{
  unsigned char byCmd;
  unsigned char byLocal;
  union
  {
    unsigned long    dwData;
    unsigned char    byFlashPageID;
    unsigned char    Sourwakeup;
//    ADC_WAKEUP_DESC  AdcWakeup;
    SLEEP_TIME_DESC  SleepTime;
    //APP_SENSOR_ITEM  AppSensorItem;
  }Data;

}BACKGROUND_CMD_PROCE;
#if ((defined __ICCARM__) || (defined __GNUARM__))
#pragma pack(1)
#endif /* __ICCARM__, __GNUARM__*/
typedef struct tag_TAG_NEIGHBOE_TABLE_ITEM
{
  unsigned short byMacAddr;
//  unsigned char  LQI;
  signed char    Rssi;
  unsigned short AdcValue;
}TAG_NEIGHBOE_TABLE_ITEM;
#if ((defined __ICCARM__) || (defined __GNUARM__))
#pragma pack()
#endif






extern BACKGROUND_CMD_PROCE  BackgroundCmdProce;

extern bool uartEnableRequire;
#ifndef VANET_REDUCE_FUNC
void RTC_EXTI_INITIAL(FunctionalState interrupt_en_or_dis);
void RTC_SET_ALARM(uint32_t sec);
void RTC_AWU_SET(void);
#endif











#define TRANSPARENT_IO_BUF_SIZE       107

#define USER_RETURN_OK          0x00
#define USER_RETURN_ERROR       0xFF

#define NO_ERROR_RETURN         0x00

#define ERROR_XOR_ERROR         1
#define ERROR_SEND_FAIL         2
#define ERROR_COMMAND           3

#define ERROR_SEND_TIMEOUT          4
#define ERROR_USER_CMD          5

#define ERROR_CMD_PARAM         6
#define ERROR_DEST_ERROR        7

#define ERROR_FLASH_PAGE        8

#define ERROR_NET_BUSY          9
#define ERROR_CMD_NO_SUPPORT    10

#define ERROR_USER_HEADER       11

#define ERROR_AT_REG_OR_VALUE   12




#define ERROR_NO_ROUTING_CAPACITY       12

#define ERROR_BUFFER_FULL       13

#define ERROR_PAYLOAD_LENGTH_0       14
#define ERROR_ROUTE_DISCOVERY_FAIL       15

#define HPP_NET_CMD_NULL                                    0xFF














uint8_t IsPlusPlusPlus(uint8_t *uart_Data, uint8_t length);
uint8_t IsThreeEqualMarks(uint8_t *uart_Data, uint8_t length);
uint8_t IsSubSubSub(uint8_t *uart_Data, uint8_t length);
#ifndef VANET_REDUCE_FUNC
uint8_t  IS_HARD_RESET_CMD(uint8_t *uart_Data, uint8_t length);
void 	InitTable(void);
unsigned char UpdateTagneighborTable(TAG_NEIGHBOE_TABLE_ITEM *TagneighborTableItem);
unsigned char GetTagNeighborTableInfo(unsigned char  * lpRetData);
#endif
#endif /* APP_UART_H_ */
