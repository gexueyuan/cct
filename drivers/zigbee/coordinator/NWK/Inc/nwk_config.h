#ifndef NWK_CONFIG_H
#define NWK_CONFIG_H
/*
#define SYS_OS
#define SYS_OS_RTT
#define _COORDINATOR_
#define USE_STDPERIPH_DRIVER
#define STM32F4XX
#define STM32F401xx
#define USE_STM3210B_EVAL
 #define NOTRACE
#define SIO_HUB
#define UART0
#define DEBUG=0
#define FFD
#define REDUCED_PARAM_CHECK
#define TAL_TYPE=AT86RF231
#define PAL_TYPE=AT91SAM3S4C
#define PAL_GENERIC_TYPE=SAM3
#define BOARD_TYPE=RZ600_231_SAM3SEK
#define HIGHEST_STACK_LAYER=MAC
#define NODMA_SPI
#define TRX_REG_RAW_VALUE
#define PA
*/

/* === Includes ============================================================= */
#include "app_config.h"
#include "types.h"
#include "bcEndian.h"
/* === Macros =============================================================== */
#define NUMBER_OF_NWK_STACK_TIMERS (5)
#define NWK_FIRST_TIMER_ID          (NUMBER_OF_APP_TIMERS + NUMBER_OF_TOTAL_STACK_TIMERS)
#define T_NWK_PERMIT_JOINING (NWK_FIRST_TIMER_ID)
#define T_NWK_LINK_STATUS (NWK_FIRST_TIMER_ID + 1)
#define T_NWK_ROUTE_DISCOVERY_TIME (NWK_FIRST_TIMER_ID + 2)
#define T_NWK_INIT_ROUTE_REQUEST_INTERVAL (NWK_FIRST_TIMER_ID + 3)
#define T_NWK_RELAY_ROUTE_REQUEST_INTERVAL (NWK_FIRST_TIMER_ID + 4)
#define NWK_LAST_TIMER_ID (T_NWK_RELAY_ROUTE_REQUEST_INTERVAL)
#define BITMAP_UNIT_SIZE 64
#define BITMAPSIZE 2
#define MAX_NEIGHBOR_TABLE_NUMBER    (20)

//#define ATMEL_RF

#ifdef ATMEL_RF
#define  GOOD_CHANNEL                (78)//需要排除的信道,根据标准大于-75dbm的，91-75=16 84-16=68                //78 = -85dbm
#else
#define  GOOD_CHANNEL                (232)//需要排除的信道,根据标准大于-75dbm的，91-75=16 84-16=68                //78 = -85dbm
#endif

#define INVALID_EXT_PAINID           (0xFFFFFFFFFFFFFFFF)
#define INVALID_CHANNEL              (32)
#define JOIN_EXT_PANID               (0x2000000000020002)
#define MAX_NWKADDRESS_MAP_NUM       (20)


#define NWK_LEAVE_SELF_REQ_PAYLOAD_LEN (16+2)
#define NWK_LEAVE_CHILD_REQ_PAYLOAD_LEN (24+2)
#define NWK_LINK_STATUS_REQ_PAYLOAD_LEN (114)
#define NWK_ROUTE_DISCOVERY_REQ_PAYLOAD_LEN (16+7)
#define NWK_ROUTE_DISCOVERY_REQ_PAYLOAD_LEN_EXT (16+15)
#define NWK_ROUTE_DISCOVERY_REPLY_PAYLOAD_LEN (16+9)//24+9
#define NWK_ROUTE_DISCOVERY_REPLY_PAYLOAD_LEN_EXT (24+25)//24=8+8+8,第一个8是网络层强制包头，后面分别是源长地址和目的长地址
#define NWK_ROUTE_RECORD_REQ_PAYLOAD_LEN (16+4)
#define NWK_STATUS_REQ_PAYLOAD_LEN (16+8)



#define NWK_MAX_HOPS (8)

#define NWK_MAX_ROUTE_TABLE_ENTRY (30)
#define NWK_MAX_ROUTE_DISCOVERY_TABLE_ENTRY (30)
#define NWK_ROUTE_DISCOVERY_TIMEOUT (10)
#define NWK_ROUTE_FAIL_TRIES (3)
#define NWK_RETRY_TIMEOUT (220000)
#define NWK_MAX_BROADCASR_TABLE_ENTRY (3)


#define APP_HEAD_LENGTH (8)

#define APP_MAX_PAYLOAD_LENGTH (108)

#define XOR_PASS (3)
#define TIME_OUT (2)
#define UART_ALLOC (4)





#ifdef ATMEL_RF
#define FCS_LENGTH              (2)
#else
#define FCS_LENGTH              (0)
#endif
/* === Types ================================================================ */

typedef enum _app_route_method_t
{
	APP_ROUTE_MESH,
	APP_ROUTE_TREE,
	APP_ROUTE_SOURCE_PC,
	APP_ROUTE_SOURCE_TABLE,
	APP_ROUTE_SOURCE_ASSIGN,
	APP_ROUTE_BROADCAST,
	APP_ROUTE_NONE
} app_route_method_t;


typedef enum _app_mode_t
{
	APP_RESPONSE,
	APP_REQUEST
} app_mode_t;

typedef enum _app_kind_t
{
    APP_CMD,
	APP_DATA
} app_kind_t;

typedef enum _app_cmd_t
{

	APP_CMD_HARD_REBOOT,
	APP_CMD_SLEEP,
	APP_CMD_POWER_DOWN,
	APP_CMD_REPORT_MY_ADDRESS,
	APP_CMD_REPORT_NEIGHBOR,
	APP_CMD_ROUTE_DISCOVERY,
	APP_CMD_ROUTE_RECORD,
	APP_CMD_GET_TAG_TABLE,
	APP_CMD_SEND_TAG_INFO,
	APP_CMD_GET_WEN_SHI_DU,
	APP_CMD_LAST
} app_cmd_t;



BEGIN_PACK
typedef struct PACK _AppFrameControl_t
{
  LITTLE_ENDIAN_OCTET(6, (
    uint8_t tries       : 2,
    uint8_t timeOut     : 2,
    uint8_t sendSuccess : 1,
    uint8_t ackReq      : 1,
    app_kind_t frameType   : 1,
    app_mode_t state       : 1
  ))

  LITTLE_ENDIAN_OCTET(2, (
	app_route_method_t routeType       : 3,
    uint8_t sourceCount     	: 5
  ))
} AppFrameControl_t;
END_PACK



//BEGIN_PACK
//typedef struct PACK _AppFrameHeader_t
//{
//  AppFrameControl_t frameControl;
//  uint8_t AppSequenceNumber;
//  uint16_t DstAddress;
//  app_cmd_t  command;
//  uint8_t  length;
//  uint8_t  AppPayload[1];
//  uint8_t  rssi;
//  uint8_t  XOR;
//} AppFrameHeader_t;
//END_PACK




BEGIN_PACK
typedef struct  _AppFrameHeader_t
{
  AppFrameControl_t frameControl;
  uint8_t AppSequenceNumber : 4;
  uint8_t hops : 4;
  uint8_t  travelTime;
  uint16_t DstAddress;
  uint8_t  length;
 // app_cmd_t  command;
  uint8_t  AppPayload[1];
 // uint8_t  XOR;
} AppFrameHeader_t;
END_PACK

/* === Externals ============================================================ */



/* === Prototypes =========================================================== */
void Delay(uint32_t nCount);
/* === Switch =========================================================== */
//#define NIBFromFlash
//#define dataReqContinue
//#define   noBufferData_andUseTreeRoute


#define UART_DMA_TX
#define ONLY_TRANSPARENT_MODE
//#define REDUCE_CHECK_PARMER4
//#define TEST
//#define WATCHDOG

//#define HIGH_DATA_RATE_SUPPORT




#define REDUCE_CHECK_PARMER//nlde_data_req check
#define REDUCE_CHECK_PARMER1//deviceType
#define REDUCE_CHECK_PARMER2//enddevice
#define REDUCE_CHECK_PARMER3//rssi travelTime hops
#define REDUCE_CHECK_PARMER5//nwkStatusframe ind


#define   NO_JOIN
//#define   SPI_COMMUNICTION


#define VANET
#define BROAD_BASE_GPS_OR_RSSI

//#define   END_DEVICE_SUPPORT
#define _NWK_PASSIVE_ACK_
#define _NWK_ROUTE_CACHE_
//#define UART_IDLE_LINE

#ifdef PA
#define EXT_RF_FRONT_END_CTRL
#define ANTENNA_DIVERSITY (1)
#endif
#endif



//#define SYS_OS


//#define WENSHIDU



//#define  REDUCE_CHECK_PARMER1   打开是没有endevice功能
//#define  REDUCE_CHECK_PARMER2

//#define _ENDDEVICE_
//#define  REDUCE_CHECK_PARMER2  打开是只有endevice功能，，，如果拥有全功能，都不能打开


#ifdef STM32F051
#define __RBIT(x) (x)
#define RTC_GetCounter() 0
#define RTC_WaitForLastTask()
#define RTC_SetPrescaler(x)
#define BKP_DeInit()
#define GPIO_EXTILineConfig(x,y)
#define NVIC_PriorityGroupConfig(x) 
#endif


#ifdef STM32F4XX
#define RTC_WaitForLastTask()
#define RTC_SetPrescaler(x)
#define BKP_DeInit()
#endif


#define VANET_REDUCE_FUNC

#ifdef VANET_REDUCE_FUNC
#define Find_NextHopAddress_InTree(x) x
#define nwkPrepareRouteRecordRelayTx(x)
#define nwkPrepareRouteRecordTx(x)
#define nwkRouteRecordFrameInd(a,b,c)
#define nwkFindRouteDiscoveryEntry(a,b) NULL
#define nwkRouteRequestFrameInd(a,b,c) 
#define nwkRouteReplyFrameInd(a,b,c) 
#define nwkFindRoutingEntry(a,b) NULL
#define nwkLinkStatusInd(a,b,c) 
#define nwkStartLinkStatusTimer()
#define nwkPrepareLinkStatusTx(x)
#define InitNeighborTable()
#define NwkAddress_Map_Add(a,b) 
#define BackgroundCmdProce()
#define IS_HARD_RESET_CMD(a,b) 0
#define RTC_EXTI_INITIAL(x)
#define RTC_SET_ALARM(x) 
#define InitTable()
#define UpdateTagneighborTable(x) 
#define GetTagNeighborTableInfo(x) 0
#define pal_sio_tx(a,b,c) 

#define tal_trx_sleep(x) 
#define tal_trx_wakeup() 
#endif




