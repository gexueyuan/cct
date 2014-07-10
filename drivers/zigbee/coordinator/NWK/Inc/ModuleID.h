/* ____________________________________________________________________________
#  Copyright (c) 2004
# 	Helicomm Inc.  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are prohibited without the written consents of Helicomm, Inc.
#
#  THIS SOFTWARE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR IMPLIED
#  WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
* _____________________________________________________________________________
*
* File:                ModuleId.h
*_______________________________________________________________________________*/
/*
* CVS Revision History:
*   
*    $Id: ModuleId.h,v 1.2 2005/03/31 15:22:39 yangfan Exp $
* 
*/


#ifndef ModuleIdH 
#define ModuleIdH 
#include "mac_internal.h"
#include "nwkIB.h"
#define FLASH_REGION _Pragma("location=\".near_func.textrw\"")

#define HAIER_APP 

#define MAC_ADDRESS_SIZE                8

#define BT_115200                       0
#define BT_57600                        1
#define BT_38400                        2
#define BT_19200                        3
#define BT_9600                         4
#define BT_4800                         5
#define BT_2400                         6
#define BT_1200							7 // sean
#define BT_28800                        8
#define BT_600							9 
#define BT_14400						10

#define BT_250K         	            0
#define BT_500K     	                1
#define BT_1M   	                    2
#define BT_2M	                        3

#define RF_CHANNEL_0               0
#define RF_CHANNEL_1               1
#define RF_CHANNEL_2               2
#define RF_CHANNEL_3               3
#define RF_CHANNEL_4               4
#define RF_CHANNEL_5               5
#define RF_CHANNEL_6               6
#define RF_CHANNEL_7               7
#define RF_CHANNEL_8               8
#define RF_CHANNEL_9               9
#define RF_CHANNEL_10              10
#define RF_CHANNEL_11              11
#define RF_CHANNEL_12              12
#define RF_CHANNEL_13              13
#define RF_CHANNEL_14              14
#define RF_CHANNEL_15              15

#ifdef STM32F051 //M4problem
#define FLASH_OFFSET					0x0800FC00
#elif defined STM32F10X_MD
#define FLASH_OFFSET					0x0801FC00
#elif defined STM32F4XX
#define FLASH_OFFSET					0x0801FC00
#endif
#define PAGE_OFFSET                     127

#define DEFAULT_RF_BAUDRATE             BT_250000
#define DEFAULT_COM_BAUDRATE            BT_38400
#define RF_BUFFER_SIZE                  116
#define USERIO_BUFFER_SIZE       		128  //(128 + 15)
#define DEFAULT_RF_FREQUENCY       		3
#define	JOIN_RFD                        0x03
#define UNDEFINED 						0
#define AODV                            0x00

#define USER_MIBEE_FRAME_MODE             		 1
#define USER_AT_COMMAND_MODE                 0
#define USER_TRANSPARENT_MODE                2






#define DEFAULT_RF_SEND_PWOER           0x00

#define MODULE_CONFIG_FLAG              0x20002000L


#define MODULE_DEVICE_TYPE              0x0B
#define FW_MAIN_VER_FLAG                01
#define FW_SECOND_VER_FLAG              01
#define FW_APP_VER_FLAG                 01
#define FW_BRANCH_VER_FLAG	        03
#define VENDOR_STR                      "FirmVer=1.01.01.03"
BEGIN_PACK
typedef struct tag_AT_CFG_REG_DESC
{
  unsigned long  byModuleCfgFlag;


  
  unsigned char  IEEE_MacAddr[MAC_ADDRESS_SIZE];               // 200-207 IEEE_MacAddress
  unsigned short wNwkNodeID;                                   // 188-189    NWK Address
  unsigned short wMacPanID;                                    // 190-191    MacLayer Pan Id
//  unsigned short wMacNodeID;                                   // 192-193    MacLayer Node Id
  
  /* COM Port Setup */
  unsigned char  CommBaudRate;                                 // 101    COM Comm Speed.
  unsigned char  CommDataBit;                                  // 102    COM Data bits;
  unsigned char  CommParity;                                   // 103    COM Parity.
  unsigned char  CommTimeout;                                  // 104    COM Out Time.
//  unsigned char  CommBufSize;                                  // 105    COM Buffer Size.
  unsigned char  CommFlowCtrol;                                // 106    COM Flow Control
  
  /* RF Setup */
  unsigned char  byRfBaudRate;                                 // 111    RF BaudRate Choose Register.    
  unsigned char  byRfSendPower;                                // 112    RF Send Power (mw)
//  unsigned char  byRfBufSize;                                  // 113    RF Accept and Send buffer size. 
  unsigned char  byRfChannelID;                                // 114    RF Channel Choose Register. 
  //unsigned char  byRfFrequency;                                // 115    RF Frequency Register.
  
  /* HPP Net Layer Setup */
  unsigned char  byHppNetRepeatSendMacPacketCount;             // 140    HPP net
  unsigned char  byHppNetWaitAckTimeOutCount;                  // 141
  unsigned char  byHppNetRetrySendRreqForMyselfCount;          // 142
  unsigned char  byHppNetRetrySendMacPacketCount;              // 143
  unsigned char  byHppNetWaitRrepTimeOutCount;                 // 144
//  unsigned char  byHppNetRetrySendRreqForOthersCount;          // 145
//  unsigned char  byHppNetTryClusterTree;                       // 146
  unsigned char  byHppNetRepeatMultiBroadCastCount;            // 147
  
  /* NETWORK PARAMETER */
  unsigned char  byNodeRouterType;                             // 150    hpp router.
//  unsigned char  byNetNodeID;                                  // 151-152
  unsigned char  byNetClusterEm;                               // 153
  unsigned char  byNetClusterLm;                               // 154
//  unsigned char  byNetChildCount;                              // 155
  unsigned char  byNetClusterRm;                                // 156
  unsigned char  byNetRoutingAlgorithm;                        // 158
  unsigned char  byTableExpirationTime;                        // 159
  unsigned char  byTopologyType;                               // 160
//  unsigned short wNetClusterParent;                            // 161-162
  unsigned char  byAodvTTLValue;                               // 163
  
  /* CLUSTER-TREE COMMAND */
//  unsigned short wAcceptOrLostChildNodeId;                     // 168-169
//  unsigned char  byNetworkState;                               // 170
//  unsigned char  byAcceptChildEnable;                          // 171
  unsigned char  byWorkMode;                                   // 173
  unsigned short wTransparentModeDestAddr;                     // 174-175
  unsigned char  dobuleAntenna;
  unsigned char  firstByte_isAddress;
  unsigned char  isJoin;
  unsigned char  requestAddress;
//  unsigned char  byTMLoopBackFlag;                             // 176
  
  /* MAC FLAG */
  unsigned char  byMacAckFlag;                                 //180
  unsigned char  byNetAckFlag;                                 //181
//  unsigned char  byTimeContrl;                                 //183
//  unsigned char  byDangerCheck;				       //184
//  unsigned short bySendDangerDest; 			       //185-186
//  unsigned char  byMacBeaconMode;                              // 194
//  unsigned char  byMacNodeType;                                // 195
  
  unsigned char  bySecurityMode;                               // 196
  unsigned char  IEEE_SecurityKey[16];                         // 211-227
  
  /*APP*/
  unsigned char  byAppLocalizerTime;	// 230
  
  
  unsigned char  byLEDfordvmflag;                              // 231
  unsigned char  byRemoteFlashflag;                            // 232
  
  unsigned char  bySleepModeflag;		// 233  
  
  unsigned char  byBaseCountSleep;                             // 234
  unsigned char  byUartTagflag;                                // 236
  
  unsigned char  byDefaultModifyFlag;                          // 254
  
  unsigned char  bySetAdcVref;				       // 242
  unsigned char  byTMChar;                                     // 243
    
  unsigned char  byIOdefaultFunction;                          // 244
  unsigned char  byIOdefaultState;                             // 245
  unsigned char  byEnableBootloader;                           // 246
}AT_CFG_REG;//共占用 个字节



typedef struct tag_MODULE_CONFIG_INFO
{
   AT_CFG_REG             AtCfgInfo;
   tal_pib_t			  talInfo;
   mac_pib_t              macInfo;
   NIB_t                  nwkInfo;
   //unsigned char          reserver[128 - sizeof(AT_CFG_REG)];
}MODULE_CONFIG_INFO;
END_PACK

extern  const MODULE_CONFIG_INFO  ModuleDefaultConfig;
extern  MODULE_CONFIG_INFO                      ModuleCfgInfo;
//extern __no_init MODULE_CONFIG_INFO                      ModuleCfgInfo                   @ (FLASH_OFFSET); keil
extern  MODULE_CONFIG_INFO const  * App_System_Para_Pointer;
extern  MODULE_CONFIG_INFO interModuleCfgInfo;
#define GET_LOCAL_NWK_ADDR()               		App_System_Para_Pointer->AtCfgInfo.wNwkNodeID  //0x0050
#define GET_LOCAL_PAN_ID()					    App_System_Para_Pointer->AtCfgInfo.wMacPanID
#define GET_LOCAL_MAC_ADDRESS()					App_System_Para_Pointer->AtCfgInfo.wNwkNodeID  //0x0050
#define GET_LOCAL_RFCHANNEL()					App_System_Para_Pointer->AtCfgInfo.byRfChannelID  //12
#define GET_LOCAL_TAG_INTERVAL()				App_System_Para_Pointer->AtCfgInfo.byAppLocalizerTime
#define GET_LOCAL_POWER()						App_System_Para_Pointer->AtCfgInfo.byRfSendPower					







//App
#define GET_USER_FRAME_MODE()                          serialMode//App_System_Para_Pointer->AtCfgInfo.byWorkMode //(ModuleCfgInfo.AtCfgInfo.byWorkMode) // USER_TRANSPARENT_MODE  //
#define AES_SECURITY                                   (ModuleCfgInfo.AtCfgInfo.bySecurityMode)
#define GET_LED_FOR_DVM_FLAG()                         (ModuleCfgInfo.AtCfgInfo.byLEDfordvmflag)
#define GET_TRANSPARENT_MODE_DEST_ADDR()               (ModuleCfgInfo.AtCfgInfo.wTransparentModeDestAddr)
#define GET_REMOTE_FLASH_FLAG()                        (ModuleCfgInfo.AtCfgInfo.byRemoteFlashflag)
#define GET_APP_UART_TAG_FLAG()                        (ModuleCfgInfo.AtCfgInfo.byUartTagflag)
#define GET_APP_LOCALIZER()                            (ModuleCfgInfo.AtCfgInfo.byAppLocalizerTime)
#define GET_SLEEP_MODE_FLAG()                          (ModuleCfgInfo.AtCfgInfo.bySleepModeflag)















void CheckSystemConfig(void);

#endif
