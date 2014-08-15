/*
___________________________________________________________________________
*
* File:                Module.c
*_______________________________________________________________________________*/
#include "ModuleID.h"
#include "tal_constants.h"
#include "mac_api.h"
#define TRUE (1)
#define FALSE (0)
 //__no_init MODULE_CONFIG_INFO                      ModuleCfgInfo                   @ (FLASH_OFFSET); //keil
   MODULE_CONFIG_INFO                      ModuleCfgInfo;
 MODULE_CONFIG_INFO interModuleCfgInfo;
 extern uint8_t serialMode;
void CheckSystemConfig(void)
{
uint32_t *jumpAddress = (uint32_t *)0x8000004; 
  if (ModuleCfgInfo.AtCfgInfo.byModuleCfgFlag != ModuleDefaultConfig.AtCfgInfo.byModuleCfgFlag || *jumpAddress == 0x8000101)
    App_System_Para_Pointer = &ModuleDefaultConfig;
  else
    App_System_Para_Pointer = &ModuleCfgInfo;

  serialMode = App_System_Para_Pointer->AtCfgInfo.byWorkMode;
}

#define	   JOIN_RN_PLUS								0x01
#define DEFAULT_TABLE_EXPIRATION_TIME   1
//#define VENDOR_STR                      "FirmVer=1.00.00.00" 

const MODULE_CONFIG_INFO  ModuleDefaultConfig = 
{
  {  
    MODULE_CONFIG_FLAG,      //unsigned long  byModuleCfgFlag;
   

    
    {0xFF,0xFF,0xFF,0xFF, 0xFF, 0xFF, 0xFF, 0xFF},                              // IEEE_MacAddress;
    0x0002,                  //unsigned short  wNwkNodeID;                      // 188,189    wNwkNodeID
    0x0058,                  //unsigned short  wMacPanID;                       // 190,191    MacLayer Pan Id
    //    0x0001,                  //unsigned short  wMacNodeID;                      // 192,193    MacLayer Node Id
    
    /* COM Port Setup */
    BT_115200,                //unsigned char  CommBaudRate;                     // 101    COM Comm Speed.
    9,                       //unsigned char  CommDataBit;                      // 102    COM Data bits;
    0,                       //unsigned char  CommParity;                       // 103    COM Parity.
    5,                       //unsigned char   CommTimeout;                     // 104    COM Out Time.
    //    USERIO_BUFFER_SIZE,      //unsigned char  CommBufSize;                // 105    COM Buffer Size.
    0,                       //unsigned char  CommFlowCtrol;                    // 106    COM Flow Control
    
    /* RF Setup */
    BT_250K,               //unsigned char  byRfBaudRate;                     // 111    RF BaudRate Choose Register.
    0,                       //unsigned char  byRfSendPower;                    // 112    RF Send Power (mw)
    //    RF_BUFFER_SIZE,          //unsigned char  byRfBufSize;                // 113    RF Accept and Send buffer size. 
    RF_CHANNEL_10,      	     //unsigned char  byRfChannelID;                    // 114    RF Channel Choose Register.
    //    DEFAULT_RF_FREQUENCY,    //unsigned char  byRfFrequency;              // 115    RF Frequency Register. 
    
    /* HPP Net Layer Setup */
    0,                       //unsigned char  byHppNetRepeatSendMacPacketCount;       // 140
    100,                     //unsigned char  byHppNetWaitAckTimeOutCount;            // 141
    1,                       //unsigned char  byHppNetRetrySendRreqForMyselfCount;    // 142
    2,                       //unsigned char  byHppNetRetrySendMacPacketCount;        // 143
    100,                     //unsigned char  byHppNetWaitRrepTimeOutCount;           // 144  //100ms*4
    //    1,                       //unsigned char  byHppNetRetrySendRreqForOthersCount;    // 145
    //    FALSE,                   //unsigned char  byHppNetTryClusterTree;     // 146
    1,                       //unsigned char  byHppNetRepeatMultiBroadCastCount;// 147
    
    /* NETWORK PARAMETER */
    DEV_TYPE_COORDINATOR,//,//,//DEV_TYPE_COORDINATOR,//DEV_TYPE_ENDDEVICE,//,            //unsigned char  byNodeRouterType;                 // 150
    //    0xFF,                          //unsigned char  byNetNodeID;          // 152
    11,                    //unsigned char  byNetClusterEm;       // 153
    6,                     //unsigned char  byNetClusterLm;       // 154
    //    0,                             //unsigned char  byNetChildCount;      // 155
    5,                             //unsigned char  byRm;        // 156
    AODV,        			 //unsigned char  byNetRoutingAlgorithm // 158
    DEFAULT_TABLE_EXPIRATION_TIME, //unsigned char  byTableExpirationTime;      // 159
    97,                            //unsigned char  byTopologyType;             // 160
    //    0xFFFF,                        //unsigned short  wNetClusterParent;   // 161-162
    20,                            //unsigned char  byAodvTTLValue;             // 163
    
    /* CLUSTER-TREE COMMAND */
    //    0xFFFF,                  //unsigned char  wAcceptOrLostChildNodeId;   // 168-169
    //    0,                             //unsigned char  byNetworkState;       // 170
    //    TRUE,                          //unsigned char  byAcceptChildEnable;  // 171
    USER_TRANSPARENT_MODE,      //unsigned char  byWorkMode;                 // 173
    0xFFFF,                        //unsigned short  byTransparentModeDestAddr;  // 174-175
    FALSE,                             //  unsigned char  dobuleAntenna;
	FALSE,							//unsigned char  firstByte_isAddress;
	TRUE,							//unsigned char  isJoin;
	TRUE,							//unsigned char  requestAddress;
    //    0x00,                          //unsigned char  byTMLoopBackFlag      // 176
    
    TRUE,                          //unsigned char  byMacAckFlag                // 180
    TRUE,                          //unsigned char  byNetAckFlag                // 181
    //    0x00,                         //unsigned char  byTimeContrl           // 183
    //    0x00,				//unsigned char  byDangerCheck          // 184        // sean
    //    0x0000,			//unsigned short bySendDangerDest; 	// 185-186    //sean
    //    0x0,                          //unsigned char  byMacBeaconMode;       // 194        //0..15.
    //    0x0,                          //byMacNodeType;                        // 195
    
    FALSE,                          //bySecurityMode;                            // 196
    {132,195,176,140,245,149,52,150,
    27,195,9,81,8,198,253,221}, //unsigned char  IEEE_SecurityKey[16];  // 211-226  helicomm
    
    5,                     //sleep interval sleep interval sleep interval sleep interval sleep interval sleep interval sleep interval sleep interval
        
    
    TRUE,			   //unsigned char  byLEDfordvmflag             // 231
    FALSE,                         //unsigned char  byRemoteFlashflag           // 232
    TRUE,                         //sleep?
    0x02,                          //unsigned char  byBaseCountSleep            // 234
    
                           
    
    FALSE,                         //unsigned char  byUartTagflag               // 236
    
    44,                                                                         //254
    0,				   //unsigned char bySetAdcVref  default is  in	// 242
    0xFF,                          //unisgned char byTMChar                     //243
    
    0x00,                          //unsigned char IOdefaultFunction            //244
    0xFF,                          //unsigned char IOdefatltState               //245
    0x00,			   //unsigned char EnableBootloader		//246	 0xF6
  },
  {
		    /**
		     * 64-bit (IEEE) address of the node.
		     */
		    0xFFFFFFFFFFFFFFFF,

		    /**
		     * Supported channels
		     */
		    TRX_SUPPORTED_CHANNELS,

		#if defined(BEACON_SUPPORT)
		    /**
		     * Holds the time at which last beacon was transmitted or received.
		     */
		    0,
		#endif  /* BEACON_SUPPORT */

		    /**
		     * 16-bit short address of the node.
		     */
		    TAL_SHORT_ADDRESS_DEFAULT,

		    /**
		     * 16-bit PAN ID
		     */
		    TAL_PANID_BC_DEFAULT,

		    /**
		     * Maximum number of symbols in a frame:
		     * = phySHRDuration + ceiling([aMaxPHYPacketSize + 1] x phySymbolsPerOctet)
		     */
		    TAL_MAX_FRAME_DURATION_DEFAULT,

		    /**
		     * CCA Mode
		     */
		    TAL_CCA_MODE_DEFAULT,

		    /**
		     * Current RF channel to be used for all transmissions and receptions.
		     */
		    TAL_CURRENT_CHANNEL_DEFAULT,

		    /**
		     * The maximum number of back-offs the CSMA-CA algorithm will attempt
		     * before declaring a CSMA_CA failure.
		     */
		    TAL_MAX_CSMA_BACKOFFS_DEFAULT,

		    /**
		     * The minimum value of the backoff exponent BE in the CSMA-CA algorithm.
		     */
		    TAL_MINBE_DEFAULT,

		    /**
		     * Indicates if the node is a PAN coordinator or not.
		     */
		    TAL_PAN_COORDINATOR_DEFAULT,

		    /**
		     * Default value of transmit power of transceiver
		     * using IEEE defined format of phyTransmitPower.
		     */
		    TAL_TRANSMIT_POWER_DEFAULT,

		#if defined(BEACON_SUPPORT)
		    /**
		     * Indication of whether battery life extension is enabled or not.
		     */
                    TAL_BATTERY_LIFE_EXTENSION_DEFAULT,
                    TAL_BEACON_ORDER_DEFAULT,
                    TAL_SUPERFRAME_ORDER_DEFAULT,
		#endif  /* BEACON_SUPPORT */

		    /**
		     * Current channel page.
		     */
		    TAL_CURRENT_PAGE_DEFAULT,

		    /**
		     * Duration of the synchronization header (SHR) in symbols for the current PHY.
		     */
		    TAL_SHR_DURATION_DEFAULT,

		    /**
		     * Number of symbols per octet for the current PHY.
		     */
		    TAL_SYMBOLS_PER_OCTET_DEFAULT,

		    /**
		     * The maximum value of the backoff exponent BE in the CSMA-CA algorithm.
		     */
		    TAL_MAXBE_DEFAULT,

		    /**
		     * The maximum number of retries allowed after a transmission failure.
		     */
		    TAL_MAXFRAMERETRIES_DEFAULT,

		#if defined(PROMISCUOUS_MODE)
		    /**
		     * Promiscuous Mode
		     */
		    bool PromiscuousMode;
		#endif

  },
  {
	    /**
	     * Holds the 64 bit address of the coordinator with which the
	     * device is associated.
	     */
	  CLEAR_ADDR_64,


	#if (MAC_INDIRECT_DATA_FFD == 1)
	    /**
	     * Holds the maximum time (in superframe periods) that a indirect transaction
	     * is stored by a PAN coordinator.
	     */
	    macTransactionPersistenceTime_def,
	#endif /* (MAC_INDIRECT_DATA_FFD == 1) */

	    /**
	     * Holds the 16 bit short address of the coordinator with which the device is
	     * associated. A value of 0xfffe indicates that the coordinator is only using
	     * its 64 bit extended address. A value of 0xffff indicates that this
	     * value is unknown.
	     */
	    macCoordShortAddress_def,


	#if ((MAC_INDIRECT_DATA_BASIC == 1) || defined(BEACON_SUPPORT))
	    /**
	     * The maximum number of CAP symbols in a beaconenabled PAN, or symbols in a
	     * nonbeacon-enabled PAN, to wait either for a frame intended as a response to
	     * a data request frame or for a broadcast frame following a beacon with the
	     * Frame Pending subfield set to one.
	     * This attribute, which shall only be set by the next higher layer, is
	     * dependent upon macMinBE, macMaxBE, macMaxCSMABackoffs and the number of
	     * symbols per octet. See 7.4.2 for the formula relating the attributes.
	     * Maximum values:
	     * O-QPSK (2.4 GHz and 900 MHz for Channel page 2): 25766
	     * BPSK (900 MHz for Channel page 0): 26564
	     * Both values are valid for
	     * macMinBE = 8
	     * macMaxBE = 8
	     * macMaxCSMABackoffs = 5
	     *
	     * This PIB attribute is only used if basic indirect data transmission is used
	     * or if beacon enabled network is enabled.
	     */
	    macMaxFrameTotalWaitTime_def,
	#endif  /* ((MAC_INDIRECT_DATA_BASIC == 1) || defined(BEACON_SUPPORT)) */

	    /**
	     * The maximum time, in multiples of aBaseSuperframeDuration, a device shall
	     * wait for a response command frame to be available following a request
	     * command frame.
	     */
	    macResponseWaitTime_def,

	#if (MAC_ASSOCIATION_INDICATION_RESPONSE == 1)
	    /**
	     * Holds the value which states whether a coordinator is currently allowing
	     * association. A value of true indicates that association is permitted.
	     */
	    macAssociationPermit_def,
	#endif /* (MAC_ASSOCIATION_INDICATION_RESPONSE == 1) */

	#if (MAC_START_REQUEST_CONFIRM == 1)
	    /**
	     * Holds the length, in octets, of the beacon payload.
	     */
	    macBeaconPayloadLength_def,
	    /**
	     * Holds the sequence number added to the transmitted beacon frame.
	     */
	    0,
	#endif  /* (MAC_START_REQUEST_CONFIRM == 1) */


	#if (MAC_ASSOCIATION_REQUEST_CONFIRM == 1)
	    /**
	     * Indication of whether the device is associated to the PAN through the PAN
	     * coordinator. A value of TRUE indicates the device has associated through the
	     * PAN coordinator. Otherwise, the value is set to FALSE.
	     */
	    macAssociatedPANCoord_def,
	#endif /* (MAC_ASSOCIATION_REQUEST_CONFIRM == 1) */

	    /**
	     * Holds the value which states whether a device automatically sends a data
	     * request command if its address is listed in the beacon frame. A value of true
	     * indicates that the data request command is automatically sent.
	     */
	    macAutoRequest_def,

	    /**
	     * Holds the value which states the number of backoff periods during which the
	     * receiver is enabled following a beacon in battery life extension mode.
	     * This value is dependent on the currently selected logical channel.
	     */
	    macBattLifeExtPeriods_def,

	    /**
	     * Holds the sequence number of the transmitted data or command frame.
	     */
	    1,

	    /**
	     * Holds the value which states whether the MAC sublayer is to enable its
	     * receiver during idle periods.
	     */
	  macRxOnWhenIdle_def,

	    /**
	     * Indication of whether the MAC sublayer has security enabled. A value of
	     * TRUE indicates that security is enabled, while a value of FALSE indicates
	     * that security is disabled.
	     */
	    macSecurityEnabled_def,

	#ifdef TEST_HARNESS
	    /* Private MAC PIB variables, only valid for testing purposes */

	    /**
	     * Holds the private MAC PIB attribute to generate a frame with an illegale
	     * frame type.
	     */
	    1,

	    /**
	     * Holds the private MAC PIB attribute which suppresses the initiation of a
	     * data request frame after association request.
	     */
	    0,

	    /**
	     * Holds the private MAC PIB attribute to pretend virtual Beacon-enabled PANs.
	     */
	    0,
	#endif /* TEST_HARNESS */



//#ifdef MAC_SECURITY_ZIP
//    /* TODO: Create a specific function for security PIB initialization? */
//    mac_sec_pib.KeyTableEntries = macKeyTableEntries_def;
//    mac_sec_pib.DeviceTableEntries = macDeviceTable_def;
//    mac_sec_pib.SecurityLevelTableEntries = macSecurityLevelTable_def;
//    mac_sec_pib.FrameCounter = macFrameCounter_def;
//#endif  /* MAC_SECURITY_ZIP */
//
//#ifdef TEST_HARNESS
//    mac_pib.privateIllegalFrameType = 1;
//    mac_pib.privateNoDataAfterAssocReq = 0;
//    mac_pib.privateVirtualPANs = 0;
//#endif /* TEST_HARNESS */


  },
  {
          /*** Attributes are initialized by NWK-layer. ***/
/** A network address of parent node. */
macCoordShortAddress_def,
/** The count of total transmissions. */
0,
/** The count of transmissions with failures. */
0,
/** The address of the designated network channel manager function. */
0,
/** This field shall contain the device capability information established at
* network joining time. */
WPAN_CAP_ALLOCADDRESS,
/** The tree depth of the neighbor device. A value of 0x00 indicates that
* the device is the ZigBee coordinator for the network. */
0,
/** A sequence number used to identify outgoing frames */
2,
/** The total delivery time for a broadcast transmission, i.e. the time
* required for a broadcast to be delivered to every device in the network. */
0xFF,
/** The value identifying a snapshot of the network settings with which this
* node is operating with. */
0,
/** Network rejoin permissions, by default end devices and routers can rejoin. */
true,


 /** This field is used by MAC layer. */
{0},
 /** The beacon payload contain the information which enables the NWK layer
  * to provide additional information to new devices that are performing
  * network discovery and allows these new devices to more efficiently select
  * a network and a particular neighbor to join. */
0,0,0,0,0,0,0,0,{0},0,

         /*** Attributes are initialized in Config Server. ***/
/** The type of the device:
*  - 0x00 - zigbee coordinator
*  - 0x01 - zigbee router
*  - 0x02 - zigbee end device */
#if defined _COORDINATOR_
DEVICE_TYPE_COORDINATOR,
#elif defined _ROUTER_
DEVICE_TYPE_ROUTER,
#else
DEV_TYPE_ENDDEVICE,
#endif
/** A value that determines the method used to assign addresses:
*  - 0x00 = use distributed address allocation
*  - 0x01 = reserved
*  - 0x02 = use stochastic address allocation. */
NWK_ADDR_ALLOC_DISTRIBUTED,
/** The current route symmetry setting. 1 means that routes are considered to
* be comprised of symmetric links. Backward and forward routes are created
* during one-route discovery and they are identical. 0 indicates that routes
* are not consider to be comprised of symmetric links. Only the forward route
* is stored during route discovery.*/
true,
/** Determines whether or not the static addressing mode will be used.
* If set to '1' then the device must use a static network address
* otherwise, the stochastic addressing mode will be employed. */
true,
/** The identifier of the ZigBee stack profile in use for this device. */
0,
/** The version of the ZigBee NWK protocol in the device. */
NWKC_PROTOCOL_ID,
/** The depth a device can have. */
6,
/** Route to neighbor directly only if the incoming cost is less than given
* threshold. */
GOOD_CHANNEL,
/** RF channel page */
TAL_CURRENT_PAGE_DEFAULT,
/** Number of RF channel */
TAL_CURRENT_CHANNEL_DEFAULT,
/** The 16-bit address that the device uses to communicate with the PAN. */
0x12,
/** The short (16-bit) pan identifier of a zigbee network. */
0x34,
/** The Extended PAN Identifier for the PAN of which the device is a member.
* The value 0x0000000000000000 means the Extended PAN Identifier is unknown.
**/
0,
/** The number of routers any one device is allowed to have as children. */
6,
/** The number of end devices any one device is allowed to have as children.
**/
0,
6,
#if defined _SECURITY_
/** Pointer to the security information base. */
NWK_SecurityIB_t securityIB;
#endif /* _SECURITY_ */
  }
};

MODULE_CONFIG_INFO  const * App_System_Para_Pointer = &ModuleDefaultConfig;
