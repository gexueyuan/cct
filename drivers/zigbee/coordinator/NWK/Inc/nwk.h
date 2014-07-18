#ifndef _NWK_H
#define _NWK_H

/* === Includes ============================================================= */

#include <stdbool.h>
#include "tal.h"
#include "qmm.h"
#include "mac_build_config.h"
#ifdef ENABLE_RTB
#include "rtb.h"
#endif  /* ENABLE_RTB */
#include "nlmePermitJoining.h"
#include "nwkFormation.h"
#include "nlmeSetGet.h"
#include "nwkDiscovery.h"


#include "nlmejoin.h"
#include "nwkJoinInd.h"
#include "nwkJoinReq.h"

#include "nwkNeighbor.h"

#include "nwkStartRouter.h"
#include "nwkLeave.h"
#include "nwkIB.h"
#include "nwkLinkStatus.h"
#include "nwkRouteDiscovery.h"
#include "nwkRoutingTable.h"
#include "nwkRouteDiscoveryTable.h"
#include "nwkRouteRecord.h"
#include "nwkStatusReq.h"
#include "ModuleID.h"
#include "nwkPassiveAck.h"
#include "nwkRouteCache.h"
/* === Macros =============================================================== */


/* === Types ================================================================ */
typedef struct _nwkSquence
{
	uint16_t srcAddress;
	uint8_t sequence;
} nwkSquence_t;


/* === Externals ============================================================ */
extern queue_t aps_nwk_q;
extern queue_t nwk_aps_q;
#ifdef  dataReqContinue
extern queue_t wait_nwk_ack_q;
#endif
extern NwkState_t NwkState;
extern NwkPermitJoiningState_t NwkPermitJoiningState;
extern NwkFormationState_t     NwkFormationState;
extern NwkDiscoveryState_t     NwkNetworkDiscoveryState;
extern NwkJoinReqState_t       NwkJoinReqState;
extern NwkJoinIndObjState_t    NwkJoinIndState;
extern NwkStartRouterState_t   NwkStartRouterState;
extern NwkLeaveState_t         NwkLeaveReqState;
extern NwkLinkStatusState_t    NwkLinkStatusState;
extern NwkRouteDiscoveryState_t NwkRouteDiscoveryState;
extern NwkRouteRecordState_t    NwkRouteRecordState;
extern NwkStateOfStatusSender_t NwkStatusState;


extern uint8_t         gNwk_RouteDiscovery_sequence;
extern NwkAddrMap_t gNwkAddrMap[];
extern uint32_t     gNwkAddrMap_BitMap;
extern NIB_t gNwk_nib;


//extern ExtPanId_t gExtendedPANId;
//extern bool       gCoordRealignment;


extern uint8_t *gNwk_conf_buf_ptr;
extern NwkNeighbor_t *gNeighborParent_p;
extern uint16_t gJoinCoordShortAddress;

//extern frame_msgtype_t         gMlmeCommStatus_FrameType;
//extern uint64_t                gMlmeCommStatus_DestAddr;

extern NWK_NibAttr_t Nwk_NibAttrValue;

extern NwkParseHeader_t  gNwkParse;
extern NwkFrameHeader_t *gNwkFrameHeader_p;
extern frame_info_t *nwk_frame_ptr;
extern uint8_t gResponseData[];
extern uint8_t gResIndex;

extern NwkRoutingTableEntry_t gNwkRoutingTableEntry[];
extern NwkRoutingTable_t      gNwkRoutingTable;

extern NwkRouteCacheRecord_t  gNwkRouteCacheRecordEntry[];
extern NwkRouteCache_t        gNwkRouteCacheTable;

extern NwkRouteDiscoveryEntry_t gNwkRouteDiscoveryTableEntry[];
extern NwkRouteDiscoveryTable_t gNwkRouteDiscoveryTable;
extern uint8_t gRouteRelayRetries;
extern uint8_t gMyAddress_report[];
extern uint8_t gSelectChannel;
extern bool    gPowerUpFlag;
extern nwkSquence_t gNwkSequence[];
extern 	uint32_t currentTime;
extern uint8_t serialMode;

extern NwkPassiveAckEntry_t gNwkPassiveAckEntry[];
extern NwkPassiveAck_t gNwkPassiveAckTable;

extern uint64_t broadDelayBitmap;
extern uint8_t minBroadDelayIndex;
extern uint8_t broadDelayCount;
extern NwkBroadDelay_t nwkBroadDelay[];
/* === Prototypes =========================================================== */


bool nwk_task(void);

void nwk_init(void);
bool NwkAddress_Map_Del_Ext(uint64_t ExtAddress);

void nlme_reset_conf(uint8_t *m);
void nlme_set_conf(uint8_t *m);//cloudm
void nlme_PermitJoining_conf(uint8_t *m);
void nlme_formation_conf(uint8_t *m);
void nlme_NetworkDiscovery_conf(uint8_t *m);
void nlme_join_conf(uint8_t *m);
void nlme_join_ind(uint8_t *m);
void nlme_StartRouter_conf(uint8_t *m);
void nlme_leave_conf(uint8_t *m);
void nlme_leave_ind(uint8_t *m);
void nlde_data_ind(uint8_t *m);
void nlde_data_conf(uint8_t *m);
void nlme_RouteDiscovery_conf(uint8_t *m);









#ifdef NWK_SECURITY_ZIP
NWK_Status_t nlme_set(NWK_NibId_t attribute, uint8_t attribute_index, pib_value_t *attribute_value, bool set_trx_to_sleep);
#else
NWK_Status_t nlme_set(NWK_NibId_t attribute, pib_value_t *attribute_value, bool set_trx_to_sleep);
#endif  /* MAC_SECURITY_ZIP */
void nlme_set_request(uint8_t *msg);
void nlme_reset_request(uint8_t *msg);
void nlme_formation_request(uint8_t *m);
void nlme_PermitJoining_request(uint8_t *m);
void nlme_NetworkDiscovery_request(uint8_t *m);
void nlme_join_request(uint8_t *m);
void nlme_StartRouter_request(uint8_t *m);
void nlme_leave_request(uint8_t *m);
void nwk_gen_nlme_leave_conf(buffer_t *buf_ptr,
                             uint8_t status,
                             uint64_t leave_long_addr);
void nlme_leave_ind_forward(uint8_t *m);
void nwk_gen_nlme_leave_ind_forward(buffer_t *buf_ptr, bool rejoin);
void nlde_data_request (uint8_t *m);
void nwk_gen_nlde_data_conf(buffer_t *buf_ptr,
                            uint8_t status,
                            void *handle,
                            uint32_t timestamp,
                            uint8_t hops);
void nlme_RouteDiscovery_request(uint8_t *m);
NWK_PRIVATE void nwkPrepareRouteReplyTx(uint8_t *m);
NWK_PRIVATE void nwkPrepareRouteReplyRelayTx(uint8_t *m);
NWK_PRIVATE void nwkPrepareRouteRequestRelayTx(uint8_t *m);
#ifndef VANET_REDUCE_FUNC
NWK_PRIVATE bool nwkRouteRequestFrameInd(const uint8_t *const payload,
										 const NwkFrameHeader_t *const header,
										 const NwkParseHeader_t *const parse);
NWK_PRIVATE bool nwkRouteReplyFrameInd(const uint8_t *const payload,
									   const NwkFrameHeader_t *const header,
									   const NwkParseHeader_t *const parse);
NWK_PRIVATE NwkRoutingTableEntry_t* nwkFindRoutingEntry(const ShortAddr_t dstAddr,
														const bool isGroupId);
NWK_PRIVATE void nwkPrepareRouteRecordTx(uint8_t *m);//(frame_info_t *const outPkt)

NWK_PRIVATE void nwkPrepareRouteRecordRelayTx(uint8_t *m);
#endif
void app_t_DataRequest_cb(frame_info_t *transmit_frame);
void nwk_t_RouteDiscoveryExpired_cb(void *callback_parameter);
void nwk_t_InitRouteRequest_cb(void *callback_parameter);
void nwk_t_RelayRouteRequest_cb(void *callback_parameter);


#ifndef VANET_REDUCE_FUNC
NWK_PRIVATE bool nwkRouteRecordFrameInd(const uint8_t *const payload,
										 NwkFrameHeader_t * header,
										const NwkParseHeader_t *const parse);
NWK_PRIVATE bool nwkStatusFrameInd(const uint8_t *const payload,
									const NwkFrameHeader_t *const header,
									const NwkParseHeader_t *const parse);
#endif									
uint8_t XORSUM(uint8_t *data, uint8_t length);
uint8_t testRssi(uint8_t rssi);

bool bubble_sort_broad(uint8_t index);

#ifdef _NWK_PASSIVE_ACK_
#ifndef VANET
void bc_data_cb(frame_info_t *transmit_frame);
#else
void bc_data_cb_vanet(frame_info_t *transmit_frame);
#endif
void bc_delay_cb(frame_info_t *transmit_frame);
#endif

uint8_t nwkFindBroadDelay(const ShortAddr_t srcAddr, const NwkSequenceNumber_t seqNum);
int8_t find_black_list(uint16_t srcAddr);
int8_t find_white_list(uint16_t srcAddr);
extern bool blackListEnable;
extern bool whiteListEnable;









#ifndef VANET_REDUCE_FUNC
uint16_t Find_NextHopAddress_InTree(uint16_t NwkDstAddress);
#endif
#endif
