#ifndef NWK_API_H
#define NWK_API_H

/* === Includes ============================================================= */

#include "pal.h"
#include "mac.h"
#include "return_val.h"
#include "qmm.h"
#include "mac_build_config.h"
#include "nwkCommon.h"








#include "nlmeSetGet.h"
#include "nlmeJoin.h"
#include "nwkRouteDiscoveryTable.h"
/* === Macros =============================================================== */


/* === Types ================================================================ */



/* === Externals ============================================================ */

/* === Prototypes =========================================================== */


bool wpan_nlme_reset_req(bool WarmStart);

bool wpan_nlme_set_req(NWK_NibId_t PIBAttribute,  void *PIBAttributeValue);

bool wpan_nlme_formation_req(ChannelsMask_t scanChannels, ScanDuration_t scanDuration, bool coordRealignment);

bool wpan_nlme_PermitJoining_req(uint8_t permitDuration);

bool wpan_nlme_NetworkDiscovery_req(ScanDuration_t scanDuration, ChannelsMask_t scanChannels,  bool clearNeighborTable);

bool wpan_nlme_join_req(ExtPanId_t extendedPANId, ScanDuration_t scanDuration, ChannelsMask_t scanChannels, NWK_RejoinNetwork_t rejoinNetwork);

bool wpan_nlme_StartRouter_req(bool coordRealignment);
void nwk_gen_nlme_join_conf(buffer_t *buf_ptr,
                                 uint8_t status,
                                 uint16_t join_short_addr,
                                 ExtPanId_t extendedPANId,
                                 Channel_t activeChannel);
bool wpan_nlme_leave_req(ExtAddr_t deviceAddress, bool removeChildren, bool rejoin);
bool wpan_nlme_leave_ind_forward(ExtAddr_t deviceAddress, bool removeChildren, bool rejoin);
bool wpan_nlme_NwkLinkStatus_CommandFrame_req(ShortAddr_t dstAddr);
bool wpan_nlde_data_req(NWK_DstAddrMode_t dstAddrMode, 
                        ShortAddr_t dstAddr, 
                        uint8_t nsduLength,
                        uint8_t *nsdu,
                        uint8_t *nsduHandle,
                        uint8_t radius,
                        uint8_t nonMemberRadius,
                        bool    discoverRoute,
                        uint8_t    secutityEnable,
                        app_route_method_t routeType);
bool wpan_nlme_RouteDiscovery_req(NWK_DstAddrMode_t dstAddrMode, ShortAddr_t dstAddr, NwkRadius_t radius, bool noRouteCache, buffer_t *msg, buffer_t *uartBuffer);
bool wpan_nlme_RouteDiscovery_reply(NwkRouteDiscoveryEntry_t *const entry, uint8_t index);
bool wpan_nlme_RouteDiscoveryRelay_req(uint8_t payloadSize, uint8_t *payload, uint8_t index, uint8_t rssi);
bool wpan_nlme_RouteRelpy_Relay_req(frame_info_t *const outPkt, NwkRouteDiscoveryEntry_t *const entry);
bool wpan_nlme_RouteRecord_CommandFrame_req(ShortAddr_t dstAddr, buffer_t *uartBuffer);
bool wpan_nlme_RouteRecord_Relay_CommandFrame_req(uint8_t payloadSize, uint8_t *payload, uint8_t *relayPayload, uint8_t realySize, uint16_t MacDestAddress);
bool wpan_nlme_NwkStatus_CommandFrame_req(NWK_StatusIndErrorCodes_t statusCode,
											ShortAddr_t targetAddr,
											ShortAddr_t dstAddr,
											ShortAddr_t nextHopAddr,
											ShortAddr_t prevHopAddr);


void usr_nlme_reset_conf(uint8_t status, uint8_t WarmStart);

void usr_nlme_set_conf(uint8_t status,  NWK_NibId_t PIBAttribute);

void usr_nlme_formation_conf(uint8_t status);

void usr_nlme_PermitJoining_conf(uint8_t status);

void usr_nlme_NetworkDiscovery_conf(uint8_t status, uint8_t networkCount, void *networkDescriptors);

void usr_nlme_join_conf(uint8_t status, uint16_t join_short_addr, ExtPanId_t extendedPANId, Channel_t activeChannel);

void usr_nlme_join_ind(uint16_t networkAddress, uint64_t extendedAddress,MAC_CapabilityInf_t capabilityInformation,
                       NWK_RejoinNetwork_t rejoinNetwork, bool secureJoin, bool isRejoin);

void usr_nlme_StartRouter_conf(uint8_t status);

void  usr_nlme_leave_conf(NWK_Status_t status,  uint64_t leave_long_addr);

void  usr_nlme_leave_ind(uint64_t deviceAddress, bool rejoin, bool removeChildren);

void usr_nlde_data_ind(NWK_DstAddrMode_t dstAddrMode, 
                       ShortAddr_t dstAddr,
                       ShortAddr_t srcAddr,
                       ShortAddr_t prevHopAddr,
                       NwkLength_t nsduLength,                      
                       uint8_t *nsdu,
                       Lqi_t linkQuality,                      
                       Rssi_t rssi,
                       uint32_t rxTime,                      
                       bool securityUse,
                       uint8_t radius
                       );

void usr_nlde_data_conf(uint8_t status, void *nsduHandle, 
                        uint32_t txTime, NwkRadius_t hops);

void  usr_nlme_RouteDiscovery_conf(NWK_Status_t status, NWK_StatusIndErrorCodes_t networkStatusCode, ShortAddr_t nextHop);


#endif
