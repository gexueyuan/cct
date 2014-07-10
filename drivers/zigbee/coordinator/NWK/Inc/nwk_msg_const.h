
#ifndef NWK_MSG_CONST_H
#define NWK_MSG_CONST_H

/* === Includes ============================================================= */


/* === Macros =============================================================== */

/**
 * This type contains the service primitives of the PHY-, MAC- and Network-layer
 * as well the command-interface primitives
 */
enum nwk_msg_code
{
    NLME_NWK_LINK_STATUS_REQUEST          = (0x00),
//
//    MLME_ASSOCIATE_REQUEST              = (0x01),
//    MLME_ASSOCIATE_RESPONSE             = (0x02),
//
    NLDE_DATA_REQUEST                     = (0x03),
    NLME_LEAVE_IND_FORWARD                = (0x04),

    NLME_LEAVE_REQUEST                    = (0x05),
    NLME_SET_REQUEST                      = (0x06),
    NLME_JOIN_REQUEST                     = (0x07),
//    MLME_GET_REQUEST                    = (0x08),
    NLME_RESET_REQUEST                    = (0x09),
    NLME_PERMIT_JOINING_REQUEST           = (0x0A),
    NLME_NETWORK_DISCOVERY_REQUEST        = (0x0B),
    NLME_FORMATION_REQUEST                = (0x0D),
    NLME_JOIN_INDICATION                  = (0x0E),
    NLME_START_ROUTER_REQUEST             = (0x0F),
//
    NLME_ROUTE_DISCOVERY_REQUEST          = (0x10),
    NLME_ROUTE_DISCOVERY_REPLY            = (0x11),
    NLME_ROUTE_DISCOVERY_RELAY_REQUEST    = (0x12),
//
    NLME_ROUTE_REPLY_RELAY_REQUEST        = (0x13),
    NLME_ROUTE_DISCOVERY_CONFIRM          = (0x14),
      NLDE_DATA_CONFIRM                   = (0x15),
      NLDE_DATA_INDICATION                = (0x16),
      NLME_LEAVE_INDICATION               = (0x17),
      NLME_LEAVE_CONFIRM                  = (0x1A),
      NLME_NETWORK_DISCOVERY_CONFIRM      = (0x1B),
      NLME_START_ROUTER_CONFIRM           = (0x1C),
      NLME_JOIN_CONFIRM                   = (0x1D),
//    MLME_GET_CONFIRM                    = (0x1E),
      NLME_SET_CONFIRM                    = (0x1F),
      NLME_RESET_CONFIRM                  = (0x20),
      NLME_PERMIT_JOINING_CONFIRM         = (0x21),
      NLME_FORMATION_CONFIRM              = (0x22),
      NLME_NWK_ROUTE_RECORD_REQUEST       = (0x23),
      NLME_ROUTE_RECORD_RELAY_REQUEST     = (0x24),
      NLME_NETWORK_STATUS_REQUEST         = (0x25)
} SHORTENUM;

#define LAST_NWK_MESSAGE NLME_NETWORK_STATUS_REQUEST
/**
 * Bump this when extending the list!
 */
#define NWK_LAST_MESSAGE                    NLME_NETWORK_STATUS_REQUEST

#endif
