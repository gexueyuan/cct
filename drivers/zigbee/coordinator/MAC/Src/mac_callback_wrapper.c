/**
 * @file mac_callback_wrapper.c
 *
 * @brief Wrapper code for MAC callback functions.
 *
 * $Id: mac_callback_wrapper.c 26610 2011-05-11 08:47:45Z sschneid $
 *
 * @author    Atmel Corporation: http://www.atmel.com
 * @author    Support email: avr@atmel.com
 */
/*
 * Copyright (c) 2009, Atmel Corporation All rights reserved.
 *
 * Licensed under Atmel's Limited License Agreement --> EULA.txt
 */

/* === Includes ============================================================ */

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "pal.h"
#include "return_val.h"
#include "bmm.h"
#include "qmm.h"
#include "tal.h"
#include "ieee_const.h"
#include "mac_msg_const.h"
#include "mac_api.h"
#include "mac_msg_types.h"
#include "mac_data_structures.h"
#include "stack_config.h"
#include "mac_internal.h"
#include "mac.h"
#include "mac_build_config.h"

#include "nwk_api.h"
#include "nwk_msg_types.h"
#include "nwk.h"
#include "nwkStateMachine.h"
/* === Macros ============================================================== */


/* === Globals ============================================================= */


/* === Prototypes ========================================================== */


/* === Implementation ====================================================== */

/**
 * @brief Wrapper function for messages of type mcps_data_ind_t
 *
 * This function is a callback for mcps data indication
 *
 * @param m Pointer to message structure
 */
void mcps_data_ind(uint8_t *m)
{
    mcps_data_ind_t *pmsg;
    wpan_addr_spec_t src_addr;
    wpan_addr_spec_t dst_addr;

    /* Get the buffer body from buffer header */
    pmsg = (mcps_data_ind_t *)BMM_BUFFER_POINTER(((buffer_t *)m));

    /* Source address spec */
    src_addr.AddrMode = pmsg->SrcAddrMode;
    src_addr.PANId = pmsg->SrcPANId;
    ADDR_COPY_DST_SRC_64(src_addr.Addr.long_address, pmsg->SrcAddr);

    /* Destination address spec */
    dst_addr.AddrMode = pmsg->DstAddrMode;
    dst_addr.PANId = pmsg->DstPANId;
    ADDR_COPY_DST_SRC_64(dst_addr.Addr.long_address, pmsg->DstAddr);

    /* Callback function */
#ifdef MAC_SECURITY_ZIP
    usr_mcps_data_ind(&src_addr,
                      &dst_addr,
                      pmsg->msduLength,
                      pmsg->msdu,
                      pmsg->mpduLinkQuality,
                      pmsg->RSSI,
                      pmsg->DSN,
    #ifdef ENABLE_TSTAMP
                      pmsg->Timestamp,
    #endif  /* ENABLE_TSTAMP */
                      pmsg->SecurityLevel,
                      pmsg->KeyIdMode,
                      pmsg->KeyIndex);
#else   /* No MAC_SECURITY */
#ifdef BEACON_SUPPORT
    usr_mcps_data_ind(&src_addr,
                      &dst_addr,
                      pmsg->msduLength,
                      pmsg->msdu,
                      pmsg->mpduLinkQuality,
    #ifdef ENABLE_TSTAMP
                      pmsg->DSN,
                      pmsg->Timestamp);
    #else
                      pmsg->DSN);
    #endif  /* ENABLE_TSTAMP */
    bmm_buffer_free((buffer_t *)m);
#else    
    usr_mcps_data_ind(&src_addr,
                      &dst_addr,
                      pmsg->msduLength,
                      pmsg->msdu,
                      pmsg->mpduLinkQuality,
                      pmsg->RSSI,
    #ifdef ENABLE_TSTAMP
                      pmsg->DSN,
                      pmsg->Timestamp);
    #else
                      pmsg->DSN,
                     (buffer_t *)m);
    #endif  /* ENABLE_TSTAMP */
#endif    
#endif  /* MAC_SECURITY */

    /* Free the buffer */
   // bmm_buffer_free((buffer_t *)m);
}



/**
 * @brief Wrapper function for messages of type mcps_data_conf_t
 *
 * This function is a callback for mcps data confirm.
 *
 * @param m Pointer to message structure
 */
void mcps_data_conf(uint8_t *m)
{
    mcps_data_conf_t *pmsg;

    /* Get the buffer body from buffer header */
    pmsg = (mcps_data_conf_t *)BMM_BUFFER_POINTER(((buffer_t *)m));

#ifdef ENABLE_TSTAMP
    usr_mcps_data_conf(pmsg->msduHandle, pmsg->status, pmsg->Timestamp);
#else
    usr_mcps_data_conf(pmsg->msduHandle, pmsg->status);
#endif  /* ENABLE_TSTAMP */

    /* Free the buffer */
    bmm_buffer_free((buffer_t *)m);
}



#if ((MAC_PURGE_REQUEST_CONFIRM == 1) && (MAC_INDIRECT_DATA_BASIC == 1))
/**
 * @brief Wrapper function for messages of type mcps_purge_conf_t
 *
 * This function is a callback for mcps purge confirm.
 *
 * @param m Pointer to message structure
 */
void mcps_purge_conf(uint8_t *m)
{
    mcps_purge_conf_t *pmsg;

    /* Get the buffer body from buffer header */
    pmsg = (mcps_purge_conf_t *)BMM_BUFFER_POINTER(((buffer_t *)m));

    usr_mcps_purge_conf(pmsg->msduHandle, pmsg->status);

    /* Free the buffer */
    bmm_buffer_free((buffer_t *)m);
}
#endif /* ((MAC_PURGE_REQUEST_CONFIRM == 1) && (MAC_INDIRECT_DATA_BASIC == 1)) */



#if (MAC_ASSOCIATION_REQUEST_CONFIRM == 1)
/**
 * @brief Wrapper function for messages of type mlme_associate_conf_t
 *
 * This function is a callback for mlme associate confirm.
 *
 * @param m Pointer to message structure
 */
void mlme_associate_conf(uint8_t *m)
{
    mlme_associate_conf_t *pmsg;

    /* Get the buffer body from buffer header */
    pmsg = (mlme_associate_conf_t *)BMM_BUFFER_POINTER(((buffer_t *)m));

    usr_mlme_associate_conf(pmsg->AssocShortAddress, pmsg->status);

    /* Free the buffer */
    bmm_buffer_free((buffer_t *)m);
}
#endif /* (MAC_ASSOCIATION_REQUEST_CONFIRM == 1) */



#if (MAC_ASSOCIATION_INDICATION_RESPONSE == 1)
/**
 * @brief Wrapper function for messages of type mlme_associate_ind_t
 *
 * This function is a callback for mlme associate indication.
 *
 * @param m Pointer to message structure
 */
void mlme_associate_ind(uint8_t *m)
{
    mlme_associate_ind_t *pmsg;
    
    //if (NwkState == NWK_MODULE_NONE )  
      //  NwkState = NWK_MODULE_ID_JOIN_IND;  
    NwkJoinIndState  = NWK_JOIN_IND_ASSOC_INDICATE_STATE;
    
    /* Get the buffer body from buffer header */
    pmsg = (mlme_associate_ind_t *)BMM_BUFFER_POINTER(((buffer_t *)m));
#if defined(BEACON_SUPPORT) && defined(_COORDINATOR_) 
    usr_mlme_associate_ind(pmsg->DeviceAddress, pmsg->CapabilityInformation);
#else    
    usr_mlme_associate_ind(pmsg->DeviceAddress, pmsg->CapabilityInformation, 
                           pmsg->mpduLinkQuality, pmsg->mpduRSSI);
#endif
    /* Free the buffer */
    bmm_buffer_free((buffer_t *)m);
}
#endif /* (MAC_ASSOCIATION_INDICATION_RESPONSE == 1) */



#if (MAC_BEACON_NOTIFY_INDICATION == 1)
/**
 * @brief Wrapper function for messages of type mlme_beacon_notify_ind_t
 *
 * This function is a callback for mlme beacon notify indication.
 *
 * @param m Pointer to message structure
 */
void mlme_beacon_notify_ind(uint8_t *m)
{
    mlme_beacon_notify_ind_t *pmsg;

    /* Get the buffer body from buffer header */
    pmsg = (mlme_beacon_notify_ind_t *)BMM_BUFFER_POINTER(((buffer_t *)m));

    /* Callback function */
    usr_mlme_beacon_notify_ind(pmsg->BSN,               // BSN
                               &(pmsg->PANDescriptor),  // PANDescriptor
                               pmsg->PendAddrSpec,      // PendAddrSpec
                               pmsg->AddrList,          // AddrList
                               pmsg->sduLength,         // sduLength
                               pmsg->sdu);              // sdu

    /* Free the buffer */
    bmm_buffer_free((buffer_t *)m);
}
#endif /* (MAC_BEACON_NOTIFY_INDICATION == 1) */



#if (MAC_COMM_STATUS_INDICATION == 1)
/**
 * @brief Wrapper function for messages of type mlme_comm_status_ind_t
 *
 * This function is a callback for mlme comm status indication.
 *
 * @param m Pointer to message structure
 */
void mlme_comm_status_ind(uint8_t *m)
{
    mlme_comm_status_ind_t *pmsg;
    wpan_addr_spec_t src_addr;
    wpan_addr_spec_t dst_addr;

    /* Get the buffer body from buffer header */
    pmsg = (mlme_comm_status_ind_t *)BMM_BUFFER_POINTER(((buffer_t *)m));

    /* Source address spec */
    src_addr.PANId = pmsg->PANId;
    src_addr.AddrMode = pmsg->SrcAddrMode;
    ADDR_COPY_DST_SRC_64(src_addr.Addr.long_address, pmsg->SrcAddr);

    /* Destintion address spec */
    dst_addr.PANId = pmsg->PANId;
    dst_addr.AddrMode = pmsg->DstAddrMode;
    ADDR_COPY_DST_SRC_64(dst_addr.Addr.long_address, pmsg->DstAddr);

    /* Callback function */
#if defined(BEACON_SUPPORT) && defined(_COORDINATOR_) 
    usr_mlme_comm_status_ind(&src_addr,
                             &dst_addr,
                             pmsg->status);
#else    
    usr_mlme_comm_status_ind(&src_addr,
                             &dst_addr,
                             pmsg->status,
                             pmsg->frame_msgtype);
#endif
    /* Free the buffer */
    bmm_buffer_free((buffer_t *)m);
}
#endif /* (MAC_COMM_STATUS_INDICATION == 1) */



#if (MAC_DISASSOCIATION_BASIC_SUPPORT == 1)
/**
 * @brief Wrapper function for messages of type mlme_disassociate_conf_t
 *
 * This function is a callback for mlme disassociate confirm.
 *
 * @param m Pointer to message structure
 */
void mlme_disassociate_conf(uint8_t *m)
{
    mlme_disassociate_conf_t *pmsg;
    wpan_addr_spec_t device_addr;

    /* Get the buffer body from buffer header */
    pmsg = (mlme_disassociate_conf_t *)BMM_BUFFER_POINTER(((buffer_t *)m));

    /* Device address spec */
    device_addr.AddrMode = pmsg->DeviceAddrMode;
    device_addr.PANId = pmsg->DevicePANId;
    ADDR_COPY_DST_SRC_64(device_addr.Addr.long_address, pmsg->DeviceAddress);

    usr_mlme_disassociate_conf(pmsg->status,
                               &device_addr);

    /* Free the buffer */
    bmm_buffer_free((buffer_t *)m);
}
#endif  /* (MAC_DISASSOCIATION_BASIC_SUPPORT == 1) */



#if (MAC_DISASSOCIATION_BASIC_SUPPORT == 1)
/**
 * @brief Wrapper function for messages of type mlme_disassociate_ind_t
 *
 * This function is a callback for mlme disassociate indication.
 *
 * @param m Pointer to message structure
 */
void mlme_disassociate_ind(uint8_t *m)
{
    mlme_disassociate_ind_t *pmsg;

    /* Get the buffer body from buffer header */
    pmsg = (mlme_disassociate_ind_t *)BMM_BUFFER_POINTER(((buffer_t *)m));

    usr_mlme_disassociate_ind(pmsg->DeviceAddress,
                              pmsg->DisassociateReason);

    /* Free the buffer */
    bmm_buffer_free((buffer_t *)m);
}
#endif /* (MAC_DISASSOCIATION_BASIC_SUPPORT == 1) */



#if (MAC_GET_SUPPORT == 1)
/**
 * @brief Wrapper function for messages of type mlme_get_conf_t
 *
 * This function is a callback for mlme get confirm.
 *
 * @param m Pointer to message structure
 */
void mlme_get_conf(uint8_t *m)
{
    mlme_get_conf_t *pmsg;

    /* Get the buffer body from buffer header */
    pmsg = (mlme_get_conf_t *)BMM_BUFFER_POINTER(((buffer_t *)m));

    /* Callback function */
    usr_mlme_get_conf(pmsg->status,
                      pmsg->PIBAttribute,
#ifdef MAC_SECURITY_ZIP
                      pmsg->PIBAttributeIndex,
#endif  /* MAC_SECURITY_ZIP */
                      &pmsg->PIBAttributeValue);

    /* Free the buffer */
    bmm_buffer_free((buffer_t *)m);
}
#endif  /* (MAC_GET_SUPPORT == 1) */



#if (MAC_ORPHAN_INDICATION_RESPONSE == 1)
/**
 * @brief Wrapper function for messages of type mlme_orphan_ind_t
 *
 * This function is a callback for mlme orphan indication.
 *
 * @param m Pointer to message structure
 */
void mlme_orphan_ind(uint8_t *m)
{
    mlme_orphan_ind_t *pmsg;

    /* Get the buffer body from buffer header */
    pmsg = (mlme_orphan_ind_t *)BMM_BUFFER_POINTER(((buffer_t *)m));

    /* Callback function */
    usr_mlme_orphan_ind(pmsg->OrphanAddress);

    /* Free the buffer */
    bmm_buffer_free((buffer_t *)m);

}
#endif /* (MAC_ORPHAN_INDICATION_RESPONSE == 1) */



#if (MAC_INDIRECT_DATA_BASIC == 1)
/**
 * @brief Wrapper function for messages of type mlme_poll_conf_t
 *
 * This function is a callback for mlme poll confirm.
 *
 * @param m Pointer to message structure
 */
void mlme_poll_conf(uint8_t *m)
{
    mlme_poll_conf_t *pmsg;

    /* Get the buffer body from buffer header */
    pmsg = (mlme_poll_conf_t *)BMM_BUFFER_POINTER(((buffer_t *)m));

    usr_mlme_poll_conf(pmsg->status);

    /* Free the buffer */
    bmm_buffer_free((buffer_t *)m);
}
#endif /* (MAC_INDIRECT_DATA_BASIC == 1) */



/**
 * @brief Wrapper function for messages of type mlme_reset_conf_t
 *
 * This function is a callback for mlme reset confirm.
 *
 * @param m Pointer to message structure
 */
void mlme_reset_conf(uint8_t *m)
{
    mlme_reset_conf_t *pmsg;

    /* Get the buffer body from buffer header */
    pmsg = (mlme_reset_conf_t *)BMM_BUFFER_POINTER(((buffer_t *)m));

    usr_mlme_reset_conf(pmsg->status);

    /* Free the buffer */
    bmm_buffer_free((buffer_t *)m);
}



#if (MAC_RX_ENABLE_SUPPORT == 1)
/**
 * @brief Wrapper function for messages of type mlme_rx_enable_conf_t
 *
 * This function is a callback for mlme rx enable confirm.
 *
 * @param m Pointer to message structure
 */
void mlme_rx_enable_conf(uint8_t *m)
{
    mlme_rx_enable_conf_t *pmsg;

    /* Get the buffer body from buffer header */
    pmsg = (mlme_rx_enable_conf_t *)BMM_BUFFER_POINTER(((buffer_t *)m));

    usr_mlme_rx_enable_conf(pmsg->status);

    /* Free the buffer */
    bmm_buffer_free((buffer_t *)m);
}
#endif /* (MAC_RX_ENABLE_SUPPORT == 1) */



#if (MAC_SCAN_SUPPORT == 1)
/**
 * @brief Wrapper function for messages of type mlme_scan_conf_t
 *
 * This function is a callback for mlme scan confirm.
 *
 * @param m Pointer to message structure
 */
void mlme_scan_conf(uint8_t *m)
{
    mlme_scan_conf_t *pmsg;

    /* Get the buffer body from buffer header */
    pmsg = (mlme_scan_conf_t *)BMM_BUFFER_POINTER(((buffer_t *)m));

    if (NwkState == NWK_MODULE_ID_DISCOVERY)
    {
      NWK_NetworkDiscoveryConf_t  *ndc; 
      ndc  = (NWK_NetworkDiscoveryConf_t *)BMM_BUFFER_POINTER(
                        ((buffer_t *)gNwk_conf_buf_ptr));
      ndc->status = (NWK_Status_t)pmsg->status;
      qmm_queue_append(&nwk_aps_q, (buffer_t *)gNwk_conf_buf_ptr);      
    } 
    else
    /* Callback */
    usr_mlme_scan_conf(pmsg->status,
                       pmsg->ScanType,
                       pmsg->ChannelPage,
                       pmsg->UnscannedChannels,
                       pmsg->ResultListSize,
                       &pmsg->scan_result_list);

    /* Free the buffer */
    bmm_buffer_free((buffer_t *)m);
}
#endif /* (MAC_SCAN_SUPPORT == 1) */



/**
 * @brief Wrapper function for messages of type mlme_set_conf_t
 *
 * This function is a callback for mlme set confirm.
 *
 * @param m Pointer to message structure
 */
void mlme_set_conf(uint8_t *m)
{
    //mlme_set_conf_t *pmsg;

    /* Get the buffer body from buffer header */
  //  pmsg = (mlme_set_conf_t *)BMM_BUFFER_POINTER(((buffer_t *)m));

#ifdef MAC_SECURITY_ZIP
    usr_mlme_set_conf(pmsg->status, pmsg->PIBAttribute, pmsg->PIBAttributeIndex);
#else
   // usr_mlme_set_conf(pmsg->status, pmsg->PIBAttribute);
#endif  /* MAC_SECURITY_ZIP */

    /* Free the buffer */
    bmm_buffer_free((buffer_t *)m);
}



#if (MAC_START_REQUEST_CONFIRM == 1)
/**
 * @brief Wrapper function for messages of type mlme_start_conf_t
 *
 * This function is a callback for mlme start confirm.
 *
 * @param m Pointer to message structure
 */
void mlme_start_conf(uint8_t *m)
{
    mlme_start_conf_t *pmsg;

    /* Get the buffer body from buffer header */
    pmsg = (mlme_start_conf_t *)BMM_BUFFER_POINTER(((buffer_t *)m));

    usr_mlme_start_conf(pmsg->status);

    /* Free the buffer */
    bmm_buffer_free((buffer_t *)m);
}
#endif /* (MAC_START_REQUEST_CONFIRM == 1) */



#if (MAC_SYNC_LOSS_INDICATION == 1)
/**
 * @brief Wrapper function for messages of type mlme_sync_loss_ind_t
 *
 * This function is a callback for mlme sync loss indication.
 *
 * @param m Pointer to message structure
 */
void mlme_sync_loss_ind(uint8_t *m)
{
    mlme_sync_loss_ind_t *pmsg;

    /* Get the buffer body from buffer header */
    pmsg = (mlme_sync_loss_ind_t *)BMM_BUFFER_POINTER(((buffer_t *)m));

    usr_mlme_sync_loss_ind(pmsg->LossReason,
                           pmsg->PANId,
                           pmsg->LogicalChannel,
                           pmsg->ChannelPage);

    /* Uses static buffer for sync loss indication and it is not freed */
}
#endif /* (MAC_SYNC_LOSS_INDICATION == 1) */

/* EOF */

