/**************************************************************************//**
\file macBanNode.h

  \brief Interface of the ban node functionality.

  \author
    Atmel Corporation: http://www.atmel.com \n
    Support email: avr@atmel.com

  Copyright (c) 2008-2011, Atmel Corporation. All rights reserved.
  Licensed under Atmel's Limited License Agreement (BitCloudTM).

  \internal
   History:
    29/03/10 M. Gekk - Created.
    18/05/10 A. Luzhetsky - link cost control is added
 ******************************************************************************/
#ifndef _MAC_BAN_NODE_H
#define _MAC_BAN_NODE_H

/******************************************************************************
                               Includes section
 ******************************************************************************/
#include <macAddr.h>

/******************************************************************************
                              Definitions section
 ******************************************************************************/
#define LINK_COST_VALUE_NODE_BANNED 0xFF

/******************************************************************************
                                Types section
 ******************************************************************************/
/** Type of node ban table entry. */
typedef struct _MAC_BanTableEntry_t
{
  /* Short address of the banned node. */
  ShortAddr_t  shortAddr;
  /* IEEE address of the banned node. */
  ExtAddr_t  extAddr;
  /* Link cost to be used for all messages from the node. */
  /* Valid range to correct the link cost (not to ban the node): 1 - 7 */
  /* Use link cost value LINK_COST_VALUE_NODE_BANNED to ban the node */
  uint8_t cost;
} MAC_BanTableEntry_t;

/** Type of the ban table size. */
typedef uint8_t MAC_BanTableSize_t;
/** Type of a ban entry index. */
typedef MAC_BanTableSize_t MAC_BanTableIndex_t;

/** Type of the ban node table. */
typedef struct _MAC_BanTable_t
{
  /* Current size of the ban node table. */
  MAC_BanTableSize_t  size;
  /* Pointer to the first entry of the ban table. */
  MAC_BanTableEntry_t  *entry;
  /* Index of the oldest banned node. */
  MAC_BanTableIndex_t  oldBannedNode;
  /* Ban all nodes by default (true) or not (false). After reset this attribute
   * is equal to false. */
  bool  banAll;
} MAC_BanTable_t;

/******************************************************************************
                              Prototypes section
 ******************************************************************************/
/**************************************************************************//**
  \brief Ban any frame from the node or correct link cost for all the frames 
    from the node.

  \param shortAddr - short address of the node affected.
    If the short address of banned node is unknown then pass MAC_NO_SHORT_ADDR.
  \param extAddr - extended address of the node affetced.
    If the extended address of banned node is unknown then pass 0.
  \param cost - link cost value for all the frames received from the node.
    If node shall be baned - use link cost value LINK_COST_VALUE_NODE_BANNED.

  \return None.
 ******************************************************************************/
void MAC_BanNode(const ShortAddr_t shortAddr, const ExtAddr_t extAddr, 
  const uint8_t cost);

/**************************************************************************//**
  \brief Set global flag banAll

  \param banAll - ban all nodes by default or not.

  \return None.
 ******************************************************************************/
void MAC_BanAllNodes(const bool banAll);
 
/**************************************************************************//**
  \brief Is given short address banned or not.

  \param shortAddr - short address of the banned node. 
  \param lqi - frame lqi pointer to be corrected if it is needed.

  \return 'true' if the node is banned otherwise return 'false'.
 ******************************************************************************/
bool MAC_IsBannedShortAddr(const ShortAddr_t shortAddr, uint8_t *const lqi);

/**************************************************************************//**
  \brief Is given extended addresss banned or not.

  \param extAddr - extended address of the banned node. 
  \param lqi - frame lqi pointer to be corrected if it is needed.

  \return 'true' if the node is banned otherwise return 'false'.
 ******************************************************************************/
bool MAC_IsBannedExtAddr(const ExtAddr_t extAddr, uint8_t *const lqi);

/******************************************************************************
  \brief Reload from the config server and reset the ban table.
 ******************************************************************************/
void MAC_ResetBanTable(void);

#endif /* _MAC_BAN_NODE_H */ 
/** eof macBanNode.h */ 

