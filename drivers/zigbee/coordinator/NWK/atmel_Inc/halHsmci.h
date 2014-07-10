/*****************************************************************************//**
\file   halHsmci.h

\brief  HSMCI interface routines header.

\author
    Atmel Corporation: http://www.atmel.com \n
    Support email: avr@atmel.com

  Copyright (c) 2008-2011, Atmel Corporation. All rights reserved.
  Licensed under Atmel's Limited License Agreement (BitCloudTM).

\internal
  History:
    24/08/11 N. Fomin - Created
**********************************************************************************/
#ifndef _HALHSMCI_H
#define _HALHSMCI_H

/******************************************************************************
                   Includes section
******************************************************************************/
#include <types.h>
#include <AT91SAM3S4.h>

/******************************************************************************
                   Define(s) section
******************************************************************************/
// hsmci mcu module is not initialize
#define BUS_NOINIT          0
// nsmci is ready to transaction
#define BUS_READY           1
// hsmci is busy
#define BUS_BUSY            2

// hsmci setting for mmc commands
#define HSMCI_CMDR_RSPTYP_NORESP    (0x0ul << 6)
#define HSMCI_CMDR_RSPTYP_48_BIT    (0x1ul << 6)
#define HSMCI_CMDR_RSPTYP_136_BIT   (0x2ul << 6)
#define HSMCI_CMDR_RSPTYP_R1B       (0x3ul << 6)
#define HSMCI_CMDR_SPCMD_STD        (0x0ul << 8)
#define HSMCI_CMDR_SPCMD_INIT       (0x1ul << 8)
#define HSMCI_CMDR_TRCMD_NO_DATA    (0x0ul << 16)
#define HSMCI_CMDR_TRCMD_START_DATA (0x1ul << 16)
#define HSMCI_CMDR_TRDIR_WRITE      (0x0ul << 18)
#define HSMCI_CMDR_TRDIR_READ       (0x1ul << 18)
#define HSMCI_CMDR_TRTYP_SINGLE     (0x0ul << 19)

// GO_IDLE_STATE (CMD0) - init command
#define POWER_ON_INIT             (HSMCI_CMDR_TRCMD_NO_DATA | \
                                   HSMCI_CMDR_SPCMD_INIT | \
                                   HSMCI_CMDR_OPDCMD | 0)
// GO_IDLE_STATE (CMD0)
#define GO_IDLE_STATE             (HSMCI_CMDR_TRCMD_NO_DATA | \
                                   HSMCI_CMDR_SPCMD_STD | 0)
// SEND_OP_COND (CMD1)
#define SEND_OP_COND              (HSMCI_CMDR_TRCMD_NO_DATA | \
                                   HSMCI_CMDR_SPCMD_STD | \
                                   HSMCI_CMDR_RSPTYP_48_BIT | \
                                   HSMCI_CMDR_OPDCMD | 1)
// ALL_SEND_CID (CMD2)
#define ALL_SEND_CID              (HSMCI_CMDR_TRCMD_NO_DATA | \
                                   HSMCI_CMDR_SPCMD_STD | \
                                   HSMCI_CMDR_RSPTYP_136_BIT | \
                                   HSMCI_CMDR_OPDCMD | 2)
// SEND_RELATIVE_ADDR (CMD3)
#define SEND_RELATIVE_ADDR        (HSMCI_CMDR_TRCMD_NO_DATA | \
                                   HSMCI_CMDR_SPCMD_STD | \
                                   HSMCI_CMDR_RSPTYP_48_BIT | \
                                   HSMCI_CMDR_OPDCMD | \
                                   HSMCI_CMDR_MAXLAT | 3)
// SWITCH (CMD6)
#define SWITCH                    (HSMCI_CMDR_TRCMD_NO_DATA | \
                                   HSMCI_CMDR_SPCMD_STD | \
                                   HSMCI_CMDR_RSPTYP_R1B | \
                                   HSMCI_CMDR_MAXLAT | 6)
// SELECT_CARD (CMD7)
#define SELECT_CARD               (HSMCI_CMDR_TRCMD_NO_DATA | \
                                   HSMCI_CMDR_SPCMD_STD | \
                                   HSMCI_CMDR_RSPTYP_R1B | \
                                   HSMCI_CMDR_MAXLAT | 7)
// SELECT_CARD (CMD7)
#define DESELECT_CARD             (HSMCI_CMDR_TRCMD_NO_DATA | \
                                   HSMCI_CMDR_SPCMD_STD | \
                                   HSMCI_CMDR_RSPTYP_NORESP | \
                                   HSMCI_CMDR_MAXLAT | 7)
// SEND_EXT_CSD (CMD8)
#define SEND_EXT_CSD              (HSMCI_CMDR_TRCMD_START_DATA | \
                                   HSMCI_CMDR_TRTYP_SINGLE | \
                                   HSMCI_CMDR_TRDIR | \
                                   HSMCI_CMDR_SPCMD_STD | \
                                   HSMCI_CMDR_RSPTYP_48_BIT | \
                                   HSMCI_CMDR_MAXLAT | 8)
// SEND_CSD (CMD9)
#define SEND_CSD                  (HSMCI_CMDR_TRCMD_NO_DATA | \
                                   HSMCI_CMDR_SPCMD_STD | \
                                   HSMCI_CMDR_RSPTYP_136_BIT | \
                                   HSMCI_CMDR_MAXLAT | 9)
// SEND_CID (CMD10)
#define SEND_CID                  (HSMCI_CMDR_TRCMD_NO_DATA | \
                                   HSMCI_CMDR_SPCMD_STD | \
                                   HSMCI_CMDR_RSPTYP_136_BIT | \
                                   HSMCI_CMDR_MAXLAT | 10)
// SEND_STATUS (CMD13)
#define SEND_STATUS               (HSMCI_CMDR_TRCMD_NO_DATA | \
                                   HSMCI_CMDR_SPCMD_STD | \
                                   HSMCI_CMDR_RSPTYP_48_BIT | \
                                   HSMCI_CMDR_MAXLAT | 13)
// SET_BLOCKLEN (CMD16)
#define SET_BLOCKLEN              (HSMCI_CMDR_TRCMD_NO_DATA | \
                                   HSMCI_CMDR_SPCMD_STD | \
                                   HSMCI_CMDR_RSPTYP_48_BIT | \
                                   HSMCI_CMDR_MAXLAT | 16)
// READ_SINGLE_BLOCK (CMD17)
#define READ_SINGLE_BLOCK         (HSMCI_CMDR_SPCMD_STD | \
                                   HSMCI_CMDR_RSPTYP_48_BIT | \
                                   HSMCI_CMDR_TRCMD_START_DATA | \
                                   HSMCI_CMDR_TRTYP_SINGLE | \
                                   HSMCI_CMDR_TRDIR_READ | \
                                   HSMCI_CMDR_MAXLAT | 17)
// WRITE_SINGLE_BLOCK (CMD24)
#define WRITE_BLOCK               (HSMCI_CMDR_SPCMD_STD | \
                                   HSMCI_CMDR_RSPTYP_48_BIT | \
                                   HSMCI_CMDR_TRCMD_START_DATA | \
                                   HSMCI_CMDR_TRTYP_SINGLE | \
                                   HSMCI_CMDR_TRDIR_WRITE | \
                                   HSMCI_CMDR_MAXLAT | 24)
// HSMCI bus width
#define HSMCI_BUS_WIDTH_1 0x00
#define HSMCI_BUS_WIDTH_4 0x80
#define HSMCI_BUS_WIDTH_8 0xC0

/******************************************************************************
                   Types section
******************************************************************************/
typedef uint32_t HsmciClockRate_t;
typedef uint32_t HsmciBusWidth_t;

typedef enum
{
  HSMCI_READ,
  HSMCI_WRITE
} HsmciDataFlowDirection_t;

typedef struct
{
  /* hsmci command argument */
  uint32_t  argument;
  /* hsmci command index and parameters */
  uint32_t  command;
  /* hsmci command response length */
  uint8_t   responseLength;
  /* hsmci command response buffer from application; it should have size of four */
  uint32_t  *responseBuffer;
} HAL_HsmciCommandDescriptor_t;

typedef struct
{
  /* hsmci read/write communication block size */
  uint16_t                 blockSize;
  /* direction of data transfer */
  HsmciDataFlowDirection_t direction;
  /* length in bytes of data buffer; it should size of 512 */
  uint32_t                 length;
  /* data buffer */
  uint8_t                 *buffer;
  /* pointer to hsmci command descriptor */
  HAL_HsmciCommandDescriptor_t *commandDescr;
} HAL_HsmciDataTransferDescriptor_t;

/******************************************************************************
                   Prototypes section
******************************************************************************/
/******************************************************************************
\brief Starts hsmci command transaction.
\param[in]
  descriptor - pointer to hsmci command descriptor;
  callback - pointer to function to notify upper layer about end of command
             transaction.
******************************************************************************/
void halWriteHsmciCommand(HAL_HsmciCommandDescriptor_t *descriptor, void (*callback)(void));

/******************************************************************************
\brief Starts hsmci data read/write transaction.
\param[in]
  descriptor - pointer to hsmci data transfer descriptor;
  callback - pointer to function to notify upper layer about end of read/write
             transaction.
******************************************************************************/
void halHsmciData(HAL_HsmciDataTransferDescriptor_t *descriptor, void (*callback)(void));

/******************************************************************************
\brief Checks if hsmci bus is free.
\return
   false - hsmci is busy;
   true - hsmci is ready.
******************************************************************************/
bool halHsmciIsBusFree(void);

/**************************************************************************//**
\brief Sets hsmci bus speed.
\param[in]
  clockRate - clock rate of hsmci bus, in Hz
\return
  noting
******************************************************************************/
void halHsmciSetClockRate(uint32_t clockRate);

/**************************************************************************//**
\brief Sets hsmci bus width.
\param[in]
  busWidth - width of hsmci bus.
\return
  noting
******************************************************************************/
void halHsmciSetBusWidth(uint32_t busWidth);

/**************************************************************************//**
\brief Sets hsmci speed mode.
\param[in]
  hsMode - spped mode of hsmci bus (true - high speed, false - normal).
\return
  noting
******************************************************************************/
void halHsmciSetHighSpeedMode(bool hsMode);

#endif /*_HALHSMCI_H*/
// eof halHsmci.h