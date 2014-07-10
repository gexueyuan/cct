/**************************************************************************//**
\file   halUsb.h

\brief  Declarations of usb hardware-dependent module.

\author
    Atmel Corporation: http://www.atmel.com \n
    Support email: avr@atmel.com

  Copyright (c) 2008-2011, Atmel Corporation. All rights reserved.
  Licensed under Atmel's Limited License Agreement (BitCloudTM).

\internal
  History:
    17/07/08 A. Khromykh - Created
*******************************************************************************/
/******************************************************************************
 *   WARNING: CHANGING THIS FILE MAY AFFECT CORE FUNCTIONALITY OF THE STACK.  *
 *   EXPERT USERS SHOULD PROCEED WITH CAUTION.                                *
 ******************************************************************************/

#ifndef _HALUSB_H
#define _HALUSB_H

/******************************************************************************
                        Includes section.
******************************************************************************/
#include <halClkCtrl.h>
#include <halTaskManager.h>
#include <usb.h>

/******************************************************************************
                   Define(s) section
******************************************************************************/
// Returns the maximum packet size of the given endpoint.
#define UDP_ENDPOINTS_MAXPACKETSIZE(i)    ((((i) == 4) || ((i) == 5)) ? 512 : 64)
// Returns the number of FIFO banks for the given endpoint.
#define UDP_ENDPOINTS_BANKS(i)            ((((i) == 0) || ((i) == 3)) ? 1 : 2)
// Number of UDP endpoints
#define USB_END_POINTS_NUMBER  8

// Endpoint states
// Endpoint is disabled
#define UDP_ENDPOINT_DISABLED       0
// Endpoint is halted (i.e. STALLs every request)
#define UDP_ENDPOINT_HALTED         1
// Endpoint is idle (i.e. ready for transmission)
#define UDP_ENDPOINT_IDLE           2
// Endpoint is sending data
#define UDP_ENDPOINT_SENDING        3
// Endpoint is receiving data
#define UDP_ENDPOINT_RECEIVING      4

// cortex peripheral ID features for power managment controller
#define START_ID_NUMBER_FOR_PMC1    32

/******************************************************************************
                   Types section
******************************************************************************/
typedef enum
{
  USB_END_POINT_0 = 0,
  USB_END_POINT_1 = 1,
  USB_END_POINT_2 = 2,
  USB_END_POINT_3 = 3,
  USB_END_POINT_4 = 4,
  USB_END_POINT_5 = 5,
  USB_END_POINT_6 = 6,
  USB_END_POINT_7 = 7
} HAL_EndPoints_t;

// Type of USB endpoints
typedef enum
{
  EP_CONTROL     = 0,
  EP_ISOCHRONOUS = 1,
  EP_BULK        = 2,
  EP_INTERRUPT   = 3
} HAL_UsbEndPointType_t;

// Direction of transaction
typedef enum
{
  EP_OUT = 0,
  EP_IN  = 1
} HAL_UsbEndPointDirect_t;

/* Describes an ongoing transfer on a UDP endpoint. */
typedef struct
{
  uint8_t *data;               // data - Pointer to a data buffer used for emission/reception
  uint32_t buffered;           // buffered - Number of bytes which have been written into the UDP internal FIFO buffers
  uint32_t transferred;        // transferred - Number of bytes which have been sent/received
  uint32_t remaining;          // remaining - Number of bytes which have not been buffered/transferred yet
  TransferCallback_t callback; // callback - Optional callback to invoke when the transfer completes
  void *argument;              // argument - Optional argument to the callback function
} UsbTransfer_t;

/* Describes the state of an endpoint of the UDP controller. */
typedef struct
{
  uint8_t state;          // Current endpoint state
  uint8_t bank;           // bank - Current reception bank (0 or 1)
  uint16_t size;          // size - Maximum packet size for the endpoint
  UsbTransfer_t transfer; // transfer - Describes an ongoing transfer (if current state is either <UDP_ENDPOINT_SENDING> or <UDP_ENDPOINT_RECEIVING>)
} UsbEndpoint_t;

/******************************************************************************
                   Inline static functions section
******************************************************************************/
/******************************************************************************
 Interrupt handler signal implementation
******************************************************************************/
INLINE void halInterruptEndPoints(void)
{
  halPostTask(HAL_USB_ENDPOINTS);
}

/******************************************************************************
 Interrupt handler signal implementation
******************************************************************************/
INLINE void halInterruptSuspend(void)
{
  halPostTask(HAL_USB_SUSPEND);
}

/******************************************************************************
 Interrupt handler signal implementation
******************************************************************************/
INLINE void halInterruptResume(void)
{
  halPostTask(HAL_USB_RESUME);
}

/******************************************************************************
 Interrupt handler signal implementation
******************************************************************************/
INLINE void halInterruptBusReset(void)
{
  halPostTask(HAL_USB_BUS_RESET);
}

/******************************************************************************
 Enables main clock on UDP for master clock domain
*******************************************************************************/
INLINE void halEnablePeripheralClock(void)
{
  PMC->PMC_PCER1 = (1ul << (ID_UDP - START_ID_NUMBER_FOR_PMC1));
}

/******************************************************************************
 Disables main clock on UDP for master clock domain
*******************************************************************************/
INLINE void halDisablePeripheralClock(void)
{
  PMC->PMC_PCDR1 = (1ul << (ID_UDP - START_ID_NUMBER_FOR_PMC1));
}

/******************************************************************************
 Enables 48 MHz clock on UDP phy for 12 MHz domain
*******************************************************************************/
INLINE void halEnableUsbClock(void)
{
  PMC->PMC_SCER = PMC_SCER_UDP;
}

/******************************************************************************
 Disables 48 MHz clock on UDP phy for 12 MHz domain
*******************************************************************************/
INLINE void halDisableUsbClock(void)
{
  PMC->PMC_SCDR = PMC_SCER_UDP;
}

/******************************************************************************
 Enables UDP transceiver
*******************************************************************************/
INLINE void halEnableTransceiver(void)
{
  UDP->UDP_TXVC &= ~UDP_TXVC_TXVDIS;
  UDP->UDP_TXVC |= UDP_TXVC_PUON;
}

/******************************************************************************
 Disables UDP transceiver
*******************************************************************************/
INLINE void halDisableTransceiver(void)
{
  UDP->UDP_TXVC |= UDP_TXVC_TXVDIS;
  UDP->UDP_TXVC &= ~UDP_TXVC_PUON;
}

/******************************************************************************
 Enables end point interrupt
*******************************************************************************/
INLINE void halEnableEndPointInterrupt(uint8_t eptnum)
{
  UDP->UDP_IER = (1 << eptnum);
}

/******************************************************************************
 Enables end point interrupt
*******************************************************************************/
INLINE void halDisableEndPointInterrupt(uint8_t eptnum)
{
  UDP->UDP_IDR = (1 << eptnum);
}

/******************************************************************************
                   Prototypes section
******************************************************************************/
/******************************************************************************
tuning and setting AIC for UDP
******************************************************************************/
void halUsbInterrupt(void);

/******************************************************************************
Handles a completed transfer on the given endpoint, invoking the configured
callback if any.

Parameters:
  eptnum - Number of the endpoint for which the transfer has completed.
  status - Result of the USB transfer.
******************************************************************************/
void halEndOfTransfer(uint8_t eptnum, uint8_t status);

/******************************************************************************
Resets all the endpoints of the UDP peripheral.
******************************************************************************/
void halResetEndpoints(void);

/******************************************************************************
Disables all endpoints of the UDP peripheral except control endpoint 0.
******************************************************************************/
void halDisableEndpoints(void);

/******************************************************************************
Endpoint interrupt handler. Manages IN, OUT & SETUP transaction, as well
as the STALL condition.

Parameters:
  eptnum - Number of the endpoint to handle interrupt for.
******************************************************************************/
void halEndpointHandler(uint8_t eptnum);

/******************************************************************************
Sets UDP endpoint parameters

Parameters:
   eptnum - endpoint number
   type - endpoint type
   direction - transaction direction
******************************************************************************/
void halConfigureEndpoint(uint8_t eptnum, uint8_t type, HAL_UsbEndPointDirect_t direction);

/******************************************************************************
Loads data to current FIFO and starts sending

Parameters:
  eptnum - Endpoint number.
******************************************************************************/
void halStartUsbWrite(uint8_t eptnum);

/******************************************************************************
Sets the HALT feature on the given endpoint.

Parameters:
  eptnum - Endpoint number.
******************************************************************************/
void halEndpointHaltState(uint8_t eptnum);

/******************************************************************************
Clears the Halt feature on the given endpoint.

Parameters:
  eptnum - Endpoint number.
******************************************************************************/
void halEndpointUnHaltState(uint8_t eptnum);

/******************************************************************************
Sends STALL packet

Parameters:
  eptnum - Endpoint number.
******************************************************************************/
void halSendStallToHost(uint8_t eptnum);

/******************************************************************************
Starts a remote wake-up procedure.
******************************************************************************/
//void HAL_RemoteWakeUp(void)

/******************************************************************************
Sets the device address to the given value.

Parameters:
  address - New device address.
******************************************************************************/
void halSetUsbAddress(uint8_t address);

/******************************************************************************
Sets the current device configuration.

Parameters:
  cfgnum - Configuration number to set.
******************************************************************************/
void halSetUsbConfigState(uint8_t cfgnum);

/******************************************************************************
End of bus reset state handler
******************************************************************************/
void halEndOfBusResetHandler(void);

#endif /* _HALUSB_H */
