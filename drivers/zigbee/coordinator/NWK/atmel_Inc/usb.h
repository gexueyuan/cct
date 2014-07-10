/**************************************************************************//**
  \file  usb.h

  \brief The header file describes the usb interface

  \author
    Atmel Corporation: http://www.atmel.com \n
    Support email: avr@atmel.com

  Copyright (c) 2008-2011, Atmel Corporation. All rights reserved.
  Licensed under Atmel's Limited License Agreement (BitCloudTM).

  \internal
  History:
    17/07/08 A. Khromykh - Created
 ******************************************************************************/
/******************************************************************************
 *   WARNING: CHANGING THIS FILE MAY AFFECT CORE FUNCTIONALITY OF THE STACK.  *
 *   EXPERT USERS SHOULD PROCEED WITH CAUTION.                                *
 ******************************************************************************/

#ifndef _USB_H
#define _USB_H

/******************************************************************************
                   Define(s) section
******************************************************************************/
// \cond
// USB device API return values
// Indicates the operation was successful
#define STATUS_SUCCESS             0
// Endpoint/device is already busy
#define STATUS_BUSY                1
// Operation has been aborted
#define STATUS_ABORTED             2
// Operation has been aborted because the device was configured
#define STATUS_RESET               3

// USB device states
// The device is currently suspended
#define DEVICE_SUSPENDED            0
// USB cable is plugged into the device
#define DEVICE_ATTACHED             1
// Host is providing +5V through the USB cable
#define DEVICE_POWERED              2
// Device has been reset
#define DEVICE_DEFAULT              3
// The device has been given an address on the bus
#define DEVICE_ADDRESS              5
// A valid configuration has been selected
#define DEVICE_CONFIGURED           6
// \endcond

/******************************************************************************
                   Types section
******************************************************************************/
// transaction callback type
typedef void (* TransferCallback_t)(void *pArg, uint8_t status, uint16_t transferred, uint16_t remaining);

BEGIN_PACK
// Usb endpoint descriptor
typedef struct PACK
{
  uint8_t  bLength;          // Size of the descriptor in bytes
  uint8_t  bDescriptorType;  // Descriptor type
  uint8_t  bEndpointAddress; // Address and direction of the endpoint
  uint8_t  bmAttributes;     // Endpoint type and additional characteristics (for isochronous endpoints)
  uint16_t wMaxPacketSize;   // Maximum packet size (in bytes) of the endpoint
  uint8_t  bInterval;        // Polling rate of the endpoint
} HAL_UsbEndPointDescptr_t;
END_PACK

/******************************************************************************
                   Prototypes section
******************************************************************************/
/**************************************************************************//**
\brief Registers user's request handler

\param[in]
  f - pointer to user's callback
******************************************************************************/
void HAL_RegisterRequestHandler(void (* f)(uint8_t *req));

/**************************************************************************//**
\brief Registers user's end of bus reset handler

\param[in]
  f - pointer to user's callback
******************************************************************************/
void HAL_RegisterEndOfBusResetHandler(void (* f)(void));

/**************************************************************************//**
\brief Registers user's resume handler

\param[in]
  f - pointer to user's callback
******************************************************************************/
void HAL_RegisterResumeHandler(void (* f)(void));

/**************************************************************************//**
\brief Registers user's suspend handler

\param[in]
  f - pointer to user's callback
******************************************************************************/
void HAL_RegisterSuspendHandler(void (* f)(void));

/**************************************************************************//**
\brief Configures an endpoint according to its Endpoint Descriptor.

\param[in]
  descriptor - Pointer to an Endpoint descriptor.
******************************************************************************/
void HAL_ConfigureEndpoint(HAL_UsbEndPointDescptr_t *descriptor);

/**************************************************************************//**
\brief Sends data through a USB endpoint. Sets up the transfer descriptor,
writes one or two data payloads (depending on the number of FIFO bank
for the endpoint) and then starts the actual transfer. The operation is
complete when all the data has been sent.

*If the size of the buffer is greater than the size of the endpoint
(or twice the size if the endpoint has two FIFO banks), then the buffer
must be kept allocated until the transfer is finished*. This means that
it is not possible to declare it on the stack (i.e. as a local variable
of a function which returns after starting a transfer).

\param[in]
  eptnum - Endpoint number.
\param[in]
  data - Pointer to a buffer with the data to send.
\param[in]
  size - Size of the data buffer.
\param[in]
  callback - Optional callback function to invoke when the transfer is complete.
\param[in]
  argument - Optional argument to the callback function.

\return
  STATUS_SUCCESS if the transfer has been started; otherwise, the
  corresponding error status code.
******************************************************************************/
uint8_t HAL_UsbWrite(uint8_t eptnum, void *data, uint32_t size, TransferCallback_t callback, void *argument);

/**************************************************************************//**
\brief Reads incoming data on an USB endpoint This methods sets the transfer
descriptor and activate the endpoint interrupt. The actual transfer is
then carried out by the endpoint interrupt handler. The Read operation
finishes either when the buffer is full, or a short packet (inferior to
endpoint maximum  size) is received.

*The buffer must be kept allocated until the transfer is finished*.

\param[in]
  eptnum - Endpoint number.
\param[in]
  data - Pointer to a data buffer.
\param[in]
  size - Size of the data buffer in bytes.
\param[in]
  callback - Optional end-of-transfer callback function.
\param[in]
  argument - Optional argument to the callback function.

\return
  STATUS_SUCCESS if the read operation has been started; otherwise,
  the corresponding error code.
******************************************************************************/
uint8_t HAL_UsbRead(uint8_t eptnum, void *data, uint32_t size, TransferCallback_t callback, void *argument);

/**************************************************************************//**
\brief Sets the HALT feature on the given endpoint (if not already in this state).

\param[in]
  eptnum - Endpoint number.
******************************************************************************/
void HAL_Halt(uint8_t eptnum);

/**************************************************************************//**
\brief Clears the Halt feature on the given endpoint.

\param[in]
  eptnum - Endpoint number.
******************************************************************************/
void HAL_Unhalt(uint8_t eptnum);

/**************************************************************************//**
\brief Returns the current Halt status of an endpoint.

\param[in]
  eptnum - Endpoint number.

\return
  1 - if the endpoint is currently halted;
  0 - otherwise.
******************************************************************************/
uint8_t HAL_IsHalted(uint8_t eptnum);

/**************************************************************************//**
\brief Causes the given endpoint to acknowledge the next packet it receives with
a STALL handshake.

\param[in]
  eptnum - Endpoint number.

\return
  STATUS_SUCCESS or STATUS_BUSY.
******************************************************************************/
uint8_t HAL_Stall(uint8_t eptnum);

/**************************************************************************//**
\brief Sets the device address to the given value.

\param[in]
  address - New device address.
******************************************************************************/
void HAL_SetAddress(uint8_t *address);

/**************************************************************************//**
\brief Sets the current device configuration.

\param[in]
  cfgnum - Configuration number to set.
******************************************************************************/
void HAL_SetConfiguration(uint8_t cfgnum);

/**************************************************************************//**
\brief Initializes the USB driver. This function must be called before host
bus reset and after pull up is connected to D+. After pull up was connected
to D+ host resets device with 100ms delay.

\param[in]
  reqMem - Memory for usb request. Memory is allocated by user.
******************************************************************************/
void HAL_UsbInit(uint8_t *reqMem);

/**************************************************************************//**
\brief Returns the current state of the USB device.

\return
  Device current state.
******************************************************************************/
uint8_t HAL_GetState(void);

#endif /* _USB_H */
