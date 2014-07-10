/**************************************************************************//**
  \file  apsCryptoKeys.h

  \brief Access interface to cryptographic keys.

  \author
    Atmel Corporation: http://www.atmel.com \n
    Support email: avr@atmel.com

  Copyright (c) 2008-2011, Atmel Corporation. All rights reserved.
  Licensed under Atmel's Limited License Agreement (BitCloudTM).

  \internal
   History:
    2010-10-31 Max Gekk - Created.
   Last change:
    $Id: apsCryptoKeys.h 18914 2011-10-18 09:13:36Z mgekk $
 ******************************************************************************/
#if !defined _APS_CRYPTO_KEYS_H
#define _APS_CRYPTO_KEYS_H

/******************************************************************************
                               Includes section
 ******************************************************************************/
#include <nwk1.h>

/******************************************************************************
                              Define(s) section
 ******************************************************************************/
/** Macro returns true if crypto key handle is valid. */
#define APS_KEY_HANDLE_IS_VALID(handle) (0 <= (handle))
/** Macro returns true if key-pair handle is successfully found. */
#define APS_KEYS_FOUND(handle) APS_KEY_HANDLE_IS_VALID(handle)
/** Macro returns true if security status is related to any security with
 *  link key.*/
#define APS_SECURED_WITH_ANY_LINK_KEY_STATUS(securityStatus) \
  ((APS_SECURED_LINK_KEY_STATUS == securityStatus) || \
   (APS_SECURED_TRUST_CENTER_LINK_KEY_STATUS == securityStatus) || \
   (APS_SECURED_HASH_OF_TRUST_CENTER_LINK_KEY_STATUS == securityStatus))


/** List of invalid and service key handle values. */
/** Initial and finish value of key-pair iterator. */
#define APS_KEY_PAIR_INITIAL -1
#define APS_KEY_PAIR_FINISH  -2
/** Memory for key-pair descriptor is out. */
#define APS_KEY_PAIR_ALLOC_FAIL -3
/** Key-pair is not found by device address. */
#define APS_KEY_PAIR_NOT_FOUND  -4
/** Parameter is out of range. */
#define APS_KEY_PAIR_INVALID_PARAMETER  -5
/** APS Key-Pair Set is not initialized. */
#define APS_KEY_PAIR_NO_INIT  -6

/** Values of reset flag. These request reset master, link keys or counters.
 * Zero or more flags can be bitwise-or'd in argument of APS_ResetKeys. */
#define APS_RESET_MASTER_KEY 0x01 /*!< Set default master key (zero key). */
#define APS_RESET_LINK_KEY   0x02 /*!< Set default link key (zero key). */
#define APS_RESET_COUNTERS   0x04 /*!< Set 0 to incoming and outgoing counters.*/

/******************************************************************************
                                Types section
 ******************************************************************************/
/* Type of outgoing frame counter. See ZigBee spec r19, Table 4.37. */
typedef uint32_t ApsOutFrameCounter_t;

/* Type of incoming frame counter. See ZigBee spec r19, Table 4.37. */
typedef uint32_t ApsInFrameCounter_t;

/** Type of cryptographic key handle. */
typedef int16_t APS_KeyHandle_t;

/** Bit map of reset flags. */
typedef uint8_t APS_KeyResetFlags_t;

/******************************************************************************
                              Prototypes section
 ******************************************************************************/
/**************************************************************************//**
  \brief Sets a link key for the device with a given extended address

  This function inserts a new entry into the APS key-pair set containing provided
  link key value and extended address.

  The function shall be called to specify the trust center link key before performing 
  network start if high security is used with ::CS_ZDO_SECURITY_STATUS set to 1 or 
  standard security with link keys is used. 
  
  In high security, if the trust center link key is unavailable
  ::CS_ZDO_SECURITY_STATUS can be set to 2 or 3; in this case a master key is used to authenticate
  the joining device to launch link key establishment procedure with the trust center.

  For communication with a node different from the trust center a separate link key is also
  needed (an application link key). If it is known to the application it shall be set with 
  the use of this function. Otherwise, the application shall either apply a master key to 
  launch link key establishment procedure (SKKE) with the partner node or request for a link
  key from the trust center.

  A typical example of the function's usage is given below:
\code
APS_KeyHandle_t apsKeyHandle; //A variable to hold a key handle
//Set a link key variable to a 16-byte value
uint8_t linkKey[SECURITY_KEY_SIZE] = {0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa};
//A variable for the trust center address predefined in ::CS_APS_TRUST_CENTER_ADDRESS.
ExtAddr_t trustCenterAddr;

//Read the trust center address
CS_ReadParameter(CS_APS_TRUST_CENTER_ADDRESS_ID, &trustCenterAddr);
//Set the trust center link key
apsKeyHandle = APS_SetLinkKey(&trustCenterAddr, linkKey);

//Check for errors
if (APS_KEY_HANDLE_IS_VALID(apsKeyHandle))
{
  ...
}
\endcode

  \param[in] deviceAddress \ref Endian "[LE]" - pointer to
                            extended (IEEE) device address
  \param[in] linkKey - pointer to a new link key

  \return A valid key handle if operation is successfully completed otherwise
          invalid key handle (Use APS_KEY_HANDLE_IS_VALID to check it out).
 ******************************************************************************/
APS_KeyHandle_t APS_SetLinkKey(const ExtAddr_t *const deviceAddress,
  const uint8_t *const linkKey);

/**************************************************************************//**
  \brief Sets a master key for the device with a given extended address

  This function inserts a new entry into the APS key-pair set containing provided
  master key value and extended address. Thus it specifies a master key used to launch
  link key establishment (SKKE) with the remote device with the corresponding extended address.

  The function is used in high security with ::CS_ZDO_SECURITY_STATUS set to 2 or 3.
  If ::CS_ZDO_SECURITY_STATUS is set to 2, then before joining the network the device
  shall specify a master key value corresponding to the trust center using this function. The
  master key in pair with the joining device's extended address must be set on the trust
  center via this function as well. If ::CS_ZDO_SECURITY_STATUS is set to 3, then the
  master key must be set only on the trust center. In this case the trust center
  trasfers the master key to the device at the beginning of the authentication procedure.

  The function can also be used to specify application master keys corresponding to devices
  different from the trust center. The need for this occurs when a link key for a remote device
  is unknown. The device shall set a master key value for a remote device with which it wishes to
  communicate and initiate the SKKE procedure to establish the link key by calling the
  APS_EstablishKeyReq() function before sending any data requests to the device. 

  See the example of usage below:
\code
APS_KeyHandle_t apsKeyHandle; //A variable to hold a key handle
//Set a master key variable to a 16-byte value
uint8_t masterKey[16] = {0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa};
//Assume the remote node extended address is held by APP_EXT_ADDR constant
ExtAddr_t partnerExtAddr = APP_EXT_ADDR;

apsKeyHandle = APS_SetMasterKey(&partnerExtAddr, masterKey); //Set the master key

//Check for errors
if (APS_KEY_HANDLE_IS_VALID(apsKeyHandle))
{
  ...
}
\endcode
Note that the APP_EXT_ADDR constant shall contain a 64-bit value in the little endian
format. For this purpose in the definition of the constant convert a common value
to the little endian format using the CCPU_TO_LE64 format as follows:
\code
#define APP_EXT_ADDR     CCPU_TO_LE64(0xAAAAAAAAAAAAAAAALL)
\endcode

An extended address corresponding to a given short address can be obtained
through Device Discovery, while short addresses of devices supporting certain services can be
discovered using Service Discovery.

  \param[in] deviceAddress \ref Endian "[LE]" - pointer to extended
                                                IEEE device address.
  \param[in] masterKey - pointer to new cryptographic master key.

  \return Valid key handle if operation is successfully completed otherwise
          invalid key handle (Use APS_KEY_HANDLE_IS_VALID to check it out).
 ******************************************************************************/
APS_KeyHandle_t APS_SetMasterKey(const ExtAddr_t *const deviceAddress,
  const uint8_t *const masterKey);

/**************************************************************************//**
  \brief Find the master key or the link key corresponding to a given extended address

  The function is used to check whether the link key or the master key has been set for a given
  device identified by its extended address and to retrieve a handle to it if one exists. To
  check whether the key has been found use the APS_KEYS_FOUND macro. If the handle is
  valid an access to a key value is got via APS_GetMasterKey() or APS_GetLinkKey().

  See the example:
\code
//Assume that the extended address of interest is held by APP_EXT_ADDR constant
ExtAddr_t partnerExtAddr = APP_EXT_ADDR;

if (!APS_KEYS_FOUND(APS_FindKeys(&partnerExtAddr)))
{
  ... //Perform required action, e.g. request for a link key, using APS_RequestKeyReq()
}
\endcode

  \param[in] deviceAddress \ref Endian "[LE]" - pointer to extended IEEE
                                                device address.

  \return Valid key handle if any key is found otherwise
          invalid key handle (Use APS_KEYS_FOUND to check it out).
 ******************************************************************************/
APS_KeyHandle_t APS_FindKeys(const ExtAddr_t *const deviceAddress);

/**************************************************************************//**
  \brief Get a pointer to the master key value by a given key handle

  The function returns a pointer to the section of memory containing the master key value
  for a given key handle. A key handle points to a certain entry in the APS key-pair set
  corresponding to a specific extended address. To obtain a key handle for a given
  extended address use the APS_FindKeys() function.

  Note that the received pointer must be only used to read the value and not to
  modify it.

  Consider the example:
\code
//Search for keys associated with the provided extended address
const APS_KeyHandle_t keyHandle = APS_FindKeys(&extAddr);

uint8_t *masterKey;

if (APS_KEYS_FOUND(keyHandle) && NULL != (masterKey = APS_GetMasterKey(keyHandle)))
{
  ... //A pointer to the master key value has been successfully captured
}
\endcode


  \param[in] handle - valid key handle.

  \return A pointer to the master key or NULL if the key handle is invalid
 ******************************************************************************/
uint8_t* APS_GetMasterKey(const APS_KeyHandle_t handle);

/**************************************************************************//**
  \brief Get a pointer to the link key value by a given key handle

  The function returns a pointer to the section of memory containing the link key value
  for a given key handle. A key handle points to a certain entry in the APS key-pair set
  corresponding to a specific extended address. To obtain a key handle for a given
  extended address use the APS_FindKeys() function.

  Note that the received pointer must be only used to read the value and not to
  modify it.

  Consider the example:
\code
//Search for keys associated with the provided extended address
const APS_KeyHandle_t keyHandle = APS_FindKeys(&extAddr);

uint8_t *linkKey;

if (APS_KEYS_FOUND(keyHandle) && (linkKey = APS_GetLinkKey(keyHandle)))
{
  ... //A pointer to the link key value has been successfully captured
}
\endcode

  \param[in] handle - valid key handle.

  \return A pointer to the link key or NULL if the key handle is invalid
 ******************************************************************************/
uint8_t* APS_GetLinkKey(const APS_KeyHandle_t handle);

/**************************************************************************//**
  \brief Delete key-pair - master and link keys.

  \param[in] deviceAddress \ref Endian "[LE]" - pointer to extended
                                                IEEE device address.
  \param[in] notify - notify to upper layer or not.

  \return 'true' if the key-pair is removed otherwise false.
 ******************************************************************************/
bool APS_DeleteKeyPair(ExtAddr_t *const deviceAddress, const bool notify);

/**************************************************************************//**
  \brief Get next key handle.

  \code Example:
    APS_KeyHandle_t handle = APS_KEY_PAIR_INITIAL;

    while (APS_KEYS_FOUND(handle = APS_NextKeys(handle)))
    {
      linkKey = APS_GetLinkKey(handle);
      if (NULL != linkKey)
        ...
      ...
    }
  \endcode

  \param[in] handle - handle of previous key-pair or APS_KEY_PAIR_INITIAL
                      if it's initial call.

  \return if next key-pair is found then return valid key handle
          otherwise return APS_KEY_PAIR_FINISH.
 ******************************************************************************/
APS_KeyHandle_t APS_NextKeys(const APS_KeyHandle_t handle);

/**************************************************************************//**
  \brief (Un)authorize cryptographic key-pair of given device.

  \param[in] deviceAddress \ref Endian "[LE]" - pointer to extended IEEE
                                                device address.
  \param[in] status - 'true' for authorized keys otherwise 'false'.

  \return None.
 ******************************************************************************/
void APS_SetAuthorizedStatus(const ExtAddr_t *const deviceAddress,
  const bool status);

/**************************************************************************//**
  \brief Check authorization of crypthographic key-pair.

  \param[in] deviceAddress \ref Endian "[LE]" - pointer to extended
                                                IEEE device address.

  \return 'true' if key-pair is authorized otherwise 'false'.
 ******************************************************************************/
bool APS_AreKeysAuthorized(const ExtAddr_t *const deviceAddress);

/**************************************************************************//**
  \brief Reset device's keys and counters to default values.

  \param[in] deviceAddress \ref Endian "[LE]" - pointer to extended IEEE
                                                device address.
  \param[in] flags - bit map of APS Key-Pair reset flags. Current implementation
                     supported only APS_RESET_LINK_KEY and APS_RESET_COUNTERS.

  \return APS Key Handle of reseting key-pair or invalid key handle if
          key-pair is not found.
 ******************************************************************************/
APS_KeyHandle_t APS_ResetKeys(const ExtAddr_t *const deviceAddress,
  const APS_KeyResetFlags_t flags);

/**************************************************************************//**
  \brief Init APS key-pair set.

  \param[in] powerFailureControl - stack restoring after power failure control bitfield;
                                  affects on initialization procedure.

  \return None.
 ******************************************************************************/
void APS_InitKeyPairSet(const NWK_PowerFailureControl_t powerFailureControl);

/**************************************************************************//**
  \brief Get extended device address of key-pair.

  \param[in] handle - valid key handle.

  \return Pointer to device address or NULL if key-pair is not found..
 ******************************************************************************/
ExtAddr_t* APS_GetKeyPairDeviceAddress(const APS_KeyHandle_t handle);

/**************************************************************************//**
  \brief Find key-pair with old address and set new address.

  \param[in] oldAddress \ref Endian "[LE]" - extended IEEE device
                                             address of key-pair.
  \param[in] newAddress \ref Endian "[LE]" - new device address of key-pair.
  \return None.
 ******************************************************************************/
void APS_ChangeKeyPairDeviceAddress(const ExtAddr_t oldAddress,
  const ExtAddr_t newAddress);

#ifdef _TC_PROMISCUOUS_MODE_
/**************************************************************************//**
  \brief Get preinstalled link key.

  \param[in] handle - valid key handle.

  \return Pointer to preinstalled link key or NULL if handle is invalid.
 ******************************************************************************/
uint8_t* APS_GetPreinstalledLinkKey(const APS_KeyHandle_t handle);

/**************************************************************************//**
  \brief Set new preinstalled link key for given device.

   This function copies value of preinstalled link key to APS Key-Pair Set.

  \param[in] deviceAddress - pointer to extended IEEE device address.
  \param[in] preinstalledLinkKey - pointer to new cryptographic preinstalled
                                   link key.

  \return Valid key handle if operation is successfully completed otherwise
          invalid key handle (Use APS_KEY_HANDLE_IS_VALID to check it out).
 ******************************************************************************/
APS_KeyHandle_t APS_SetPreinstalledLinkKey(const ExtAddr_t *const deviceAddress,
  const uint8_t *const preinstalledLinkKey);

/**************************************************************************//**
  \brief Restores link key from preinstalled one.
         Uses to accept TC reestablishing keys with previously joined device.

  \param[in] deviceAddress - pointer to extended IEEE device address.

  \return None.
 ******************************************************************************/
void APS_RestorePreinstalledLinkKey(const ExtAddr_t *const deviceAddress);

#endif /* _TC_PROMISCUOUS_MODE_ */

#ifdef _LINK_SECURITY_ 
/**************************************************************************//**
  \brief Get current value of outgoing security frames counter.

  \param[in] keyHandle - valid key handle.

  \return Current value of outgoing frame counter.
 ******************************************************************************/
ApsOutFrameCounter_t APS_GetOutSecFrameCounter(const APS_KeyHandle_t keyHandle);
#endif /* _LINK_SECURITY_ */


#endif /* _APS_CRYPTO_KEYS_H */
/** eof apsCryptoKeys.h */

