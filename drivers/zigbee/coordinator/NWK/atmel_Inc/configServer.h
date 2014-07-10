/**************************************************************************//**
\file configServer.h

  \brief
    Configuration Server header file

  \author
    Atmel Corporation: http://www.atmel.com \n
    Support email: avr@atmel.com

  Copyright (c) 2010 , Atmel Corporation. All rights reserved.
  Licensed under Atmel's Limited License Agreement (BitCloudTM).

  \internal
    History:
    18.10.10 A. Razinkov - Created.
******************************************************************************/

#ifndef _CONFIG_SERVER_H
#define _CONFIG_SERVER_H

/******************************************************************************
                    Includes section
******************************************************************************/
#include <types.h>
#include <csDefaults.h>

/******************************************************************************
                    Types section
******************************************************************************/
/*
 * \brief List of the Configuration Server parameter identifiers.
 *        Identifiers are automatically sorted by memory location
 *        and item types (atomic parameter or memory region).
 */
typedef enum _CS_MemoryItemId_t
{
#define SEPARATOR(id)
#define DUMMY_MEMORY(id)
#define RAM_PARAMETER(id, addr) id,
#define FLASH_PARAMETER(id, addr)
#define MEMORY_REGION(id, addr)
#include "csParamTable.h"
#undef SEPARATOR
#undef DUMMY_MEMORY
#undef RAM_PARAMETER
#undef FLASH_PARAMETER
#undef MEMORY_REGION

  CS_FLASH_PARAMETERS_START_ID,
#define SEPARATOR(id)
#define DUMMY_MEMORY(id)
#define RAM_PARAMETER(id, addr)
#define FLASH_PARAMETER(id, addr) id,
#define MEMORY_REGION(id, addr)
#include "csParamTable.h"
#undef SEPARATOR
#undef DUMMY_MEMORY
#undef RAM_PARAMETER
#undef FLASH_PARAMETER
#undef MEMORY_REGION

#define SEPARATOR(id)
#define DUMMY_MEMORY(id) id,
#define RAM_PARAMETER(id, addr)
#define FLASH_PARAMETER(id, addr)
#define MEMORY_REGION(id, addr) id,
#include "csParamTable.h"
#undef SEPARATOR
#undef DUMMY_MEMORY
#undef RAM_PARAMETER
#undef FLASH_PARAMETER
#undef MEMORY_REGION

} CS_MemoryItemId_t;

/******************************************************************************
                    Functions prototypes section
******************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif


/****************************************************************************//**
\brief Configuration Server initialization
******************************************************************************/
void CS_Init(void);

/******************************************************************************//**
\brief Gets the value of the parameter specified by an ID and writes it at the provided address in memory

The function reads a value of a ConfigServer parameter specified by its ID. A parameter ID is a constant
which name can be obtained by adding "_ID" suffix to the name of the parameter as it can be observed
in csDefaults.h. For example, the identifier of the CS_NWK_PANID parameter will be CS_NWK_PANID_ID.
The function copies the value of the parameter to the location in memory specified by the second argument.
An example of a typical use of the function is reading the extended address of the current device:

\code
ExtAddr_t ownExtAddr;
CS_ReadParameter(CS_UID_ID, &ownExtAddr);
\endcode

\param[in] parameterId -  an ID of the parameter to be read
\param[out] memoryPtr - a variable to which the value of the parameter is written
******************************************************************************/
void CS_ReadParameter(CS_MemoryItemId_t parameterId, void *memoryPtr);

/******************************************************************************//**
\brief Sets a value of a certain ConfigServer parameter specified by ID

The function assigns the specified parameter to a new value.  The value is copied from the address
specified by the second argument, while the first takes a parameter identifier formed by adding "_ID"
suffix to the parameter's name. For example, to set a specific value for own extended address
proceed with the following:

\code
ExtAddr_t ownExtAddr = 0x0123456789ABCDEF;
CS_WriteParameter(CS_UID_ID, &ownExtAddr);
\endcode

\param[in] parameterId - an ID of the parameter being written
\param[out] memoryPtr - a pointer to a new value of the parameter
******************************************************************************/
void CS_WriteParameter(CS_MemoryItemId_t parameterId, const void *parameterValue);

/***********************************************************************************//**
\brief Gets a pointer to the memory allocated for a specific internal structure

The function is used to obtain a pointer to the memory marked with a special identifier. The memory
is allocated by the stack for certain internl buffers and table. For example, this function can be
emplued to get the contents of bidning, routing, key pair descrptors, or other tables. For some of 
these objects there is a dedicated API in BitCloud, in such cases the application must use this API.
In case the API is absent the user can use this function to observe the contents of the momory, but
must not use the pointer obtained with the function to write data to the memory. Otherwise, the memory
may be corrupted and the application may encounter an unexpected failure. For all purposes when it
is needed to write data to the tables, ButCloud provides appropriate API.
The function should be provided with an address of a pointer variable which will be assigned to
the starting point of the memory.

  \param[in] memoryId - and ID of a parameter or a specific part of the memory
  \param[out] memoryPtr - a pointer to which a starting address of the memory is written

***************************************************************************************/
void CS_GetMemory(CS_MemoryItemId_t memoryId, void **memoryPtr);

/**************************************************************************//**
\brief Set persist default values
*****************************************************************************/
void CS_PdsDefaultValue(void);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* _CONFIG_SERVER_H */
/* eof configServer.h */
