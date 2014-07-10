/**************************************************************************//**
  \file  apsEndpoint.h

  \brief Private interface.of APS endpoints.

  \author
    Atmel Corporation: http://www.atmel.com \n
    Support email: avr@atmel.com

  Copyright (c) 2008-2011, Atmel Corporation. All rights reserved.
  Licensed under Atmel's Limited License Agreement (BitCloudTM).

  \internal
   History:
    2010-10-22 Max Gekk - Created.
   Last change:
    $Id: apsEndpoint.h 17448 2011-06-09 13:53:59Z ataradov $
 ******************************************************************************/
#if !defined _APS_ENDPOINT_H
#define _APS_ENDPOINT_H

/******************************************************************************
                               Includes section
 ******************************************************************************/
#include <apsdeEndpoint.h>

/******************************************************************************
                              Prototypes section
 ******************************************************************************/
/******************************************************************************
  \brief Search the registered endpoint descriptor.

  \param[in] endpoint - valid endpoint number.

  \return Valid endpoint descriptor pointer if one found NULL - at other case.
 ******************************************************************************/
APS_PRIVATE
APS_EndpointReg_t* apsFindEndpoint(const Endpoint_t endpoint);

/******************************************************************************
  \brief Reset queue of endpoints.
 ******************************************************************************/
APS_PRIVATE void apsResetEndpoints(void);

#endif /* _APS_ENDPOINT_H */
/** eof apsEndpoint.h */

