/******************************************************************************

 @file  mt_mac.h

 @brief Monitor/Test command/response definitions for MAC subsystem

 Group: WCS LPC
 Target Device: cc13x2_26x2

 ******************************************************************************
 
 Copyright (c) 2016-2019, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ******************************************************************************
 
 
 *****************************************************************************/
#ifndef MTMAC_H
#define MTMAC_H

/******************************************************************************
 Includes
 *****************************************************************************/
#include "api_mac.h"

#include "mt.h"

#ifdef __cplusplus
extern "C"
{
#endif

/******************************************************************************
 Public Function Prototypes
 *****************************************************************************/
/*!
 * @brief   Process incoming MT MAC command messages
 *
 * @param   pMpb - pointer to incoming message parameter block
 *
 * @return  error - command processing error status
 */
extern uint8_t MtMac_commandProcessing(Mt_mpb_t *pMpb);

/*!
 * @brief   Checks if command requires an asynchronous callback
 *
 * @param   cmd1 - RPC subsystem command ID number
 *
 * @return  bool - True if command requires an asynchronous callback
 */
extern bool MtMac_isAsyncCallback(uint8_t cmd1);

#ifdef __cplusplus
}
#endif

#endif /* MTMAC_H */
