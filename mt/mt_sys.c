/******************************************************************************

 @file  mt_sys.c

 @brief Monitor/Test functions for MT SYS commands

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

/******************************************************************************
 Includes
 *****************************************************************************/
#include <string.h>

#include "api_mac.h"
#include "mt_pkt.h"
#include "mt_sys.h"
#include "mt_util.h"

/******************************************************************************
 Macros
 *****************************************************************************/
/*! AREQ RPC response for MT_MAC callbacks (indications/confirms) */
#define MT_AREQ_SYS ((uint8_t)MTRPC_CMD_AREQ | (uint8_t)MTRPC_SYS_SYS)

/******************************************************************************
  Callback Subscription ID Bit Defintions
  *****************************************************************************/
/*! SYS Callback subscription bit */
#define CBSID_RESET_IND    0x00000001
/*! All SYS callback subscription bits */
#define CBSID_ALL          0x00000001

/*! SYS callback disable command bit */
#define CBSID_DISABLE_CMD  0x80000000

/*! Default callbacks to be enabled at system reset */
#define CBSID_DEFAULT      (CBSID_ALL)

/******************************************************************************
 Local Variables
 *****************************************************************************/
static MtSys_resetIndCallback_t resetIndCallback = NULL;

/******************************************************************************
 Local Function Prototypes
 *****************************************************************************/
static void resetInd(Mt_mpb_t *pMpb);

/******************************************************************************
 Public Functions
 *****************************************************************************/
/*!
 Processes MT SYS commands received from the host

 Public function that is defined in mt_sys.h
 */
uint8_t MtSys_commandProcessing(Mt_mpb_t *pMpb)
{
    uint8_t status = ApiMac_status_success;

    switch(pMpb->cmd1)
    {
        case MTSYS_RESET_IND:
            resetInd(pMpb);
            break;

        default:
            status = ApiMac_status_commandIDError;
            break;
    }

    return status;
}


/*!
 Checks if command requires an asynchronous callback

 Public function that is defined in mt_sys.h
 */
bool MtSys_isAsyncCallback(uint8_t cmd1)
{
    return (cmd1 == MT_SYS_RESET_REQ);
}

void MtSys_registerResetIndCallback(MtSys_resetIndCallback_t callback)
{
    resetIndCallback = callback;
}

/*!
 Send an MT "reset response" message

 Public function that is defined in mt_sys.h
 */
static void resetInd(Mt_mpb_t *pMpb)
{
    uint8_t *pBuf = pMpb->pData;

    if (pMpb->length != 6)
        return;

    // Ignore other information.

    if (resetIndCallback != NULL)
        resetIndCallback(pBuf[0]);
}

/*!
 Process a Reset Request

 Public function that is defined in mt_sys.h
 */
void MtSys_resetReq(void)
{
    uint8_t data = MTSYS_RESET_HARD;
    MT_sendResponse(MT_AREQ_SYS, MT_SYS_RESET_REQ, 1, &data);
}
