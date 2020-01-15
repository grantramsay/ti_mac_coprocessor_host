/******************************************************************************

 @file  mt_mac.c

 @brief Monitor/Test functions for MT MAC commands/callbacks

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

#include "mt_mac.h"
#include "mt_pkt.h"
#include "mt_rpc.h"
#include "mt_util.h"

/******************************************************************************
 * Macros
 *****************************************************************************/
/*! AREQ RPC response for MT_MAC callbacks (indications/confirms) */
#define MT_AREQ_MAC ((uint8_t)MTRPC_CMD_AREQ | (uint8_t)MTRPC_SYS_MAC)
#define MT_SREQ_MAC ((uint8_t)MTRPC_CMD_SREQ | (uint8_t)MTRPC_SYS_MAC)
#define MT_SRSP_MAC ((uint8_t)MTRPC_CMD_SRSP | (uint8_t)MTRPC_SYS_MAC)

/******************************************************************************
 Constants
 *****************************************************************************/
/*!
 A MAC PIB table of Device-Descriptor entries, each indicating a remote device
 with which this device securely communicates. This PIB attribute should not
 be accessed by the host device.
 */
#define MT_MAC_PIB_DEVICE_TABLE  0x72
/*!
 A MAC PIB table of SecurityLevel-Descriptor entries, each with information
 about the minimum security level expected depending on incoming frame type
 and subtype. This PIB attribute should not be accessed by the host device.
 */
#define MT_MAC_PIB_SECURITY_LEVEL_TABLE  0x73

/*
 TX Options - these should track values in the MAC file: mac_api.h
 */
/*! MAC will attempt to retransmit the frame until it is acknowledged */
#define MT_MAC_TXOPTION_ACK         0x01
/*! GTS transmission (currently unsupported) */
#define MT_MAC_TXOPTION_GTS         0x02
/*! MAC will queue the data and wait for the destination device to poll */
#define MT_MAC_TXOPTION_INDIRECT    0x04
/*! Force the pending bit set for direct transmission */
#define MT_MAC_TXOPTION_PEND_BIT    0x08
/*! Prevent the frame from being retransmitted */
#define MT_MAC_TXOPTION_NO_RETRANS  0x10
/*! Prevent a MAC_MCPS_DATA_CNF event from being sent */
#define MT_MAC_TXOPTION_NO_CNF      0x20
/*! Use PIB value MAC_ALT_BE for the minimum backoff exponent */
#define MT_MAC_TXOPTION_ALT_BE      0x40
/*! Use the power/channel values in DataReq instead of PIB values */
#define MT_MAC_TXOPTION_PWR_CHAN    0x80

/*!
 MT indication/confirm minimum packed serial buffer sizes
 */
/*! Minimum serial packet size - Async Data Ind */
#define MT_MAC_LEN_ASYNC_IND     sizeof(MtPkt_adataInd_t)
/*! Required serial packet size - Associate Cnf */
#define MT_MAC_LEN_ASSOC_CNF     sizeof(MtPkt_assocCnf_t)
/*! Required serial packet size - Associate Ind */
#define MT_MAC_LEN_ASSOC_IND     sizeof(MtPkt_assocInd_t)
/*! Required serial packet size - Associate Ind */
#define MT_MAC_LEN_ASYNC_CNF     sizeof(MtPkt_asyncCnf_t)
/*! Minimum serial packet size - Beacon Ind */
#define MT_MAC_LEN_BEACON_IND    sizeof(MtPkt_beaconInd_t)
/*! Required serial packet size - Comm Status Ind */
#define MT_MAC_LEN_COMMST_IND    sizeof(MtPkt_commstInd_t)
/*! Required serial packet size - Data Cnf */
#define MT_MAC_LEN_DATA_CNF      sizeof(MtPkt_dataCnf_t)
/*! Minimum serial packet size - Data Ind */
#define MT_MAC_LEN_DATA_IND      sizeof(MtPkt_dataInd_t)
/*! Required serial packet size - Disassociate Cnf */
#define MT_MAC_LEN_DISASSOC_CNF  sizeof(MtPkt_dassocCnf_t)
/*! Required serial packet size - Disassociate Ind */
#define MT_MAC_LEN_DISASSOC_IND  sizeof(MtPkt_dassocInd_t)
/*! Required serial packet size - eBeacon Ind */
#define MT_MAC_LEN_EBEACON_IND   sizeof(MtPkt_ebeaconInd_t)
/*! Required serial packet size - Orphan Ind */
#define MT_MAC_LEN_ORPHAN_IND    sizeof(MtPkt_orphanInd_t)
/*! Required serial packet size - Poll Cnf */
#define MT_MAC_LEN_POLL_CNF      sizeof(MtPkt_pollCnf_t)
/*! Required serial packet size - Poll Ind */
#define MT_MAC_LEN_POLL_IND      sizeof(MtPkt_pollInd_t)
/*! Required serial packet size - Purge Cnf */
#define MT_MAC_LEN_PURGE_CNF     sizeof(MtPkt_purgeCnf_t)
/*! Minimum serial packet size - Scan Cnf */
#define MT_MAC_LEN_SCAN_CNF      sizeof(MtPkt_scanCnf_t)
/*! Required serial packet size - Start Cnf */
#define MT_MAC_LEN_START_CNF     sizeof(MtPkt_startCnf_t)
/*! Required serial packet size - Poll Ind */
#define MT_MAC_LEN_SYLOSS_IND    sizeof(MtPkt_sylossInd_t)

/*! Packed Security data block length */
#define MT_MAC_LEN_SECINFO  sizeof(MtPkt_secInfo_t)

/*! Packed Pan Descriptor data block length */
#define MT_MAC_LEN_PANDESC  sizeof(MtPkt_panDesc_t)

/*! Maximum net name size in FHPIB (see fh_pib.h) */
#define FHPIB_NET_NAME_SIZE_MAX  32

/*! Maximum parameter size for FHPIB (see macNetName[] in fh_pib.h) */
#define MAX_PARAM_SIZE_FH  (FHPIB_NET_NAME_SIZE_MAX * sizeof(uint8_t))

/*! Maximum parameter size for MAC PIB (see mac_pib.h) */
#define MAX_PARAM_SIZE_MAC  (sizeof(ApiMac_sAddr_t))

/*! Maximum parameter size for MAC PIB (see mac_security_pib.h) */
#define MAX_PARAM_SIZE_SEC  (APIMAC_KEY_SOURCE_MAX_LEN*sizeof(uint8_t))

/*! Maximum active scan results (to limit allocated buffer size) */
#define MAX_ACTIVE_SCAN_RESULTS  6

/******************************************************************************
 Callback Subscription ID Bit Defintions
 *****************************************************************************/
/* MAC Callback subscription ID bits */
#define CBSID_ASSOCIATE_CNF      0x00000001
#define CBSID_ASSOCIATE_IND      0x00000002
#define CBSID_BEACON_NOTIFY_IND  0x00000004
#define CBSID_COMM_STATUS_IND    0x00000008
#define CBSID_DATA_CNF           0x00000010
#define CBSID_DATA_IND           0x00000020
#define CBSID_DISASSOCIATE_CNF   0x00000040
#define CBSID_DISASSOCIATE_IND   0x00000080
#define CBSID_ORPHAN_IND         0x00000100
#define CBSID_POLL_CNF           0x00000200
#define CBSID_POLL_IND           0x00000400
#define CBSID_PURGE_CNF          0x00000800
#define CBSID_SCAN_CNF           0x00001000
#define CBSID_START_CNF          0x00002000
#define CBSID_SYNC_LOSS_IND      0x00004000
#define CBSID_WS_ASYNC_CNF       0x00008000
#define CBSID_WS_ASYNC_IND       0x00010000

#define CBSID_ALL                0x0001FFFF
#define CBSID_DISABLE_CMD        0x80000000

/* Default callbacks to be enabled at system reset */
#define CBSID_DEFAULT            (CBSID_ALL)

/******************************************************************************
 Local Variables
 *****************************************************************************/
static ApiMac_callbacks_t macCallbacks = { 0 };

/******************************************************************************
 Local Function Prototypes
 *****************************************************************************/
/* MT API request/response functions */
static void macDataCnf(const Mt_mpb_t *pMpb);
static void macDataInd(const Mt_mpb_t *pMpb);

static void macAsyncInd(const Mt_mpb_t *pMpb);

static ApiMac_status_t setReq(uint8_t pibAttribute, const void *data, uint8_t len);
static void dataInd(uint8_t indType, const Mt_mpb_t *pMpb);

/* General utility functions */
static ApiMac_status_t syncRequestWithStatusReply(uint8_t cmd, uint16_t len, uint8_t *pReq);
static uint8_t *copyExtAdr(uint8_t *pDst, uint8_t *pSrc);
static uint8_t *macAdrToSba(uint8_t *pDst, ApiMac_sAddr_t *pSrc);
static void macSbaToAdr(ApiMac_sAddr_t *pDst, uint8_t *pSrc);
static void macSbaToSec(ApiMac_sec_t *pDst, uint8_t *pSrc);
static void macSecToSba(uint8_t *pDst, ApiMac_sec_t *pSrc);
static uint8_t *bufferTxOptBits(uint8_t *pBuf, const ApiMac_txOptions_t *txOptions);

/******************************************************************************
 Public Functions
 *****************************************************************************/
void ApiMac_registerCallbacks(ApiMac_callbacks_t *pCallbacks)
{
    /* Save the application's callback table */
    macCallbacks = *pCallbacks;
}

/*!
 Processes MT MAC commands received from the host

 Public function that is defined in mt_mac.h
 */
uint8_t MtMac_commandProcessing(Mt_mpb_t *pMpb)
{
    uint8_t status = ApiMac_status_success;

    switch(pMpb->cmd1)
    {
        case MT_MAC_DATA_CNF:
            macDataCnf(pMpb);
            break;

        case MT_MAC_DATA_IND:
            macDataInd(pMpb);
            break;

        case MT_MAC_ASYNC_IND:
            macAsyncInd(pMpb);
            break;

        default:
            status = ApiMac_status_commandIDError;
            break;
    }

    return status;
}

/*!
 Checks if command requires an asynchronous callback

 Public function that is defined in mt_mac.h
 */
bool MtMac_isAsyncCallback(uint8_t cmd1)
{
    bool res;
    switch (cmd1)
    {
        case MT_MAC_SYNC_LOSS_IND:
        case MT_MAC_ASSOCIATE_IND:
        case MT_MAC_ASSOCIATE_CNF:
        case MT_MAC_BEACON_NOTIFY_IND:
        case MT_MAC_DATA_CNF:
        case MT_MAC_DATA_IND:
        case MT_MAC_DISASSOCIATE_IND:
        case MT_MAC_DISASSOCIATE_CNF:
        case MT_MAC_ORPHAN_IND:
        case MT_MAC_POLL_CNF:
        case MT_MAC_SCAN_CNF:
        case MT_MAC_COMM_STATUS_IND:
        case MT_MAC_START_CNF:
        case MT_MAC_PURGE_CNF:
        case MT_MAC_POLL_IND:
        case MT_MAC_ASYNC_CNF:
        case MT_MAC_ASYNC_IND:
            res = true;
            break;
        default:
            res = false;
            break;
    }
    return res;
}

/******************************************************************************
 * Local API Request Functions
 *****************************************************************************/
/*!
 * @brief   Process MAC_DATA_REQ command issued by host
 *
 * @param   pMpb - pointer to incoming message parameter block
 */
ApiMac_status_t ApiMac_mcpsDataReq(ApiMac_mcpsDataReq_t *pData)
{
    uint16_t len = sizeof(MtPkt_dataReq_t) + pData->msdu.len + pData->payloadIELen;
    uint8_t buff[len];
    uint8_t *pBuf = buff;

    /* Destination address */
    pBuf = macAdrToSba(pBuf, &pData->dstAddr);

    /* Destination Pan ID */
    pBuf = Util_bufferUint16(pBuf, pData->dstPanId);

    /* Source address mode */
    *pBuf++ = pData->srcAddrMode;

    /* MSDU handle */
    *pBuf++ = pData->msduHandle;

    /* TX options bit mask */
    pBuf = bufferTxOptBits(pBuf, &pData->txOptions);

    /* TX channel */
    *pBuf++ = pData->channel;

    /* TX power */
    *pBuf++ = pData->power;

    /* Security parameters */
    macSecToSba(pBuf, &pData->sec);
    pBuf += MT_MAC_LEN_SECINFO;

    /* Frequency hopping Protocol Dispatch - HARDWIRED FOR NOW */

    /* Frequency hopping bitmask for IEs to include */
    pBuf = Util_bufferUint32(pBuf, pData->includeFhIEs);

    /* MSDU length */
    pBuf = Util_bufferUint16(pBuf, pData->msdu.len);

    /* Payload IE length */
    pBuf = Util_bufferUint16(pBuf, pData->payloadIELen);

    /* MSDU */
    memcpy(pBuf, pData->msdu.p, pData->msdu.len);
    pBuf += pData->msdu.len;

    /* IE List */
    memcpy(pBuf, pData->pIEList, pData->payloadIELen);

    /* gpOffset and gpDuration - HARDWIRED FOR NOW */

    /* Send request */
    ApiMac_status_t status = syncRequestWithStatusReply(
            MT_MAC_DATA_REQ, len, buff);

    return status;
}

/*!
 * @brief   Process MAC_RESET_REQ command issued by host
 *
 * @param   pMpb - pointer to incoming message parameter block
 */
ApiMac_status_t ApiMac_mlmeResetReq(bool setDefaultPib)
{
    uint8_t buff[sizeof(MtPkt_resetReq_t)];
    uint8_t *pBuf = buff;

    *pBuf = setDefaultPib;

    /* Send request */
    ApiMac_status_t status = syncRequestWithStatusReply(
            MT_MAC_RESET_REQ, sizeof(buff), buff);

    return status;
}

/*!
 * @brief   Process MAC_GET_REQ command issued by host
 *
 * @param   pMpb - pointer to incoming message parameter block
 */
static ApiMac_status_t getReq(uint8_t pibAttribute, void *data, uint8_t len)
{
    uint8_t req = (uint8_t)pibAttribute;
    uint8_t responseBuff[1 + 16];
    uint16_t responseLength = 1 + len;
    memset(responseBuff, 0, sizeof(responseBuff));

    if (len > 16)
        return ApiMac_status_lengthError;

    bool got_response = MT_sendSyncRequest(
            MT_SREQ_MAC, MT_MAC_GET_REQ, 1, &req,
            MT_SRSP_MAC, MT_MAC_GET_REQ, &responseLength, responseBuff);

    ApiMac_status_t status = ApiMac_status_subSystemError;
    if (got_response && responseLength >= 1) {
        status = (ApiMac_status_t) responseBuff[0];
        memcpy(data, responseBuff + 1, MIN(len, responseLength - 1));
    }

    return status;
}

/*!
 * @brief       This direct execute function retrieves an attribute value from
 *              the MAC PIB.
 *
 * @param       pibAttribute - The attribute identifier
 * @param       pValue - pointer to the attribute value
 *
 * @return      The status of the request
 */
ApiMac_status_t ApiMac_mlmeGetReqBool(
        ApiMac_attribute_bool_t pibAttribute,
        bool *pValue)
{
    uint8_t value = 0;
    ApiMac_status_t res = getReq(pibAttribute, &value, 1);
    *pValue = (bool)value;
    return res;
}

/*!
 * @brief       This direct execute function retrieves an attribute value from
 *              the MAC PIB.
 *
 * @param       pibAttribute - The attribute identifier
 * @param       pValue - pointer to the attribute value
 *
 * @return      The status of the request
 */
ApiMac_status_t ApiMac_mlmeGetReqUint8(
        ApiMac_attribute_uint8_t pibAttribute,
        uint8_t *pValue)
{
    return getReq(pibAttribute, pValue, 1);
}

/*!
 * @brief       This direct execute function retrieves an attribute value from
 *              the MAC PIB.
 *
 * @param       pibAttribute - The attribute identifier
 * @param       pValue - pointer to the attribute value
 *
 * @return      The status of the request
 */
ApiMac_status_t ApiMac_mlmeGetReqUint16(
        ApiMac_attribute_uint16_t pibAttribute,
        uint16_t *pValue)
{
    return getReq(pibAttribute, pValue, 2);
}

/*!
 * @brief       This direct execute function retrieves an attribute value from
 *              the MAC PIB.
 *
 * @param       pibAttribute - The attribute identifier
 * @param       pValue - pointer to the attribute value
 *
 * @return      The status of the request
 */
ApiMac_status_t ApiMac_mlmeGetReqUint32(
        ApiMac_attribute_uint32_t pibAttribute,
        uint32_t *pValue)
{
    return getReq(pibAttribute, pValue, 4);
}

/*!
 * @brief       This direct execute function retrieves an attribute value from
 *              the MAC PIB.
 *
 * @param       pibAttribute - The attribute identifier
 * @param       pValue - pointer to the attribute value
 *
 * @return      The status of the request
 */
ApiMac_status_t ApiMac_mlmeGetReqArray(
        ApiMac_attribute_array_t pibAttribute,
        uint8_t *pValue)
{
    return getReq(pibAttribute, pValue, 16);
}

/*!
 * @brief   Process MAC_SET_REQ command issued by host
 *
 * @param   pMpb - pointer to incoming message parameter block
 */
static ApiMac_status_t setReq(uint8_t pibAttribute, const void *data, uint8_t len)
{
    uint8_t buff[sizeof(MtPkt_setReq_t) + 16];
    uint8_t *pBuf = buff;
    memset(pBuf, 0, sizeof(buff));

    if (len > 16)
        return ApiMac_status_lengthError;

    *pBuf++ = pibAttribute;
    memcpy(pBuf, data, len);

    /* Send request */
    ApiMac_status_t status = syncRequestWithStatusReply(
            MT_MAC_SET_REQ, sizeof(buff), buff);

    return status;
}

ApiMac_status_t ApiMac_mlmeSetReqBool(ApiMac_attribute_bool_t pibAttribute,
                                      bool value)
{
    uint8_t valueUint8 = (uint8_t)value;
    return setReq(pibAttribute, &valueUint8, 1);
}

ApiMac_status_t ApiMac_mlmeSetReqUint8(ApiMac_attribute_uint8_t pibAttribute,
                                       uint8_t value)
{
    return setReq(pibAttribute, &value, 1);
}

ApiMac_status_t ApiMac_mlmeSetReqUint16(ApiMac_attribute_uint16_t pibAttribute,
                                        uint16_t value)
{
    uint8_t buff[2];
    Util_bufferUint16(buff, value);
    return setReq(pibAttribute, buff, 2);
}

ApiMac_status_t ApiMac_mlmeSetReqUint32(ApiMac_attribute_uint32_t pibAttribute,
                                        uint32_t value)
{
    uint8_t buff[4];
    Util_bufferUint32(buff, value);
    return setReq(pibAttribute, buff, 4);
}

ApiMac_status_t ApiMac_mlmeSetReqArray(ApiMac_attribute_array_t pibAttribute,
                                       uint8_t *pValue)
{
    return setReq(pibAttribute, pValue, 16);
}

//******************************************************************************
// Public API Callback Functions
// *****************************************************************************/
/*!
 * Process MAC_ASYNC_IND callback issued by MAC
 *
 * Public function that is defined in mt_mac.h
 */
static void macAsyncInd(const Mt_mpb_t *pMpb)
{
    /* Hand over to shared DATA_IND processor */
    dataInd(MT_MAC_ASYNC_IND, pMpb);
}

/*!
 * Process MAC_DATA_CNF callback issued by MAC
 *
 * Public function that is defined in mt_mac.h
 */
static void macDataCnf(const Mt_mpb_t *pMpb)
{
    if(pMpb->length != sizeof(MtPkt_dataCnf_t))
        return;

    ApiMac_mcpsDataCnf_t dataCnf;
    uint8_t *pBuf = pMpb->pData;

    /* Status */
    dataCnf.status = *pBuf++;

    /* MSDU handle */
    dataCnf.msduHandle = *pBuf++;

    /* Timestamp (backoffs) */
    dataCnf.timestamp = Util_parseUint32(pBuf);
    pBuf += 4;

    /* Timestamp2 (MAC units) */
    dataCnf.timestamp2 = Util_parseUint16(pBuf);
    pBuf += 2;

    /* Retries */
    dataCnf.retries = *pBuf++;

    /* Link quality */
    dataCnf.mpduLinkQuality = *pBuf++;

    /* Correlation */
    dataCnf.correlation = *pBuf++;

    /* RSSI */
    dataCnf.rssi = *pBuf++;

    /* Frame counter */
    dataCnf.frameCntr = Util_parseUint32(pBuf);

    if (macCallbacks.pDataCnfCb != NULL)
        macCallbacks.pDataCnfCb(&dataCnf);
}

/*!
 * Process MAC_DATA_IND callback issued by MAC
 *
 * Public function that is defined in mt_mac.h
 */
static void macDataInd(const Mt_mpb_t *pMpb)
{
    /* Hand over to shared DATA_IND processor */
    dataInd(MT_MAC_DATA_IND, pMpb);
}

//******************************************************************************
// Local Callback Utility Functions
// *****************************************************************************/
/*!
 * @brief   Process ASYNC/DATA_IND callback issued by MAC
 *
 * @param   indType - MT_MAC_DATA_IND or MT_MAC_ASYNC_IND
 * @param   pInd - pointer to incoming callback data structure
 */
static void dataInd(uint8_t indType, const Mt_mpb_t *pMpb)
{
    ApiMac_mcpsDataInd_t dataInd;
    uint8_t *pBuf = pMpb->pData;
    uint16_t dLen;

    if (indType == MT_MAC_DATA_IND)
    {
        /* Fixed length of data indication */
        dLen = MT_MAC_LEN_DATA_IND;
    }
    else
    {
        /* Fixed length of async indication */
        dLen = MT_MAC_LEN_ASYNC_IND;
    }

    /* Source Address Mode */
    dataInd.srcAddr.addrMode = (ApiMac_addrType_t)*pBuf++;

    /* Source device address */
    macSbaToAdr(&dataInd.srcAddr, pBuf);
    pBuf += APIMAC_SADDR_EXT_LEN;

    /* Destination Address Mode */
    dataInd.dstAddr.addrMode = (ApiMac_addrType_t)*pBuf++;

    /* Destination device address */
    macSbaToAdr(&dataInd.dstAddr, pBuf);
    pBuf += APIMAC_SADDR_EXT_LEN;

    /* Timestamp (backoffs) */
    dataInd.timestamp = Util_parseUint32(pBuf);
    pBuf += 4;

    /* Timestamp2 (MAC units) */
    dataInd.timestamp2 = Util_parseUint16(pBuf);
    pBuf += 2;

    /* Source PAN ID */
    dataInd.srcPanId = Util_parseUint16(pBuf);
    pBuf += 2;

    /* Destination PAN ID */
    dataInd.dstPanId = Util_parseUint16(pBuf);
    pBuf += 2;

    /* Link quality */
    dataInd.mpduLinkQuality = *pBuf++;

    /* Correlation */
    dataInd.correlation = *pBuf++;

    /* RSSI */
    dataInd.rssi = *pBuf++;

    /* Data sequence number */
    dataInd.dsn = *pBuf++;

    /* Security */
    macSbaToSec(&dataInd.sec, pBuf);
    pBuf += MT_MAC_LEN_SECINFO;

    /* Frame counter */
    dataInd.frameCntr = Util_parseUint32(pBuf);
    pBuf += 4;

    dataInd.fhFrameType = ApiMac_fhFrameType_invalid;
    if (indType == MT_MAC_ASYNC_IND)
    {
        /* Frequency hopping frame type */
        dataInd.fhFrameType = *pBuf++;
    }

    /* Payload MDSU data length */
    dataInd.msdu.len = Util_parseUint16(pBuf);
    dLen += dataInd.msdu.len;
    pBuf += 2;

    /* Payload IE data length */
    dataInd.payloadIeLen = Util_parseUint16(pBuf);
    dLen += dataInd.payloadIeLen;
    pBuf += 2;

    /* MSDU - just put MT bufffer pointer in ApiMac structure */
    dataInd.msdu.p = pBuf;
    pBuf += dataInd.msdu.len;

    /* IE data - just put MT bufffer pointer in ApiMac structure */
    dataInd.pPayloadIE = pBuf;

    if(pMpb->length != dLen)
        return;

    if (indType == MT_MAC_DATA_IND && macCallbacks.pDataIndCb != NULL)
        macCallbacks.pDataIndCb(&dataInd);
    else if (indType == MT_MAC_ASYNC_IND && macCallbacks.pWsAsyncIndCb != NULL)
        macCallbacks.pWsAsyncIndCb(&dataInd);
}

/******************************************************************************
 Local Utility Functions
 *****************************************************************************/
static ApiMac_status_t syncRequestWithStatusReply(uint8_t cmd, uint16_t len, uint8_t *pReq)
{
    uint8_t statusUint8 = 0;
    uint16_t responseLength = 1;

    bool got_response = MT_sendSyncRequest(
            MT_SREQ_MAC, cmd, len, pReq,
            MT_SRSP_MAC, cmd, &responseLength, &statusUint8);

    ApiMac_status_t status = ApiMac_status_subSystemError;
    if (got_response && responseLength >= 1) {
        status = (ApiMac_status_t)statusUint8;
    }

    return status;
}

/*!
 * @brief   Copy extended address, return ptr to next destination byte
 *
 * @param   pDst - Pointer to destination address byte array
 * @param   pSrc - Pointer to source address byte array
 *
 * @return  pDst - Pointer to next location in destination byte array
 */
static uint8_t* copyExtAdr(uint8_t *pDst, uint8_t *pSrc)
{
    memcpy(pDst, pSrc, APIMAC_SADDR_EXT_LEN);
    pDst += APIMAC_SADDR_EXT_LEN;

    /* Ptr to next location in destination byte array */
    return(pDst);
}

/*!
 * @brief   Copy an ApiMac address structure to serial byte array
 *
 * @param   pSrc - Pointer to source address structure
 * @param   pDst - Pointer to destination serial byte array
 *
 * @return  pDst - Pointer to next location in destination byte array
 */
static uint8_t* macAdrToSba(uint8_t *pDst, ApiMac_sAddr_t *pSrc)
{
    *pDst++ = pSrc->addrMode;

    if(pSrc->addrMode == ApiMac_addrType_short)
    {
        memset(pDst, 0, APIMAC_SADDR_EXT_LEN);
        (void)Util_bufferUint16(pDst, pSrc->addr.shortAddr);
        pDst += APIMAC_SADDR_EXT_LEN;
    }
    else
    {
        pDst = copyExtAdr(pDst, pSrc->addr.extAddr);
    }

    return(pDst);
}

/*!
 * @brief   Copy an address from an serial byte array to an ApiMac struct
 *          The addrMode in pDst must already be set.
 *
 * @param   pDst - Pointer to destination address structure
 * @param   pSrc - Pointer to source address byte array
 */
static void macSbaToAdr(ApiMac_sAddr_t *pDst, uint8_t *pSrc)
{
    if(pDst->addrMode == ApiMac_addrType_short)
    {
        pDst->addr.shortAddr = Util_parseUint16(pSrc);
    }
    else if(pDst->addrMode == ApiMac_addrType_extended)
    {
        (void)copyExtAdr(pDst->addr.extAddr, pSrc);
    }
}

/*!
 * @brief   Copy security parameters from ApiMac struct to serial byte array
 *
 * @param   pSba - Pointer to serial byte array
 * @param   pSec - Pointer to security structure
 */
static void macSecToSba(uint8_t *pSba, ApiMac_sec_t *pSec)
{
    /* Key source */
    memcpy(pSba, pSec->keySource, APIMAC_KEY_SOURCE_MAX_LEN);
    pSba += APIMAC_KEY_SOURCE_MAX_LEN;

    /* Security level */
    *pSba++ = pSec->securityLevel;

    /* Key identifier mode */
    *pSba++ = pSec->keyIdMode;

    /* Key index */
    *pSba = pSec->keyIndex;
}

/*!
 * @brief   Copy security parameters from serial byte array to ApiMac struct
 *
 * @param   pSec - Pointer to security structure
 * @param   pSba - Pointer to serial byte array
 */
static void macSbaToSec(ApiMac_sec_t *pSec, uint8_t *pSba)
{
    /* Key source */
    memcpy(pSec->keySource, pSba, APIMAC_KEY_SOURCE_MAX_LEN);
    pSba += APIMAC_KEY_SOURCE_MAX_LEN;

    /* Security level */
    pSec->securityLevel = *pSba++;

    /* Key identifier mode */
    pSec->keyIdMode = *pSba++;

    /* Key index */
    pSec->keyIndex = *pSba;
}

/*!
 * @brief   Parse txOptions bits to ApiMac txOptions structure
 *
 * @param   optBits - MT command TX options bit mask
 * @param   txOptions - ptr to output TX options structure
 */
static uint8_t *bufferTxOptBits(uint8_t *pBuf, const ApiMac_txOptions_t *txOptions)
{
    uint8_t optBits = 0;
    if (txOptions->ack)
        optBits |= MT_MAC_TXOPTION_ACK;
    /* MT_MAC_TXOPTION_GTS Not used */

    if (txOptions->indirect)
        optBits |= MT_MAC_TXOPTION_INDIRECT;

    if (txOptions->pendingBit)
        optBits |= MT_MAC_TXOPTION_PEND_BIT;

    if (txOptions->noRetransmits)
        optBits |= MT_MAC_TXOPTION_NO_RETRANS;

    if (txOptions->noConfirm)
        optBits |= MT_MAC_TXOPTION_NO_CNF;

    if (txOptions->useAltBE)
        optBits |= MT_MAC_TXOPTION_ALT_BE;

    if (txOptions->usePowerAndChannel)
        optBits |= MT_MAC_TXOPTION_PWR_CHAN;

    *pBuf++ = optBits;

    return pBuf;
}
