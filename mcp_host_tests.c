#include <assert.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>

#include "api_mac.h"
#include "mcp_host.h"
#include "mt.h"
#include "osal_port.h"

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

#define MT_SOF 0xFE
#define MT_SRSP_MAC ((uint8_t)MTRPC_CMD_SREQ | (uint8_t)MTRPC_SYS_MAC)

static void Mac_DataInd(ApiMac_mcpsDataInd_t *pInd);
static void tx_data_callback(const uint8_t *data, uint16_t len);
static void insert_mt_message(uint8_t *msg, uint16_t len);
static uint8_t calcFCS(uint8_t *pMsg, uint16_t len);

/*!
 API MAC Callback table
 */
static ApiMac_callbacks_t macCallbacks = {
    /*! Associate Indication callback */
    NULL,
    /*! Associate Confirmation callback */
    NULL,
    /*! Disassociate Indication callback */
    NULL,
    /*! Disassociate Confirmation callback */
    NULL,
    /*! Beacon Notify Indication callback */
    NULL,
    /*! Orphan Indication callback */
    NULL,
    /*! Scan Confirmation callback */
    NULL,
    /*! Start Confirmation callback */
    NULL,
    /*! Sync Loss Indication callback */
    NULL,
    /*! Poll Confirm callback */
    NULL,
    /*! Comm Status Indication callback */
    NULL,
    /*! Poll Indication Callback */
    NULL,
    /*! Data Confirmation callback */
    NULL,
    /*! Data Indication callback */
    Mac_DataInd,
    /*! Purge Confirm callback */
    NULL,
    /*! WiSUN Async Indication callback */
    NULL,
    /*! WiSUN Async Confirmation callback */
    NULL,
    /*! Unprocessed message callback */
    NULL
};

int main(void)
{
    mcp_host_init(tx_data_callback);
//    ApiMac_registerCallbacks(&macCallbacks);

    uint8_t dummy_rx_message[] = { MT_SOF, 0, MT_SRSP_MAC, MT_MAC_INIT_REQ };
    insert_mt_message(dummy_rx_message, sizeof(dummy_rx_message));

    uint16_t pan_id = 0x1234;
    uint16_t msdu_handle = 0x12;
    uint8_t txData[] = { 0x12, 0x34 };
    uint16_t len = sizeof(txData);
    ApiMac_mcpsDataReq_t dataReq = {
        .dstAddr = {
            .addr = { .shortAddr = 0xFFFF },
            .addrMode = ApiMac_addrType_short
        },
        .dstPanId = pan_id,
        .srcAddrMode = ApiMac_addrType_short,
        .msduHandle = msdu_handle,
        .txOptions = { .ack = true },
        .channel = 0, // Unused without txOption usePowerAndChannel
        .power = 0, // Unused without txOption usePowerAndChannel
        .pIEList = NULL,
        .payloadIELen = 0,
        .fhProtoDispatch = ApiMac_fhDispatchType_none,
        .includeFhIEs = 0,
        .msdu = { .p = txData, .len = len },
        .sec = {
            .keySource = { 0, 0, 0, 0, 0, 0, 0, 0 },
            .securityLevel = 0,
            .keyIdMode = 0,
            .keyIndex = 0
        },
        .gpOffset = 0,
        .gpDuration = 0
    };
    ApiMac_status_t status = ApiMac_mcpsDataReq(&dataReq);

    printf("Allocated memory block count: %" PRId32 "\n", OsalPort_allocatedMemoryBlockCount());

    return EXIT_SUCCESS;
}

static void Mac_DataInd(ApiMac_mcpsDataInd_t *pInd)
{
    (void)pInd;
}

static void tx_data_callback(const uint8_t *data, uint16_t len)
{
    printf("TX:");
    for (uint16_t i = 0; i < len; i++)
        printf(" %02X", data[i]);
    printf("\n");
}

static void insert_mt_message(uint8_t *msg, uint16_t len)
{
    uint8_t buff[len + 1];
    memcpy(buff, msg, len);
    buff[len] = calcFCS(buff + 1, len - 1);
    len++;
    mcp_host_receive_data(buff, len);
}

static uint8_t calcFCS(uint8_t *pMsg, uint16_t len)
{
    uint8_t result = 0;
    while (len--)
        result ^= *pMsg++;
    return result;
}
