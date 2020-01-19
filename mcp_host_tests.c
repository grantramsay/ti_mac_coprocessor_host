#include <assert.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>

#include "api_mac.h"
#include "mcp_host.h"
#include "mt.h"
#include "osal_port.h"

#define MT_SOF 0xFE
#define MT_SRSP_MAC ((uint8_t)MTRPC_CMD_SREQ | (uint8_t)MTRPC_SYS_MAC)

static void Mac_DataCnf(ApiMac_mcpsDataCnf_t *pDataCnf);
static void Mac_DataInd(ApiMac_mcpsDataInd_t *pInd);
static void tx_data_callback(const void *data, uint16_t len);
static int rx_data_callback(void *data, uint16_t *len, int wait_time_us);
static void insert_mt_message(uint8_t cmd0, uint8_t cmd1, const uint8_t *msg, uint16_t len);
static uint8_t calcFCS(const uint8_t *pMsg, uint16_t len);

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
    Mac_DataCnf,
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

static uint16_t rx_data_packet_len;
static uint16_t rx_data_packet_index;
static uint8_t rx_data_buf[1024];
static int data_cnf_received;

int main(void)
{
    mcp_host_init(tx_data_callback, rx_data_callback);
    ApiMac_registerCallbacks(&macCallbacks);

    data_cnf_received = 0;
    uint8_t dataCnfBuff[] = { 0, 0x12, 0x67, 0x45, 0x23, 0x01, 0xAB, 0x89, 0x00, 0x01, 0x01, 0x7F, 0x67, 0x45, 0x23, 0x01 };
    insert_mt_message(MT_SRSP_MAC, MT_MAC_DATA_CNF, dataCnfBuff, sizeof(dataCnfBuff));
    insert_mt_message(MT_SRSP_MAC, MT_MAC_DATA_CNF, dataCnfBuff, sizeof(dataCnfBuff));
    insert_mt_message(MT_SRSP_MAC, MT_MAC_DATA_CNF, dataCnfBuff, sizeof(dataCnfBuff));

    // Data conf messages should be processed as async callback
    assert(data_cnf_received == 3);

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
    ApiMac_mcpsDataReq(&dataReq);

    printf("Allocated memory block count: %" PRId32 "\n", OsalPort_allocatedMemoryBlockCount());

    mcp_host_deinit();

    return EXIT_SUCCESS;
}

static void Mac_DataInd(ApiMac_mcpsDataInd_t *pInd)
{
    (void)pInd;
}

static void Mac_DataCnf(ApiMac_mcpsDataCnf_t *pDataCnf)
{
    (void)pDataCnf;
    data_cnf_received++;
}

static void tx_data_callback(const void *data, uint16_t len)
{
    printf("TX:");
    for (uint16_t i = 0; i < len; i++)
        printf(" %02X", ((uint8_t*)data)[i]);
    printf("\n");
}

static int rx_data_callback(void *data, uint16_t *len, int wait_time_us)
{
    *len = MIN(rx_data_packet_len, *len);
    memcpy(data, rx_data_buf + rx_data_packet_index, *len);
    rx_data_packet_len -= *len;
    rx_data_packet_index += *len;
    return wait_time_us;
}

static void insert_mt_message(uint8_t cmd0, uint8_t cmd1, const uint8_t *msg, uint16_t len)
{
    rx_data_packet_len = len + MTRPC_FRAME_HDR_SZ + 2;
    rx_data_packet_index = 0;

    assert(len + 4u < sizeof(rx_data_buf));

    rx_data_buf[0] = MT_SOF;
    rx_data_buf[1] = len;
    rx_data_buf[2] = cmd0;
    rx_data_buf[3] = cmd1;
    memcpy(rx_data_buf + 4, msg, len);
    rx_data_buf[rx_data_packet_len - 1] = calcFCS(rx_data_buf + 1, len + MTRPC_FRAME_HDR_SZ);

    mcp_host_update();
}

static uint8_t calcFCS(const uint8_t *pMsg, uint16_t len)
{
    uint8_t result = 0;
    while (len--)
        result ^= *pMsg++;
    return result;
}
