// TI MAC Co-Processor host
#include "mcp_host.h"

#include "mt.h"
#include "npi_frame.h"
#include "osal_port.h"

static void incomingFrameCallback(uint8_t frameSize, uint8_t *pFrame, NPIMSG_Type msgType);
static int process_rx_data(int wait_time_us);

static uint8_t *async_msg_queue = NULL;
static int32_t allocated_memory_block_count = 0;
static mcp_host_tx_data_callback_t tx_data_callback = NULL;
static mcp_host_rx_data_callback_t rx_data_callback = NULL;

void mcp_host_init(mcp_host_tx_data_callback_t tx_data_cb,
                   mcp_host_rx_data_callback_t rx_data_cb)
{
    tx_data_callback = tx_data_cb;
    rx_data_callback = rx_data_cb;
    MT_init(process_rx_data);
    NPIFrame_initialize(incomingFrameCallback);

    OsalPort_MSG_Q_INIT(&async_msg_queue);
}

void mcp_host_deinit(void)
{
    // Clear out message queue
    while (!OsalPort_MSG_Q_EMPTY(&async_msg_queue)) {
        uint8_t *pMsg = OsalPort_MSG_Q_HEAD(&async_msg_queue);
        async_msg_queue = OsalPort_MSG_NEXT(async_msg_queue);
        OsalPort_msgDeallocate(pMsg);
    }
}

void mcp_host_update(void)
{
    process_rx_data(0);

    // Process any pending async messages
    while (!OsalPort_MSG_Q_EMPTY(&async_msg_queue)) {
        uint8_t *pMsg = OsalPort_MSG_Q_HEAD(&async_msg_queue);
        async_msg_queue = OsalPort_MSG_NEXT(async_msg_queue);
        MT_processIncoming(pMsg);
        OsalPort_msgDeallocate(pMsg);
    }
}

void* OsalPort_malloc(uint32_t size)
{
    allocated_memory_block_count++;
    return malloc(size);
}

void OsalPort_free(void* buf)
{
    allocated_memory_block_count--;
    free(buf);
}

uint8_t * OsalPort_msgAllocate(uint16_t len )
{
    uint8_t *pMsg = NULL;
    OsalPort_MsgHdr* pHdr;

    if ( len == 0 )
        return ( NULL );

    pHdr = (OsalPort_MsgHdr*) OsalPort_malloc( len + sizeof( OsalPort_MsgHdr ) );

    if ( pHdr )
    {
        pHdr->next = NULL;
        pHdr->len = len;
        pHdr->dest_id = OsalPort_TASK_NO_TASK;

        pMsg = (uint8_t *)((uint8_t *)pHdr + sizeof( OsalPort_MsgHdr ));
    }

    return pMsg;
}

uint8_t OsalPort_msgDeallocate( uint8_t *pMsg )
{
    uint8_t *x;

    if ( pMsg == NULL )
        return ( OsalPort_INVALID_MSG_POINTER );

    // don't deallocate queued buffer
    if ( OsalPort_MSG_ID( pMsg ) != OsalPort_TASK_NO_TASK )
        return ( OsalPort_MSG_BUFFER_NOT_AVAIL );

    x = (uint8_t *)((uint8_t *)pMsg - sizeof( OsalPort_MsgHdr ));

    OsalPort_free( (void *)x );

    return ( OsalPort_SUCCESS );
}

uint8_t OsalPort_msgSend(uint8_t *pMsg)
{
    // NPIFrame_frameMsg always deallocates pMsg
    NPIMSG_msg_t *npiMsg = NPIFrame_frameMsg(pMsg);
    if (npiMsg == NULL)
        return OsalPort_FAILURE;

    if (tx_data_callback != NULL)
        tx_data_callback(npiMsg->pBuf, npiMsg->pBufSize);

    MAP_ICall_freeMsg(npiMsg->pBuf);
    MAP_ICall_free(npiMsg);

    return OsalPort_SUCCESS;
}

int32_t OsalPort_allocatedMemoryBlockCount(void)
{
    return allocated_memory_block_count;
}

static void queue_async_frame(uint8_t *pMsg)
{
    // Append to end of queue
    if (OsalPort_MSG_Q_EMPTY(&async_msg_queue)) {
        OsalPort_MSG_Q_HEAD(&async_msg_queue) = pMsg;
    } else {
        uint8_t *msg_queue = async_msg_queue;
        while (OsalPort_MSG_NEXT(msg_queue) != NULL)
            msg_queue = OsalPort_MSG_NEXT(msg_queue);
        OsalPort_MSG_NEXT(msg_queue) = pMsg;
    }
}

static void incomingFrameCallback(uint8_t frameSize, uint8_t *pFrame, NPIMSG_Type msgType) {
    (void)msgType;
    (void)frameSize;

    if (MT_isAsyncCallback(pFrame))
        queue_async_frame(pFrame);
    else {
        MT_processIncoming(pFrame);
        MAP_ICall_freeMsg(pFrame);
    }
}

static int process_rx_data(int wait_time_us)
{
    int waited_time_us = 0;
    uint8_t buf[256];
    uint16_t len;

    if (rx_data_callback == NULL)
        return wait_time_us;

    do {
        len = sizeof(buf);
        waited_time_us += rx_data_callback(buf, &len, wait_time_us - waited_time_us);
        if (len > 0)
            NPIFrame_receiveData(buf, len);
    } while (len == sizeof(buf));

    return waited_time_us;
}
