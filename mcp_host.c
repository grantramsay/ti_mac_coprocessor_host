// TI MAC Co-Processor host
#include "mcp_host.h"

#include "mt.h"
#include "npi_frame.h"
#include "npi_rxbuf.h"
#include "npi_tl.h"
#include "osal_port.h"

static void incomingFrameCallback(uint8_t frameSize, uint8_t *pFrame, NPIMSG_Type msgType);
static void transportTxCallback(int size);
static void transportRxCallback(int size);

static int32_t allocated_memory_block_count = 0;
static uint8_t *lastQueuedTxMsg = NULL;

void mcp_host_init(void)
{
    NPITL_initTL(transportTxCallback, transportRxCallback, NULL);
    NPIFrame_initialize(incomingFrameCallback);
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

uint8_t OsalPort_msgSend(uint8_t destinationTask, uint8_t *pMsg )
{
    (void)destinationTask;

    if (NPITL_checkNpiBusy()) {
        MAP_ICall_freeMsg(pMsg);
        return OsalPort_FAILURE;
    }

    // NPIFrame_frameMsg always deallocates pMsg
    NPIMSG_msg_t *npiMsg = NPIFrame_frameMsg(pMsg);
    if (npiMsg == NULL)
        return OsalPort_FAILURE;

    // npiMsg->pBuf is deallocated after TX complete.
    lastQueuedTxMsg = npiMsg->pBuf;

    NPITL_writeTL(npiMsg->pBuf, npiMsg->pBufSize);
    MAP_ICall_free(npiMsg);

    return OsalPort_SUCCESS;
}

uint32_t OsalPort_enterCS(void)
{
    return 0;
}

void OsalPort_leaveCS(uint32_t key)
{
    (void)key;
}

int32_t OsalPort_allocatedMemoryBlockCount(void)
{
    return allocated_memory_block_count;
}

static void incomingFrameCallback(uint8_t frameSize, uint8_t *pFrame, NPIMSG_Type msgType) {
    (void)frameSize;
    (void)msgType;
    MT_processIncoming(pFrame);
    MAP_ICall_freeMsg(pFrame);
}

static void transportTxCallback(int size)
{
    (void)size;
    if (lastQueuedTxMsg != NULL)
    {
        //Deallocate most recent message being transmitted.
        MAP_ICall_freeMsg(lastQueuedTxMsg);
        lastQueuedTxMsg = NULL;
    }
}

static void transportRxCallback(int size)
{
    NPIRxBuf_Read(size);
    NPIFrame_collectFrameData();
}