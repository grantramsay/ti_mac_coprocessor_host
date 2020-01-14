/******************************************************************************
 @file  osal_port.h

 @brief This API maps ICALL and OSAL API's used by the MAC to TIRTOS API's

 Group: WCS, LPC, BTS
 Target Device: cc13x2_26x2

 ******************************************************************************
 
 Copyright (c) 2004-2019, Texas Instruments Incorporated
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

#ifndef OsalPort_H
#define OsalPort_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

/*********************************************************************
 * MACROS
 */
#define OsalPort_MSG_NEXT(pMsg)      ((OsalPort_MsgHdr *) (pMsg) - 1)->next
#define OsalPort_MSG_Q_INIT(pQ)      *(pQ) = NULL
#define OsalPort_MSG_Q_EMPTY(pQ)     (*(pQ) == NULL)
#define OsalPort_MSG_Q_HEAD(pQ)      (*(pQ))
#define OsalPort_MSG_LEN(pMsg)      ((OsalPort_MsgHdr *) (pMsg) - 1)->len
#define OsalPort_MSG_ID(pMsg)      ((OsalPort_MsgHdr *) (pMsg) - 1)->dest_id

#define OsalPort_OFFSET_OF(type, member) ((uint32) &(((type *) 0)->member))

/*********************************************************************
 * CONSTANTS
 */

#define OsalPort_SYS_EVENT_MSG            0x8000

/*** Generic Status Return Values ***/
#define OsalPort_SUCCESS                   0x00
#define OsalPort_FAILURE                   0x01
#define OsalPort_INVALIDPARAMETER          0x02
#define OsalPort_INVALID_TASK              0x03
#define OsalPort_MSG_BUFFER_NOT_AVAIL      0x04
#define OsalPort_INVALID_MSG_POINTER       0x05
#define OsalPort_INVALID_EVENT_ID          0x06
#define OsalPort_INVALID_INTERRUPT_ID      0x07
#define OsalPort_NO_TIMER_AVAIL            0x08
#define OsalPort_NV_ITEM_UNINIT            0x09
#define OsalPort_NV_OPER_FAILED            0x0A
#define OsalPort_INVALID_MEM_SIZE          0x0B
#define OsalPort_NV_BAD_ITEM_LEN           0x0C

#define OsalPort_TASK_NO_TASK              0xFF

#define OsalPort_PWR_CONSERVE 0
#define OsalPort_PWR_HOLD     1

/*********************************************************************
 * TYPEDEFS
 */
typedef struct
{
  void   *next;

  /* OSAL port to TI-RTOS requires compatibility with ROM
   * code compiled with USE_ICALL compile flag.  */
  uint32_t  reserved;

  uint16_t len;
  uint8_t  dest_id;
} OsalPort_MsgHdr;

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * FUNCTIONS
 */

/*********************************************************************
 * @fn      OsalPort_msgAllocate
 *
 * @brief
 *
 *    This function is called by a task to allocate a message buffer
 *    into which the task will encode the particular message it wishes
 *    to send.  This common buffer scheme is used to strictly limit the
 *    creation of message buffers within the system due to RAM size
 *    limitations on the microprocessor.   Note that all message buffers
 *    are a fixed size (at least initially).  The parameter len is kept
 *    in case a message pool with varying fixed message sizes is later
 *    created (for example, a pool of message buffers of size LARGE,
 *    MEDIUM and SMALL could be maintained and allocated based on request
 *    from the tasks).
 *
 *
 * @param   uint8_t len  - wanted buffer length
 *
 *
 * @return  pointer to allocated buffer or NULL if allocation failed.
 */
extern uint8_t * OsalPort_msgAllocate(uint16_t len );

/*********************************************************************
 * @fn      OsalPort_msgDeallocate
 *
 * @brief
 *
 *    This function is used to deallocate a message buffer. This function
 *    is called by a task (or processing element) after it has finished
 *    processing a received message.
 *
 *
 * @param   uint8_t *pMsg - pointer to new message buffer
 *
 * @return  SUCCESS, INVALID_MSG_POINTER
 */
extern uint8_t OsalPort_msgDeallocate( uint8_t *pMsg );

/*********************************************************************
 * @fn      OsalPort_msgSend
 *
 * @brief
 *
 *    This function is called by a task to send a command message to
 *    another task or processing element.  The sending_task field must
 *    refer to a valid task, since the task ID will be used
 *    for the response message.  This function will also set a message
 *    ready event in the destination tasks event list.
 *
 *
 * @param   uint8_t *pMsg - pointer to new message buffer
 *
 * @return  SUCCESS, INVALID_TASK, INVALID_MSG_POINTER
 */
extern uint8_t OsalPort_msgSend( uint8_t *pMsg );

/*********************************************************************
 * @fn      OsalPort_malloc
 *
 * @brief
 *
 *   Allocates memory from the heap. Currently maps directly to system malloc.
 *
 * @param   size - size of allocation
 *
 * @return  pointer o allocated memory
 */
void* OsalPort_malloc(uint32_t size);

/*********************************************************************
 * @fn      OsalPort_malloc
 *
 * @brief
 *
 *   Frees memory allocated in the heap. Currently maps directly to system free
 *
 * @param   size - size of allocation
 *
 * @return  pointer o allocated memory
 */
void OsalPort_free(void* buf);

int32_t OsalPort_allocatedMemoryBlockCount(void);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* OsalPort_H */
