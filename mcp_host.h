// TI MAC Co-Processor host

#ifndef MCP_HOST_H
#define MCP_HOST_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

typedef void (*mcp_host_tx_data_callback_t)(const void *data, uint16_t len);
typedef void (*mcp_host_processing_required_callback_t)(void);

void mcp_host_init(mcp_host_tx_data_callback_t tx_data_callback,
                   mcp_host_processing_required_callback_t processing_required_callback);

void mcp_host_deinit(void);

uint16_t mcp_host_receive_data(const void *data, uint16_t len);

// Must be called on same thread as MAC API
void mcp_host_process_callbacks(void);

#ifdef __cplusplus
}
#endif

#endif
