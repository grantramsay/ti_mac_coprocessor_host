// TI MAC Co-Processor host

#ifndef MCP_HOST_H
#define MCP_HOST_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

typedef void (*mcp_host_tx_data_callback_t)(const void *data, uint16_t len);

void mcp_host_init(mcp_host_tx_data_callback_t callback);

// Must be called on same thread as other processing
void mcp_host_receive_data(const void *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif
