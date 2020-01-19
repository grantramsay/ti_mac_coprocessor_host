// TI MAC Co-Processor host

#ifndef MCP_HOST_H
#define MCP_HOST_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

typedef void (*mcp_host_tx_data_callback_t)(const void *data, uint16_t len);
typedef int (*mcp_host_rx_data_callback_t)(void *data, uint16_t *len, int wait_time_us);

void mcp_host_init(mcp_host_tx_data_callback_t tx_data_callback,
                   mcp_host_rx_data_callback_t rx_data_callback);

void mcp_host_deinit(void);

void mcp_host_update(void);

#ifdef __cplusplus
}
#endif

#endif
