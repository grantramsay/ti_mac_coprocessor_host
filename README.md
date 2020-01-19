TI 15.4-Stack Coprocessor Host
============================

Simple host implementation for the TI Coprocessor, based on the [TI Coprocessor device](https://github.com/leoniloris/TI-15.4-Coprocessor./tree/master/Application/CoP).

See posix_coprocessor_host.c for examples.

The main interfaces are mac/api_mac.h and mcp_host.h (MCP -> MAC Co-Processor), mt/mt_sys.h and mt/mt_util.h may also be useful.

`mcp_host_update` should be called after receiving serial data (or can be polled).
The API should only be accessed from a single thread.
Additionally the API should not be accessed from the mcp_host TX/RX data callbacks.

MAC API calls are blocking when a serial ACK/response is required.
It's safe to access the MAC API from MAC async callbacks, async callbacks are always dispatched from `mcp_host_update` to avoid nested API access if async messages are received while waiting for a synchronous ACK/response.

Currently only handles a few basic commands to transmit/receive radio messages.
Feel free to add additional message handling.
