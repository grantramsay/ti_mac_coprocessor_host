TI 15.4-Stack Coprocessor Host
============================

Simple host implementation for the TI Coprocessor, based on the [TI Coprocessor device](https://github.com/leoniloris/TI-15.4-Coprocessor./tree/master/Application/CoP).

See posix_coprocessor_host.c for examples.

The main interfaces are mac/api_mac.h and mcp_host.h (MCP -> MAC Co-Processor), mt/mt_sys.h and mt/mt_util.h may also be useful.

Currently only handles a few basic commands to transmit/receive radio messages.
Feel free to add additional message handling.

# Design Goals
To be as OS independent as possible, purely processing data.

This fell through a little with wanting to handle synchronous messaging via threading (as below), although the threading is abstracted through a simple OSAL.

# Implementation Details
## Threading
To handle synchronous requests and match the MAC API a processing thread is used.

For example `status = ApiMac_mlmeSetReqUint16(ApiMac_attribute_panId, &pan_id);` must send a message and receive a response synchronously to match the MAC API.
When a function like this is called it waits for a response or timeout and sets the returned status accordingly.

However when an asynchronous message requiring a callback is received (e.g. `MT_MAC_DATA_IND`) we want the callback returned on the thread that is accessing the MAC API.
When one of these messages is received it is queued for processing, `processing_required_callback` (mcp_host.h) is called to inform the application that processing is required and `mcp_host_process_callbacks` should be called to process these (alternatively `mcp_host_process_callbacks` may be polled).
