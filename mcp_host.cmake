set(MCP_HOST_INCLUDE_DIRS
        .
        2_4g
        combo
        mac
        mt
        npi/src/inc
        osal_port
        subg)

set(MCP_HOST_SOURCE_FILES
        2_4g/ti_154stack_features.h
        mcp_host.c
        combo/ti_154stack_features.h
        mac/api_mac.h
        mac/mac_util.c
        mac/mac_util.h
        mt/mt.c
        mt/mt.h
        mt/mt_ext.c
        mt/mt_ext.h
        mt/mt_mac.c
        mt/mt_mac.h
        mt/mt_pkt.h
        mt/mt_rpc.h
#        mt/mt_sys.c
        mt/mt_sys.h
        mt/mt_util.c
        mt/mt_util.h
        npi/src/inc/npi_client.h
        npi/src/inc/npi_config.h
        npi/src/inc/npi_data.h
        npi/src/inc/npi_frame.h
        npi/src/inc/npi_rxbuf.h
#        npi/src/inc/npi_task.h
        npi/src/inc/npi_tl.h
#        npi/src/inc/npi_tl_spi.h
#        npi/src/inc/npi_tl_uart.h
#        npi/src/npi_client_mt.c
        npi/src/npi_frame_mt.c
#        npi/src/npi_rxbuf.c
#        npi/src/npi_task.c
#        npi/src/npi_tl.c
#        npi/src/npi_tl_spi.c
#        npi/src/npi_tl_uart.c
#        osal_port/api_mac.c
        osal_port/osal_port.h
        subg/ti_154stack_features.h
#        main.c
#        mcp.c
        mcp.h)
