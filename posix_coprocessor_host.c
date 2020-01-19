#include <assert.h>
#include <stdio.h>
#include <stdlib.h>

#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <pthread.h>
#include <signal.h>
#include <sys/time.h>

#include "api_mac.h"
#include "mcp_host.h"
#include "mt_sys.h"

#define ASSERT_MAC_SUCCESS(MacCall) (assert(MacCall == ApiMac_status_success))

static void mt_sys_reset_ind_callback(uint8_t reason);
static void mac_data_ind_callback(ApiMac_mcpsDataInd_t *pDataInd);
static void mac_data_cnf_callback(ApiMac_mcpsDataCnf_t *pDataCnf);
static void send_radio_tx(void *data, uint16_t len);
static void tx_data_callback(const void *data, uint16_t len);
static int rx_data_callback(void *data, uint16_t *len, int wait_time_us);
static void *serial_read_thread_function(void *ptr);
static void sigint_handler(int sig);
static int serial_set_interface_attribs(int fd, int speed, int parity);

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
        mac_data_cnf_callback,
        /*! Data Indication callback */
        mac_data_ind_callback,
        /*! Purge Confirm callback */
        NULL,
        /*! WiSUN Async Indication callback */
        NULL,
        /*! WiSUN Async Confirmation callback */
        NULL,
        /*! Unprocessed message callback */
        NULL
};

static int serial_fd;
static uint16_t pan_id = 0x7455;
static volatile bool finished = false;

static pthread_mutex_t rx_data_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t rx_data_cond = PTHREAD_COND_INITIALIZER;
static uint16_t rx_data_packet_len;
static uint16_t rx_data_packet_index;
static uint8_t rx_data_buf[1024];

int main(int argc, char *argv[])
{
    if (argc < 2) {
        fprintf(stderr, "Usage: %s [port e.g. /dev/ttyUSB0]\n", argv[0]);
        return EXIT_FAILURE;
    }
    char *portname = argv[1];

    pan_id = 0x7455;
    uint16_t short_address = 0x2222;

    signal(SIGINT, sigint_handler);

    serial_fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (serial_fd < 0) {
        fprintf(stderr, "error %d opening %s: %s", errno, portname, strerror (errno));
        return EXIT_FAILURE;
    }
    // set speed to 115,200 bps, 8n1 (no parity)
    serial_set_interface_attribs(serial_fd, B115200, 0);
    // set serial non blocking
    struct termios tio;
    memset(&tio, 0, sizeof tio);
    tcgetattr(serial_fd, &tio);
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;
    tcsetattr(serial_fd, TCSANOW, &tio);

    mcp_host_init(tx_data_callback, rx_data_callback);
    ApiMac_registerCallbacks(&macCallbacks);
    MtSys_registerResetIndCallback(mt_sys_reset_ind_callback);

    pthread_t serial_read_thread;
    pthread_create(&serial_read_thread, NULL, serial_read_thread_function, NULL);

    ASSERT_MAC_SUCCESS(ApiMac_mlmeResetReq(true));
    usleep(50000);

    ASSERT_MAC_SUCCESS(ApiMac_mlmeSetReqUint8(ApiMac_attribute_phyCurrentDescriptorId, 3));
    ASSERT_MAC_SUCCESS(ApiMac_mlmeSetReqUint8(ApiMac_attribute_channelPage, 9));
    ASSERT_MAC_SUCCESS(ApiMac_mlmeSetReqUint8(ApiMac_attribute_logicalChannel, 0));
    ASSERT_MAC_SUCCESS(ApiMac_mlmeSetReqBool(ApiMac_attribute_RxOnWhenIdle, true));
    ASSERT_MAC_SUCCESS(ApiMac_mlmeSetReqBool(ApiMac_attribute_promiscuousMode, false));
    ASSERT_MAC_SUCCESS(ApiMac_mlmeSetReqBool(ApiMac_attribute_securityEnabled, false));
    ASSERT_MAC_SUCCESS(ApiMac_mlmeSetReqUint16(ApiMac_attribute_panId, pan_id));
    ASSERT_MAC_SUCCESS(ApiMac_mlmeSetReqUint16(ApiMac_attribute_shortAddress, short_address));

    // CSMA backoff exponent
    ASSERT_MAC_SUCCESS(ApiMac_mlmeSetReqUint8(ApiMac_attribute_backoffExponent, 3));
    ASSERT_MAC_SUCCESS(ApiMac_mlmeSetReqUint8(ApiMac_attribute_maxBackoffExponent, 5));
    ASSERT_MAC_SUCCESS(ApiMac_mlmeSetReqUint8(ApiMac_attribute_maxFrameRetries, 5));
    ASSERT_MAC_SUCCESS(ApiMac_mlmeSetReqUint8(ApiMac_attribute_maxCsmaBackoffs, 5));

    // Disable stdin canonical mode (buffered i/o), and set to non-blocking
    memset(&tio, 0, sizeof tio);
    tcgetattr(STDIN_FILENO, &tio);
    tio.c_lflag &= ~ICANON;
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &tio);

    while (!finished) {
        usleep(1000);

        uint8_t buf[64];
        int n = read(STDIN_FILENO, buf, sizeof(buf));
        if (n > 0) {
            send_radio_tx(buf, n);

//            uint8_t tx_buff[900];
//            for (uint16_t i = 0; i < sizeof(tx_buff); i++)
//                tx_buff[i] = (uint8_t)i;
//            send_radio_tx(tx_buff, sizeof(tx_buff));
        }

        mcp_host_update();
    }

    close(serial_fd);
    pthread_join(serial_read_thread, NULL);
    pthread_cond_destroy(&rx_data_cond);
    mcp_host_deinit();

    return EXIT_SUCCESS;
}

static void mt_sys_reset_ind_callback(uint8_t reason)
{
    fprintf(stderr, "Radio device has reset: %d\n", (int)reason);
}

static void mac_data_ind_callback(ApiMac_mcpsDataInd_t *pDataInd)
{
    write(STDOUT_FILENO, pDataInd->msdu.p, pDataInd->msdu.len);
    fflush(stdout);
}

static void mac_data_cnf_callback(ApiMac_mcpsDataCnf_t *pDataCnf)
{
    switch (pDataCnf->status) {
        case ApiMac_status_success:
        case ApiMac_status_noAck:
            break;
        default:
            fprintf(stderr, "Data TX failed: %d\n", (int)pDataCnf->status);
            break;
    }
}

static void send_radio_tx(void *data, uint16_t len)
{
    static uint8_t msdu_handle = 0;
    msdu_handle++;
    if (msdu_handle == 0)
        msdu_handle++;

    ApiMac_mcpsDataReq_t dataReq = {
        .dstAddr = {
            .addr = {.shortAddr = 0xFFFF},
            .addrMode = ApiMac_addrType_short
        },
        .dstPanId = pan_id,
        .srcAddrMode = ApiMac_addrType_short,
        .msduHandle = msdu_handle,
        .txOptions = {.ack = true},
        .channel = 0, // Unused without txOption usePowerAndChannel
        .power = 0, // Unused without txOption usePowerAndChannel
        .pIEList = NULL,
        .payloadIELen = 0,
        .fhProtoDispatch = ApiMac_fhDispatchType_none,
        .includeFhIEs = 0,
        .msdu = {.p = data, .len = len},
        .sec = {
            .keySource = {0, 0, 0, 0, 0, 0, 0, 0},
            .securityLevel = 0,
            .keyIdMode = 0,
            .keyIndex = 0
        },
        .gpOffset = 0,
        .gpDuration = 0
    };
    ApiMac_status_t status = ApiMac_mcpsDataReq(&dataReq);
    assert(status == ApiMac_status_success);
}

static void tx_data_callback(const void *data, uint16_t len)
{
//    printf("TX:");
//    for (uint16_t i = 0; i < len; i++)
//        printf(" %02X", ((uint8_t*)data)[i]);
//    printf("\n");

    write(serial_fd, data, len);
}

static int rx_data_callback(void *data, uint16_t *len, int wait_time_us)
{
    pthread_mutex_lock(&rx_data_mutex);

    struct timeval t_start, t_end, t_diff;
    gettimeofday(&t_start, NULL);

    if (rx_data_packet_len == 0 && wait_time_us > 0) {
        struct timespec time_to_wait;
        time_to_wait.tv_nsec = (t_start.tv_usec + wait_time_us) * 1000LL;
        time_to_wait.tv_sec = t_start.tv_sec + time_to_wait.tv_nsec / 1000000000uLL;
        time_to_wait.tv_nsec %= 1000000000uLL;
        // Not worried about spurious wakeups
        pthread_cond_timedwait(&rx_data_cond, &rx_data_mutex, &time_to_wait);
    }
    *len = MIN(rx_data_packet_len, *len);
    if (*len > 0) {
        memcpy(data, rx_data_buf + rx_data_packet_index, *len);
        rx_data_packet_len -= *len;
        rx_data_packet_index += *len;
    }

    gettimeofday(&t_end, NULL);
    timersub(&t_end, &t_start, &t_diff);
    wait_time_us = t_diff.tv_sec * 1000000LL + t_diff.tv_usec;

    pthread_mutex_unlock(&rx_data_mutex);

    return wait_time_us;
}

static void *serial_read_thread_function(void *ptr) {
    (void)ptr;
    while (!finished) {
        usleep(1);
        // read up to 256 characters if ready to read
        uint8_t buf[256];
        int n = read(serial_fd, buf, sizeof(buf));

        if (n > 0) {
            pthread_mutex_lock(&rx_data_mutex);

            // Circular buffer would be better...
            if (rx_data_packet_len == 0)
                rx_data_packet_index = 0;
            n = MIN((unsigned)n, sizeof(rx_data_buf) - rx_data_packet_index);
            memcpy(rx_data_buf + rx_data_packet_index, buf, n);
            rx_data_packet_len += n;

            pthread_cond_signal(&rx_data_cond);
            pthread_mutex_unlock(&rx_data_mutex);

//            printf("RX:");
//            for (uint16_t i = 0; i < n; i++)
//                printf(" %02X", buf[i]);
//            printf("\n");
        }
    }
    return NULL;
}

static void sigint_handler(int sig) {
    (void)sig;
    finished = true;
}

static int serial_set_interface_attribs(int fd, int speed, int parity)
{
    struct termios tty;
    if (tcgetattr (fd, &tty) != 0)
    {
        fprintf(stderr, "error %d from tcgetattr", errno);
        return -1;
    }

    cfsetospeed (&tty, speed);
    cfsetispeed (&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    tty.c_iflag &= ~IGNBRK;         // disable break processing
    tty.c_lflag = 0;                // no signaling chars, no echo,
    // no canonical processing
    tty.c_oflag = 0;                // no remapping, no delays
    tty.c_cc[VMIN]  = 0;            // read doesn't block
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    tty.c_iflag &= ~(ICRNL | ICANON | IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
    // enable reading
    tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    tty.c_cflag |= parity;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr (fd, TCSANOW, &tty) != 0)
    {
        fprintf(stderr, "error %d from tcsetattr", errno);
        return -1;
    }
    return 0;
}
