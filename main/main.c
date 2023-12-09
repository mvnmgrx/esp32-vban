/* Ethernet Basic Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#define VBAN_16BIT_PCM

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_netif.h"
#include "esp_eth.h"
#include "esp_event.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/i2s_std.h"
#include "sdkconfig.h"
#include "esp_task_wdt.h"

#include <sys/param.h>
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
//#include "protocol_examples_common.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
//#include "addr_from_stdin.h"

#include "vban/vban_frame.h"
#include "vban/vban_ringbuf.h"

//#if defined(CONFIG_EXAMPLE_IPV4)
//#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV4_ADDR
//#elif defined(CONFIG_EXAMPLE_IPV6)
//#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV6_ADDR
//#else//
//#define HOST_IP_ADDR ""
//#endif

typedef struct COUNTERS {
    unsigned int uiFrameOk;
    unsigned int uiI2SBytesWritten;
    unsigned int uiFrameTooLong;
    unsigned int uiFrameTooShort;
    unsigned int uiInvalidPreamble;
    unsigned int uiUnexpectedSubproto;
    unsigned int uiUnexpectedCodec;
    unsigned int uiUnsupportedBitRes;
    unsigned int uiInvalidNumSamples;
    unsigned int uiInvalidNumChannels;
    unsigned int uiBrBit3NotZero;
    unsigned int uiBufferOverflow;
    unsigned int uiBufferUnderflow;
} COUNTERS_T;

#define PORT 6980

#if CONFIG_ETH_USE_SPI_ETHERNET
#include "driver/spi_master.h"
#endif // CONFIG_ETH_USE_SPI_ETHERNET

static const char *TAG = "vban";

#if CONFIG_EXAMPLE_USE_SPI_ETHERNET
#define INIT_SPI_ETH_MODULE_CONFIG(eth_module_config, num)                                      \
    do {                                                                                        \
        eth_module_config[num].spi_cs_gpio = CONFIG_EXAMPLE_ETH_SPI_CS ##num## _GPIO;           \
        eth_module_config[num].int_gpio = CONFIG_EXAMPLE_ETH_SPI_INT ##num## _GPIO;             \
        eth_module_config[num].phy_reset_gpio = CONFIG_EXAMPLE_ETH_SPI_PHY_RST ##num## _GPIO;   \
        eth_module_config[num].phy_addr = CONFIG_EXAMPLE_ETH_SPI_PHY_ADDR ##num;                \
    } while(0)

typedef struct {
    uint8_t spi_cs_gpio;
    uint8_t int_gpio;
    int8_t phy_reset_gpio;
    uint8_t phy_addr;
}spi_eth_module_config_t;
#endif


#define EXAMPLE_STD_MCLK_IO1        0
#define EXAMPLE_STD_BCLK_IO1        15
#define EXAMPLE_STD_WS_IO1          14
#define EXAMPLE_STD_DOUT_IO1        12

#define MAX_UDP_BYTES_PER_PACKET    1472
#define MAX_VBAN_NO_DATA_CNT        50


static bool i2s_running = false;
static i2s_chan_handle_t tx_chan;
static VBAN_RB_T s_tRingBuf;
static COUNTERS_T s_tCounters;
static SemaphoreHandle_t s_tSemaphore;
static VBAN_BR_T s_tLastBitRes = VBAN_BIT_RES_INT16;
static uint32_t s_aulTempBuffer[2*VBAN_FRAME_MAX_SAMPLES];

static i2s_std_config_t tx_std_cfg = {
    .clk_cfg = {
        .sample_rate_hz = 48000,
        .clk_src = I2S_CLK_SRC_PLL_160M, /* Don't use APLL as its needed for Ethernet */
        .mclk_multiple = 384
    },
    .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_24BIT, I2S_SLOT_MODE_STEREO),
    .gpio_cfg = {
        .mclk = EXAMPLE_STD_MCLK_IO1,
        .bclk = EXAMPLE_STD_BCLK_IO1,
        .ws   = EXAMPLE_STD_WS_IO1,
        .dout = EXAMPLE_STD_DOUT_IO1,
        .din  = I2S_GPIO_UNUSED,
        .invert_flags = {
            .mclk_inv = false,
            .bclk_inv = false,
            .ws_inv   = false,
        },
    },
};

/** Event handler for Ethernet events */
static void eth_event_handler(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data)
{
    uint8_t mac_addr[6] = {0};
    /* we can get the ethernet driver handle from event data */
    esp_eth_handle_t eth_handle = *(esp_eth_handle_t *)event_data;

    switch (event_id) {
    case ETHERNET_EVENT_CONNECTED:
        esp_eth_ioctl(eth_handle, ETH_CMD_G_MAC_ADDR, mac_addr);
        ESP_LOGI(TAG, "Ethernet Link Up");
        ESP_LOGI(TAG, "Ethernet HW Addr %02x:%02x:%02x:%02x:%02x:%02x",
                 mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
        break;
    case ETHERNET_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "Ethernet Link Down");
        break;
    case ETHERNET_EVENT_START:
        ESP_LOGI(TAG, "Ethernet Started");
        break;
    case ETHERNET_EVENT_STOP:
        ESP_LOGI(TAG, "Ethernet Stopped");
        break;
    default:
        break;
    }
}

/** Event handler for IP_EVENT_ETH_GOT_IP */
static void got_ip_event_handler(void *arg, esp_event_base_t event_base,
                                 int32_t event_id, void *event_data)
{
    ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
    const esp_netif_ip_info_t *ip_info = &event->ip_info;

    ESP_LOGI(TAG, "Ethernet Got IP Address");
    ESP_LOGI(TAG, "~~~~~~~~~~~");
    ESP_LOGI(TAG, "ETHIP:" IPSTR, IP2STR(&ip_info->ip));
    ESP_LOGI(TAG, "ETHMASK:" IPSTR, IP2STR(&ip_info->netmask));
    ESP_LOGI(TAG, "ETHGW:" IPSTR, IP2STR(&ip_info->gw));
    ESP_LOGI(TAG, "~~~~~~~~~~~");
}

static void deinit_i2s_tx_only()
{
    ESP_LOGI(TAG, "Deinitializing i2s driver");
    ESP_ERROR_CHECK(i2s_channel_disable(tx_chan));
    ESP_ERROR_CHECK(i2s_del_channel(tx_chan));
    i2s_running = false;
}

static void init_i2s_tx_only(uint32_t sample_rate, VBAN_BR_T tBitRes)
{
    if(i2s_running) {
        deinit_i2s_tx_only();
    }
    ESP_LOGI(TAG, "Configuring i2s driver:");
    ESP_LOGI(TAG, " - Sample Rate: %u", (int)(sample_rate));
    i2s_chan_config_t tx_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&tx_chan_cfg, &tx_chan, NULL));
    
    /* Reconfigure depending on sample rate and data type */
    tx_std_cfg.clk_cfg.sample_rate_hz = sample_rate;
    switch(tBitRes) {
        default:
        case VBAN_BIT_RES_INT16:
            ESP_LOGI(TAG, " - Bit resolution: INT16");
            tx_std_cfg.clk_cfg.mclk_multiple = 256;
            tx_std_cfg.slot_cfg.data_bit_width = I2S_DATA_BIT_WIDTH_16BIT;
            tx_std_cfg.slot_cfg.slot_bit_width = I2S_DATA_BIT_WIDTH_16BIT;
            tx_std_cfg.slot_cfg.ws_width = I2S_DATA_BIT_WIDTH_16BIT;
            tx_std_cfg.slot_cfg.msb_right = true;
            break;
        case VBAN_BIT_RES_INT24:
            ESP_LOGI(TAG, " - Bit resolution: INT24");
            tx_std_cfg.clk_cfg.mclk_multiple = 384;
            tx_std_cfg.slot_cfg.data_bit_width = I2S_DATA_BIT_WIDTH_24BIT;
            tx_std_cfg.slot_cfg.slot_bit_width = I2S_DATA_BIT_WIDTH_24BIT;
            tx_std_cfg.slot_cfg.ws_width = I2S_DATA_BIT_WIDTH_24BIT;
            tx_std_cfg.slot_cfg.msb_right = false;
            break;
    }

    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_chan, &tx_std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(tx_chan));
    i2s_running = true;
    s_tLastBitRes = tBitRes;
}

static void udp_server_task(void *pvParameters)
{
    int addr_family = (int)pvParameters;
    int ip_protocol = 0;
    struct sockaddr_in6 dest_addr;

    while (1) {

        if (addr_family == AF_INET) {
            struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
            dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
            dest_addr_ip4->sin_family = AF_INET;
            dest_addr_ip4->sin_port = htons(PORT);
            ip_protocol = IPPROTO_IP;
        } else if (addr_family == AF_INET6) {
            bzero(&dest_addr.sin6_addr.un, sizeof(dest_addr.sin6_addr.un));
            dest_addr.sin6_family = AF_INET6;
            dest_addr.sin6_port = htons(PORT);
            ip_protocol = IPPROTO_IPV6;
        }

        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created");

#if defined(CONFIG_EXAMPLE_IPV4) && defined(CONFIG_EXAMPLE_IPV6)
        if (addr_family == AF_INET6) {
            // Note that by default IPV6 binds to both protocols, it is must be disabled
            // if both protocols used at the same time (used in CI)
            int opt = 1;
            setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
            setsockopt(sock, IPPROTO_IPV6, IPV6_V6ONLY, &opt, sizeof(opt));
        }
#endif
        // Set timeout
        struct timeval timeout;
        timeout.tv_sec = 0;
        timeout.tv_usec = 0;
        setsockopt (sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof timeout);

        int err = bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err < 0) {
            ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        }
        ESP_LOGI(TAG, "Socket bound, port %d", PORT);

        struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
        socklen_t socklen = sizeof(source_addr);

        VBAN_FRAME_T* ptFrame = NULL;
        VBAN_FRAME_T** pptFrame = &ptFrame;
        char abDummy[1600];
        while (1) {
            if(VBAN_RingBuf_GetNextFreeFrame(&s_tRingBuf, pptFrame) != VBAN_RB_OKAY) {
                s_tCounters.uiBufferOverflow++;
                recvfrom(sock, abDummy, sizeof(abDummy), 0, (struct sockaddr *)&source_addr, &socklen);
            } 
            else {
                int uiBytesReceived = recvfrom(sock, &(ptFrame->tPacket), VBAN_FRAME_MAX_LENGTH, 0, (struct sockaddr *)&source_addr, &socklen);

                // Error occurred during receiving
                if (uiBytesReceived < 0)
                {
                    if(errno != 11) /* EAGAIN */ {
                        ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
                        break;
                    }
                }
                else {
                    VBAN_Frame_SetTotalLength(ptFrame, uiBytesReceived);
                    VBAN_RingBuf_Push(&s_tRingBuf);
                    xSemaphoreGive(s_tSemaphore);
                }
            }
        }

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}

void task_print_stats(void* ptArgs)
{
    while(1)
    {
        ESP_LOGI(TAG, "");
        ESP_LOGI(TAG, "");
        ESP_LOGI(TAG, "FRAME_OK             : %d", s_tCounters.uiFrameOk);
        ESP_LOGI(TAG, "I2S_BYTES_WRITTEN    : %d", s_tCounters.uiI2SBytesWritten);
        ESP_LOGI(TAG, "FRAME_TOO_LONG       : %d", s_tCounters.uiFrameTooLong);
        ESP_LOGI(TAG, "FRAME_TOO_SHORT      : %d", s_tCounters.uiFrameTooShort);
        ESP_LOGI(TAG, "INVALID_PREAMBLE     : %d", s_tCounters.uiInvalidPreamble);
        ESP_LOGI(TAG, "UNEXPECTED_SUBPROTO  : %d", s_tCounters.uiUnexpectedSubproto);
        ESP_LOGI(TAG, "UNEXPECTED_CODEC     : %d", s_tCounters.uiUnexpectedCodec);
        ESP_LOGI(TAG, "UNSUPPORTED_BIT_RES  : %d", s_tCounters.uiUnsupportedBitRes);
        ESP_LOGI(TAG, "INVALID_NUM_SAMPLES  : %d", s_tCounters.uiInvalidNumSamples);
        ESP_LOGI(TAG, "INVALID_NUM_CHANNELS : %d", s_tCounters.uiInvalidNumChannels);
        ESP_LOGI(TAG, "BR_BIT_3_NOT_ZERO    : %d", s_tCounters.uiBrBit3NotZero);
        ESP_LOGI(TAG, "BUFFER_OVERFLOW      : %d", s_tCounters.uiBufferOverflow);
        ESP_LOGI(TAG, "BUFFER_UNDERFLOW     : %d", s_tCounters.uiBufferUnderflow);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void audio_write_task(void* ptArgs) 
{
    unsigned int uiNoDataCnt = 0;  
    size_t bytesOut = 0;      
    VBAN_FRAME_T* ptFrame = NULL;
    VBAN_FRAME_T** pptFrame = &ptFrame;
    bool fRunning = false;
    while(1) 
    {
        /* Wait for task to be synchronized */
        if(!fRunning) 
        {
            if(xSemaphoreTake(s_tSemaphore, 1 / portTICK_PERIOD_MS) != pdTRUE)
            {
                if(i2s_running) {
                    uiNoDataCnt++;
                    if(uiNoDataCnt > 20) {
                        deinit_i2s_tx_only();
                        uiNoDataCnt = 0;
                    }
                } 
                continue;
            } 
            else {
                fRunning = true;
            }
        }

        /* Get next data frame from ring buffer */
        if(VBAN_RingBuf_GetNextDataFrame(&s_tRingBuf, pptFrame) != VBAN_RB_OKAY) {
            s_tCounters.uiBufferUnderflow++;
            fRunning = false;
            continue;
        }

        /* Check if packet length meets VBAN specification */
        VBAN_ERR_T retval = VBAN_Frame_Validate(ptFrame, VBAN_PROTOCOL_AUDIO, VBAN_CODEC_PCM);
        if(retval != VBAN_OK) {
            VBAN_RingBuf_Pop(&s_tRingBuf);
            switch(retval) {
                case VBAN_ERR_PACKET_TOO_LONG: s_tCounters.uiFrameTooLong++; break;
                case VBAN_ERR_PACKET_TOO_SHORT: s_tCounters.uiFrameTooShort++; break;
                case VBAN_ERR_INVALID_PREAMBLE: s_tCounters.uiInvalidPreamble++; break;
                case VBAN_ERR_UNEXPECTED_SUBPROTO: s_tCounters.uiUnexpectedSubproto++; break;
                case VBAN_ERR_UNEXPECTED_CODEC: s_tCounters.uiUnexpectedCodec++; break;
                case VBAN_ERR_UNSUPPORTED_BIT_RES: s_tCounters.uiUnsupportedBitRes++; break;
                case VBAN_ERR_INVALID_NUM_SAMPLES: s_tCounters.uiInvalidNumSamples++; break;
                case VBAN_ERR_INVALID_NUM_CHANNELS: s_tCounters.uiInvalidNumChannels++; break;
                case VBAN_ERR_BR_BIT3_NOT_ZERO: s_tCounters.uiBrBit3NotZero++; break;
                default:
                    break;
            }
            continue;
        }

        /* If necessary, reconfigre I2S to match VBAN format */
        unsigned int uiSampleRate = VBAN_Frame_GetSampleRate(ptFrame);
        VBAN_BR_T tBitRes = VBAN_Frame_GetBitResolution(ptFrame);
        if(!i2s_running || uiSampleRate != tx_std_cfg.clk_cfg.sample_rate_hz || tBitRes != s_tLastBitRes) {
            init_i2s_tx_only(uiSampleRate, tBitRes);
        }

        /* Write PCM data to I2S interface */
        switch(tBitRes) {
            case VBAN_BIT_RES_INT24: {
                /* See: https://www.esp32.com/viewtopic.php?t=6650 */
                uint8_t* ptAudioData = VBAN_Frame_GetData(ptFrame);

                /* Sort all samples into dwords: [ sample[23:16], sample[15:8], sample[7:0], 8'h0 ]*/
                // ESP_LOGI(TAG, "Samples to process: %u", VBAN_Frame_GetNumSamples(ptFrame)*2);
                // ESP_LOGI(TAG, "Maximum possible samples: %u", 2*VBAN_FRAME_MAX_SAMPLES);
                // ESP_LOGI(TAG, "VBAN raw data length: %u", VBAN_Frame_GetDataLen(ptFrame));
                // ESP_LOGI(TAG, "Temp buffer length: %u", VBAN_Frame_GetNumSamples(ptFrame)*2*4);

                for(unsigned int i = 0; i < VBAN_Frame_GetNumSamples(ptFrame)*2; i++) {
                    unsigned int uiCurSampleOffs = i*3;
                    s_aulTempBuffer[i] = ptAudioData[uiCurSampleOffs] << 8 
                                     | ptAudioData[uiCurSampleOffs + 1] << 16 
                                     | ptAudioData[uiCurSampleOffs + 2] << 24;
                    // ESP_LOGI(TAG, "Bytes: %x, %x, %x", (unsigned int)ptAudioData[uiCurSampleOffs], (unsigned int)ptAudioData[uiCurSampleOffs + 1], (unsigned int)ptAudioData[uiCurSampleOffs + 2]);
                    // ESP_LOGI(TAG, "Dword: %x", (unsigned int)s_aulTempBuffer[i]);
                }

                /* Give dword buffer to i2s peripherial */
                i2s_channel_write(tx_chan, s_aulTempBuffer, VBAN_Frame_GetNumSamples(ptFrame)*2*4, &bytesOut, portMAX_DELAY);
                break;
            }
            default: {
                i2s_channel_write(tx_chan, VBAN_Frame_GetData(ptFrame), VBAN_Frame_GetDataLen(ptFrame), &bytesOut, portMAX_DELAY);
                break;
            }
        }
        VBAN_RingBuf_Pop(&s_tRingBuf);
        s_tCounters.uiFrameOk++;
        s_tCounters.uiI2SBytesWritten += bytesOut;
        uiNoDataCnt = 0;
    };
}

void app_main(void)
{
    // Initialize TCP/IP network interface (should be called only once in application)
    ESP_ERROR_CHECK(esp_netif_init());
    // Create default event loop that running in background
    ESP_ERROR_CHECK(esp_event_loop_create_default());

#if CONFIG_EXAMPLE_USE_INTERNAL_ETHERNET
    // Create new default instance of esp-netif for Ethernet
    esp_netif_config_t cfg = ESP_NETIF_DEFAULT_ETH();
    esp_netif_t *eth_netif = esp_netif_new(&cfg);

    // Init MAC and PHY configs to default
    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();

    phy_config.phy_addr = CONFIG_EXAMPLE_ETH_PHY_ADDR;
    phy_config.reset_gpio_num = CONFIG_EXAMPLE_ETH_PHY_RST_GPIO;
    eth_esp32_emac_config_t esp32_emac_config = ETH_ESP32_EMAC_DEFAULT_CONFIG();
    esp32_emac_config.smi_mdc_gpio_num = CONFIG_EXAMPLE_ETH_MDC_GPIO;
    esp32_emac_config.smi_mdio_gpio_num = CONFIG_EXAMPLE_ETH_MDIO_GPIO;
    esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&esp32_emac_config, &mac_config);
#if CONFIG_EXAMPLE_ETH_PHY_IP101
    esp_eth_phy_t *phy = esp_eth_phy_new_ip101(&phy_config);
#elif CONFIG_EXAMPLE_ETH_PHY_RTL8201
    esp_eth_phy_t *phy = esp_eth_phy_new_rtl8201(&phy_config);
#elif CONFIG_EXAMPLE_ETH_PHY_LAN87XX
    esp_eth_phy_t *phy = esp_eth_phy_new_lan87xx(&phy_config);
#elif CONFIG_EXAMPLE_ETH_PHY_DP83848
    esp_eth_phy_t *phy = esp_eth_phy_new_dp83848(&phy_config);
#elif CONFIG_EXAMPLE_ETH_PHY_KSZ80XX
    esp_eth_phy_t *phy = esp_eth_phy_new_ksz80xx(&phy_config);
#endif
    esp_eth_config_t config = ETH_DEFAULT_CONFIG(mac, phy);
    esp_eth_handle_t eth_handle = NULL;
    ESP_ERROR_CHECK(esp_eth_driver_install(&config, &eth_handle));
    /* attach Ethernet driver to TCP/IP stack */
    ESP_ERROR_CHECK(esp_netif_attach(eth_netif, esp_eth_new_netif_glue(eth_handle)));
#endif //CONFIG_EXAMPLE_USE_INTERNAL_ETHERNET

#if CONFIG_EXAMPLE_USE_SPI_ETHERNET
    // Create instance(s) of esp-netif for SPI Ethernet(s)
    esp_netif_inherent_config_t esp_netif_config = ESP_NETIF_INHERENT_DEFAULT_ETH();
    esp_netif_config_t cfg_spi = {
        .base = &esp_netif_config,
        .stack = ESP_NETIF_NETSTACK_DEFAULT_ETH
    };
    esp_netif_t *eth_netif_spi[CONFIG_EXAMPLE_SPI_ETHERNETS_NUM] = { NULL };
    char if_key_str[10];
    char if_desc_str[10];
    char num_str[3];
    for (int i = 0; i < CONFIG_EXAMPLE_SPI_ETHERNETS_NUM; i++) {
        itoa(i, num_str, 10);
        strcat(strcpy(if_key_str, "ETH_SPI_"), num_str);
        strcat(strcpy(if_desc_str, "eth"), num_str);
        esp_netif_config.if_key = if_key_str;
        esp_netif_config.if_desc = if_desc_str;
        esp_netif_config.route_prio = 30 - i;
        eth_netif_spi[i] = esp_netif_new(&cfg_spi);
    }

    // Init MAC and PHY configs to default
    eth_mac_config_t mac_config_spi = ETH_MAC_DEFAULT_CONFIG();
    eth_phy_config_t phy_config_spi = ETH_PHY_DEFAULT_CONFIG();

    // Install GPIO ISR handler to be able to service SPI Eth modlues interrupts
    gpio_install_isr_service(0);

    // Init SPI bus
    spi_bus_config_t buscfg = {
        .miso_io_num = CONFIG_EXAMPLE_ETH_SPI_MISO_GPIO,
        .mosi_io_num = CONFIG_EXAMPLE_ETH_SPI_MOSI_GPIO,
        .sclk_io_num = CONFIG_EXAMPLE_ETH_SPI_SCLK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(CONFIG_EXAMPLE_ETH_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));

    // Init specific SPI Ethernet module configuration from Kconfig (CS GPIO, Interrupt GPIO, etc.)
    spi_eth_module_config_t spi_eth_module_config[CONFIG_EXAMPLE_SPI_ETHERNETS_NUM];
    INIT_SPI_ETH_MODULE_CONFIG(spi_eth_module_config, 0);
#if CONFIG_EXAMPLE_SPI_ETHERNETS_NUM > 1
    INIT_SPI_ETH_MODULE_CONFIG(spi_eth_module_config, 1);
#endif

    // Configure SPI interface and Ethernet driver for specific SPI module
    esp_eth_mac_t *mac_spi[CONFIG_EXAMPLE_SPI_ETHERNETS_NUM];
    esp_eth_phy_t *phy_spi[CONFIG_EXAMPLE_SPI_ETHERNETS_NUM];
    esp_eth_handle_t eth_handle_spi[CONFIG_EXAMPLE_SPI_ETHERNETS_NUM] = { NULL };
    spi_device_interface_config_t spi_devcfg = {
        .mode = 0,
        .clock_speed_hz = CONFIG_EXAMPLE_ETH_SPI_CLOCK_MHZ * 1000 * 1000,
        .queue_size = 20
    };
    for (int i = 0; i < CONFIG_EXAMPLE_SPI_ETHERNETS_NUM; i++) {
        // Set SPI module Chip Select GPIO
        spi_devcfg.spics_io_num = spi_eth_module_config[i].spi_cs_gpio;
        // Set remaining GPIO numbers and configuration used by the SPI module
        phy_config_spi.phy_addr = spi_eth_module_config[i].phy_addr;
        phy_config_spi.reset_gpio_num = spi_eth_module_config[i].phy_reset_gpio;
#if CONFIG_EXAMPLE_USE_KSZ8851SNL
        eth_ksz8851snl_config_t ksz8851snl_config = ETH_KSZ8851SNL_DEFAULT_CONFIG(CONFIG_EXAMPLE_ETH_SPI_HOST, &spi_devcfg);
        ksz8851snl_config.int_gpio_num = spi_eth_module_config[i].int_gpio;
        mac_spi[i] = esp_eth_mac_new_ksz8851snl(&ksz8851snl_config, &mac_config_spi);
        phy_spi[i] = esp_eth_phy_new_ksz8851snl(&phy_config_spi);
#elif CONFIG_EXAMPLE_USE_DM9051
        eth_dm9051_config_t dm9051_config = ETH_DM9051_DEFAULT_CONFIG(CONFIG_EXAMPLE_ETH_SPI_HOST, &spi_devcfg);
        dm9051_config.int_gpio_num = spi_eth_module_config[i].int_gpio;
        mac_spi[i] = esp_eth_mac_new_dm9051(&dm9051_config, &mac_config_spi);
        phy_spi[i] = esp_eth_phy_new_dm9051(&phy_config_spi);
#elif CONFIG_EXAMPLE_USE_W5500
        eth_w5500_config_t w5500_config = ETH_W5500_DEFAULT_CONFIG(CONFIG_EXAMPLE_ETH_SPI_HOST, &spi_devcfg);
        w5500_config.int_gpio_num = spi_eth_module_config[i].int_gpio;
        mac_spi[i] = esp_eth_mac_new_w5500(&w5500_config, &mac_config_spi);
        phy_spi[i] = esp_eth_phy_new_w5500(&phy_config_spi);
#endif
        esp_eth_config_t eth_config_spi = ETH_DEFAULT_CONFIG(mac_spi[i], phy_spi[i]);
        ESP_ERROR_CHECK(esp_eth_driver_install(&eth_config_spi, &eth_handle_spi[i]));

        /* The SPI Ethernet module might not have a burned factory MAC address, we cat to set it manually.
       02:00:00 is a Locally Administered OUI range so should not be used except when testing on a LAN under your control.
        */
        ESP_ERROR_CHECK(esp_eth_ioctl(eth_handle_spi[i], ETH_CMD_S_MAC_ADDR, (uint8_t[]) {
            0x02, 0x00, 0x00, 0x12, 0x34, 0x56 + i
        }));

        // attach Ethernet driver to TCP/IP stack
        ESP_ERROR_CHECK(esp_netif_attach(eth_netif_spi[i], esp_eth_new_netif_glue(eth_handle_spi[i])));
    }
#endif // CONFIG_ETH_USE_SPI_ETHERNET

    // Register user defined event handers
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &got_ip_event_handler, NULL));

    /* start Ethernet driver state machine */
#if CONFIG_EXAMPLE_USE_INTERNAL_ETHERNET
    ESP_ERROR_CHECK(esp_eth_start(eth_handle));
#endif // CONFIG_EXAMPLE_USE_INTERNAL_ETHERNET
#if CONFIG_EXAMPLE_USE_SPI_ETHERNET
    for (int i = 0; i < CONFIG_EXAMPLE_SPI_ETHERNETS_NUM; i++) {
        ESP_ERROR_CHECK(esp_eth_start(eth_handle_spi[i]));
    }
#endif // CONFIG_EXAMPLE_USE_SPI_ETHERNET

    //init_i2s_tx_only(44100);

    VBAN_RingBuf_Init(&s_tRingBuf);

    s_tSemaphore = xSemaphoreCreateBinary();

    xTaskCreate(udp_server_task, "udp_server", 4096, (void*)AF_INET, 5, NULL);
    xTaskCreate(audio_write_task, "audio", 4096, NULL, 4, NULL);
    xTaskCreate(task_print_stats, "stats", 4096, NULL, 0, NULL);

    esp_task_wdt_deinit();
}
