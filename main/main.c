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
#include "vban/stream.h"
#include "vban/stream_ctrl.h"
#include "interfaces/ethernet.h"

//#if defined(CONFIG_ESP32_VBAN_IPV4)
//#define HOST_IP_ADDR CONFIG_ESP32_VBAN_IPV4_ADDR
//#elif defined(CONFIG_ESP32_VBAN_IPV6)
//#define HOST_IP_ADDR CONFIG_ESP32_VBAN_IPV6_ADDR
//#else//
//#define HOST_IP_ADDR ""
//#endif

#define PORT 6980

#if CONFIG_ETH_USE_SPI_ETHERNET
#include "driver/spi_master.h"
#endif // CONFIG_ETH_USE_SPI_ETHERNET

static const char *TAG = "vban";

#if CONFIG_ESP32_VBAN_USE_SPI_ETHERNET
#define INIT_SPI_ETH_MODULE_CONFIG(eth_module_config, num)                                      \
    do {                                                                                        \
        eth_module_config[num].spi_cs_gpio = CONFIG_ESP32_VBAN_ETH_SPI_CS ##num## _GPIO;           \
        eth_module_config[num].int_gpio = CONFIG_ESP32_VBAN_ETH_SPI_INT ##num## _GPIO;             \
        eth_module_config[num].phy_reset_gpio = CONFIG_ESP32_VBAN_ETH_SPI_PHY_RST ##num## _GPIO;   \
        eth_module_config[num].phy_addr = CONFIG_ESP32_VBAN_ETH_SPI_PHY_ADDR ##num;                \
    } while(0)

typedef struct {
    uint8_t spi_cs_gpio;
    uint8_t int_gpio;
    int8_t phy_reset_gpio;
    uint8_t phy_addr;
}spi_eth_module_config_t;
#endif


#define ESP32_VBAN_STD_MCLK_IO1        0
#define ESP32_VBAN_STD_BCLK_IO1        15
#define ESP32_VBAN_STD_WS_IO1          14
#define ESP32_VBAN_STD_DOUT_IO1        12

#define MAX_UDP_BYTES_PER_PACKET    1472
#define MAX_VBAN_NO_DATA_CNT        50


static bool i2s_running = false;
static i2s_chan_handle_t tx_chan;
static VBAN_BR_T s_tLastBitRes = VBAN_BIT_RES_INT16;
static uint32_t s_aulTempBuffer[2*VBAN_FRAME_MAX_SAMPLES];

static STREAM_CTRL_T s_tStreamCtrl;

static i2s_std_config_t tx_std_cfg = {
    .clk_cfg = {
        .sample_rate_hz = 48000,
        .clk_src = I2S_CLK_SRC_PLL_160M, /* Don't use APLL as its needed for Ethernet */
        .mclk_multiple = 384
    },
    .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_24BIT, I2S_SLOT_MODE_STEREO),
    .gpio_cfg = {
        .mclk = ESP32_VBAN_STD_MCLK_IO1,
        .bclk = ESP32_VBAN_STD_BCLK_IO1,
        .ws   = ESP32_VBAN_STD_WS_IO1,
        .dout = ESP32_VBAN_STD_DOUT_IO1,
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

#if defined(CONFIG_ESP32_VBAN_IPV4) && defined(CONFIG_ESP32_VBAN_IPV6)
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

        VBAN_FRAME_T tFrame;
        STREAM_T* ptStream = NULL; 

        while (1) 
        {
            int uiBytesReceived = recvfrom(sock, &(tFrame.tPacket), VBAN_FRAME_MAX_LENGTH, 0, (struct sockaddr *)&source_addr, &socklen);

            // Error occurred during receiving
            if (uiBytesReceived < 0)
            {
                if(errno != 11) /* EAGAIN */ {
                    ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
                    break;
                }
            }
            else {
                VBAN_Frame_SetTotalLength(&tFrame, uiBytesReceived);
                tFrame.tPacket.tHeader.cStreamName[15] = '\0'; /* null byte termination */

                if(!strcmp(VBAN_Frame_GetStreamName(&tFrame), "VBAN Service")) {
                    continue;
                }

                if(STREAM_CTRL_OKAY != StreamCtrl_GetStreamByName(&s_tStreamCtrl, VBAN_Frame_GetStreamName(&tFrame), &ptStream)) {
                  continue;  
                }

                Stream_AddFrame(ptStream, &tFrame);
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
        StreamCtrl_PrintStats(&s_tStreamCtrl, TAG);
        StreamCtrl_Update(&s_tStreamCtrl);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void audio_write_task(void* ptArgs) 
{
    unsigned int uiNoDataCnt = 0;  
    size_t bytesOut = 0;      
    VBAN_FRAME_T tFrame;

    while(1) 
    {
        if(STREAM_CTRL_OKAY != StreamCtrl_ComputeNextFrame(&s_tStreamCtrl, &tFrame)) 
        {
            if(i2s_running) {
                uiNoDataCnt++;
                if(uiNoDataCnt > 20) {
                    deinit_i2s_tx_only();
                    uiNoDataCnt = 0;
                }
            } 
            vTaskDelay(10 / portTICK_PERIOD_MS);  /* don't busy-wait and give ethernet task a chance to run */
            continue;
            // taskYIELD(); /* don't busy-wait and give ethernet task a chance to run /// OLD */
            // continue;
        }

        /* If necessary, reconfigre I2S to match VBAN format */
        unsigned int uiSampleRate = VBAN_Frame_GetSampleRate(&tFrame);
        VBAN_BR_T tBitRes = VBAN_Frame_GetBitResolution(&tFrame);
        if(!i2s_running || uiSampleRate != tx_std_cfg.clk_cfg.sample_rate_hz || tBitRes != s_tLastBitRes) {
            init_i2s_tx_only(uiSampleRate, tBitRes);
        }

        /* Write PCM data to I2S interface */
        switch(tBitRes) {
            case VBAN_BIT_RES_INT24: {
                /* See: https://www.esp32.com/viewtopic.php?t=6650 */
                uint8_t* ptAudioData = VBAN_Frame_GetData(&tFrame);

                /* Sort all samples into dwords: [ sample[23:16], sample[15:8], sample[7:0], 8'h0 ]*/
                // ESP_LOGI(TAG, "Samples to process: %u", VBAN_Frame_GetNumSamples(ptFrame)*2);
                // ESP_LOGI(TAG, "Maximum possible samples: %u", 2*VBAN_FRAME_MAX_SAMPLES);
                // ESP_LOGI(TAG, "VBAN raw data length: %u", VBAN_Frame_GetDataLen(ptFrame));
                // ESP_LOGI(TAG, "Temp buffer length: %u", VBAN_Frame_GetNumSamples(ptFrame)*2*4);

                for(unsigned int i = 0; i < VBAN_Frame_GetNumSamples(&tFrame)*2; i++) {
                    unsigned int uiCurSampleOffs = i*3;
                    s_aulTempBuffer[i] = ptAudioData[uiCurSampleOffs] << 8 
                                     | ptAudioData[uiCurSampleOffs + 1] << 16 
                                     | ptAudioData[uiCurSampleOffs + 2] << 24;
                    // ESP_LOGI(TAG, "Bytes: %x, %x, %x", (unsigned int)ptAudioData[uiCurSampleOffs], (unsigned int)ptAudioData[uiCurSampleOffs + 1], (unsigned int)ptAudioData[uiCurSampleOffs + 2]);
                    // ESP_LOGI(TAG, "Dword: %x", (unsigned int)s_aulTempBuffer[i]);
                }

                /* Give dword buffer to i2s peripherial */
                i2s_channel_write(tx_chan, s_aulTempBuffer, VBAN_Frame_GetNumSamples(&tFrame)*2*4, &bytesOut, portMAX_DELAY);
                break;
            }
            default: {
                i2s_channel_write(tx_chan, VBAN_Frame_GetData(&tFrame), VBAN_Frame_GetDataLen(&tFrame), &bytesOut, portMAX_DELAY);
                break;
            }
        }
        s_tStreamCtrl.tCounters.uiI2SBytesWritten += bytesOut;
        uiNoDataCnt = 0;
        
        taskYIELD(); /* don't busy-wait and give ethernet task a chance to run */
    };
}

void app_main(void)
{
    // Initialize TCP/IP network interface (should be called only once in application)
    ESP_ERROR_CHECK(esp_netif_init());
    // Create default event loop that running in background
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    Ethernet_Init();

    // Register user defined event handers
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &got_ip_event_handler, NULL));

    Ethernet_Start();

    //init_i2s_tx_only(44100);

    xTaskCreate(udp_server_task, "udp_server", 4096, (void*)AF_INET, 5, NULL);
    xTaskCreate(audio_write_task, "audio", 4096, NULL, configMAX_PRIORITIES - 1, NULL);
    xTaskCreate(task_print_stats, "stats", 4096, NULL, 0, NULL);

    esp_task_wdt_deinit();
}
