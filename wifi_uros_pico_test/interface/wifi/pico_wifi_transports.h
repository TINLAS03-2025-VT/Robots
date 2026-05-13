#ifndef __MICRO_ROS_PICOSDK_WIFI__
#define __MICRO_ROS_PICOSDK_WIFI__

#include "../common/transport_common.h"

bool pico_wifi_transport_open(struct uxrCustomTransport *transport);
bool pico_wifi_transport_close(struct uxrCustomTransport *transport);
size_t pico_wifi_transport_write(struct uxrCustomTransport *transport, const uint8_t *buf, size_t len, uint8_t *err);
size_t pico_wifi_transport_read(struct uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout, uint8_t *err);

static inline bool set_microros_wifi_transports(char *ssid, char *pass, char *agent_ip, uint agent_port)
{

        stdio_init_all();

    if (cyw43_arch_init())
    {
        printf("failed to initialise\n");
        return 1;
    }

    cyw43_arch_enable_sta_mode();

    sleep_ms(1000);

    printf("Connecting to Wi-Fi...\n");
    if (cyw43_arch_wifi_connect_timeout_ms(ssid, pass, CYW43_AUTH_WPA2_AES_PSK, 20000))
    {
        printf("failed to connect.\n");
        return 1;
    }
    else
    {
        printf("Connected.\n");
    }

    static struct micro_ros_agent_locator locator;
    ipaddr_aton(agent_ip, &locator.address);
    locator.port = agent_port;

    rmw_uros_set_custom_transport(
        false,
        (void *)&locator,
        pico_wifi_transport_open,
        pico_wifi_transport_close,
        pico_wifi_transport_write,
        pico_wifi_transport_read);

    return 0;
}

#endif
