#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/wifi_mgmt.h>

#include <microros_transports.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/int32.h>
#include <uxr/client/client.h> /* for struct uxrCustomTransport */

LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

#define WIFI_SSID "sml_legacy"
#define WIFI_PSK "SML4admin."
#define AGENT_IP "10.0.0.107"
#define AGENT_PORT "8888"
#define PUB_MS 1000

static struct net_if *sta;

/* ------------------------------------------------------------------ */
static void micro_ros_thread(void *, void *, void *);

K_THREAD_DEFINE(uros_tid, 4096, micro_ros_thread, NULL, NULL, NULL, 5, 0, 0);

static void wifi_event_cb(struct net_mgmt_event_callback *cb,
                          uint32_t evt, struct net_if *iface) {
    if (evt != NET_EVENT_WIFI_CONNECT_RESULT)
        return;

    const struct wifi_status *st = (const struct wifi_status *)cb->info;
    if (st->status == 0) {
        LOG_INF("Wi-Fi connected, waiting for DHCP…");
    } else {
        LOG_ERR("Wi-Fi connect failed: %d", st->status);
    }
}

/* ------------------------------------------------------------------ */
static void micro_ros_thread(void *a1, void *a2, void *a3) {
    ARG_UNUSED(a1);
    ARG_UNUSED(a2);
    ARG_UNUSED(a3);

    /* Wait until DHCP gives us an IPv4 address */
    struct net_if_addr *if_addr;
    int retries = 20;
    while (!(if_addr = net_if_ipv4_get_addr(sta, NET_ADDR_DHCP, NULL)) && retries--) {
        k_msleep(500);
    }
    if (!if_addr) {
        LOG_ERR("DHCP timeout");
        return;
    }

    char ipbuf[NET_IPV4_ADDR_LEN];
    net_addr_ntop(AF_INET, &if_addr->address, ipbuf, sizeof(ipbuf));
    LOG_INF("DHCP OK, IP: %s", ipbuf);

    /* micro-ROS transport */
    zephyr_transport_params_t tp = {.ip = AGENT_IP, .port = AGENT_PORT};
    rmw_ret_t rmw_rc = rmw_uros_set_custom_transport(
        false, &tp,
        zephyr_transport_open,
        zephyr_transport_close,
        zephyr_transport_write,
        zephyr_transport_read);
    if (rmw_rc != RMW_RET_OK) {
        LOG_ERR("transport setup failed: %d", rmw_rc);
        return;
    }

    /* rcl/rclc */
    rcl_allocator_t alloc = rcl_get_default_allocator();
    rclc_support_t support;
    rcl_ret_t rc = rclc_support_init(&support, 0, NULL, &alloc);
    if (rc != RCL_RET_OK)
        goto fail;

    rcl_node_t node;
    rc = rclc_node_init_default(&node, "esp32c3", "", &support);
    if (rc != RCL_RET_OK)
        goto fail;

    rcl_publisher_t pub;
    rc = rclc_publisher_init_default(
        &pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "wifi_int");
    if (rc != RCL_RET_OK)
        goto fail;

    rcl_timer_t timer;
    rc = rclc_timer_init_default2(
        &timer, &support, RCL_MS_TO_NS(PUB_MS),
        [], (rcl_timer_t *, int64_t) {
            static std_msgs__msg__Int32 msg;
            msg.data++;
            rcl_publish(&pub, &msg, NULL);
        },
        true);
    if (rc != RCL_RET_OK)
        goto fail;

    rclc_executor_t exec;
    rc = rclc_executor_init(&exec, &support.context, 1, &alloc);
    if (rc != RCL_RET_OK)
        goto fail;

    rc = rclc_executor_add_timer(&exec, &timer);
    if (rc != RCL_RET_OK)
        goto fail;

    LOG_INF("micro-ROS running, publishing every %d ms", PUB_MS);
    while (true) {
        rclc_executor_spin_some(&exec, RCL_MS_TO_NS(100));
        k_msleep(10);
    }

fail:
    LOG_ERR("micro-ROS init failed: %d", rc);
}

/* ------------------------------------------------------------------ */
int main(void) {
    sta = net_if_get_default();
    if (!sta) {
        LOG_ERR("no network interface");
        return -ENODEV;
    }

    struct net_mgmt_event_callback cb;
    net_mgmt_init_event_callback(&cb, wifi_event_cb,
                                 NET_EVENT_WIFI_CONNECT_RESULT);
    net_mgmt_add_event_callback(&cb);

    struct wifi_connect_req_params params = {
        .ssid = WIFI_SSID,
        .ssid_length = strlen(WIFI_SSID),
        .psk = WIFI_PSK,
        .psk_length = strlen(WIFI_PSK),
        .security = WIFI_SECURITY_TYPE_PSK,
        .channel = WIFI_CHANNEL_ANY,
    };
    int ret = net_mgmt(NET_REQUEST_WIFI_CONNECT, sta, &params, sizeof(params));
    if (ret) {
        LOG_ERR("Wi-Fi connect request failed: %d", ret);
        return ret;
    }

    return 0;
}