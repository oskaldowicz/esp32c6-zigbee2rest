#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_wifi.h"
#include "esp_timer.h"
#include "esp_coexist.h"
#include "esp_zigbee_core.h"
#include "zcl/esp_zigbee_zcl_power_config.h"
#include "zdo/esp_zigbee_zdo_command.h"
#include "zcl/esp_zigbee_zcl_command.h"

#define TAG "ZB_GW"

#define GATEWAY_ENDPOINT        1
#define MAX_CHILDREN            10
#define INSTALLCODE_POLICY      false

#if CONFIG_GATEWAY_ZIGBEE_CHANNEL
#define ZIGBEE_CHANNEL_MASK     (1l << CONFIG_GATEWAY_ZIGBEE_CHANNEL)
#else
#define ZIGBEE_CHANNEL_MASK     (1l << 13)
#endif

#if CONFIG_GATEWAY_MAX_SENSORS
#define MAX_SENSORS             CONFIG_GATEWAY_MAX_SENSORS
#else
#define MAX_SENSORS             10
#endif

typedef struct {
    uint16_t short_addr;
    esp_zb_ieee_addr_t ieee_addr;
    float temperature;
    float humidity;
    uint8_t battery;
    int64_t last_seen;
    bool valid;
    bool bound;
} sensor_t;

static sensor_t sensors[MAX_SENSORS];
static int sensor_count = 0;
static SemaphoreHandle_t sensor_mutex;

#define MAX_ENDPOINTS 10

typedef struct {
    uint16_t short_addr;
    esp_zb_ieee_addr_t ieee_addr;
    uint8_t ep_list[MAX_ENDPOINTS];
    uint8_t ep_count;
    int current_ep_idx;
    bool rediscovery;
} discovery_ctx_t;

static void discovery_alarm_cb(void *param);
static void startup_rediscover_cb(void *param);
static void mgmt_lqi_cb(const esp_zb_zdo_mgmt_lqi_rsp_t *rsp, void *user_ctx);
static void schedule_next_rediscovery(void);

#define NVS_SENSOR_NAMESPACE "sensors"

static void save_sensor_list(void)
{
    nvs_handle_t nvs;
    if (nvs_open(NVS_SENSOR_NAMESPACE, NVS_READWRITE, &nvs) == ESP_OK) {
        for (int i = 0; i < sensor_count; i++) {
            if (!sensors[i].valid) continue;
            char key[16];
            snprintf(key, sizeof(key), "s%d", i);
            nvs_set_blob(nvs, key, sensors[i].ieee_addr, sizeof(esp_zb_ieee_addr_t));
        }
        nvs_set_u16(nvs, "count", sensor_count);
        nvs_commit(nvs);
        nvs_close(nvs);
    }
}

static void load_sensor_list(void)
{
    nvs_handle_t nvs;
    if (nvs_open(NVS_SENSOR_NAMESPACE, NVS_READONLY, &nvs) == ESP_OK) {
        uint16_t count = 0;
        if (nvs_get_u16(nvs, "count", &count) == ESP_OK && count > 0) {
            sensor_count = 0;
            for (int i = 0; i < count && sensor_count < MAX_SENSORS; i++) {
                char key[16];
                snprintf(key, sizeof(key), "s%d", i);
                size_t len = sizeof(esp_zb_ieee_addr_t);
                if (nvs_get_blob(nvs, key, sensors[sensor_count].ieee_addr, &len) == ESP_OK) {
                    sensors[sensor_count].short_addr = 0xFFFF;
                    sensors[sensor_count].temperature = -273.0f;
                    sensors[sensor_count].humidity = -1.0f;
                    sensors[sensor_count].battery = 0;
                    sensors[sensor_count].valid = true;
                    sensors[sensor_count].bound = false;
                    sensors[sensor_count].last_seen = 0;
                    sensor_count++;
                }
            }
            ESP_LOGI(TAG, "Loaded %d sensors from NVS", sensor_count);
        }
        nvs_close(nvs);
    }
}

static int find_sensor_by_ieee(esp_zb_ieee_addr_t ieee_addr)
{
    for (int i = 0; i < sensor_count; i++) {
        if (sensors[i].valid && memcmp(sensors[i].ieee_addr, ieee_addr, sizeof(esp_zb_ieee_addr_t)) == 0)
            return i;
    }
    return -1;
}

static int add_or_update_sensor(uint16_t short_addr, esp_zb_ieee_addr_t ieee_addr)
{
    int idx = find_sensor_by_ieee(ieee_addr);
    if (idx >= 0) {
        if (sensors[idx].short_addr != short_addr) {
            ESP_LOGI(TAG, "Sensor %d changed short 0x%04x -> 0x%04x",
                     idx, sensors[idx].short_addr, short_addr);
            sensors[idx].short_addr = short_addr;
        }
        return idx;
    }
    if (sensor_count >= MAX_SENSORS)
        return -1;
    idx = sensor_count++;
    sensors[idx].short_addr = short_addr;
    memcpy(sensors[idx].ieee_addr, ieee_addr, sizeof(esp_zb_ieee_addr_t));
    sensors[idx].temperature = -273.0f;
    sensors[idx].humidity = -1.0f;
    sensors[idx].battery = 0;
    sensors[idx].valid = true;
    sensors[idx].bound = false;
    sensors[idx].last_seen = 0;
    return idx;
}

static int find_sensor_by_short(uint16_t short_addr)
{
    for (int i = 0; i < sensor_count; i++) {
        if (sensors[i].valid && sensors[i].short_addr == short_addr)
            return i;
    }
    return -1;
}

#define BIND_CLUSTERS 3

static const uint16_t BIND_CLUSTER_IDS[BIND_CLUSTERS] = {
    ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
    ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT,
    ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG,
};

typedef struct {
    uint16_t short_addr;
    uint8_t ep;
    uint8_t cluster_idx;
    bool rediscovery;
} config_step_t;

static void config_report_step_cb(void *param)
{
    config_step_t *step = param;

    if (step->cluster_idx >= BIND_CLUSTERS) {
        ESP_LOGI(TAG, "Configured reporting for sensor 0x%04x", step->short_addr);
        int s_idx = find_sensor_by_short(step->short_addr);
        if (s_idx >= 0)
            sensors[s_idx].bound = true;
        bool was_rediscovery = step->rediscovery;
        free(step);
        if (was_rediscovery)
            schedule_next_rediscovery();
        return;
    }

    uint16_t cluster = BIND_CLUSTER_IDS[step->cluster_idx];
    esp_zb_zcl_config_report_record_t record = {0};
    esp_zb_zcl_config_report_cmd_t cmd = {0};
    int16_t temp_change = 50;
    uint16_t hum_change = 0;
    uint8_t batt_change = 0;

    cmd.zcl_basic_cmd.dst_addr_u.addr_short = step->short_addr;
    cmd.zcl_basic_cmd.dst_endpoint = step->ep;
    cmd.zcl_basic_cmd.src_endpoint = GATEWAY_ENDPOINT;
    cmd.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
    cmd.clusterID = cluster;
    cmd.record_number = 1;
    cmd.record_field = &record;

    record.direction = ESP_ZB_ZCL_REPORT_DIRECTION_SEND;

    if (cluster == ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT) {
        record.attributeID = ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID;
        record.attrType = ESP_ZB_ZCL_ATTR_TYPE_S16;
        record.min_interval = 1;
        record.max_interval = 60;
        record.reportable_change = &temp_change;
    } else if (cluster == ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT) {
        record.attributeID = ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID;
        record.attrType = ESP_ZB_ZCL_ATTR_TYPE_U16;
        record.min_interval = 1;
        record.max_interval = 60;
        record.reportable_change = &hum_change;
    } else if (cluster == ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG) {
        record.attributeID = ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID;
        record.attrType = ESP_ZB_ZCL_ATTR_TYPE_U8;
        record.min_interval = 1;
        record.max_interval = 360;
        record.reportable_change = &batt_change;
    } else {
        step->cluster_idx++;
        esp_zb_scheduler_user_alarm(config_report_step_cb, step, 200);
        return;
    }

    esp_zb_zcl_config_report_cmd_req(&cmd);
    ESP_LOGI(TAG, "Config report 0x%04x cluster 0x%04x (step %d/%d)",
             step->short_addr, cluster, step->cluster_idx + 1, BIND_CLUSTERS);

    step->cluster_idx++;
    esp_zb_scheduler_user_alarm(config_report_step_cb, step, 200);
}

static void configure_sensor_reporting(discovery_ctx_t *ctx)
{
    config_step_t *step = malloc(sizeof(config_step_t));
    if (!step) {
        ESP_LOGE(TAG, "OOM for config_step_t");
        return;
    }
    step->short_addr = ctx->short_addr;
    step->ep = ctx->ep_list[ctx->current_ep_idx];
    step->cluster_idx = 0;
    step->rediscovery = ctx->rediscovery;
    esp_zb_scheduler_user_alarm(config_report_step_cb, step, 0);
}

static void simple_desc_cb(esp_zb_zdp_status_t status, esp_zb_af_simple_desc_1_1_t *simple_desc, void *user_ctx)
{
    discovery_ctx_t *ctx = user_ctx;
    if (status == ESP_ZB_ZDP_STATUS_SUCCESS && simple_desc) {
        uint16_t *clusters = simple_desc->app_cluster_list;
        int total = simple_desc->app_input_cluster_count + simple_desc->app_output_cluster_count;
        for (int i = 0; i < total; i++) {
            if (clusters[i] == ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT ||
                clusters[i] == ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT ||
                clusters[i] == ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG) {
                uint8_t ep = simple_desc->endpoint;
                ESP_LOGI(TAG, "Found sensor cluster 0x%04x on ep %d for 0x%04x",
                         clusters[i], ep, ctx->short_addr);
                ctx->ep_list[0] = ep;
                ctx->ep_count = 1;
                ctx->current_ep_idx = 0;
                configure_sensor_reporting(ctx);
                free(ctx);
                return;
            }
        }
    }
    ctx->current_ep_idx++;
    if (ctx->current_ep_idx < ctx->ep_count) {
        esp_zb_zdo_simple_desc_req_param_t req = {
            .addr_of_interest = ctx->short_addr,
            .endpoint = ctx->ep_list[ctx->current_ep_idx],
        };
        esp_zb_zdo_simple_desc_req(&req, simple_desc_cb, ctx);
    } else {
        ESP_LOGE(TAG, "No sensor endpoint via Simple Desc for 0x%04x", ctx->short_addr);
        bool was_rediscovery = ctx->rediscovery;
        free(ctx);
        if (was_rediscovery)
            schedule_next_rediscovery();
    }
}

static void match_cb(esp_zb_zdp_status_t status, uint16_t addr, uint8_t endpoint, void *user_ctx)
{
    discovery_ctx_t *ctx = user_ctx;
    if (status == ESP_ZB_ZDP_STATUS_SUCCESS && endpoint != 0xFF) {
        ESP_LOGI(TAG, "Found sensor endpoint %d for 0x%04x", endpoint, addr);
        ctx->ep_list[0] = endpoint;
        ctx->ep_count = 1;
        ctx->current_ep_idx = 0;
        configure_sensor_reporting(ctx);
        free(ctx);
    } else {
        ESP_LOGW(TAG, "Match desc failed for 0x%04x, fallback to Simple Desc", ctx->short_addr);
        ctx->current_ep_idx = 0;
        if (ctx->ep_count > 0) {
            esp_zb_zdo_simple_desc_req_param_t req = {
                .addr_of_interest = ctx->short_addr,
                .endpoint = ctx->ep_list[0],
            };
            esp_zb_zdo_simple_desc_req(&req, simple_desc_cb, ctx);
        } else {
            ESP_LOGE(TAG, "No endpoints for 0x%04x", ctx->short_addr);
            bool was_rediscovery = ctx->rediscovery;
            free(ctx);
            if (was_rediscovery)
                schedule_next_rediscovery();
        }
    }
}

static void active_ep_cb(esp_zb_zdp_status_t status, uint8_t ep_count, uint8_t *ep_id_list, void *user_ctx)
{
    discovery_ctx_t *ctx = user_ctx;
    if (status != ESP_ZB_ZDP_STATUS_SUCCESS || ep_count == 0) {
        ESP_LOGW(TAG, "Active EP failed for 0x%04x status=%d", ctx->short_addr, status);
        bool was_rediscovery = ctx->rediscovery;
        free(ctx);
        if (was_rediscovery)
            schedule_next_rediscovery();
        return;
    }
    ctx->ep_count = (ep_count > MAX_ENDPOINTS) ? MAX_ENDPOINTS : ep_count;
    memcpy(ctx->ep_list, ep_id_list, ctx->ep_count);

    static const uint16_t match_clusters[] = { ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT };
    esp_zb_zdo_match_desc_req_param_t match = {
        .dst_nwk_addr = ctx->short_addr,
        .addr_of_interest = ctx->short_addr,
        .profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .num_in_clusters = 1,
        .num_out_clusters = 0,
        .cluster_list = (uint16_t *)match_clusters,
    };
    esp_zb_zdo_match_cluster(&match, match_cb, ctx);
}

static int rediscovery_pending_idx = 0;

static void schedule_next_rediscovery(void)
{
    for (int i = rediscovery_pending_idx; i < sensor_count; i++) {
        if (sensors[i].valid && !sensors[i].bound && sensors[i].short_addr != 0xFFFF) {
            ESP_LOGI(TAG, "Post-startup rediscovery for sensor %d (0x%04x)", i, sensors[i].short_addr);
            discovery_ctx_t *ctx = malloc(sizeof(discovery_ctx_t));
            if (ctx) {
                ctx->short_addr = sensors[i].short_addr;
                memcpy(ctx->ieee_addr, sensors[i].ieee_addr, sizeof(esp_zb_ieee_addr_t));
                ctx->ep_count = 0;
                ctx->current_ep_idx = 0;
                ctx->rediscovery = true;
                rediscovery_pending_idx = i + 1;
                esp_zb_scheduler_user_alarm(discovery_alarm_cb, ctx, 2000);
            }
            return;
        }
    }
    ESP_LOGI(TAG, "Post-startup rediscovery complete");
}

static void mgmt_lqi_cb(const esp_zb_zdo_mgmt_lqi_rsp_t *rsp, void *user_ctx)
{
    if (rsp->status != 0) {
        ESP_LOGW(TAG, "Mgmt LQI failed status=%d", rsp->status);
        return;
    }
    ESP_LOGI(TAG, "Mgmt LQI: %d neighbors (total %d, start %d)",
             rsp->neighbor_table_list_count, rsp->neighbor_table_entries, rsp->start_index);

    for (uint8_t i = 0; i < rsp->neighbor_table_list_count; i++) {
        const esp_zb_zdo_neighbor_table_list_record_t *record = &rsp->neighbor_table_list[i];
        const uint8_t *ieee = record->extended_addr;
        ESP_LOGI(TAG, "  Neighbor 0x%04x IEEE=%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x",
                 record->network_addr,
                 ieee[0], ieee[1], ieee[2], ieee[3],
                 ieee[4], ieee[5], ieee[6], ieee[7]);

        for (int j = 0; j < sensor_count; j++) {
            if (sensors[j].valid && !sensors[j].bound &&
                memcmp(sensors[j].ieee_addr, record->extended_addr, sizeof(esp_zb_ieee_addr_t)) == 0) {
                if (sensors[j].short_addr == 0xFFFF) {
                    sensors[j].short_addr = record->network_addr;
                    ESP_LOGI(TAG, "Resolved short_addr for sensor %d: 0x%04x", j, record->network_addr);
                }
                break;
            }
        }
    }

    uint8_t next_start = rsp->start_index + rsp->neighbor_table_list_count;
    if (next_start < rsp->neighbor_table_entries) {
        esp_zb_zdo_mgmt_lqi_req_param_t req = { .start_index = next_start, .dst_addr = 0x0000 };
        esp_zb_zdo_mgmt_lqi_req(&req, mgmt_lqi_cb, NULL);
        return;
    }

    rediscovery_pending_idx = 0;
    schedule_next_rediscovery();
}

static void startup_rediscover_cb(void *param)
{
    ESP_LOGI(TAG, "Starting post-startup sensor rediscovery via Mgmt LQI");
    esp_zb_zdo_mgmt_lqi_req_param_t req = {
        .start_index = 0,
        .dst_addr = 0x0000,
    };
    esp_zb_zdo_mgmt_lqi_req(&req, mgmt_lqi_cb, NULL);
}

static void discovery_alarm_cb(void *param)
{
    discovery_ctx_t *ctx = param;
    esp_zb_zdo_active_ep_req_param_t req = {
        .addr_of_interest = ctx->short_addr,
    };
    esp_zb_zdo_active_ep_req(&req, active_ep_cb, ctx);
}

static void update_sensor(uint16_t short_addr, uint16_t cluster, uint16_t attr_id, const void *value)
{
    int idx = find_sensor_by_short(short_addr);
    if (idx < 0)
        return;
    sensors[idx].last_seen = esp_timer_get_time() / 1000;
    if (cluster == ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT &&
        attr_id == ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID) {
        int16_t raw = *(const int16_t *)value;
        sensors[idx].temperature = raw / 100.0f;
        ESP_LOGI(TAG, "Temp 0x%04x: %.2f C", short_addr, sensors[idx].temperature);
    } else if (cluster == ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT &&
               attr_id == ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID) {
        uint16_t raw = *(const uint16_t *)value;
        if (raw != 0xFFFF)
            sensors[idx].humidity = raw / 100.0f;
        ESP_LOGI(TAG, "Humidity 0x%04x: %.1f%%", short_addr, sensors[idx].humidity);
    } else if (cluster == ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG) {
        if (attr_id == ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID) {
            sensors[idx].battery = *(const uint8_t *)value / 2;
            ESP_LOGI(TAG, "Battery 0x%04x: %d%%", short_addr, sensors[idx].battery);
        } else if (attr_id == ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_ID) {
            uint8_t mv = *(const uint8_t *)value;
            sensors[idx].battery = (mv - 20) * 100 / 10;
            if (sensors[idx].battery > 100) sensors[idx].battery = 100;
            ESP_LOGI(TAG, "Battery 0x%04x: %dmV (%d%%)", short_addr, mv * 100, sensors[idx].battery);
        }
    } else {
        ESP_LOGI(TAG, "Unknown report 0x%04x cluster 0x%04x attr 0x%04x", short_addr, cluster, attr_id);
    }
}

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    if (callback_id == ESP_ZB_CORE_REPORT_ATTR_CB_ID) {
        const esp_zb_zcl_report_attr_message_t *msg = message;
        uint16_t short_addr = msg->src_address.u.short_addr;
        if (xSemaphoreTake(sensor_mutex, pdMS_TO_TICKS(100))) {
            update_sensor(short_addr, msg->cluster, msg->attribute.id, msg->attribute.data.value);
            xSemaphoreGive(sensor_mutex);
        }
    }
    return ESP_OK;
}

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    esp_zb_bdb_start_top_level_commissioning(mode_mask);
}

static httpd_handle_t start_http_server(void);

static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *disconn = data;
        ESP_LOGI(TAG, "Wi-Fi disconnected (reason %d), retrying...", disconn->reason);
        esp_wifi_connect();
    } else if (id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
    }
}

static void wifi_init_after_zigbee(void)
{
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED,
                     &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                     &wifi_event_handler, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_GATEWAY_WIFI_SSID,
            .password = CONFIG_GATEWAY_WIFI_PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    esp_wifi_connect();
    ESP_LOGI(TAG, "Wi-Fi connecting to %s...", CONFIG_GATEWAY_WIFI_SSID);
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;

    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        esp_coex_wifi_i154_enable();
        wifi_init_after_zigbee();
        start_http_server();
        ESP_LOGI(TAG, "Initialize Zigbee stack");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;

    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK) {
            bool factory_new = esp_zb_bdb_is_factory_new();
            ESP_LOGI(TAG, "Device started (factory_new=%d, PAN=0x%04x)",
                     factory_new, esp_zb_get_pan_id());
            if (factory_new) {
                ESP_LOGI(TAG, "Start network formation");
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_FORMATION);
            } else {
                esp_zb_bdb_open_network(180);
                ESP_LOGI(TAG, "Device rebooted, PAN=0x%04x restored, network open for 180s",
                         esp_zb_get_pan_id());
                esp_zb_scheduler_user_alarm(startup_rediscover_cb, NULL, 10000);
            }
        } else {
            ESP_LOGE(TAG, "Failed to initialize Zigbee stack: %s", esp_err_to_name(err_status));
        }
        break;

    case ESP_ZB_BDB_SIGNAL_FORMATION:
        if (err_status == ESP_OK) {
            esp_zb_ieee_addr_t ieee;
            esp_zb_get_long_address(ieee);
            ESP_LOGI(TAG, "Network formed - PAN: 0x%04x, Ch: %d, Addr: 0x%04x",
                     esp_zb_get_pan_id(), esp_zb_get_current_channel(),
                     esp_zb_get_short_address());
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            esp_zb_scheduler_user_alarm(startup_rediscover_cb, NULL, 10000);
        } else {
            esp_zb_scheduler_alarm(bdb_start_top_level_commissioning_cb,
                                   ESP_ZB_BDB_MODE_NETWORK_FORMATION, 1000);
        }
        break;

    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Network steering started");
            esp_zb_bdb_open_network(180);
            ESP_LOGI(TAG, "Network open for 180 seconds - devices can join");
        }
        break;

    case ESP_ZB_ZDO_SIGNAL_DEVICE_ANNCE: {
        esp_zb_zdo_signal_device_annce_params_t *params =
            esp_zb_app_signal_get_params(p_sg_p);
        ESP_LOGI(TAG, "Device joined - short: 0x%04x", params->device_short_addr);

        if (xSemaphoreTake(sensor_mutex, pdMS_TO_TICKS(100))) {
            int idx = add_or_update_sensor(params->device_short_addr, params->ieee_addr);
            if (idx >= 0) {
                ESP_LOGI(TAG, "Sensor 0x%04x at index %d", params->device_short_addr, idx);
                save_sensor_list();
            } else {
                ESP_LOGW(TAG, "Sensor list full");
            }
            xSemaphoreGive(sensor_mutex);
        }

        // Immediate configure_reporting to endpoint 1 (no discovery delay)
        if (xSemaphoreTake(sensor_mutex, pdMS_TO_TICKS(100))) {
            int sidx = find_sensor_by_ieee(params->ieee_addr);
            if (sidx >= 0 && !sensors[sidx].bound) {
                discovery_ctx_t *ictx = malloc(sizeof(discovery_ctx_t));
                if (ictx) {
                    ictx->short_addr = params->device_short_addr;
                    memcpy(ictx->ieee_addr, params->ieee_addr, sizeof(esp_zb_ieee_addr_t));
                    ictx->ep_list[0] = 1;
                    ictx->ep_count = 1;
                    ictx->current_ep_idx = 0;
                    ictx->rediscovery = false;
                    configure_sensor_reporting(ictx);
                    free(ictx);
                }
            }
            xSemaphoreGive(sensor_mutex);
        }

        // Delayed discovery as fallback (for sensors with endpoint != 1)
        discovery_ctx_t *ctx = malloc(sizeof(discovery_ctx_t));
        if (ctx) {
            ctx->short_addr = params->device_short_addr;
            memcpy(ctx->ieee_addr, params->ieee_addr, sizeof(esp_zb_ieee_addr_t));
            ctx->ep_count = 0;
            ctx->current_ep_idx = 0;
            ctx->rediscovery = false;
            esp_zb_scheduler_user_alarm(discovery_alarm_cb, ctx, 2000);
        } else {
            ESP_LOGE(TAG, "OOM for discovery_ctx");
        }
        break;
    }

    case ESP_ZB_NWK_SIGNAL_PERMIT_JOIN_STATUS:
        if (err_status == ESP_OK) {
            uint8_t *duration = esp_zb_app_signal_get_params(p_sg_p);
            if (duration && *duration)
                ESP_LOGI(TAG, "Network open for %d seconds", *duration);
            else
                ESP_LOGW(TAG, "Network closed for joining");
        }
        break;

    case ESP_ZB_ZDO_SIGNAL_PRODUCTION_CONFIG_READY:
        esp_zb_set_node_descriptor_manufacturer_code(0x131B);
        break;

    default:
        ESP_LOGI(TAG, "ZDO signal: 0x%x, status: %s", sig_type, esp_err_to_name(err_status));
        break;
    }
}

static void zigbee_task(void *pvParameters)
{
    esp_zb_cfg_t zb_nwk_cfg = {
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_COORDINATOR,
        .install_code_policy = INSTALLCODE_POLICY,
        .nwk_cfg.zczr_cfg = {
            .max_children = MAX_CHILDREN,
        },
    };
    esp_zb_aps_src_binding_table_size_set(32);
    esp_zb_aps_dst_binding_table_size_set(32);
    esp_zb_init(&zb_nwk_cfg);
    esp_zb_set_primary_network_channel_set(ZIGBEE_CHANNEL_MASK);

    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();

    esp_zb_endpoint_config_t endpoint_cfg = {
        .endpoint = GATEWAY_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_HOME_GATEWAY_DEVICE_ID,
        .app_device_version = 0,
    };

    esp_zb_attribute_list_t *basic_cluster = esp_zb_basic_cluster_create(NULL);
    esp_zb_cluster_list_add_basic_cluster(cluster_list, basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_identify_cluster(cluster_list, esp_zb_identify_cluster_create(NULL),
                                             ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    esp_zb_attribute_list_t *temp_meas_cluster = esp_zb_temperature_meas_cluster_create(NULL);
    esp_zb_cluster_list_add_temperature_meas_cluster(cluster_list, temp_meas_cluster,
                                                     ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);
    esp_zb_attribute_list_t *humidity_cluster = esp_zb_humidity_meas_cluster_create(NULL);
    esp_zb_cluster_list_add_humidity_meas_cluster(cluster_list, humidity_cluster,
                                                   ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);
    esp_zb_attribute_list_t *power_cluster = esp_zb_power_config_cluster_create(NULL);
    esp_zb_cluster_list_add_power_config_cluster(cluster_list, power_cluster,
                                                  ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);

    esp_zb_ep_list_add_ep(ep_list, cluster_list, endpoint_cfg);
    esp_zb_device_register(ep_list);

    esp_zb_core_action_handler_register(zb_action_handler);

    esp_zb_nvram_erase_at_start(false);
    ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_stack_main_loop();
    vTaskDelete(NULL);
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    sensor_mutex = xSemaphoreCreateMutex();

    esp_zb_platform_config_t platform_config = {
        .radio_config = {
            .radio_mode = ZB_RADIO_MODE_NATIVE,
        },
        .host_config = {
            .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE,
        },
    };
    ESP_ERROR_CHECK(esp_zb_platform_config(&platform_config));

    load_sensor_list();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    xTaskCreate(zigbee_task, "Zigbee", 8192, NULL, 5, NULL);

    ESP_LOGI(TAG, "System started. Wi-Fi: %s, Zigbee coordinator on ch %d",
             CONFIG_GATEWAY_WIFI_SSID, CONFIG_GATEWAY_ZIGBEE_CHANNEL);
}

static void sensor_to_json(char *buf, size_t size, int i)
{
    snprintf(buf, size,
             "{\"id\":%d,\"short_addr\":\"0x%04x\",\"temperature\":%.2f,\"humidity\":%.1f,"
             "\"battery\":%d,\"last_seen\":%lld}",
             i, sensors[i].short_addr, sensors[i].temperature,
             sensors[i].humidity, sensors[i].battery, sensors[i].last_seen);
}

static esp_err_t sensors_list_handler(httpd_req_t *req)
{
    char buf[2048];
    int off = 0;
    off += snprintf(buf + off, sizeof(buf) - off, "[");

    if (xSemaphoreTake(sensor_mutex, pdMS_TO_TICKS(500))) {
        bool first = true;
        for (int i = 0; i < sensor_count; i++) {
            if (!sensors[i].valid)
                continue;
            if (!first)
                off += snprintf(buf + off, sizeof(buf) - off, ",");
            first = false;
            sensor_to_json(buf + off, sizeof(buf) - off, i);
            off = strlen(buf);
        }
        xSemaphoreGive(sensor_mutex);
    }
    off += snprintf(buf + off, sizeof(buf) - off, "]");

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, buf, off);
    return ESP_OK;
}

static esp_err_t sensor_detail_handler(httpd_req_t *req)
{
    char buf[512];
    const char *id_str = req->uri + strlen("/api/sensors/");
    int id = atoi(id_str);
    int matched = 0;

    if (xSemaphoreTake(sensor_mutex, pdMS_TO_TICKS(500))) {
        if (id >= 0 && id < sensor_count && sensors[id].valid) {
            matched = 1;
            sensor_to_json(buf, sizeof(buf), id);
        }
        xSemaphoreGive(sensor_mutex);
    }

    if (!matched) {
        httpd_resp_set_status(req, "404 Not Found");
        httpd_resp_sendstr(req, "{\"error\":\"sensor not found\"}");
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, buf, strlen(buf));
    return ESP_OK;
}

static esp_err_t join_handler(httpd_req_t *req)
{
    esp_zb_bdb_open_network(180);
    ESP_LOGI(TAG, "Network opened for 180s via API");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"status\":\"network opened for 180 seconds\"}");
    return ESP_OK;
}

static esp_err_t temp_handler(httpd_req_t *req)
{
    char json[128];
    float temp = -273.0f;

    if (xSemaphoreTake(sensor_mutex, pdMS_TO_TICKS(100))) {
        for (int i = 0; i < sensor_count; i++) {
            if (sensors[i].valid) {
                temp = sensors[i].temperature;
                break;
            }
        }
        xSemaphoreGive(sensor_mutex);
    }

    snprintf(json, sizeof(json), "{\"temperature\": %.2f}", temp);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static const httpd_uri_t uris[] = {
    { .uri = "/temperature", .method = HTTP_GET, .handler = temp_handler },
    { .uri = "/api/sensors", .method = HTTP_GET, .handler = sensors_list_handler },
    { .uri = "/api/sensors/*", .method = HTTP_GET, .handler = sensor_detail_handler },
    { .uri = "/api/join", .method = HTTP_POST, .handler = join_handler },
};

static httpd_handle_t start_http_server(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 8;
    httpd_handle_t srv = NULL;

    if (httpd_start(&srv, &config) == ESP_OK) {
        for (size_t i = 0; i < sizeof(uris) / sizeof(uris[0]); i++)
            httpd_register_uri_handler(srv, &uris[i]);
        ESP_LOGI(TAG, "HTTP server started");
    }
    return srv;
}
