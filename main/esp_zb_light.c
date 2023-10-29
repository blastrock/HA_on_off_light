/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 *
 * Zigbee customized client Example
 *
 * This example code is in the Public Domain (or CC0 licensed, at your option.)
 *
 * Unless required by applicable law or agreed to in writing, this
 * software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied.
 */
#include "esp_zb_light.h"
#include "driver/rtc_io.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "nvs_flash.h"
#include "string.h"
#include "zcl/esp_zigbee_zcl_common.h"
#ifdef CONFIG_PM_ENABLE
#include "esp_pm.h"
#include "esp_private/esp_clk.h"
#endif

char modelid[] = {11, 'S', 'o', 'm', 'f', 'y', 'S', 'w', 'i', 't', 'c', 'h'};
char manufname[] = {9, 'B', 'l', 'a', 's', 't', 'R', 'o', 'c', 'k'};

#if !defined ZB_ED_ROLE
#error Define ZB_ED_ROLE in idf.py menuconfig to compile light switch (End Device) source code.
#endif

static const char* TAG = "zigbee";

static switch_func_pair_t button_func_pair[] = {
    {GPIO_INPUT_IO_TOGGLE_SWITCH, SWITCH_ONOFF_TOGGLE_CONTROL}};
bool pendingAction;

static void esp_zb_buttons_handler(switch_func_pair_t* button_func_pair)
{
  switch (button_func_pair->func)
  {
  case SWITCH_ONOFF_TOGGLE_CONTROL:
  {
    /* send on-off toggle command to remote device */
    esp_zb_zcl_on_off_cmd_t cmd_req;
    cmd_req.zcl_basic_cmd.dst_addr_u.addr_short = 0;
    cmd_req.zcl_basic_cmd.dst_endpoint = 0;
    cmd_req.zcl_basic_cmd.src_endpoint = HA_ONOFF_SWITCH_ENDPOINT;
    cmd_req.address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
    cmd_req.on_off_cmd_id = ESP_ZB_ZCL_CMD_ON_OFF_TOGGLE_ID;
    // esp_zb_zcl_custom_cluster_cmd_req_t cmd_req;
    // cmd_req.zcl_basic_cmd.src_endpoint = HA_ONOFF_SWITCH_ENDPOINT;
    // cmd_req.address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
    // cmd_req.cluster_id = ESP_ZB_ZCL_CLUSTER_ID_COMMISSIONING;
    // cmd_req.custom_cmd_id = 0x34;
    // cmd_req.data_type = ESP_ZB_ZCL_ATTR_TYPE_NULL;
    ESP_EARLY_LOGI(TAG, "Send 'on_off toggle' command");
    // esp_zb_zcl_custom_cluster_cmd_req(&cmd_req);
    esp_zb_zcl_on_off_cmd_req(&cmd_req);
    ESP_EARLY_LOGI(TAG, "done!");
  }
  break;
  default:
    break;
  }
}

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
  ESP_ERROR_CHECK(esp_zb_bdb_start_top_level_commissioning(mode_mask));
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t* signal_struct)
{
  uint32_t* p_sg_p = signal_struct->p_app_signal;
  esp_err_t err_status = signal_struct->esp_err_status;
  esp_zb_app_signal_type_t sig_type = *p_sg_p;
  esp_zb_zdo_signal_leave_params_t* leave_params = NULL;

  ESP_LOGI(
      TAG,
      "ZDO signal: %s (0x%x), status: %s",
      esp_zb_zdo_signal_to_string(sig_type),
      sig_type,
      esp_err_to_name(err_status));

  switch (sig_type)
  {
  case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
    ESP_LOGI(TAG, "Zigbee stack initialized");
    esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
    break;
  case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
  case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
    if (err_status == ESP_OK)
    {
      ESP_LOGI(TAG, "Start network steering");
      esp_zb_bdb_start_top_level_commissioning(
          ESP_ZB_BDB_MODE_NETWORK_STEERING);
    }
    else
    {
      /* commissioning failed */
      ESP_LOGW(
          TAG, "Failed to initialize Zigbee stack (status: %d)", err_status);
    }
    break;
  case ESP_ZB_BDB_SIGNAL_STEERING:
    if (err_status == ESP_OK)
    {
      esp_zb_ieee_addr_t extended_pan_id;
      esp_zb_get_extended_pan_id(extended_pan_id);
      ESP_LOGI(
          TAG,
          "Joined network successfully (Extended PAN ID: "
          "%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, "
          "Channel:%d)",
          extended_pan_id[7],
          extended_pan_id[6],
          extended_pan_id[5],
          extended_pan_id[4],
          extended_pan_id[3],
          extended_pan_id[2],
          extended_pan_id[1],
          extended_pan_id[0],
          esp_zb_get_pan_id(),
          esp_zb_get_current_channel());

      if (pendingAction)
      {
        ESP_LOGI(TAG, "pending action to send!");
        esp_zb_buttons_handler(&button_func_pair[0]);
      }
    }
    else
    {
      ESP_LOGI(
          TAG, "Network steering was not successful (status: %d)", err_status);
      esp_zb_scheduler_alarm(
          (esp_zb_callback_t)bdb_start_top_level_commissioning_cb,
          ESP_ZB_BDB_MODE_NETWORK_STEERING,
          1000);
    }
    break;
  case ESP_ZB_ZDO_SIGNAL_LEAVE:
    leave_params =
        (esp_zb_zdo_signal_leave_params_t*)esp_zb_app_signal_get_params(p_sg_p);
    if (leave_params->leave_type == ESP_ZB_NWK_LEAVE_TYPE_RESET)
    {
      ESP_LOGI(TAG, "Reset device");
    }
    break;
  case ESP_ZB_COMMON_SIGNAL_CAN_SLEEP:
    //  ESP_ERROR_CHECK(
    //      gpio_wakeup_enable(GPIO_INPUT_IO_TOGGLE_SWITCH,
    //      GPIO_INTR_LOW_LEVEL));
    // ESP_ERROR_CHECK(esp_sleep_enable_ext1_wakeup(
    //     1ULL << GPIO_INPUT_IO_TOGGLE_SWITCH, ESP_EXT1_WAKEUP_ANY_HIGH));
    // rtc_gpio_pullup_dis(GPIO_INPUT_IO_TOGGLE_SWITCH);
    // rtc_gpio_pulldown_en(GPIO_INPUT_IO_TOGGLE_SWITCH);
    esp_zb_sleep_now();
    ESP_LOGI(TAG, "zigbee wakeup");
    // esp_light_sleep_start();
    // ESP_LOGI(TAG, "wake up cause %d", esp_sleep_get_wakeup_cause());

    // if (esp_sleep_get_wakeup_cause() & ESP_SLEEP_WAKEUP_GPIO)
    //   check_gpio(button_func_pair, PAIR_SIZE(button_func_pair));
    break;
  default:
    break;
  }
}

static void power_save_init(void)
{
#ifdef CONFIG_PM_ENABLE
  int cur_cpu_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ;
  esp_pm_config_t pm_config = {
    .max_freq_mhz = cur_cpu_freq_mhz,
    .min_freq_mhz = cur_cpu_freq_mhz,
#if CONFIG_FREERTOS_USE_TICKLESS_IDLE
    .light_sleep_enable = true
#endif
  };
  ESP_ERROR_CHECK(esp_pm_configure(&pm_config));
  ESP_ERROR_CHECK(
      esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON));
#endif
}

static esp_err_t zb_attribute_reporting_handler(
    const esp_zb_zcl_report_attr_message_t* message)
{
  ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
  ESP_RETURN_ON_FALSE(
      message->status == ESP_ZB_ZCL_STATUS_SUCCESS,
      ESP_ERR_INVALID_ARG,
      TAG,
      "Received message: error status(%d)",
      message->status);
  ESP_LOGI(
      TAG,
      "Reveived report from address(0x%x) src endpoint(%d) to dst endpoint(%d) "
      "cluster(0x%x)",
      message->src_address.u.short_addr,
      message->src_endpoint,
      message->dst_endpoint,
      message->cluster);
  ESP_LOGI(
      TAG,
      "Received report information: attribute(0x%x), type(0x%x), value(%d)\n",
      message->attribute.id,
      message->attribute.data.type,
      message->attribute.data.value ? *(uint8_t*)message->attribute.data.value
                                    : 0);
  return ESP_OK;
}

static esp_err_t zb_read_attr_resp_handler(
    const esp_zb_zcl_cmd_read_attr_resp_message_t* message)
{
  ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
  ESP_RETURN_ON_FALSE(
      message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS,
      ESP_ERR_INVALID_ARG,
      TAG,
      "Received message: error status(%d)",
      message->info.status);
  ESP_LOGI(
      TAG,
      "Read attribute response: status(%d), cluster(0x%x), attribute(0x%x), "
      "type(0x%x), value(%d)",
      message->info.status,
      message->info.cluster,
      message->attribute.id,
      message->attribute.data.type,
      message->attribute.data.value ? *(uint8_t*)message->attribute.data.value
                                    : 0);
  return ESP_OK;
}

static esp_err_t zb_configure_report_resp_handler(
    const esp_zb_zcl_cmd_config_report_resp_message_t* message)
{
  ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
  ESP_RETURN_ON_FALSE(
      message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS,
      ESP_ERR_INVALID_ARG,
      TAG,
      "Received message: error status(%d)",
      message->info.status);
  ESP_LOGI(
      TAG,
      "Configure report response: status(%d), cluster(0x%x), attribute(0x%x)",
      message->info.status,
      message->info.cluster,
      message->attribute_id);
  return ESP_OK;
}

static esp_err_t zb_action_handler(
    esp_zb_core_action_callback_id_t callback_id, const void* message)
{
  esp_err_t ret = ESP_OK;
  switch (callback_id)
  {
  case ESP_ZB_CORE_REPORT_ATTR_CB_ID:
    ret = zb_attribute_reporting_handler(
        (esp_zb_zcl_report_attr_message_t*)message);
    break;
  case ESP_ZB_CORE_CMD_READ_ATTR_RESP_CB_ID:
    ret = zb_read_attr_resp_handler(
        (esp_zb_zcl_cmd_read_attr_resp_message_t*)message);
    break;
  case ESP_ZB_CORE_CMD_REPORT_CONFIG_RESP_CB_ID:
    ret = zb_configure_report_resp_handler(
        (esp_zb_zcl_cmd_config_report_resp_message_t*)message);
    break;
  default:
    ESP_LOGW(TAG, "Receive Zigbee action(0x%x) callback", callback_id);
    break;
  }
  return ret;
}

static void esp_zb_task(void* pvParameters)
{
  /* initialize Zigbee stack */
  esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();
  /* Enable zigbee light sleep */
  esp_zb_sleep_enable(true);
  esp_zb_init(&zb_nwk_cfg);
  //   esp_zb_ieee_addr_t addr = {0x00, 0x00, 0x51, 0x09, 0x00, 0x00, 0x00,
  //   0x00}; esp_zb_set_long_address(addr);
  uint8_t test_attr;
  test_attr = 0;
  /* basic cluster create with fully customized */
  esp_zb_attribute_list_t* esp_zb_basic_cluster =
      esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_BASIC);
  esp_zb_basic_cluster_add_attr(
      esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_ZCL_VERSION_ID, &test_attr);
  esp_zb_basic_cluster_add_attr(
      esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_POWER_SOURCE_ID, &test_attr);
  esp_zb_cluster_update_attr(
      esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_ZCL_VERSION_ID, &test_attr);
  esp_zb_basic_cluster_add_attr(
      esp_zb_basic_cluster,
      ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID,
      &modelid[0]);
  esp_zb_basic_cluster_add_attr(
      esp_zb_basic_cluster,
      ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID,
      &manufname[0]);
  /* identify cluster create with fully customized */
  esp_zb_attribute_list_t* esp_zb_identify_cluster =
      esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY);
  esp_zb_identify_cluster_add_attr(
      esp_zb_identify_cluster,
      ESP_ZB_ZCL_ATTR_IDENTIFY_IDENTIFY_TIME_ID,
      &test_attr);
  /* create client role of the cluster */
  esp_zb_attribute_list_t* esp_zb_on_off_client_cluster =
      esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_ON_OFF);
  esp_zb_attribute_list_t* esp_zb_identify_client_cluster =
      esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY);
  /* create cluster lists for this endpoint */
  esp_zb_cluster_list_t* esp_zb_cluster_list = esp_zb_zcl_cluster_list_create();
  esp_zb_cluster_list_add_basic_cluster(
      esp_zb_cluster_list,
      esp_zb_basic_cluster,
      ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
  esp_zb_cluster_list_add_identify_cluster(
      esp_zb_cluster_list,
      esp_zb_identify_cluster,
      ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
  esp_zb_cluster_list_add_on_off_cluster(
      esp_zb_cluster_list,
      esp_zb_on_off_client_cluster,
      ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);
  esp_zb_cluster_list_add_identify_cluster(
      esp_zb_cluster_list,
      esp_zb_identify_client_cluster,
      ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);

  esp_zb_ep_list_t* esp_zb_ep_list = esp_zb_ep_list_create();
  ESP_ERROR_CHECK(esp_zb_ep_list_add_ep(
      esp_zb_ep_list,
      esp_zb_cluster_list,
      HA_ONOFF_SWITCH_ENDPOINT,
      ESP_ZB_AF_HA_PROFILE_ID,
      ESP_ZB_HA_ON_OFF_SWITCH_DEVICE_ID));
  // esp_zb_device_register(esp_zb_ep_list);

  esp_zb_on_off_light_cfg_t light_cfg = ESP_ZB_DEFAULT_ON_OFF_LIGHT_CONFIG();
  esp_zb_ep_list_t* esp_zb_on_off_light_ep =
      esp_zb_on_off_light_ep_create(2, &light_cfg);
  // esp_zb_device_register(esp_zb_on_off_light_ep);
  ESP_ERROR_CHECK(esp_zb_device_register(esp_zb_ep_list));
  esp_zb_core_action_handler_register(zb_action_handler);
  ESP_ERROR_CHECK(
      esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK));
  // esp_zb_set_secondary_network_channel_set(ESP_ZB_SECONDARY_CHANNEL_MASK);
  ESP_ERROR_CHECK(esp_zb_start(false));
  esp_zb_main_loop_iteration();
}

static void deep_sleep_task(void* args)
{
  switch (esp_sleep_get_wakeup_cause())
  {
  case ESP_SLEEP_WAKEUP_GPIO:
  {
    uint64_t wakeup_pin_mask = esp_sleep_get_gpio_wakeup_status();
    if (wakeup_pin_mask != 0)
    {
      int pin = __builtin_ffsll(wakeup_pin_mask) - 1;
      ESP_LOGI(TAG, "Wake up from GPIO %d", pin);

      pendingAction = true;
    }
    else
    {
      ESP_LOGI(TAG, "Wake up from GPIO unknown %lld", wakeup_pin_mask);
    }
    break;
  }
  default:
    ESP_LOGI(TAG, "unknown wake up reason: %d", esp_sleep_get_wakeup_cause());
  }
  vTaskDelay(10000 / portTICK_PERIOD_MS);

  ESP_EARLY_LOGI(TAG, "Entering deep sleep");

  // enter deep sleep
  esp_deep_sleep_start();
}

void app_main(void)
{
  uint64_t wakeup_pin_mask = esp_sleep_get_gpio_wakeup_status();
  ESP_EARLY_LOGI(TAG, "wake up early %lld", wakeup_pin_mask);

  esp_zb_platform_config_t config = {
      .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
      .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
  };
  ESP_ERROR_CHECK(nvs_flash_init());
  /* esp zigbee light sleep initialization*/
  power_save_init();
  ESP_ERROR_CHECK(esp_zb_platform_config(&config));
  // ESP_ERROR_CHECK(
  //     esp_deep_sleep_enable_gpio_wakeup(BIT(0), ESP_GPIO_WAKEUP_GPIO_HIGH));
  if (!switch_driver_init(
          button_func_pair,
          PAIR_SIZE(button_func_pair),
          esp_zb_buttons_handler))
  {
    ESP_LOGI(TAG, "Failed to set up GPIOs");
    abort();
  }

  xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);

  // xTaskCreate(deep_sleep_task, "deep_sleep_task", 4096, NULL, 6, NULL);
}
