#include "jk_bms.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatt_common_api.h"
#include "esp_gattc_api.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include <stdlib.h>
#include <string.h>

// Удалены собственные реализации функций сканирования - они вызывали панику
// Используем прямое подключение через GATT Client без предварительного
// сканирования

static const char *TAG = "JK_BMS";

// MAC адрес BMS (без двоеточий)
static uint8_t bms_addr[6] = {0xC8, 0x47, 0x80, 0x21, 0x7F, 0x97};

// GATT Client параметры
#define GATTC_TAG "JK_BMS_GATTC"
#define PROFILE_NUM 1
#define PROFILE_APP_IDX 0
#define INVALID_HANDLE 0
#define CONNECT_TIMEOUT_MS 10000

// UUID сервисов и характеристик JK BMS (из рабочей версии)
// JK BMS использует стандартные UUID
static esp_bt_uuid_t service_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {.uuid16 =
                 0xFFE0} // Service UUID: 0000ffe0-0000-1000-8000-00805f9b34fb
};

// Characteristic UUID: 0000ffe1-0000-1000-8000-00805f9b34fb
static esp_bt_uuid_t char_uuid = {.len = ESP_UUID_LEN_16,
                                  .uuid = {.uuid16 = 0xFFE1}};

// Структура профиля GATT Client
struct gattc_profile_inst {
  esp_gattc_cb_t gattc_cb;
  esp_gatt_if_t gattc_if;
  uint16_t app_id;
  uint16_t conn_id;
  uint16_t service_start_handle;
  uint16_t service_end_handle;
  uint16_t char_handle;
  esp_bd_addr_t remote_bda;
};

static struct gattc_profile_inst profile_tab[PROFILE_NUM] = {
    [PROFILE_APP_IDX] =
        {
            .gattc_cb = NULL,
            .gattc_if = ESP_GATT_IF_NONE,
        },
};

// Глобальные переменные
static jk_bms_status_t bms_status = JK_BMS_DISCONNECTED;
static jk_bms_data_t bms_data = {0};
static EventGroupHandle_t ble_event_group = NULL;
static const int CONNECTED_BIT = BIT0;
static const int DISCONNECTED_BIT = BIT1;

// Буфер для данных BMS (JK-BMS отправляет данные частями, ~300 байт)
#define BMS_BUFFER_SIZE 512
static uint8_t bms_buffer[BMS_BUFFER_SIZE];
static size_t bms_buffer_pos = 0;

#define MIN_RESPONSE_SIZE 300
#define MAX_RESPONSE_SIZE 400

// CRC calculation (sum of all bytes except CRC byte)
static uint8_t calculate_crc(const uint8_t *data, uint16_t len) {
  uint8_t crc = 0;
  for (uint16_t i = 0; i < len; i++) {
    crc += data[i];
  }
  return crc;
}

// Прототипы функций
static void esp_gap_cb(esp_gap_ble_cb_event_t event,
                       esp_ble_gap_cb_param_t *param);
static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                         esp_ble_gattc_cb_param_t *param);
static void gattc_profile_event_handler(esp_gattc_cb_event_t event,
                                        esp_gatt_if_t gattc_if,
                                        esp_ble_gattc_cb_param_t *param);
static void parse_bms_data(uint8_t *data, uint16_t len);
static esp_err_t ble_init(void);

// Удалены собственные реализации функций сканирования - они вызывали панику
// Используем прямое подключение через GATT Client

// Параметры сканирования (из официального примера)
static esp_ble_scan_params_t ble_scan_params = {
    .scan_type = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval = 0x50, // 100ms
    .scan_window = 0x30,   // 60ms
    .scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE};

// Флаги для сканирования
static bool scan_active = false;
static bool device_found = false;

// Обработчик событий GAP (из официального примера)
static void esp_gap_cb(esp_gap_ble_cb_event_t event,
                       esp_ble_gap_cb_param_t *param) {
  esp_err_t err;

  switch (event) {
  case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
    if ((err = param->scan_param_cmpl.status) != ESP_BT_STATUS_SUCCESS) {
      ESP_LOGE(TAG, "Scan param set failed: %s", esp_err_to_name(err));
      break;
    }
    // the unit of the duration is second, 0 means scan permanently
    uint32_t duration = 30; // Сканируем 30 секунд
    esp_ble_gap_start_scanning(duration);
    break;
  }
  case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
    if ((err = param->scan_start_cmpl.status) != ESP_BT_STATUS_SUCCESS) {
      ESP_LOGE(TAG, "Scanning start failed, err %s", esp_err_to_name(err));
      scan_active = false;
    } else {
      ESP_LOGI(TAG, "Scanning start successfully");
      scan_active = true;
    }
    break;
  case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
    if ((err = param->scan_stop_cmpl.status) != ESP_BT_STATUS_SUCCESS) {
      ESP_LOGE(TAG, "Scanning stop failed, err %s", esp_err_to_name(err));
    } else {
      ESP_LOGI(TAG, "Scanning stop successfully");
    }
    scan_active = false;
    break;
  case ESP_GAP_BLE_SCAN_RESULT_EVT: {
    esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
    switch (scan_result->scan_rst.search_evt) {
    case ESP_GAP_SEARCH_INQ_RES_EVT:
      // Проверяем MAC адрес
      if (memcmp(scan_result->scan_rst.bda, bms_addr, 6) == 0) {
        ESP_LOGI(TAG, "✓ Found target BMS device!");
        ESP_LOGI(TAG, "Device MAC: " ESP_BD_ADDR_STR,
                 ESP_BD_ADDR_HEX(scan_result->scan_rst.bda));

        if (!device_found) {
          device_found = true;
          esp_ble_gap_stop_scanning();
          ESP_LOGI(TAG, "Connect to the remote device.");

          // Параметры подключения (из официального примера)
          esp_ble_conn_params_t phy_1m_conn_params = {0};
          phy_1m_conn_params.interval_max = 32;
          phy_1m_conn_params.interval_min = 32;
          phy_1m_conn_params.latency = 0;
          phy_1m_conn_params.supervision_timeout = 600;

          esp_ble_gatt_creat_conn_params_t creat_conn_params = {0};
          memcpy(&creat_conn_params.remote_bda, scan_result->scan_rst.bda,
                 ESP_BD_ADDR_LEN);
          creat_conn_params.remote_addr_type =
              scan_result->scan_rst.ble_addr_type;
          creat_conn_params.own_addr_type = BLE_ADDR_TYPE_PUBLIC;
          creat_conn_params.is_direct = true;
          creat_conn_params.is_aux = false;
          creat_conn_params.phy_mask = ESP_BLE_PHY_1M_PREF_MASK;
          creat_conn_params.phy_1m_conn_params = &phy_1m_conn_params;

          esp_ble_gattc_enh_open(profile_tab[PROFILE_APP_IDX].gattc_if,
                                 &creat_conn_params);
        }
      }
      break;
    case ESP_GAP_SEARCH_INQ_CMPL_EVT:
      ESP_LOGI(TAG, "Scan complete");
      scan_active = false;
      break;
    default:
      break;
    }
    break;
  }
  default:
    break;
  }
}

// Обработчик событий GATT Client
static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                         esp_ble_gattc_cb_param_t *param) {
  // Обновляем gattc_if в профиле при регистрации
  if (event == ESP_GATTC_REG_EVT) {
    profile_tab[PROFILE_APP_IDX].gattc_if = gattc_if;
    ESP_LOGI(GATTC_TAG, "REG_EVT, gattc_if=%d", gattc_if);
  }

  switch (event) {
  case ESP_GATTC_REG_EVT:
    break;
  case ESP_GATTC_CONNECT_EVT:
    ESP_LOGI(GATTC_TAG, "ESP_GATTC_CONNECT_EVT conn_id %d, if %d",
             param->connect.conn_id, gattc_if);
    profile_tab[PROFILE_APP_IDX].conn_id = param->connect.conn_id;
    memcpy(profile_tab[PROFILE_APP_IDX].remote_bda, param->connect.remote_bda,
           sizeof(esp_bd_addr_t));
    // Обновляем параметры соединения
    esp_ble_conn_update_params_t conn_params = {0};
    memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
    conn_params.latency = 0;
    conn_params.max_int = 0x20; // max_int = 0x20*1.25ms = 40ms
    conn_params.min_int = 0x10; // min_int = 0x10*1.25ms = 20ms
    conn_params.timeout = 400;  // timeout = 400*10ms = 4000ms
    esp_ble_gap_update_conn_params(&conn_params);
    if (ble_event_group) {
      xEventGroupSetBits(ble_event_group, CONNECTED_BIT);
    }
    bms_status = JK_BMS_CONNECTED;
    break;
  case ESP_GATTC_DISCONNECT_EVT:
    ESP_LOGI(GATTC_TAG, "ESP_GATTC_DISCONNECT_EVT reason %d (0x%x)",
             param->disconnect.reason, param->disconnect.reason);
    if (param->disconnect.reason == 0x100 || param->disconnect.reason == 256) {
      ESP_LOGW(
          GATTC_TAG,
          "Connection failed: Device may not be in range or not advertising");
      ESP_LOGW(GATTC_TAG,
               "Try scanning first to ensure device is discoverable");
    }
    if (ble_event_group) {
      xEventGroupSetBits(ble_event_group, DISCONNECTED_BIT);
    }
    bms_status = JK_BMS_DISCONNECTED;
    break;
  default:
    break;
  }

  if (event != ESP_GATTC_REG_EVT) {
    gattc_profile_event_handler(event, gattc_if, param);
  }
}

// Обработчик событий профиля
static void gattc_profile_event_handler(esp_gattc_cb_event_t event,
                                        esp_gatt_if_t gattc_if,
                                        esp_ble_gattc_cb_param_t *param) {
  esp_ble_gattc_cb_param_t *p_data = param;

  switch (event) {
  case ESP_GATTC_REG_EVT:
    ESP_LOGI(GATTC_TAG, "REG_EVT, gattc_if=%d", gattc_if);
    profile_tab[PROFILE_APP_IDX].gattc_if = gattc_if;
    // Начинаем сканирование после регистрации (из официального примера)
    if (bms_status == JK_BMS_CONNECTING) {
      esp_ble_gap_set_scan_params(&ble_scan_params);
    }
    break;
  case ESP_GATTC_CONNECT_EVT:
    ESP_LOGI(GATTC_TAG, "ESP_GATTC_CONNECT_EVT conn_id=%d, if=%d",
             p_data->connect.conn_id, gattc_if);
    memcpy(profile_tab[PROFILE_APP_IDX].remote_bda, p_data->connect.remote_bda,
           sizeof(esp_bd_addr_t));
    profile_tab[PROFILE_APP_IDX].conn_id = p_data->connect.conn_id;
    ESP_LOGI(GATTC_TAG, "Starting service search...");
    esp_ble_gattc_search_service(gattc_if, p_data->connect.conn_id, NULL);
    break;
  case ESP_GATTC_DISCONNECT_EVT:
    ESP_LOGI(GATTC_TAG, "ESP_GATTC_DISCONNECT_EVT reason %d",
             p_data->disconnect.reason);
    break;
  case ESP_GATTC_SEARCH_RES_EVT:
    ESP_LOGI(GATTC_TAG, "SEARCH_RES_EVT");
    if (p_data->search_res.srvc_id.uuid.len == ESP_UUID_LEN_16) {
      ESP_LOGI(GATTC_TAG, "Found service UUID: 0x%04X",
               p_data->search_res.srvc_id.uuid.uuid.uuid16);
      if (p_data->search_res.srvc_id.uuid.uuid.uuid16 ==
          service_uuid.uuid.uuid16) {
        ESP_LOGI(GATTC_TAG, "Target service found!");
        profile_tab[PROFILE_APP_IDX].service_start_handle =
            p_data->search_res.start_handle;
        profile_tab[PROFILE_APP_IDX].service_end_handle =
            p_data->search_res.end_handle;
      }
    }
    break;
  case ESP_GATTC_SEARCH_CMPL_EVT:
    ESP_LOGI(GATTC_TAG, "SEARCH_CMPL_EVT status=%x",
             p_data->search_cmpl.status);
    if (p_data->search_cmpl.status != ESP_GATT_OK) {
      ESP_LOGE(GATTC_TAG, "search service failed, error status = %x",
               p_data->search_cmpl.status);
      break;
    }
    if (profile_tab[PROFILE_APP_IDX].service_start_handle ==
        profile_tab[PROFILE_APP_IDX].service_end_handle) {
      ESP_LOGE(GATTC_TAG, "service not found");
      break;
    }
    // Service found, получаем все характеристики сервиса
    ESP_LOGI(GATTC_TAG,
             "Service found (handle %d-%d), getting all characteristics...",
             profile_tab[PROFILE_APP_IDX].service_start_handle,
             profile_tab[PROFILE_APP_IDX].service_end_handle);

    // Получаем все характеристики сервиса
    // Сначала получаем количество характеристик
    uint16_t char_count = 0;
    esp_err_t ret = esp_ble_gattc_get_attr_count(
        gattc_if, p_data->search_cmpl.conn_id, ESP_GATT_DB_CHARACTERISTIC,
        profile_tab[PROFILE_APP_IDX].service_start_handle,
        profile_tab[PROFILE_APP_IDX].service_end_handle, INVALID_HANDLE,
        &char_count);
    if (ret != ESP_OK || char_count == 0) {
      ESP_LOGE(GATTC_TAG, "Failed to get char count: %s (count=%d)",
               esp_err_to_name(ret), char_count);
      // Fallback: используем service_end_handle + 1
      ESP_LOGW(GATTC_TAG, "Falling back to service_end_handle + 1");
      profile_tab[PROFILE_APP_IDX].char_handle =
          profile_tab[PROFILE_APP_IDX].service_end_handle + 1;
      esp_ble_gattc_register_for_notify(
          gattc_if, profile_tab[PROFILE_APP_IDX].remote_bda,
          profile_tab[PROFILE_APP_IDX].char_handle);
      break;
    }

    // Выделяем память для характеристик
    esp_gattc_char_elem_t *char_elem_result = (esp_gattc_char_elem_t *)malloc(
        sizeof(esp_gattc_char_elem_t) * char_count);
    if (!char_elem_result) {
      ESP_LOGE(GATTC_TAG, "Failed to allocate memory for characteristics");
      break;
    }

    // Получаем характеристики по UUID
    ret = esp_ble_gattc_get_char_by_uuid(
        gattc_if, p_data->search_cmpl.conn_id,
        profile_tab[PROFILE_APP_IDX].service_start_handle,
        profile_tab[PROFILE_APP_IDX].service_end_handle, char_uuid,
        char_elem_result, &char_count);
    if (ret != ESP_OK || char_count == 0) {
      ESP_LOGE(GATTC_TAG, "Failed to get characteristic by UUID: %s (count=%d)",
               esp_err_to_name(ret), char_count);
      free(char_elem_result);
      // Fallback: используем service_end_handle + 1
      ESP_LOGW(GATTC_TAG, "Falling back to service_end_handle + 1");
      profile_tab[PROFILE_APP_IDX].char_handle =
          profile_tab[PROFILE_APP_IDX].service_end_handle + 1;
      esp_ble_gattc_register_for_notify(
          gattc_if, profile_tab[PROFILE_APP_IDX].remote_bda,
          profile_tab[PROFILE_APP_IDX].char_handle);
      break;
    }

    // Нашли характеристику
    ESP_LOGI(GATTC_TAG,
             "✓ Found characteristic! Handle=%d, UUID=0x%04X, Prop=0x%02X",
             char_elem_result[0].char_handle,
             char_elem_result[0].uuid.uuid.uuid16,
             char_elem_result[0].properties);
    profile_tab[PROFILE_APP_IDX].char_handle = char_elem_result[0].char_handle;
    free(char_elem_result);

    // Регистрируемся на уведомления (notifications)
    esp_ble_gattc_register_for_notify(gattc_if,
                                      profile_tab[PROFILE_APP_IDX].remote_bda,
                                      profile_tab[PROFILE_APP_IDX].char_handle);
    ESP_LOGI(GATTC_TAG, "Registered for notifications on handle %d",
             profile_tab[PROFILE_APP_IDX].char_handle);

    // Также нужно записать в CCC descriptor для включения уведомлений
    // Handle для CCC обычно = char_handle + 1
    uint16_t ccc_handle = profile_tab[PROFILE_APP_IDX].char_handle + 1;
    uint16_t notify_en = 1; // Enable notifications
    esp_ble_gattc_write_char_descr(
        gattc_if, p_data->search_cmpl.conn_id, ccc_handle, sizeof(notify_en),
        (uint8_t *)&notify_en, ESP_GATT_WRITE_TYPE_NO_RSP,
        ESP_GATT_AUTH_REQ_NONE);
    ESP_LOGI(GATTC_TAG,
             "Wrote CCC descriptor (handle %d) to enable notifications",
             ccc_handle);
    break;
  case ESP_GATTC_NOTIFY_EVT:
    // Уведомления от BMS (данные приходят автоматически)
    // Минимальное логирование - только для больших пакетов
    if (p_data->notify.value_len > 20) {
      ESP_LOGI(GATTC_TAG, "NOTIFY_EVT: handle=%d, len=%d",
               p_data->notify.handle, p_data->notify.value_len);
    }

    if (p_data->notify.handle == profile_tab[PROFILE_APP_IDX].char_handle) {
      parse_bms_data(p_data->notify.value, p_data->notify.value_len);
    } else {
      // Пробуем парсить в любом случае
      parse_bms_data(p_data->notify.value, p_data->notify.value_len);
    }
    break;
  case ESP_GATTC_READ_CHAR_EVT:
    if (p_data->read.status != ESP_GATT_OK) {
      ESP_LOGE(GATTC_TAG, "read char failed, error status = %x",
               p_data->read.status);
      break;
    }
    ESP_LOGI(GATTC_TAG,
             "=== READ_CHAR_EVT: len=%d ===", p_data->read.value_len);
    if (p_data->read.value_len > 0) {
      ESP_LOGI(GATTC_TAG, "First 16 bytes: ");
      for (int i = 0; i < p_data->read.value_len && i < 16; i++) {
        printf("%02X ", p_data->read.value[i]);
      }
      printf("\n");
    }
    parse_bms_data(p_data->read.value, p_data->read.value_len);
    break;
  case ESP_GATTC_WRITE_CHAR_EVT:
    ESP_LOGI(GATTC_TAG,
             "=== WRITE_CHAR_EVT: status=0x%x ===", p_data->write.status);
    if (p_data->write.status == ESP_GATT_OK) {
      ESP_LOGI(GATTC_TAG,
               "✓ Command written successfully, waiting for notification...");
    } else {
      ESP_LOGE(GATTC_TAG, "✗ Write failed: status=0x%x", p_data->write.status);
    }
    break;
  case ESP_GATTC_REG_FOR_NOTIFY_EVT:
    ESP_LOGI(GATTC_TAG, "REG_FOR_NOTIFY_EVT: status=%x",
             p_data->reg_for_notify.status);
    if (p_data->reg_for_notify.status == ESP_GATT_OK) {
      ESP_LOGI(GATTC_TAG, "Successfully registered for notifications");
      // Согласно esphome-jk-bms: сначала отправляем COMMAND_DEVICE_INFO (0x97)
      // Затем в update() отправляется COMMAND_CELL_INFO (0x96) для получения
      // основных данных
      vTaskDelay(pdMS_TO_TICKS(500)); // Небольшая задержка
      // Отправляем команду device info (0x97)
      uint8_t cmd_device[20];
      cmd_device[0] = 0xAA;
      cmd_device[1] = 0x55;
      cmd_device[2] = 0x90;
      cmd_device[3] = 0xEB;
      cmd_device[4] = 0x97; // COMMAND_DEVICE_INFO
      cmd_device[5] = 0x00; // length
      for (int i = 6; i < 19; i++)
        cmd_device[i] = 0x00;
      uint8_t crc = 0;
      for (int i = 0; i < 19; i++)
        crc += cmd_device[i];
      cmd_device[19] = crc;

      esp_ble_gattc_write_char(
          profile_tab[PROFILE_APP_IDX].gattc_if,
          profile_tab[PROFILE_APP_IDX].conn_id,
          profile_tab[PROFILE_APP_IDX].char_handle, sizeof(cmd_device),
          cmd_device,
          ESP_GATT_WRITE_TYPE_NO_RSP, // Без ответа, как в esphome
          ESP_GATT_AUTH_REQ_NONE);
      ESP_LOGI(GATTC_TAG, "Sent COMMAND_DEVICE_INFO (0x97)");

      // Затем через небольшую задержку отправляем COMMAND_CELL_INFO (0x96)
      vTaskDelay(pdMS_TO_TICKS(500));
      jk_bms_update(); // Отправляет COMMAND_CELL_INFO (0x96)
    }
    break;
  default:
    break;
  }
}

// Удаляем собственные реализации - они вызывают панику
// Вместо этого используем прямое подключение через GATT Client
// без предварительного сканирования

// Парсинг данных от BMS (улучшенная версия с буферизацией)
static void parse_bms_data(uint8_t *data, uint16_t len) {
  if (data == NULL || len == 0) {
    return;
  }

  // Проверяем, не являются ли данные повторяющимися "AT\r\n"
  // Если это просто эхо "AT\r\n", игнорируем после первого раза
  static int at_response_count = 0;
  if (len == 4 && data[0] == 0x41 && data[1] == 0x54 && data[2] == 0x0D &&
      data[3] == 0x0A) {
    at_response_count++;
    if (at_response_count <= 3) {
      ESP_LOGI(TAG, "Received AT response #%d (AT\\r\\n) - ignoring",
               at_response_count);
    } else if (at_response_count == 4) {
      ESP_LOGW(TAG, "Too many AT responses, ignoring all further AT\\r\\n");
    }
    return; // Игнорируем повторяющиеся AT\r\n
  }

  // Сбрасываем счетчик, если пришли другие данные
  if (at_response_count > 0) {
    at_response_count = 0;
  }

  // Минимальное логирование - только для не-AT данных и только первые несколько
  // раз
  static int data_log_count = 0;
  if (data_log_count < 3 && len > 4) {
    data_log_count++;
    ESP_LOGI(TAG, "Received BMS data chunk, len=%d, buffer_pos=%d", len,
             bms_buffer_pos);
    ESP_LOGI(TAG, "First 16 bytes: ");
    for (int i = 0; i < len && i < 16; i++) {
      printf("%02X ", data[i]);
    }
    printf("\n");
  }

  // Согласно esphome-jk-bms: собираем пакеты правильно
  // 1. Если в буфере видим 55 AA EB 90 - очищаем буфер (новый пакет)
  // 2. Добавляем данные в буфер
  // 3. Когда размер >= 300 байт, проверяем CRC и парсим

  // Согласно esphome: очищаем буфер если приходит 55 AA EB 90 в начале данных
  // Это означает начало нового пакета
  if (len >= 4 && data[0] == 0x55 && data[1] == 0xAA && data[2] == 0xEB &&
      data[3] == 0x90) {
    // Новый пакет - очищаем буфер
    bms_buffer_pos = 0;
    ESP_LOGD(TAG, "New packet detected (55 AA EB 90), clearing buffer");
  }

  // Проверяем переполнение буфера
  if (bms_buffer_pos + len > BMS_BUFFER_SIZE) {
    ESP_LOGW(TAG, "Buffer overflow! Resetting buffer.");
    bms_buffer_pos = 0;
    return;
  }

  // Добавляем данные в буфер
  memcpy(bms_buffer + bms_buffer_pos, data, len);
  bms_buffer_pos += len;

  // Если буфер >= 300 байт, проверяем CRC и парсим (как в esphome)
  if (bms_buffer_pos >= MIN_RESPONSE_SIZE) {
    const uint8_t *raw = bms_buffer;
    const uint16_t frame_size = 300; // CRC на позиции 299

    // Проверяем, что пакет начинается с правильного заголовка
    if (raw[0] != 0x55 || raw[1] != 0xAA || raw[2] != 0xEB || raw[3] != 0x90) {
      ESP_LOGW(TAG,
               "Invalid packet header! Expected 55 AA EB 90, got %02X %02X "
               "%02X %02X",
               raw[0], raw[1], raw[2], raw[3]);
      bms_buffer_pos = 0;
      return;
    }

    // Проверяем CRC (сумма всех байт кроме последнего)
    uint8_t computed_crc = calculate_crc(raw, frame_size - 1);
    uint8_t remote_crc = raw[frame_size - 1];

    if (computed_crc != remote_crc) {
      ESP_LOGW(TAG, "CRC check failed! 0x%02X != 0x%02X (packet size=%d)",
               computed_crc, remote_crc, bms_buffer_pos);
      // НЕ сбрасываем буфер сразу - возможно пакет еще не полностью собран
      // Сбрасываем только если буфер переполнен
      if (bms_buffer_pos >= MAX_RESPONSE_SIZE) {
        bms_buffer_pos = 0;
      }
      return;
    }

    // CRC OK - парсим пакет
    uint8_t frame_type = (bms_buffer_pos > 4) ? bms_buffer[4] : 0;

    // Логируем пакет для отладки (только для пакета 0x02, и только первый раз)
    static bool first_packet_logged = false;
    if (frame_type == 0x02 && !first_packet_logged) {
      ESP_LOGI(TAG,
               "=== Packet 0x02 received, size=%d bytes ===", bms_buffer_pos);

      // Выводим весь пакет (первые 300 байт для анализа)
      int dump_size = (bms_buffer_pos < 300) ? bms_buffer_pos : 300;
      ESP_LOGI(TAG, "Full packet dump (first %d bytes):", dump_size);
      for (int i = 0; i < dump_size; i++) {
        printf("%02X ", bms_buffer[i]);
        if ((i + 1) % 16 == 0)
          printf("\n");
      }
      if (dump_size % 16 != 0)
        printf("\n");

      // Логируем байты вокруг разных offset'ов для поиска данных
      if (bms_buffer_pos >= 142) {
        ESP_LOGI(TAG, "=== Checking various offsets ===");
        ESP_LOGI(TAG, "Offset 118-121 (voltage JK02): %02X %02X %02X %02X",
                 bms_buffer[118], bms_buffer[119], bms_buffer[120],
                 bms_buffer[121]);
        ESP_LOGI(TAG, "Offset 126-129 (current JK02): %02X %02X %02X %02X",
                 bms_buffer[126], bms_buffer[127], bms_buffer[128],
                 bms_buffer[129]);
        ESP_LOGI(TAG, "Offset 130-131 (temp JK02): %02X %02X", bms_buffer[130],
                 bms_buffer[131]);
        ESP_LOGI(TAG, "Offset 141 (SOC JK02): %02X", bms_buffer[141]);
      }
      first_packet_logged = true;
    }

    if (frame_type == 0x02 && bms_buffer_pos >= 200) {
// Пакет типа 0x02 (cell info) - основные данные
// Пытаемся определить версию протокола (offset 0, 16 или 32)
// JK_PB2A16S15P использует offset = 32

// Вспомогательные макросы для чтения little-endian
#define JK_GET_16BIT(i)                                                        \
  ((uint16_t)(bms_buffer[(i) + 1] << 8) | (uint16_t)(bms_buffer[(i)]))
#define JK_GET_32BIT(i)                                                        \
  (((uint32_t)JK_GET_16BIT((i) + 2) << 16) | (uint32_t)JK_GET_16BIT((i)))

      uint8_t offset = 16; // Default

      // Проверяем данные для offset = 0 (JK02_24S)
      uint32_t v0_raw = JK_GET_32BIT(118);
      float v0 = v0_raw * 0.001f;
      uint8_t soc0 = bms_buffer[141];

      // Проверяем данные для offset = 16 (JK02_32S)
      uint32_t v16_raw = JK_GET_32BIT(118 + 16);
      float v16 = v16_raw * 0.001f;
      uint8_t soc16 = bms_buffer[141 + 16];

      // Проверяем данные для offset = 32 (JK_PB2A16S15P)
      uint32_t v32_raw = JK_GET_32BIT(118 + 32);
      float v32 = v32_raw * 0.001f;
      uint8_t soc32 = bms_buffer[141 + 32];

      ESP_LOGI(TAG,
               "Offset detection: V0=%.2fV SOC0=%d%% | V16=%.2fV SOC16=%d%% | "
               "V32=%.2fV SOC32=%d%%",
               v0, soc0, v16, soc16, v32, soc32);

      // Эвристика: Напряжение должно быть разумным (10-150В) и SOC <= 100
      bool v0_valid = (v0 > 10.0f && v0 < 150.0f);
      bool soc0_valid = (soc0 <= 100);

      bool v16_valid = (v16 > 10.0f && v16 < 150.0f);
      bool soc16_valid = (soc16 <= 100);

      bool v32_valid = (v32 > 10.0f && v32 < 150.0f);
      bool soc32_valid = (soc32 <= 100);

      if (v32_valid && soc32_valid) {
        offset = 32;
        ESP_LOGI(TAG, "Detected JK_PB2A16S15P protocol (offset 32)");
      } else if (v16_valid && soc16_valid) {
        offset = 16;
        ESP_LOGI(TAG, "Detected JK02_32S protocol (offset 16)");
      } else if (v0_valid && soc0_valid) {
        offset = 0;
        ESP_LOGI(TAG, "Detected JK02_24S protocol (offset 0)");
      } else {
        ESP_LOGW(TAG, "Could not detect protocol version reliably, defaulting "
                      "to offset 32 (most likely for PB model)");
        // Если ничего не подошло, но модель PB, пробуем 32
        offset = 32;
      }

      // Battery voltage (offset 118 + offset, 4 bytes, 0.001V)
      uint32_t voltage_raw = JK_GET_32BIT(118 + offset);
      bms_data.voltage = voltage_raw * 0.001f;

      // Current (offset 126 + offset, 4 bytes, signed, 0.001A)
      int32_t current_raw = (int32_t)JK_GET_32BIT(126 + offset);
      bms_data.current = current_raw * 0.001f;

      // Temperature 1 (offset 130 + offset, 2 bytes, signed, 0.1°C)
      // Для temperature offset умножается на 2 в esphome, но для PB модели
      // (offset 32) проверим логику: 130 + 32 = 162. В дампе там E1 00 (22.5C).
      // Если использовать offset*2 = 64, то 130+64 = 194.
      // Давайте пока использовать простое смещение, так как оно сработало для
      // 162. Если offset=16, то 130+16=146. В esphome 130+32=162. Видимо для PB
      // модели offset просто добавляется.

      int16_t temp1_raw;
      if (offset == 16) {
        // Для JK02_32S (offset 16) esphome делает offset*2 для температуры?
        // В коде esphome: jk_get_16bit(130 + offset * 2)
        // 130 + 32 = 162.
        temp1_raw = (int16_t)JK_GET_16BIT(130 + offset * 2);
      } else {
        // Для offset 32 (PB) и offset 0
        // 130 + 32 = 162. Это совпадает с дампом.
        temp1_raw = (int16_t)JK_GET_16BIT(130 + offset);
      }

      bms_data.temperature = temp1_raw * 0.1f;

      // SOC (offset 141 + offset, 1 byte, 0-100%)
      bms_data.soc = bms_buffer[141 + offset];

      ESP_LOGI(TAG,
               "✓ Parsed packet 0x02 (offset=%d): V=%.2fV, I=%.3fA, SOC=%d%%, "
               "T=%.1f°C",
               offset, bms_data.voltage, bms_data.current, bms_data.soc,
               bms_data.temperature);
      bms_data.is_valid = true;

#undef JK_GET_16BIT
#undef JK_GET_32BIT
    } else if (frame_type == 0x01) {
      ESP_LOGD(TAG, "Packet type 0x01 (settings) - skipping");
    } else if (frame_type == 0x03) {
      ESP_LOGD(TAG, "Packet type 0x03 (device info) - skipping");
    } else {
      ESP_LOGW(TAG, "Unknown frame type: 0x%02X", frame_type);
    }

    // Очищаем буфер после обработки
    bms_buffer_pos = 0;
    return;
  }

  // Если буфер слишком большой, сбрасываем
  if (bms_buffer_pos > MAX_RESPONSE_SIZE) {
    ESP_LOGW(TAG, "Buffer too large (%d bytes), resetting", bms_buffer_pos);
    bms_buffer_pos = 0;
    return;
  }
}

// Инициализация BLE
static esp_err_t ble_init(void) {
  esp_err_t ret;

  // Инициализация контроллера Bluetooth
  // ESP32-C3 поддерживает только BLE, не нужно освобождать память для Classic
  esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
  ret = esp_bt_controller_init(&bt_cfg);
  if (ret) {
    ESP_LOGE(TAG, "init controller failed: %s", esp_err_to_name(ret));
    return ret;
  }

  ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
  if (ret) {
    ESP_LOGE(TAG, "enable controller failed: %s", esp_err_to_name(ret));
    return ret;
  }

  ret = esp_bluedroid_init();
  if (ret) {
    ESP_LOGE(TAG, "init bluedroid failed: %s", esp_err_to_name(ret));
    return ret;
  }

  ret = esp_bluedroid_enable();
  if (ret) {
    ESP_LOGE(TAG, "enable bluedroid failed: %s", esp_err_to_name(ret));
    return ret;
  }

  // Регистрируем GAP callback для обработки подключений
  ret = esp_ble_gap_register_callback(esp_gap_cb);
  if (ret) {
    ESP_LOGE(TAG, "gap register failed: %s", esp_err_to_name(ret));
    return ret;
  }

  ret = esp_ble_gattc_register_callback(esp_gattc_cb);
  if (ret) {
    ESP_LOGE(TAG, "gattc register failed: %s", esp_err_to_name(ret));
    return ret;
  }

  ret = esp_ble_gattc_app_register(PROFILE_APP_IDX);
  if (ret) {
    ESP_LOGE(TAG, "gattc app register failed: %s", esp_err_to_name(ret));
    return ret;
  }

  ESP_LOGI(TAG, "BLE initialized");
  return ESP_OK;
}

// Публичные функции

bool jk_bms_init(void) {
  ESP_LOGI(TAG, "Initializing JK BMS module");

  // Создаем event group
  ble_event_group = xEventGroupCreate();
  if (ble_event_group == NULL) {
    ESP_LOGE(TAG, "Failed to create event group");
    return false;
  }

  // Инициализируем BLE
  esp_err_t ret = ble_init();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "BLE initialization failed");
    return false;
  }

  // Инициализируем данные
  memset(&bms_data, 0, sizeof(bms_data));
  bms_status = JK_BMS_DISCONNECTED;

  // Устанавливаем callback после инициализации
  profile_tab[PROFILE_APP_IDX].gattc_cb = gattc_profile_event_handler;

  ESP_LOGI(TAG, "JK BMS module initialized");
  return true;
}

bool jk_bms_connect(void) {
  if (bms_status == JK_BMS_CONNECTED || bms_status == JK_BMS_CONNECTING) {
    ESP_LOGW(TAG, "Already connected or connecting");
    return false;
  }

  if (profile_tab[PROFILE_APP_IDX].gattc_if == ESP_GATT_IF_NONE) {
    ESP_LOGE(TAG, "GATT Client not registered yet");
    return false;
  }

  ESP_LOGI(TAG, "Connecting to JK BMS: " JK_BMS_MAC_ADDR);
  bms_status = JK_BMS_CONNECTING;

  // Ждем регистрации GATT Client
  vTaskDelay(pdMS_TO_TICKS(1000));

  // Подключаемся к устройству через сканирование и GATT Client
  ESP_LOGI(TAG,
           "Attempting to connect to BMS MAC: %02X:%02X:%02X:%02X:%02X:%02X",
           bms_addr[0], bms_addr[1], bms_addr[2], bms_addr[3], bms_addr[4],
           bms_addr[5]);

  // Используем прямое подключение с повторными попытками
  // Сканирование не используется, так как функции не линкуются
  esp_err_t ret;

  // Ждем регистрации GATT Client
  ESP_LOGI(TAG, "Waiting for GATT Client registration...");
  int wait_count = 0;
  while (profile_tab[PROFILE_APP_IDX].gattc_if == ESP_GATT_IF_NONE &&
         wait_count < 100) {
    vTaskDelay(pdMS_TO_TICKS(100));
    wait_count++;
  }

  if (profile_tab[PROFILE_APP_IDX].gattc_if == ESP_GATT_IF_NONE) {
    ESP_LOGE(TAG, "GATT Client not registered after %d attempts", wait_count);
    bms_status = JK_BMS_DISCONNECTED;
    return false;
  }

  ESP_LOGI(TAG, "GATT Client registered (gattc_if=%d)",
           profile_tab[PROFILE_APP_IDX].gattc_if);
  ESP_LOGI(TAG, "Starting BLE scan to find BMS device...");

  // Очищаем event group перед началом
  if (ble_event_group) {
    xEventGroupClearBits(ble_event_group, CONNECTED_BIT | DISCONNECTED_BIT);
  }

  device_found = false;
  scan_active = false;

  // Сканирование начнется автоматически в ESP_GATTC_REG_EVT
  // Ждем либо обнаружения устройства и подключения, либо таймаута
  const int SCAN_TIMEOUT_MS = 35000; // 35 секунд на сканирование и подключение
  unsigned long start_time = xTaskGetTickCount();

  ESP_LOGI(TAG,
           "Waiting for device to be found and connected (timeout: %d ms)...",
           SCAN_TIMEOUT_MS);

  while ((xTaskGetTickCount() - start_time < pdMS_TO_TICKS(SCAN_TIMEOUT_MS))) {
    if (bms_status == JK_BMS_CONNECTED) {
      ESP_LOGI(TAG, "✓✓✓ Connected to JK BMS ✓✓✓");
      return true;
    }

    // Если сканирование завершилось и устройство не найдено, пробуем прямое
    // подключение
    if (!scan_active && !device_found &&
        (xTaskGetTickCount() - start_time > pdMS_TO_TICKS(5000))) {
      ESP_LOGW(TAG,
               "Device not found during scan, trying direct connection...");

      // Параметры подключения (из официального примера)
      esp_ble_conn_params_t phy_1m_conn_params = {.interval_max = 32,
                                                  .interval_min = 32,
                                                  .latency = 0,
                                                  .supervision_timeout = 600};

      esp_ble_gatt_creat_conn_params_t creat_conn_params = {0};
      memcpy(creat_conn_params.remote_bda, bms_addr, ESP_BD_ADDR_LEN);
      creat_conn_params.remote_addr_type = BLE_ADDR_TYPE_PUBLIC;
      creat_conn_params.own_addr_type = BLE_ADDR_TYPE_PUBLIC;
      creat_conn_params.is_direct = true;
      creat_conn_params.is_aux = false;
      creat_conn_params.phy_mask = ESP_BLE_PHY_1M_PREF_MASK;
      creat_conn_params.phy_1m_conn_params = &phy_1m_conn_params;

      ret = esp_ble_gattc_enh_open(profile_tab[PROFILE_APP_IDX].gattc_if,
                                   &creat_conn_params);
      if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Direct connection failed: %s", esp_err_to_name(ret));
      } else {
        ESP_LOGI(TAG, "Direct connection request sent, waiting...");
      }

      // Ждем подключения
      if (ble_event_group) {
        EventBits_t bits = xEventGroupWaitBits(
            ble_event_group, CONNECTED_BIT | DISCONNECTED_BIT, pdFALSE, pdFALSE,
            pdMS_TO_TICKS(CONNECT_TIMEOUT_MS));

        if (bits & CONNECTED_BIT) {
          ESP_LOGI(TAG, "✓✓✓ Connected to JK BMS via direct connection ✓✓✓");
          return true;
        }
      }

      // Если прямое подключение не удалось, выходим
      break;
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }

  ESP_LOGE(TAG, "Failed to connect to JK BMS (timeout after %d ms)",
           SCAN_TIMEOUT_MS);
  ESP_LOGE(TAG,
           "Make sure BMS device is powered on, in range, and advertising");
  bms_status = JK_BMS_DISCONNECTED;
  return false;
}

void jk_bms_disconnect(void) {
  if (bms_status == JK_BMS_DISCONNECTED) {
    return;
  }

  ESP_LOGI(TAG, "Disconnecting from JK BMS");
  if (profile_tab[PROFILE_APP_IDX].gattc_if != ESP_GATT_IF_NONE) {
    esp_ble_gattc_close(profile_tab[PROFILE_APP_IDX].gattc_if,
                        profile_tab[PROFILE_APP_IDX].conn_id);
  }
  bms_status = JK_BMS_DISCONNECTED;
}

jk_bms_status_t jk_bms_get_status(void) { return bms_status; }

bool jk_bms_get_data(jk_bms_data_t *data) {
  if (data == NULL) {
    return false;
  }

  if (!bms_data.is_valid) {
    return false;
  }

  memcpy(data, &bms_data, sizeof(jk_bms_data_t));
  return true;
}

void jk_bms_update(void) {
  // Периодически читаем данные от BMS
  if (bms_status == JK_BMS_CONNECTED &&
      profile_tab[PROFILE_APP_IDX].char_handle != INVALID_HANDLE &&
      profile_tab[PROFILE_APP_IDX].gattc_if != ESP_GATT_IF_NONE) {

    // Очищаем буфер перед новой командой
    bms_buffer_pos = 0;

    // JK-BMS command format (согласно esphome-jk-bms):
    // AA 55 90 EB [address] [length] [value 4 bytes] [zeros] [CRC]
    // COMMAND_CELL_INFO = 0x96 - для получения основных данных (voltage,
    // current, SOC, temperature) COMMAND_DEVICE_INFO = 0x97 - для получения
    // информации об устройстве

    // Формируем команду для чтения основных данных (COMMAND_CELL_INFO = 0x96)
    uint8_t cmd[20];
    cmd[0] = 0xAA; // start sequence
    cmd[1] = 0x55; // start sequence
    cmd[2] = 0x90; // start sequence
    cmd[3] = 0xEB; // start sequence
    cmd[4] = 0x96; // COMMAND_CELL_INFO (для основных данных)
    cmd[5] = 0x00; // length (0 для чтения)
    cmd[6] = 0x00; // value byte 0
    cmd[7] = 0x00; // value byte 1
    cmd[8] = 0x00; // value byte 2
    cmd[9] = 0x00; // value byte 3
    // Заполняем нулями
    for (int i = 10; i < 19; i++) {
      cmd[i] = 0x00;
    }
    // Вычисляем CRC (сумма всех байт кроме последнего)
    uint8_t crc = 0;
    for (int i = 0; i < 19; i++) {
      crc += cmd[i];
    }
    cmd[19] = crc;

    ESP_LOGI(TAG,
             "Sending COMMAND_CELL_INFO (0x96) to BMS (handle=%d, conn_id=%d)",
             profile_tab[PROFILE_APP_IDX].char_handle,
             profile_tab[PROFILE_APP_IDX].conn_id);

    // Согласно esphome-jk-bms используем ESP_GATT_WRITE_TYPE_NO_RSP (без
    // ответа)
    esp_err_t ret = esp_ble_gattc_write_char(
        profile_tab[PROFILE_APP_IDX].gattc_if,
        profile_tab[PROFILE_APP_IDX].conn_id,
        profile_tab[PROFILE_APP_IDX].char_handle, sizeof(cmd), cmd,
        ESP_GATT_WRITE_TYPE_NO_RSP, // Без ответа, как в esphome
        ESP_GATT_AUTH_REQ_NONE);
    if (ret == ESP_OK) {
      ESP_LOGI(TAG, "✓ COMMAND_CELL_INFO (0x96) sent successfully");
    } else {
      ESP_LOGE(TAG, "✗ Failed to send COMMAND_CELL_INFO: %s",
               esp_err_to_name(ret));
    }
  } else {
    ESP_LOGW(TAG, "Cannot send BMS command: status=%d, handle=%d, gattc_if=%d",
             bms_status, profile_tab[PROFILE_APP_IDX].char_handle,
             profile_tab[PROFILE_APP_IDX].gattc_if);
  }
}
