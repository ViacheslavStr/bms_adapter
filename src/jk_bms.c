#include "jk_bms.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_common_api.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include <string.h>

// Удалены собственные реализации функций сканирования - они вызывали панику
// Используем прямое подключение через GATT Client без предварительного сканирования

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
    .uuid = {.uuid16 = 0xFFE0}  // Service UUID: 0000ffe0-0000-1000-8000-00805f9b34fb
};

// Characteristic UUID: 0000ffe1-0000-1000-8000-00805f9b34fb
// Используется в коде через прямое значение 0xFFE1

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
    [PROFILE_APP_IDX] = {
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

// Прототипы функций
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
static void parse_bms_data(uint8_t *data, uint16_t len);
static esp_err_t ble_init(void);

// Удалены собственные реализации функций сканирования - они вызывали панику
// Используем прямое подключение через GATT Client

// Параметры сканирования (из официального примера)
static esp_ble_scan_params_t ble_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = 0x50,  // 100ms
    .scan_window            = 0x30,   // 60ms
    .scan_duplicate         = BLE_SCAN_DUPLICATE_DISABLE
};

// Флаги для сканирования
static bool scan_active = false;
static bool device_found = false;

// Обработчик событий GAP (из официального примера)
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    esp_err_t err;
    
    switch (event) {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
        if((err = param->scan_param_cmpl.status) != ESP_BT_STATUS_SUCCESS){
            ESP_LOGE(TAG, "Scan param set failed: %s", esp_err_to_name(err));
            break;
        }
        // the unit of the duration is second, 0 means scan permanently
        uint32_t duration = 30;  // Сканируем 30 секунд
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
                ESP_LOGI(TAG, "Device MAC: "ESP_BD_ADDR_STR, ESP_BD_ADDR_HEX(scan_result->scan_rst.bda));
                
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
                    memcpy(&creat_conn_params.remote_bda, scan_result->scan_rst.bda, ESP_BD_ADDR_LEN);
                    creat_conn_params.remote_addr_type = scan_result->scan_rst.ble_addr_type;
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
static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    // Обновляем gattc_if в профиле при регистрации
    if (event == ESP_GATTC_REG_EVT) {
        profile_tab[PROFILE_APP_IDX].gattc_if = gattc_if;
        ESP_LOGI(GATTC_TAG, "REG_EVT, gattc_if=%d", gattc_if);
    }
    
    switch (event) {
        case ESP_GATTC_REG_EVT:
            break;
        case ESP_GATTC_CONNECT_EVT:
            ESP_LOGI(GATTC_TAG, "ESP_GATTC_CONNECT_EVT conn_id %d, if %d", param->connect.conn_id, gattc_if);
            profile_tab[PROFILE_APP_IDX].conn_id = param->connect.conn_id;
            memcpy(profile_tab[PROFILE_APP_IDX].remote_bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
            // Обновляем параметры соединения
            esp_ble_conn_update_params_t conn_params = {0};
            memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
            conn_params.latency = 0;
            conn_params.max_int = 0x20;    // max_int = 0x20*1.25ms = 40ms
            conn_params.min_int = 0x10;    // min_int = 0x10*1.25ms = 20ms
            conn_params.timeout = 400;      // timeout = 400*10ms = 4000ms
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
                ESP_LOGW(GATTC_TAG, "Connection failed: Device may not be in range or not advertising");
                ESP_LOGW(GATTC_TAG, "Try scanning first to ensure device is discoverable");
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
static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
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
            memcpy(profile_tab[PROFILE_APP_IDX].remote_bda, p_data->connect.remote_bda, sizeof(esp_bd_addr_t));
            profile_tab[PROFILE_APP_IDX].conn_id = p_data->connect.conn_id;
            ESP_LOGI(GATTC_TAG, "Starting service search...");
            esp_ble_gattc_search_service(gattc_if, p_data->connect.conn_id, NULL);
            break;
        case ESP_GATTC_DISCONNECT_EVT:
            ESP_LOGI(GATTC_TAG, "ESP_GATTC_DISCONNECT_EVT reason %d", p_data->disconnect.reason);
            break;
        case ESP_GATTC_SEARCH_RES_EVT:
            ESP_LOGI(GATTC_TAG, "SEARCH_RES_EVT");
            if (p_data->search_res.srvc_id.uuid.len == ESP_UUID_LEN_16) {
                ESP_LOGI(GATTC_TAG, "Found service UUID: 0x%04X", p_data->search_res.srvc_id.uuid.uuid.uuid16);
                if (p_data->search_res.srvc_id.uuid.uuid.uuid16 == service_uuid.uuid.uuid16) {
                    ESP_LOGI(GATTC_TAG, "Target service found!");
                    profile_tab[PROFILE_APP_IDX].service_start_handle = p_data->search_res.start_handle;
                    profile_tab[PROFILE_APP_IDX].service_end_handle = p_data->search_res.end_handle;
                }
            }
            break;
        case ESP_GATTC_SEARCH_CMPL_EVT:
            ESP_LOGI(GATTC_TAG, "SEARCH_CMPL_EVT status=%x", p_data->search_cmpl.status);
            if (p_data->search_cmpl.status != ESP_GATT_OK) {
                ESP_LOGE(GATTC_TAG, "search service failed, error status = %x", p_data->search_cmpl.status);
                break;
            }
            if (profile_tab[PROFILE_APP_IDX].service_start_handle == profile_tab[PROFILE_APP_IDX].service_end_handle) {
                ESP_LOGE(GATTC_TAG, "service not found");
                break;
            }
            // Service found, используем упрощенный подход
            // Для JK BMS характеристика обычно находится сразу после service_end_handle
            // Или используем известный handle (обычно service_end_handle + 1)
            ESP_LOGI(GATTC_TAG, "Service found (handle %d-%d), using characteristic handle %d",
                     profile_tab[PROFILE_APP_IDX].service_start_handle,
                     profile_tab[PROFILE_APP_IDX].service_end_handle,
                     profile_tab[PROFILE_APP_IDX].service_end_handle + 1);
            
            // Устанавливаем handle характеристики (обычно сразу после service_end_handle)
            profile_tab[PROFILE_APP_IDX].char_handle = profile_tab[PROFILE_APP_IDX].service_end_handle + 1;
            
            // Регистрируемся на уведомления (notifications)
            esp_ble_gattc_register_for_notify(gattc_if,
                                            profile_tab[PROFILE_APP_IDX].remote_bda,
                                            profile_tab[PROFILE_APP_IDX].char_handle);
            ESP_LOGI(GATTC_TAG, "Registered for notifications on handle %d", 
                     profile_tab[PROFILE_APP_IDX].char_handle);
            
            // Также нужно записать в CCC descriptor для включения уведомлений
            // Handle для CCC обычно = char_handle + 1
            uint16_t ccc_handle = profile_tab[PROFILE_APP_IDX].char_handle + 1;
            uint16_t notify_en = 1;  // Enable notifications
            esp_ble_gattc_write_char_descr(gattc_if,
                                          p_data->search_cmpl.conn_id,
                                          ccc_handle,
                                          sizeof(notify_en),
                                          (uint8_t*)&notify_en,
                                          ESP_GATT_WRITE_TYPE_NO_RSP,
                                          ESP_GATT_AUTH_REQ_NONE);
            ESP_LOGI(GATTC_TAG, "Wrote CCC descriptor (handle %d) to enable notifications", ccc_handle);
            break;
        case ESP_GATTC_NOTIFY_EVT:
            // Уведомления от BMS (данные приходят автоматически)
            ESP_LOGI(GATTC_TAG, "NOTIFY_EVT: handle=%d, len=%d", 
                     p_data->notify.handle, p_data->notify.value_len);
            if (p_data->notify.handle == profile_tab[PROFILE_APP_IDX].char_handle) {
                ESP_LOGI(GATTC_TAG, "Received notification from BMS, parsing data...");
                parse_bms_data(p_data->notify.value, p_data->notify.value_len);
            } else {
                ESP_LOGW(GATTC_TAG, "Notification from unknown handle: %d (expected: %d)", 
                         p_data->notify.handle, profile_tab[PROFILE_APP_IDX].char_handle);
            }
            break;
        case ESP_GATTC_READ_CHAR_EVT:
            if (p_data->read.status != ESP_GATT_OK) {
                ESP_LOGE(GATTC_TAG, "read char failed, error status = %x", p_data->read.status);
                break;
            }
            ESP_LOGI(GATTC_TAG, "READ_CHAR_EVT: len=%d", p_data->read.value_len);
            parse_bms_data(p_data->read.value, p_data->read.value_len);
            break;
        case ESP_GATTC_REG_FOR_NOTIFY_EVT:
            ESP_LOGI(GATTC_TAG, "REG_FOR_NOTIFY_EVT: status=%x", p_data->reg_for_notify.status);
            if (p_data->reg_for_notify.status == ESP_GATT_OK) {
                ESP_LOGI(GATTC_TAG, "Successfully registered for notifications");
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
static void parse_bms_data(uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0) {
        return;
    }

    ESP_LOGI(TAG, "Received BMS data chunk, len=%d, buffer_pos=%d", len, bms_buffer_pos);
    
    // Добавляем данные в буфер
    if (bms_buffer_pos + len < BMS_BUFFER_SIZE) {
        memcpy(bms_buffer + bms_buffer_pos, data, len);
        bms_buffer_pos += len;
    } else {
        ESP_LOGW(TAG, "Buffer overflow! Resetting buffer.");
        bms_buffer_pos = 0;
        return;
    }
    
    // Проверяем, есть ли полный пакет
    // JK-BMS отправляет данные частями, полный пакет обычно ~300 байт
    // Ищем маркер начала пакета: 55 AA EB 90 или FE FF FF FF (в пакете 3)
    
    // Ищем начало пакета 55 AA EB 90
    int packet_start = -1;
    for (int i = 0; i < bms_buffer_pos - 3; i++) {
        if (bms_buffer[i] == 0x55 && bms_buffer[i+1] == 0xAA && 
            bms_buffer[i+2] == 0xEB && bms_buffer[i+3] == 0x90) {
            packet_start = i;
            break;
        }
    }
    
    // Если не нашли 55 AA EB 90, ищем FE FF FF FF (пакет 3 с бинарными данными)
    if (packet_start == -1) {
        for (int i = 0; i < bms_buffer_pos - 3; i++) {
            if (bms_buffer[i] == 0xFE && bms_buffer[i+1] == 0xFF && 
                bms_buffer[i+2] == 0xFF && bms_buffer[i+3] == 0xFF) {
                packet_start = i;
                ESP_LOGI(TAG, "Found packet 3 marker (FE FF FF FF) at offset %d", i);
                break;
            }
        }
    }
    
    // Если нашли начало пакета и данных достаточно (минимум 200 байт для полного пакета)
    if (packet_start >= 0 && bms_buffer_pos - packet_start >= 200) {
        ESP_LOGI(TAG, "=== FULL BMS PACKET DETECTED (total %d bytes) ===", bms_buffer_pos - packet_start);
        
        // Выводим полный hex-дамп для анализа
        ESP_LOGI(TAG, "Full packet hex dump (offset %d):", packet_start);
        for (int i = packet_start; i < bms_buffer_pos && i < packet_start + 300; i++) {
            printf("%02X ", bms_buffer[i]);
            if ((i - packet_start + 1) % 16 == 0) {
                printf("  | ");
                // ASCII representation
                for (int j = i - 15; j <= i; j++) {
                    char c = (bms_buffer[j] >= 32 && bms_buffer[j] < 127) ? bms_buffer[j] : '.';
                    printf("%c", c);
                }
                printf("\n");
            }
        }
        if ((bms_buffer_pos - packet_start) % 16 != 0) printf("\n");
        
        uint8_t *packet_data = bms_buffer + packet_start;
        size_t packet_len = bms_buffer_pos - packet_start;
        
        // Парсим данные согласно документации JK-BMS
        // Пакет 3 (128 байт) начинается с FE FF FF FF
        // Offset 8-11: Ток (mA, signed, little-endian, 4 bytes)
        // Offset 12-15: Напряжение (mV, little-endian, 4 bytes)
        // Offset 92: SOC (проценты, 0-100, 1 byte)
        
        // Ищем пакет 3 с бинарными данными
        int packet3_start = -1;
        for (int i = 0; i < packet_len - 3; i++) {
            if (packet_data[i] == 0xFE && packet_data[i+1] == 0xFF && 
                packet_data[i+2] == 0xFF && packet_data[i+3] == 0xFF) {
                packet3_start = i;
                break;
            }
        }
        
        if (packet3_start >= 0 && packet_len - packet3_start >= 100) {
            ESP_LOGI(TAG, "Found packet 3 at offset %d, parsing binary data...", packet3_start);
            
            // Parse Current - offset 8-11 (4 bytes, signed, little-endian, в мА)
            if (packet3_start + 11 < packet_len) {
                int32_t current_ma = (int32_t)(packet_data[packet3_start + 8] | 
                                               (packet_data[packet3_start + 9] << 8) |
                                               (packet_data[packet3_start + 10] << 16) |
                                               (packet_data[packet3_start + 11] << 24));
                bms_data.current = current_ma / 1000.0f;  // Конвертируем мА в А
                ESP_LOGI(TAG, "Current parsed from offset %d: %ld mA (%.3f A)", 
                         packet3_start + 8, current_ma, bms_data.current);
            }
            
            // Parse Voltage - offset 12-15 (4 bytes, little-endian, в мВ)
            if (packet3_start + 15 < packet_len) {
                uint32_t voltage_mv = (uint32_t)(packet_data[packet3_start + 12] | 
                                                 (packet_data[packet3_start + 13] << 8) |
                                                 (packet_data[packet3_start + 14] << 16) |
                                                 (packet_data[packet3_start + 15] << 24));
                bms_data.voltage = voltage_mv / 1000.0f;  // Конвертируем мВ в В
                ESP_LOGI(TAG, "Voltage parsed from offset %d: %lu mV (%.2f V)", 
                         packet3_start + 12, voltage_mv, bms_data.voltage);
            }
            
            // Parse SOC - offset 92 (1 byte, percentage)
            if (packet3_start + 92 < packet_len) {
                uint8_t soc = packet_data[packet3_start + 92];
                if (soc <= 100) {
                    bms_data.soc = soc;
                    ESP_LOGI(TAG, "SOC parsed from offset %d: %d%%", packet3_start + 92, bms_data.soc);
                } else {
                    ESP_LOGW(TAG, "Invalid SOC value: %d (expected 0-100)", soc);
                }
            }
            
            // Также пробуем старый формат (55 AA EB 90) для совместимости
            // Parse SOC - offset 8 от начала пакета 55 AA EB 90
            if (packet_data[0] == 0x55 && packet_data[1] == 0xAA && 
                packet_data[2] == 0xEB && packet_data[3] == 0x90 && packet_len >= 9) {
                uint8_t soc_alt = packet_data[8];
                if (soc_alt <= 100 && bms_data.soc == 0) {
                    bms_data.soc = soc_alt;
                    ESP_LOGI(TAG, "SOC parsed from format 55 AA EB 90, offset 8: %d%%", soc_alt);
                }
            }
            
            bms_data.is_valid = true;
            ESP_LOGI(TAG, "=== PARSED BMS Data: V=%.2fV, SOC=%d%%, I=%.3fA ===", 
                     bms_data.voltage, bms_data.soc, bms_data.current);
        } else {
            ESP_LOGW(TAG, "Packet 3 not found or too short (packet3_start=%d, packet_len=%d)", 
                     packet3_start, packet_len);
        }
        
        // Очищаем буфер после обработки
        bms_buffer_pos = 0;
    } else if (bms_buffer_pos > BMS_BUFFER_SIZE - 100) {
        // Если буфер почти полный, но пакет не найден, сбрасываем
        ESP_LOGW(TAG, "Buffer almost full but no complete packet found, resetting");
        bms_buffer_pos = 0;
    }
}

// Инициализация BLE
static esp_err_t ble_init(void)
{
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

bool jk_bms_init(void)
{
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

bool jk_bms_connect(void)
{
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
    ESP_LOGI(TAG, "Attempting to connect to BMS MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             bms_addr[0], bms_addr[1], bms_addr[2], bms_addr[3], bms_addr[4], bms_addr[5]);
    
    // Используем прямое подключение с повторными попытками
    // Сканирование не используется, так как функции не линкуются
    esp_err_t ret;
    
    
    // Ждем регистрации GATT Client
    ESP_LOGI(TAG, "Waiting for GATT Client registration...");
    int wait_count = 0;
    while (profile_tab[PROFILE_APP_IDX].gattc_if == ESP_GATT_IF_NONE && wait_count < 100) {
        vTaskDelay(pdMS_TO_TICKS(100));
        wait_count++;
    }
    
    if (profile_tab[PROFILE_APP_IDX].gattc_if == ESP_GATT_IF_NONE) {
        ESP_LOGE(TAG, "GATT Client not registered after %d attempts", wait_count);
        bms_status = JK_BMS_DISCONNECTED;
        return false;
    }
    
    ESP_LOGI(TAG, "GATT Client registered (gattc_if=%d)", profile_tab[PROFILE_APP_IDX].gattc_if);
    ESP_LOGI(TAG, "Starting BLE scan to find BMS device...");
    
    // Очищаем event group перед началом
    if (ble_event_group) {
        xEventGroupClearBits(ble_event_group, CONNECTED_BIT | DISCONNECTED_BIT);
    }
    
    device_found = false;
    scan_active = false;
    
    // Сканирование начнется автоматически в ESP_GATTC_REG_EVT
    // Ждем либо обнаружения устройства и подключения, либо таймаута
    const int SCAN_TIMEOUT_MS = 35000;  // 35 секунд на сканирование и подключение
    unsigned long start_time = xTaskGetTickCount();
    
    ESP_LOGI(TAG, "Waiting for device to be found and connected (timeout: %d ms)...", SCAN_TIMEOUT_MS);
    
    while ((xTaskGetTickCount() - start_time < pdMS_TO_TICKS(SCAN_TIMEOUT_MS))) {
        if (bms_status == JK_BMS_CONNECTED) {
            ESP_LOGI(TAG, "✓✓✓ Connected to JK BMS ✓✓✓");
            return true;
        }
        
        // Если сканирование завершилось и устройство не найдено, пробуем прямое подключение
        if (!scan_active && !device_found && (xTaskGetTickCount() - start_time > pdMS_TO_TICKS(5000))) {
            ESP_LOGW(TAG, "Device not found during scan, trying direct connection...");
            
            // Параметры подключения (из официального примера)
            esp_ble_conn_params_t phy_1m_conn_params = {
                .interval_max = 32,
                .interval_min = 32,
                .latency = 0,
                .supervision_timeout = 600
            };
            
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
                EventBits_t bits = xEventGroupWaitBits(ble_event_group,
                                                       CONNECTED_BIT | DISCONNECTED_BIT,
                                                       pdFALSE,
                                                       pdFALSE,
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
    
    ESP_LOGE(TAG, "Failed to connect to JK BMS (timeout after %d ms)", SCAN_TIMEOUT_MS);
    ESP_LOGE(TAG, "Make sure BMS device is powered on, in range, and advertising");
    bms_status = JK_BMS_DISCONNECTED;
    return false;
}

void jk_bms_disconnect(void)
{
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

jk_bms_status_t jk_bms_get_status(void)
{
    return bms_status;
}

bool jk_bms_get_data(jk_bms_data_t *data)
{
    if (data == NULL) {
        return false;
    }

    if (!bms_data.is_valid) {
        return false;
    }

    memcpy(data, &bms_data, sizeof(jk_bms_data_t));
    return true;
}

void jk_bms_update(void)
{
    // Периодически читаем данные от BMS
    if (bms_status == JK_BMS_CONNECTED && 
        profile_tab[PROFILE_APP_IDX].char_handle != INVALID_HANDLE &&
        profile_tab[PROFILE_APP_IDX].gattc_if != ESP_GATT_IF_NONE) {
        
        // JK-BMS command to read all data (из рабочей версии):
        // AA 55 90 EB 97 00 00 00 00 00 00 00 00 00 00 00 00 00 00 11
        uint8_t cmd[] = {0xAA, 0x55, 0x90, 0xEB, 0x97, 0x00, 0x00, 0x00, 0x00, 0x00, 
                         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x11};
        
        ESP_LOGI(TAG, "Sending read command to BMS (handle=%d, conn_id=%d)", 
                 profile_tab[PROFILE_APP_IDX].char_handle, profile_tab[PROFILE_APP_IDX].conn_id);
        
        // Отправляем команду для чтения данных
        esp_err_t ret = esp_ble_gattc_write_char(profile_tab[PROFILE_APP_IDX].gattc_if,
                                                 profile_tab[PROFILE_APP_IDX].conn_id,
                                                 profile_tab[PROFILE_APP_IDX].char_handle,
                                                 sizeof(cmd),
                                                 cmd,
                                                 ESP_GATT_WRITE_TYPE_NO_RSP,
                                                 ESP_GATT_AUTH_REQ_NONE);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to write command to BMS: %s", esp_err_to_name(ret));
        } else {
            ESP_LOGI(TAG, "✓ Read command sent to BMS successfully");
        }
    } else {
        ESP_LOGW(TAG, "Cannot send BMS command: status=%d, handle=%d, gattc_if=%d",
                 bms_status, profile_tab[PROFILE_APP_IDX].char_handle, 
                 profile_tab[PROFILE_APP_IDX].gattc_if);
    }
}
