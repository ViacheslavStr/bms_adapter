#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"

// Включаем заголовки SDK в блоке extern "C"
extern "C" {
    #include "wifi.h"
    #include "mcu_api.h"
}

static const char *TAG = "WBR3_TEST";

// Настройки UART для связи с WBR3
// ESP32-C3 SuperMini → WBR3:
// ESP32 GPIO → WBR3 RXD (прием WBR3)
// ESP32 GPIO → WBR3 TXD (передача WBR3)
// ESP32 GND → WBR3 GND
// ESP32 3.3V → WBR3 VCC
#define WBR3_UART_NUM        UART_NUM_0
#define WBR3_UART_BAUD       9600  // Стандартная скорость для Tuya (может быть 9600 или 115200)
// ВАЖНО: Проверьте ваши пины! В рабочей прошивке используются GPIO20/21
// Если используете GPIO20/21 как в рабочей прошивке, раскомментируйте следующие строки:
#define WBR3_UART_TX_PIN     21    // ESP32-C3 GPIO21 → WBR3 RXD (ESP32 отправляет данные в WBR3)
#define WBR3_UART_RX_PIN     20    // ESP32-C3 GPIO20 → WBR3 TXD (ESP32 получает данные от WBR3)
// Если используете другие пины (например GPIO4/5), раскомментируйте:
// #define WBR3_UART_TX_PIN     4
// #define WBR3_UART_RX_PIN     5
#define WBR3_BUF_SIZE        1024

// Эти функции переопределяют те, что в protocol.c с #error
// Функция отправки данных в UART (нужна для SDK)
// ВАЖНО: SDK вызывает эту функцию для каждого байта отдельно
static uint8_t tx_buffer[256];
static int tx_buffer_pos = 0;
static bool capturing_packet = false;

void uart_transmit_output(unsigned char value)
{
    uart_write_bytes(WBR3_UART_NUM, &value, 1);
    
    // Захватываем начало пакета (55 AA)
    if (!capturing_packet && value == 0x55) {
        tx_buffer_pos = 0;
        tx_buffer[tx_buffer_pos++] = value;
        capturing_packet = true;
    } else if (capturing_packet) {
        tx_buffer[tx_buffer_pos++] = value;
        
        // Если это второй байт AA, начинаем захват
        if (tx_buffer_pos == 2 && value == 0xAA) {
            // Продолжаем захват
        }
        
        // Если пакет завершен (после checksum), логируем
        if (tx_buffer_pos >= 6) {
            int packet_len = (tx_buffer[3] << 8) | tx_buffer[4];
            int total_len = 6 + packet_len; // header + data + checksum
            
            if (tx_buffer_pos >= total_len) {
                // Пакет завершен, логируем
                ESP_LOGI(TAG, ">>> Sending packet to WBR3 (cmd=0x%02X, len=%d):", tx_buffer[3], packet_len);
                for (int i = 0; i < total_len && i < 32; i++) {
                    printf("%02X ", tx_buffer[i]);
                    if ((i + 1) % 16 == 0) printf("\n");
                }
                printf("\n");
                
                // Определяем тип отправляемой команды
                uint8_t cmd = tx_buffer[3];
                const char* cmd_name = "UNKNOWN";
                switch(cmd) {
                    case 0x00: cmd_name = "HEARTBEAT_RESPONSE"; break;
                    case 0x01: cmd_name = "PRODUCT_INFO_RESPONSE"; break;
                    case 0x02: cmd_name = "WORK_MODE_RESPONSE"; break;
                    case 0x03: cmd_name = "WIFI_STATE_RESPONSE"; break;
                    case 0x07: cmd_name = "STATE_UPLOAD"; break;
                }
                ESP_LOGI(TAG, ">>> Sending: %s (cmd=0x%02X)", cmd_name, cmd);
                
                // Если это STATE_UPLOAD (0x07), это отправка DP данных в облако!
                if (cmd == 0x07) {
                    ESP_LOGI(TAG, ">>> ✓✓✓ DP DATA SENT TO CLOUD! ✓✓✓");
                }
                
                capturing_packet = false;
                tx_buffer_pos = 0;
            }
        }
        
        // Защита от переполнения
        if (tx_buffer_pos >= sizeof(tx_buffer)) {
            capturing_packet = false;
            tx_buffer_pos = 0;
        }
    }
}

// Глобальные переменные для хранения текущих значений DP
// В реальном проекте эти значения должны обновляться из ваших датчиков/устройства
static unsigned long current_temperature = 45;      // Текущая температура (°C)
static unsigned char current_status = 1;             // Текущий статус (enum)
static unsigned long current_soc = 85;               // Текущий заряд батареи (%)
static unsigned long battery_current = 500;  // Текущий ток батареи (мА)
static unsigned long battery_voltage = 3700; // Текущее напряжение батареи (мВ)

// Функция обработки всех данных (нужна для SDK)
// Вызывается при STATE_QUERY_CMD (0x08) - WBR3 запрашивает все DP состояния
void all_data_update(void)
{
    ESP_LOGI(TAG, "all_data_update called - reporting all DP states");
    
    // ВАЖНО: Здесь должны быть актуальные значения из вашего устройства!
    // Например:
    // - Температура из датчика температуры
    // - Статус из логики вашего устройства
    // - Данные батареи из BMS или мониторинга питания
    
    // DPID_STATE_OF_CHARGE (101) - Battery Percentage
    // Range: 0-100, Scale: 0, Unit: %
    ESP_LOGI(TAG, "  → Sending DP 101 (Battery Percentage): %lu%%", current_soc);
    unsigned char ret = mcu_dp_value_update(DPID_STATE_OF_CHARGE, current_soc);
    if (ret == SUCCESS) {
        ESP_LOGI(TAG, "  ✓ DP 101 sent successfully");
    } else {
        ESP_LOGW(TAG, "  ✗ DP 101 update failed");
    }
    
    // DPID_BATTERY_CURRENT (102) - Battery Current
    // Range: -200-200, Scale: 3, Unit: A
    // ВАЖНО: Scale: 3 означает, что значение нужно умножить на 1000 для отправки
    // 500 мА = 0.5 А, отправляем 500 (0.5 * 1000)
    unsigned long current_value = battery_current; // 500 мА = 0.5 А = 500 (с scale 3)
    ESP_LOGI(TAG, "  → Sending DP 102 (Battery Current): %lu mA (%.3f A) -> sending %lu", 
             battery_current, battery_current / 1000.0f, current_value);
    ret = mcu_dp_value_update(DPID_BATTERY_CURRENT, current_value);
    if (ret == SUCCESS) {
        ESP_LOGI(TAG, "  ✓ DP 102 sent successfully");
    } else {
        ESP_LOGW(TAG, "  ✗ DP 102 update failed");
    }
    
    // DPID_STATUS (103) - Status (Enum)
    // Enum: charging(0), discharging(1), fault(2), idle(3)
    // current_status = 1 означает "charging"
    ESP_LOGI(TAG, "  → Sending DP 103 (Status): %d", current_status);
    ret = mcu_dp_enum_update(DPID_STATUS, current_status);
    if (ret == SUCCESS) {
        ESP_LOGI(TAG, "  ✓ DP 103 sent successfully");
    } else {
        ESP_LOGW(TAG, "  ✗ DP 103 update failed");
    }
    
    // DPID_COOK_TEMPERATURE (104) - Cook Temperature
    // Range: -50-100, Scale: 0, Unit: °C
    ESP_LOGI(TAG, "  → Sending DP 104 (Cook Temperature): %lu°C", current_temperature);
    ret = mcu_dp_value_update(DPID_COOK_TEMPERATURE, current_temperature);
    if (ret == SUCCESS) {
        ESP_LOGI(TAG, "  ✓ DP 104 sent successfully");
    } else {
        ESP_LOGW(TAG, "  ✗ DP 104 update failed");
    }
    
    // DPID_BATTERY_VOLTAGE (105) - Battery Voltage
    // Range: 0-100, Scale: 1, Unit: V
    // ВАЖНО: Scale: 1 означает, что значение нужно умножить на 10 для отправки
    // 3700 мВ = 3.7 В, отправляем 37 (3.7 * 10)
    unsigned long voltage_value = battery_voltage / 100; // 3700 мВ / 100 = 37 (3.7V * 10)
    ESP_LOGI(TAG, "  → Sending DP 105 (Battery Voltage): %lu mV (%.2f V) -> sending %lu", 
             battery_voltage, battery_voltage / 1000.0f, voltage_value);
    ret = mcu_dp_value_update(DPID_BATTERY_VOLTAGE, voltage_value);
    if (ret == SUCCESS) {
        ESP_LOGI(TAG, "  ✓ DP 105 sent successfully");
    } else {
        ESP_LOGW(TAG, "  ✗ DP 105 update failed");
    }
    
    ESP_LOGI(TAG, "All DP states reported successfully");
    ESP_LOGI(TAG, "Check logs for 'STATE_UPLOAD' packets - these are sent to cloud!");
}

// Вспомогательные функции для обновления значений DP (можно вызывать из других частей кода)
// Эти функции можно использовать для обновления значений при изменении данных
void update_temperature(unsigned long temp)
{
    current_temperature = temp;
    mcu_dp_value_update(DPID_COOK_TEMPERATURE, current_temperature);
    ESP_LOGI(TAG, "Temperature updated: %lu°C", current_temperature);
}

void update_status(unsigned char status)
{
    current_status = status;
    mcu_dp_enum_update(DPID_STATUS, current_status);
    ESP_LOGI(TAG, "Status updated: %d", current_status);
}

void update_battery_data(unsigned long soc, unsigned long current, unsigned long voltage)
{
    current_soc = soc;
    battery_current = current;
    battery_voltage = voltage;
    
    mcu_dp_value_update(DPID_STATE_OF_CHARGE, current_soc);
    mcu_dp_value_update(DPID_BATTERY_CURRENT, battery_current); // Scale: 3, уже в правильном формате
    mcu_dp_value_update(DPID_BATTERY_VOLTAGE, battery_voltage / 100); // Scale: 1, вольты * 10
    
    ESP_LOGI(TAG, "Battery data updated: SOC=%lu%%, I=%lu mA, U=%lu mV", 
             current_soc, battery_current, battery_voltage);
}

// Вспомогательная функция для обработки температуры (нужна для protocol.c)
static unsigned char dp_download_cook_temperature_handle(const unsigned char value[], unsigned short length)
{
    ESP_LOGI(TAG, "Cook temperature: len=%d", length);
    return 1; // SUCCESS
}

// Обработка DP команд (нужна для SDK)
unsigned char dp_download_handle(unsigned char dpid, const unsigned char value[], unsigned short length)
{
    ESP_LOGI(TAG, "DP download: dpid=%d, len=%d", dpid, length);
    
    // Обработка конкретных DP
    switch(dpid) {
        case 101: // DPID_STATE_OF_CHARGE - Battery Percentage (можно изменять из приложения)
            ESP_LOGI(TAG, "Battery Percentage changed from app");
            return 1; // SUCCESS
        case 104: // DPID_COOK_TEMPERATURE - Cook Temperature (можно изменять из приложения)
            return dp_download_cook_temperature_handle(value, length);
        default:
            ESP_LOGI(TAG, "Unknown DP: %d", dpid);
            return 1; // SUCCESS
    }
}

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "WBR3 Test for ESP32-C3");
    ESP_LOGI(TAG, "========================================");
    
    // Настройка UART
    uart_config_t uart_config = {
        .baud_rate = WBR3_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    // ВАЖНО: Устанавливаем буферы ПЕРЕД установкой пинов
    // RX буфер должен быть достаточно большим для приема данных
    ESP_ERROR_CHECK(uart_driver_install(WBR3_UART_NUM, WBR3_BUF_SIZE * 2, WBR3_BUF_SIZE * 2, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(WBR3_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(WBR3_UART_NUM, WBR3_UART_TX_PIN, WBR3_UART_RX_PIN, 
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    
    ESP_LOGI(TAG, "UART initialized: TX=%d, RX=%d, BAUD=%d", 
             WBR3_UART_TX_PIN, WBR3_UART_RX_PIN, WBR3_UART_BAUD);
    
    // Проверка UART - очистка буферов
    uart_flush(WBR3_UART_NUM);
    ESP_LOGI(TAG, "UART buffers flushed");
    
    // Инициализация протокола Tuya
    ESP_LOGI(TAG, "Initializing Tuya protocol...");
    wifi_protocol_init();
    ESP_LOGI(TAG, "Protocol initialized");
    
    // Пауза перед началом (даем WBR3 время на инициализацию)
    ESP_LOGI(TAG, "Waiting for WBR3 to initialize...");
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // Проверяем текущее состояние WiFi перед запуском SmartConfig
    // ВАЖНО: Не запускаем SmartConfig сразу - даем WBR3 время восстановить сохраненные настройки
    ESP_LOGI(TAG, "Checking WiFi connection status...");
    ESP_LOGI(TAG, "Waiting for WBR3 to restore saved WiFi settings (up to 10 seconds)...");
    
    // Даем больше времени WBR3 обработать запросы и восстановить состояние
    // WBR3 должен автоматически восстановить сохраненные WiFi настройки
    // Проверяем состояние WiFi несколько раз с увеличивающимися интервалами
    unsigned char wifi_state = WIFI_SATE_UNKNOW;
    bool wifi_configured = false;
    
    for (int i = 0; i < 100; i++) {  // До 10 секунд (100 * 100ms)
        wifi_uart_service();
        vTaskDelay(pdMS_TO_TICKS(100));
        
        // Проверяем состояние WiFi каждые 500ms
        if (i % 5 == 0) {
            wifi_state = mcu_get_wifi_work_state();
            
            // Если устройство подключено к облаку или WiFi, значит настройки сохранены
            if (wifi_state == WIFI_CONN_CLOUD || wifi_state == WIFI_CONNECTED) {
                wifi_configured = true;
                ESP_LOGI(TAG, "WiFi connection restored! State: 0x%02X", wifi_state);
                break;
            }
            
            // Если устройство в режиме SmartConfig или AP, значит настройки не сохранены
            if (wifi_state == SMART_CONFIG_STATE || wifi_state == AP_STATE) {
                ESP_LOGI(TAG, "WiFi not configured. State: 0x%02X", wifi_state);
                break;
            }
        }
    }
    
    // Финальная проверка состояния WiFi
    wifi_state = mcu_get_wifi_work_state();
    ESP_LOGI(TAG, "Final WiFi state: 0x%02X", wifi_state);
    
    // Запускаем SmartConfig ТОЛЬКО если устройство не подключено к облаку/WiFi
    // И если устройство явно в режиме настройки (SMART_CONFIG или AP)
    // НЕ запускаем SmartConfig, если состояние неизвестно - даем больше времени
    if (wifi_state == SMART_CONFIG_STATE || wifi_state == AP_STATE || 
        (wifi_state == WIFI_NOT_CONNECTED && !wifi_configured)) {
        ESP_LOGI(TAG, "WiFi not connected. Starting SmartConfig mode...");
        ESP_LOGI(TAG, "To connect WBR3 to WiFi:");
        ESP_LOGI(TAG, "  1. Open Tuya Smart app on your phone");
        ESP_LOGI(TAG, "  2. Add device -> WiFi device");
        ESP_LOGI(TAG, "  3. Enter your WiFi password");
        ESP_LOGI(TAG, "  4. WBR3 will connect automatically");
        mcu_set_wifi_mode(0); // 0 = SMART_CONFIG mode
        vTaskDelay(pdMS_TO_TICKS(500));
    } else if (wifi_state == WIFI_CONN_CLOUD || wifi_state == WIFI_CONNECTED) {
        ESP_LOGI(TAG, "WiFi already configured! Device should reconnect automatically.");
        ESP_LOGI(TAG, "No need to enter SmartConfig mode.");
    } else if (wifi_state == WIFI_SATE_UNKNOW || wifi_state == WIFI_LOW_POWER) {
        ESP_LOGI(TAG, "WiFi state unknown or in low power mode. Waiting for connection...");
        ESP_LOGI(TAG, "Device may be trying to reconnect. Not starting SmartConfig.");
        ESP_LOGI(TAG, "If device doesn't connect, it will need to be added again in Tuya app.");
    } else {
        ESP_LOGI(TAG, "WiFi state: 0x%02X. Waiting for connection...", wifi_state);
    }
    
    // Проверяем, есть ли данные в буфере
    size_t available = 0;
    uart_get_buffered_data_len(WBR3_UART_NUM, &available);
    if (available > 0) {
        ESP_LOGI(TAG, "Found %d bytes in UART buffer (clearing)", available);
        uart_flush_input(WBR3_UART_NUM);
    }
    
    ESP_LOGI(TAG, "Starting main loop...");
    ESP_LOGI(TAG, "Waiting for heartbeat from WBR3...");
    ESP_LOGI(TAG, "If no data received, check:");
    ESP_LOGI(TAG, "  1. UART speed matches WBR3 (try 9600 or 115200)");
    ESP_LOGI(TAG, "  2. TX/RX pins are connected correctly");
    ESP_LOGI(TAG, "  3. WBR3 is powered (3.3V)");
    
    // Тестовая отправка - отправляем тестовый байт для проверки TX
    ESP_LOGI(TAG, "Sending test byte to check TX line...");
    uint8_t test_byte = 0x55;
    uart_write_bytes(WBR3_UART_NUM, &test_byte, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Проверяем состояние RX пина (может помочь диагностике)
    gpio_set_direction((gpio_num_t)WBR3_UART_RX_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode((gpio_num_t)WBR3_UART_RX_PIN, GPIO_PULLUP_ONLY);
    int rx_level = gpio_get_level((gpio_num_t)WBR3_UART_RX_PIN);
    ESP_LOGI(TAG, "RX pin (GPIO%d) level: %d (1=idle/high, 0=low)", WBR3_UART_RX_PIN, rx_level);
    if (rx_level == 0) {
        ESP_LOGW(TAG, "WARNING: RX pin is LOW - check connection or WBR3 power!");
    }
    
    uint8_t data[256];
    int len;
    int total_received = 0;
    int loop_count = 0;
    bool tried_115200 = false;
    
    // Главный цикл
    while (1) {
        loop_count++;
        
        // Периодически выводим статус
        if (loop_count % 100 == 0) {
            size_t available = 0;
            uart_get_buffered_data_len(WBR3_UART_NUM, &available);
            ESP_LOGI(TAG, "Loop %d: total received=%d, buffer=%d bytes", 
                     loop_count, total_received, available);
            
            // Если долго нет данных, попробуем переключить скорость на 115200
            if (total_received == 0 && loop_count == 300 && !tried_115200) {
                ESP_LOGW(TAG, "No data received after 300 loops. Trying 115200 baud...");
                uart_set_baudrate(WBR3_UART_NUM, 115200);
                tried_115200 = true;
                ESP_LOGI(TAG, "UART speed changed to 115200. Waiting for data...");
                // Очищаем буферы
                uart_flush(WBR3_UART_NUM);
            }
        }
        
        // Чтение данных из UART - простая логика как в рабочей прошивке
        // Читаем все доступные данные
        len = uart_read_bytes(WBR3_UART_NUM, data, sizeof(data) - 1, pdMS_TO_TICKS(50));
        
        if (len > 0) {
            total_received += len;
            
            // Логируем все данные для диагностики
            ESP_LOGI(TAG, "Received %d bytes (total: %d)", len, total_received);
            
            // Вывод hex дампа для отладки
            ESP_LOGI(TAG, "Raw data (hex):");
            for (int i = 0; i < len && i < 64; i++) {
                printf("%02X ", data[i]);
                if ((i + 1) % 16 == 0) printf("\n");
            }
            if (len <= 64) printf("\n");
            printf("\n");
            
            // КРИТИЧНО: Передаем ВСЕ данные в SDK для обработки
            // SDK сам соберет пакеты из отдельных байтов и отфильтрует мусор
            // НЕ фильтруем байты сами - SDK знает формат пакетов лучше нас!
            for (int i = 0; i < len; i++) {
                uart_receive_input(data[i]);
            }
            
            // Определяем тип команды от WBR3 (только для логирования)
            if (len >= 6 && data[0] == 0x55 && data[1] == 0xAA) {
                uint8_t cmd = data[3];
                const char* cmd_name = "UNKNOWN";
                switch(cmd) {
                    case 0x00: cmd_name = "HEARTBEAT"; break;
                    case 0x01: cmd_name = "PRODUCT_INFO"; break;
                    case 0x02: cmd_name = "WORK_MODE"; break;
                    case 0x03: cmd_name = "WIFI_STATE"; break;
                    case 0x06: cmd_name = "DATA_QUERY"; break;
                    case 0x07: cmd_name = "STATE_UPLOAD"; break;
                    case 0x08: cmd_name = "STATE_QUERY"; break;
                }
                ESP_LOGI(TAG, ">>> Command received: %s (0x%02X)", cmd_name, cmd);
                
                // Проверяем статус WiFi из WIFI_STATE команды
                if (cmd == 0x03 && len >= 8) {
                    uint8_t wifi_state = data[6];
                    const char* wifi_state_name = "UNKNOWN";
                    switch(wifi_state) {
                        case 0x00: wifi_state_name = "SMART_CONFIG"; break;
                        case 0x01: wifi_state_name = "AP_MODE"; break;
                        case 0x02: wifi_state_name = "WIFI_NOT_CONNECTED"; break;
                        case 0x03: wifi_state_name = "WIFI_CONNECTED"; break;
                        case 0x04: wifi_state_name = "WIFI_CONN_CLOUD"; break;  // Подключено к облаку!
                        case 0x05: wifi_state_name = "LOW_POWER"; break;
                        case 0x06: wifi_state_name = "SMART_AP_MODE"; break;
                    }
                    ESP_LOGI(TAG, ">>> WiFi State: %s (0x%02X)", wifi_state_name, wifi_state);
                    if (wifi_state == 0x04) {
                        ESP_LOGI(TAG, ">>> ✓✓✓ CONNECTED TO TUYA CLOUD! Data will be sent! ✓✓✓");
                    } else if (wifi_state == 0x03) {
                        ESP_LOGW(TAG, ">>> ⚠ WiFi connected but NOT to cloud yet");
                    } else {
                        ESP_LOGW(TAG, ">>> ⚠ WiFi NOT connected - data won't reach cloud");
                    }
                }
            }
        }
        
        // ВАЖНО: wifi_uart_service должен вызываться постоянно,
        // даже если данных нет (для обработки таймеров и отправки ответов)
        wifi_uart_service();
        
        // Периодически проверяем статус WiFi подключения
        if (loop_count % 500 == 0) {
            unsigned char wifi_state = mcu_get_wifi_work_state();
            const char* wifi_state_name = "UNKNOWN";
            switch(wifi_state) {
                case 0x00: wifi_state_name = "SMART_CONFIG"; break;
                case 0x01: wifi_state_name = "AP_MODE"; break;
                case 0x02: wifi_state_name = "WIFI_NOT_CONNECTED"; break;
                case 0x03: wifi_state_name = "WIFI_CONNECTED"; break;
                case 0x04: wifi_state_name = "WIFI_CONN_CLOUD"; break;  // Подключено к облаку!
                case 0x05: wifi_state_name = "LOW_POWER"; break;
                case 0x06: wifi_state_name = "SMART_AP_MODE"; break;
            }
            ESP_LOGI(TAG, "WiFi Status Check: %s (0x%02X)", wifi_state_name, wifi_state);
            if (wifi_state == 0x04) {
                ESP_LOGI(TAG, ">>> ✓✓✓ CONNECTED TO TUYA CLOUD - Data will be sent! ✓✓✓");
            } else if (wifi_state == 0x03) {
                ESP_LOGW(TAG, ">>> ⚠ WiFi connected but NOT to cloud yet");
            } else if (wifi_state == 0x00) {
                ESP_LOGI(TAG, ">>> 📱 SmartConfig mode - Use Tuya Smart app to connect WiFi");
            } else if (wifi_state == 0x05) {
                ESP_LOGW(TAG, ">>> ⚠ LOW_POWER mode - Device may be trying to reconnect");
                // НЕ запускаем SmartConfig автоматически - даем устройству время восстановить подключение
                // WBR3 должен сам попытаться переподключиться с сохраненными настройками
            } else {
                ESP_LOGW(TAG, ">>> ⚠ WiFi NOT connected - data won't reach cloud");
            }
        }
        
        // Небольшая задержка для стабильности
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

