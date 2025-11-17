#ifndef JK_BMS_H
#define JK_BMS_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// MAC адрес JK BMS
#define JK_BMS_MAC_ADDR "C8:47:80:21:7F:97"

// Статусы подключения
typedef enum {
    JK_BMS_DISCONNECTED,
    JK_BMS_CONNECTING,
    JK_BMS_CONNECTED,
    JK_BMS_ERROR
} jk_bms_status_t;

// Структура для хранения данных BMS
typedef struct {
    uint8_t soc;              // State of Charge (0-100%)
    float voltage;            // Напряжение (В)
    float current;            // Ток (А)
    float temperature;        // Температура (°C)
    bool is_valid;            // Флаг валидности данных
} jk_bms_data_t;

/**
 * @brief Инициализация JK BMS модуля
 * @return true если успешно, false в случае ошибки
 */
bool jk_bms_init(void);

/**
 * @brief Подключение к JK BMS
 * @return true если подключение начато, false в случае ошибки
 */
bool jk_bms_connect(void);

/**
 * @brief Отключение от JK BMS
 */
void jk_bms_disconnect(void);

/**
 * @brief Получить текущий статус подключения
 * @return Статус подключения
 */
jk_bms_status_t jk_bms_get_status(void);

/**
 * @brief Получить последние данные от BMS
 * @param data Указатель на структуру для записи данных
 * @return true если данные получены, false если данных нет
 */
bool jk_bms_get_data(jk_bms_data_t *data);

/**
 * @brief Обновление состояния BMS (вызывать периодически)
 */
void jk_bms_update(void);

#ifdef __cplusplus
}
#endif

#endif // JK_BMS_H

