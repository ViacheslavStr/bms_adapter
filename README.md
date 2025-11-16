# Тест WBR3 для ESP32-C3

Простой тестовый код для проверки связи с WBR3.

## Настройка пинов

Отредактируйте `main/main.c` и измените пины под вашу распайку:

```c
#define WBR3_UART_TX_PIN     4     // Ваш GPIO для TX
#define WBR3_UART_RX_PIN     5     // Ваш GPIO для RX
#define WBR3_UART_BAUD       9600  // Может быть 115200
```

## Сборка

```bash
cd wbr3_test_esp32c3
idf.py set-target esp32c3
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

## Что проверяет:

1. Инициализация UART
2. Инициализация протокола Tuya
3. Heartbeat от WBR3
4. Обмен данными по протоколу Tuya
5. Выводит hex дамп всех полученных данных

## Что смотреть в логах:

- "UART initialized" - UART настроен
- "Protocol initialized" - протокол Tuya инициализирован
- "Received X bytes from WBR3" - данные получены
- Hex дамп данных - показывает что именно приходит от WBR3

Если видите heartbeat (регулярные сообщения) - WBR3 работает!
Если видите данные протокола Tuya - все настроено правильно!

