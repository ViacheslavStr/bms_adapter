# Тест WBR3 для ESP32-C3 (PlatformIO)

## Использование:

1. **Откройте проект в PlatformIO:**
   ```bash
   cd wbr3_test_esp32c3
   # Откройте в VS Code с расширением PlatformIO
   ```

2. **Настройте пины** в `src/main.cpp`:
   ```cpp
   #define WBR3_UART_TX_PIN     4     // Ваш GPIO для TX
   #define WBR3_UART_RX_PIN     5     // Ваш GPIO для RX
   #define WBR3_UART_BAUD       9600  // Попробуйте 9600 или 115200
   ```

3. **Соберите и загрузите:**
   - В PlatformIO: Build → Upload
   - Или через терминал: `pio run -t upload`

4. **Откройте монитор:**
   - В PlatformIO: Monitor
   - Или: `pio device monitor`

## Что проверяет:

- Инициализация UART
- Инициализация протокола Tuya
- Heartbeat от WBR3
- Обмен данными по протоколу Tuya
- Выводит hex дамп всех полученных данных

## Что смотреть в логах:

- "UART initialized" - UART настроен
- "Protocol initialized" - протокол Tuya инициализирован
- "Received X bytes from WBR3" - данные получены
- Hex дамп данных - показывает что именно приходит от WBR3

Если видите heartbeat (регулярные сообщения) - WBR3 работает!
Если видите данные протокола Tuya - все настроено правильно!

