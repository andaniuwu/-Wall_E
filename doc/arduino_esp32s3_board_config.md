# Configuración de Arduino IDE para ESP32-S3 (YD-ESP32-S3-N16RB)

Esta configuración ha sido probada y funciona correctamente para cargar el firmware en el módulo YD-ESP32-S3-N16RB (2022-V1.3) y similares.

## Resumen de opciones seleccionadas

- **Board:** `Fri3d Badge 2024 (ESP32-S3-WROOM-1)`
- **Port:** (el puerto COM correspondiente, por ejemplo `COM4`)
- **USB CDC On Boot:** `Enabled`
- **CPU Frequency:** `240MHz (WiFi)`
- **Core Debug Level:** `None`
- **USB DFU On Boot:** `Disabled`
- **Erase All Flash Before Sketch Upload:** `Disabled`
- **Events Run On:** `Core 1`
- **Flash Mode:** `QIO 80MHz`
- **Flash Size:** `16MB (128Mb)`
- **JTAG Adapter:** `Disabled`
- **Arduino Runs On:** `Core 1`
- **USB Firmware MSC On Boot:** `Disabled`
- **Partition Scheme:** `Default 4MB with spiffs (1.2MB APP/1.5MB SPIFFS)`
- **PSRAM:** `OPI PSRAM`
- **Upload Mode:** `UART0 / Hardware CDC`
- **Upload Speed:** `921600`
- **USB Mode:** `Hardware CDC and JTAG`

## Notas
- El nombre de la placa puede variar según la versión del core de ESP32 instalado, pero debe ser una variante ESP32-S3 con soporte para 16MB de flash y PSRAM.
- Si tienes problemas de carga, prueba con diferentes velocidades de subida (`Upload Speed`) o modos de USB.
- Asegúrate de seleccionar el puerto correcto en el menú `Tools > Port`.

## Imagen de referencia

La imagen de configuración utilizada se encuentra en el archivo:
- `doc/arduino_esp32s3_board_config.jpg`

---

> **Recomendación:** Si cambias de computadora o reinstalas el IDE, revisa que todas estas opciones coincidan para evitar problemas de carga o ejecución.
