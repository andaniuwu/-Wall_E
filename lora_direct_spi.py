#!/usr/bin/env python3
"""
Wall-E LoRa Receiver - Direct SPI Implementation
============================================================
Este programa implementa un receptor LoRa sin usar la librería SX127x.
Comunica directamente con el módulo SX1276/SX1278 a través del protocolo SPI.

Configuración de Hardware:
- Frecuencia: 433 MHz
- Ancho de banda: 125 kHz
- Spreading Factor: 7
- Coding Rate: 4/5

Pines GPIO Raspberry Pi 4:
- DIO0 (GPIO 4 / Pin 7):  Interrupción RX done
- DIO1 (GPIO 17 / Pin 11): Interrupción timeout
- DIO2 (GPIO 18 / Pin 12): Interrupción FhssChangeChannel
- DIO3 (GPIO 27 / Pin 13): Interrupción CadDone
- RST  (GPIO 22 / Pin 15): Reset del módulo

Pines SPI (utilizados automáticamente):
- MOSI: GPIO 10 (Pin 19)
- MISO: GPIO 9 (Pin 21)
- CLK:  GPIO 11 (Pin 23)
- CE0:  GPIO 8 (Pin 24)
"""

import time
import RPi.GPIO as GPIO
import spidev
import sys

# ============================================================
# CONFIGURACIÓN DE PINES GPIO
# ============================================================
DIO0 = 4    # Data IO pin 0 - Señal RX Done
DIO1 = 17   # Data IO pin 1 - Señal Timeout
DIO2 = 18   # Data IO pin 2 - Señal FHSS Change Channel
DIO3 = 27   # Data IO pin 3 - Señal CAD Done
RST = 22    # Reset pin
CS = 8      # Chip Select (SPI)

# ============================================================
# REGISTROS DEL MÓDULO LORA (SX1276/SX1278)
# ============================================================
# Estos valores hexadecimales representan las direcciones de
# los registros internos del módulo LoRa que configuran su
# funcionamiento (frecuencia, ancho de banda, etc.)
REG_FIFO = 0x00                  # Buffer FIFO para datos RX/TX
REG_FIFO_ADDR_PTR = 0x0D        # Puntero de dirección FIFO
REG_FIFO_RX_BASE_ADDR = 0x0F    # Dirección base FIFO RX
REG_FIFO_RX_CURRENT_ADDR = 0x10 # Dirección actual lectura FIFO RX
REG_IRQ_FLAGS = 0x12             # Flags de interrupciones
REG_RX_NB_BYTES = 0x13           # Número de bytes recibidos
REG_OP_MODE = 0x01               # Modo de operación
REG_FREQ_MSB = 0x06              # Bits altos de frecuencia
REG_FREQ_MID = 0x07              # Bits medios de frecuencia
REG_FREQ_LSB = 0x08              # Bits bajos de frecuencia
REG_PA_CONFIG = 0x09             # Configuración amplificador
REG_MODEM_CONFIG_1 = 0x1D        # Configuración módem 1
REG_MODEM_CONFIG_2 = 0x1E        # Configuración módem 2
REG_SYNC_WORD = 0x39             # Palabra de sincronización
REG_PREAMBLE_MSB = 0x20          # Preámbulo MSB
REG_PREAMBLE_LSB = 0x21          # Preámbulo LSB

# ============================================================
# VARIABLES GLOBALES
# ============================================================
spi = None              # Objeto del dispositivo SPI
message_count = 0       # Contador de mensajes recibidos

# ============================================================
# FUNCIONES DE COMUNICACIÓN SPI
# ============================================================

def spi_read(reg):
    """
    Lee un valor de un registro del módulo LoRa via SPI
    
    Parámetros:
        reg: Dirección del registro (0x00-0xFF)
    
    Retorna:
        Valor del registro (0-255)
    
    Protocolo SPI:
        - Byte 1: Dirección (con bit 7 = 0 para lectura)
        - Byte 2: 0x00 (dummy para recibir el dato)
    """
    resp = spi.xfer2([reg & 0x7F, 0x00])  # 0x7F limpia bit 7 (lectura)
    return resp[1]  # Retorna el segundo byte (el valor)

def spi_write(reg, value):
    """
    Escribe un valor en un registro del módulo LoRa via SPI
    
    Parámetros:
        reg: Dirección del registro (0x00-0xFF)
        value: Valor a escribir (0-255)
    
    Protocolo SPI:
        - Byte 1: Dirección (con bit 7 = 1 para escritura)
        - Byte 2: Valor a escribir
    """
    spi.xfer2([reg | 0x80, value])  # 0x80 pone bit 7 (escritura)

# ============================================================
# INICIALIZACIÓN DEL HARDWARE
# ============================================================

def initialize():
    """
    Inicializa los pines GPIO y la interfaz SPI
    
    Configuración GPIO:
        - DIO0-DIO3: Entrada (INT signals del módulo)
        - RST: Salida (control de reset)
    
    Proceso:
        1. Configura los pines GPIO en modo BCM (By Chip Number)
        2. Configura pines DIO como entradas
        3. Configura pin RST como salida
        4. Hace reset del módulo LoRa (LOW -> HIGH)
        5. Inicializa interfaz SPI con velocidad 3.9 MHz
    
    Retorna:
        True si la inicialización fue exitosa
        False si hubo error
    """
    global spi
    
    # INICIALIZAR GPIO
    try:
        GPIO.setmode(GPIO.BCM)  # Usa numeración de pines por chip (GPIO 4, 17, etc)
        GPIO.setwarnings(False) # Desactiva advertencias de GPIO ya en uso
        
        # Configura pines DIO como entradas (reciben señales del módulo)
        GPIO.setup([DIO0, DIO1, DIO2, DIO3], GPIO.IN)
        # Configura pin RST como salida (controlamos el reset del módulo)
        GPIO.setup(RST, GPIO.OUT)
        
        # RESET DEL MÓDULO LORA
        # El reset requiere que RST vaya LOW luego HIGH
        GPIO.output(RST, GPIO.LOW)   # Pone RST en bajo (reset activo)
        time.sleep(0.01)              # Espera 10ms
        GPIO.output(RST, GPIO.HIGH)  # Pone RST en alto (reset liberado)
        time.sleep(0.1)               # Espera 100ms para que arranque el módulo
        print("✓ LoRa module reset")
    except Exception as e:
        print(f"✗ GPIO Error: {e}")
        return False

    # INICIALIZAR SPI
    try:
        spi = spidev.SpiDev()        # Crea objeto SPI
        spi.open(0, 0)               # Abre bus 0, chip select 0
        spi.max_speed_hz = 3900000   # Velocidad: 3.9 MHz (compatible con SX1276)
        print("✓ SPI initialized")
    except Exception as e:
        print(f"✗ SPI Error: {e}")
        return False
    
    return True



def setup_lora():
    """
    Configura el módulo LoRa para operar en modo RX a 433 MHz
    
    Configuración aplicada:
    - Frecuencia: 433 MHz
    - Ancho de banda: 125 kHz
    - Spreading Factor: 7 (balance entre alcance y velocidad)
    - Coding Rate: 4/5 (corrección de errores)
    - Modo: RX Continuo (escucha permanente)
    - Palabra de sincronización: 0x34 (redes privadas)
    
    Proceso:
    1. Calcula el valor de frecuencia desde 433 MHz a registro SX1276
    2. Escribe valores en registros de configuración
    3. Pone el módulo en modo RX continuo (0x85)
    4. Espera a que el módulo se estabilice
    
    Retorna:
        True si la configuración fue exitosa
        False si hubo error
    """
    try:
        # ============================================================
        # 1. CONFIGURAR FRECUENCIA A 433 MHz
        # ============================================================
        # La fórmula es: Frf = Frecuencia_deseada / (Fxosc / 2^19)
        # Donde Fxosc = 32 MHz (frecuencia de reloj del SX1276)
        # Frf = 433,000,000 / 61.03515625 ≈ 7,095,552 = 0x6C8000
        
        freq = 433000000  # Frecuencia en Hz (433 MHz)
        frf = int(freq / 61.03515625)  # Convierte a valor de registro
        
        # Escribe el valor de frecuencia en 3 registros (24 bits)
        # MSB: bits 23-16, MID: bits 15-8, LSB: bits 7-0
        spi_write(REG_FREQ_MSB, (frf >> 16) & 0xFF)  # Bits altos
        spi_write(REG_FREQ_MID, (frf >> 8) & 0xFF)   # Bits medios
        spi_write(REG_FREQ_LSB, frf & 0xFF)          # Bits bajos
        
        # ============================================================
        # 2. CONFIGURAR AMPLIFICADOR DE POTENCIA
        # ============================================================
        # 0xFF = Máxima potencia (20 dBm)
        spi_write(REG_PA_CONFIG, 0xFF)
        
        # ============================================================
        # 3. CONFIGURAR MÓDEM 1 (Ancho de banda y Coding Rate)
        # ============================================================
        # 0x72 = 0111 0010 en binario
        #   - Bits 7-4: 0111 = Ancho de banda 125 kHz
        #   - Bits 3-1: 001 = Coding Rate 4/5 (1 bit de paridad por 4 de datos)
        #   - Bit 0: 0 = Modo explícito (header presente)
        spi_write(REG_MODEM_CONFIG_1, 0x72)
        
        # ============================================================
        # 4. CONFIGURAR MÓDEM 2 (Spreading Factor y CRC)
        # ============================================================
        # 0x74 = 0111 0100 en binario
        #   - Bits 7-4: 0111 = Spreading Factor 7 (128 chips/símbolo)
        #   - Bit 3: 0 = TX normal (no modo continuo)
        #   - Bit 2: 1 = RX con CRC habilitado (detecta errores)
        #   - Bits 1-0: 00 = (bits reservados)
        # SF más alto = más alcance pero más lento. SF 7 es buen balance
        spi_write(REG_MODEM_CONFIG_2, 0x74)
        
        # ============================================================
        # 5. PALABRA DE SINCRONIZACIÓN
        # ============================================================
        # 0x34 es la palabra estándar para redes privadas LoRa
        # (0x12 se usa para redes LoRaWAN públicas)
        spi_write(REG_SYNC_WORD, 0x34)
        
        # ============================================================
        # 6. LONGITUD DEL PREÁMBULO
        # ============================================================
        # 8 símbolos de preámbulo (valor por defecto)
        # El preámbulo ayuda al receptor a sincronizar
        spi_write(REG_PREAMBLE_MSB, 0x00)
        spi_write(REG_PREAMBLE_LSB, 0x08)
        
        # ============================================================
        # 7. PONER EN MODO RX CONTINUO
        # ============================================================
        # 0x85 = 1000 0101 en binario
        #   - Bits 7-5: 100 = Modo LoRa (no FSK)
        #   - Bits 4-2: 001 = Modo RX Continuo (escucha permanente)
        #   - Bits 1-0: 01 = (modo especificado arriba)
        spi_write(REG_OP_MODE, 0x85)
        
        # Espera a que el módulo estabilice su estado
        time.sleep(0.1)
        
        # Imprime confirmación con parámetros
        print("✓ LoRa configured:")
        print("  - Frequency: 433 MHz")
        print("  - Bandwidth: 125 kHz")
        print("  - Spreading Factor: 7")
        print("  - Coding Rate: 4/5")
        return True
    except Exception as e:
        print(f"✗ Configuration error: {e}")
        return False

def read_message():
    """
    Lee un mensaje del buffer FIFO si está disponible
    
    Proceso:
    1. Lee el registro de flags de interrupción (IRQ_FLAGS)
    2. Verifica si hay un mensaje completamente recibido (RxDone = bit 6)
    3. Si hay mensaje:
       - Lee el número de bytes recibidos
       - Apunta el puntero FIFO a la dirección de lectura
       - Lee los bytes uno por uno del buffer FIFO
       - Limpia los flags de interrupción (reset)
       - Incrementa contador de mensajes
       - Retorna los datos
    4. Si no hay mensaje, retorna None
    
    Flags de interrupción importantes:
    - Bit 6 (RxDone): Mensaje completamente recibido
    - Bit 5 (RxTimeout): Timeout esperando mensaje
    - Bit 4 (PayloadCrcError): Error en CRC (corrupción)
    
    Retorna:
        (bytes_del_mensaje, contador) si hay mensaje
        (None, contador) si no hay mensaje
    """
    global message_count
    
    try:
        # Lee el registro de estados de interrupción
        # Este registro indica qué eventos han ocurrido
        irq_flags = spi_read(REG_IRQ_FLAGS)
        
        # Verifica el bit 6 (RxDone) = mensaje completamente recibido
        # 0x40 en binario es 0100 0000 (bit 6 activado)
        if irq_flags & 0x40:
            # ========== MENSAJE RECIBIDO ==========
            
            # Lee cuántos bytes se recibieron
            nb_bytes = spi_read(REG_RX_NB_BYTES)
            
            # Apunta el puntero FIFO a la dirección de inicio
            # del mensaje recibido
            rx_base = spi_read(REG_FIFO_RX_BASE_ADDR)
            spi_write(REG_FIFO_ADDR_PTR, rx_base)
            
            # Lee todos los bytes del buffer FIFO
            # El FIFO es una cola: cada lectura da el siguiente byte
            data = []
            for i in range(nb_bytes):
                byte = spi_read(REG_FIFO)  # Lee un byte
                data.append(byte)
            
            # Limpia todos los flags de interrupción para la próxima lectura
            # 0xFF limpia todos los bits (16 bits = 0xFFFF en algunos chips)
            spi_write(REG_IRQ_FLAGS, 0xFF)
            
            # Incrementa el contador de mensajes recibidos
            message_count += 1
            
            # Retorna los datos como bytes
            return bytes(data), message_count
    except Exception as e:
        # Si hay error, lo ignora silenciosamente (polling mode)
        pass
    
    # No hay mensaje disponible
    return None, message_count


def main():
    """
    Función principal - Inicia el receptor LoRa
    
    Proceso:
    1. Imprime banner de bienvenida
    2. Inicializa hardware (GPIO + SPI)
    3. Configura módulo LoRa (433 MHz)
    4. Entra en loop de escucha continua
    5. Al recibir Ctrl+C, limpia recursos y sale
    
    Loop principal:
    - Cada 50ms: intenta leer un mensaje
    - Si hay mensaje: lo imprime con timestamp y contador
    - Cada 5 segundos: imprime status de escucha
    - Captura Ctrl+C para salida limpia
    """
    print("=" * 70)
    print("     Wall-E LoRa Receiver - Direct SPI Implementation")
    print("=" * 70)
    
    # Inicializa hardware (GPIO + SPI)
    if not initialize():
        print("✗ Failed to initialize hardware")
        return
    
    # Configura el módulo LoRa para 433 MHz
    if not setup_lora():
        print("✗ Failed to configure LoRa")
        return
    
    print("\n✓ Listening for LoRa messages on 433 MHz...")
    print("Press Ctrl+C to exit\n")
    
    try:
        # Control de tiempo para imprimir status
        last_print = time.time()
        
        # ========== LOOP PRINCIPAL ==========
        while True:
            # Intenta leer un mensaje (retorna None si no hay)
            msg, count = read_message()
            
            if msg:
                # ===== MENSAJE RECIBIDO =====
                # Formatea con timestamp y cantidad de bytes
                timestamp = time.strftime("%H:%M:%S")
                print(f"[{timestamp}] Message #{count}: {msg} ({len(msg)} bytes)")
            else:
                # ===== ESPERANDO MENSAJE =====
                # Cada 5 segundos, imprime que sigue escuchando
                if time.time() - last_print > 5:
                    timestamp = time.strftime("%H:%M:%S")
                    print(f"[{timestamp}] Listening... (Messages received: {count})")
                    last_print = time.time()
            
            # Pequeña pausa para no saturar CPU (50ms)
            time.sleep(0.05)
            
    except KeyboardInterrupt:
        # Usuario presionó Ctrl+C
        print("\n✓ Exiting gracefully...")
    except Exception as e:
        # Error inesperado
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # ========== LIMPIEZA DE RECURSOS ==========
        # Importante: siempre cerrar SPI y GPIO, aunque haya error
        try:
            spi.close()           # Cierra comunicación SPI
            GPIO.cleanup()        # Libera pines GPIO
            print("✓ Resources cleaned up")
        except:
            # Si hay error en cleanup, lo ignora
            pass

# ============================================================
# PUNTO DE ENTRADA DEL PROGRAMA
# ============================================================
if __name__ == "__main__":
    # Solo ejecuta main() si se llama directamente
    # (no si se importa como módulo en otro script)
    main()


