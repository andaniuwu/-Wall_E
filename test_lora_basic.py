#!/usr/bin/env python3
"""
Test básico del módulo LoRa SX1278 en Raspberry Pi
Diagnóstico de comunicación SPI
"""

import time
import RPi.GPIO as GPIO
import spidev

# GPIO Pin mappings (BCM numbering)
GPIO_RST = 22

# LoRa Registers
REG_OP_MODE = 0x01
REG_FRF_MSB = 0x06
REG_FRF_MID = 0x07
REG_FRF_LSB = 0x08
REG_MODEM_CONFIG_1 = 0x1D
REG_MODEM_CONFIG_2 = 0x1E
REG_SYNC_WORD = 0x39
REG_VERSION = 0x42

spi = None

def spi_read(register):
    """Read from SX1278 register via SPI"""
    result = spi.xfer2([register & 0x7F, 0x00])
    return result[1]

def spi_write(register, value):
    """Write to SX1278 register via SPI"""
    spi.xfer2([register | 0x80, value])

def main():
    global spi
    
    print("="*70)
    print("          Diagnóstico LoRa SX1278 - Raspberry Pi")
    print("="*70)
    
    # Initialize GPIO
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(GPIO_RST, GPIO.OUT)
    
    # Hard reset
    print("\n1. Reseteando módulo LoRa...")
    GPIO.output(GPIO_RST, GPIO.LOW)
    time.sleep(0.1)
    GPIO.output(GPIO_RST, GPIO.HIGH)
    time.sleep(0.1)
    print("   ✓ Reset completado")
    
    # Initialize SPI
    print("\n2. Inicializando SPI...")
    spi = spidev.SpiDev()
    spi.open(0, 0)  # Bus 0, Device 0
    spi.max_speed_hz = 500000
    spi.mode = 0
    print("   ✓ SPI inicializado (500 kHz)")
    
    # Read version register
    print("\n3. Leyendo registro de versión...")
    version = spi_read(REG_VERSION)
    print(f"   Versión: 0x{version:02X}")
    if version == 0x12:
        print("   ✓ Módulo SX1278 detectado correctamente!")
    else:
        print(f"   ✗ ADVERTENCIA: Versión inesperada (esperado 0x12)")
    
    # Set to sleep mode
    print("\n4. Configurando modo sleep...")
    spi_write(REG_OP_MODE, 0x80)
    time.sleep(0.01)
    mode = spi_read(REG_OP_MODE)
    print(f"   Modo: 0x{mode:02X} (esperado 0x80)")
    
    # Configure frequency 433 MHz
    print("\n5. Configurando frecuencia 433 MHz...")
    freq = int(433E6 / (32E6 / (2**19)))
    spi_write(REG_FRF_MSB, (freq >> 16) & 0xFF)
    spi_write(REG_FRF_MID, (freq >> 8) & 0xFF)
    spi_write(REG_FRF_LSB, freq & 0xFF)
    
    msb = spi_read(REG_FRF_MSB)
    mid = spi_read(REG_FRF_MID)
    lsb = spi_read(REG_FRF_LSB)
    print(f"   Frecuencia configurada: MSB=0x{msb:02X} MID=0x{mid:02X} LSB=0x{lsb:02X}")
    
    # Configure LoRa mode
    print("\n6. Configurando modo LoRa...")
    spi_write(REG_OP_MODE, 0x81)  # LoRa + Standby
    time.sleep(0.01)
    mode = spi_read(REG_OP_MODE)
    print(f"   Modo: 0x{mode:02X} (esperado 0x81)")
    
    # Configure modem
    print("\n7. Configurando parámetros LoRa...")
    spi_write(REG_MODEM_CONFIG_1, 0x72)  # BW=125kHz, CR=4/5, Implicit Header OFF
    spi_write(REG_MODEM_CONFIG_2, 0x74)  # SF=7, CRC ON
    spi_write(REG_SYNC_WORD, 0x21)
    
    cfg1 = spi_read(REG_MODEM_CONFIG_1)
    cfg2 = spi_read(REG_MODEM_CONFIG_2)
    sync = spi_read(REG_SYNC_WORD)
    print(f"   Modem Config 1: 0x{cfg1:02X} (esperado 0x72)")
    print(f"   Modem Config 2: 0x{cfg2:02X} (esperado 0x74)")
    print(f"   Sync Word: 0x{sync:02X} (esperado 0x21)")
    
    # Test transmission
    print("\n8. Probando transmisión...")
    spi_write(0x0D, 0x00)  # FIFO PTR
    spi_write(0x22, 4)     # Payload length
    spi_write(0x00, 0xA5)  # Write test data to FIFO
    spi_write(0x00, 0x10)
    spi_write(0x00, 0x01)
    spi_write(0x00, 0x01)
    
    spi_write(0x12, 0xFF)  # Clear IRQ
    spi_write(REG_OP_MODE, 0x83)  # TX mode
    print("   Transmitiendo paquete de prueba...")
    
    # Wait for TX done
    start = time.time()
    tx_done = False
    while time.time() - start < 2.0:
        irq = spi_read(0x12)
        if irq & 0x08:
            tx_done = True
            break
        time.sleep(0.01)
    
    if tx_done:
        print(f"   ✓ Transmisión completada! (IRQ=0x{irq:02X})")
    else:
        print(f"   ✗ Transmisión falló (timeout, IRQ=0x{irq:02X})")
    
    # Return to RX
    spi_write(0x12, 0xFF)
    spi_write(REG_OP_MODE, 0x85)
    
    print("\n" + "="*70)
    print("Diagnóstico completado")
    print("="*70)
    
    # Cleanup
    spi.close()
    GPIO.cleanup()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nInterrumpido por usuario")
        if spi:
            spi.close()
        GPIO.cleanup()
    except Exception as e:
        print(f"\n✗ Error: {e}")
        if spi:
            spi.close()
        GPIO.cleanup()
