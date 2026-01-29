#!/usr/bin/env python3
"""
================================================================================
                    Wall-E Coordinator - Raspberry Pi
              Request/Response LoRa Communication Handler
================================================================================

This program runs on the Raspberry Pi 4 and acts as the central coordinator
for the Wall-E monitoring system. It:

1. Periodically queries each of the 9 remote ESP32 devices
2. Receives status responses from the devices
3. Logs and displays the sensor data
4. Alerts on status changes or communication failures

PROTOCOL:
  Request: [NET_ID | MSG_REQ | TARGET_ID | REQ_CODE]
  Response: [NET_ID | MSG_RESP | DEVICE_ID | SEQ_LO | SEQ_HI | AC | UV1 | UV2]

HARDWARE:
  - LoRa module: SX1278 on 433MHz
  - Connected via SPI to GPIO pins (see constants below)
  - GPIO4=DIO0, GPIO17=DIO1, GPIO18=DIO2, GPIO27=DIO3, GPIO22=RST

CONFIGURATION:
  - NUM_DEVICES: Number of remote nodes (1-9)
  - QUERY_INTERVAL: Seconds between polling cycles
  - RESPONSE_TIMEOUT: Seconds to wait for response from each device
  - FREQUENCY: LoRa frequency (433E6 for 433 MHz)

================================================================================
"""

import time
import RPi.GPIO as GPIO
import spidev
import sys
from datetime import datetime

# ============================================================================
# HARDWARE CONFIGURATION
# ============================================================================

# GPIO Pin mappings (BCM numbering)
GPIO_DIO0 = 4
GPIO_DIO1 = 17
GPIO_DIO2 = 18
GPIO_DIO3 = 27
GPIO_RST = 22

# SPI Configuration
SPI_BUS = 0
SPI_DEVICE = 0
SPI_SPEED = 3900000  # 3.9 MHz

# ============================================================================
# PROTOCOL CONSTANTS
# ============================================================================

NET_ID = 0xA5          # Network identifier
MSG_REQ = 0x10         # Request message type
MSG_RESP = 0x90        # Response message type
REQ_READ_DATA = 0x01   # Read sensor data request code

# ============================================================================
# LORA REGISTERS (SX1276/SX1278)
# ============================================================================

REG_FIFO = 0x00
REG_FIFO_ADDR_PTR = 0x0D
REG_FIFO_RX_BASE_ADDR = 0x0F
REG_FIFO_RX_CURRENT_ADDR = 0x10
REG_IRQ_FLAGS = 0x12
REG_RX_NB_BYTES = 0x13
REG_OP_MODE = 0x01
REG_FREQ_MSB = 0x06
REG_FREQ_MID = 0x07
REG_FREQ_LSB = 0x08
REG_PA_CONFIG = 0x09
REG_MODEM_CONFIG_1 = 0x1D
REG_MODEM_CONFIG_2 = 0x1E
REG_SYNC_WORD = 0x39
REG_PREAMBLE_MSB = 0x20
REG_PREAMBLE_LSB = 0x21

# ============================================================================
# SYSTEM CONFIGURATION
# ============================================================================

NUM_DEVICES = 9                    # Number of remote nodes (1-9)
QUERY_INTERVAL = 5.0               # Seconds between query cycles
RESPONSE_TIMEOUT = 2.0             # Seconds to wait for each response
FREQUENCY = 433E6                  # LoRa frequency (Hz)

# ============================================================================
# GLOBAL STATE
# ============================================================================

spi = None
last_response = {}                 # Track last response from each device
device_stats = {}                  # Statistics for each device

# Status definitions
STATUS_OK = 0
STATUS_ALERT = 1
STATUS_NAMES = {0: "OK", 1: "ALERT"}

# ============================================================================
# SPI COMMUNICATION FUNCTIONS
# ============================================================================

def spi_read(reg):
    """Read from LoRa register via SPI"""
    resp = spi.xfer2([reg & 0x7F, 0x00])
    return resp[1]

def spi_write(reg, value):
    """Write to LoRa register via SPI"""
    spi.xfer2([reg | 0x80, value])

# ============================================================================
# LORA INITIALIZATION
# ============================================================================

def initialize_hardware():
    """Initialize GPIO and SPI for LoRa module"""
    global spi
    
    # Setup GPIO
    try:
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup([GPIO_DIO0, GPIO_DIO1, GPIO_DIO2, GPIO_DIO3], GPIO.IN)
        GPIO.setup(GPIO_RST, GPIO.OUT)
        
        # Reset LoRa module
        GPIO.output(GPIO_RST, GPIO.LOW)
        time.sleep(0.01)
        GPIO.output(GPIO_RST, GPIO.HIGH)
        time.sleep(0.1)
        print("✓ GPIO initialized and LoRa module reset")
    except Exception as e:
        print(f"✗ GPIO Error: {e}")
        return False

    # Setup SPI
    try:
        spi = spidev.SpiDev()
        spi.open(SPI_BUS, SPI_DEVICE)
        spi.max_speed_hz = SPI_SPEED
        print("✓ SPI initialized")
    except Exception as e:
        print(f"✗ SPI Error: {e}")
        return False
    
    return True

def configure_lora():
    """Configure LoRa module for receiver operation"""
    try:
        # Set frequency to 433 MHz
        freq = int(FREQUENCY)
        frf = int(freq / 61.03515625)
        
        spi_write(REG_FREQ_MSB, (frf >> 16) & 0xFF)
        spi_write(REG_FREQ_MID, (frf >> 8) & 0xFF)
        spi_write(REG_FREQ_LSB, frf & 0xFF)
        
        # Power configuration
        spi_write(REG_PA_CONFIG, 0xFF)
        
        # Modem config for 125 kHz bandwidth, SF7, CR4/5
        spi_write(REG_MODEM_CONFIG_1, 0x72)
        spi_write(REG_MODEM_CONFIG_2, 0x74)
        
        # Sync word (private network)
        spi_write(REG_SYNC_WORD, 0x34)
        
        # Preamble
        spi_write(REG_PREAMBLE_MSB, 0x00)
        spi_write(REG_PREAMBLE_LSB, 0x08)
        
        # Receive mode
        spi_write(REG_OP_MODE, 0x85)
        
        time.sleep(0.1)
        print("✓ LoRa configured:")
        print("  - Frequency: 433 MHz")
        print("  - Spreading Factor: 7")
        print("  - Bandwidth: 125 kHz")
        print("  - Coding Rate: 4/5")
        return True
    except Exception as e:
        print(f"✗ Configuration error: {e}")
        return False

# ============================================================================
# TRANSMISSION FUNCTIONS
# ============================================================================

def send_request(device_id):
    """Send a read request to specific device"""
    try:
        # Build 4-byte request packet
        request = bytearray([NET_ID, MSG_REQ, device_id, REQ_READ_DATA])
        
        # Send via SPI (write to FIFO)
        spi_write(REG_OP_MODE, 0x81)  # TX mode
        
        for byte in request:
            spi_write(REG_FIFO, byte)
        
        # Wait for transmission
        start_time = time.time()
        while (spi_read(REG_IRQ_FLAGS) & 0x08) == 0:  # Wait for TxDone
            if time.time() - start_time > 1.0:
                print(f"✗ Transmission timeout for device {device_id}")
                return False
            time.sleep(0.01)
        
        # Clear interrupt and return to RX mode
        spi_write(REG_IRQ_FLAGS, 0xFF)
        spi_write(REG_OP_MODE, 0x85)  # RX continuous mode
        
        return True
    except Exception as e:
        print(f"✗ Send error: {e}")
        return False

# ============================================================================
# RECEPTION FUNCTIONS
# ============================================================================

def wait_response(device_id, timeout=RESPONSE_TIMEOUT):
    """Wait for response from specific device"""
    start_time = time.time()
    
    while time.time() - start_time < timeout:
        try:
            irq_flags = spi_read(REG_IRQ_FLAGS)
            
            # Check for RxDone flag
            if irq_flags & 0x40:
                # Read packet
                nb_bytes = spi_read(REG_RX_NB_BYTES)
                
                if nb_bytes == 8:  # Expected response size
                    rx_base = spi_read(REG_FIFO_RX_BASE_ADDR)
                    spi_write(REG_FIFO_ADDR_PTR, rx_base)
                    
                    # Read packet bytes
                    packet = []
                    for i in range(nb_bytes):
                        packet.append(spi_read(REG_FIFO))
                    
                    # Clear interrupt
                    spi_write(REG_IRQ_FLAGS, 0xFF)
                    
                    return bytes(packet)
        except Exception as e:
            pass
        
        time.sleep(0.05)
    
    return None

# ============================================================================
# PACKET PARSING
# ============================================================================

def parse_response(packet, device_id):
    """Parse 8-byte response packet"""
    if len(packet) != 8:
        return None
    
    net_id = packet[0]
    msg_type = packet[1]
    resp_id = packet[2]
    seq_lo = packet[3]
    seq_hi = packet[4]
    ac_status = packet[5]
    uv1_status = packet[6]
    uv2_status = packet[7]
    
    # Validate response
    if net_id != NET_ID or msg_type != MSG_RESP or resp_id != device_id:
        return None
    
    seq = (seq_hi << 8) | seq_lo
    
    return {
        'device_id': resp_id,
        'seq': seq,
        'ac': ac_status,
        'uv1': uv1_status,
        'uv2': uv2_status,
        'timestamp': datetime.now()
    }

# ============================================================================
# QUERY CYCLE
# ============================================================================

def query_device(device_id):
    """Query single device and collect response"""
    print(f"\n  [Device {device_id}] ", end="", flush=True)
    
    if send_request(device_id):
        response_data = wait_response(device_id)
        
        if response_data:
            response = parse_response(response_data, device_id)
            if response:
                print(f"✓ Response: AC={STATUS_NAMES[response['ac']]} " +
                      f"UV1={STATUS_NAMES[response['uv1']]} " +
                      f"UV2={STATUS_NAMES[response['uv2']]} (Seq={response['seq']})")
                
                # Update statistics
                if device_id not in device_stats:
                    device_stats[device_id] = {
                        'responses': 0,
                        'failures': 0,
                        'last_status': None
                    }
                
                device_stats[device_id]['responses'] += 1
                device_stats[device_id]['last_status'] = response
                
                return response
            else:
                print("✗ Invalid response format")
        else:
            print("✗ No response (timeout)")
    else:
        print("✗ Send failed")
    
    # Update failure count
    if device_id not in device_stats:
        device_stats[device_id] = {'responses': 0, 'failures': 0, 'last_status': None}
    device_stats[device_id]['failures'] += 1
    
    return None

def query_all_devices():
    """Query all devices in sequence"""
    print(f"\n{'='*70}")
    print(f"Query Cycle: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print(f"{'='*70}")
    
    responses = {}
    for device_id in range(1, NUM_DEVICES + 1):
        response = query_device(device_id)
        if response:
            responses[device_id] = response
        
        time.sleep(0.5)  # Delay between requests
    
    return responses

# ============================================================================
# MAIN PROGRAM
# ============================================================================

def main():
    """Main coordinator loop"""
    print("="*70)
    print("   Wall-E Coordinator - Raspberry Pi LoRa Receiver")
    print("="*70)
    print(f"Configuration:")
    print(f"  - Number of devices: {NUM_DEVICES}")
    print(f"  - Query interval: {QUERY_INTERVAL} seconds")
    print(f"  - Response timeout: {RESPONSE_TIMEOUT} seconds")
    print(f"  - Frequency: {FREQUENCY/1e6} MHz")
    print("="*70)
    
    # Initialize hardware
    if not initialize_hardware():
        print("✗ Hardware initialization failed")
        return
    
    # Configure LoRa
    if not configure_lora():
        print("✗ LoRa configuration failed")
        return
    
    print("\n✓ Coordinator ready!")
    print("Press Ctrl+C to exit\n")
    
    # Main loop
    try:
        cycle_count = 0
        while True:
            cycle_count += 1
            
            # Query all devices
            responses = query_all_devices()
            
            # Print summary
            print(f"\nCycle Summary:")
            print(f"  - Total responses: {len(responses)}/{NUM_DEVICES}")
            print(f"  - Alerts: {sum(1 for r in responses.values() if r['ac'] or r['uv1'] or r['uv2'])}")
            
            # Print device statistics
            print(f"\nDevice Statistics:")
            for device_id in range(1, NUM_DEVICES + 1):
                if device_id in device_stats:
                    stats = device_stats[device_id]
                    success_rate = (stats['responses'] * 100) / (stats['responses'] + stats['failures'])
                    print(f"  Device {device_id}: {stats['responses']} responses, " +
                          f"{stats['failures']} failures ({success_rate:.1f}% success)")
            
            # Wait for next cycle
            print(f"\nWaiting {QUERY_INTERVAL} seconds for next cycle...", end="", flush=True)
            time.sleep(QUERY_INTERVAL)
            print(" done")
    
    except KeyboardInterrupt:
        print("\n\n✓ Exiting coordinator...")
    finally:
        # Cleanup
        try:
            spi.close()
            GPIO.cleanup()
            print("✓ Resources cleaned up")
        except:
            pass

if __name__ == "__main__":
    main()
