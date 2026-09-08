#!/usr/bin/env python3
"""
================================================================================
                    Wall-E Coordinator - Raspberry Pi
              Request/Response LoRa Communication Handler
              
              Developed by: Andani Emmanuel Lopez Arechar
================================================================================

This program runs on the Raspberry Pi 4 and acts as the central coordinator
for the Wall-E monitoring system. It:

1. Periodically queries each configured remote ESP32 device
2. Receives status responses from the devices
3. Logs and displays the sensor data
4. Alerts on status changes or communication failures

PROTOCOL:
  Request: [NET_ID | MSG_REQ | TARGET_ID | REQ_CODE]
  Response: [NET_ID | MSG_RESP | DEVICE_ID | SEQ_LO | SEQ_HI | AC_V_SCALED | CURR1_mA | CURR2_mA | CURR3_mA | CURR4_mA]
           where scaled values: 0-255 = voltage 0-130V RMS, current 0-2550 mA per channel

HARDWARE:
  - LoRa module: SX1278 on 433MHz
  - Connected via SPI to GPIO pins (see constants below)
  - GPIO4=DIO0, GPIO17=DIO1, GPIO18=DIO2, GPIO27=DIO3, GPIO22=RST

CONFIGURATION:
    - NUM_DEVICES: Number of remote nodes (1-255)
  - QUERY_INTERVAL: Seconds between polling cycles
  - RESPONSE_TIMEOUT: Seconds to wait for response from each device
  - FREQUENCY: LoRa frequency (433E6 for 433 MHz)

================================================================================
"""

import time
import sys
import os
import subprocess
import json
from datetime import datetime
import threading
import tkinter as tk
from tkinter import messagebox
from PIL import Image, ImageTk
import random

# Try to import RPi-specific modules; gracefully skip in demo mode
try:
    import RPi.GPIO as GPIO
except ImportError:
    GPIO = None

try:
    import spidev
except ImportError:
    spidev = None

# ============================================================================
# HARDWARE CONFIGURATION
# ============================================================================

# GPIO Pin mappings (BCM numbering)
GPIO_DIO0 = 4
GPIO_DIO1 = 17
GPIO_DIO2 = 18
GPIO_DIO3 = 27
GPIO_RST = 22

# Tower (stack light) GPIO Pins (BCM numbering)
# OUTPUT,GPIO,PIN,RELAY_INPUT
# Green,0,27,1
# Yellow,5,29,2
# Red,6,31,3
# Buzzer,13,33,4
PIN_GREEN_TURRET = 0    # PIN GREEN_COLOR TORRET
PIN_YELLOW_TURRET = 5   # PIN YELLOW_COLOR TORRET
PIN_RED_TURRET = 6      # PIN RED_COLOR TORRET
PIN_BUZZER = 13         # PIN BUZZER TORRET
RELAY_ACTIVE_LOW = True  # Set True if relay outputs are active-low

def set_relay(pin, on):
    """Helper to drive relay outputs with optional active-low logic."""
    if GPIO is None or DEMO_MODE:
        return  # Skip GPIO operations in demo mode or when GPIO unavailable
    try:
        if RELAY_ACTIVE_LOW:
            GPIO.output(pin, GPIO.LOW if on else GPIO.HIGH)
        else:
            GPIO.output(pin, GPIO.HIGH if on else GPIO.LOW)
    except:
        pass

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
REG_MODEM_CONFIG_3 = 0x26
REG_SYNC_WORD = 0x39
REG_PREAMBLE_MSB = 0x20
REG_PREAMBLE_LSB = 0x21
REG_PAYLOAD_LENGTH = 0x22

# ============================================================================
# SYSTEM CONFIGURATION
# ============================================================================

NUM_DEVICES = 50                   # Number of remote nodes to poll
MIN_DEVICE_ID = 1                  # Protocol minimum device ID
MAX_DEVICE_ID = 255                # 1 byte in packet supports IDs up to 255
HMI_GRID_COLUMNS = 4               # Grid columns for node cards in HMI
NODES_PER_PAGE = 20                # 4 cols x 5 rows per page
QUERY_INTERVAL = 5.0               # Seconds between query cycles
RESPONSE_TIMEOUT = 4.0             # Seconds to wait for each response
FREQUENCY = 433E6                  # LoRa frequency (Hz)

# HMI THRESHOLDS
VOLTAGE_MIN = 100.0                # Minimum acceptable voltage (V)
VOLTAGE_MAX = 140.0                # Maximum acceptable voltage (V)
VOLTAGE_SCALE_MAX = 130.0          # Telemetry scaling max (must match ESP32 packet scaling)
CURRENT_MIN = 150.0                # Minimum acceptable current per lamp (mA) = 0.15A
CURRENT_MAX = 5000.0               # Maximum acceptable current per lamp (mA) = 5.0A
CURRENT_RED_THRESHOLD = 150.0      # <150mA = red fault

# ALARM TOLERANCE CONFIGURATION
MAX_CONSECUTIVE_FAILURES = 3       # Number of consecutive failures before triggering alarm
                                   # This prevents false alarms during LoRa module recovery
                                   # With QUERY_INTERVAL=5s, 3 failures = 15 seconds tolerance

# DEMO MODE (for testing without real hardware)
DEMO_MODE = False                  # Set to False for real hardware testing with ESP32
DEMO_UPDATE_INTERVAL = 2.0         # Seconds between demo data updates

# Persistent node configuration
CONFIG_FILE = "walle_system_config.json"
NODE_MODEL_OPTIONS = ["Model A", "Model B", "Model C"]

# ============================================================================
# GLOBAL STATE
# ============================================================================

spi = None
last_response = {}                 # Track last response from each device
device_stats = {}                  # Statistics for each device
device_consecutive_failures = {}   # Track consecutive failures per device for alarm tolerance
device_consecutive_sensor_errors = {}  # Track consecutive sensor-error reads per device
system_config = {}
config_lock = threading.Lock()

# Shared data structure for HMI (thread-safe)
device_data_lock = threading.Lock()
shared_device_data = {}            # Device status shared between coordinator thread and HMI
current_scanning_device = 0        # Currently scanning device ID (for HMI display)
coordinator_running = False        # Flag to control coordinator thread
coordinator_thread = None          # Reference to coordinator thread
restart_requested = False          # Request flag to relaunch app after clean shutdown


def create_default_node_entry(device_id):
    return {
        'device_id': device_id,
        'model': NODE_MODEL_OPTIONS[0],
        'maintenance': False,
    }


def build_default_system_config(node_count=NUM_DEVICES, setup_completed=False):
    try:
        normalized_count = int(node_count)
    except (TypeError, ValueError):
        normalized_count = NUM_DEVICES
    normalized_count = max(MIN_DEVICE_ID, min(MAX_DEVICE_ID, normalized_count))
    return {
        'setup_completed': setup_completed,
        'node_count': normalized_count,
        'nodes': [create_default_node_entry(device_id) for device_id in range(1, MAX_DEVICE_ID + 1)]
    }


def normalize_system_config(raw_config):
    default_config = build_default_system_config()
    if not isinstance(raw_config, dict):
        return default_config

    try:
        node_count = int(raw_config.get('node_count', NUM_DEVICES))
    except (TypeError, ValueError):
        node_count = NUM_DEVICES
    node_count = max(MIN_DEVICE_ID, min(MAX_DEVICE_ID, node_count))

    raw_nodes = raw_config.get('nodes', [])
    normalized_nodes = []
    for device_id in range(1, MAX_DEVICE_ID + 1):
        raw_node = raw_nodes[device_id - 1] if isinstance(raw_nodes, list) and len(raw_nodes) >= device_id else {}
        if not isinstance(raw_node, dict):
            raw_node = {}
        model = raw_node.get('model', NODE_MODEL_OPTIONS[0])
        if model not in NODE_MODEL_OPTIONS:
            model = NODE_MODEL_OPTIONS[0]
        normalized_nodes.append({
            'device_id': device_id,
            'model': model,
            'maintenance': bool(raw_node.get('maintenance', False)),
        })

    return {
        'setup_completed': bool(raw_config.get('setup_completed', False)),
        'node_count': node_count,
        'nodes': normalized_nodes,
    }


def save_system_config(config=None):
    global system_config

    config_to_save = normalize_system_config(system_config if config is None else config)
    with config_lock:
        system_config = config_to_save
        with open(CONFIG_FILE, 'w', encoding='utf-8') as config_file:
            json.dump(system_config, config_file, indent=2)
    return config_to_save


def load_system_config(force_reset=False):
    global system_config

    if force_reset:
        return save_system_config(build_default_system_config(setup_completed=False))

    try:
        with open(CONFIG_FILE, 'r', encoding='utf-8') as config_file:
            loaded_config = json.load(config_file)
        normalized_config = normalize_system_config(loaded_config)
    except (FileNotFoundError, json.JSONDecodeError, OSError):
        normalized_config = build_default_system_config(setup_completed=False)

    return save_system_config(normalized_config)


def get_system_config():
    with config_lock:
        cached_config = normalize_system_config(system_config) if system_config else None
    if cached_config is not None:
        return cached_config
    return load_system_config()


def get_configured_node_count():
    return get_system_config().get('node_count', NUM_DEVICES)


def get_node_config(device_id):
    config = get_system_config()
    if 1 <= device_id <= len(config['nodes']):
        return dict(config['nodes'][device_id - 1])
    return create_default_node_entry(device_id)


def get_node_model(device_id):
    return get_node_config(device_id).get('model', NODE_MODEL_OPTIONS[0])


def is_node_in_maintenance(device_id):
    return bool(get_node_config(device_id).get('maintenance', False))


def get_active_device_ids():
    config = get_system_config()
    active_ids = []
    for device_id in range(1, config['node_count'] + 1):
        if not config['nodes'][device_id - 1].get('maintenance', False):
            active_ids.append(device_id)
    return active_ids


def clear_runtime_state_for_node(device_id):
    global current_scanning_device

    with device_data_lock:
        shared_device_data.pop(device_id, None)
        if current_scanning_device == device_id:
            current_scanning_device = 0
    device_consecutive_failures.pop(device_id, None)
    device_consecutive_sensor_errors.pop(device_id, None)
    device_stats.pop(device_id, None)


def synchronize_runtime_state():
    configured_count = get_configured_node_count()
    active_ids = set(get_active_device_ids())

    for device_id in range(1, MAX_DEVICE_ID + 1):
        if device_id > configured_count or device_id not in active_ids:
            clear_runtime_state_for_node(device_id)

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
    
    if DEMO_MODE:
        print("✓ Demo mode: skipping hardware initialization")
        return True
    
    # Setup GPIO
    try:
        if GPIO is None:
            print("✗ RPi.GPIO not available")
            return False
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
        if spidev is None:
            print("✗ spidev not available")
            return False
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
    if DEMO_MODE:
        print("\u2713 Demo mode: skipping LoRa configuration")
        return True
    try:
        # 1. Set to sleep mode first
        spi_write(REG_OP_MODE, 0x80)  # Sleep mode
        time.sleep(0.01)
        
        # 2. Set to LoRa mode (not FSK)
        spi_write(REG_OP_MODE, 0x80)  # Sleep mode + LoRa mode
        time.sleep(0.01)
        
        # 3. Set frequency to 433 MHz
        freq = int(FREQUENCY)
        frf = int(freq / 61.03515625)
        
        spi_write(REG_FREQ_MSB, (frf >> 16) & 0xFF)
        spi_write(REG_FREQ_MID, (frf >> 8) & 0xFF)
        spi_write(REG_FREQ_LSB, frf & 0xFF)
        
        # 4. Power configuration (max power)
        spi_write(REG_PA_CONFIG, 0xFF)
        
        # 5. Set FIFO base addresses
        spi_write(REG_FIFO_RX_BASE_ADDR, 0x00)
        spi_write(0x0E, 0x00)  # REG_FIFO_TX_BASE_ADDR
        
        # 6. Robust modem config: BW=125kHz, CR=4/8, explicit header
        # REG_MODEM_CONFIG_1: 0x7E
        #   BW=125kHz(0x70) | CR=4/8(0x0E) | explicit header(0x00)
        spi_write(REG_MODEM_CONFIG_1, 0x7E)

        # 7. SF10 + CRC ON
        # REG_MODEM_CONFIG_2: 0xA4
        #   SF10(0xA0) | TxContinuous=0(0x00) | CRC ON(0x04)
        spi_write(REG_MODEM_CONFIG_2, 0xA4)

        # 8. AGC ON (improves robustness in variable signal environments)
        # REG_MODEM_CONFIG_3: 0x04
        spi_write(REG_MODEM_CONFIG_3, 0x04)
        
        # 9. Sync word (private network) - DEBE COINCIDIR CON ESP32
        spi_write(REG_SYNC_WORD, 0x21)
        
        # 10. Preamble length (12 symbols for better detection)
        spi_write(REG_PREAMBLE_MSB, 0x00)
        spi_write(REG_PREAMBLE_LSB, 0x0C)
        
        # 11. Set to standby mode
        spi_write(REG_OP_MODE, 0x81)  # Standby mode
        time.sleep(0.01)
        
        # 12. Set to receive continuous mode
        spi_write(REG_OP_MODE, 0x85)  # RX continuous
        time.sleep(0.1)
        
        # Verify configuration
        print("✓ LoRa configured:")
        print("  - Frequency: 433 MHz")
        print("  - Spreading Factor: 10")
        print("  - Bandwidth: 125 kHz")
        print("  - Coding Rate: 4/8")
        print("  - Preamble Length: 12")
        print("  - AGC: ON")
        print("  - Sync Word: 0x21")
        print(f"  - Mode register: 0x{spi_read(REG_OP_MODE):02X}")
        print(f"  - Modem Config 1: 0x{spi_read(REG_MODEM_CONFIG_1):02X}")
        print(f"  - Modem Config 2: 0x{spi_read(REG_MODEM_CONFIG_2):02X}")
        print(f"  - Modem Config 3: 0x{spi_read(REG_MODEM_CONFIG_3):02X}")
        print(f"  - Sync Word read: 0x{spi_read(REG_SYNC_WORD):02X}")
        return True
    except Exception as e:
        print(f"✗ Configuration error: {e}")
        return False

# ============================================================================
# TRANSMISSION FUNCTIONS
# ============================================================================

def send_request(device_id):
    """Send a read request to specific device"""
    if not (MIN_DEVICE_ID <= device_id <= MAX_DEVICE_ID):
        print(f"[TX ERROR] Device ID out of range: {device_id}")
        return False

    try:
        # Build 4-byte request packet
        request = bytearray([NET_ID, MSG_REQ, device_id, REQ_READ_DATA])
        
        # 1. Set to standby mode
        spi_write(REG_OP_MODE, 0x81)  # Standby mode
        time.sleep(0.01)
        
        # 2. Set FIFO address to start
        spi_write(REG_FIFO_ADDR_PTR, 0x00)
        
        # 3. Write payload length
        spi_write(0x22, len(request))  # REG_PAYLOAD_LENGTH
        
        # 4. Write data to FIFO
        for byte in request:
            spi_write(REG_FIFO, byte)
        
        # 5. Clear IRQ flags
        spi_write(REG_IRQ_FLAGS, 0xFF)
        time.sleep(0.01)
        
        # 6. Set to TX mode
        spi_write(REG_OP_MODE, 0x83)  # TX mode
        time.sleep(0.01)
        
        # 7. Wait for transmission complete
        start_time = time.time()
        tx_complete = False
        while time.time() - start_time < 1.0:
            irq_flags = spi_read(REG_IRQ_FLAGS)
            if irq_flags & 0x08:  # TxDone flag
                tx_complete = True
                break
            time.sleep(0.01)
        
        # 8. Clear interrupt and return to RX mode (ALWAYS, regardless of success)
        spi_write(REG_IRQ_FLAGS, 0xFF)
        time.sleep(0.01)
        spi_write(REG_OP_MODE, 0x85)  # RX continuous mode
        time.sleep(0.01)
        
        if not tx_complete:
            print(f"[TX ERROR] Device {device_id}: TX timeout! IRQ=0x{irq_flags:02X}")
            return False
        
        return True
    except Exception as e:
        # Even on exception, try to return to RX mode
        try:
            spi_write(REG_OP_MODE, 0x85)
        except:
            pass
        print(f"✗ Send error: {e}")
        return False

# ============================================================================
# RECEPTION FUNCTIONS
# ============================================================================

def unscale_voltage(scaled_value):
    """Convert 8-bit scaled value back to voltage (0-255 = 0-130V RMS)."""
    return (scaled_value / 255.0) * VOLTAGE_SCALE_MAX

def unscale_current(scaled_value):
    """Convert 8-bit scaled value back to current in mA (0-255 = 0-2550 mA)"""
    return (scaled_value / 255.0) * 2550.0

def wait_response(device_id, timeout=RESPONSE_TIMEOUT):
    """Wait for response from specific device"""
    start_time = time.time()
    
    while time.time() - start_time < timeout:
        try:
            irq_flags = spi_read(REG_IRQ_FLAGS)
            
            # CRC error flag - silently clear and retry
            if irq_flags & 0x20:
                spi_write(REG_IRQ_FLAGS, 0xFF)
                continue

            # Check for RxDone flag
            if irq_flags & 0x40:
                # Read packet
                nb_bytes = spi_read(REG_RX_NB_BYTES)

                if nb_bytes == 10:  # Expected response size (now 10 bytes with 4 current sensors)
                    rx_addr = spi_read(REG_FIFO_RX_CURRENT_ADDR)
                    spi_write(REG_FIFO_ADDR_PTR, rx_addr)

                    # Read packet bytes
                    packet = []
                    for i in range(nb_bytes):
                        packet.append(spi_read(REG_FIFO))

                    # Clear interrupt
                    spi_write(REG_IRQ_FLAGS, 0xFF)
                    time.sleep(0.01)
                    
                    # Verify we're back in RX mode
                    mode = spi_read(REG_OP_MODE)
                    if (mode & 0x07) != 0x05:  # Not in RX continuous
                        spi_write(REG_OP_MODE, 0x85)  # Force RX continuous
                        time.sleep(0.01)

                    return bytes(packet)
                else:
                    # Unexpected packet size - clear IRQ and continue listening
                    spi_write(REG_IRQ_FLAGS, 0xFF)
                    time.sleep(0.01)
                    spi_write(REG_OP_MODE, 0x85)  # Ensure RX continuous
        except Exception as e:
            pass

        time.sleep(0.02)

    return None

# ============================================================================
# PACKET PARSING
# ============================================================================

def parse_response(packet, device_id):
    """Parse 10-byte response packet with scaled sensor values"""
    if len(packet) != 10:
        return None
    
    net_id = packet[0]
    msg_type = packet[1]
    resp_id = packet[2]
    seq_lo = packet[3]
    seq_hi = packet[4]
    ac_v_scaled = packet[5]
    curr1_scaled = packet[6]
    curr2_scaled = packet[7]
    curr3_scaled = packet[8]
    curr4_scaled = packet[9]
    
    # Validate response
    if not (MIN_DEVICE_ID <= resp_id <= MAX_DEVICE_ID):
        return None

    if net_id != NET_ID or msg_type != MSG_RESP or resp_id != device_id:
        return None
    
    seq = (seq_hi << 8) | seq_lo
    
    # Unscale values to actual measurements
    ac_voltage_V = unscale_voltage(ac_v_scaled)
    curr1_mA = unscale_current(curr1_scaled)
    curr2_mA = unscale_current(curr2_scaled)
    curr3_mA = unscale_current(curr3_scaled)
    curr4_mA = unscale_current(curr4_scaled)
    
    return {
        'device_id': resp_id,
        'seq': seq,
        'ac_voltage_V': ac_voltage_V,
        'curr1_mA': curr1_mA,
        'curr2_mA': curr2_mA,
        'curr3_mA': curr3_mA,
        'curr4_mA': curr4_mA,
        'timestamp': datetime.now()
    }


def has_sensor_error(response):
    """Return True when a node reading is in error according to direct thresholds."""
    voltage = response.get('ac_voltage_V', 0.0)
    curr1 = response.get('curr1_mA', 0.0)
    curr3 = response.get('curr3_mA', 0.0)

    voltage_error = voltage < VOLTAGE_MIN
    current_error = (
        curr1 < CURRENT_RED_THRESHOLD or curr1 > CURRENT_MAX or
        curr3 < CURRENT_RED_THRESHOLD or curr3 > CURRENT_MAX
    )
    return voltage_error or current_error

# ============================================================================
# QUERY CYCLE
# ============================================================================

def query_device(device_id):
    """Query single device and collect response"""
    global current_scanning_device, device_data_lock, device_consecutive_failures, device_consecutive_sensor_errors

    if is_node_in_maintenance(device_id):
        clear_runtime_state_for_node(device_id)
        print(f"\n  [Device {device_id}] MAINTENANCE MODE")
        return None
    
    # Update shared variable so HMI knows which device we're scanning
    with device_data_lock:
        current_scanning_device = device_id
    
    print(f"\n  [Device {device_id}] ", end="", flush=True)
    max_retries = 9  # 10 intentos en total
    short_timeout = 0.8
    for attempt in range(1, max_retries + 2):
        if send_request(device_id):
            response_data = wait_response(device_id, timeout=short_timeout)
            if response_data:
                response = parse_response(response_data, device_id)
                if response:
                    print(f"✓ AC={response['ac_voltage_V']:.1f}V " +
                          f"I1={response['curr1_mA']:.0f}mA " +
                          f"I2={response['curr2_mA']:.0f}mA " +
                          f"I3={response['curr3_mA']:.0f}mA " +
                          f"I4={response['curr4_mA']:.0f}mA (Seq={response['seq']})")
                    # Update statistics
                    if device_id not in device_stats:
                        device_stats[device_id] = {
                            'responses': 0,
                            'failures': 0,
                            'last_status': None
                        }
                    device_stats[device_id]['responses'] += 1
                    device_stats[device_id]['last_status'] = response
                    
                    # Reset consecutive failure counter on successful response
                    device_consecutive_failures[device_id] = 0

                    # Track consecutive sensor errors (used for delayed tower alarm)
                    if has_sensor_error(response):
                        device_consecutive_sensor_errors[device_id] = device_consecutive_sensor_errors.get(device_id, 0) + 1
                    else:
                        device_consecutive_sensor_errors[device_id] = 0
                    
                    return response
                else:
                    pass
            else:
                pass
        else:
            pass
        time.sleep(0.2)
    
    # Update failure count
    if device_id not in device_stats:
        device_stats[device_id] = {'responses': 0, 'failures': 0, 'last_status': None}
    device_stats[device_id]['failures'] += 1
    
    # Increment consecutive failure counter
    if device_id not in device_consecutive_failures:
        device_consecutive_failures[device_id] = 0
    device_consecutive_failures[device_id] += 1
    
    # Display failure message with tolerance info
    consecutive_count = device_consecutive_failures[device_id]
    if consecutive_count >= MAX_CONSECUTIVE_FAILURES:
        print(f"✗ No response ({consecutive_count} consecutive failures - ALARM ACTIVE)")
    else:
        print(f"✗ No response ({consecutive_count}/{MAX_CONSECUTIVE_FAILURES} until alarm)")
    
    return None

def query_all_devices():
    """Query all devices in sequence"""
    print(f"\n{'='*70}")
    print(f"Query Cycle: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print(f"{'='*70}")
    
    # FULL RESET of LoRa module before each cycle
    try:
        print("[RESET] Performing full LoRa module reset...")
        
        # 1. Go to sleep mode first
        spi_write(REG_OP_MODE, 0x80)
        time.sleep(0.05)
        
        # 2. Clear all IRQ flags
        spi_write(REG_IRQ_FLAGS, 0xFF)
        time.sleep(0.01)
        
        # 3. Reset FIFO pointers
        spi_write(REG_FIFO_ADDR_PTR, 0x00)
        spi_write(REG_FIFO_RX_BASE_ADDR, 0x00)
        time.sleep(0.01)
        
        # 4. Go back to RX continuous mode
        spi_write(REG_OP_MODE, 0x85)
        time.sleep(0.05)
        
        # Verify state
        mode = spi_read(REG_OP_MODE)
        irq = spi_read(REG_IRQ_FLAGS)
        print(f"[DIAGNOSTIC] LoRa reset complete: MODE=0x{mode:02X}, IRQ=0x{irq:02X}")
        
    except Exception as e:
        print(f"[WARNING] Error during LoRa reset: {e}")
    
    responses = {}
    synchronize_runtime_state()
    for device_id in get_active_device_ids():
        response = query_device(device_id)
        if response:
            responses[device_id] = response
        time.sleep(0.5)  # Delay between requests
    
    return responses

# ============================================================================
# DEMO MODE - Generate simulated device data for testing
# ============================================================================

def generate_demo_data(device_id):
    """Generate simulated sensor data for testing without real hardware"""
    # Simulate realistic voltage variations (100-135V for real operation)
    voltage = random.uniform(100.0, 135.0)
    
    # 70% chance of normal operation, 30% chance of fault
    if random.random() < 0.7:
        # Normal operation - all currents present
        curr1 = random.uniform(300, 500)
        curr2 = random.uniform(300, 500)
        curr3 = random.uniform(300, 500)
        curr4 = random.uniform(300, 500)
    else:
        # Fault state - some lamps off
        if random.random() < 0.5:
            voltage = random.uniform(8.0, 10.5)  # Low voltage fault
        curr1 = random.uniform(10, 100) if random.random() < 0.5 else random.uniform(300, 500)
        curr2 = random.uniform(10, 100) if random.random() < 0.5 else random.uniform(300, 500)
        curr3 = random.uniform(10, 100) if random.random() < 0.5 else random.uniform(300, 500)
        curr4 = random.uniform(10, 100) if random.random() < 0.5 else random.uniform(300, 500)
    
    return {
        'device_id': device_id,
        'seq': random.randint(0, 65535),
        'ac_voltage_V': voltage,
        'curr1_mA': curr1,
        'curr2_mA': curr2,
        'curr3_mA': curr3,
        'curr4_mA': curr4,
        'timestamp': datetime.now()
    }

def demo_loop():
    """Demo mode coordinator loop - simulates device responses"""
    global coordinator_running, shared_device_data, device_data_lock
    
    print("\n✓ DEMO MODE - Coordinator thread started (simulated data)")
    
    cycle_count = 0
    while coordinator_running:
        try:
            cycle_count += 1
            print(f"\n{'='*70}")
            print(f"Demo Cycle #{cycle_count}: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
            print(f"{'='*70}")
            
            synchronize_runtime_state()

            # Generate and update simulated data for all active devices
            for device_id in get_active_device_ids():
                response = generate_demo_data(device_id)
                
                # Update shared data structure (thread-safe)
                with device_data_lock:
                    shared_device_data[device_id] = response
                
                # Print simulated response
                print(f"  [Device {device_id}] ✓ AC={response['ac_voltage_V']:.1f}V " +
                      f"I1={response['curr1_mA']:.0f}mA " +
                      f"I2={response['curr2_mA']:.0f}mA " +
                      f"I3={response['curr3_mA']:.0f}mA " +
                      f"I4={response['curr4_mA']:.0f}mA (Seq={response['seq']})")
                
                time.sleep(0.3)  # Delay between simulated requests
            
            # Print summary
            print(f"\nDemo Summary:")
            print(f"  - Total active devices simulated: {len(get_active_device_ids())}")
            
            # Wait for next cycle
            print(f"\nWaiting {DEMO_UPDATE_INTERVAL} seconds for next cycle...")
            time.sleep(DEMO_UPDATE_INTERVAL)
    
        except Exception as e:
            print(f"✗ Demo loop error: {e}")
            time.sleep(1)
    
    print("\n✓ Demo coordinator thread stopped")

# ============================================================================
# HMI (HUMAN-MACHINE INTERFACE)
# ============================================================================

class AppIndustrial:
    def __init__(self, root):
        self.root = root
        self.root.title("WALL-E MONITOR")
        self.root.geometry("480x800")  # 7-inch touch screen (portrait) with taskbar
        self.root.configure(bg="#483698")
        
        self.sonido_habil = True  
        self.falla_activa = False
        self.torreta_red_active = False
        self.torreta_yellow_active = False
        self.torreta_green_active = False
        self.last_tower_state = None  # Track last tower state to reduce updates
        self.last_buzzer_time = 0.0
        self.buzzer_active = False
        self.buzzer_off_time = 0.0
        self.test_mode = False
        self.leds_v = []
        self.lbls_v_val = []
        self.frames_robot = []
        self.uv_lamps = []
        self.model_labels = []
        self.mode_labels = []

        # --- HEADER (Logos, Title, Clock) ---
        self.header = tk.Frame(self.root, bg="#483698")
        self.header.pack(fill="x", padx=15, pady=10)

        # Left frame for Bimbo logo
        left_frame = tk.Frame(self.header, bg="#483698")
        left_frame.pack(side="left")
        
        try:
            img_b = Image.open("WALL-E HMI images/Grupo_Bimbo.png").convert("RGBA")
            self.photo = ImageTk.PhotoImage(img_b.resize((70, 35), Image.LANCZOS))
            tk.Label(left_frame, image=self.photo, bg="#483698").pack()
        except:
            tk.Label(left_frame, text="BIMBO", fg="white", bg="#483698", font=("Arial", 8, "bold")).pack()

        # Center frame for title and clock
        center_frame = tk.Frame(self.header, bg="#483698")
        center_frame.pack(side="left", expand=True, fill="both")
        
        tk.Label(center_frame, text="UV LAMP MONITORING", font=("Arial", 10, "bold"), fg="white", bg="#483698").pack()
        
        self.lbl_reloj = tk.Label(center_frame, text="", font=("Courier", 11, "bold"), fg="#00ff00", bg="#483698")
        self.lbl_reloj.pack()
        self.actualizar_hora()

        # Right frame for Moldex logo
        right_frame = tk.Frame(self.header, bg="#483698")
        right_frame.pack(side="right")
        
        try:
            img_m = Image.open("WALL-E HMI images/Moldex1.png").convert("RGBA")
            self.photo2 = ImageTk.PhotoImage(img_m.resize((70, 35), Image.LANCZOS))
            tk.Label(right_frame, image=self.photo2, bg="#483698").pack()
        except:
            tk.Label(right_frame, text="MOLDEX", fg="white", bg="#483698", font=("Arial", 8, "bold")).pack()

        # --- SCANNING STATUS ---
        self.status_frame = tk.Frame(self.root, bg="#2a1a5a")
        self.status_frame.pack(fill="x", padx=10, pady=2)
        self.lbl_scanning = tk.Label(self.status_frame, text="Scanning: W-0", font=("Arial", 8, "bold"), 
                                     fg="#00ff00", bg="#2a1a5a")
        self.lbl_scanning.pack()

        self.ensure_configuration_ready()
        if not self.root.winfo_exists():
            self.initialization_aborted = True
            return

        self.initialization_aborted = False

        # --- PAGINATION ---
        self.current_page = 0
        self.total_pages = 1
        
        self.page_frame = tk.Frame(self.root, bg="#483698")
        self.page_frame.pack(fill="x", padx=5, pady=2)
        
        tk.Button(self.page_frame, text="◀ PREV", font=("Arial", 8, "bold"), bg="#ffc72c", fg="black",
                 command=self.pagina_anterior, width=12, height=2).pack(side="left", padx=3, pady=3)
        
        self.lbl_page = tk.Label(self.page_frame, text=f"Page 1 of {self.total_pages}", font=("Arial", 7, "bold"),
                                bg="#483698", fg="#ffc72c")
        self.lbl_page.pack(side="left", expand=True, padx=5)
        
        tk.Button(self.page_frame, text="NEXT ▶", font=("Arial", 8, "bold"), bg="#ffc72c", fg="black",
                 command=self.pagina_siguiente, width=12, height=2).pack(side="right", padx=3, pady=3)

        # --- PANEL DE ROBOTS ---
        self.container = tk.Frame(self.root, bg="#483698")
        self.container.pack(expand=True, fill="both", padx=2, pady=1)

        # Configurar columnas iguales
        for j in range(HMI_GRID_COLUMNS):
            self.container.grid_columnconfigure(j, weight=1)

        self.rebuild_node_grid()

        # --- BOTONERA INFERIOR ---
        self.f_btn = tk.Frame(self.root, bg="#483698")
        self.f_btn.pack(side="bottom", fill="x", pady=1)
        
        b_style = {"font": ("Arial", 7, "bold"), "bg": "#ffc72c", "height": 1, "activebackground": "#e6b422"}
        
        tk.Button(self.f_btn, text="MUTE", command=self.silenciar, **b_style).grid(row=0, column=0, sticky="we", padx=2)
        tk.Button(self.f_btn, text="SOUND ON", command=self.reset, **b_style).grid(row=0, column=1, sticky="we", padx=2)
        tk.Button(self.f_btn, text="LOGS", command=self.abrir_historial, **b_style).grid(row=0, column=2, sticky="we", padx=2)
        tk.Button(self.f_btn, text="MAP", command=self.mostrar_imagen_layout, **b_style).grid(row=0, column=3, sticky="we", padx=2)
        tk.Button(self.f_btn, text="SETTINGS", command=self.open_settings_dialog, **b_style).grid(row=0, column=4, sticky="we", padx=2)
        # tk.Button(self.f_btn, text="TEST MODE ON", command=self.test_mode_on, **b_style).grid(row=1, column=0, columnspan=2, sticky="we", padx=2, pady=2)
        # tk.Button(self.f_btn, text="TEST MODE OFF", command=self.test_mode_off, **b_style).grid(row=1, column=2, columnspan=2, sticky="we", padx=2, pady=2)
        
        # Admin buttons
        admin_style = {"font": ("Arial", 8, "bold"), "bg": "#ff6b6b", "fg": "white", "relief": "raised", "bd": 2}
        tk.Button(self.f_btn, text="RESTART PROGRAM", command=self.reiniciar_programa, **admin_style).grid(row=1, column=0, columnspan=5, sticky="we", padx=2, pady=2)
        self.f_btn.grid_columnconfigure((0,1,2,3,4), weight=1)

        # Carga imagen para el Mapa
        try:
            m_img = Image.open("WALL-E HMI images/walle_location_plan_santa_maria_numbered.png")
            self.img_layout_full = ImageTk.PhotoImage(m_img.resize((440, 550), Image.LANCZOS))
        except: 
            self.img_layout_full = None

        # Setup GPIO for tower indicators
        try:
            if GPIO is not None and not DEMO_MODE:
                initial_off = GPIO.HIGH if RELAY_ACTIVE_LOW else GPIO.LOW
                GPIO.setup(PIN_GREEN_TURRET, GPIO.OUT, initial=initial_off)
                GPIO.setup(PIN_YELLOW_TURRET, GPIO.OUT, initial=initial_off)
                GPIO.setup(PIN_RED_TURRET, GPIO.OUT, initial=initial_off)
                GPIO.setup(PIN_BUZZER, GPIO.OUT, initial=initial_off)
                print("✓ Tower GPIO initialized")
            else:
                print("⚠ Tower GPIO skipped (demo mode or GPIO unavailable)")
        except Exception as e:
            print(f"⚠ Tower GPIO warning: {e}")

        # Start periodic update of device status from shared data
        self.actualizar_datos_dispositivos()

    def ensure_configuration_ready(self):
        if not get_system_config().get('setup_completed', False):
            if not self.show_configuration_dialog(first_run=True):
                save_system_config(get_system_config())
                self.registrar_log("Initial setup skipped; using current configuration")

    def rebuild_node_grid(self):
        configured_count = get_configured_node_count()
        self.total_pages = max(1, (configured_count + NODES_PER_PAGE - 1) // NODES_PER_PAGE)
        self.current_page = min(self.current_page, self.total_pages - 1)

        for widget in self.container.winfo_children():
            widget.destroy()

        self.leds_v = []
        self.lbls_v_val = []
        self.frames_robot = []
        self.uv_lamps = []
        self.model_labels = []
        self.mode_labels = []

        for device_id in range(1, configured_count + 1):
            frame = tk.Frame(self.container, bg="#3a2a7a", bd=1, relief="flat")
            pos_in_page = (device_id - 1) % NODES_PER_PAGE
            row = pos_in_page // HMI_GRID_COLUMNS
            col = pos_in_page % HMI_GRID_COLUMNS
            frame.grid(row=row, column=col, padx=1, pady=1, sticky="nsew")
            frame.device_id = device_id
            frame.page_num = (device_id - 1) // NODES_PER_PAGE
            self.frames_robot.append(frame)

            tk.Label(frame, text=f"W-{device_id}", font=("Arial", 8, "bold"), bg="#ffc72c", fg="black").pack(fill="x", pady=0)

            model_label = tk.Label(frame, text=get_node_model(device_id), font=("Arial", 6, "bold"), bg="#3a2a7a", fg="#9ed8ff")
            model_label.pack()
            self.model_labels.append(model_label)

            mode_text = "MAINTENANCE" if is_node_in_maintenance(device_id) else "OPERATING"
            mode_color = "#ffb347" if is_node_in_maintenance(device_id) else "#d9f99d"
            mode_label = tk.Label(frame, text=mode_text, font=("Arial", 6, "bold"), bg="#3a2a7a", fg=mode_color)
            mode_label.pack()
            self.mode_labels.append(mode_label)

            cv = tk.Canvas(frame, width=35, height=35, bg="#3a2a7a", highlightthickness=0)
            cv.pack(pady=0)
            circ_v = cv.create_oval(6, 6, 29, 29, fill="#555555", outline="white")
            self.leds_v.append((cv, circ_v))

            voltage_label = tk.Label(frame, text="--- V", font=("Arial", 7, "bold"), bg="#3a2a7a", fg="#ff4444")
            voltage_label.pack()
            self.lbls_v_val.append(voltage_label)

            lamps_frame = tk.Frame(frame, bg="#3a2a7a")
            lamps_frame.pack(pady=0)
            lamp_widgets = []
            for lamp_idx in range(2):
                lamp_canvas = tk.Canvas(lamps_frame, width=14, height=14, bg="#3a2a7a", highlightthickness=0)
                lamp_canvas.grid(row=0, column=lamp_idx, padx=1)
                lamp_circle = lamp_canvas.create_oval(2, 2, 12, 12, fill="#555555", outline="white")
                lamp_widgets.append((lamp_canvas, lamp_circle))
            self.uv_lamps.append(lamp_widgets)

            tk.Button(frame, text="VIEW", font=("Arial", 6, "bold"), bg="#ffc72c", fg="black",
                     command=lambda node_id=device_id: self.mostrar_detalle_lamparas(node_id),
                     height=1, padx=2).pack(fill="x", pady=2)
            frame.bind("<Button-1>", lambda e, node_id=device_id: self.mostrar_detalle_lamparas(node_id))

        self.refresh_page()

    def apply_system_configuration(self, new_config):
        save_system_config(new_config)
        synchronize_runtime_state()
        self.rebuild_node_grid()

    def show_configuration_dialog(self, first_run=False):
        current_config = get_system_config()
        dialog = tk.Toplevel(self.root)
        dialog.title("Initial Node Setup" if first_run else "System Settings")
        dialog.geometry("460x680")
        dialog.configure(bg="#1f1f2e")
        dialog.transient(self.root)
        dialog.grab_set()

        tk.Label(dialog,
                 text="Configure Nodes" if first_run else "Node Settings",
                 font=("Arial", 14, "bold"), bg="#1f1f2e", fg="white").pack(pady=(12, 4))
        tk.Label(dialog,
                 text="Select how many nodes are installed and assign a model to each one.",
                 font=("Arial", 9), bg="#1f1f2e", fg="#d1d5db").pack(pady=(0, 10))

        top_frame = tk.Frame(dialog, bg="#1f1f2e")
        top_frame.pack(fill="x", padx=12)
        tk.Label(top_frame, text="Number of Nodes", font=("Arial", 10, "bold"), bg="#1f1f2e", fg="white").pack(side="left")

        node_count_var = tk.IntVar(value=current_config.get('node_count', NUM_DEVICES))
        node_count_spinbox = tk.Spinbox(top_frame, from_=1, to=MAX_DEVICE_ID, width=6, textvariable=node_count_var)
        node_count_spinbox.pack(side="right")

        list_frame = tk.Frame(dialog, bg="#1f1f2e")
        list_frame.pack(expand=True, fill="both", padx=12, pady=10)

        canvas = tk.Canvas(list_frame, bg="#1f1f2e", highlightthickness=0)
        scrollbar = tk.Scrollbar(list_frame, orient="vertical", command=canvas.yview)
        rows_frame = tk.Frame(canvas, bg="#1f1f2e")
        rows_frame.bind("<Configure>", lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.create_window((0, 0), window=rows_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        canvas.pack(side="left", expand=True, fill="both")
        scrollbar.pack(side="right", fill="y")

        row_vars = {}
        dialog_state = {'saved': False}

        def build_rows(*_args):
            try:
                requested_count = int(node_count_var.get())
            except (TypeError, ValueError):
                requested_count = current_config.get('node_count', NUM_DEVICES)
            requested_count = max(MIN_DEVICE_ID, min(MAX_DEVICE_ID, requested_count))

            for widget in rows_frame.winfo_children():
                widget.destroy()

            for device_id in range(1, requested_count + 1):
                existing_node = current_config['nodes'][device_id - 1]
                if device_id not in row_vars:
                    row_vars[device_id] = {
                        'model': tk.StringVar(value=existing_node.get('model', NODE_MODEL_OPTIONS[0])),
                        'maintenance': tk.BooleanVar(value=existing_node.get('maintenance', False)),
                    }

                row = tk.Frame(rows_frame, bg="#2a2a3a", pady=4)
                row.pack(fill="x", pady=2)

                tk.Label(row, text=f"W-{device_id}", width=6, anchor="w", font=("Arial", 9, "bold"), bg="#2a2a3a", fg="white").pack(side="left", padx=6)

                option = tk.OptionMenu(row, row_vars[device_id]['model'], *NODE_MODEL_OPTIONS)
                option.config(width=10, bg="#ffc72c", fg="black", highlightthickness=0)
                option.pack(side="left", padx=4)

                tk.Checkbutton(row,
                               text="Maintenance mode",
                               variable=row_vars[device_id]['maintenance'],
                               bg="#2a2a3a", fg="#ffcc80", selectcolor="#2a2a3a",
                               activebackground="#2a2a3a", activeforeground="#ffcc80").pack(side="right", padx=8)

        def save_dialog():
            try:
                requested_count = int(node_count_var.get())
            except (TypeError, ValueError):
                messagebox.showerror("Invalid Value", "Node count must be a whole number.", parent=dialog)
                return

            requested_count = max(MIN_DEVICE_ID, min(MAX_DEVICE_ID, requested_count))
            new_config = normalize_system_config(current_config)
            new_config['setup_completed'] = True
            new_config['node_count'] = requested_count
            for device_id in range(1, requested_count + 1):
                new_config['nodes'][device_id - 1]['model'] = row_vars[device_id]['model'].get()
                new_config['nodes'][device_id - 1]['maintenance'] = bool(row_vars[device_id]['maintenance'].get())

            self.apply_system_configuration(new_config)
            dialog_state['saved'] = True
            dialog.destroy()

        def reset_configuration():
            if not messagebox.askyesno("Reset Configuration",
                                       "This will clear the saved node setup and reopen the setup wizard. Continue?",
                                       parent=dialog):
                return
            load_system_config(force_reset=True)
            dialog.destroy()
            self.show_configuration_dialog(first_run=True)

        def on_close():
            if first_run and not dialog_state['saved']:
                if messagebox.askyesno("Skip Setup", "Skip initial setup and continue with the current configuration?", parent=dialog):
                    dialog.destroy()
                return
            dialog.destroy()

        node_count_var.trace_add('write', build_rows)
        build_rows()

        button_frame = tk.Frame(dialog, bg="#1f1f2e")
        button_frame.pack(fill="x", padx=12, pady=(4, 12))
        if not first_run:
            tk.Button(button_frame, text="RESET CONFIGURATION", command=reset_configuration,
                     bg="#b91c1c", fg="white", font=("Arial", 9, "bold")).pack(side="left")
        tk.Button(button_frame, text="CANCEL", command=on_close,
                 bg="#6b7280", fg="white", font=("Arial", 9, "bold")).pack(side="right", padx=4)
        tk.Button(button_frame, text="SAVE", command=save_dialog,
                 bg="#16a34a", fg="white", font=("Arial", 9, "bold")).pack(side="right", padx=4)

        dialog.protocol("WM_DELETE_WINDOW", on_close)
        self.root.wait_window(dialog)
        return dialog_state['saved']

    def open_settings_dialog(self):
        self.show_configuration_dialog(first_run=False)

    # =======================================================
    # MÉTODOS DE LÓGICA Y CONTROL
    # =======================================================
    def actualizar_hora(self):
        self.lbl_reloj.config(text=datetime.now().strftime("%H:%M:%S"))
        self.actualizar_estado_escaneo()
        self.root.after(1000, self.actualizar_hora)
    
    def pagina_anterior(self):
        """Navigate to previous page"""
        if self.current_page > 0:
            self.current_page -= 1
            self.refresh_page()
    
    def pagina_siguiente(self):
        """Navigate to next page"""
        if self.current_page < self.total_pages - 1:
            self.current_page += 1
            self.refresh_page()
    
    def refresh_page(self):
        """Update visibility of frames based on current page"""
        for f in self.frames_robot:
            if f.page_num == self.current_page:
                f.grid()
            else:
                f.grid_remove()
        
        # Update page label
        start_device = self.current_page * NODES_PER_PAGE + 1
        end_device = min((self.current_page + 1) * NODES_PER_PAGE, get_configured_node_count())
        self.lbl_page.config(text=f"Page {self.current_page + 1} of {self.total_pages} (W-{start_device} to W-{end_device})")
    
    def actualizar_estado_escaneo(self):
        """Update scanning device status"""
        global current_scanning_device, device_data_lock
        try:
            with device_data_lock:
                device_id = current_scanning_device
            if device_id > 0:
                self.lbl_scanning.config(text=f"Scanning: W-{device_id}")
            else:
                self.lbl_scanning.config(text="Scanning: W-0")
        except:
            pass

    def test_mode_on(self):
        self.test_mode = True
        self.registrar_log("TEST MODE ON")

    def test_mode_off(self):
        self.test_mode = False
        self.registrar_log("TEST MODE OFF")

    def reiniciar_programa(self):
        """Restart program: closes and reopens automatically."""
        if messagebox.askyesno("Restart", "Restart the program?"):
            self.registrar_log("RESTARTING PROGRAM...")
            global coordinator_running, restart_requested
            restart_requested = True
            coordinator_running = False
            self.root.after(500, lambda: self.root.quit())

    def classify_current_status(self, current_mA):
        """Return (status_code, color, description) for active current indicators."""
        if current_mA < CURRENT_RED_THRESHOLD:
            return "RED", "#e74c3c", "Both lamps failed"
        if current_mA <= CURRENT_MAX:
            return "GREEN", "#2ecc71", "Lamps working OK"
        return "RED", "#e74c3c", "Both lamps failed"

    def actualizar_torreta(self, confirmed_sensor_failure, communication_failure):
        """
        Update physical tower light and buzzer based on system health
        
        Args:
            confirmed_sensor_failure: True if any node remains in sensor error >= MAX_CONSECUTIVE_FAILURES reads
            communication_failure: True if any device has >= MAX_CONSECUTIVE_FAILURES
        """
        try:
            # Yellow tower behavior intentionally disabled.
            red_active = communication_failure or confirmed_sensor_failure
            yellow_active = False
            green_active = (not red_active) and (not yellow_active)

            # Only update relays if state changed to reduce GPIO interference with SPI
            if green_active != self.torreta_green_active:
                set_relay(PIN_GREEN_TURRET, green_active)
                self.torreta_green_active = green_active
                
            if yellow_active != self.torreta_yellow_active:
                set_relay(PIN_YELLOW_TURRET, yellow_active)
                self.torreta_yellow_active = yellow_active
            
            # Save old red state before updating
            red_was_active = self.torreta_red_active
            if red_active != self.torreta_red_active:
                set_relay(PIN_RED_TURRET, red_active)
                self.torreta_red_active = red_active

            now = time.time()

            # Buzzer logic: Only works when red is active
            if red_active and self.sonido_habil:
                # If red just turned on, initialize buzzer timer
                if not red_was_active:
                    self.last_buzzer_time = now - 10.0
                    print(f"[BUZZER] Red activated, buzzer initialized")
                # Trigger buzzer every 10 seconds
                if not self.buzzer_active and (now - self.last_buzzer_time) >= 10.0:
                    set_relay(PIN_BUZZER, True)
                    self.buzzer_active = True
                    self.buzzer_off_time = now + 1.0
                    self.last_buzzer_time = now
                    print(f"[BUZZER] ON after {now - self.last_buzzer_time:.1f}s")
                # Turn off buzzer after 1 second
                if self.buzzer_active and now >= self.buzzer_off_time:
                    set_relay(PIN_BUZZER, False)
                    self.buzzer_active = False
                    print(f"[BUZZER] OFF after 1s")
            else:
                # Red is not active, make sure buzzer is off
                if self.buzzer_active:
                    set_relay(PIN_BUZZER, False)
                    self.buzzer_active = False
        except Exception as e:
            print(f"[TOWER ERROR] {e}")
            import traceback
            traceback.print_exc()
    
    def actualizar_buzzer(self):
        """Update buzzer independently every 100ms to ensure it triggers while red is active"""
        if not self.torreta_red_active or not self.sonido_habil:
            return
        
        now = time.time()
        # Trigger buzzer every 10 seconds
        if not self.buzzer_active and (now - self.last_buzzer_time) >= 10.0:
            set_relay(PIN_BUZZER, True)
            self.buzzer_active = True
            self.buzzer_off_time = now + 1.0
            self.last_buzzer_time = now
            print(f"[BUZZER] Sound ON")
        # Turn off buzzer after 1 second
        if self.buzzer_active and now >= self.buzzer_off_time:
            set_relay(PIN_BUZZER, False)
            self.buzzer_active = False
            print(f"[BUZZER] Sound OFF")

    def actualizar_datos_dispositivos(self):
        """Update HMI display with latest device data from coordinator"""
        global shared_device_data, device_data_lock, device_consecutive_failures, device_consecutive_sensor_errors

        confirmed_sensor_failure_detected = False
        communication_failure_detected = False

        try:
            with device_data_lock:
                configured_count = get_configured_node_count()
                for device_id in range(1, configured_count + 1):
                    idx = device_id - 1
                    node_config = get_node_config(device_id)

                    self.model_labels[idx].config(text=node_config['model'])
                    if node_config['maintenance']:
                        self.mode_labels[idx].config(text="MAINTENANCE", fg="#ffb347")
                        self.leds_v[idx][0].itemconfig(self.leds_v[idx][1], fill="#6b7280")
                        for lamp_i in range(2):
                            self.uv_lamps[idx][lamp_i][0].itemconfig(self.uv_lamps[idx][lamp_i][1], fill="#6b7280")
                        self.lbls_v_val[idx].config(text="MAINT", fg="#ffb347")
                        continue
                    else:
                        self.mode_labels[idx].config(text="OPERATING", fg="#d9f99d")
                    
                    if device_id in shared_device_data:
                        data = shared_device_data[device_id]
                        
                        # Extract sensor values
                        voltage = data.get('ac_voltage_V', 0)
                        curr1 = data.get('curr1_mA', 0)
                        curr3 = data.get('curr3_mA', 0)
                        
                        # Direct error: voltage is only faulted when below minimum threshold.
                        v_ok = voltage >= VOLTAGE_MIN
                        current_status = [
                            self.classify_current_status(curr1),
                            self.classify_current_status(curr3),
                        ]

                        # Confirmed sensor error only after MAX_CONSECUTIVE_FAILURES reads per node
                        if (not self.test_mode or device_id == 1) and device_consecutive_sensor_errors.get(device_id, 0) >= MAX_CONSECUTIVE_FAILURES:
                            confirmed_sensor_failure_detected = True

                        # Check consecutive failures instead of timestamp
                        # Only trigger alarm after MAX_CONSECUTIVE_FAILURES (prevents false alarms during LoRa recovery)
                        consecutive_failures = device_consecutive_failures.get(device_id, 0)
                        if consecutive_failures >= MAX_CONSECUTIVE_FAILURES:
                            # In test mode, only Device 1 can trigger communication failure alarm
                            if not self.test_mode or device_id == 1:
                                communication_failure_detected = True
                        
                        # Update visual indicators
                        color_v = "#2ecc71" if v_ok else "#e74c3c"
                        
                        self.leds_v[idx][0].itemconfig(self.leds_v[idx][1], fill=color_v)
                        # Update active current indicators (CH1 and CH3)
                        for lamp_i, (_, lamp_color, _) in enumerate(current_status):
                            self.uv_lamps[idx][lamp_i][0].itemconfig(self.uv_lamps[idx][lamp_i][1], fill=lamp_color)
                        self.lbls_v_val[idx].config(text=f"{voltage:.1f} V", fg="white" if v_ok else "#ff4444")

                    else:
                        # No data available for this device - check consecutive failures
                        consecutive_failures = device_consecutive_failures.get(device_id, 0)
                        
                        if self.test_mode:
                            # In test mode, treat missing devices as OK (except Device 1)
                            self.leds_v[idx][0].itemconfig(self.leds_v[idx][1], fill="#2ecc71")
                            for lamp_i in range(2):
                                self.uv_lamps[idx][lamp_i][0].itemconfig(self.uv_lamps[idx][lamp_i][1], fill="#2ecc71")
                            self.lbls_v_val[idx].config(text="OK", fg="white")
                            # Only trigger alarm if Device 1 has exceeded failure threshold
                            if device_id == 1 and consecutive_failures >= MAX_CONSECUTIVE_FAILURES:
                                communication_failure_detected = True
                            if device_id == 1 and device_consecutive_sensor_errors.get(device_id, 0) >= MAX_CONSECUTIVE_FAILURES:
                                confirmed_sensor_failure_detected = True
                        else:
                            self.leds_v[idx][0].itemconfig(self.leds_v[idx][1], fill="#555555")
                            for lamp_i in range(2):
                                self.uv_lamps[idx][lamp_i][0].itemconfig(self.uv_lamps[idx][lamp_i][1], fill="#555555")
                            self.lbls_v_val[idx].config(text="--- V", fg="#ff4444")
                            # Only trigger alarm if exceeded failure threshold
                            if consecutive_failures >= MAX_CONSECUTIVE_FAILURES:
                                communication_failure_detected = True
                            if device_consecutive_sensor_errors.get(device_id, 0) >= MAX_CONSECUTIVE_FAILURES:
                                confirmed_sensor_failure_detected = True
        
        except Exception as e:
            print(f"HMI update error: {e}")

        # Update tower status based on system health (only if state changed)
        current_state = (confirmed_sensor_failure_detected, communication_failure_detected)
        if current_state != self.last_tower_state:
            self.actualizar_torreta(confirmed_sensor_failure_detected, communication_failure_detected)
            self.last_tower_state = current_state
        
        # Update buzzer independently (called every 1 second)
        self.actualizar_buzzer()

        if confirmed_sensor_failure_detected or communication_failure_detected:
            self.activar_alerta("SYSTEM FAULT")
        else:
            self.limpiar_alerta()
        
        # Schedule next update
        self.root.after(1000, self.actualizar_datos_dispositivos)

    def mostrar_imagen_layout(self):
        if not self.img_layout_full:
            messagebox.showwarning("Error", "Map image not found: WALL-E HMI images/walle_location_plan_santa_maria_numbered.png")
            return
        
        # Create a new window for the map
        top = tk.Toplevel(self.root)
        top.title("Plant Map - Wall-E")
        top.geometry("450x650")
        top.configure(bg="#222222")
        top.resizable(True, True)
        
        # Frame for image and close button
        frame_img = tk.Frame(top, bg="#222222")
        frame_img.pack(expand=True, fill="both", padx=10, pady=10)
        
        # Display image
        lbl_img = tk.Label(frame_img, image=self.img_layout_full, bg="#222222")
        lbl_img.pack(expand=True, fill="both")
        
        # Frame for buttons at bottom
        frame_btn = tk.Frame(top, bg="#222222")
        frame_btn.pack(side="bottom", fill="x", padx=10, pady=10)
        
        tk.Button(frame_btn, text="CLOSE", command=top.destroy, bg="red", fg="white", 
                 font=("Arial", 10, "bold"), width=20).pack(pady=5)

    def mostrar_detalle_lamparas(self, device_id):
        """Open a window showing per-lamp UV status for a device"""
        top = tk.Toplevel(self.root)
        top.title(f"Current Status - W-{device_id}")
        top.geometry("420x320")
        top.configure(bg="#1a1a1a")

        tk.Label(top, text=f"W-{device_id} - {get_node_model(device_id)}",
                 font=("Arial", 12, "bold"), bg="#1a1a1a", fg="#00ff00").pack(pady=10)

        if is_node_in_maintenance(device_id):
            tk.Label(top, text="Node is in maintenance mode. Alerts are disabled.",
                     bg="#1a1a1a", fg="#ffb347", font=("Arial", 10, "bold")).pack(pady=(0, 8))

        frame = tk.Frame(top, bg="#1a1a1a")
        frame.pack(expand=True, fill="both", padx=10, pady=10)

        with device_data_lock:
            data = shared_device_data.get(device_id)

        if not data:
            tk.Label(frame, text="No data from node.", bg="#1a1a1a", fg="white").pack(pady=10)
        else:
            currents = [
                ("CH1 UV Lamps 1 & 2", data.get('curr1_mA', 0)),
                ("CH2 UV Lamps 3 & 4", data.get('curr3_mA', 0)),
            ]

            for channel_name, curr in currents:
                _, color, description = self.classify_current_status(curr)

                row = tk.Frame(frame, bg="#1a1a1a")
                row.pack(fill="x", pady=4)

                lamp_canvas = tk.Canvas(row, width=20, height=20, bg="#1a1a1a", highlightthickness=0)
                lamp_canvas.pack(side="left", padx=8)
                lamp_canvas.create_oval(3, 3, 17, 17, fill=color, outline="white")

                tk.Label(row, text=f"{channel_name}: {curr:.0f} mA",
                         font=("Arial", 10), bg="#1a1a1a", fg="white").pack(side="left")
                tk.Label(row, text=description, font=("Arial", 10, "bold"), bg="#1a1a1a", fg=color).pack(side="right")

        tk.Button(top, text="CLOSE", command=top.destroy, bg="red", fg="white",
                  font=("Arial", 10, "bold"), width=20).pack(pady=10)

    def activar_alerta(self, msg):
        if not self.falla_activa:  # Only log once when fault first detected
            self.falla_activa = True
            self.registrar_log(msg)

    def limpiar_alerta(self):
        self.falla_activa = False

    def silenciar(self):
        self.sonido_habil = False
        try:
            set_relay(PIN_BUZZER, False)
        except:
            pass

    def reset(self):
        """Re-enable sound"""
        self.sonido_habil = True
        print("[SOUND] Sound re-enabled")
        self.registrar_log("Sound re-enabled")

    def registrar_log(self, info):
        try:
            with open("log_seguridad.csv", "a") as f:
                f.write(f"{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}, {info}\n")
        except Exception as e:
            print(f"Log write error: {e}")

    def abrir_historial(self):
        # Create a new window for logs
        pop = tk.Toplevel(self.root)
        pop.title("Event History - Wall-E")
        pop.geometry("350x450")
        pop.resizable(True, True)
        
        # Frame for title
        frame_title = tk.Frame(pop, bg="#1a1a1a", height=40)
        frame_title.pack(fill="x")
        tk.Label(frame_title, text="RECENT EVENTS", font=("Arial", 12, "bold"), 
                bg="#1a1a1a", fg="#00ff00").pack(pady=5)
        
        # Frame for text and scrollbar
        frame_text = tk.Frame(pop, bg="#111111")
        frame_text.pack(expand=True, fill="both", padx=5, pady=5)
        
        # Create text widget with scrollbar
        scrollbar = tk.Scrollbar(frame_text)
        scrollbar.pack(side="right", fill="y")
        
        txt = tk.Text(frame_text, font=("Courier", 9), bg="#111111", fg="#00ff00",
                     yscrollcommand=scrollbar.set, wrap="word")
        txt.pack(expand=True, fill="both", side="left")
        scrollbar.config(command=txt.yview)
        
        # Load log content
        try:
            with open("log_seguridad.csv", "r") as f:
                logs = f.readlines()[-50:]  # Show last 50 lines
                content = "".join(reversed(logs))
                txt.insert("1.0", content)
        except:
            txt.insert("1.0", "No previous records.")
        
        txt.config(state="disabled")  # Make it read-only
        
        # Frame for buttons at bottom
        frame_btn = tk.Frame(pop, bg="#1a1a1a", height=50)
        frame_btn.pack(fill="x", padx=5, pady=5)
        
        tk.Button(frame_btn, text="CLOSE", command=pop.destroy, bg="red", fg="white",
                 font=("Arial", 10, "bold"), width=20).pack(side="left", padx=5)
        tk.Button(frame_btn, text="CLEAR LOGS", command=self.limpiar_logs, bg="orange", fg="white",
                 font=("Arial", 10, "bold"), width=20).pack(side="left", padx=5)
    
    def limpiar_logs(self):
        """Clear the security log file"""
        try:
            with open("log_seguridad.csv", "w") as f:
                f.write("Logs cleared on {}\n".format(datetime.now().strftime("%Y-%m-%d %H:%M:%S")))
            messagebox.showinfo("Success", "Logs cleared successfully")
        except Exception as e:
            messagebox.showerror("Error", f"Error clearing logs: {e}")

# ============================================================================
# COORDINATOR THREAD
# ============================================================================

def coordinator_loop():
    """Main coordinator loop running in background thread"""
    global coordinator_running, shared_device_data, device_data_lock
    
    if DEMO_MODE:
        demo_loop()
    else:
        print("\n✓ Coordinator thread started")
        
        cycle_count = 0
        while coordinator_running:
            try:
                cycle_count += 1
                print(f"\n{'='*70}")
                print(f"Query Cycle #{cycle_count}: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
                print(f"{'='*70}")
                
                # FULL RESET of LoRa module before each cycle
                try:
                    print("[RESET] Performing HARD RESET of LoRa module via GPIO...")
                    
                    # Hard reset via GPIO RST pin
                    GPIO.output(GPIO_RST, GPIO.LOW)
                    time.sleep(0.05)
                    GPIO.output(GPIO_RST, GPIO.HIGH)
                    time.sleep(0.1)
                    
                    # Now reconfigure the module
                    configure_lora()
                    
                except Exception as e:
                    print(f"[WARNING] Error during hard reset: {e}")

                synchronize_runtime_state()
                
                # Query all devices
                for device_id in get_active_device_ids():
                    response = query_device(device_id)
                    
                    # Update shared data structure (thread-safe)
                    with device_data_lock:
                        if response:
                            # Update with new data
                            shared_device_data[device_id] = response
                        else:
                            # Remove stale data when no response to avoid showing old data
                            if device_id in shared_device_data:
                                del shared_device_data[device_id]
                    
                    time.sleep(0.5)  # Delay between requests
                
                # Print device statistics
                print(f"\nDevice Statistics:")
                for device_id in range(1, get_configured_node_count() + 1):
                    if device_id in device_stats:
                        stats = device_stats[device_id]
                        success_rate = (stats['responses'] * 100) / (stats['responses'] + stats['failures']) if (stats['responses'] + stats['failures']) > 0 else 0
                        print(f"  Device {device_id}: {stats['responses']} responses, " +
                              f"{stats['failures']} failures ({success_rate:.1f}% success)")
                
                # Wait for next cycle
                print(f"\nWaiting {QUERY_INTERVAL} seconds for next cycle...")
                time.sleep(QUERY_INTERVAL)
        
            except Exception as e:
                print(f"✗ Coordinator error: {e}")
                time.sleep(1)
        
        print("\n✓ Coordinator thread stopped")

# ============================================================================
# MAIN PROGRAM
# ============================================================================

def main():
    """Main entry point - initializes hardware and launches HMI + coordinator"""
    global coordinator_running, coordinator_thread, restart_requested

    load_system_config()

    configured_count = get_configured_node_count()

    if not (MIN_DEVICE_ID <= configured_count <= MAX_DEVICE_ID):
        print(f"✗ Invalid node count={configured_count}. Valid range: {MIN_DEVICE_ID}-{MAX_DEVICE_ID}")
        return 1
    
    print("="*70)
    if DEMO_MODE:
        print("          Wall-E Coordinator with HMI - DEMO MODE")
    else:
        print("          Wall-E Coordinator with HMI - Starting Up")
    print("="*70)
    print(f"Configuration:")
    print(f"  - Number of devices: {configured_count}")
    print(f"  - Supported ID range: {MIN_DEVICE_ID}-{MAX_DEVICE_ID}")
    if DEMO_MODE:
        print(f"  - Demo update interval: {DEMO_UPDATE_INTERVAL} seconds")
    else:
        print(f"  - Query interval: {QUERY_INTERVAL} seconds")
        print(f"  - Response timeout: {RESPONSE_TIMEOUT} seconds")
    print(f"  - Frequency: {FREQUENCY/1e6} MHz")
    print("="*70)
    
    # Initialize hardware (skip in demo mode)
    if not DEMO_MODE:
        if not initialize_hardware():
            print("✗ Hardware initialization failed. Exiting.")
            return 1
        
        if not configure_lora():
            print("✗ LoRa configuration failed. Exiting.")
            return 1
    else:
        print("✓ Demo mode enabled - using simulated data")
    
    print("\n✓ System ready. Starting coordinator and HMI...")

    # Launch HMI (runs in main thread)
    try:
        root = tk.Tk()
        app = AppIndustrial(root)

        if not root.winfo_exists() or getattr(app, 'initialization_aborted', False):
            print("⚠ HMI closed during initial setup")
            return 0

        coordinator_running = True
        coordinator_thread = threading.Thread(target=coordinator_loop, daemon=True)
        coordinator_thread.start()
        
        print("\n✓ HMI launched. Coordinator running in background.")
        if DEMO_MODE:
            print("   Demo data will update every {} seconds.".format(DEMO_UPDATE_INTERVAL))
        print("   Close the GUI window to exit.\n")
        
        root.mainloop()
        
    except KeyboardInterrupt:
        print("\n\n✓ Keyboard interrupt received...")
    except Exception as e:
        print(f"\n✗ HMI Error: {e}")
    finally:
        # Cleanup
        print("\n✓ Shutting down...")
        coordinator_running = False
        
        if coordinator_thread and coordinator_thread.is_alive():
            print("  Waiting for coordinator thread to stop...")
            coordinator_thread.join(timeout=5)
        
        if not DEMO_MODE:
            try:
                if spi is not None:
                    spi.close()
                if GPIO is not None:
                    GPIO.cleanup()
                print("✓ Resources cleaned up")
            except Exception as e:
                print(f"⚠ Cleanup warning: {e}")

        if restart_requested:
            try:
                restart_requested = False
                cmd = [sys.executable] + sys.argv
                subprocess.Popen(cmd, cwd=os.getcwd())
                print("✓ Reinicio de programa solicitado: nueva instancia iniciada")
            except Exception as e:
                print(f"✗ No se pudo reiniciar el programa automaticamente: {e}")
        
        print("✓ Exited successfully\n")
    
    return 0

if __name__ == "__main__":
    main()
