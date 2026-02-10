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
  Response: [NET_ID | MSG_RESP | DEVICE_ID | SEQ_LO | SEQ_HI | AC_V_SCALED | CURR1_mA | CURR2_mA | CURR3_mA | CURR4_mA]
           where scaled values: 0-255 = voltage 0-130V RMS, current 0-2550 mA per channel

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
import threading
import tkinter as tk
from tkinter import messagebox
from PIL import Image, ImageTk
import random

# ============================================================================
# HARDWARE CONFIGURATION
# ============================================================================

# GPIO Pin mappings (BCM numbering)
GPIO_DIO0 = 4
GPIO_DIO1 = 17
GPIO_DIO2 = 18
GPIO_DIO3 = 27
GPIO_RST = 22

# HMI GPIO Pins
PIN_VERDE = 17    # Green LED (shared with GPIO_DIO1, will manage carefully)
PIN_ROJO = 27     # Red LED (shared with GPIO_DIO3, will manage carefully) 
PIN_BUZZER = 22   # Buzzer (shared with GPIO_RST, will manage carefully)

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
REG_PAYLOAD_LENGTH = 0x22

# ============================================================================
# SYSTEM CONFIGURATION
# ============================================================================

NUM_DEVICES = 9                    # Number of remote nodes (1-9)
QUERY_INTERVAL = 5.0               # Seconds between query cycles
RESPONSE_TIMEOUT = 4.0             # Seconds to wait for each response
FREQUENCY = 433E6                  # LoRa frequency (Hz)

# HMI THRESHOLDS
VOLTAGE_MIN = 100.0                # Minimum acceptable voltage (V)
VOLTAGE_MAX = 135.0                # Maximum acceptable voltage (V)
CURRENT_MIN = 500.0                # Minimum acceptable current per lamp (mA)
CURRENT_MAX = 1200.0               # Maximum acceptable current per lamp (mA)

# DEMO MODE (for testing without real hardware)
DEMO_MODE = False                  # Set to False for real hardware testing with ESP32
DEMO_UPDATE_INTERVAL = 2.0         # Seconds between demo data updates

# ============================================================================
# GLOBAL STATE
# ============================================================================

spi = None
last_response = {}                 # Track last response from each device
device_stats = {}                  # Statistics for each device

# Shared data structure for HMI (thread-safe)
device_data_lock = threading.Lock()
shared_device_data = {}            # Device status shared between coordinator thread and HMI
coordinator_running = False        # Flag to control coordinator thread
coordinator_thread = None          # Reference to coordinator thread

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
        
        # 6. Modem config for 125 kHz bandwidth, SF7, CR4/5, explicit header
        spi_write(REG_MODEM_CONFIG_1, 0x72)
        spi_write(REG_MODEM_CONFIG_2, 0x74)
        
        # 7. Enable CRC
        spi_write(0x1E, 0x74)  # REG_MODEM_CONFIG_2 with CRC on
        
        # 8. Sync word (private network) - DEBE COINCIDIR CON ESP32
        spi_write(REG_SYNC_WORD, 0x21)
        
        # 9. Preamble length (8 symbols)
        spi_write(REG_PREAMBLE_MSB, 0x00)
        spi_write(REG_PREAMBLE_LSB, 0x08)
        
        # 10. Set to standby mode
        spi_write(REG_OP_MODE, 0x81)  # Standby mode
        time.sleep(0.01)
        
        # 11. Set to receive continuous mode
        spi_write(REG_OP_MODE, 0x85)  # RX continuous
        time.sleep(0.1)
        
        # Verify configuration
        print("✓ LoRa configured:")
        print("  - Frequency: 433 MHz")
        print("  - Spreading Factor: 7")
        print("  - Bandwidth: 125 kHz")
        print("  - Coding Rate: 4/5")
        print("  - Sync Word: 0x21")
        print(f"  - Mode register: 0x{spi_read(REG_OP_MODE):02X}")
        print(f"  - Modem Config 1: 0x{spi_read(REG_MODEM_CONFIG_1):02X}")
        print(f"  - Modem Config 2: 0x{spi_read(REG_MODEM_CONFIG_2):02X}")
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
        
        # 6. Set to TX mode
        spi_write(REG_OP_MODE, 0x83)  # TX mode
        
        # 7. Wait for transmission complete
        start_time = time.time()
        while (spi_read(REG_IRQ_FLAGS) & 0x08) == 0:  # Wait for TxDone
            if time.time() - start_time > 1.0:
                print(f"✗ Transmission timeout for device {device_id}")
                return False
            time.sleep(0.01)
        
        # 8. Clear interrupt and return to RX mode
        spi_write(REG_IRQ_FLAGS, 0xFF)
        spi_write(REG_OP_MODE, 0x85)  # RX continuous mode
        time.sleep(0.01)
        
        return True
    except Exception as e:
        print(f"✗ Send error: {e}")
        return False

# ============================================================================
# RECEPTION FUNCTIONS
# ============================================================================

def unscale_voltage(scaled_value):
    """Convert 8-bit scaled value back to voltage (0-255 = 0-130V RMS)"""
    return (scaled_value / 255.0) * 130.0

def unscale_current(scaled_value):
    """Convert 8-bit scaled value back to current in mA (0-255 = 0-2550 mA)"""
    return (scaled_value / 255.0) * 2550.0

def wait_response(device_id, timeout=RESPONSE_TIMEOUT):
    """Wait for response from specific device"""
    start_time = time.time()
    
    while time.time() - start_time < timeout:
        try:
            irq_flags = spi_read(REG_IRQ_FLAGS)
            # CRC error flag
            if irq_flags & 0x20:
                print(f"[DEBUG] CRC error detected for device {device_id}, clearing IRQ flags.")
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
                    spi_write(REG_OP_MODE, 0x85)  # Back to RX continuous

                    print(f"[DEBUG] Received packet from device {device_id}: {packet}")
                    return bytes(packet)
                else:
                    print(f"[DEBUG] Unexpected packet size {nb_bytes} from device {device_id}, clearing IRQ.")
                    # Clear IRQ and continue listening
                    spi_write(REG_IRQ_FLAGS, 0xFF)
                    spi_write(REG_OP_MODE, 0x85)
        except Exception as e:
            print(f"[DEBUG] Exception while waiting for response from device {device_id}: {e}")

        time.sleep(0.02)

    print(f"[DEBUG] Timeout waiting for response from device {device_id} after {timeout} seconds.")
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

# ============================================================================
# QUERY CYCLE
# ============================================================================

def query_device(device_id):
    """Query single device and collect response"""
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
                    return response
                else:
                    print("✗ Invalid response format")
            else:
                print(f"✗ No response (timeout) [Intento {attempt}]")
        else:
            print(f"✗ Send failed [Intento {attempt}]")
        time.sleep(0.2)
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
            
            # Generate and update simulated data for all devices
            for device_id in range(1, NUM_DEVICES + 1):
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
            print(f"  - Total devices simulated: {NUM_DEVICES}")
            
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
        self.root.geometry("480x800")
        self.root.configure(bg="#483698")
        
        self.sonido_habil = True  
        self.falla_activa = False

        # --- ENCABEZADO (Logos e Iconos) ---
        self.header = tk.Frame(self.root, bg="#483698")
        self.header.pack(fill="x", padx=15, pady=10)

        try:
            # Logo Bimbo como icono (sin recuadro blanco)
            img_b = Image.open("WALL-E HMI images/Grupo_Bimbo.png").convert("RGBA")
            self.photo = ImageTk.PhotoImage(img_b.resize((70, 35), Image.LANCZOS))
            tk.Label(self.header, image=self.photo, bg="#483698").pack(side="left")

            # Logo Moldex como icono
            img_m = Image.open("WALL-E HMI images/Moldex1.png").convert("RGBA")
            self.photo2 = ImageTk.PhotoImage(img_m.resize((70, 35), Image.LANCZOS))
            tk.Label(self.header, image=self.photo2, bg="#483698").pack(side="left", padx=15)
        except:
            tk.Label(self.header, text="DASHBOARD", fg="white", bg="#483698", font=("Arial", 10, "bold")).pack(side="left")

        # Reloj en la esquina superior derecha
        self.lbl_reloj = tk.Label(self.header, text="", font=("Courier", 12, "bold"), fg="#00ff00", bg="#483698")
        self.lbl_reloj.pack(side="right")
        self.actualizar_hora()

        tk.Label(self.root, text="MONITOREO DE LÁMPARAS", font=("Arial", 13, "bold"), fg="white", bg="#483698").pack(pady=5)

        # --- PANEL DE ROBOTS (DISTRIBUCIÓN 3-3-3) ---
        self.container = tk.Frame(self.root, bg="#483698")
        self.container.pack(expand=True, fill="both", padx=5)

        self.leds_v, self.lbls_v_val, self.frames_robot = [], [], []
        self.uv_lamps = []  # per-node list of 4 lamp indicators

        for i in range(NUM_DEVICES):
            # Creamos una "tarjeta" para cada robot
            f = tk.Frame(self.container, bg="#3a2a7a", bd=1, relief="flat")
            f.grid(row=i // 3, column=i % 3, padx=5, pady=8, sticky="nsew")
            self.frames_robot.append(f)

            tk.Label(f, text=f"W-{i+1}", font=("Arial", 9, "bold"), bg="#ffc72c", fg="black").pack(fill="x")
            
            # LED Alimentación
            cv = tk.Canvas(f, width=50, height=50, bg="#3a2a7a", highlightthickness=0)
            cv.pack()
            circ_v = cv.create_oval(8, 8, 42, 42, fill="#555555", outline="white")
            self.leds_v.append((cv, circ_v))
            
            lv = tk.Label(f, text="--- V", font=("Arial", 8, "bold"), bg="#3a2a7a", fg="#ff4444")
            lv.pack()
            self.lbls_v_val.append(lv)

            # Indicadores por lámpara UV (4 focos pequeños)
            lamps_frame = tk.Frame(f, bg="#3a2a7a")
            lamps_frame.pack(pady=2)
            lamp_widgets = []
            for lamp_idx in range(4):
                lamp_canvas = tk.Canvas(lamps_frame, width=16, height=16, bg="#3a2a7a", highlightthickness=0)
                lamp_canvas.grid(row=0, column=lamp_idx, padx=2)
                lamp_circle = lamp_canvas.create_oval(3, 3, 13, 13, fill="#555555", outline="white")
                lamp_widgets.append((lamp_canvas, lamp_circle))
            self.uv_lamps.append(lamp_widgets)

            # Botón DETALLE para ver lámparas individuales
            tk.Button(f, text="DETALLE", font=("Arial", 7, "bold"), bg="#ffc72c", fg="black",
                     command=lambda device_id=i+1: self.mostrar_detalle_lamparas(device_id),
                     height=1, padx=2).pack(fill="x", pady=1)

            # Click para ver detalle por nodo
            f.bind("<Button-1>", lambda e, node_id=i+1: self.mostrar_detalle_lamparas(node_id))

        # Configurar columnas iguales
        for j in range(3): self.container.grid_columnconfigure(j, weight=1)

        # --- BOTONERA INFERIOR ---
        self.f_btn = tk.Frame(self.root, bg="#483698")
        self.f_btn.pack(side="bottom", fill="x", pady=15)
        
        b_style = {"font": ("Arial", 8, "bold"), "bg": "#ffc72c", "height": 2, "activebackground": "#e6b422"}
        
        tk.Button(self.f_btn, text="SILENCIAR", command=self.silenciar, **b_style).grid(row=0, column=0, sticky="we", padx=2)
        tk.Button(self.f_btn, text="RESET", command=self.reset, **b_style).grid(row=0, column=1, sticky="we", padx=2)
        tk.Button(self.f_btn, text="LOGS", command=self.abrir_historial, **b_style).grid(row=0, column=2, sticky="we", padx=2)
        tk.Button(self.f_btn, text="MAPA", command=self.mostrar_imagen_layout, **b_style).grid(row=0, column=3, sticky="we", padx=2)
        self.f_btn.grid_columnconfigure((0,1,2,3), weight=1)

        # Carga imagen para el Mapa
        try:
            m_img = Image.open("WALL-E HMI images/Bimbo.png")
            self.img_layout_full = ImageTk.PhotoImage(m_img.resize((440, 550), Image.LANCZOS))
        except: 
            self.img_layout_full = None

        # Setup GPIO for HMI indicators (configure outputs that don't conflict with LoRa)
        try:
            GPIO.setup(PIN_BUZZER, GPIO.OUT, initial=GPIO.LOW)
            # Note: PIN_VERDE and PIN_ROJO overlap with LoRa DIO pins, so we'll be careful
            print("✓ HMI GPIO initialized")
        except Exception as e:
            print(f"⚠ HMI GPIO warning: {e}")

        # Start periodic update of device status from shared data
        self.actualizar_datos_dispositivos()

    # =======================================================
    # MÉTODOS DE LÓGICA Y CONTROL
    # =======================================================
    def actualizar_hora(self):
        self.lbl_reloj.config(text=datetime.now().strftime("%H:%M:%S"))
        self.root.after(1000, self.actualizar_hora)

    def actualizar_datos_dispositivos(self):
        """Update HMI display with latest device data from coordinator"""
        global shared_device_data, device_data_lock
        
        try:
            with device_data_lock:
                for device_id in range(1, NUM_DEVICES + 1):
                    idx = device_id - 1
                    
                    if device_id in shared_device_data:
                        data = shared_device_data[device_id]
                        
                        # Extract sensor values
                        voltage = data.get('ac_voltage_V', 0)
                        curr1 = data.get('curr1_mA', 0)
                        curr2 = data.get('curr2_mA', 0)
                        curr3 = data.get('curr3_mA', 0)
                        curr4 = data.get('curr4_mA', 0)
                        
                        # Check thresholds
                        v_ok = VOLTAGE_MIN <= voltage <= VOLTAGE_MAX
                        lamp_status = [
                            CURRENT_MIN <= curr1 <= CURRENT_MAX,
                            CURRENT_MIN <= curr2 <= CURRENT_MAX,
                            CURRENT_MIN <= curr3 <= CURRENT_MAX,
                            CURRENT_MIN <= curr4 <= CURRENT_MAX,
                        ]
                        uv_ok = all(lamp_status)
                        
                        # Update visual indicators
                        color_v = "#2ecc71" if v_ok else "#e74c3c"
                        
                        self.leds_v[idx][0].itemconfig(self.leds_v[idx][1], fill=color_v)
                        # Update per-lamp indicators
                        for lamp_i, lamp_ok in enumerate(lamp_status):
                            lamp_color = "#2ecc71" if lamp_ok else "#e74c3c"
                            self.uv_lamps[idx][lamp_i][0].itemconfig(self.uv_lamps[idx][lamp_i][1], fill=lamp_color)
                        self.lbls_v_val[idx].config(text=f"{voltage:.1f} V", fg="white" if v_ok else "#ff4444")

                        # Trigger alert if any device has issues
                        if not v_ok or not uv_ok:
                            self.activar_alerta(f"FALLA W-{device_id}")
                        else:
                            # Only clear if no devices have active faults
                            any_faults = False
                            for check_id in range(1, NUM_DEVICES + 1):
                                if check_id in shared_device_data:
                                    check_data = shared_device_data[check_id]
                                    check_v = check_data.get('ac_voltage_V', 0)
                                    check_v_ok = VOLTAGE_MIN <= check_v <= VOLTAGE_MAX
                                    check_c1 = check_data.get('curr1_mA', 0)
                                    check_c2 = check_data.get('curr2_mA', 0)
                                    check_c3 = check_data.get('curr3_mA', 0)
                                    check_c4 = check_data.get('curr4_mA', 0)
                                    check_lamps_ok = (
                                        CURRENT_MIN <= check_c1 <= CURRENT_MAX and
                                        CURRENT_MIN <= check_c2 <= CURRENT_MAX and
                                        CURRENT_MIN <= check_c3 <= CURRENT_MAX and
                                        CURRENT_MIN <= check_c4 <= CURRENT_MAX
                                    )
                                    if not check_v_ok or not check_lamps_ok:
                                        any_faults = True
                                        break
                            
                            if not any_faults:
                                self.limpiar_alerta()
                    else:
                        # No data available for this device
                        self.leds_v[idx][0].itemconfig(self.leds_v[idx][1], fill="#555555")
                        for lamp_i in range(4):
                            self.uv_lamps[idx][lamp_i][0].itemconfig(self.uv_lamps[idx][lamp_i][1], fill="#555555")
                        self.lbls_v_val[idx].config(text="--- V", fg="#ff4444")
        
        except Exception as e:
            print(f"HMI update error: {e}")
        
        # Schedule next update
        self.root.after(1000, self.actualizar_datos_dispositivos)

    def mostrar_imagen_layout(self):
        if not self.img_layout_full:
            messagebox.showwarning("Error", "No se encontró el mapa WALL-E HMI images/Bimbo.png")
            return
        
        # Create a new window for the map
        top = tk.Toplevel(self.root)
        top.title("Mapa de Planta - Wall-E")
        top.geometry("500x750")
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
        
        tk.Button(frame_btn, text="CERRAR", command=top.destroy, bg="red", fg="white", 
                 font=("Arial", 10, "bold"), width=20).pack(pady=5)

    def mostrar_detalle_lamparas(self, device_id):
        """Open a window showing per-lamp UV status for a device"""
        top = tk.Toplevel(self.root)
        top.title(f"Detalle Lámparas UV - W-{device_id}")
        top.geometry("420x320")
        top.configure(bg="#1a1a1a")

        tk.Label(top, text=f"W-{device_id} - Estado de Lámparas UV",
                 font=("Arial", 12, "bold"), bg="#1a1a1a", fg="#00ff00").pack(pady=10)

        frame = tk.Frame(top, bg="#1a1a1a")
        frame.pack(expand=True, fill="both", padx=10, pady=10)

        with device_data_lock:
            data = shared_device_data.get(device_id)

        if not data:
            tk.Label(frame, text="Sin datos del nodo.", bg="#1a1a1a", fg="white").pack(pady=10)
        else:
            currents = [
                data.get('curr1_mA', 0),
                data.get('curr2_mA', 0),
                data.get('curr3_mA', 0),
                data.get('curr4_mA', 0),
            ]

            for idx, curr in enumerate(currents, start=1):
                ok = CURRENT_MIN <= curr <= CURRENT_MAX
                color = "#2ecc71" if ok else "#e74c3c"
                status = "OK" if ok else "FALLA"

                row = tk.Frame(frame, bg="#1a1a1a")
                row.pack(fill="x", pady=4)

                lamp_canvas = tk.Canvas(row, width=20, height=20, bg="#1a1a1a", highlightthickness=0)
                lamp_canvas.pack(side="left", padx=8)
                lamp_canvas.create_oval(3, 3, 17, 17, fill=color, outline="white")

                tk.Label(row, text=f"Lámpara {idx}: {curr:.0f} mA",
                         font=("Arial", 10), bg="#1a1a1a", fg="white").pack(side="left")
                tk.Label(row, text=status, font=("Arial", 10, "bold"), bg="#1a1a1a", fg=color).pack(side="right")

        tk.Button(top, text="CERRAR", command=top.destroy, bg="red", fg="white",
                  font=("Arial", 10, "bold"), width=20).pack(pady=10)

    def activar_alerta(self, msg):
        if not self.falla_activa:  # Only log once when fault first detected
            self.falla_activa = True
            self.registrar_log(msg)
        
        # Visual/audio alerts
        try:
            if self.sonido_habil: 
                GPIO.output(PIN_BUZZER, GPIO.HIGH)
        except:
            pass

    def limpiar_alerta(self):
        self.falla_activa = False
        try:
            GPIO.output(PIN_BUZZER, GPIO.LOW)
        except:
            pass

    def silenciar(self):
        self.sonido_habil = False
        try:
            GPIO.output(PIN_BUZZER, GPIO.LOW)
        except:
            pass

    def reset(self):
        self.sonido_habil = True
        if not self.falla_activa: 
            self.limpiar_alerta()

    def registrar_log(self, info):
        try:
            with open("log_seguridad.csv", "a") as f:
                f.write(f"{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}, {info}\n")
        except Exception as e:
            print(f"Log write error: {e}")

    def abrir_historial(self):
        # Create a new window for logs
        pop = tk.Toplevel(self.root)
        pop.title("Historial de Eventos - Wall-E")
        pop.geometry("500x600")
        pop.resizable(True, True)
        
        # Frame for title
        frame_title = tk.Frame(pop, bg="#1a1a1a", height=40)
        frame_title.pack(fill="x")
        tk.Label(frame_title, text="ÚLTIMOS EVENTOS", font=("Arial", 12, "bold"), 
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
            txt.insert("1.0", "No hay registros previos.")
        
        txt.config(state="disabled")  # Make it read-only
        
        # Frame for buttons at bottom
        frame_btn = tk.Frame(pop, bg="#1a1a1a", height=50)
        frame_btn.pack(fill="x", padx=5, pady=5)
        
        tk.Button(frame_btn, text="CERRAR", command=pop.destroy, bg="red", fg="white",
                 font=("Arial", 10, "bold"), width=20).pack(side="left", padx=5)
        tk.Button(frame_btn, text="LIMPIAR LOGS", command=self.limpiar_logs, bg="orange", fg="white",
                 font=("Arial", 10, "bold"), width=20).pack(side="left", padx=5)
    
    def limpiar_logs(self):
        """Clear the security log file"""
        try:
            with open("log_seguridad.csv", "w") as f:
                f.write("Logs limpiados el {}\n".format(datetime.now().strftime("%Y-%m-%d %H:%M:%S")))
            messagebox.showinfo("Éxito", "Logs limpiados correctamente")
        except Exception as e:
            messagebox.showerror("Error", f"Error al limpiar logs: {e}")

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
                
                # Query all devices
                for device_id in range(1, NUM_DEVICES + 1):
                    response = query_device(device_id)
                    
                    if response:
                        # Update shared data structure (thread-safe)
                        with device_data_lock:
                            shared_device_data[device_id] = response
                    
                    time.sleep(0.5)  # Delay between requests
                
                # Print device statistics
                print(f"\nDevice Statistics:")
                for device_id in range(1, NUM_DEVICES + 1):
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
    global coordinator_running, coordinator_thread
    
    print("="*70)
    if DEMO_MODE:
        print("          Wall-E Coordinator with HMI - DEMO MODE")
    else:
        print("          Wall-E Coordinator with HMI - Starting Up")
    print("="*70)
    print(f"Configuration:")
    print(f"  - Number of devices: {NUM_DEVICES}")
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
    
    # Start coordinator thread
    coordinator_running = True
    coordinator_thread = threading.Thread(target=coordinator_loop, daemon=True)
    coordinator_thread.start()
    
    # Launch HMI (runs in main thread)
    try:
        root = tk.Tk()
        app = AppIndustrial(root)
        
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
                spi.close()
                GPIO.cleanup()
                print("✓ Resources cleaned up")
            except Exception as e:
                print(f"⚠ Cleanup warning: {e}")
        
        print("✓ Exited successfully\n")
    
    return 0

if __name__ == "__main__":
    main()
