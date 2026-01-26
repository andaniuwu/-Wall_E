# Raspberry Pi LoRa Receiver and Transmitter for Wall-E System
# Uses SX127x library for LoRa communication
# Install: pip install SX127x

import tkinter as tk
from tkinter import messagebox
import RPi.GPIO as GPIO
import threading
import time
import csv
from datetime import datetime
from SX127x.LoRa import *
from SX127x.board_config import BOARD

# =======================================================
# 1. CONFIGURACIÓN DE HARDWARE (PINES GPIO)
# =======================================================
PIN_VERDE = 17    # Relevador Luz Verde
PIN_ROJO = 27     # Relevador Luz Roja
PIN_BUZZER = 22   # Relevador Sirena/Buzzer

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup([PIN_VERDE, PIN_ROJO, PIN_BUZZER], GPIO.OUT, initial=GPIO.LOW)















# =======================================================
# 2. LÓGICA DE COMUNICACIÓN LORA (433 MHz)
# =======================================================
# Protocol constants (match ESP32)
NET_ID = 0xA5
MSG_REQ = 0x10
MSG_RESP = 0x90

class LoRaController(LoRa):
    def __init__(self, callback=None):
        super(LoRaController, self).__init__(verbose=False)
        self.set_mode(MODE.SLEEP)
        self.set_pa_config(pa_select=1)  # PA_BOOST for higher power
        self.set_freq(433.0)  # Frequency in MHz
        self.set_spreading_factor(7)  # Match ESP32
        self.set_signal_bandwidth(125000)  # 125 kHz
        self.set_coding_rate(5)  # 4/5
        self.set_sync_word(0x21)  # Match ESP32 sync word
        self.enable_crc()  # Enable CRC
        self.callback = callback

    # THIS METHOD IS CALLED AUTOMATICALLY BY THE LIBRARY WHEN A PACKET IS RECEIVED
    def on_rx_done(self):
        # Clear the receive done interrupt flag
        self.clear_irq_flags(RxDone=1)
        # Read the raw payload as a list of bytes
        payload = self.read_payload(nocheck=True)  # e.g., [165, 144, 1, 0, 0, 0, 0, 0]
        # Call the callback function with the payload
        if self.callback:
            self.callback(payload)
        # Set back to continuous receive mode to listen for more packets
        self.set_mode(MODE.RXCONT)

    def send_request(self, tx_id):
        # Switch to standby mode to prepare for transmission
        self.set_mode(MODE.STDBY)
        # Start building the packet
        self.begin_packet()
        # Write the protocol bytes: NET_ID, MSG_REQ, TX_ID, Request code (1)
        self.write(NET_ID)  # Network ID
        self.write(MSG_REQ)  # Message type: request
        self.write(tx_id)    # Target TX ID
        self.write(1)        # Request code
        # End and transmit the packet
        self.end_packet()
        # Switch back to receive mode
        self.set_mode(MODE.RXCONT)












# =======================================================
# 3. INTERFAZ GRÁFICA Y LÓGICA DE CONTROL
# =======================================================
class AppIndustrial:
    def __init__(self, root):
        self.root = root
        self.root.title("Monitor Industrial UV LoRa")
        self.root.geometry("600x650")
        self.root.configure(bg="#121212") # Fondo oscuro profesional
        
        self.sonido_habil = True  # Control del botón de silencio
        self.falla_activa = False # Estado actual de seguridad
        self.current_tx = 1  # Current TX to poll
        self.last_poll_time = time.time()
        self.tx_data = {i: {'ac': 0, 'uv1': 0, 'uv2': 0} for i in range(1, 10)}  # Store data for each TX

        # --- Encabezado y Reloj ---
        self.lbl_reloj = tk.Label(root, text="", font=("Arial", 14), fg="gray", bg="#121212")
        self.lbl_reloj.pack(anchor="ne", padx=20, pady=10)
        self.actualizar_hora()

        tk.Label(root, text="SISTEMA DE MONITOREO UV", font=("Arial", 18, "bold"), fg="white", bg="#121212").pack()

        # --- Panel de LEDs Virtuales ---
        self.frame_leds = tk.Frame(root, bg="#121212")
        self.frame_leds.pack(pady=30)

        # LED de Energía (Voltaje) - Aggregate or per TX?
        self.canvas_v = tk.Canvas(self.frame_leds, width=180, height=200, bg="#121212", highlightthickness=0)
        self.canvas_v.grid(row=0, column=0, padx=20)
        self.led_v = self.canvas_v.create_oval(25, 25, 155, 155, fill="#333333", outline="white")
        tk.Label(self.frame_leds, text="ENERGÍA", font=("Arial", 12, "bold"), fg="white", bg="#121212").grid(row=1, column=0)
        self.lbl_v_num = tk.Label(self.frame_leds, text="---", font=("Arial", 11), fg="#f1c40f", bg="#121212")
        self.lbl_v_num.grid(row=2, column=0)

        # LED de Lámparas (UV)
        self.canvas_uv = tk.Canvas(self.frame_leds, width=180, height=200, bg="#121212", highlightthickness=0)
        self.canvas_uv.grid(row=0, column=1, padx=20)
        self.led_uv = self.canvas_uv.create_oval(25, 25, 155, 155, fill="#333333", outline="white")
        tk.Label(self.frame_leds, text="LÁMPARAS UV", font=("Arial", 12, "bold"), fg="white", bg="#121212").grid(row=1, column=1)
        tk.Label(self.frame_leds, text="S1 & S2", font=("Arial", 11), fg="gray", bg="#121212").grid(row=2, column=1)

        # --- Botonera ---
        btn_estilo = {"font": ("Arial", 12, "bold"), "width": 20, "height": 2, "fg": "white"}
        
        tk.Button(root, text="SILENCIAR ALARMA", command=self.silenciar, bg="#e67e22", **btn_estilo).pack(pady=5)
        tk.Button(root, text="RESET SISTEMA", command=self.reset, bg="#27ae60", **btn_estilo).pack(pady=5)
        tk.Button(root, text="HISTORIAL DE FALLAS", command=self.abrir_historial, bg="#34495e", **btn_estilo).pack(pady=15)











#//////////////////////////////////////////////////////////////////////////////////////////////////////////
        # --- Inicialización LoRa ---
        BOARD.setup()  # Initialize SPI and GPIO for LoRa
        self.lora = LoRaController(callback=self.procesar_recepcion)  # Create LoRa instance with callback
        self.lora.set_mode(MODE.RXCONT)  # Set to continuous receive mode to listen for packets
        
        # Hilo para polling TX (sends requests every 30s)
        threading.Thread(target=self.polling_loop, daemon=True).start()

    def actualizar_hora(self):
        ahora = datetime.now().strftime("%H:%M:%S")
        self.lbl_reloj.config(text=ahora)
        self.root.after(1000, self.actualizar_hora)

    def procesar_recepcion(self, payload):
        # Payload: [NET_ID, MSG_RESP, TX_ID, SEQ_L, SEQ_H, AC, UV1, UV2]
        # Example: [165, 144, 1, 0, 0, 0, 0, 0] for TX1, all OK
        if len(payload) >= 8 and payload[0] == NET_ID and payload[1] == MSG_RESP:
            tx_id = payload[2]  # Which TX responded
            ac = payload[5]     # AC status: 0=OK, 1=Alert
            uv1 = payload[6]    # UV1 status: 0=OK, 1=Alert
            uv2 = payload[7]    # UV2 status: 0=OK, 1=Alert
            self.tx_data[tx_id] = {'ac': ac, 'uv1': uv1, 'uv2': uv2}
            print(f"Received from TX{tx_id}: AC={ac}, UV1={uv1}, UV2={uv2}")
            self.update_gui()  # Update the GUI LEDs based on data

    def update_gui(self):
        # Aggregate status: Alert if any TX has issue
        ac_ok = all(d['ac'] == 0 for d in self.tx_data.values())
        uv_ok = all(d['uv1'] == 0 and d['uv2'] == 0 for d in self.tx_data.values())
        
        self.canvas_v.itemconfig(self.led_v, fill="#2ecc71" if ac_ok else "#e74c3c")
        self.canvas_uv.itemconfig(self.led_uv, fill="#2ecc71" if uv_ok else "#e74c3c")
        self.lbl_v_num.config(text=f"TX{self.current_tx}")

        if not ac_ok or not uv_ok:
            motivo = "Falla en uno o más TX"
            self.activar_alerta(motivo)
        else:
            self.limpiar_alerta()

    def polling_loop(self):
        while True:
            if time.time() - self.last_poll_time >= 30:  # Every 30 seconds
                self.lora.send_request(self.current_tx)  # Send request to current TX
                self.last_poll_time = time.time()
                self.current_tx = (self.current_tx % 9) + 1  # Cycle to next TX (1-9)
            time.sleep(1)  # Check every second

    def activar_alerta(self, msg):
        self.falla_activa = True
        GPIO.output(PIN_ROJO, GPIO.HIGH)
        GPIO.output(PIN_VERDE, GPIO.LOW)
        if self.sonido_habil:
            GPIO.output(PIN_BUZZER, GPIO.HIGH)
        self.registrar_log(msg)

    def limpiar_alerta(self):
        self.falla_activa = False
        GPIO.output(PIN_ROJO, GPIO.LOW)
        GPIO.output(PIN_VERDE, GPIO.HIGH)
        GPIO.output(PIN_BUZZER, GPIO.LOW)

    def silenciar(self):
        self.sonido_habil = False
        GPIO.output(PIN_BUZZER, GPIO.LOW)

    def reset(self):
        self.sonido_habil = True
        if not self.falla_activa:
            self.limpiar_alerta()

    def registrar_log(self, info):
        with open("log_seguridad.csv", "a") as f:
            f.write(f"{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}, {info}\n")

    def abrir_historial(self):
        popup = tk.Toplevel(self.root)
        popup.title("Histórico de Fallas")
        popup.geometry("500x350")
        txt = tk.Text(popup, bg="#f5f5f5", font=("Courier", 10))
        txt.pack(expand=True, fill="both")
        try:
            with open("log_seguridad.csv", "r") as f:
                logs = f.readlines()[-30:]
                txt.insert("1.0", "".join(reversed(logs)))
        except FileNotFoundError:
            txt.insert("1.0", "Historial vacío.")















# =======================================================
# 4. INICIO DEL PROGRAMA
# =======================================================
if __name__ == "__main__":
    root = tk.Tk()
    app = AppIndustrial(root)
    try:
        root.mainloop()
    finally:
        GPIO.cleanup()
        BOARD.teardown()







''' 
🧠 Visual Explanation — FULL EXECUTION LIFECYCLE
STEP 1 — Start program
↓
STEP 2 — Build GUI (widgets, labels, buttons, LEDs)
↓
STEP 3 — Initialize LoRa hardware (SPI, GPIO)
↓
STEP 4 — Start the DAEMON THREAD (polling_loop)
This loop runs independently:
while True:
    every 30 sec send request

↓
STEP 5 — Enter Tkinter main loop
root.mainloop()   <-- YOUR main loop like while(1)

↓
STEP 6 — If LoRa receives data → on_rx_done() → procesar_recepcion()
No polling needed, it is automatic.
↓
STEP 7 — GUI updates itself based on reception
LEDs change, logs are written, relays activated.
↓
STEP 8 — When window closes

Tkinter main loop ends
Daemon thread stops
GPIO cleanup runs
Program ends cleanly



'''