import tkinter as tk
from tkinter import messagebox
import RPi.GPIO as GPIO
import threading
import time
from datetime import datetime
from PIL import Image, ImageTk

# =======================================================
# 1. CONFIGURACIÓN DE HARDWARE (PINES GPIO)
# =======================================================
PIN_VERDE = 17    
PIN_ROJO = 27     
PIN_BUZZER = 22   

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup([PIN_VERDE, PIN_ROJO, PIN_BUZZER], GPIO.OUT, initial=GPIO.LOW)

# =======================================================
# 2. INTERFAZ GRÁFICA PROFESIONAL (480x800)
# =======================================================
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

        # --- PANEL DE ROBOTS (DISTRIBUCIÓN 3-3-1) ---
        self.container = tk.Frame(self.root, bg="#483698")
        self.container.pack(expand=True, fill="both", padx=5)

        self.leds_v, self.leds_uv, self.lbls_v_val, self.frames_robot = [], [], [], []

        for i in range(7):
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

            # LED Lámparas
            cuv = tk.Canvas(f, width=50, height=50, bg="#3a2a7a", highlightthickness=0)
            cuv.pack()
            circ_uv = cuv.create_oval(8, 8, 42, 42, fill="#555555", outline="white")
            self.leds_uv.append((cuv, circ_uv))
            tk.Label(f, text="UV", font=("Arial", 7), bg="#3a2a7a", fg="#aaaaaa").pack()

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
        except: self.img_layout_full = None

    # =======================================================
    # MÉTODOS DE LÓGICA Y CONTROL
    # =======================================================
    def actualizar_hora(self):
        self.lbl_reloj.config(text=datetime.now().strftime("%H:%M:%S"))
        self.root.after(1000, self.actualizar_hora)

    def mostrar_imagen_layout(self):
        if not self.img_layout_full:
            messagebox.showwarning("Error", "No se encontró el mapa WALL-E HMI images/Bimbo.png")
            return
        top = tk.Toplevel(self.root)
        top.geometry("480x700")
        top.configure(bg="#222222")
        tk.Label(top, image=self.img_layout_full, bg="#222222").pack(pady=10)
        tk.Button(top, text="CERRAR", command=top.destroy, bg="red", fg="white", font=("Arial", 10, "bold")).pack(pady=5)

    def procesar_recepcion(self, mensaje):
        """Process incoming sensor data from devices"""
        try:
            datos = mensaje.split(",")
            idx = int(datos[0]) - 1
            if 0 <= idx < 7:
                v, u1, u2 = float(datos[1]), float(datos[2]), float(datos[3])
                v_ok = v >= 11.0
                uv_ok = (u1 >= 20 and u2 >= 20)
                
                # Actualizar indicadores visuales
                color_v = "#2ecc71" if v_ok else "#e74c3c"
                color_uv = "#2ecc71" if uv_ok else "#e74c3c"
                
                self.leds_v[idx][0].itemconfig(self.leds_v[idx][1], fill=color_v)
                self.leds_uv[idx][0].itemconfig(self.leds_uv[idx][1], fill=color_uv)
                self.lbls_v_val[idx].config(text=f"{v} V", fg="white" if v_ok else "#ff4444")

                if not v_ok or not uv_ok:
                    self.activar_alerta(f"FALLA W-{idx+1}")
                else:
                    self.limpiar_alerta()
        except: pass

    def activar_alerta(self, msg):
        self.falla_activa = True
        GPIO.output(PIN_ROJO, GPIO.HIGH)
        GPIO.output(PIN_VERDE, GPIO.LOW)
        if self.sonido_habil: GPIO.output(PIN_BUZZER, GPIO.HIGH)
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
        if not self.falla_activa: self.limpiar_alerta()

    def registrar_log(self, info):
        with open("log_seguridad.csv", "a") as f:
            f.write(f"{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}, {info}\n")

    def abrir_historial(self):
        pop = tk.Toplevel(self.root)
        pop.geometry("400x500")
        txt = tk.Text(pop, font=("Courier", 10), bg="#111111", fg="#00ff00")
        txt.pack(expand=True, fill="both")
        try:
            with open("log_seguridad.csv", "r") as f:
                logs = f.readlines()[-25:]
                txt.insert("1.0", "".join(reversed(logs)))
        except: txt.insert("1.0", "No hay registros previos.")

# =======================================================
# 3. LANZAMIENTO
# =======================================================
if __name__ == "__main__":
    root = tk.Tk()
    app = AppIndustrial(root)
    try:
        root.mainloop()
    finally:
        GPIO.cleanup()