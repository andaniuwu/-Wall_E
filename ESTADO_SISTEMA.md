# Estado del Sistema - Wall-E UV & Voltage Monitor

**Fecha:** 9 de Febrero 2026  
**Modo:** 🔴 PRUEBAS REALES (Hardware Real)

---

## ✅ Sistema Listo para Recibir Señales del ESP32

El sistema ha sido configurado para **pruebas reales** con hardware:

### Configuración Actual

| Parámetro | Valor | Estado |
|-----------|-------|--------|
| **DEMO_MODE** | `False` | ✅ Desactivado |
| **Voltaje Mín** | 100.0 V | ✅ Configurado |
| **Voltaje Máx** | 135.0 V | ✅ Configurado |
| **Corriente Mín** | 500 mA | ✅ Configurado |
| **Corriente Máx** | 1200 mA | ✅ Configurado |
| **Frecuencia LoRa** | 433 MHz | ✅ Configurado |

---

## 🎯 Componentes del Sistema

### Hardware
- **Coordinador:** Raspberry Pi con módulo SX1278 (LoRa)
- **Nodos Remotos:** ESP32 (x9) con sensores UV y voltaje
- **Comunicación:** LoRa 433MHz
- **Display:** GUI Tkinter 480x800px

### Software
- **Lenguaje:** Python 3.13.5
- **Framework GUI:** Tkinter + PIL
- **Threading:** Coordinador (background) + HMI (main thread)
- **Protocolo:** LoRa (requete/respuesta)

---

## 📊 Interfaz de Usuario

### Disposición
```
┌─────────────────────────────────────┐
│      Wall-E UV Voltage Monitor      │
│  9 Nodos (3x3 Grid)                │
│                                    │
│  ┌──────────┐ ┌──────────┐ ...   │
│  │ W-1      │ │ W-2      │ ... │
│  │ ●        │ │ ●        │ ...  │
│  │ ●●●●     │ │ ●●●●     │ ...  │
│  │ DETALLE  │ │ DETALLE  │ ...  │
│  └──────────┘ └──────────┘ ...   │
│                                    │
│  [SILENCIAR] [RESET] [LOGS] [MAPA]│
└─────────────────────────────────────┘
```

### Indicadores por Nodo
- **LED Grande (1):** Voltaje AC (Verde=OK, Rojo=Falla)
- **LEDs Pequeños (4):** Lámparas UV individuales (Verde/Rojo)
- **Valor:** Voltaje en Volts
- **Botón:** DETALLE para ver estado individual de las 4 lámparas

---

## 🔌 Formato de Datos Esperado

El ESP32 debe enviar **cada nodo** con estructura:

```
Nodo ID | AC Voltage | Curr1 | Curr2 | Curr3 | Curr4 | Seq
--------|------------|-------|-------|-------|-------|----
1-9     | 100-135V   | 0-500+ | 0-500+ | 0-500+ | 0-500+ | 0-65535
```

### Evaluación de Thresholds
- **Voltaje OK:** `100V ≤ AC_Voltage ≤ 135V`
- **Lámpara OK:** `500mA ≤ Corriente ≤ 1200mA` (cada una)
- **Nodo OK:** Voltaje OK AND todas las 4 lámparas OK

---

## ⚠️ Estados de Alerta

| Condición | Indicador | Acción |
|-----------|-----------|--------|
| Voltaje < 100V | Rojo | Buzzer + FALLA |
| Voltaje > 135V | Rojo | Buzzer + FALLA |
| Lámpara < 500mA | Rojo | Buzzer + FALLA |
| Lámpara > 1200mA | Rojo | Buzzer + FALLA |

---

## 🚀 Próximos Pasos

1. **Arrancar el coordinador:**
   ```bash
   /home/rasp_raccoon_berry/Documents/Wall_E_UV-and-Voltage-sense/.venv/bin/python \
   Wall_E_coordinator_raspberry.py
   ```

2. **Enviar señales desde ESP32** con formato correcto

3. **Monitorear en GUI:**
   - Voltajes en rango 100-135V
   - Corrientes en rango 500-1200mA (o verde/rojo si fallan)
   - Botón DETALLE para ver detalles de lámparas individuales

---

## 📝 Notas de Implementación

- ✅ Sistema completamente integrado (Coordinador + HMI en un proceso)
- ✅ Thread-safe con locks para datos compartidos
- ✅ Sincronización automática cada 1 segundo en GUI
- ✅ Logs en consola para debugging
- ✅ Ventana de LOGS con historial scrolleable
- ✅ Ventana de MAPA con imagen del layout

---

## 🔧 Archivo de Prueba (COMENTADO)

`test_verify.py` - Desactivado. Para re-activarlo:
```python
# if __name__ == "__main__":
#     ... código aquí
```

---

**Sistema Listo para Producción** ✅
