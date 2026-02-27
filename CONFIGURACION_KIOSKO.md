# Configuración de Usuario Kiosko para Wall-E

## Usuario Creado
- **Nombre:** walle_kiosko
- **Contraseña:** Walle1234
- **Privilegios:** Usuario normal (SIN sudo)

## Seguridad Implementada

### 1. Restricción de Acceso a Archivos
- Los directorios `/home/rasp_raccoon_berry` y `/home/rasp_raccoon_berry/Documents` ahora tienen permisos 700 (solo el propietario puede acceder)
- El usuario walle_kiosko NO puede ver ni acceder a tus archivos personales

### 2. Inicio Automático del Programa
- **Archivo:** `/home/walle_kiosko/.xinitrc`
- **Función:** Inicia automáticamente Wall_E_coordinator_raspberry.py al iniciar sesión
- **Sesión:** Openbox (entorno minimalista sin barra de tareas)

### 3. Bloqueos de Seguridad
- Sin permisos de sudo
- Sin acceso SSH
- .bashrc bloqueado para forzar solo ejecución del programa
- Openbox configurado para pantalla completa sin decoraciones

### 4. Acceso a Hardware
- Grupos: gpio, spi, i2c, input (necesarios para LoRa y sensores)

## Cómo Usar

### Iniciar Sesión
1. Reinicia la Raspberry Pi
2. En la pantalla de login, selecciona el usuario **walle_kiosko**
3. Asegúrate de seleccionar la sesión **"Openbox"** (ícono de engranaje en la esquina)
4. Ingresa la contraseña: **Walle1234**
5. El programa Wall-E debería iniciarse automáticamente en pantalla completa

### Cerrar el Programa
- Si el programa no tiene botón de salida, puedes:
  - Presionar Ctrl+Alt+F1 para ir a una terminal virtual
  - Iniciar sesión con tu usuario (rasp_raccoon_berry)
  - Ejecutar: `sudo pkill -u walle_kiosko`
  - Regresar a la interfaz gráfica con Ctrl+Alt+F7

### Solución de Problemas

#### El programa no se inicia automáticamente
1. Verifica que la sesión sea "Openbox" (no LXDE ni otro escritorio)
2. Revisa el log de errores:
   ```bash
   sudo cat /home/walle_kiosko/.xsession-errors
   ```

#### Error de permisos GPIO/SPI
```bash
sudo usermod -aG gpio,spi,i2c walle_kiosko
```

#### Restablecer configuración
Si necesitas volver a empezar:
```bash
sudo deluser --remove-home walle_kiosko
```

## Archivos de Configuración

### `/home/walle_kiosko/.xinitrc`
Script de inicio que ejecuta el programa automáticamente

### `/home/walle_kiosko/.xsession`
Fuerza el uso de .xinitrc

### `/home/walle_kiosko/.dmrc`
Define Openbox como sesión por defecto

### `/home/walle_kiosko/.config/openbox/rc.xml`
Configuración de Openbox (pantalla completa, sin decoraciones)

### `/home/walle_kiosko/.bashrc`
Bloquea acceso por terminal y fuerza ejecución del programa

## Notas Importantes

- El usuario walle_kiosko tiene su propia copia del proyecto en `/home/walle_kiosko/Wall_E_UV-and-Voltage-sense/`
- Incluye su propio entorno virtual Python con todas las dependencias
- Los cambios en tu proyecto original NO se reflejan automáticamente en la copia de walle_kiosko
- Para actualizar el proyecto de walle_kiosko:
  ```bash
  sudo cp -r ~/Documents/Wall_E_UV-and-Voltage-sense/* /home/walle_kiosko/Wall_E_UV-and-Voltage-sense/
  sudo chown -R walle_kiosko:walle_kiosko /home/walle_kiosko/Wall_E_UV-and-Voltage-sense/
  ```
