#!/bin/bash
# Script de instalación automática para Raspberry Pi 3
# Solo ejecuta esto en la Pi 3 después de copiar el proyecto

set -e  # Detener si hay error

echo "=== Instalando Wall-E en Raspberry Pi 3 ==="

# 1. Instalar dependencias del sistema
echo "Paso 1: Instalando dependencias..."
sudo apt-get update
sudo apt-get install -y python3 python3-venv python3-pip python3-tk

# 2. Crear ambiente virtual e instalar paquetes Python
echo "Paso 2: Configurando ambiente Python..."
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install spidev RPi.GPIO Pillow

# 3. Crear servicio de autostart
echo "Paso 3: Configurando autostart..."
sudo tee /etc/systemd/system/wall-e-coordinator.service > /dev/null <<EOF
[Unit]
Description=Wall-E Coordinator
After=network.target

[Service]
Type=simple
User=pi
WorkingDirectory=$(pwd)
ExecStart=$(pwd)/.venv/bin/python Wall_E_coordinator_raspberry.py
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
EOF

# 4. Activar el servicio
sudo systemctl daemon-reload
sudo systemctl enable wall-e-coordinator.service

echo ""
echo "✅ ¡Instalación completa!"
echo ""
echo "Para iniciar ahora:"
echo "  sudo systemctl start wall-e-coordinator.service"
echo ""
echo "Para ver logs:"
echo "  sudo journalctl -u wall-e-coordinator.service -f"
echo ""
echo "El programa se ejecutará automáticamente al reiniciar."
