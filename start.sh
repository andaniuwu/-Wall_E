#!/bin/bash
# Quick start script for Wall-E UV & Voltage Monitor
# Starts the coordinator with real hardware mode

echo "========================================"
echo "   Wall-E Coordinator + HMI Startup"
echo "========================================"
echo ""
echo "Mode: REAL HARDWARE (ESP32 + LoRa)"
echo "Expected: Signals from ESP32 devices"
echo ""
echo "Press Ctrl+C to stop"
echo "========================================"
echo ""

cd /home/rasp_raccoon_berry/Documents/Wall_E_UV-and-Voltage-sense

/home/rasp_raccoon_berry/Documents/Wall_E_UV-and-Voltage-sense/.venv/bin/python \
Wall_E_coordinator_raspberry.py

echo ""
echo "========================================"
echo "   Coordinator Stopped"
echo "========================================"
