# Wall-E System - Communication Protocol & Setup Guide

## System Overview

Wall-E is a distributed UV lamp monitoring system with:
- **1 Central Coordinator** (Raspberry Pi 4)
- **Up to 9 Remote Nodes** (ESP32 with LoRa modules)
- **433 MHz LoRa** point-to-point communication

## Architecture Diagram

```
┌──────────────────┐
│  Raspberry Pi 4  │  Coordinator
│  (Receiver/Hub)  │  - Queries all devices
└────────┬─────────┘  - Collects responses
         │ LoRa       - Logs data
    433 MHz           - Sends alerts
         │
    ┌────┴────────────────────────────────────────┐
    │                                             │
    ▼                ▼                 ▼          ▼
┌────────┐    ┌────────┐       ┌────────┐  ...
│ESP32-1 │    │ESP32-2 │       │ESP32-9 │
│Node ID 1│   │Node ID 2│      │Node ID 9│
└────────┘    └────────┘       └────────┘
```

## Communication Protocol

### Message Format

#### Request Packet (Coordinator → Node)
**Size: 4 bytes**

```
Byte 0: NET_ID (0xA5)
Byte 1: MSG_TYPE = 0x10 (REQUEST)
Byte 2: TARGET_ID (1-9, which device to query)
Byte 3: REQ_CODE = 0x01 (READ_SENSOR_DATA)
```

**Example**: Query device #3 for sensor data
```
0xA5 0x10 0x03 0x01
```

#### Response Packet (Node → Coordinator)
**Size: 8 bytes**

```
Byte 0: NET_ID (0xA5)
Byte 1: MSG_TYPE = 0x90 (RESPONSE)
Byte 2: DEVICE_ID (1-9, sender device)
Byte 3: SEQ_LO (sequence number low byte)
Byte 4: SEQ_HI (sequence number high byte)
Byte 5: AC_STATUS (0=OK, 1=ALERT)
Byte 6: UV1_STATUS (0=OK, 1=ALERT)
Byte 7: UV2_STATUS (0=OK, 1=ALERT)
```

**Example**: Device #3 responding with all sensors OK, sequence #42
```
0xA5 0x90 0x03 0x2A 0x00 0x00 0x00 0x00
              seq=42^  ^AC  ^UV1  ^UV2
```

## Status Codes

| Code | Meaning | Condition |
|------|---------|-----------|
| 0 | OK | Sensor value ≥ threshold (lamp ON, power present) |
| 1 | ALERT | Sensor value < threshold (lamp OFF, no power) |

## LoRa Physical Layer Configuration

| Parameter | Value | Notes |
|-----------|-------|-------|
| Frequency | 433 MHz | 433E6 Hz |
| Spreading Factor | 7 | Moderate range (3-5 km line-of-sight) |
| Bandwidth | 125 kHz | Standard, good efficiency |
| Coding Rate | 4/5 | 5-8 bits per symbol |
| Sync Word | 0x21 | Private network identifier |
| CRC | Enabled | Error detection |

## Configuration Instructions

### For Each ESP32 Node

1. **Edit TX_ID** (Line ~79 in .ino file):
   ```cpp
   #define TX_ID 1  // Change to 1, 2, 3, ..., 9 for each device
   ```

2. **Calibrate Thresholds** (Lines ~129-133):
   ```cpp
   int UV_THRESHOLD = 2000;    // Adjust based on your sensor
   int AC_THRESHOLD = 2000;    // Adjust based on your ZMPT101B circuit
   ```

3. **Calibration Procedure**:
   - Upload code with VERY HIGH thresholds (3500)
   - Open Serial Monitor (Tools → Serial Monitor, 115200 baud)
   - Note the "Raw:" values when lamps are ON
   - Set threshold to ~500 below the normal-operation value
   - Re-upload and test

### For Raspberry Pi Coordinator

1. **Edit NUM_DEVICES** (coordinator script, line ~136):
   ```python
   NUM_DEVICES = 9  # Number of nodes you actually have
   ```

2. **Edit QUERY_INTERVAL** (coordinator script, line ~137):
   ```python
   QUERY_INTERVAL = 5.0  # Seconds between polling cycles
   ```

3. **Run the coordinator**:
   ```bash
   sudo python3 Wall_E_coordinator_raspberry.py
   ```

## Pin Connections

### ESP32 LoRa Module Connections

| ESP32 Pin | LoRa Pin | Function |
|-----------|----------|----------|
| GPIO 18 | NSS | SPI Chip Select |
| GPIO 14 | RST | Reset (active LOW) |
| GPIO 26 | DIO0 | Interrupt/RX Done |
| GPIO 19 | MOSI | SPI Data In |
| GPIO 27 | MISO | SPI Data Out |
| GPIO 5 | CLK | SPI Clock |
| 3.3V | VCC | Power |
| GND | GND | Ground |

### Sensor Connections

| ESP32 Pin | Sensor | Function |
|-----------|--------|----------|
| GPIO 34 | UV Sensor 1 | ADC1_CH6 |
| GPIO 35 | UV Sensor 2 | ADC1_CH7 |
| GPIO 32 | AC Monitor | ADC1_CH4 |

### Raspberry Pi LoRa Module Connections

| RPi GPIO | LoRa Pin | Function |
|----------|----------|----------|
| GPIO 8 | NSS | SPI Chip Select |
| GPIO 22 | RST | Reset (active LOW) |
| GPIO 4 | DIO0 | Interrupt/RX Done |
| GPIO 10 | MOSI | SPI Data In |
| GPIO 9 | MISO | SPI Data Out |
| GPIO 11 | CLK | SPI Clock |
| 3.3V | VCC | Power |
| GND | GND | Ground |

## Operation Flow

### Coordinator (Raspberry Pi)

1. **Initialization**:
   - Setup GPIO and SPI
   - Reset LoRa module
   - Configure LoRa parameters (433 MHz, SF7, BW125k)
   - Enter RX mode

2. **Query Cycle**:
   - For each device (1-9):
     a. Build request packet
     b. Switch to TX mode
     c. Send request packet
     d. Wait for TX done
     e. Switch to RX mode
     f. Wait for response (up to 2 seconds)
     g. Parse and log response
   - Wait 5 seconds
   - Repeat

3. **Monitoring**:
   - Tracks response/failure ratio per device
   - Logs all alerts
   - Identifies communication failures

### Remote Node (ESP32)

1. **Initialization**:
   - Setup GPIO, ADC, and SPI
   - Reset LoRa module
   - Configure LoRa parameters (433 MHz, SF7, BW125k)
   - Enter RX mode

2. **Listen Loop**:
   - Continuously read sensor values
   - Poll for incoming requests
   - When request arrives:
     a. Validate packet (NET_ID, MSG_TYPE, TARGET_ID)
     b. Read sensor values
     c. Compare against thresholds
     d. Build response packet
     e. Send response
     f. Return to RX mode
   - 50ms delay, then repeat

## Troubleshooting

### ESP32 Won't Respond

1. **Check LoRa Module**:
   ```
   - Verify 3.3V power (check with multimeter)
   - Verify SPI connections (use continuity tester)
   - Try different SPI speed (reduce to 1MHz for testing)
   ```

2. **Check TX_ID**:
   - Ensure TX_ID matches what coordinator sends
   - Each device must have unique TX_ID 1-9

3. **Serial Monitor Debug**:
   - Open Serial Monitor at 115200 baud
   - Should see "LoRa initialized" message
   - Try changing UV_THRESHOLD to very low value (100) to force ALERT status

### Coordinator Can't Receive

1. **Check RX Mode**:
   - Verify LoRa is in RX mode (0x85 = 10000101 binary)
   - Check IRQ flags register (0x12)

2. **Check Same Frequency**:
   - Both devices must use exact same frequency (433E6)
   - Both must use same SF (7), BW (125k), CR (4/5)

3. **Test SPI Communication**:
   ```bash
   sudo python3 << 'EOF'
   import spidev
   spi = spidev.SpiDev()
   spi.open(0, 0)
   resp = spi.xfer2([0x42, 0x00])  # Read VERSION register
   print(f"LoRa version: 0x{resp[1]:02x}")
   spi.close()
   EOF
   ```
   Expected: `0x12` for SX1276/SX1278

### Communication Latency

- **Normal**: 100-500 ms per device
- **If >2 seconds**: Check for interference or antenna placement
- **Solution**: Increase RESPONSE_TIMEOUT in coordinator

## Performance Metrics

| Metric | Value |
|--------|-------|
| Packet Size (Request) | 4 bytes |
| Packet Size (Response) | 8 bytes |
| Airtime per packet (SF7, BW125k) | ~60-120 ms |
| Query time per device | ~200-500 ms |
| Full cycle (9 devices) | ~2-5 seconds |
| Typical range | 3-5 km (line-of-sight) |

## Network Privacy

- **NET_ID (0xA5)**: Identifies your private network
  - Other networks/applications won't receive your packets
  - Reduces noise/interference

- **Sync Word (0x21)**: Additional privacy layer
  - LoRa module will ignore packets with different sync words

## Next Steps

1. Flash each ESP32 with correct TX_ID (1-9)
2. Power up devices and verify LED indicators
3. Run coordinator on Raspberry Pi
4. Monitor Serial output and logs
5. Calibrate thresholds based on actual sensor values
6. Deploy in production

## Support

For issues:
1. Check Serial Monitor output on ESP32
2. Monitor coordinator logs on Raspberry Pi
3. Verify all connections with continuity tester
4. Test each device individually first
5. Increase verbosity in debug output
