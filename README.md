# Wall-E LoRa Project

Wireless communication project using LoRa module SX1278/SX1276 between Raspberry Pi 4 and Arduino/ESP32.

## Project Overview

- **Receiver**: Raspberry Pi 4 (4GB RAM) with LoRa module
- **Transmitter**: Arduino/ESP32 with LoRa module  
- **Frequency**: 433 MHz
- **Communication**: Peer-to-peer LoRa wireless

## Files

- `lora_direct_spi.py` - **Main receiver** (Raspberry Pi) - Direct SPI implementation ✓ Working
- `Wall_E_raspberry_receiver_hub_controller_updated.py` - Original receiver with fallback mode
- `Wall_E_program_arduino_ESP32.ino` - Transmitter code (Arduino/ESP32)
- `venv/` - Python virtual environment

## Hardware Setup

### Raspberry Pi 4 LoRa Connection
| GPIO Pin | Function | Physical Pin |
|----------|----------|--------------|
| GPIO 4   | DIO0     | Pin 7        |
| GPIO 17  | DIO1     | Pin 11       |
| GPIO 18  | DIO2     | Pin 12       |
| GPIO 27  | DIO3     | Pin 13       |
| GPIO 22  | RST      | Pin 15       |
| GPIO 10  | MOSI     | Pin 19       |
| GPIO 9   | MISO     | Pin 21       |
| GPIO 11  | CLK      | Pin 23       |
| GPIO 8   | CS/NSS   | Pin 24       |

### Prerequisites
- Raspberry Pi 4 with SPI enabled
- Python 3.10+
- Required packages: `RPi.GPIO`, `spidev`

```bash
sudo raspi-config  # Enable SPI in Interfacing Options
pip install spidev RPi.GPIO
```

## Usage

### Start Receiver (Raspberry Pi)
```bash
sudo python3 lora_direct_spi.py
```

Output:
```
======================================================================
     Wall-E LoRa Receiver - Direct SPI Implementation
======================================================================
✓ LoRa module reset
✓ SPI initialized
✓ LoRa configured:
  - Frequency: 433 MHz
  - Bandwidth: 125 kHz
  - Spreading Factor: 7
  - Coding Rate: 4/5

✓ Listening for LoRa messages on 433 MHz...
```

### Configuration

LoRa Module Settings (in `lora_direct_spi.py`):
- Frequency: 433 MHz
- Bandwidth: 125 kHz  
- Spreading Factor: 7
- Coding Rate: 4/5
- Sync Word: 0x34

## Implementation Details

### Receiver Architecture
- Direct SPI communication with SX1276/SX1278 module
- No interrupt-based detection (polling mode)
- Continuous RX mode
- Real-time message logging

### Why Direct SPI?
The PyLoRa library (SX127x) had issues with GPIO edge detection on Raspberry Pi 4. Direct SPI implementation:
- ✓ More stable and reliable
- ✓ Works without interrupt handlers
- ✓ Better error handling
- ✓ Simpler debugging

## Testing

Currently in development phase. Code-only testing completed.

Future tests:
- Arduino transmitter integration
- Range testing
- Data integrity verification
- Power consumption analysis

## Status

- ✓ Receiver initialization: Working
- ✓ LoRa module communication: Working  
- ✓ 433 MHz configuration: Working
- ⏳ Full end-to-end testing: Pending (hardware transmitter)

## Development Branch

This code is being developed on the `feature/lora-implementation` branch.

## License

TBD

## Author

rasp_raccoon_berry
