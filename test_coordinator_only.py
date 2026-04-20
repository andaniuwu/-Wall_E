#!/usr/bin/env python3
"""
Test script - runs only the coordinator thread without HMI for cleaner output
"""

import time
import RPi.GPIO as GPIO
import spidev
from datetime import datetime
import threading

# Import everything from the main coordinator
import sys
sys.path.insert(0, '/home/rasp_raccoon_berry/Documents/Wall_E_UV-and-Voltage-sense')

# Import the coordinator functions
from Wall_E_coordinator_raspberry import (
    initialize_hardware, configure_lora, query_device,
    coordinator_running, coordinator_thread, device_stats,
    NUM_DEVICES, QUERY_INTERVAL, FREQUENCY, RESPONSE_TIMEOUT
)

def test_coordinator_loop():
    """Main coordinator loop without HMI"""
    global coordinator_running, coordinator_thread
    
    print("\n✓ Coordinator thread started (no HMI)")
    
    cycle_count = 0
    while cycle_count < 120:  # ~10 minutes with 5s interval
        try:
            cycle_count += 1
            print(f"\n{'='*70}")
            print(f"Query Cycle #{cycle_count}: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
            print(f"{'='*70}")
            
            # Query all devices
            for device_id in range(1, NUM_DEVICES + 1):
                response = query_device(device_id)
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
    
    print("\n✓ Coordinator test completed")

def main():
    """Main entry point"""
    
    print("="*70)
    print("          Wall-E Coordinator TEST - No HMI")
    print("="*70)
    print(f"Configuration:")
    print(f"  - Number of devices: {NUM_DEVICES}")
    print(f"  - Query interval: {QUERY_INTERVAL} seconds")
    print(f"  - Response timeout: {RESPONSE_TIMEOUT} seconds")
    print(f"  - Frequency: {FREQUENCY/1e6} MHz")
    print(f"  - Test duration: ~10 minutes (120 cycles)")
    print("="*70)
    
    # Initialize hardware
    if not initialize_hardware():
        print("✗ Hardware initialization failed. Exiting.")
        return 1
    
    if not configure_lora():
        print("✗ LoRa configuration failed. Exiting.")
        return 1
    
    print("\n✓ System ready. Starting test...\n")
    
    try:
        test_coordinator_loop()
    except KeyboardInterrupt:
        print("\n\n✓ Keyboard interrupt received...")
    except Exception as e:
        print(f"\n✗ Error: {e}")
    finally:
        # Cleanup
        print("\n✓ Shutting down...")
        try:
            import spidev as sp
            spi_inst = sp.SpiDev()
            spi_inst.close()
            GPIO.cleanup()
            print("✓ Resources cleaned up")
        except Exception as e:
            print(f"⚠ Cleanup warning: {e}")
        
        print("✓ Test completed\n")
    
    return 0

if __name__ == "__main__":
    main()
