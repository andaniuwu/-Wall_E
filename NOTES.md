# Notes

## Current Sensor Updates

- Sensor: SCT-013-030 (30A:1V).
- ADC reads are now in millivolts using `analogReadMilliVolts()`.
- ADC attenuation set to 11dB for current and voltage channels.
- DC offset `VREF_mV` should be measured at the ADC input and updated in code.
- Sampling increased (`ADC_SAMPLES`) and moving average is per-channel.
- `CURRENT_FLOOR_A` clamps idle noise to 0A without masking real loads.

## Known Behavior

- Floating current channels (not connected) can saturate and show max values on the Raspberry Pi.
- Clamp the sensor around the hot wire only (not both conductors).
