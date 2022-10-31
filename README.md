# ProTag Test Fixture LED Controller

## Overview
This program is designed to run on the nRF52833DK host processor and provide a simple control interface for the LED board via USB CDC ACM.

When running and with an active USB connection it will accept 5 single byte commands.

- `'0'` - fade the LEDs on and off at full brightness
- `'1'` - turn the LEDs on to full brightness
- `'2'` - turn the LEDs off
- `'3'` - turn the LEDs to low brightness
- `'4'` - fade the LEDs on and off at low brightness

Note that these are asci numbers not decimal.

The LED boards PWM input should be connected to `P0.13` of the nRF52833 on the DK.
