# ProTag Test Fixture LED Controller

## Overview
This program is designed to run on the nRF52833DK host processor and provide a simple control interface for the LED board via USB CDC ACM.

When running and with an active USB connection it will accept three single byte commands.

- `'0'` - fade the LEDs on and off
- `'1'` - turn the LEDs on
- `'2'` - turn the LEDs off

Note that these are asci numbers not decimal.

The LED boards PWM input should be connected to `P0.13`.
