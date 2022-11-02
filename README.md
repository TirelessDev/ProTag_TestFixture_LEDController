# ProTag Test Fixture LED Controller

## Overview
This program is designed to run on the nRF52833DK host processor and provide a simple control interface for the LED board and NFC reader via USB CDC ACM.

When running and with an active USB connection it will accept 7 single byte commands.

- `'0'` - fade the LEDs on and off at full brightness
- `'1'` - turn the LEDs on to full brightness
- `'2'` - turn the LEDs off
- `'3'` - turn the LEDs to low brightness
- `'4'` - fade the LEDs on and off at low brightness
- `'5'` - turn the nfc reader's field off
- `'6'` - turn the nfc reader's field on

Note that these are asci numbers not decimal.

The LED boards PWM input should be connected to `P0.13` of the nRF52833 on the DK.

The the nRF52833DK should be connected to the NFC reader as follows:

- `P0.23` - SCK
- `P0.21` - MOSI
- `P0.22` - MISO
- `P0.20` - CS
- `P0.03` - IRQ
- `VDD`   - I/O Vref

The text records of any type 2 NFC tags read will be reported with a `'nfc: '` prefix.

Once a tag has been read the field will automatically turn off. A subsequent call to `'6'` will be required to read again.