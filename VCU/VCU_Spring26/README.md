
# VCU Firmware (Teensy 4.1) Spring 26

Firmware for the Cal Poly Pomona FSAE Electric 2026 **Vehicle Control Unit (VCU)** running on a **Teensy 4.1**.

## What it does
- Sends command/control data to the **PM100DX inverter** (via CAN)
- Reads **accelerator and pedal position sensors** and applies validation/plausibility checks (as implemented)
- Drives the **ST7796S display** for driver feedback (states, warnings, live values)
- Publishes vehicle status on **CAN** for the rest of the system / logging
