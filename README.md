# Nixie_SW
Arduino Nano firmware for Nixie_PCB

Based on an idea from Martijn and Ronald, a Nixie PCB has been made. This project contains the firmware for it and uses an Arduino Nano. It used 6 IN-14 Nixie-Tubes, 6 RGB LEDs, a DS3231 RTC, IR-receiver, a BMP180 and a DHT22 sensor and a dedicated high-voltage circuit to generate 170 Volts.

The firmware contains the following features: UART (via USB) interrupt-driven communication, command-interpreter, infrared communication, I2C communication and a task-scheduler (non pre-emptive).

Use with Atmel Studio v6 or higher.
