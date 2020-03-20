# QuadcopterV1
This repository contains the source code for the first version of a home built quadcopter + radio transmitter

This first version of the quadcopter is based on a Arduino Nano, a MPU-6050 6 axis accelerometer + gyroscope, and a NRF24l01 transceiver.
The remote will be a old RC remote retrofit with another Arduino Nano and a NRF24l01 transceiver.

The second version of the quadcopter board will include a custom PCB with an ATmega328P and the MPU-6050 as well as power distribution.

# Technical specs:
X configuration quadcopter with:
  -4x Racerstar BR2208 2200KV rated brushless motors
  -4x Emax SimonK 12A ESC
  -1x 3S1P 1200mAh LiPo Battery
  -1x Arduino Nano V3
  -1x GY-521 I2C 6-axis sensor (based on MPU-6050 chip)
  -1x NRF24l01 transceiver w/ SMA antenna
Remote control:
  -1x Arduino Nano V3
  -1x GY-521 I2C 6-axis sensor (based on MPU-6050 chip)
  -1x NRF24l01 transceiver w/ SMA antenna
