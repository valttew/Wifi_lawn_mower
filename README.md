# Wifi_lawn_mower
Wifi controlled lawn mower

- Based on ESP32-C6 devkit
- Neopixel used for debug throttle and steering
- generates throttle voltage on pin9 which will be smoothed by physical RC filter.
- generates enable signal for stepper driver and pulses for direction and steps.
- Generates wifi "Mower-RC" with password "mower1234".
- Simple mobile friendly control UI at 192.168.4.1.
  
