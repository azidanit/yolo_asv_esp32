# yolo_asv_esp32
Simple MultiTask with FREERTOS - Arduino

Feature:
- UDP Communication
- PWM in for RC Controller
- PWM out to Drive Motor ESC
- Addresable RGB LED for indicator

## Wiring
### Ethernet
| ESP32                              | W5500 |
|------------------------------------|-------|
| D5                                 | CS    |
| D18                                | SCK   |
| D19                                | MISO  |
| D23                                | MOSI  |
| 3.3v (better with external 200mha) | VCC   |
| GND                                | GND   |
### PWM RC Input
| ESP32 | Channel |
|-------|---------|
| 34    | 0       |
| 35    | 1       |
| 32    | 2       |
| 33    | 3       |
| GND   | GND     |
### PWM RC Ouput
| ESP32 | Channel     |
|-------|-------------|
| 26    | Motor Left  |
| 25    | Motor Right |
### Addressable RGB Output 
| ESP32 | Channel     |
|-------|-------------|
| 17    | ARGB Data  |
