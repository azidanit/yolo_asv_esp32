# yolo_asv_esp32

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
