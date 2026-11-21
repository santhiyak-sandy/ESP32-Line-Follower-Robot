
# Circuit Connections

## Components

- ESP32 NodeMCU
- TB6612FNG Motor Driver
- IR Sensor Array
- Left and Right DC Motors

## ESP32 to Motor Driver

| ESP32 Pin | TB6612FNG Pin |
|---|---|
| GPIO 19 | AIN1 |
| GPIO 18 | AIN2 |
| GPIO 21 | BIN1 |
| GPIO 13 | BIN2 |
| GPIO 23 | STBY |

## Sensor Connections

| Sensor | ESP32 Pin |
|---|---|
| Sensor 1 | GPIO 12 |
| Sensor 2 | GPIO 27 |
| Sensor 3 | GPIO 33 |
| Sensor 4 | GPIO 25 |
| Sensor 5 | GPIO 26 |
| Sensor 6 | GPIO 35 |
| Sensor 7 | GPIO 32 |
| Sensor 8 | GPIO 34 |

## Power Supply

- Motors are powered through the motor driver.
- The ESP32 is powered using the appropriate supply.
- Ensure a common ground between the controller
  and motor driver.

## Notes

Verify all connections against the final hardware
before powering the circuit.
