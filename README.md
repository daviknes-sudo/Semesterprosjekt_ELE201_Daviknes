# Semesterprosjekt_ELE201_Daviknes
Prosjekt 9. Sammenligning av ultralyd og infrarød avstandsmåler.
# ELE201 Semester Project 9: Ultrasonic vs IR Distance Measurement

Dual-technology distance measurement on an STM32F767ZI Nucleo-144. An HC-SR04 ultrasonic
sensor and a Sharp GP2Y0A21YK0F infrared sensor measure the same target, so their accuracy
and precision can be compared on different surfaces and conditions.
Course: ELE201 Microcontrollers and Data Networks (HVL), autumn 2026. Author: Markus.

## Features
- Ultrasonic distance from the echo pulse width (timer input capture), about 2 to 95 cm.
- IR distance from the sensor voltage (12-bit ADC, average of 16 readings), about 10 to 65 cm.
- 16x2 LCD shows both distances.
- Proximity alert: green / yellow / red LEDs (above 30 cm, 16 to 30 cm, 15 cm or less) and a
  buzzer whose pitch rises as the target gets closer (below 50 cm).
- Serial logging of raw data (CSV) for analysis in a spreadsheet.

## Hardware
| Part | Notes |
|---|---|
| STM32F767ZI Nucleo-144 | 108 MHz system clock |
| HC-SR04 | 5 V supply, ECHO through a 1k / 2k voltage divider to 3.3 V |
| Sharp GP2Y0A21YK0F | 5 V supply, analog output to ADC |
| TC1602A 16x2 LCD | 4-bit mode |
| 3 LEDs + 220 ohm | green, yellow, red |
| Passive piezo buzzer | driven with PWM |
| TMP36 | used to measure air temperature during the tests |

## Pin assignment
| Function | Pin | Peripheral |
|---|---|---|
| Ultrasonic TRIG | PD15 | TIM4_CH4 (PWM, 10 us every 60 ms) |
| Ultrasonic ECHO | PD14 | TIM4_CH3 (input capture) |
| IR sensor output | PA3 | ADC1_IN3 |
| Buzzer | PE9 | TIM1_CH1 (PWM) |
| LEDs green / yellow / red | PE13 / PF15 / PF14 | GPIO output |
| LCD RS, E | PF12, PF13 | GPIO output |
| LCD D4 to D7 | PG9, PG14, PB9, PB8 | GPIO output |
| Serial (ST-LINK VCP) | PD8 / PD9 | USART3 TX / RX, 115200 baud |

## How it works
- **Ultrasonic:** distance = c * t / 2, which is about t / 58 (t in us, result in cm).
- **IR:** the sensor gives a voltage that is roughly proportional to 1/d. The code uses
  d = a / (V - b) with a = 26.18 cm*V and b = 0.074 V, fitted on white paper (20 to 95 cm).
  Outside the trusted voltage range the LCD shows "out of range".
- Timers run at 1 us per tick (prescaler 107 at 108 MHz).

## Build and run
1. Open the project in VS Code with PlatformIO (or regenerate it from the `.ioc` file in STM32CubeMX).
2. Build and upload to the Nucleo board.
3. Open a serial terminal (for example PuTTY) on the ST-LINK virtual COM port, 115200 baud.
4. `LOG_CSV` in `main.c`: 1 sends `n,pulse_us,adc` lines for logging, 0 sends readable text.

## Measurement data and analysis
The spreadsheet in this repository holds the raw data (pulse width and ADC average), the
calculated distances, and the summary per surface and distance (mean, standard deviation, bias).
Series: white paper, dark jacket, aluminium foil, bright light, 30 degree tilt.
Distances: 10, 20, 40, 60, 80, 95 cm, 30 samples per point.

## Main findings
- Ultrasonic: stable bias of about -1 to -2.5 cm on hard surfaces; fails on a tilted target.
- IR: within about 3 cm up to 80 cm on paper and fabric after calibration; fails on mirror-like
  surfaces (aluminium foil) and becomes inaccurate beyond about 60 cm because the curve flattens.

## Repository content
- `src/main.c` (or `Core/Src/main.c`): application code
- `platformio.ini`, `*.ioc`: project configuration
- `ELE201_maling_*.xlsx`: measurement data and analysis

## Notes
- Code comments are in English.
- AI assistance (Claude) was used during the project; see the AI declaration in the report.
