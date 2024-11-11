# PIC16F1936 PWM Motor Control with 7-Segment Display

This project allows motor speed control using PWM and includes a 7-segment display to show RPM. Features include increment/decrement buttons, a debounce mechanism, and EEPROM storage to save the last set RPM.

## Features

- **Motor Speed Control**: Uses PWM on the PIC16F1936.
- **7-Segment Display**: Displays RPM with an 8ms timer interrupt for smooth viewing (no flickering due to persistence of vision).
- **Increment/Decrement Keys**: Adjust RPM with debounced buttons.
- **EEPROM Storage**: Saves last RPM setting on power-off, retaining it on restart.

## Requirements

- **Hardware**: PIC16F1936, 7-segment display, motor driver, buttons for RPM control.
- **Software**: MPLAB X IDE, XC8 Compiler.

## Getting Started

1. Clone the repository.
2. Open the project in MPLAB X IDE.
3. Set up the PWM, 7-segment display, and button connections.
4. Configure EEPROM and timer interrupts as per your application requirements.
5. Compile and upload the code to the PIC16F1936.

## Usage

1. Use increment and decrement buttons to adjust the motor RPM.
2. The 7-segment display shows the current RPM, with no visible flickering due to an 8ms timer interrupt.
3. The RPM setting is saved in EEPROM, allowing it to persist between power cycles.

## License

This project is open-source under the MIT License.
