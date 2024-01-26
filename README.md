# CPU Cooling Fan & LCD Control System

This repository contains the source code and documentation for a variable speed fan and Liquid Crystal Display (LCD) control system implemented on an Atmega128 microcontroller. This system allows users to manipulate the fan's duty cycle and power status using a Rotary Pulse Generator (RPG) and a pushbutton, with real-time feedback displayed on the LCD.

## Table of Contents
- Introduction
- Hardware
  - Schematic
  - Components
  - Connections
- Software
  - Hardware Initialization
    - Port Configuration
  - Interrupt Handling
    - RPG Interrupt
    - Pushbutton Interrupt
  - LCD Control
    - LCD Initialization
    - Display Functions
  - PWM Generation
    - Duty Cycle Control
  - Delays
- Conclusion

## Introduction

The goal of this project is to design a versatile control system that combines hardware and software to achieve precise control over a fan's speed and power state. The system uses interrupts to handle user input asynchronously, ensuring responsive interactions. Additionally, it provides real-time feedback on the LCD, displaying the fan's duty cycle as a percentage and its power status ("ON" or "OFF").

## Hardware

### Schematic

(Insert schematic here)

### Components

The hardware components used in this project include:
- Atmega128 microcontroller
- Rotary Pulse Generator (RPG)
- Pushbutton
- Liquid Crystal Display (LCD)
- DC Fan
- Various electronic components (resistors, capacitors, etc.)

### Connections

The hardware setup involves configuring the RPG, debouncing the pushbutton, and connecting the LCD and fan to the microcontroller. Detailed hardware specifications and connections are as follows:

#### Hardware Initialization

##### Port Configuration

- Ports B, C, and D are set to input/output for connecting to the LCD, fan, RPG, and pushbutton.
- Port B is configured to output the RS and enable signals for the LCD.
- Port D is configured to generate the PWM signal for fan speed control.
- Port C is configured to control various functions like LCD data and set the LCD backlight.

## Software

### Interrupt Handling

The software component of this project is written in assembly language and runs on the Atmega128 microcontroller. It consists of several key functions and features:

#### RPG Interrupt

- The system utilizes pin change interrupts (PCINT2) to detect state changes in the Rotary Pulse Generator (RPG).
- The RPGInterrupt subroutine is triggered when a change occurs.
- The current state of the RPG channels is captured, and previous state information is updated.
- Based on the change direction (clockwise or counterclockwise), the duty cycle of the fan is adjusted within the range of 0% to 100%.

#### Pushbutton Interrupt

- Pin change interrupts (PCINT0) are used to detect state changes in the pushbutton.
- The buttonInterrupt subroutine is triggered when a change occurs.
- The pushbutton's state is polled and processed.
- Pressing the button toggles the fan's power status between "ON" and "OFF."

### LCD Control

#### LCD Initialization

- The Liquid Crystal Display (LCD) is initialized following a specific sequence of commands to set it up for character display.
- Commands include setting the function (8-bit, 1-line), clearing the display, configuring display settings, and setting the entry mode.

#### Display Functions

- The system can display predetermined outputs to the LCD in addition to dynamic elements.
- It updates and displays the fan's duty cycle as a percentage and its power status ("ON" or "OFF") in real-time.

### PWM Generation

#### Duty Cycle Control

- Pulse Width Modulation (PWM) is used to control the fan's speed.
- PWM signals are generated using the built-in 8-bit timer (TCNT2).
- The system adjusts the fan's duty cycle based on user input through the RPG.
- Duty cycle values range from 0% to 100%, directly influencing the fan's speed.

### Delays

- Delays are implemented to ensure precise timing, especially during LCD initialization and control.
- The system uses the built-in 8-bit timer (TCNT0) to create various time-length delays, including 50 ms, 5 ms, and 100 microseconds.

## Conclusion

This project demonstrated our knowledge of PWM generation, use of interrupts, and interfacing multiple complex peripherals together. In addition, it tested our debugging skills as there were many tough hardware and software bugs ranging from simple interrupts not firing to deeper issues with having interference on our PWM signal. However, working as a team we were able to dynamically problem solve by working systematically to isolate working components and understand crucial implementation details through consultation of datasheets and documentation.

