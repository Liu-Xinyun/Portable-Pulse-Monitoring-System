# Portable Pulse Monitoring System (STM32-Based)

A full-stack embedded system for real-time pulse monitoring,
integrating analog signal conditioning, microcontroller-based processing,
and wireless data transmission.

## Background
This project was developed to design a portable pulse monitoring device
capable of accurately measuring human pulse signals and displaying
heart rate information in real time. The system emphasizes
hardware–software co-design and robustness in noisy physiological signal environments.

## System Overview
The system uses an optical pulse sensing method based on infrared light.
Pulse signals are acquired from a finger or earlobe, conditioned by analog circuits,
converted into digital signals, and processed by an STM32 microcontroller.

Main system components include:
- Optical pulse sensor
- Analog amplification and filtering circuits
- STM32F103 microcontroller
- OLED display
- Bluetooth module
- USB-to-serial communication
- Buzzer for alarm indication

## Hardware Design
- **MCU:** STM32F103C8T6 (Cortex-M3)
- **Signal Conditioning:**  
  - Two-stage low-pass amplification using LM358  
  - Comparator with hysteresis for pulse shaping  
- **Input Signal Range:** Typical pulse frequency 0.5–3 Hz
- **Voltage Adaptation:** Output signal scaled to 3.3 V for MCU GPIO input
- **PCB Design:** Complete schematic design, PCB layout, soldering, and debugging

Both Multisim simulation and hardware measurements were used to verify
signal amplification, filtering performance, and voltage compatibility.

## Software Design
- **Language:** C
- **Development Tools:** STM32CubeMX, Keil µVision
- **Architecture:** Modular design with main loop scheduling
- **Key Functional Modules:**
  - ADC sampling and pulse counting
  - Digital moving-average filtering
  - Button scanning and state control
  - OLED display update
  - Bluetooth and USB serial communication
  - Buzzer alarm control

The system supports multiple operating modes:
- 60-second pulse measurement mode
- Real-time pulse measurement mode
- Configurable heart rate alarm thresholds

## Data Processing & Visualization
- Real-time waveform output via serial port
- External visualization using VOFA+ and MATLAB
- MATLAB scripts used to reconstruct pulse waveforms and verify timing accuracy

Measured pulse waveforms and calculated BPM values are consistent with
oscilloscope observations and physiological expectations.

## Results
- Accurate pulse measurement across simulated and real physiological signals
- Stable real-time BPM output with fast stabilization (~5 s)
- Reliable alarm triggering for abnormal heart rate conditions
- Successful Bluetooth and USB data transmission

## Skills Demonstrated
- Embedded system design (hardware + firmware)
- Analog signal processing for biomedical signals
- STM32 development and peripheral control
- PCB design and hardware debugging
- Real-time data acquisition and processing
- Serial communication and data visualization

## Author
Liu Xinyun
LZhejiang University  
