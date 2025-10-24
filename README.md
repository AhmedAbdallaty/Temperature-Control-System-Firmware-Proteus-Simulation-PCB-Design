🌡️ Temperature Control System

This project is a Temperature Control System that automatically monitors and regulates temperature using an embedded microcontroller. The system combines firmware programming, Proteus simulation, and PCB design, providing a complete workflow from concept to hardware implementation.

🧠 Overview

The system reads the surrounding temperature using an LM35 sensor and controls a DC fan automatically based on predefined thresholds.
When the temperature exceeds the set limit (30°C), the microcontroller generates a PWM signal to control the fan speed. The temperature readings are also sent through UART for serial monitoring.

This project demonstrates practical embedded system integration — covering sensor interfacing, PWM control, ADC reading, UART communication, and PCB design for real-world hardware implementation.

⚙️ Key Features

Real-time temperature monitoring

Automatic fan speed control via PWM

UART communication for data logging

Proteus simulation for circuit verification

Compact PCB design for hardware implementation

Firmware written in Embedded C using STM32 HAL

🧩 Tools & Components

Microcontroller: STM32F103C8T6 (Blue Pill)

Sensor: LM35 Temperature Sensor

Actuator: DC Fan Motor (controlled through a transistor or MOSFET)

Software Tools:

STM32CubeIDE (Firmware development)

Proteus 8 (Circuit simulation)

Altium Designer (PCB design)

🧰 System Operation

The LM35 sensor produces an analog voltage proportional to the ambient temperature.

The MCU reads this signal using the ADC and converts it into a temperature value.

If the temperature exceeds the set threshold, the MCU adjusts the PWM duty cycle to increase fan speed.

When the temperature falls below the limit, the fan speed reduces or turns off.

The temperature readings are sent to a serial terminal for observation.

💡 Results

The fan activates automatically when temperature exceeds 30°C.

Smooth PWM control of fan speed.

Accurate ADC readings and stable UART communication.

Verified circuit functionality in Proteus simulation.

Final PCB design ready for fabrication.

🚀 Project Files

Firmware/ – Contains all embedded C source files.

Proteus_Simulation

Full schematic and PCB design.


🔮 Future Improvements

Add LCD display for direct temperature readout.

Integrate IoT module for remote monitoring.

Implement PID-based fan speed control for smoother operation.

Add buzzer and overheat alert feature.


