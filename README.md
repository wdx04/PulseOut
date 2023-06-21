# PulseOut
Mbed Pulse Generator library for driving step motors.

It can generate a given number of pulses on a specific digital output pin.

Two classes are provided in this library:

* class OPMPulseOut: STM32 only, based on STM32 Timer's One-Pulse Mode, can generate high frequency pulses up to 500KHz(50% duty), with low CPU usage.
* class TickerPulseOut: based on Mbed Ticker class, can generate pulses on any GPIO pins. Not suitable for generating high-frequency signals because of the interrupt latency. Max usable frequency on a 32MHz STM32L0 MCU is 20KHz(50% duty).
