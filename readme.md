
![Microchip logo](https://raw.githubusercontent.com/wiki/Microchip-MPLAB-Harmony/Microchip-MPLAB-Harmony.github.io/images/microchip_logo.png)

# Sensor interface with LX7730 Example

- [Sensor interface with LX7730 Example Summary](#sensor-interface-with-LX7730-summary)
- [Building The Application](#building-the-application)
- [MPLAB Harmony Configurations](#mplab-harmony-configurations)
- [Hardware Setup](#hardware-setup)
- [Running The Application](#running-the-application)

---

## Sensor interface with LX7730 Example Summary

This example code runs on a SAMRH71F20-EK board to interface in between an LX7730 board and a GUI running on a computer.
This firmware convert GUI commands received on the UART to SPI commands in order to read and write LX7730 registers.
Several analog sensors are connected via a daughter board on the LX7730 board.

---

## Building The Application 
To build the application, open the project file in MPLABX IDE.

**Application Path:**   
* **For SAMRH71F20-EK board** :firmware\sensor_intf_lx7730_sam_rh71_ek.X

---

## MPLAB Harmony Configurations 

Refer to the MHC project graph for the components used and the respective configuration options.

* PIO configuration
    * The peripheral function is selected for the pins used by FLEXCOM1, FLEXCOM4 and FLEXCOM5.
    * Pin PA10 is connected on PCK peripheral with a frequency of 500kHz.
    * Pin PA9 is configure as output.
* FLEXCOM Peripheral:
    * FLEXCOM1
        * This peripheral is used in UART configuration for standard print and application logs.
    * FLEXCOM4
        * This peripheral is used in SPI configuration to communicate with the LX7730.
    * FLEXCOM5
        * This peripheral is used in UART configuration to communicate with the computer GUI.

---

## Hardware Setup

1. Project sensor_intf_lx7730_sam_rh71_ek.X
    * Hardware Used
        * SAMRH71 Evaluation Kit.
        * LX7730 Daughter board with sensor board and the 5 different sensors connected (Temperature, Pressure, Magnetic field strength, Distance and
accelerator)
        * The LX7730-DB to SAMRH71F20-EK Linker board.
        * JTAG debugger probe for SAMRH71-EK (ICD4, J32, SAM-ICE, ...)
    * Hardware Setup
        * Connect the debugger probe to J33 on the SAMRH71-EK board.
        * Connect the J15 USB port of the board to a computer using a mini USB cable.
        * Connect the UART pins to computer UART using TTL to USB or RS232 convertor:
            | SAMRH71F20-EK        | Computer       |  LX7730-DB to SAMRH71F20-EK Linker board |
            | :------------- :     |:-------------: | :--------------------------------------: |
            | PC9 (J24.14)         | RX             | Jumper between PL_ETC.5 and PL_ETC.11    |
            | PC10 (J24.13)        | TX             | Jumper between PL_ETC.6 and PL_ETC.12    |
        * Connect the followings signals between the SAMRH71-EK board and the LX7730 Daughter board:
            | SAMRH71F20-EK        | LX7730 Daughter board  |  LX7730-DB to SAMRH71F20-EK Linker board |
            | :-------------:      |:---------------------: | :--------------------------------------: |
            | GND                  |   GND                  |                                          |
            | PC0 (J24.16)         |   SPIB_MOSI            | Jumper between PL_SPIB.4 and PL_SPIB.8   |
            | PC1 (J24.17)         |   SPIB_MISO            | Jumper between PL_SPIB.1 and PL_SPIB.5   |
            | PC2 (J24.18)         |   SPIB_CLK             | Jumper between PL_SPIB.2 and PL_SPIB.6   |
            | PC3 (J24.15)         |   SPIB_NCS             | Jumper between PL_SPIB.3 and PL_SPIB.7   |
            | PA9 (J24.3)          |   RESET                | Jumper between PL_ETC.2 and PL_ETC.8     |
            | PA10 (J24.4)         |   CLK                  | Jumper between PL_ETC.1 and PL_ETC.7     |
        * Make sure the LX7730-DB SPI_B switch SW4 is set low and SPI_A switch SW3 is set high.

---

## Running The Application

1. Build and Program the application using the MPLAB X IDE.
2. Launch the computer GUI and select the appropriate COM port.
