
# eGoCart

## Overview

This repository contains a BLDC-Motor-Controller with field-oriented-motor-control (FOC) using the TI C2000 TMS320F280049 with InstaSPIN-FOC control. This control is able to auto-detect a connected motor or can use a predefined motor.

The C2000 microcontroller receives its set-points either via SCI/UART based on a simple ASCII-command-system or via an analog-input. Via SCI/UART it will return status-information about voltages, currents, speed, temperature, motor-parameter and more. A headup-display is implemented using a RaspberryPi Zero and python.

(Planned) Features:
* field-oriented-motor-control using TI InstaSPIN-FOC, TI Motorware and CodeComposerStudio 12.4
* speed-controlled (auto-cruise) or torque-controlled
* two full-featured B6-inverters to control up to two motors independently
* touchscreen headup-display using a RaspberryPi Zero
* input-voltages either 24V, 36V or 48V with input-currents of up to 20A (1kW@48V)

## Information

This project is within the early planning-phase. More information will follow.

Time-schedule:
* 07/2023: first tests with TI Motorware (Already done. Sensorless BLDC is rotating with working speed-controller controlled via SCI/UART)
* Begin of 08/2023: design of the hardware (two 3-phase-inverters with voltage/current-measurement, gate-drivers and MOSFETs)
* Mid of 08/2023: Programming of motor-controller
* 09/2023: test of the design and programming of the headup-display
* 10/2023: whole system working

## Instructions
* Download and install TI CodeComposerStudio from https://www.ti.com/tool/download/CCSTUDIO (v12.4 has been used for compiling)
* Download and install TI C2000Ware from https://www.ti.com/tool/C2000WARE
* Download and install TI C2000Ware-Motorcontrol-SDK from https://www.ti.com/tool/C2000WARE-MOTORCONTROL-SDK
* Clone the GitHub-Project to a directory of your choice and load the CCS-project
* change the file main.h to your needs and compile the project
* Upload the binary to an LAUNCHXL-F280049C with TI BOOSTXL-DRV8320RS or the self-made eGoCart-PCB
* connect motor(s), connect the RaspberryPi Zero and have fun!


## If compilation of project fails
* Update compiler-variable ${C2000WARE_MC_SDK_LOC} to the correct path (standard is "C:\ti\c2000\C2000Ware_MotorControl_SDK_5_00_00_00")
* following paths have to be set as search paths in compiler include options: ${C2000WARE_MC_SDK_LOC}/c2000ware/device_support/f28004x/headers/include/ and ${C2000WARE_MC_SDK_LOC}/c2000ware/device_support/f28004x/common/include/
* following files have to be included to the project: ${C2000WARE_MC_SDK_LOC}/c2000ware/device_support/f28004x/...  .../headers/cmd/f28004x_header_nonbios.cmd, .../headers/source/f28004x_globalvariabledefs.c, .../common/targetConfigs/TMS320F280049C_LaunchPad.ccxml
