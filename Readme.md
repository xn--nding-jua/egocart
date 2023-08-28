
# eGoCart

## Overview

This repository contains a BLDC-Motor-Controller with field-oriented-motor-control (FOC) using the TI C2000 TMS320F280049 with InstaSPIN-FOC control using a TI DRV8320RS SPI-Gatedrive.

The C2000 microcontroller receives its set-points either via SCI/UART based on a simple ASCII-command-system. Via SCI/UART it will return status-information about status-information for a planned Headup-Display.

## (Planned) Features:
* [x] input-voltages either 24V, 36V or 48V with input-currents of up to 20A (1kW@48V)
* [x] field-oriented-motor-control using TI InstaSPIN-FOC, TI Motorcontrol-SDK v5.0 and CodeComposerStudio 12.4
* [x] speed-controller controlled via SCI
* [ ] two full-featured B6-inverters to control up to two motors independently. Up to now I'm using two TI BOOSTXL-DRV8320RS for testing
* [ ] touchscreen headup-display using a RaspberryPi Zero. Up to now there is a Windows-GUI programmed in Delphi 7 that can be used to control the hardware

## Commands
* Set forward-speed for motor 1/2: AS0xxxxE / AS1xxxxE (with xxxx between 0rpm and 9999rpm)
* Set reverse-speed for motor 1/2: AR0xxxxE / AR1xxxxE (with xxxx between 0rpm and 9999rpm)
* Set maximum acceleration for motor 1/2: AA0xxxxE / AA1xxxxE (with xxxx between 0rpm/s and 9999rpm/s)
* Start/Stop system: AFM0003E / AFM0000E
* Read data for motor 1/2: AV00000E / AV10000E
* Get general information about the control: AI00000E

## Instructions to use this project
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
