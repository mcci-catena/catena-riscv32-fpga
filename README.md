# MCCI Catena RISC-V FPGA Core

[This repository](https://github.com/mcci-catena/catena-riscv32-fpga) contains the Verilog RTL code for the MCCI Catena Risc-V V32I CPU with suitable peripherals for the MCCI Catena 4710 for the Lattice Semiconductor iCE-40 Ultra Plus FPGA.

**Contents**:

<!-- markdownlint-capture -->
<!-- markdownlint-disable -->
<!-- TOC depthFrom:2 updateOnSave:true -->

- [Documentation](#documentation)
- [Software](#software)
- [Building the FPGA](#building-the-fpga)
- [Building software](#building-software)
- [Downloading images](#downloading-images)
- [Meta](#meta)
	- [License](#license)
	- [Support Open Source Hardware and Software](#support-open-source-hardware-and-software)
	- [Trademarks](#trademarks)

<!-- /TOC -->
<!-- markdownlint-restore -->

The repository contains both FGPA code and some test software; the test software is intended to be used with the MCCI Catena RISC-V SDK.

The following hardware platforms are supported as build targets. The directories are for the Lattice iCECube2 projects files, and all begin with `hw/syn/ice40up/`.

Platform                | Directory          | Project file
------------------------|--------------------|--------------
MCCI Catena 4710        | `catena4710-riscv` | `catena4710-riscv_sbt.project`
Lattice iCE40-UP MDP (FPGA "C")  | `larva_mdp_c`      | `larva_mdp_c_sbt.project`
Lattice iCE40-UP Breakout Board  | `larva_bb`         | `larva_bb_sbt.project`
Himax demo board        | `larva_himax`      | `larva_himax_sbt.project`

## Documentation

Documents in this repository include:

- [hw/syn/ice40up/README.md](./hw/syn/ice40up/README.md): instructions on how to create a new FPGA design.
- [hw/src/boards/Catena4710/PERIPHERALS.md](./hw/src/boards/Catena4710/PERIPHERALS.md): address map and documentation for the peripherals included in the Catena 4710
- [hw/src/boards/lora-mdp/PERIPHERALS.md](./hw/src/boards/lora-mdp/PERIPHERALS.md): information about peripherals included when building for the Lattice Semiconductor MDP.
- [hw/syn/ice40up/larva_bb/README.md](./hw/syn/ice40up/larva_bb/README.md): information about using the FPGA on the Lattice iCE40UP breakout board.

## Software

The following demo programs are supported, all in `${top}/sw`.

Directory | Description
----------|------------
cxxtest   | C++ test program and simple "it's alive" test case.
ttn_sensor1 | A demo (targeting the MDP) which exercises the sensors on the MDP board, transmitting data using LoRaWAN and The Things Network ("ttn").
lorawan_raw | A simple radio test, useful for verifying that things are working. Use another lorawan_raw device, and you'll be able to see transmit and receive working.

## Building the FPGA

This project currently requires the Lattice Semiconductor iCECube2 FPGA software for building the FPGA.

1. Launch iCube2. We used release `2017.01.27914`.

2. In the Design tree (on the left), select Open Project, and open `{top}/fpga/hw/syn/ice40up/*DIR*/*PROJECT*`

3. From the menu bar, select `Tool>Run All`.

## Building software

Building the software requires use of the SDK. Normally the SDK lives in a parallel directory to the `fpga` directory. Please see instructions in the SDK.

## Downloading images

The SDK includes scripts for using either Lattice Diamond Programmer or `iceprog` from the iceStorm toolchain.

## Meta

### License

This repository is released under [GPLv3](./LICENSE.md). Commercial licenses are also available from MCCI Corporation.

### Support Open Source Hardware and Software

MCCI invests time and resources providing this open source code, please support MCCI and open-source hardware by purchasing products from MCCI, Adafruit and other open-source hardware/software vendors!

For information about MCCI's products, please visit [store.mcci.com](https://store.mcci.com/).

### Trademarks

MCCI and MCCI Catena are registered trademarks of MCCI Corporation. All other marks are the property of their respective owners.
