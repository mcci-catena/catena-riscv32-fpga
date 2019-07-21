# Larva RISC-V for Lattice Breakout Board

`larva_bb` is the design variant targeting the Lattice iCE40 UltraPlus breakout board.

Please note that you **must** remove the "ICECLK" jumper, J51, before trying to download the design. If you don't, apparently the unprogrammed FPGA loads the clock to the point that the FTDI UART stops working during a download. This then causes Diamond Programmer to lock up, and even causes nasty driver problems leading to kernel crashes.

Check the project Kicad repository for wiring information at the system level. Pin assignments can be found in `larva_bb_pcf_sbt.pcf` in this directory.

The RPI UART cable should be connected to J3 (HEADER C) as follows:

| J3 Pin | Cable Pin | Function
|:------:|:---------:|:------------|
|  19    | Black     | Ground
|  17    | Orange    | RX data (input to FPGA)
|  15    | Yellow    | TX data (output from FPGA)

The UART transmits at 230,400 baud.

You should set your terminal emulator to treat linefeed (`\n`) as CR/LF.

Test with the cxx_test1 sample app. Build the FPGA, then launch a Larva Cygwin terminal. Change to the top of the development tree (i.e., above the `fpga` and `sdk` directories), and enter the following command to build the images.

```shell
for t in  free checked  ; do make LMIC_CFG_region=2 -C fpga/sw/cxx_test/software PROJECT_FPGA=larva_bb MDP_SERIAL=
1000 BUILDTYPE=$t -j4 clean || break ; done
for t in  free checked  ; do make LMIC_CFG_region=2 -C fpga/sw/cxx_test/software PROJECT_FPGA=larva_bb MDP_SERIAL=
1000 BUILDTYPE=$t -j4 || break ; done
```

In other words, first clean, then build. The clean is needed to make sure that the `PROJECT_FPGA=larva_bb` setting really takes effect.
