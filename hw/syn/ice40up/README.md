# Adding a new FPGA Design Variant

<!-- TOC depthFrom:2 updateOnSave:true -->

- [Introduction](#introduction)
- [Name your new FPGA design variant](#name-your-new-fpga-design-variant)
- [Create a top-level Verilog file](#create-a-top-level-verilog-file)
- [Create the synthesis part of the design from a template](#create-the-synthesis-part-of-the-design-from-a-template)
- [Finish the design variant](#finish-the-design-variant)
- [Tips](#tips)

<!-- /TOC -->

## Introduction

FPGA design variants are added in this directory. Assuming that you already have the HDL sources prepared, but merely want to map an existing design into a new board, you proceed as follows.

In this workflow, you may have one FPGA Verilog description, but then a variety of designs using the FPGA. The designs can vary in a number of ways:

- You might use a different package.
- You might be using different pins for connecting to the same logical function.
- You might have a different top-level design.
- You might use a different Flash EEPROM for storing the data.
- You might use a different FTDI programming topology.

## Name your new FPGA design variant

Each FPGA design variant has its own directory in `fpga/hw/syn/ice40up`.

Choose a name for the top-level directory for your new variant. Following the Lattice conventions, we'll also use this name as a filename fragment to insert in the synthesis implementation directory name.  In the rest of this discussion, we'll use `${DESIGN}` to represent your chosen name.

## Create a top-level Verilog file

This procedure assumes your top-level Verilog file is in a directory at `../../../src/boards/${BOARD}/${TOPLEVEL}.v`. Here are some examples:

Target board          | `DESIGN`           | `BOARD`      | `TOPLEVEL`       | Comments
----------------------|--------------------|--------------|------------------|----------
MCCI Catena 4710      | `catena4710-riscv` | `Catena4710` | `catena4710_top` |
Lattice MDP (block C) | `larva_mdp_c`      | `lora-mdp`   | `lora_mdp_c_top` |
Himax Lattice Proto   | `larva_himax`      | `lora-himax` | `lora_himax_iCE40UP` |
Lattice Breakout Board| `larva_bb`         | `lora-himax` | `lora_himax_iCE40UP` | Because pins are defined in the SYN project, we can share design.

You need to create at least an empty toplevel file in your new board directory, otherwise the script in the next step will refuse to run.

## Create the synthesis part of the design from a template

Create the directory `fpga/hw/syn/ice40up/${DESIGN}`.

Your starting point is based on your FPGA type. This will be either an ICE40 UltraPlus UG30 or SG48. Suitable templates exist for each, in the `_templates` direcotry.

You need to create three files:

1. the iCEcube `*_sbt.project` file,
2. the Synplify `*.sdc` file, and
3. the synthesis pin constraint file, `*_pcf_sbt.pcf`.

(Note: the "sbt" abbreviation stands for "Silicon Blue Tools"; Silicon Blue was the inventor of the iCE technology, and was acquired by Lattice Semiconductor in 2011.)

The following script will do all the work:

```bash
./_templates/new-design.sh -b $BOARD -d $DESIGN -t $TOPLEVEL -p $PACKAGE
```

`$PACKAGE` selects the FPGA type, `uwg30` or `sg48`.

Starting with the files in `_template/uwg30` or `_template/sg48`, the script will:

1. Copy `_template/${PACKAGE}/DESIGN_sbt.project` to `${DESIGN}/${DESIGN}_sbt.project`.
2. Copy `_template/${PACKAGE}/DESIGN.sdc` to `${DESIGN}/${DESIGN}.sdc`
3. Copy `_template/${PACKAGE}/DESIGN_pcf_sbt.pcf` to `${DESIGN}/${DESIGN}_pcf_sbt.pcf`.
4. Edit the variables into the templates.

## Finish the design variant

- Add `${DESIGN}/README.md` file describing the use of this variant.

- Edit the `fpga_sdk_setup.mk` file to set up the programming port and flash variants.

- Edit the pin-out file `${DESIGN}_pcf_sbt.pcf` to match the pin-outs needed on your target board. For the sanity of those who follow you, be sure to list all the relevant names. For example:

    ```pcf
    #        Verilog     Pkg     Schematic    Conn     External name
    #        -------     ---     ---------    ----     -------------
    set_io   gpio1_pin   C3    # "SPARE_C0"   Jx-14   "NRST"
    set_io   gpio0_pin   B1    # "SPARE_C1"   Jx-16   "CS"
    ...
    ```

- Double-check the `${DESIGN}_sbt.project` file to confirm that it looks correct.

## Tips

- The first time you run iCECube for the new design, you should be careful to check the resulting pinout, and make sure that ICECube accepted your pinout file. If it doesn't, it will silently delete it from your project.

- One easy way to track what ICECube has done is to use `git add` to stage the new files _without committing_ before opening the file the first time. Then after running a full synthesis with ICECube, use `git diff` to see whether ICECube changed any of the files you're tracking.

- If you have to change the top-level after running one or more synthesis runs, do the following.

   1. Close the project if it's open in iCECube.
   2. Edit the .project file, and change the top-level (in three places).
   3. Very important: remove the old `*_Implmnt` directory.
   4. Now open the project in iCECube and select "Tools>Run all".
