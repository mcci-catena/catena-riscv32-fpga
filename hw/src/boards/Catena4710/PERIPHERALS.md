# Catena 4710 Peripherals

Several peripherals are built into this microcontroller.
<!-- markdownlint-capture -->
<!-- markdownlint-disable -->
<!-- TOC depthFrom:2 updateOnSave:true -->

- [Introduction](#introduction)
- [Bit Description Notation](#bit-description-notation)
	- [Bit Type Codes](#bit-type-codes)
	- [Bit Access Codes](#bit-access-codes)
- [Address Map](#address-map)
- [UART](#uart)
	- [UART Registers](#uart-registers)
		- [UART Data](#uart-data)
		- [UART Control/Status](#uart-controlstatus)
- [I2C Interface](#i2c-interface)
	- [I2C Registers](#i2c-registers)
		- [I2C Data](#i2c-data)
- [GPIO Interface](#gpio-interface)
	- [GPIO Registers](#gpio-registers)
		- [GPIO Data](#gpio-data)
- [SPI Interface](#spi-interface)
	- [SPI Registers](#spi-registers)
		- [SPI Data](#spi-data)
- [Cycle Counter](#cycle-counter)
	- [Cycle Counter Registers](#cycle-counter-registers)
	- [Cycle-Counter Data Register](#cycle-counter-data-register)
- [SOC Information](#soc-information)
	- [SOCINFO Registers](#socinfo-registers)
	- [SOCINFO Data](#socinfo-data)
- [PDM Microphone](#pdm-microphone)
	- [PDM Microphone Registers](#pdm-microphone-registers)
	- [PDM Microphone Control Register](#pdm-microphone-control-register)
	- [PDM Microphone Data Register](#pdm-microphone-data-register)
	- [PDM Microphone Config Register](#pdm-microphone-config-register)

<!-- /TOC -->
<!--markdownlint-restore -->
## Introduction

The SOC FPGA Design for the Catena 4470 adds a variety of built-in peripherals: two UARTs, I2C, SPI, and GPIO support for software running on the RISC-V core.

The design is a little unusual in a couple of ways.

1. The RISC-V core doesn't implement interrupts.
2. The RISC-V core doesn't issue unambiguous read requests.

Thus, the cycle counter is just a free-running upcounter of CPU clock cycles. The UARTS have no read FIFO, but they have write FIFOs, becuase otherwise we found that otherwise debug and other output was just too slow and inconvenient (we want to do non-blocking writes and still get output). Every character read from the UART FIFO must be explicitly acknowledge (because we can't tell if the operation is really a read or just a speculative issue on the address bus).

## Bit Description Notation

### Bit Type Codes

- _**in**_ means that the bit represents an input.
- _**out**_ means that the bit represents an output. (Reads might represent the external value, see the [_access code_](#bit-access-codes)).
- _**i/o od**_ means that the bit represents an I/O pin that is open drain.

### Bit Access Codes

- _**rw**_ means a normal read/write bit.
- _**rwc**_ means that the bit can be read and written, but that writes have a special control function -- consult the description.
- _**ro**_ means that the bit is read-only; write values are ignored.
- _**r0wi**_ means that the bit reads as zero, and write values are ignored. (This is a special case of _ro_.)
- _**r1wi**_ means that the bit reads as one, and write values are ignored.
- _**rw1c**_ means that the bit has a value that is set by hardware. Writes with this bit zero have no effect; writes with this bit set clear the bit.
- _**rpwd**_ means that the bit reads the value from the pin, and writes a data value to a register. Normally this means that the output is controlled by the written value, but the read-back depends on what is happening externally to the FPGA. For example, an open-drain wire-ORed output might read back zero even after the register has been written to 1, because some other device on the wire is still driving the wire to zero.

## Address Map

| Address | Bytes | Name | Description |
|:-------:|:-----:|:-----|:------------|
| 0x80000000 | 16 | `UART0` | [Simple UART](#uart) instance 0 |
| 0x80000010 | 16 | `UART1` | [Simple UART](#uart) instance 1 |
| 0x80000020 | 16 | `I2C`   | [Simple I2C](#i2c) interface |
| 0x80000030 | 16 | `GPIO`  | [GPIO](#gpio) Interface |
| 0x80000040 | 16 | `SPI`   | [Simple SPI](#spi) interface |
| 0x80000050 | 16 | `CYCLES` | [Cycle counter](#cycle-counter) |
| 0x80000060 | 16 | `SOCINFO` | [SOC Information](#soc-information) |
| 0x80000070 | 16 | `MIC` | [PDM Microphone](#pdm-microphone) |

## UART

Each UART interface operates at a fixed baud rate set in the Verilog code -- check the top-level for your design. Typically this is 230,400 baud.

As is traditional, data bytes are read and written from a `UART_DATA` register; control-plane information is accessed via a `UART_CSR` register.

The transmit path has a write FIFO (16 bytes deep in the default implementation, but again, check your design to be sure). Software typically writes until the FIFO is full, then waits for the FIFO to drain, polling bits in the UART CSR.

The receive path has no FIFO at all in the current design, but the character buffer is separate from the shift register. It's up to software to keep up, but software has roughly 9 bit times to consume the character before it can be overwritten.

The current microcontroller's implementation is not great about disambiguating reads -- it simply copies whatever appears at the ALU output to the read register, without generating a strobe for the LD instruction. Rather than fix the core, we currently require software to explicitly acknowledge each byte read.

### UART Registers

Base address:

- UART0: 0x80000000.
- UART1: 0x80000010.

| Offset  | Width | Name | Description |
|:-------:|:-----:|:-----|:------------|
| 0x0 | 32 | [`DATA`](#uart-data) | Data to/from the UART |
| 0x4 | 32 | [`CSR`](##uart-controlstatus) | Control/status register |
| 0x8 | 32 | reserved | Not implemented |
| 0xC | 32 | reserved | Not implemented |

#### UART Data

| Bit | 31 .. 8 | 7 .. 0 |
|:---:|:-------:|:------:|
| **Name** | reserved | `FIFO` |
| **Reset** |  0 | 0 |
| **Access** | r0wi | rwc |

#### UART Control/Status

| Bit        | 31 .. 8  | 7      | 6 .. 2 | 1 |  0 |
|:----------:|:--------:|:------:|:------:|:-:|:-:|
| **Name**   | reserved | `RXD`  |reserved     | `RXRDY` | `TXFULL` |
| **Reset**  |  0       | 0      | 0      | 0 | 0 |
| **Access** | r0wi     | ro     |r0wi    | rw1c | ro |

The `RXRDY` bit has two functions.

- When read, it is 1 if there is valid receive data in the `DATA.FIFO` register.
- When written, if this bit is set, it acknowledges the previous character, clearing the `RXRDY` bit until a new character arrives.

The `TXFULL` bit is set by hardware whenever the transmit FIFO is full. It is cleared whenever the FIFO drops below full. (Yes, it would be nice to have empty and 50% status bits.)

`RXD` represents the current state of the UART's receive pin. On UART0, for the LoRaWAN face-detect demo, it's connected to `FACE`, the "face present" signal (1 == face present), a non-inverting input.

## I2C Interface

The I2C implementation is completely software driven; but it has a dedicated GPIO register to make programming easier and faster.

Two pins, I2C_SCL and I2C_SDA, are controlled by this interface. Both are open-drain signals -- in other words, they alternate between high-impedance (no drive at all), and driving actively to ground. Each pin is in the high-impedance (or "Z") state when the corresponding register bit is set to 1. A pin is in the low-impedance to ground (or "L") state when the corresponding register bit is set to 0.

### I2C Registers

Base address: 0x80000020.

| Offset  | Width | Name | Description |
|:-------:|:-----:|:-----|:------------|
| 0x0 | 32 | [`DATA`](#i2c-data) | Data to/from the pins registers |
| 0x4 | 32 | reserved | not implemented |
| 0x8 | 32 | reserved | Not implemented |
| 0xC | 32 | reserved | Not implemented |

#### I2C Data

| Bit        | 31 .. 2  | 1 | 0 |
|:----------:|:--------:|:-:|:-:|
| **Name**   | reserved | `SDA`   | `SCL`  |
| **Reset**  |  0       |   1     |   1     |
| **Type**   | n/a      |   i/o od  |  i/o od    |
| **Access** | r0wi     |   rpwd    |  rpwd     |

Software is responsible for bit-banging the interface and for implementing the I2C protocol.

Note that the value read back is the current value at the pin of the device, not the last value written to the register.

## GPIO Interface

The GPIOs are architected as classic software-configured GPIOs. They are programmable for input/output direction and (if output) for totem-pole vs open-drain drive. However, in this implementation we remove the programmability; so they're fixed function.

### GPIO Registers

Base address: 0x80000030.

| Offset  | Width | Name | Description |
|:-------:|:-----:|:-----|:------------|
| 0x0 | 32 | [`DATA`](#gpio-data) | Data to/from the GPIO registers |
| 0x4 | 32 | reserved | not implemented |
| 0x8 | 32 | reserved | Not implemented |
| 0xC | 32 | reserved | Not implemented |

#### GPIO Data

| Bit        | 31 .. 6  | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
|:----------:|:--------:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|
| **Name**   | reserved | `GPIO6` | `GPIO5` | `GPIO4` | `GPIO3`| `GPIO2` | `GPIO1` | `GPIO0` |
| **Reset**  |  0       |  0      |  0      |  0      | 0      |   0     |   1     |    0    |
| **Type**   | n/a      |  in     |  in     |  in     | in     |   in    | i/o od  |   out   |
| **Access** | r0wi     |  ro     |  ro     |  ro     | ro     |   ro    |  rpwd   |   rpwd  |

The GPIO bits are intended to be used as follows:

Bit name | LoRaWAN radio signal | Comments
:-------:|:--------------------:|:---------
`GPIO0`  | `CS`                 | chip select output to radio.
`GPIO1`  | `NRST`               | reset output to radio. open drain output, active low, set to 1 by reset.
`GPIO2`  | `DIO1`               | DIO1 output of radio, non-inverting input.
`GPIO3`  | `IRQ`                | Interrupt request from radio, non-inverting input.
`GPIO4`  | `DIO2`               | DIO2 output of radio, non-invering input.
`GPIO5`  | `SPARE4`             | Spare non-inverting input, connected to screw terminal JP3-1 on the LoRaWAN radio board.
`GPIO6`  | `SPARE5`             | Spare non-inverting input, connected to screw terminal JP3-2 on the LoRaWAN radio board.

Due to pin and board-design limitations, `GPIO5` and `GPIO6` are not available on the MDP implementation of this design, and will always read as zero. Instead, UART0 TX and RX are connected to `SPARE_C4` and `SPARE_C5`, repectively.

## SPI Interface

The SPI implementation is completely software driven; but it has a dedicated GPIO register to make programming easier and faster.

### SPI Registers

Base address: 0x80000040.

| Offset  | Width | Name | Description |
|:-------:|:-----:|:-----|:------------|
| 0x0 | 32 | [`DATA`](#spi-data) | Data to/from the pins registers |
| 0x4 | 32 | reserved | not implemented |
| 0x8 | 32 | reserved | Not implemented |
| 0xC | 32 | reserved | Not implemented |

#### SPI Data

| Bit        | 31 .. 3  | 2 | 1 | 0 |
|:----------:|:--------:|:-:|:-:|:-:|
| **Name**   | reserved | `SCK`   | `MOSI`  | `MISO`  |
| **Reset**  |  0       |   0     |   0     |    0    |
| **Type**   | n/a      |   out   |  out    |   in    |
| **Access** | r0wi     |   rpwd  |  rpwd   |   ro    |

SCK and MOSI are the classic SPI outputs (to the LoRaWAN radio, in this case). MISO is the classic SPI input. Software is responsible for bit-banging the interface. Software is also responsible for asserting `CS` (via the GPIO interface) before a SPI access to the radio, and clearing it afterwards.

## Cycle Counter

A free-running 32-bit counter is provided. It is incremented on every CPU clock cycle. Software can change the value by writing to the data register.

### Cycle Counter Registers

| Offset  | Width | Name | Description |
|:-------:|:-----:|:----:|:------------|
| 0x0 | 32 | [`DATA`](#cycle-counter-data-register) | Counter Data |
| 0x4 | 32 | reserved | Not implemented |
| 0x8 | 32 | reserved | Not implemented |
| 0xC | 32 | reserved | Not implemented |

### Cycle-Counter Data Register

| Bit | 31 ..  0 |
|:---:|:-------:|
| **Name** | `COUNT` |
| **Reset** |  0 |
| **Access** | rw |

This register is incremented by hardware on every CPU clock cycle. Software may change the value by writing to this register; but note that it will never read back exactly what it wrote, because the value is constantly incrementing.

## SOC Information

It's convenient for software to have information about the FPGA configuration and version, as well as about the hardware. So we have some registers with that info.

### SOCINFO Registers

### SOCINFO Data

## PDM Microphone

The PDM microphone subsystem captures data from the external PDM-encoded MEMS microphone, and converts the data to PCM audio. The external clock runs at 2 MHz, and data is integrated for 250 clock cycles, yielding an 8-bit sample at 8 kHz. A simple CPU loop can monitor the samples and send them to the PC over the UART.

### PDM Microphone Registers

| Offset  | Width | Name | Description |
|:-------:|:-----:|:----:|:------------|
| 0x0 | 32 | [`CTRL`](#pdm-microphone-control-register) | PDM Control |
| 0x4 | 32 | [`DATA`](#pdm-microphone-data-register) | PDM Data |
| 0x8 | 32 | [`CONFIG`](#pdm-microphone-config-register) | Configuration |
| 0xC | 32 | reserved | Not implemented |

### PDM Microphone Control Register

| Bit | 31 .. 16 | 15 .. 1 | 0  |
|:---:|:--------:|:-------:|:---|
| **Name**   | reserved | reserved | `RUN` |
| **Reset**  |  0 | 0 | 0 |
| **Access** | ro |r0wi | rw |

Software sets the `RUN` bit to start the microphone clock and start accumulating data. It clears the `RUN` bit to stop the clock and stop data acquisition.

### PDM Microphone Data Register

| Bit | 31 | 30 .. 16 | 15..0  |
|:---:|:--:|:-------:|:-----:|
| **Name**   | `RDY` | reserved | `PCMDATA` |
| **Reset**  |  0 | 0 | 0 |
| **Access** | rw1c | r0wi | ro |

Hardware sets the `RDY` bit whenever a new sample is delivered to the register. Software clears the `RDY` bit by writing a one to the bit.

Hardware accumulates `PCMDATA` via a hardware integration step. The most significant bit is bit 15; the least significant bit varies based on the configured precision.

### PDM Microphone Config Register

| Bit | 31 .. 16 | 15 .. 0  |
|:---:|:--------:|:--------:|
| **Name**   | `COUNTS` | `HERTZ` |
| **Reset**  |  375 | 8000 |
| **Access** | ro |ro |

This field reports the FPGA's configuration. As stated here, the default is 375 counts per sample period, with 8000 sample periods per second, for a sample clock of 3 MHz.
