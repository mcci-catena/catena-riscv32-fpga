/*

Module:  i2c_scan.c

Function:
	Simple i2c scan utility.

Author:
	Terry Moore, MCCI Corporation

Copyright Notice and License Information:
	Copyright 2019 MCCI Corporation

	This file is part of the MCCI Catena RISC-V FPGA core,
	https://github.com/mcci-catena/catena-riscv32-fpga.

	catena-risc32v-fpga is free software: you can redistribute it
	and/or modify it under the terms of the GNU General Public License
	as published by the Free Software Foundation, either version 3 of
	the License, or (at your option) any later version.

	catena-risc32v-fpga is distributed in the hope that it will
	be useful, but WITHOUT ANY WARRANTY; without even the implied
	warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
	See the GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <https://www.gnu.org/licenses/>.

*/

#include "riscv_ino.h"
#include "i2c.h"
#include "log.h"

void
setup(void)
	{
	unsigned nMatches;
	char match[I2C_ADDRESS_GENERAL_END - I2C_ADDRESS_GENERAL_START + 1];

	log_printf("Scanning i2c bus:\n");
	i2c_begin();

	nMatches = 0;
	for (uint8_t addr = I2C_ADDRESS_GENERAL_START;
	     addr < I2C_ADDRESS_GENERAL_END + 1; ++addr)
		{
		log_printf("%02x ", addr);
		log_flush();

		bool fProbed = i2c_probe(addr, /* fRead */ true);

		if (fProbed)
			{
			++nMatches;
			log_printf("\nMatched %02x\n", addr);
			log_flush();
			}

		match[addr - I2C_ADDRESS_GENERAL_START] = fProbed;
		}
	log_printf("\n\n");

	/* now, display a summary */
	if (nMatches == 0)
		{
		log_printf("No devices found\n");
		}
	else
		{
		log_printf(
			"%d device%s were found at the following address%s:\n",
			nMatches,
			nMatches != 1 ? "s": "",
			nMatches != 1 ? "es" : ""
			);

		for (unsigned i = 0; i < sizeof(match); ++i)
			{
			if (match[i])
				{
				log_printf("%02x ", i + I2C_ADDRESS_GENERAL_START);
				}
			}

		log_printf("\n");
		}

	log_printf("\n");
	}

void
loop(void)
	{
	log_printf("nothing to do in loop()\n");
	log_flush();
	while(1);
	}
