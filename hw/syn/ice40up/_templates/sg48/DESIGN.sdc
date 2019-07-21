//
// Module:  DESIGN.sdc
//
// Function:
//	Clock net .sdc file template -- expand with ../new-design.sh.
//
//	DESIGN is the name of the target design.
//
// Author:
//	Terry Moore, MCCI Corporation
//
// Copyright Notice and License Information:
//	Copyright 2019 MCCI Corporation
//
//	This file is part of the MCCI Catena RISC-V FPGA core,
//	https://github.com/mcci-catena/catena-riscv32-fpga.
//
//	catena-risc32v-fpga is free software: you can redistribute it
//	and/or modify it under the terms of the GNU General Public License
//	as published by the Free Software Foundation, either version 3 of
//	the License, or (at your option) any later version.
//
//	catena-risc32v-fpga is distributed in the hope that it will
//	be useful, but WITHOUT ANY WARRANTY; without even the implied
//	warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
//	See the GNU General Public License for more details.
//
//	You should have received a copy of the GNU General Public License
//	along with this program.  If not, see <https://www.gnu.org/licenses/>.
//
create_clock -name clk_i -period 100 [get_nets clk_i]
