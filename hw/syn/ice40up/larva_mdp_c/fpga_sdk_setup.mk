##############################################################################
#
# Module:  fpga_sdk_setup.mk
#
# Function:
#	GNU makefile fragment -- sets up for larva_mdp_c
#
# Author:
#	Terry Moore, MCCI Corporation
#
# Copyright Notice and License Information:
#	Copyright 2019 MCCI Corporation
#
#	This file is part of the MCCI Catena RISC-V FPGA core,
#	https://github.com/mcci-catena/catena-riscv32-fpga.
#
#	catena-risc32v-fpga is free software: you can redistribute it
#	and/or modify it under the terms of the GNU General Public License
#	as published by the Free Software Foundation, either version 3 of
#	the License, or (at your option) any later version.
#
#	catena-risc32v-fpga is distributed in the hope that it will
#	be useful, but WITHOUT ANY WARRANTY; without even the implied
#	warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
#	See the GNU General Public License for more details.
#
#	You should have received a copy of the GNU General Public License
#	along with this program.  If not, see <https://www.gnu.org/licenses/>.
#
##############################################################################

PROJECT_TOPLEVEL ?= lora_mdp_c_top
PROJECT_FLASH	 ?= M25P80
PROJECT_PORT	 ?= 0

#### end of file ####
