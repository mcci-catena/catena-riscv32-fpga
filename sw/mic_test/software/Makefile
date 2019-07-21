##############################################################################
#
# Module:  Makefile
#
# Function:
#	Makefile for mic_test
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

include ${LATTICE_MK}setup.mk

##############################################################################
#
#	Components of mic_test
#
##############################################################################

PROGRAMS += mic_test

#
# the source files
#
SOURCES_mic_test =						\
	mic_test.c						\
### end SOURCES

#
# the input include directories
#
INCLUDES_mic_test =						\
	${RISCV_INO_INCLUDES}					\
#### end INCLUDES

#
# The input libraries, added to dependencies
#
LIBS_mic_test =							\
	$(RISCV_INO_LIBS)					\
#### end LIBS_mic_test

include $(LATTICE_MK)tail.mk

#### end of Makefile ####
