##############################################################################
#
# Module:  Makefile
#
# Function:
#	Makefile for lorawan_raw application.
#
# Author:
#	Terry Moore, MCCI Corporation
#
# Copyright notice:
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

# set up the framework.
include ${LATTICE_MK}setup.mk

##############################################################################
#
#	lorawan_raw
#
##############################################################################

PROGRAMS += lorawan_raw

SOURCES_lorawan_raw := lorawan_raw.c

INCLUDES_lorawan_raw := $(LATTICE_PKGSRC)/arduino-lmic/src
INCLUDES_lorawan_raw += $(LATTICE_LIBRARIES)/mcci-lmic-riscv/i
INCLUDES_lorawan_raw += $(MCCIXDK_SRC)/i $(MCCIXDK_SRC)/lib/base/i

LIBS_lorawan_raw :=							\
	$(LATTICE_BSP_LIB)						\
	$(LATTICE_LIBRARIES)/mcci-lmic-riscv/$(T_OBJDIR)/libmcci-lmic-riscv.a	\
	$(LATTICE_LIBRARIES)/mccixdk-riscv/$(T_OBJDIR)/libmccixdk-riscv.a \
	$(LATTICE_BSP_ROOT)/$(T_OBJDIR)/libriscv_ino.a \
# end LIBS_lorawan_raw


LDADD_lorawan_raw :=				\
	# none


#
# finish setting up the build.
#
include $(LATTICE_MK)tail.mk

#### end of makefile ####
