##############################################################################
#
# Module:  Makefile
#
# Function:
#	Rules for creating the app.
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

include ${LATTICE_MK}setup.mk


##############################################################################
#
#	Rules for building cxx_test1
#
##############################################################################

PROGRAMS += cxx_test1

SOURCES_cxx_test1 := cxx_test1.cpp

# include paths
INCLUDES_cxx_test1 := $(LATTICE_PKGSRC)/arduino-lmic/src
INCLUDES_cxx_test1 += $(LATTICE_LIBRARIES)/mcci-lmic-riscv/i
INCLUDES_cxx_test1 += $(LATTICE_PKGSRC)/arduino-lmic/src
INCLUDES_cxx_test1 += $(MCCIXDK_INCLUDES)
INCLUDES_cxx_test1 += $(LATTICE_LIBRARIES)/sensor_bmp180/i

CXXFLAGS_cxx_test1 +=

CPPFLAGS_cxx_test1 += -DMDP_SERIAL=$(_MDP_SERIAL) \
			-DARDUINO_LMIC_PROJECT_CONFIG_H=mcci_lmic_riscv_config.h

ifdef LMIC_CFG_region
CPPFLAGS_cxx_test1 +=	-DLMIC_CFG_region=${LMIC_CFG_region}
endif

# required libraries
LIBS_cxx_test1 :=							\
	$(LATTICE_BSP_LIB)						\
	$(LATTICE_LIBRARIES)/mccixdk-riscv/$(T_OBJDIR)/libmccixdk-riscv.a \
	$(LATTICE_BSP_ROOT)/$(T_OBJDIR)/libriscv_ino.a \
# end LIBS_cxx_test1

# the libraries from outside the build system
LDADD_cxx_test1 :=				\
	# none

# the map file
LDFLAGS_cxx_test1 = -Map $(T_OBJDIR)/cxx_test1.map

#### common framework post-processing:
include ${LATTICE_MK}tail.mk

#### end of Makefile ####
