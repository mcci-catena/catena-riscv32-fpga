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

include ${LATTICE_MK}mdp_serial.mk


##############################################################################
#
#	Rules for building test_face
#
##############################################################################

PROGRAMS += test_face

SOURCES_test_face := test_face.c

INCLUDES_test_face += $(LATTICE_LIBRARIES)/facedetect/i

LDFLAGS_test_face = -Map $(T_OBJDIR)/test_face.map

LIBS_test_face :=							\
	$(LATTICE_BSP_LIB)						\
	$(LATTICE_LIBRARIES)/mccixdk-riscv/$(T_OBJDIR)/libmccixdk-riscv.a \
	$(LATTICE_BSP_ROOT)/$(T_OBJDIR)/libriscv_ino.a 			\
	$(LATTICE_LIBRARIES)/facedetect/${T_OBJDIR}/libfacedetect.a	\
### end of LIBS_test_face

##############################################################################
#
#	Rules for building ttn_face
#
##############################################################################

PROGRAMS += ttn_face

SOURCES_ttn_face := ttn_face.c

# include paths
INCLUDES_ttn_face := $(LATTICE_PKGSRC)/arduino-lmic/src
INCLUDES_ttn_face += $(LATTICE_LIBRARIES)/mcci-lmic-riscv/i
INCLUDES_ttn_face += $(MCCIXDK_INCLUDES)
INCLUDES_ttn_face += $(LATTICE_LIBRARIES)/facedetect/i
INCLUDES_ttn_face += $(LATTICE_LIBRARIES)/cayenne-lpp/i

CPPFLAGS_ttn_face += -DMDP_SERIAL=$(_MDP_SERIAL) \
			-DARDUINO_LMIC_PROJECT_CONFIG_H=mcci_lmic_riscv_config.h

ifdef LMIC_CFG_region
CPPFLAGS_ttn_face +=	-DLMIC_CFG_region=${LMIC_CFG_region}
endif

# required libraries
LIBS_ttn_face :=							\
	$(LATTICE_BSP_LIB)						\
	$(LATTICE_LIBRARIES)/mcci-lmic-riscv/$(T_OBJDIR)/libmcci-lmic-riscv.a	\
	$(LATTICE_LIBRARIES)/mccixdk-riscv/$(T_OBJDIR)/libmccixdk-riscv.a \
	$(LATTICE_LIBRARIES)/facedetect/${T_OBJDIR}/libfacedetect.a	\
	$(LATTICE_BSP_ROOT)/$(T_OBJDIR)/libriscv_ino.a \
# end LIBS_ttn_face

# the libraries from outside the build system
LDADD_ttn_face :=				\
	# none

# the map file
LDFLAGS_ttn_face = -Map $(T_OBJDIR)/ttn_face.map

# add another dependency so that we'll recompile ttn_face if serial number
$(T_OBJDIR)/ttn_face.o : $(T_MDP)


#### common framework post-processing:
include ${LATTICE_MK}tail.mk

#### end of Makefile ####
