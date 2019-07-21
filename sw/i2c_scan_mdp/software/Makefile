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
#	Components of i2c_scan_mdp
#
##############################################################################

PROGRAMS += i2c_scan_mdp

#
# the source files
#
SOURCES_i2c_scan_mdp =						\
	i2c_scan_mdp.c						\
### end SOURCES

#
# the input include directories
#
INCLUDES_i2c_scan_mdp =						\
	${RISCV_INO_INCLUDES}					\
	${LATTICE_LIBRARIES}/sensor_bmp180/i			\
#### end INCLUDES

#
# The input libraries, added to dependencies
#
LIBS_i2c_scan_mdp =							\
	$(RISCV_INO_LIBS)					\
	${LATTICE_LIBRARIES}/sensor_bmp180/${T_OBJDIR}/libsensor_bmp180.a \
#### end LIBS_i2c_scan_mdp

include $(LATTICE_MK)tail.mk

#### end of Makefile ####
