##############################################################################
#
# Module:  Makefile
#
# Function:
#	GNU makefile for simple FPGA operations using APIO
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

VERILATOR = verilator
IVERILOG= iverilog

# the path to the ICE40 tech files
APIO_HOME = ${HOME}/apio

TOP_MODULE = catena4710_top

TECH_INCLUDES =		\
	${APIO_HOME}/packages/toolchain-verilator/share

# the path to the source tree
SRCDIR = 	../../../src/

# where to put our test results
TESTDIR =	icestorm/test/

# the project include directories
PROJECT_INCLUDES =	\
	lib/i		\
# end PROJECT_INCLUDES

# the project sources
PROJECT_SRC = 								\
	lib/riscv32i_v6T/riscv32.v					\
	boards/Catena4710/catena4710_top.v				\
	lib/common/irom_loader.v					\
	lib/common/simple_uart.v					\
	lib/common/simple_uart_rx.v					\
	lib/pdm_audio/pdm_audio.v					\
	lib/pdm_audio/pdm_audio_clk.v					\
	lib/pdm_audio/pdm_audio_data.v					\
	lib/pdm_audio/pdm_filter.v					\
# end PROJECT_SRC

#### lint the project ###
lint:
	${VERILATOR} --lint-only --relative-includes \
		${patsubst %,--top-module %,${TOP_MODULE}} \
		${patsubst %,-I%,$(abspath ${TECH_INCLUDES})} \
		$(patsubst %,-I%,$(abspath $(addprefix ${SRCDIR},${PROJECT_INCLUDES}))) \
		$(abspath $(addprefix ${SRCDIR},${PROJECT_SRC}))

#### compile the audio test -- this is not yet implemented ####
${TESTDIR}pdm_audio_tb: $(abspath ${addprefix ${SRCDIR}lib/pdm_audio/,pdm_audio_tb.v}) $(addprefix ${SRCDIR},${PROJECT_SRC})
	# make sure we have the output directory
	@if [ X"${TESTDIR}" != X ]; then test -d ${TESTDIR} || mkdir -p ${TESTDIR} ; fi
	# compile the simulation
	${IVERILOG} -grelative-include ${patsubst $,-s,$@} \
		-o $@
		${patsubst %, -I%,${abspath ${TECH_INCLUDES}}} \
		$(patsubst %,-I%,$(abspath $(addprefix ${SRCDIR},${PROJECT_INCLUDES}))) \
		$(patsubst %,-l%,$(abspath $(addprefix ${SRCDIR},${PROJECT_SRC}))) \
		@<

#### run the audio test ####
pdm_audio_test: ${TESTDIR}pdm_audio_tb
	cd $(dir $<) && ./$(notdir $<)

.PHONY: pdm_audio_test lint
