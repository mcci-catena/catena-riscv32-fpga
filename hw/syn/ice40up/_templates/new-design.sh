#!/bin/bash

##############################################################################
#
# Module:  new-design.sh
#
# Function:
#	Create a new design from a template.
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

BOARD=
DESIGN=
TOPLEVEL=
PACKAGE=sg48

while getopts b:d:t:p: c
do
	case "$c" in
	b)	BOARD="$OPTARG";;
	d)	DESIGN="$OPTARG";;
	t)	TOPLEVEL="$OPTARG";;
	p)	PACKAGE="$OPTARG";;
	*)	echo "Usage: -b BOARD -d DESIGN -t TOPLEVEL -p PACKAGE" 1>&2
		exit 1;;
	esac
done

if [ X"$DESIGN" = X ]; then
  echo "Design (-d) not specified" 1>&2
  exit 1
fi

if [ X"$BOARD" = X ]; then
  echo "Board (-b) not specified" 1>&2
  exit 1
fi

if [ X"$TOPLEVEL" = X ]; then
  echo "Top-level Verilog module (-t) not specified" 1>&2
  exit 1
fi

if [ ! -f "../../src/boards/${BOARD}/${TOPLEVEL}.v" ]; then
  echo "Can't find top-level: boards/$BOARD/$TOPLEVEL" 1>&2
  exit 1
fi

if [ ! -f _templates/"$PACKAGE"/DESIGN_sbt.project ]; then
  echo "Package (-p) not recognized: $PACKAGE" 1>&2
  exit 1
fi

function _filter {
    # shellcheck disable=SC2016
    sed -e '\;^//;d' -e 's/${DESIGN}/'"$DESIGN/g"  -e 's/${TOPLEVEL}/'"$TOPLEVEL/g" -e 's/${BOARD}/'"$BOARD/g" "$@"
}

if [ -d "${DESIGN}" ]; then
  echo "design already exists: $DESIGN" 1>&2
  exit 1
fi

mkdir -p "${DESIGN}" && \
  for i in "_templates/${PACKAGE}/DESIGN"* _templates/fpga_sdk_setup.mk ; do
    j=$(basename "$i" | sed -e 's;DESIGN;'"${DESIGN};")
    _filter "$i" >"${DESIGN}/$j"
  done
