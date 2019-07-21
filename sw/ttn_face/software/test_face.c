/*

Module:  test_face.c

Function:
	Test app for face sensing application for iCE40 Ultra Plus MDP / Risc-V

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

#include "facedetect.h"
#include "log.h"

#include <stdbool.h>

/****************************************************************************\
|
|	Manifest constants & typedefs.
|
\****************************************************************************/

static void
setup_sensors(void);

/****************************************************************************\
|
|	Read-only data.
|
\****************************************************************************/



/****************************************************************************\
|
|	VARIABLES:
|
\****************************************************************************/

bool g_fLastFaceDetect;
unsigned g_nFaces;

/*

Name:	setup()

Function:
	Arduino-like setup for this app.

Definition:
	#include "riscv_ino.h"
	void setup(void);

Description:
	This routine is called once from the main loop during initialization.
	We set up the LoRaWAN connection and set up the sensors to be
	measured; and we arrange to periodically send data uplink.

Returns:
	No explicit result.

*/

static const char dash59[] = "===========================================================";
#define dash14 (dash59 + sizeof(dash59) - 1 - 14)

void
setup(void)
	{
	log_printfnl("\n"
		   "%s\n"
		   "\n"
		   "test_face: " __DATE__ " "__TIME__ "\n"
		   "\n"
		   "%s\n",
		   dash59,
		   dash59
		   );
	log_flush();

	setup_sensors();
	}

/****************************************************************************\
|
|	Setup the face detector
|
\****************************************************************************/

static void
setup_sensors(void)
	{
	// there's nothing to do for I/O
	g_fLastFaceDetect = false;
	}

/*

Name:	loop()

Function:
	The polling routine, called continually from the main loop.

Definition:
	#include "riscv_ino.h"
	void loop(void);

Description:
	In this simple application, we use the LMIC facilities to drive
	things forward. So the only thing loop has to do is call the LMIC
	polling routine (which will end up calling everything else).

Returns:
	No explicit result.

*/

void loop()
	{
	bool const fIsPresent = facedetect_IsFacePresent();

	if (fIsPresent != g_fLastFaceDetect)
		{
		g_fLastFaceDetect = fIsPresent;

		if (fIsPresent)
			{
			++g_nFaces;
			log_printf("** Face Present: %u **\n", g_nFaces);
			}
		else
			{
			log_printf("-- departed --\n");
			}
		}
	}
