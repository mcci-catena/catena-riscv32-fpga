/*

Module:  ttn_otaa_ino.c

Function:
	A simple Arduino-like hello-world for Risc-V/iCE40 Ultra Plus/LoRaWAN

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

#include "log.h"

#include "arduino_lmic.h"

/*******************************************************************************
 * Based on the arduino-lmic ttn_otaa app.
 *
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example sends a valid LoRaWAN packet with payload "Hello,
 * world!", using frequency and encryption settings matching those of
 * the The Things Network.
 *
 * This uses OTAA (Over-the-air activation), where where a DevEUI and
 * application key is configured, which are used in an over-the-air
 * activation procedure where a DevAddr and session keys are
 * assigned/generated for use with all further communication.
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
 * violated by this sketch when left running for longer)!

 * To use this sketch, first register your application and device with
 * the things network, to set or generate an AppEUI, DevEUI and AppKey.
 * Multiple devices can use the same AppEUI, but each device has its own
 * DevEUI and AppKey.
 *
 * Do not forget to define the radio type correctly in config.h.
 *
 *******************************************************************************/


// you must replace the values marked FILLMEIN with registration values
// obtained from the TTN console!
#define FILLMEIN 0

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. When copying from the TTN console, this means to select LSB
// mode prior to copying.
static const uint8_t DEVEUI[8]= { FILLMEIN };

// This should also be in little endian format, see above.
static const uint8_t APPEUI[8]= { FILLMEIN };

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// other words, use the "msb" format to copy the key from the TTN console.
static const uint8_t APPKEY[16] = { FILLMEIN };

static uint8_t mydata[] = "Hello, world!";
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 60;

void do_send(osjob_t* j);

void os_getArtEui(u1_t* buf)
	{
	memcpy(buf, APPEUI, 8);
	}

void os_getDevEui(u1_t* buf)
	{
	memcpy(buf, DEVEUI, 8);
	}

void os_getDevKey(u1_t* buf)
	{
	memcpy(buf, APPKEY, 16);
	}

static void
printHexBuf(
	const uint8_t *pBuf,
	size_t nBuf
	)
	{
	for (unsigned i=0; i < nBuf; ++i)
		{
		log_printf("%02x", pBuf[i]);
		}
	}

static void
log_printev(
	const char *pEvName
	)
	{
	log_printf("%6lu: EV_%s\n", os_getTime(), pEvName);
	}

void
onEvent(
	ev_t ev
	)
	{
	switch(ev)
		{
	case EV_SCAN_TIMEOUT:
		log_printev("SCAN_TIMEOUT");
		break;
	case EV_BEACON_FOUND:
		log_printev("BEACON_FOUND");
		break;
	case EV_BEACON_MISSED:
		log_printev("BEACON_MISSED");
		break;
	case EV_BEACON_TRACKED:
		log_printev("BEACON_TRACKED");
		break;
	case EV_JOINING:
		log_printev("JOINING");
		break;
	case EV_JOINED:
		log_printev("JOINED");
			{
			uint32_t netid = 0;
			devaddr_t devaddr = 0;
			uint8_t NwkSKey[16];
			uint8_t AppSKey[16];

			LMIC_getSessionKeys(
				&netid, &devaddr, NwkSKey, AppSKey
				);

			log_printfnl("netid: %lu", netid);
			log_printfnl("devaddr: %08lx\n", devaddr);
			log_puts("AppSKey: ");
			printHexBuf(AppSKey, sizeof(AppSKey));
			log_puts("\nNwkSKey: ");
			printHexBuf(NwkSKey, sizeof(NwkSKey));
			log_puts("\n");
			}

		// Disable link check validation (automatically enabled
		// during join, but not supported by TTN at this time).
		LMIC_setLinkCheckMode(0);
		break;

	case EV_RFU1:
		log_printev("RFU1");
		break;

	case EV_JOIN_FAILED:
		{
		static unsigned nJoinFail = 0;

		++nJoinFail;
		log_printf("%6lu: EV_JOIN_FAILED (%u)\n", os_getTime(), nJoinFail);
		}
		break;

	case EV_REJOIN_FAILED:
		log_printev("REJOIN_FAILED");
		break;

	case EV_TXCOMPLETE:
		log_printfnl(
			"%6lu: EV_TXCOMPLETE "
			"(includes waiting for RX windows)",
			os_getTime()
			);

		if (LMIC.txrxFlags & TXRX_ACK)
			log_printfnl("Received ack");

		if (LMIC.dataLen)
			{
			log_printfnl(
				"Received %u bytes of payload",
				LMIC.dataLen
				);
			}

		// Schedule next transmission
		os_setTimedCallback(
			&sendjob,
			os_getTime()+sec2osticks(TX_INTERVAL),
			do_send
			);
		break;

	case EV_LOST_TSYNC:
		log_printev("LOST_TSYNC");
		break;

	case EV_RESET:
		log_printev("RESET");
		break;
	case EV_RXCOMPLETE:
		// data received in ping slot
		log_printev("RXCOMPLETE");
		break;
	case EV_LINK_DEAD:
		log_printev("LINK_DEAD");
		break;
	case EV_LINK_ALIVE:
		log_printev("LINK_ALIVE");
		break;
	case EV_TXSTART:
		{
		static unsigned nTxStart = 0;

		++nTxStart;
		log_printf("%6lu: EV_TXSTART (%u) ch %d\n",
				os_getTime(), nTxStart, LMIC.txChnl
				);
		}
		break;
	default:
		log_printfnl("Unknown event");
		break;
		}
	}

void do_send(osjob_t* j)
	{
	// Check if there is not a current TX/RX job running
	if (LMIC.opmode & OP_TXRXPEND)
		{
		log_printfnl("OP_TXRXPEND, not sending");
		}
	else
		{
		// Prepare upstream data transmission at the next possible
		// time.  Send on port 1.
		LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);
		log_printfnl("Packet queued");
		}

	// Next TX is scheduled after TX_COMPLETE event.
	}

static bool
checkNonZero(const uint8_t *pBuffer, size_t nBuffer)
	{
	for (; nBuffer != 0; ++pBuffer, --nBuffer)
		{
		if (pBuffer != 0)
			return true;
		}
	return false;
	}

static void
log_lebuf(const char *pPrefix, const uint8_t *pBuffer, size_t nBuffer)
	{
	for (; nBuffer != 0; --nBuffer)
		{
		log_printf("%s%02x", pPrefix, pBuffer[nBuffer - 1]);
		pPrefix = "-";
		}
	log_puts("\n");
	}

static void
log_bebuf(const char *pPrefix, const uint8_t *pBuffer, size_t nBuffer)
	{
	for (; nBuffer != 0; ++pBuffer, --nBuffer)
		{
		log_printf("%s%02x", pPrefix, *pBuffer);
		pPrefix = "-";
		}
	log_puts("\n");
	}

void setup()
	{
	log_puts("\n"
		   "================================\n"
		   "ttn_otaa_ino: " __DATE__ " "__TIME__ "\n"
		   "================================\n\n"
		   );
	log_flush();

	log_printf("Provisioning info:\n");
	log_lebuf("DEVEUI: ", DEVEUI, sizeof(DEVEUI));
	log_lebuf("APPEUI: ", APPEUI, sizeof(APPEUI));
	log_bebuf("APPKEY: ", APPKEY, sizeof(APPKEY));

	// verify that we actually have non-zero DEVEUI, APPEUI, APPKEY
	if (! (checkNonZero(DEVEUI, sizeof(DEVEUI)) &&
	       checkNonZero(APPEUI, sizeof(APPEUI)) &&
	       checkNonZero(APPKEY, sizeof(APPKEY))))
		{
		log_printf("FATAL: registration info must be non-zero!.\n");
		log_flush();
		while (1);
		}


	// LMIC init
	os_init_ex(NULL);

	// Reset the MAC state. Session and pending data transfers will be
	// discarded.
	LMIC_reset();

	// clock error means many things, basically a measure of delay and latency
	// we allocate a generous error because the spi interface is slow, which
	// messes up in-built latency calculations.
	LMIC_setClockError(MAX_CLOCK_ERROR * 5 / 100);

	// TTN doesn't support link check mode.
	LMIC_setLinkCheckMode(0);

	// set the default data rate and power.
	LMIC_setDrTxpow(DR_SF7,14);

	// set subband 2
	LMIC_selectSubBand(1);

	// Start job (sending automatically starts OTAA too)
	do_send(&sendjob);
	}

void loop()
	{
	os_runloop_once();
	}
