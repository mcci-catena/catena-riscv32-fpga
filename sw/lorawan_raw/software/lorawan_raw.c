/*

Module:  lorawan_raw.c

Function:
	Raw commmunication node-to-node. Based on arduino-lmic/examples/raw.

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

/*******************************************************************************
 * Copyright (c) 2015 Matthijs Kooijman
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example transmits data on hardcoded channel and receives data
 * when not transmitting. Running this sketch on two nodes should allow
 * them to communicate.
 *******************************************************************************/

#include "riscv_ino.h"
#include "log.h"
#include "arduino_lmic.h"

// How often to send a packet. Note that this sketch bypasses the normal
// LMIC duty cycle limiting, so when you change anything in this sketch
// (payload length, frequency, spreading factor), be sure to check if
// this interval should not also be increased.
// See this spreadsheet for an easy airtime and duty cycle calculator:
// https://docs.google.com/spreadsheets/d/1voGAtQAjC1qBmaVuP1ApNKs1ekgUjavHuVQIXyYSvNc
#define TX_INTERVAL 2000

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

void onEvent (ev_t ev) {
}

osjob_t txjob;
osjob_t timeoutjob;
static void tx_func (osjob_t* job);
void tx(const char *str, osjobcb_t func);
void rx(osjobcb_t func);

// Transmit the given string and call the given function afterwards
void tx(const char *str, osjobcb_t func) {
  os_radio(RADIO_RST); // Stop RX first
  delay(1); // Wait a bit, without this os_radio below asserts, apparently because the state hasn't changed yet
  LMIC.dataLen = 0;
  while (*str)
    LMIC.frame[LMIC.dataLen++] = *str++;
  LMIC.osjob.func = func;
  os_radio(RADIO_TX);
  log_printf("TX ");
}

// Enable rx mode and call func when a packet is received
void rx(osjobcb_t func) {
  LMIC.osjob.func = func;
  LMIC.rxtime = os_getTime(); // RX _now_
  // Enable "continuous" RX (e.g. without a timeout, still stops after
  // receiving a packet)
  os_radio(RADIO_RXON);
  log_printf("RX ");
}

static void rxtimeout_func(osjob_t *job) {
  log_printf(" ");
}

static void rx_func (osjob_t* job) {
  // Originally: Blink once to confirm reception and then keep the led on

  // Timeout RX (i.e. update led status) after 3 periods without RX
  os_setTimedCallback(&timeoutjob, os_getTime() + ms2osticks(3*TX_INTERVAL), rxtimeout_func);

  // Reschedule TX so that it should not collide with the other side's
  // next TX
  os_setTimedCallback(&txjob, os_getTime() + ms2osticks(TX_INTERVAL/2), tx_func);

  log_printf("\nGot %u bytes\n", LMIC.dataLen);

  void (*const pPrintf)(const char *, ...) = log_printf;

  (*pPrintf)("%b\n", LMIC.frame, LMIC.dataLen);

  // Restart RX
  rx(rx_func);
}

static void txdone_func (osjob_t* job) {
  rx(rx_func);
}

// log text to USART and toggle LED
static void tx_func (osjob_t* job) {
  // say hello
  tx("Hello, world!", txdone_func);
  // reschedule job every TX_INTERVAL (plus a bit of random to prevent
  // systematic collisions), unless packets are received, then rx_func
  // will reschedule at half this time.

  // we don't need a lot of randomness here, so we can use a single byte
  // and map onto the 0..499 range.
  uint32_t randval = os_getRndU1();
  randval = (randval * 500) >> 8;

  os_setTimedCallback(job, os_getTime() + ms2osticks(TX_INTERVAL + randval), tx_func);
}

// application entry point
void setup() {
  log_printf("Starting\n");
  log_flush();

  // initialize runtime env
  delay(5);

  while (! os_init_ex(NULL))
	{
	log_printf("radio failed to initialize!\n");
	log_flush();
	delay(2000);
	}

  // Set up these settings once, and use them for both TX and RX

#if defined(CFG_eu868)

  // Use a frequency in the g3 which allows 10% duty cycling.
  LMIC.freq = 869525000;
  // Use a medium spread factor. This can be increased up to SF12 for
  // better range, but then the interval should be (significantly)
  // lowered to comply with duty cycle limits as well.
  LMIC.datarate = DR_SF10;

  // Maximum TX power
  LMIC.txpow = 27;

#elif defined(CFG_us915)
  const static bool fDownlink = true;
  const static uint8_t kDownlinkChannel = 3;
  const static uint8_t kUplinkChannel = 8 + 3;
  uint32_t uBandwidth;

  if (! fDownlink)
	{
	if (kUplinkChannel < 64)
		{
		LMIC.freq = US915_125kHz_UPFBASE +
			    kUplinkChannel * US915_125kHz_UPFSTEP;
		uBandwidth = 125;
		}
	else
		{
		LMIC.freq = US915_500kHz_UPFBASE +
			    (kUplinkChannel - 64) * US915_500kHz_UPFSTEP;
		uBandwidth = 500;
		}
	}
  else
	{
	// downlink channel
	LMIC.freq = US915_500kHz_DNFBASE +
		    kDownlinkChannel * US915_500kHz_DNFSTEP;
	uBandwidth = 500;
	}

  // Use a suitable spreading factor
  if (uBandwidth < 500)
        LMIC.datarate = DR_SF10;        // DR0
  else
        LMIC.datarate = DR_SF12CR;      // DR8

  // default tx power for US: 21 dBm
  LMIC.txpow = 21;
#else
# error unsupported confguration CFG_...
#endif

  // disable RX IQ inversion
  LMIC.noRXIQinversion = true;

  // This sets CR 4/5, BW125 (except for DR_SF7B, which uses BW250)
  LMIC.rps = updr2rps(LMIC.datarate);

  log_printf(
	"Frequency: %lu.%lu MHz  LMIC.datarate: %d  Tx Power: %d\n",
	(LMIC.freq / 1000000u),
	(LMIC.freq /  100000u) % 10,
	LMIC.datarate,
	LMIC.txpow
	);

  log_printf("Started\n");
  log_flush();

  // setup initial job
  os_setCallback(&txjob, tx_func);
}

void loop() {
  // execute scheduled jobs and events
  os_runloop_once();
}

