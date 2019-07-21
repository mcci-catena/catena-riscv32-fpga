/*

Module:  mic_test.c

Function:
	Capture microphone data and send to UART

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
#include "ice40up_larva.h"
#include "hal.h"
#include "debug.h"

MCCIXDK_FORCEINLINE static
uint32_t mask2lsb(uint32_t mask)
	{
	return mask & (~mask + 1);
	}

MCCIXDK_FORCEINLINE static
uint32_t getfield(uint32_t v, uint32_t mask)
	{
	return (v & mask) / mask2lsb(mask);
	}

MCCIXDK_FORCEINLINE static
uint32_t setfield(uint32_t v, uint32_t mask, uint32_t f)
	{
	return (v & ~mask) | (f * mask2lsb(mask));
	}

MCCIXDK_FORCEINLINE static
uint32_t getmicreg32(uint32_t r)
	{
	return *(volatile uint32_t *)(ICE40UP_LARVA_MIC + r);
	}

MCCIXDK_FORCEINLINE static
uint16_t getmicreg16(uint32_t r)
	{
	return *(volatile uint16_t *)(ICE40UP_LARVA_MIC + r);
	}

MCCIXDK_FORCEINLINE static
uint8_t getmicreg8(uint32_t r)
	{
	return *(volatile uint8_t *)(ICE40UP_LARVA_MIC + r);
	}

MCCIXDK_FORCEINLINE static
void putmicreg32(uint32_t r, uint32_t v)
	{
	*(volatile uint32_t *)(ICE40UP_LARVA_MIC + r) = v;
	}

void log_report_assert_fail(const char *pMsg, const char *pFile, unsigned line);
int16_t highpass(int32_t sample);
char base64_encode(uint8_t sextet);
void base64_put8(uint8_t thisByte);
void base64_flush(void);
void test_filter(void);

void log_report_assert_fail(const char *pMsg, const char *pFile, unsigned line)
	{
	log_printf("**Test Failure** %s:%u: %s\n", pFile, line, pMsg);
	}

#define log_assert(e) 	\
	((e) ? true : (log_report_assert_fail(#e, __FILE__, __LINE__), false))

uint32_t g_dataStartTime;
uint32_t g_clocksPerSample;
bool g_highpassdebug;
uint32_t g_lg2gain = 0;


void
setup(void)
	{
	log_printf("mic_test.\n");

	/* read and print the  count and hertz registers */
	uint32_t const rConfig = getmicreg32(ICE40UP_LARVA_MIC_CONFIG);
	uint16_t const micHz = getfield(rConfig, ICE40UP_LARVA_MIC_CONFIG_HERTZ);
	uint16_t const micCount = getfield(rConfig, ICE40UP_LARVA_MIC_CONFIG_COUNTS);
	uint8_t b0, b1, b2, b3;
	uint16_t w0, w1;

	b0 = getmicreg8(ICE40UP_LARVA_MIC_CONFIG + 0);
	b1 = getmicreg8(ICE40UP_LARVA_MIC_CONFIG + 1);
	b2 = getmicreg8(ICE40UP_LARVA_MIC_CONFIG + 2);
	b3 = getmicreg8(ICE40UP_LARVA_MIC_CONFIG + 3);

	w0 = getmicreg16(ICE40UP_LARVA_MIC_CONFIG + 0);
	w1 = getmicreg16(ICE40UP_LARVA_MIC_CONFIG + 2);

	log_printf(
		"CONFIG (@%08lx): %08lx  counts=%d  hz=%d\n"
		"       bytes: %02x %02x %02x %02x  words %04x %04x\n\n",
		(uint32_t)ICE40UP_LARVA_MIC + ICE40UP_LARVA_MIC_CONFIG,
		rConfig, micCount, micHz,
		b0, b1, b2, b3,
		w0, w1
		);

	(void) log_assert(getfield(rConfig, 0xFFu     ) == b0);
	(void) log_assert(getfield(rConfig, 0xFFu << 8) == b1);
	(void) log_assert(getfield(rConfig, 0xFFu << 16) == b2);
	(void) log_assert(getfield(rConfig, 0xFFu << 24) == b3);
	(void) log_assert(getfield(rConfig, 0xFFFFu << 0) == w0);
	(void) log_assert(getfield(rConfig, 0xFFFFu << 16) == w1);

	uint32_t rCtrl = getmicreg32(ICE40UP_LARVA_MIC_CTRL);
	log_printf(
		"CTRL (@%08lx): %08lx\n",
		(uint32_t)ICE40UP_LARVA_MIC + ICE40UP_LARVA_MIC_CTRL,
		rCtrl
		);
	log_assert(rCtrl == 0);

	uint32_t rData = getmicreg32(ICE40UP_LARVA_MIC_DATA);
	log_printf(
		"DATA (@%08lx): %08lx\n",
		(uint32_t)ICE40UP_LARVA_MIC + ICE40UP_LARVA_MIC_DATA,
		rData
		);
	log_assert(rData == 0);

	log_assert(micCount == 375);

	log_printf("Starting audio clock\n");

	g_clocksPerSample = COUNTER_TICKS_HZ / micHz + 100;
	putmicreg32(ICE40UP_LARVA_MIC_CTRL, ICE40UP_LARVA_MIC_CTRL_RUN);

	g_dataStartTime = counter_get();

	rCtrl = getmicreg32(ICE40UP_LARVA_MIC_CTRL);
	if (! log_assert(rCtrl == ICE40UP_LARVA_MIC_CTRL_RUN))
		log_printf("actual: %08lx\n", rCtrl);

	test_filter();
	g_highpassdebug = false;
	log_printf("\n\npress 'c' to start audio sampling, 'r' for raw samples, 's' to stop\n");
	}

bool gRun = false;
bool gfRaw = false;

void
loop(void)
	{
	if (debug_checkbyte())
		{
		uint8_t key = debug_getbyte() & 0x7f;

		if (key == 'r' && (! gRun || ! gfRaw))
			{
			gfRaw = true;
			gRun = true;
			base64_flush();
			log_printf("\n\n-- raw mode: gain %u --\n", 1u << g_lg2gain);
			log_flush();
			}
		else if (key == 'c' && (! gRun || gfRaw))
			{
			gfRaw = false;
			gRun = true;
			base64_flush();
			log_printf("\n\n-- cooked mode: gain %u --\n", 1u << g_lg2gain);
			log_flush();
			}
		else if (key == 's' && gRun)
			{
			gRun = false;
			base64_flush();
			log_printf("\n\n-- stopped: gain %u --\n", 1u << g_lg2gain);
			}
		else if (key == '+')
			{
			if (g_lg2gain < 7)
				++g_lg2gain;
			}
		else if (key == '-')
			{
			if (g_lg2gain > 0)
				--g_lg2gain;
			}
		}

	if (! gRun)
		return;

	uint32_t const rData = getmicreg32(ICE40UP_LARVA_MIC_DATA);

	if (! (rData & ICE40UP_LARVA_MIC_DATA_RDY))
		{
		/* check for too long */
		uint32_t deltaT = counter_get() - g_dataStartTime;

		if (! log_assert(deltaT < g_clocksPerSample))
			{
			log_printf(
				"deltaT: %lu g_clocksPerSample: %lu  rData: %08lx  rCtrl: %08lx\n",
				deltaT, g_clocksPerSample, rData,
				getmicreg32(ICE40UP_LARVA_MIC_CTRL)
				);
			log_flush();
			for (;;)
				;
			}

		return;
		}

	/* ack the data */
	putmicreg32(ICE40UP_LARVA_MIC_DATA, rData);
	g_dataStartTime = counter_get();
	uint32_t rTestData = getmicreg32(ICE40UP_LARVA_MIC_DATA);

	if (! log_assert((rTestData & ICE40UP_LARVA_MIC_DATA_RDY) == 0))
		{
		log_printf("rTestData: %08lx\n", rTestData);
		log_flush();
		for (;;)
			;
		}

	int16_t sample = rData & ICE40UP_LARVA_MIC_DATA_PCMDATA_MASK;

	if (gfRaw)
		base64_put8(sample >> 8);
	else
		{
		int32_t filteredsample = highpass(sample << g_lg2gain);
		uint16_t filtered8;

		// map to 0..65535, make unsigned by u+s rules, round, then convert to 8 bits
		if (filteredsample <= -0x8080)
			filtered8 = 0;
		else
			filtered8 = (uint16_t) (filteredsample + 0x8080u) >> 8;

		// if input was above 0x7F80, then output will be 0x100; so...
		if (filtered8 > 0xFF)
			filtered8 = 0xFF;

		// send as PCM (signed).
		base64_put8((uint8_t)(filtered8 - 0x80));
		}
	}

int16_t g_filter = 0;
int32_t g_lastsample = 0;

int16_t highpass(int32_t sample)
	{
	int32_t normsample = sample;

	// highpass -- all of these have binary point at 8.
	uint32_t const k1 = 0xFCu;
	uint32_t const k2 = 0xFCu;
	uint32_t const k3 = 0xF8u;

	// unsigned multiplies are directly supported, so we try to do unsigned.
	int32_t hipass1 = g_lastsample * k1 - normsample * k2 + g_filter * k3;

	g_lastsample = normsample;

	if (hipass1 > 0x7FFF7F)
		g_filter = 0x7FFF;
	else if (hipass1 < -0x800000 - 0x80)
		g_filter = -0x8000;
	else
		/* for portability, convert to unsigned, shift right, then convert back */
		g_filter = (int16_t) (((hipass1 + 0x800080u) >> 8) - 0x8000);

	if (g_highpassdebug)
		{
		log_printf("sample: %04lx highpass1 %8ld  g_filter: %6d  8bit: %3d\n",
			sample & 0xFFFF,
			hipass1,
			g_filter,
			(g_filter + 128) / 256
			);
		log_flush();
		}

	return g_filter;
	}

unsigned g_nBits;
unsigned g_acc;
unsigned g_nLine;

char base64_encode(uint8_t sextet)
	{
	if (sextet < 26)
		return 'A' + sextet;
	else if (sextet < 52)
		return 'a' + (sextet - 26);
	else if (sextet < 62)
		return '0' + (sextet - 52);
	else if (sextet < 63)
		return '+';
	else
		return '/';
	}

void base64_put8(uint8_t thisByte)
	{
	unsigned nBits = g_nBits;
	unsigned acc = g_acc;
	unsigned nc;
	char s[4];

	nBits += 8;
	acc <<= 8;
	acc |= thisByte;

	/* pick off the top 6 bits */
	for (nc = 0; nBits >= 6; nBits -= 6, ++nc)
		{
		unsigned nShift = nBits - 6;
		uint8_t sextet = (acc >> (nBits - 6)) & 0x3F;
		acc &= (1 << nShift) - 1;

		s[nc] = base64_encode(sextet);
		}

	s[nc] = 0;
	log_puts(s);
	g_nBits = nBits;
	g_acc = acc;
	g_nLine += nc;
	if (g_nLine >= 76 && nBits == 0)
		{
		log_puts("\n");
		g_nLine = 0;
		}
	}

void base64_flush(void)
	{
	unsigned nBits = g_nBits;

	if (nBits == 2)
		{
		base64_put8(0);
		log_puts("==");
		}
	else if (nBits == 4)
		{
		char s[4];
		uint8_t sextet = (g_acc << 2);
		s[0] = base64_encode(sextet);
		s[1] = '=';
		s[2] = 0;
		log_puts(s);
		}
	g_nBits = 0;
	g_acc = 0;
	}

const uint16_t testdata[] = {
	0x0000,
	0x0000,
	0x01a5,
	0x1089,
	0x2567,
	0x3553,
	0x3e6f,
	0x3f1a,
	0x376c,
	0x2865,
	0x145d,
	0xfe6b,
	0xe9e1,
	0xd9cb,
	0xd0d7,
	0xd01f,
	0xd7e7,
	0xe6e8,
	0xfae1,
	};

void test_filter(void)
	{
	g_filter = 0;
	g_highpassdebug = true;

	for (unsigned i = 0; i < MCCIXDK_LENOF(testdata); ++i)
		{
		uint16_t const sample = testdata[i];
		(void) highpass(sample);
		}
	}