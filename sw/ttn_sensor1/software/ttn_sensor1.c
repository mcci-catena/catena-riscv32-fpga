/* ttn_sensor1.c	Sat Apr  8 2017 17:47:04 tmm */

/*

Module:  ttn_sensor1.c

Function:
	Remote sensing application for iCE40 Ultra Plus MDP / Risc-V

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

#include "arduino_lmic.h"
#include "log.h"
#include "sensor_bmp180.h"

/****************************************************************************\
|
|	Manifest constants & typedefs.
|
\****************************************************************************/

static bool
checkNonZero(const uint8_t *pBuffer, size_t nBuffer);

static void
do_send(osjob_t* j);

static void
log_bebuf(const char *pPrefix, const uint8_t *pBuffer, size_t nBuffer);

static void
log_lebuf(const char *pPrefix, const uint8_t *pBuffer, size_t nBuffer);

static void
receiveMessage(
	uint8_t port,
	const uint8_t *pMessage,
	size_t nMessage
	);

static void
scheduleNextTx(void);

static bool
sendMessage(void);

static void
setup_sensors(void);

static void
setup_lmic(void);

#ifndef TX_INTERVAL_DEFAULT
# define TX_INTERVAL_DEFAULT	(6 * 60)
#endif

#ifndef TX_INTERVAL_COUNT
# define TX_INTERVAL_COUNT	100
#endif

#define TX_INTERVAL_MIN		(10)
#define	TX_INTERVAL_MAX		(TX_INTERVAL_DEFAULT)


/****************************************************************************\
|
|	Read-only data.
|
\****************************************************************************/

// You must replace the values marked FILLMEIN with registration values
// obtained from the TTN console!
#define FILLMEIN 0

// DEVEUI:
//
// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. When copying from the TTN console, this means to select LSB
// mode prior to copying.  In a real device, it would be better to store
// this outside the source code for easier provisioning.
//
// APPEUI:
//
// This should also be in little-endian format, but doesn't need to differ
// from device to device.
//
// APPKEY:
//
// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// other words, use the "msb" format to copy the key from the TTN console.
// This will change for each device.
//

// For the purposes of this demo, we have pre-provisioned 8 devices. The
// compile-time variable MDP_SERIAL should be set to the device you're
// provisioning. The network will silently discard data for one of the two
// devices if two are configured with the same ID. Best practice would be
// to key this to the MDP index; or simply update the first entry as needed
// for each board you provision.

#ifndef MDP_SERIAL
# error "You must set MDP_SERIAL to the serial number of the board you're using"
#endif

#if MDP_SERIAL == 21

static const char szMdpName[] = "s/n 021";
static const uint8_t DEVEUI[8]= { 0x27, 0xA5, 0x89, 0x5E, 0xA9, 0xD0, 0xDF, 0x00 };
static const uint8_t APPKEY[16] = { 0x6C, 0x81, 0x88, 0x2E, 0xC5, 0x59, 0x3D, 0x08, 0x11, 0x82, 0xB9, 0x0E, 0xFF, 0x69, 0x2F, 0x62 };
// following is used for RWC5020A testing:
// static const uint8_t APPKEY[16] = { 0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 1 };

#elif MDP_SERIAL == 8

static const char szMdpName[] = "s/n 008";
static const uint8_t DEVEUI[8]= { 0x21, 0x7B, 0xE3, 0x3F, 0xA3, 0x04, 0xCD, 0x00 };
static const uint8_t APPKEY[16] = { 0x7C, 0x89, 0x55, 0x24, 0x4F, 0xC4, 0x44, 0x03, 0xA8, 0x81, 0x38, 0x90, 0x53, 0x9F, 0x13, 0x5D };

#elif MDP_SERIAL == 1000

static const char szMdpName[] = "s/n 1000 (MCCI)";
static const uint8_t DEVEUI[8]= { 0xD8, 0xB4, 0x44, 0xCD, 0x4A, 0x10, 0xA3, 0x00 };
static const uint8_t APPKEY[16] = { 0x9B, 0x6F, 0x97, 0xEC, 0xA8, 0x4C, 0x67, 0xAA, 0xB5, 0xEF, 0x77, 0x88, 0x6E, 0x5E, 0xC4, 0xC3 };

#else
# error MDP board serial number (MDP_SERIAL) not known. Add a section.
#endif

// This should also be in little endian format, see above. It does not
// change per device.
static const uint8_t APPEUI[8]= { 0x88, 0x44, 0x00, 0xF0, 0x7E, 0xD5, 0xB3, 0x70 };


/****************************************************************************\
|
|	VARIABLES:
|
\****************************************************************************/

SENSOR_BMP180 gBmp180;

//
// this job is used for scheduling message transmission.
//
static osjob_t sendjob;

//
// txInterval is the number of seconds between transmisison. A downlink
// to port 1 can change that for a period of time.
//
uint32_t gTxInterval = TX_INTERVAL_DEFAULT;
uint32_t gTxIntervalCount;

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
		   "ttn_sensor1: " __DATE__ " "__TIME__ "\n"
		   "\n"
		   "MDP board %s\n"
		   "\n"
		   "%s\n",
		   dash59,
		   szMdpName,
		   dash59
		   );
	log_flush();

	log_printfnl("Provisioning info:");
	log_lebuf("DEVEUI: ", DEVEUI, sizeof(DEVEUI));
	log_lebuf("APPEUI: ", APPEUI, sizeof(APPEUI));
	log_bebuf("APPKEY: ", APPKEY, sizeof(APPKEY));

	// verify that we actually have non-zero DEVEUI, APPEUI, APPKEY
	if (! (checkNonZero(DEVEUI, sizeof(DEVEUI)) &&
	       checkNonZero(APPEUI, sizeof(APPEUI)) &&
	       checkNonZero(APPKEY, sizeof(APPKEY))))
		{
		log_printfnl("FATAL: registration info must be non-zero!");
		log_flush();
		while (1);
		}

	setup_sensors();
	setup_lmic();
	}

/****************************************************************************\
|
|	Setup the BMP180
|
\****************************************************************************/

static void
setup_sensors(void)
	{
	bool flag;

	flag = sensor_bmp180_begin(&gBmp180);
	if (! flag)
		{
		log_printfnl("%s: failed to initalize BMP180", __func__);
		}
	if (flag)
		{
		flag = sensor_bmp180_setSampling(
			&gBmp180,
			SENSOR_BMP180_SAMPLING_8
			);

		if (! flag)
			log_printfnl(
				"%s: couldn't select BMP180 sampling rate %d",
				__func__,
				SENSOR_BMP180_SAMPLING_8
				);
		}

	if (! flag)
		{
		log_flush();
		log_printfnl("BMP180 init failure!");
		log_flush();

		// we don't hang here, because it's hard to see on power
		// up. Instead we'll check the flag when we're about to
		// send data.
		}
	}

/****************************************************************************\
|
|	Setup the LoRaWAN MAC
|
\****************************************************************************/

static void
setup_lmic(void)
	{
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

#if CFG_region == LMIC_REGION_us915
	// set the default data rate and power.
	LMIC_setDrTxpow(US915_DR_SF7, 14);
#endif

#if CFG_LMIC_US_like
	// set subband 2
	LMIC_selectSubBand(1);
#endif

	// Start job (sending automatically starts OTAA too)
	do_send(&sendjob);
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
	os_runloop_once();
	}

/****************************************************************************\
|
|	Utility routines
|
\****************************************************************************/

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

/****************************************************************************\
|
|	Required outcalls for the LoRaWAN code
|
\****************************************************************************/

/* LMIC calls this to get the APPEUI */
void os_getArtEui(u1_t* buf)
	{
	memcpy(buf, APPEUI, 8);
	}

/* LMIC calls this to get the DEVEUI */
void os_getDevEui(u1_t* buf)
	{
	memcpy(buf, DEVEUI, 8);
	}

/* LMIC calls this to get the APPKEY */
void os_getDevKey(u1_t* buf)
	{
	memcpy(buf, APPKEY, 16);
	}

/*

Name:	onevent()

Function:
	Outcall from LMIC code for processing LoRaWAN lower MAC events.

Definition:
	#include "arduino_lmic.h"

	void onEvent(ev_t ev);

Description:
	The LMIC code calls this function (by name) whenever an event occurs.

	In a more complete environment (such as MCCI's production
	environment), this is handled by the framework, but for this example
	we provide this directly as part of the application. This routine
	needs to respond correctly to JOIN events, data-received events,
	and TX-complete events.

	Anything too complicated should be deferred. In our case, we do that
	whenever a message completes, by scheduling a job that will
	(eventually) cause another message to be transmitted.

Returns:
	No explicit result.

*/

static void
log_printev(
	const char *pEvName
	)
	{
	log_printfnl("%6lu: EV_%s", os_getTime(), pEvName);
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
		log_printfnl("%s", dash14);
		log_printev("JOINED");
		log_printfnl("%s", dash14);
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
			log_bebuf("AppSKey: ", AppSKey, sizeof(AppSKey));
			log_bebuf("NwkSKey: ", NwkSKey, sizeof(NwkSKey));

			// printout other data from join
#if CFG_LMIC_EU_like
			u2_t channelMap = LMIC.channelMap;
			for (unsigned ch = 0; ch < MAX_CHANNELS; ++ch)
				{
				if (channelMap & (1 << ch))
					{
					u1_t band = (u1_t) LMIC.channelFreq[ch] & 3;
					unsigned freq = ((LMIC.channelFreq[ch] & ~3) + 50000) / 100000;
					log_printfnl("ch %u: %u.%u MHz %d dBm",
						      ch,
						      freq / 10,
						      freq % 10,
						      LMIC.bands[band].txpow
						      );
					}

				}
#elif CFG_LMIC_US_like
			log_printf("channel map:");
			for (unsigned iMap = 0; iMap < 72/16; ++iMap)
				{
				log_printf(" %04x", LMIC.channelMap[iMap]);
				}
			log_printf("\n");
#endif

#if LMIC_ENABLE_TxParamSetupReq
			log_printfnl("TxParam from TxParamSetupReq: %02x",
				LMIC.txParam
				);
#endif
			}

		// Disable link check validation (automatically enabled
		// during join, but not supported by TTN at this time).
#ifdef CFG_us915
		LMIC_setLinkCheckMode(0);
#endif
		break;

	case EV_RFU1:
		log_printev("RFU1");
		break;

	case EV_JOIN_FAILED:
		{
		static unsigned nJoinFail = 0;

		++nJoinFail;
		log_printfnl(
			"%6lu: EV_JOIN_FAILED (%u)",
			os_getTime(), nJoinFail
			);
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
			uint8_t port;

			log_printfnl(
				"Received %u bytes of payload",
				LMIC.dataLen
				);

			port = 0;
			if (LMIC.txrxFlags & TXRX_PORT)
				port = LMIC.frame[LMIC.dataBeg - 1];

			receiveMessage(
				port,
				LMIC.frame + LMIC.dataBeg,
				LMIC.dataLen
				);
			}

		// Schedule next transmission
		scheduleNextTx();
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
		log_printfnl("%6lu: EV_TXSTART (%u) ch %d",
				os_getTime(), nTxStart, LMIC.txChnl
				);
		}
		break;
	default:
		log_printfnl("Unknown event");
		break;
		}
	}

/*

Name:	scheduleNextTx(void)

Function:
	Schedule a transmission.

Definition:
	void scheduleNextTx(void);

Description:
	The next transmission is scheduled for an appropriate amount of
	time in the future.

Returns:
	No explicit result.

*/

static void
scheduleNextTx(void)
	{
	os_setTimedCallback(
		&sendjob,
		os_getTime()+sec2osticks(gTxInterval),
		do_send
		);
	log_printfnl("next transmit in %u seconds", (unsigned) gTxInterval);
	}

/*

Name:	do_send()

Function:
	Called to prepare a message to send to the network.

Definition:
	static void do_send(osjob *j);

Description:
	This function is an osjobcb_t function, called from an osjob when
	it's time to send another message.

	It acquires the current sensor data, formats up a message, and
	sends it.

	In this application, we use the port key as an indication of the
	message format. Port 1 messages have the following format:

	byte 0 is a bit mask of values that are present or absent in the
	data.

		bit 0:	pressure and temperature are present
		bits 1..7: reserved, shall be zero.

	Bytes 1..n-1 are the data, transmitted in the order given above.

	Pressure is transmitted as a two-byte field, big-endian form.
	The pressure is transmitted as millibars * 25 (or pascals/4).

	Temperature is transmitted as a two byte field, big-endian form.

Returns:
	No explicit result.

*/

static void
do_send(osjob_t* j)
	{
	// format and send the message
	bool fSent = sendMessage();

	// allow for fast and slow messages. Slow is the default.
	// if we get a downlink that specifies fast messages, we
	// set gTxIntervalCount to a small number, and downcount
	// it. Once it goes to zero, we revert to the default (slow)
	// tx rate.
	if (gTxIntervalCount > 1)
		--gTxIntervalCount;
	else
		{
		gTxIntervalCount = 0;
		gTxInterval = TX_INTERVAL_DEFAULT;

		log_printfnl("reset tx interval to default: %u s",
			(unsigned) gTxInterval
			);
		}

	// if we didn't succeed in sending, schedule another
	// send
	if (! fSent)
		scheduleNextTx();
	}

/*

Name:	sendMessage()

Function:
	Called to prepare a message to send to the network.

Definition:
	static bool sendMessage();

Description:
	This function formats and queues a message.

	It acquires the current sensor data, formats up a message, and
	sends it.

	In this application, we use the port key as an indication of the
	message format. Port 1 messages have the following format:

	byte 0 is a bit mask of values that are present or absent in the
	data.

		bit 0:	pressure and temperature are present
		bits 1..7: reserved, shall be zero.

	Bytes 1..n-1 are the data, transmitted in the order given above.

	Pressure is transmitted as a two-byte field, big-endian form.
	The pressure is transmitted as millibars * 25 (or pascals/4).

	Temperature is transmitted as a two byte field, big-endian form,
	scaled in tenths of degree C.

Returns:
	true if a message was launched.

*/

static bool
sendMessage(void)
	{
	uint8_t mydata[16];
	unsigned pressure;
	int temperature;
	uint8_t *p;
	uint8_t *pFlag;

	// Check if there is not a current TX/RX job running
	if (LMIC.opmode & OP_TXRXPEND)
		{
		log_printfnl("OP_TXRXPEND, not sending");
		return false;
		}

	// format the data.
	memset(mydata, 0, sizeof(mydata));
	pFlag = mydata;
	p = pFlag + 1;

	// check if the BMP180 failed to initialize
	if (! gBmp180.fInitialized)
		{
		log_printfnl("BMP180 failed!");
		}
	else
		{
		// take a measurement
		pressure = sensor_bmp180_getPressure(&gBmp180, &temperature);

		// display it locally
		log_printfnl(
			"BMP180: %u.%02u millibar (%d.%u C)",
			pressure / 100,
			(pressure % 100),
			temperature / 10,
			(temperature < 0 ? -temperature : temperature) % 10
			);

		/* append the pressure and temperature */
			{
			*pFlag |= (1 << 0);	// remember that we have pressure/temp.

			// Pressure is certianly less than 2621 millibar
			// so if divided by 4 it will be less than 65536.
			// Still, this is a measurement, so we're careful.
			uint16_t p_word;

			if (pressure < 0x3FFFFu)
				p_word = (uint16_t) ((pressure + 2) / 4);
			else
				p_word = 0xFFFFu;

			*p++ = (uint8_t) (p_word >> 8);
			*p++ = (uint8_t) p_word;

			// send temperature in tenths degree C, signed
			// right shifts by a power of two are not portable.
			if (temperature <= -32768)
				temperature = -32768;
			else if (temperature > 32767)
				temperature = 32767;

			*p++ = (uint8_t) ((temperature / 256) & 0xFFu);
			*p++ = (uint8_t) (temperature & 0xFFu);
			}
		}

	// Prepare upstream data transmission at the next possible
	// time.  Send on port 1. Don't request an ack.
	LMIC_setTxData2(1, mydata, p - mydata, 0);

	// use this line if you want an ack.
	// LMIC_setTxData2(1, mydata, p - mydata, 1);

	log_printfnl("Packet queued");
	return true;
	}

/*

Name:	receiveMessage()

Function:
	Process downlink messages.

Definition:
	void receiveMessage(
		uint8_t port,
		const uint8_t *pMessage,
		size_t nMessage
		);

Description:
	This routine is called from the local onEvent() handler to process
	a downlink message. We current handle only one message, which must
	be sent to port 1, with the following form:

		[0..1]:	the interval between transmission attempts, in
			seconds (big endian)
		[2]:	optionally, the count of messages to send at the
			higher rate.

	The interval must be at least 10, and cannot be larger than the
	default (6 minutes).

Returns:
	No explicit result.

*/

static void
receiveMessage(
	uint8_t port,
	const uint8_t *pMessage,
	size_t nMessage
	)
	{
	unsigned txInterval;
	unsigned txCount;

	if (! (port == 1 && 2 <= nMessage && nMessage <= 3))
		{
		log_printfnl("invalid message port(%02x)/length(%zx)",
			port, nMessage
			);
		return;
		}

	txInterval = (pMessage[0] << 8) | pMessage[1];

	if (txInterval < TX_INTERVAL_MIN || txInterval > TX_INTERVAL_MAX)
		{
		log_printfnl("tx interval out of range: %u", txInterval);
		return;
		}

	// byte [2], if present, is the repeat count.
	txCount = TX_INTERVAL_COUNT;
	if (nMessage >= 3)
		{
		txCount = pMessage[2];
		if (txCount == 0)
			txCount = TX_INTERVAL_COUNT;
		}

	log_printfnl(
		"message interval %u seconds for %u messages",
		txInterval, txCount
		);

	gTxInterval = txInterval;
	gTxIntervalCount = txCount;
	}
