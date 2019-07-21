/*

Module:  ttn_face.c

Function:
	LoRaWAN face-sensing application for iCE40 Ultra Plus MDP / Risc-V

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
#include "cayenne_lpp.h"
#include "facedetect.h"
#include "log.h"
#include "mccixdk_env.h"

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

static bool
sendMessage(bool, bool);

static void
setup_sensors(void);

static void
setup_lmic(void);

// set the face message port.
#define	TTN_FACE_MESSAGE_PORT	5

// set the Cayenne sensor ID
#define CHANNEL_FACE		1

#ifndef TX_INTERVAL_DEFAULT
# define TX_INTERVAL_DEFAULT	(6 * 60)
#endif

#ifndef TX_INTERVAL_COUNT
# define TX_INTERVAL_COUNT	100
#endif

#define TX_INTERVAL_MIN		(10)
#define	TX_INTERVAL_MAX		(TX_INTERVAL_DEFAULT)

// the sensor SENSOR FSM -- very simple.
typedef struct SENSOR_FSM_s SENSOR_FSM;

struct SENSOR_FSM_s
	{
	bool		fSynchronized;
	bool		fFaceDetected;
	unsigned	nFaces;
	};

extern SENSOR_FSM sensorFsm;

static MCCIXDK_FORCEINLINE
SENSOR_FSM *SensorFSM_get(void)
	{
	return &sensorFsm;
	};

static void
SensorFSM_eval(void);

// the LORA FSM
typedef struct LORA_FSM_s LORA_FSM;
typedef unsigned LORA_FSM_STATE;

typedef MCCIXDK_SAL_Function_class(LORA_FSM_ENTRY_FN)
LORA_FSM_STATE (LORA_FSM_ENTRY_FN)(
	LORA_FSM *
	);

typedef MCCIXDK_SAL_Function_class(LORA_FSM_EVAL_FN)
LORA_FSM_STATE (LORA_FSM_EVAL_FN)(
	LORA_FSM *
	);

typedef struct LORA_FSM_DISPATCH_s LORA_FSM_DISPATCH;

struct LORA_FSM_DISPATCH_s
	{
	LORA_FSM_ENTRY_FN	*pEntry;
	LORA_FSM_EVAL_FN	*pEval;
	};

enum
	{
	LORA_FSM_stNoChange,
	LORA_FSM_stInitial,
	LORA_FSM_stSyncing,
	LORA_FSM_stIdle,
	LORA_FSM_stUpdate,
	};

#define LORA_FSM_STATE_NAMES__INIT				\
	"LORA_FSM_stNoChange",					\
	"LORA_FSM_stInitial",					\
	"LORA_FSM_stSyncing",					\
	"LORA_FSM_stIdle",					\
	"LORA_FSM_stUpdate"

static LORA_FSM_ENTRY_FN	LoRaFSMI_stSyncing_Entry;
static LORA_FSM_ENTRY_FN	LoRaFSMI_stIdle_Entry;
static LORA_FSM_ENTRY_FN	LoRaFSMI_stUpdate_Entry;

static LORA_FSM_EVAL_FN		LoRaFSMI_stInitial_Eval;
static LORA_FSM_EVAL_FN		LoRaFSMI_stSyncing_Eval;
static LORA_FSM_EVAL_FN		LoRaFSM_stIdle_Eval;
static LORA_FSM_EVAL_FN		LoRaFSMI_stUpdate_Eval;

struct LORA_FSM_s
	{
	unsigned	CurrentState;

	/* standard reentrancy control flags */
	bool		fBusy;
	bool		fDeferred;

	/* set true when time to send unsolicited */
	bool		fTimeToSend;
	bool		fFaceDetectUplink;
	bool		fTxPending;
	bool		fTxAcked;
	bool		fTimerPending;
	};

extern LORA_FSM LoRaFsm;

static MCCIXDK_FORCEINLINE
LORA_FSM *LoRaFSM_get(void)
	{
	return &LoRaFsm;
	};

static void
scheduleNextTx(LORA_FSM *pLoRaFsm);

static void
LoRaFSM_eval(void);

static void
LoRaFSM_init(void);

static LORA_FSM_STATE
LoRaFSMI_switch(LORA_FSM *pLoRaFsm, bool fNewState);

static const char *
LoRaFSMI_State_Name(LORA_FSM_STATE);

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

#if MDP_SERIAL == 6

static const char szMdpName[] = "himax s/n 006";
static const uint8_t DEVEUI[8]= { 0x59, 0x20, 0xC2, 0x72, 0x30, 0x04, 0xBE, 0x00 };
static const uint8_t APPKEY[16] = { 0xFE, 0xAF, 0x8B, 0xF6, 0x09, 0xB7, 0xB8, 0xB3, 0x04, 0xB1, 0xA1, 0xA1, 0x99, 0x64, 0x70, 0x4F };

#else
# error MDP board serial number (MDP_SERIAL) not known. Add a section.
#endif

// This should also be in little endian format, see above. It does not
// change per device.
static const uint8_t APPEUI[8]= { 0x91, 0x90, 0x00, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };



/****************************************************************************\
|
|	VARIABLES:
|
\****************************************************************************/

//
// this job is used for scheduling message transmission.
//
static osjob_t timerjob;

//
// txInterval is the number of seconds between transmisison. A downlink
// to port 1 can change that for a period of time.
//
uint32_t gTxInterval = TX_INTERVAL_DEFAULT;
uint32_t gTxIntervalCount;

SENSOR_FSM sensorFsm;
LORA_FSM LoRaFsm;

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
		   "ttn_face: " __DATE__ " "__TIME__ "\n"
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
	LoRaFSM_init();
	setup_lmic();
	}

/****************************************************************************\
|
|	Setup the face detector
|
\****************************************************************************/

static void
setup_sensors(void)
	{
	SENSOR_FSM * const pSensorFsm = SensorFSM_get();

	// there's nothing to do for I/O
	pSensorFsm->fSynchronized = false;
	pSensorFsm->fFaceDetected = facedetect_IsFacePresent();
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
	SensorFSM_eval();
	LoRaFSM_eval();
	}

/****************************************************************************\
|
|	Sensor FSM
|
\****************************************************************************/

static void
SensorFSM_eval(void)
	{
	SENSOR_FSM * const pSensorFsm = SensorFSM_get();
	bool fFaceDetected = facedetect_IsFacePresent();

	if (pSensorFsm->fFaceDetected != fFaceDetected)
		{
		pSensorFsm->fFaceDetected = fFaceDetected;
		pSensorFsm->fSynchronized = false;

		if (fFaceDetected)
			{
			++pSensorFsm->nFaces;
			log_printf("** Face Present: %u **\n", pSensorFsm->nFaces);
			}
		else
			{
			log_printf("-- departed --\n");
			}
		}
	}

/****************************************************************************\
|
|	LoRa FSM
|
\****************************************************************************/

static const char *
LoRaFSMI_State_Name(
	LORA_FSM_STATE state
	)
	{
	static const char sUnknown[] = "<<unknown>>";
	const char * const tNames[] = { LORA_FSM_STATE_NAMES__INIT };

	if (state < MCCIXDK_LENOF(tNames))
		{
		return tNames[state];
		}
	else
		{
		return sUnknown;
		}
	}

static void
LoRaFSM_init(void)
	{
	LORA_FSM * const pLoRaFsm = LoRaFSM_get();

	pLoRaFsm->CurrentState = LORA_FSM_stInitial;
	}

static void
LoRaFSM_eval(void)
	{
	LORA_FSM * const pLoRaFsm = LoRaFSM_get();

	if (pLoRaFsm->fBusy)
		{
		pLoRaFsm->fDeferred = true;
		return;
		}

	pLoRaFsm->fBusy = true;

	for (bool fNewState = false, fDone = false; !fDone; )
		{
		LORA_FSM_STATE const nextState = LoRaFSMI_switch(pLoRaFsm, fNewState);

		if (nextState != LORA_FSM_stNoChange)
			{
			fNewState = true;
			log_printfnl("%s: change state %s(%u) => %s(%u)",
				__func__,
				LoRaFSMI_State_Name(pLoRaFsm->CurrentState),
				pLoRaFsm->CurrentState,
				LoRaFSMI_State_Name(nextState),
				nextState
				);

			pLoRaFsm->CurrentState = nextState;
			}
		else if (pLoRaFsm->fDeferred)
			{
			// loop one more time
			pLoRaFsm->fDeferred = false;
			fNewState = false;
			}
		else
			fDone = true;
		}

	pLoRaFsm->fBusy = false;
	}

static const LORA_FSM_DISPATCH sk_LoRaFSMI_Dispatch[] =
	{
	/* stInitial */		{ NULL, LoRaFSMI_stInitial_Eval },
	/* stSyncing */		{ LoRaFSMI_stSyncing_Entry, LoRaFSMI_stSyncing_Eval },
	/* stIdle */		{ LoRaFSMI_stIdle_Entry, LoRaFSM_stIdle_Eval} ,
	/* stUpdate */		{ LoRaFSMI_stUpdate_Entry, LoRaFSMI_stUpdate_Eval }
	};

static LORA_FSM_STATE
LoRaFSMI_switch(
	LORA_FSM *pLoRaFsm,
	bool fNewState
	)
	{
	LORA_FSM_STATE const currentState = pLoRaFsm->CurrentState;
	const LORA_FSM_DISPATCH * const pDispatch =
		&sk_LoRaFSMI_Dispatch[currentState - LORA_FSM_stInitial];
	LORA_FSM_STATE newState;

	newState = LORA_FSM_stNoChange;

	if (fNewState)
		{
		LORA_FSM_ENTRY_FN * const pEntry = pDispatch->pEntry;

		if (pEntry != NULL)
			{
			newState = (*pEntry)(pLoRaFsm);
			if (newState != LORA_FSM_stNoChange)
				return newState;
			}
		}

	LORA_FSM_EVAL_FN * const pEval = pDispatch->pEval;

	if (pEval != NULL)
		{
		newState = (*pEval)(pLoRaFsm);
		}

	return newState;
	}

static LORA_FSM_STATE
LoRaFSMI_stInitial_Eval(
	LORA_FSM *pLoRaFsm
	)
	{
	// do any initialization

	return LORA_FSM_stIdle;
	}


static LORA_FSM_STATE
LoRaFSMI_stSyncing_Entry(
	LORA_FSM *pLoRaFsm
	)
	{
	// launch the message: we're not busy.
	pLoRaFsm->fTxPending = true;
	pLoRaFsm->fTxAcked = false;

	return LORA_FSM_stNoChange;
	}

static LORA_FSM_STATE
LoRaFSMI_stSyncing_Eval(
	LORA_FSM *pLoRaFsm
	)
	{
	LORA_FSM_STATE nextState;

	nextState = LORA_FSM_stNoChange;

	// wait for transmit to complete.
	if (! pLoRaFsm->fTxPending)
		{
		// the transmit completed.
		if (pLoRaFsm->fTxAcked)
			{
			SENSOR_FSM * const pSensorFsm = SensorFSM_get();

			if (pLoRaFsm->fFaceDetectUplink == pSensorFsm->fFaceDetected)
				pSensorFsm->fSynchronized = true;
			}

		nextState = LORA_FSM_stIdle;
		}

	return nextState;
	}

static LORA_FSM_STATE
LoRaFSMI_stIdle_Entry(
	LORA_FSM *pLoRaFsm
	)
	{
	// idle, waiting for something to happen
	// start a timed job.
	scheduleNextTx(pLoRaFsm);

	return LORA_FSM_stNoChange;
	}

static LORA_FSM_STATE
LoRaFSM_stIdle_Eval(
	LORA_FSM *pLoRaFsm
	)
	{
	// idle, waiting for something to happen
	SENSOR_FSM * const pSensorFsm = SensorFSM_get();
	LORA_FSM_STATE nextState;

	nextState = LORA_FSM_stNoChange;

	if (! pSensorFsm->fSynchronized)
		{
		// got to tell the server
		bool fSent = sendMessage(
				pSensorFsm->fFaceDetected,
				true
				);

		// if it didn't launch, tough
		if (! fSent)
			{
			log_printfnl(
				"?%s: couldn't launch message",
				__func__
				);
			}
		else
			{
			if (pLoRaFsm->fTimerPending)
				os_clearCallback(&timerjob);

			// remember what we transmitted
			pLoRaFsm->fFaceDetectUplink = pSensorFsm->fFaceDetected;
			nextState = LORA_FSM_stSyncing;
			}
		}
	else if (pLoRaFsm->fTimeToSend)
		{
		// got to tell the server
		bool fSent = sendMessage(
				pSensorFsm->fFaceDetected,
				false
				);

		// if it didn't launch, tough
		if (! fSent)
			{
			log_printfnl(
				"?%s: couldn't launch timed message",
				__func__
				);

			scheduleNextTx(pLoRaFsm);
			}
		else
			{
			pLoRaFsm->fTimeToSend = false;
			nextState = LORA_FSM_stUpdate;
			}
		}

	return nextState;
	}

static LORA_FSM_STATE
LoRaFSMI_stUpdate_Entry(
	LORA_FSM *pLoRaFsm
	)
	{
	// launch the message: we're not busy.
	pLoRaFsm->fTxPending = true;
	pLoRaFsm->fTxAcked = false;

	return LORA_FSM_stNoChange;
	}

static LORA_FSM_STATE
LoRaFSMI_stUpdate_Eval(
	LORA_FSM *pLoRaFsm
	)
	{
	LORA_FSM_STATE nextState;

	nextState = LORA_FSM_stNoChange;

	if (! pLoRaFsm->fTxPending)
		{
		nextState = LORA_FSM_stIdle;
		}

	return nextState;
	}

static void
LoRaFSM_evTimeToSend(
	void
	)
	{
	LORA_FSM * const pLoRaFsm = LoRaFSM_get();

	pLoRaFsm->fTimeToSend = true;
	// we could eval here, but we'll let it happen from the loop.
	}

static void
LoRaFSM_evTxComplete(
	bool fTxAcked
	)
	{
	LORA_FSM * const pLoRaFsm = LoRaFSM_get();

	pLoRaFsm->fTxPending = false;
	pLoRaFsm->fTxAcked = fTxAcked;
	// we could eval here, but we'll let it happen from the loop.
	}


/****************************************************************************\
|
|	LoRaWAN Utility routines
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
			for (unsigned iMap = 0; iMap < (72 + 15 - 1)/16; ++iMap)
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
#ifdef CFG_LMIC_US_like
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
			"(includes waiting for RX windows) flags %p:%#x",
			os_getTime(),
			&LMIC.txrxFlags,
			LMIC.txrxFlags
			);

		if (LMIC.txrxFlags & TXRX_ACK)
			{
			log_printfnl("Received ack");
			}

		LoRaFSM_evTxComplete((LMIC.txrxFlags & TXRX_ACK) != 0);

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
scheduleNextTx(
	LORA_FSM *pLoRaFsm
	)
	{
	log_printfnl("next passive transmit in %u seconds", (unsigned) gTxInterval);

	os_setTimedCallback(
		&timerjob,
		os_getTime()+sec2osticks(gTxInterval),
		do_send
		);

	pLoRaFsm->fTimerPending = true;
	pLoRaFsm->fTimeToSend = false;

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
	}

/*

Name:	do_send()

Function:
	Called whenever the timer expires, suggesting that it's time to send.

Definition:
	static void do_send(osjob *j);

Description:
	This function is an osjobcb_t function, called from an osjob when
	it's time to consider sending another message.

	We just set a flag and depend on the runloop to do the next step.

Returns:
	No explicit result.

*/

static void
do_send(osjob_t* j)
	{
	LoRaFSM_evTimeToSend();
	}

/*

Name:	sendMessage()

Function:
	Called to prepare a message to send to the network.

Definition:
	static bool sendMessage(
			bool fPresence,
			bool fIsEvent
			);

Description:
	This function formats and queues a message.

	It formats up a message based on the value of fPresence, and
	sends it.

	If this is a significant event (fIsEvent is not false), then
	the message is sent in confirmed mode.

	In this application, we use the port key as an indication of the
	message format. Port 5 (TTN_FACE_MESSAGE_PORT) messages are formatted
	as Cayenne-compatible messages, consisting of a single point:

		( CHANNEL_FACE, LPP_PRESENCE, vPresence )

	Where `CHANNEL_FACE` is 1, and designates the face detect sensor;
	`LPP_PRESENCE` is 102, based on the [Cayenne LPP protocol)[1], and
	vPresence is 0x01 or 0x00 as fPresence is true or false.

	[1]: https://mydevices.com/cayenne/docs_stage/lora/#lora-cayenne-low-power-payload

	fIsEvent indicates whether this is an important event. If so,
	the value is transmitted using a confirmed transmission; if not,
	(for example, if this is just a heartbeat), the value is transmitted
	unconfirmed.

Returns:
	true if a message was launched.

*/

static uint8_t *addPresence(
	uint8_t *p,
	uint8_t *pEnd,
	uint8_t bChannel,
	bool fPresence
	)
	{
	if (p < pEnd && pEnd - p >= LPP_PRESENCE_SIZE)
		{
		*p++ = bChannel;
		*p++ = LPP_PRESENCE;
		*p++ = fPresence ? 1 : 0;
		}

	return p;
	}

static bool
sendMessage(
	bool fPresence,
	bool fIsEvent
	)
	{
	uint8_t mydata[16];
	uint8_t * const pEnd = mydata + sizeof(mydata);
	uint8_t *p;
	int rc;

	// Check if there is not a current TX/RX job running
	if (LMIC.opmode & OP_TXRXPEND)
		{
		log_printfnl("OP_TXRXPEND, not sending");
		return false;
		}

	// format the data.
	memset(mydata, 0, sizeof(mydata));
	p = mydata;

	// add presence info.
	p = addPresence(p, pEnd, CHANNEL_FACE, fPresence);

	// Prepare upstream data transmission at the next possible
	// time.  Send on port 5. Request an ack.
	rc = LMIC_setTxData2(
		TTN_FACE_MESSAGE_PORT,
		mydata, p - mydata,
		/* confirmed? */ fIsEvent ? 1 : 0
		);

	if (rc != 0)
		{
		log_printfnl("tx error: %d", rc);
		return false;
		}
	else
		{
		log_printfnl("Packet queued");
		return true;
		}
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
