/*

Module:  i2c_scan_mdp.c

Function:
	Simple i2c scan utility.

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
#include "i2c.h"
#include "log.h"

#include "ice40up_mdp.h"
#include "bmp180.h"
#include "lsm330_accel.h"
#include "lsm330_gyro.h"
#include "lsm303_accel.h"
#include "lsm303_mag.h"
#include "lis2ds12.h"

#include "sensor_bmp180.h"

bool	fGotLsm303;

static void
test1reg(
	const char *pName,
	uint8_t i2cAddr,
	uint8_t reg,
	uint8_t expected
	);

static void
test_lsm303_t(void);

static void
test_bmp180_t(void);

static int
read_reg(
	uint8_t i2caddr,
	uint8_t reg
	);

SENSOR_BMP180 gBmp180;

/****************************************************************************\
|
|	setup()
|
\****************************************************************************/

void
setup(void)
	{
	bool flag;

	log_printf("Setup:\n");
	i2c_begin();

	flag = i2c_reg_write(
		MDP_I2C_LSM303_MAG,
		LSM303_R_CRA_REG_M,
		LSM303_R_CRA_REG_M_TEMP_EN |
		LSM303_R_CRA_REG_M_DR_15
		);

	if (flag)
		flag = i2c_reg_write(
			MDP_I2C_LSM303_MAG,
			LSM303_R_MR_REG_M,
			LSM303_R_MR_REG_M_MD_CONT
			);

	if (! flag)
		{
		log_printf("failed to initalize LSM303\n");
		fGotLsm303 = false;
		}
	else
		fGotLsm303 = true;


	flag = sensor_bmp180_begin(&gBmp180);
	if (! flag)
		{
		log_printf("%s: failed to initalize BMP180\n", __func__);
		}
	if (flag)
		{
		flag = sensor_bmp180_setSampling(
			&gBmp180,
			SENSOR_BMP180_SAMPLING_8
			);

		if (! flag)
			log_printf(
				"%s: couldn't select BMP180 sampling rate %d\n",
				__func__,
				SENSOR_BMP180_SAMPLING_8
				);

		flag = sensor_bmp180_dataSheetTest();
		if (! flag)
			log_printf(
				"%s: datasheet test failed\n",
				__func__
				);
		}
	}

/****************************************************************************\
|
|	loop()
|
\****************************************************************************/

void
loop(void)
	{
	static uint32_t t0;
	uint32_t now = millis();
	bool fNow;

	// only do this every 5 seconds
	fNow = false;
	if (now - t0 > 5000)
		{
		t0 = t0 + ((now - t0)) / 5000 * 5000;
		fNow = true;
		}

	if (fNow)
		{
		log_printf("scanning:\n");
		log_flush();

		test1reg(
			"Temp/Pressure BME180 ID",
			MDP_I2C_BMP180, BMP180_R_ID,
			0x55
			);
		test1reg(
			"LSM330 Gyro",
			MDP_I2C_LSM330_GYRO, LSM330_R_WHO_AM_I_G,
			LSM330_R_WHO_AM_I_G_VALUE
			);
		test1reg(
			"LSM330 Accel",
			MDP_I2C_LSM330_ACCEL, LSM330_R_WHO_AM_I_A,
			LSM330_R_WHO_AM_I_A_VALUE
			);

		// read the temperature
		test_lsm303_t();

		// reand the temp from the BMP180
		test_bmp180_t();
		}
	}

static void
test1reg(
	const char *pName,
	uint8_t i2cAddr,
	uint8_t reg,
	uint8_t expected
	)
	{
	// get the register and print it.
	uint8_t rValue;
	bool flag;

	rValue = 0;
	flag = i2c_reg_read(&rValue, i2cAddr, reg);

	log_printf(
		"%s: ", pName
		);
	if (flag)
		{
		log_printf(
			"%02x",
			rValue
			);

		if (rValue != expected)
			{
			log_printf(" != expected(%02x)", expected);
			}
		}
	else
		{
		log_printf(
			"*failed*"
			);
		}

	log_puts("\n");
	}


static void
test_lsm303_t(void)
	{
	bool flag;
	uint8_t temp[2];

	log_printf("LSM303 temperature: ");

	flag = fGotLsm303;

	if (flag)
		{
		flag = i2c_reg_read(
			&temp[0],
			MDP_I2C_LSM303_MAG,
			LSM303_R_TEMP_OUT_H_M
			);
		}
	if (flag)
		{
		flag = i2c_reg_read(
			&temp[1],
			MDP_I2C_LSM303_MAG,
			LSM303_R_TEMP_OUT_L_M
			);
		}

	if (! flag)
		{
		if (fGotLsm303)
			log_printf("** error **");
		else
			log_printf("** LSM303 did not initalize **");
		}
	else
		{
		const int16_t sT = (int16_t) ((temp[0] << 8) + temp[1]);

		/*
		|| The datasheet implies that 1 lsb == 1/8 degree C, but
		|| the data from the part makes no sense unless interpeted
		|| as being 1/4 degree C. Convert to F, rounding.
		*/
		int t = (sT * 9 + 5 * 32) / (5 * 64) + 32;

		/* print (and print raw data, too) */
		log_printf("%d (%02x %02x)", t, temp[0], temp[1]);

		/* if the result is crazy, then print the mode register */
		if (t == 0)
			{
			log_printf("  CRA_REG: %02x",
					read_reg(
						MDP_I2C_LSM303_MAG,
						LSM303_R_CRA_REG_M
						)
				);
			}
		}

	log_printf("\n");
	}

static int
read_reg(
	uint8_t i2caddr,
	uint8_t reg
	)
	{
	uint8_t result;

	if (i2c_reg_read(&result, i2caddr, reg))
		return result;
	else
		return -1;
	}


static void
test_bmp180_t(void)
	{
	bool flag;

	log_printf("BMP180 temperature/pressure: ");

	flag = gBmp180.fInitialized;

	if (flag)
		{
		MCCIXDK_INT32 temp = sensor_bmp180_getTemp(&gBmp180);

		log_printf(
			"%d.%u C  ",
			temp / 10,
			(temp < 0 ? -temp : temp) % 10
			);

		MCCIXDK_UINT32 p = sensor_bmp180_getPressure(
					&gBmp180, &temp
					);

		log_printf(
			"%u.%02u millibar (%d.%u C)",
			p / 100,
			(p % 100),
			temp / 10,
			(temp < 0 ? -temp : temp) % 10
			);
		}
	else
		{
		log_printf("**not initialized**");
		}

	log_printf("\n");
	}
