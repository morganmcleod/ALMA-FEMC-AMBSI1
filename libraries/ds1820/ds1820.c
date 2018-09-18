/*
 ****************************************************************************
 # $Id: ds1820.c,v 1.9 2006/05/26 18:25:26 avaccari Exp $
 #
 # Copyright (C) 1999
 # Associated Universities, Inc. Washington DC, USA.
 #
 # Correspondence concerning ALMA should be addressed as follows:
 #        Internet email: mmaswgrp@nrao.edu
 ****************************************************************************
 */
/*
 ****************************************************************************
 *  DS1820.C 
 *
 *  Implementation file for DS1820 library for dealing with Dallas Semiconductor 
 *	 DS1820 Temperature sensor.  Much of the code in here is general to
 *  any 1-Wire device.  This code is written for a C167 micro and makes use
 *  of the Timer 2 in the general purpose timer unit.
 *
 ****************************************************************************
 */

#include <reg167.h>
#include <intrins.h>
#include <math.h>

#include "ds1820.h"

/* Global */
static ubyte ds1820Running=0;

/* Reset one wire bus and test for presence pulse */
ubyte Reset_1W(void)
{
	unsigned short pin;

	/* Set port pin to output */
	SET_OUTPUT;
	
	/* Set pin low */
	RESET_PIN;

	/* Start the timer */
	CLEAR_T2;
	START_T2;

	/* Wait for 500 usec */
	while (READ_T2 < 78) ;

	/* Set pin to input */
	SET_INPUT;

	/* Wait up to 60 usecs for line to go high */
	CLEAR_T2;
	START_T2;

	while ((READ_T2 < 9) &&
		   !(pin = READ_PIN)) ;

	if (!pin) /* line never went high, so failure */
		return 0;
		
	/* Test for presence pulse for up to 240 usec */
	while ((READ_T2 < 46) &&
		   (pin = READ_PIN)) ;

	/* Wait around to end procedure 500 usec */
	while (READ_T2 < 78) ;

	/* Return last known pin value (ie if presence was asserted or not) */
	return !pin;
}

/* Write a byte on the One Wire bus */
void Write_1W(ubyte tx_byte)
{
	int i;

	/* Make sure pin will be high */
	SET_PIN;

	/* Set port pin to output */
	SET_OUTPUT;
	
	for (i=0; i<8; i++) {
		/* Start timer */
		CLEAR_T2;
		START_T2;

		/* Set pin low to initiate timeslot */
		RESET_PIN;

		/* Wait for at least 1 usec (6.4 actually) */
		while (READ_T2 < 1) ;

		/* Write the bit if necessary */
		if ((tx_byte >> i) & 0x01)
			SET_PIN;

		/* Wait out til end of timeslot */
		while (READ_T2 < 12) ;	

		/* Bring line high */
		SET_PIN;

		/* Wait another usec before next timeslot */
		while (READ_T2 < 13) ;
	}
}

/* Read a byte from the One Wire bus */
ubyte Read_1W(void)
{
	int i;
	ubyte rx_byte = 0x0; /* initialise to zero */

	/* Make sure pin will be high */
	SET_PIN;

	/* Set port pin to output */
	SET_OUTPUT;
	
	for (i=0; i<8; i++) {
		/* Start timer */
		CLEAR_T2;
		START_T2;

		/* Set pin low to initiate timeslot */
		RESET_PIN;

		/* Wait for at least 1 usec (6.4 actually) */
		while (READ_T2 < 1) ;

		/* Make pin an input */
		SET_INPUT;

		/* Wait for slave to write bit */
		while (READ_T2 < 2) ;

		/* Sample line */
		if (READ_PIN) 
			rx_byte |= (0x01 << i);

		/* Wait out til end of timeslot */
		while (READ_T2 < 11) ;	

		/* Bring line high */
		SET_PIN;

		/* Set port pin to output */
		SET_OUTPUT;

		/* Wait another usec before next timeslot */
		while (READ_T2 < 12) ;
	}
	
	return rx_byte; /* Return the byte read */
}

/* Convert from first two bytes of temperature data to degrees C */
/* Gives 1/2 degree C resolution */
float Do_1W_Temperature(ubyte MSB, ubyte LSB)
{
	float amt = 0.0;
	char temp;

/* Least significant bit signifies half a degree */
	if (LSB & 0x01)
		amt = 0.5;

/* shift the byte to remove the least significant bit */
	temp = LSB>>1;
	
/* Most significant byte indicates sign */
	if (MSB)
		temp |= 0x80;

/* Convert from two's complement signed 8 bit value to float */
	amt += (float) temp;

	return amt;
}

/* Convert temperature with full resolution */
float Do_1W_Temperature_Full(ubyte MSB, ubyte LSB,
							 ubyte count_remain, ubyte count_per_C)
{
	float amt = 0.0;
	char temp;

/* shift the byte to remove the least significant bit */
	temp = LSB>>1;
	
/* Most significant byte indicates sign */
	if (MSB)
		temp |= 0x80;

/* Convert from two's complement signed 8 bit value to float */
	amt = (float) temp;

/* Calculation from p4 of the DS1820 Data Sheet */
	if (fabs((double) count_per_C) > 1e-9) {
		amt += ((float) count_per_C - (float) count_remain)/((float) count_per_C) - 0.25;
	}

	return amt;
}

/*
 * Routine to calculate 8 bit CRC from DalSemi
 * Polynomial is CRC = X^8 + X^5 + X^4 + 1
 * Adapted from assembly language in Dallas Semiconductor
 * Application Note 27
 */
ubyte Do_1W_CRC(ubyte next_byte, ubyte CRC)
{
	int i;

	/* Move the bits in one by one and do some stuff */
	for (i=0; i<8; i++) {
		if ((next_byte ^ CRC) & 0x01) {
			CRC ^= 0x18; /* The generator polynomial */
			CRC >>= 1;
			CRC |= 0x80;
		} else {
			CRC >>= 1;
		}
		next_byte >>= 1;
	}

	return CRC;
}

short ds1820_init(void)
{
  /* ---------- Timer 2 Control Register ----------
   *  timer 2 works in timer mode
   *  prescaler factor is 128 (6.4 usec resolution)
   *  timer 2 run bit is reset
   *  up/down control bit is reset 
   *  external up/down control is disabled
   */
  T2CON = 0x0004;
  T2    = 0x0000;  /* load timer 2 register */

	/* Reset pulse and presence sequence */
	if (!Reset_1W())
		return -1;
	return 0;
}

short ds1820_get_sn(ubyte sn[8])
{
	int i;
	ubyte CRC;

	/* Start cycle on 1 wire bus by issuing reset and detecting presence */
	Reset_1W();

	/* Send Read ROM command (only works if ONE DS device on wire) */
	Write_1W(0x33);

	CRC = 0x0;

	/* Read 8 bytes of serial number etc */
	for (i=0; i<8; i++) {
		sn[i] = Read_1W();
		CRC = Do_1W_CRC(sn[i], CRC);
	}

	if (CRC != 0x0)
		return -1;
	return 0;
}

short ds1820_get_temp(ubyte *MSB, ubyte *LSB, ubyte *count_remain, ubyte *count_per_C)
{
	int i;
	uword wait_count;
	ubyte rx_buffer[10];
	ubyte CRC;

	if(ds1820Running){ // Check if the subrutine is still running
		return 0;
	}	

	ds1820Running = 1; // Signal that this is running

	/* Start temperature reading cycle of DS1820 */
	Reset_1W();

	Write_1W(0xCC); /* Skip ROM */
	Write_1W(0x44); /* Start conversion command */

	/* Wait for conversion to complete (signalled by all 1s) */
	wait_count = 0;
	while ((Read_1W() != 0xff) &&     /* Avoid lockup by quitting after 1000 */
			 (wait_count++ < 1000)) ;   /* byte reads (~600 ms) */

	if (wait_count > 999){ /* Error: wait for temperature conversion timed out */
		ds1820Running = 0; // Function is done and can be called again
		return -1;
	}

	/* Reset bus */
	Reset_1W();

	Write_1W(0xCC); /* Skip ROM */
	Write_1W(0xBE); /* Read scratchpad */

	CRC = 0x0;

	/* read 9 data bytes */
	for (i=0; i<9; i++) {
		rx_buffer[i] = Read_1W();
		CRC = Do_1W_CRC(rx_buffer[i], CRC);
	}

	if (CRC != 0x0){
		ds1820Running = 0; // Function is done and can be called again
		return -2;
	}

	/* Low accuracy temperature is in first two bytes */
	*LSB = rx_buffer[0];
 	*MSB = rx_buffer[1];

	/* Higher resolution data is in bytes 6 and 7 */
	*count_remain = rx_buffer[6];
	*count_per_C = rx_buffer[7];

	ds1820Running = 0; // Function is done and can be called again

	return 0;
}


