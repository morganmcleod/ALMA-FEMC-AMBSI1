/*
 ****************************************************************************
 # $Id: ds1820.h,v 1.3 2000/03/18 00:17:44 mbrooks Exp $
 #
 # Copyright (C) 1999
 # Associated Universities, Inc. Washington DC, USA.
 #
 # Correspondence concerning ALMA should be addressed as follows:
 #        Internet email: mmaswgrp@nrao.edu
 ****************************************************************************
 */
/*****************************************************************************
 *
 *  DS1820.H 
 *
 *  Header file for DS1820 library for dealing with Dallas Semiconductor 
 *	 DS1820 Temperature sensor.  Much of the code in here is general to
 *  any 1-Wire device.  This code is written for a C167 micro and makes use
 *  of the Timer 2 in the general purpose timer unit.
 *
 ****************************************************************************
 */

/*
 ****************************************************************************
 *  Prototypes of global functions
 ****************************************************************************
 */

#define ulong unsigned long
#define uword unsigned int
#define ubyte unsigned char

/**
 * Routines for ALMA specific things.  All routines return 0 for success and -1
 * on error.
 */
short ds1820_init(void);
short ds1820_get_sn(ubyte sn[8]);
short ds1820_get_temp(ubyte *MSB, ubyte *LSB, ubyte *count_remain, ubyte *count_per_C);

/**
 * Generic 1 Wire primitive functions 
 */
ubyte Reset_1W(void);              /* Reset bus and test for presence pulse */
void  Write_1W(ubyte tx_byte);	  /* Write a byte to the bus */
ubyte Read_1W(void);				     /* Read a byte from the bus */

ubyte Do_1W_CRC(ubyte next_byte, ubyte CRC);   /* Calculate CRC */
float Do_1W_Temperature(ubyte MSB, ubyte LSB); /* Calculate temperature from DS1820 data with 0.5C resolution */
float Do_1W_Temperature_Full(ubyte MSB, ubyte LSB, /* Calculate accurate temperature from DS1820 data */
							 ubyte count_remain, ubyte count_per_C); 

/*
 ****************************************************************************
 * Macros
 ****************************************************************************
 */
/**
 * This macro starts the Timer 2. The timer continues
 * to count from where it had stopped
 */

#define START_T2 T2R = 1

/**
 * This macro stops Timer 2. The contents of
 * the timer register remain unchanged.
 */

#define STOP_T2 T2R = 0                     

/**
 * This macro stops Timer 2 and sets the timer
 * register to 0.
 */

#define CLEAR_T2 T2R = 0; T2 = 0x0000 

/**
 * This macro returns the contents of the selected Timer 2.
 * The timer is not stopped. Access of the CPU to the
 * registers of the timer is of higher priority than a timer
 * increment, a reload or a capture event.
 */

#define READ_T2 T2

/**
 * The following macros vary depending on the hardware.  Use different
 * targets to create different libraries
 */

#ifdef SK167

sbit  P7_0 = P7^0;

#define READ_PIN P7_0
#define SET_PIN P7_0 = 1
#define RESET_PIN P7_0 = 0
#define SET_INPUT DP7 &= ~0x01
#define SET_OUTPUT DP7 |= 0x01

#endif

#ifdef AMBSI

sbit  P3_0 = P3^0;

#define READ_PIN P3_0
#define SET_PIN P3_0 = 1
#define RESET_PIN P3_0 = 0
#define SET_INPUT DP3 &= ~0x01
#define SET_OUTPUT DP3 |= 0x01

#endif

