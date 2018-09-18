/*# $Id: spi_pic.c,v 1.2 1999/12/17 21:06:36 mbrooks Exp $
/*#
/*# Copyright (C) 1999
/*# Associated Universities, Inc. Washington DC, USA.
/*#
/*# Correspondence concerning ALMA should be addressed as follows:
/*#        Internet email: mmaswgrp@nrao.edu
/*****************************************************************************
/*
/*  SPI_PIC.C
/*
/*  Implementation file for PIC architecture Serial to Parallel Interface.  It
/*  is intended that this file be used in conjunction with the AMB M&C library.
/*
/*****************************************************************************

/* <summary>
/* </summary>

/* <synopsis>
/* </synopsis>

/* <example>
/* <srcblock>
/* </srcblock>
/* </example> */

#include 	"spi_pic.h"		/* SPI functions */

/* Globals */
unsigned char SPIDummy;

/* -----------------------------------------------------------------------------------
 * Function Name: SPI_Reset()
 * This functions resets the SPI Device.
   -----------------------------------------------------------------------------------*/
void SPI_Reset(){
	int i;

	SPI_CS = 0;

	for (i=0; i<16; i++) {
		SSPBUF = SPI_RESET;		/* Send the serial reset instruction */
		while (!STAT_BF);			/* wait for ssp to finish */
		SPIDummy = SSPBUF;
	}

	SPI_CS = 1;
}

/* -----------------------------------------------------------------------------------
 * Function Name: SPI_Read(uint address)
 * This functions reads one byte from the SPI address specified.
   -----------------------------------------------------------------------------------*/
#pragma interrupt_level 1		/*  Tells compiler that this routine will not be
											 called by Main() and the ISR at the same time. */

unsigned char SPI_Read(unsigned char address){
	unsigned char spi_data;

	SPI_CS = 0;

	SSPBUF = address;	   /* Send the register address */
	while (!STAT_BF);		/* wait for ssp to finish */
	spi_data = SSPBUF;	/* Should be AAh */

	SSPBUF = SPI_READ;	/* Send the read request */
	while (!STAT_BF);		/* wait for ssp to finish */
	spi_data = SSPBUF;	/* Should be 55h */

	SSPBUF = 0x00;			/* SPIDummy tx to initiate bus cycle */
	while (!STAT_BF);		/* wait for ssp to finish */

	SPI_CS = 1;
	return SSPBUF;
}

/* -----------------------------------------------------------------------------------
 * Function Name: SPI_Write(uint address)
 * This functions writes one byte to the SPI address specified.
 * -----------------------------------------------------------------------------------*/
 #pragma interrupt_level 1
void SPI_Write(unsigned char address, unsigned char data){

	SPI_CS = 0;

	SSPBUF = address;		/* Send the register address */
	while (!STAT_BF);		/* wait for ssp to finish */
	SPIDummy = SSPBUF;   /* Should be AAh */

	SSPBUF = SPI_WRITE;		/* Send the write instruction */
	while (!STAT_BF);			/* wait for ssp to finish */
	SPIDummy = SSPBUF;		/* Should be 55h */

	SSPBUF = data;			/* Write the data to the register specified */
	while (!STAT_BF);		/* wait for ssp to finish */
	SPIDummy = SSPBUF;

	SPI_CS = 1;
}

/*=============================================================
    DELAY_MS - add delay in milliseconds (ms_ctr)
    Note: Compile with Full Optimization Only
=============================================================*/
void delay_ms(unsigned char ms_ctr)
{
unsigned char us_ctr;
unsigned char i;
do
{
    i = 4;
    do
    {
        us_ctr = 996/4;
        while(--us_ctr != 0) continue;
    }
    while(--i);
}
while(--ms_ctr);
}

/* -----------------------------------------------------------------------------------
 * Function Name: SPI_Init()
 * This functions initializes the SPI bus.
 * Note: The PIC is the master on the SPI bus.
 * ----------------------------------------------------------------------------------- */
void SPI_Init(void){

	INTCON = 0;  // Interrupt control
	INTEDG = 0;  // Trigger on falling edge

	TRISB = 0x01;   // Port B direction
	TRISC = 0x90;	 // Port C Direction

	SSPCON = 0x10;		/* Make Master Mode Fosc/4 */
	CKP = 1;				/* Transmit of falling receive on rising */
	SSPEN = 1;		 	/* Enable the SPI bus */
	SPI_CS = 1;			/* Don't select the chip yet.*/

	/* reset the device */
	SPI_RST = 1;
	SPI_RST = 0;

	delay_ms(2);

	SPI_RST = 1;
}


