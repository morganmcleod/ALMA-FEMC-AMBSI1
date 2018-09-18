/*# $Id: spi_pic.h,v 1.2 1999/12/17 21:06:36 mbrooks Exp $
//#
//# Copyright (C) 1999
//# Associated Universities, Inc. Washington DC, USA.
//#
//# Correspondence concerning ALMA should be addressed as follows:
//#        Internet email: mmaswgrp@nrao.edu
//****************************************************************************
//
//  SPI_PIC.H
//
//  Header file for Serial Parallel Interface on a PIC 16C74  This file is the
//  interface to the SPI_PIC module which implements SPI access routines on
//  the PIC architecture.  It is intended for use with the Intel 82527 CAN
//  controller only.
//
//****************************************************************************

// <summary>
// </summary>

// <synopsis>
// </synopsis>

// <example>
// <srcblock>
// </srcblock>
// </example>
*/

#ifndef SPI_PIC_H

#include <pic.h>

/* SPI Instruction Definitions */
#define	SPI_RESET	0xFF
#define 	SPI_READ		0x01
#define 	SPI_WRITE	0x81
#define 	SPI_STATUS	0xA0

#define PORTBIT(adr, bit)       ((unsigned)(&adr)*8+(bit))

static bit      SPI_CS	 	@ PORTBIT(PORTC, 0); 		// SPI Chip Select
static bit      SPI_SCK	 	@ PORTBIT(PORTC, 3); 		// SPI Clock
static bit      SPI_SDI	 	@ PORTBIT(PORTC, 4); 		// SPI Data In
static bit      SPI_SDO	 	@ PORTBIT(PORTC, 5); 		// SPI Data Out
static bit      SPI_RST	 	@ PORTBIT(PORTC, 6); 		// SPI Device reset (active low)

/*
 *   Function prototypes
 */

extern void SPI_Reset();
extern unsigned char SPI_Read(unsigned char address);
extern void SPI_Write(unsigned char address, unsigned char data);
extern void SPI_Init(void);
extern void delay_ms(unsigned char ms_ctr);
#endif /* SPI_PIC_H */
