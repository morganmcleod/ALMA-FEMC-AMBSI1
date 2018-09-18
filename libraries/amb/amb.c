/*
 *****************************************************************************
 # $Id: amb.c,v 1.15 2004/11/18 19:57:56 avaccari Exp $
 #
 # Copyright (C) 1999
 # Associated Universities, Inc. Washington DC, USA.
 #
 # Correspondence concerning ALMA should be addressed as follows:
 #        Internet email: mmaswgrp@nrao.edu
 ****************************************************************************
 *
 *  AMB.C 
 *
 *  Implementation file for ALMA Monitor and Control Bus Slave library.  This
 *  file makes use of libraries for the support of Dallas Semiconductor device
 *  used for a unique serial number.  It is also intended to be the basis for
 *  slave implementations on hardware other than the suggested interface.
 *
 *****************************************************************************
 */

/* Version of AMB protocol */
#define PROTOCOL_VERSION_MAJOR 1
#define PROTOCOL_VERSION_MINOR 1
#define PROTOCOL_VERSION_PATCH 1

/* REVISION HISTORY */
/*
 * Version 01.01.01 - Released as Ver_1_1_1
		   			  Patch by Andrea Vaccari - NRAO NTC
		   			  Added the function "amb_unregister_last_function" declaration in the
		   			  amb.h file.
 * Version 01.01.00 - Released as Ver_1_1_0
					  Minor change by Andrea Vaccari - NRAO NTC
					  Added the function "amb_unregister_last_function".
					  If any callback function has been registered, this function will
                      unregister the last registered function.
 * Version 01.00.01 - Released as Ver_1_0_1
		              Patch by Andrea Vaccari - NRAO Tucson
      		          This patch modified the location of all the global variables
                      and moved them from external memory to internal memory (IDATA).
		              The result was an increased speed of execution.
 * Version 01.00.00 - Original Version developed by Mick Brooks - NRAO Tucson
*/



#ifdef C167_ARCH /* Include C167 register definitions */

	#include <reg167.h>
	#include <intrins.h>

#endif /* C167_ARCH */

#include "amb.h"

#ifdef C167_ARCH /* Include Dallas Semiconductor support for C167 */

	#include "..\ds1820\ds1820.h"

#endif /* C167_ARCH */

#ifdef PIC_ARCH /* Include SPI support for talking to 82527 */

	#include <pic.h>
	#include "spi_pic.h"

#endif /* PIC_ARCH */

/* Definitions of CAN controller structure */

#ifdef C167_ARCH /* For Siemens C167CR */

	/* Locations of CAN controller registers */
	#define C1CSR   (*((uword volatile sdata *) 0xEF00)) /* Control/Status Register */
	#define C1IR    (*((uword volatile sdata *) 0xEF02)) /* Interrupt Register */
	#define C1BTR   (*((uword volatile sdata *) 0xEF04)) /* Bit Timing Register */
	#define C1GMS   (*((uword volatile sdata *) 0xEF06)) /* Global Mask Short */
	#define C1UGML  (*((uword volatile sdata *) 0xEF08)) /* Upper Global Mask Long */
	#define C1LGML  (*((uword volatile sdata *) 0xEF0A)) /* Lower Global Mask Long */
	#define C1UMLM  (*((uword volatile sdata *) 0xEF0C)) /* Upper Mask of Last Message */
	#define C1LMLM  (*((uword volatile sdata *) 0xEF0E)) /* Lower Mask of Last Message */

	/* 
	 * Structure for a single 82527 CAN object
	 * A total of 15 such object structures exists
	 */

	struct can_obj {
	  uword  MCR;       /* Message Control Register */
	  uword  UAR;       /* Upper Arbitration Register */
	  uword  LAR;       /* Lower Arbitration Register */
	  ubyte  MCFG;      /* Message Configuration Register */
	  ubyte  Data[8];   /* Message Data 0 .. 7 */
	  ubyte  Customer;  /* Reserved for application specific data */
	} ;

	/* Location of first CAN object */
	#define CAN_OBJ ((struct can_obj volatile sdata *) 0xEF10)

#endif /* C167_ARCH */

#ifdef PIC_ARCH /* For PIC 16C74 and Intel 82527 */

	/* General purpose 82527 register addresses */
	#define CONTROL	0x00  /* Control register */
	#define STATUS  0x01  /* Status register */
	#define CPUIF	0x02  /* CPU Interface register */
	#define HISPLO	0x04  /* High Speed read register low byte */
	#define HISPHI  0x05  /* High Speed read register high byte */
	#define GMS0	0x06  /* Global Mask Standard register: bits 3-11 */
	#define GMS1	0x07  /* Global Mask Standard register: bits 0-2 */
	#define GME0	0x08  /* Global Mask Extended register: bits 21-28 */
	#define GME1	0x09  /* Global Mask Extended register: bits 13-20 */
	#define GME2	0x0A  /* Global Mask Extended register: bits 5-12 */
	#define GME3	0x0B  /* Global Mask Extended register: bits 0-4 */
	#define MLM0	0x0C  /* Mask of Last Message register: bits 21-28 */
	#define MLM1	0x0D  /* Mask of Last Message register: bits 13-20 */
	#define MLM2	0x0E  /* Mask of Last Message register: bits 5-12 */
	#define MLM3	0x0F  /* Mask of Last Message register: bits 0-4 */
	#define CLKOUT  0x1F  /* Clockout register */
	#define BUSCONF 0x2F  /* Bus Configuration register */
	#define BTR0	0x3F  /* Bit Timing Register 0 */
	#define BTR1	0x4F  /* Bit Timing Register 1 */
	#define INTR	0x5F  /* Interrupt register */
	#define SERRST  0xFF  /* Serial Reset address */

	/* Addresses of the 15 message objects */
	#define MSGOBJ1  0x10
	#define MSGOBJ2  0x20
	#define MSGOBJ3  0x30
	#define MSGOBJ4  0x40
	#define MSGOBJ5  0x50
	#define MSGOBJ6  0x60
	#define MSGOBJ7  0x70
	#define MSGOBJ8  0x80
	#define MSGOBJ9  0x90
	#define MSGOBJ10 0xA0
	#define MSGOBJ11 0xB0
	#define MSGOBJ12 0xC0
	#define MSGOBJ13 0xD0
	#define MSGOBJ14 0xE0
	#define MSGOBJ15 0xF0

	/* Each message object has a similar structure separated by 16 bytes */
	/* The following macros return the correct address for msg in the range 1-15 */
	#define MSGCTL0(msg)       (msg+0x0)       	/* Control 0 register */
	#define MSGCTL1(msg)       (msg+0x1)	 	/* Control 1 register */
	#define MSGARB0(msg)       (msg+0x2)	    /* Arbitration 0 register: bits 21-28 */
	#define MSGARB1(msg)       (msg+0x3)	    /* Arbitration 1 register: bits 13-20 */
	#define MSGARB2(msg)       (msg+0x4)	    /* Arbitration 2 register: bits 5-12 */
	#define MSGARB3(msg)       (msg+0x5)	    /* Arbitration 3 register: bits 0-4 */
	#define MSGCFG(msg)        (msg+0x6)       	/* Message Configuration register */
	#define MSGDATA(msg,byte)  (msg+0x7+byte)	/* Data bytes: byte must be in range 0-7 */

#endif /* PIC_ARCH */

/*
 ****************************************************************************
 * Interrupt Vectors
 ****************************************************************************
 */

#ifdef C167_ARCH /* For Siemens C167CR */

	#define XP0INT   0x40

#endif /* C167_ARCH */

/* Local Function prototypes */
static ubyte 	amb_get_node_address();
static int		amb_get_serial_number();
static int		amb_setup_CAN_hw();
static void		amb_handle_transaction();
static void		amb_transmit_monitor();

/* All pertinent slave data */
#ifdef PIC_ARCH
	static bank1 struct slave_node {
#else
	static struct slave_node {
#endif
	ubyte 		serial_number[8];	/* From hardware device */
	ubyte 		node_address;		/* From DIP switch */
	ulong		base_address;		/* From node address */

	ubyte		revision_level[3];	/* Protocol version */
	uword		num_errors;			/* Number of CAN errors */
	ubyte		last_slave_error;	/* Last internal slave error */
	ulong		num_transactions;	/* Number of completed transactions */

	ubyte		identify_mode;		/* True when responding to identify broadcast */

	ubyte		num_cbs;			/* No of callbacks registered */
	CALLBACK_STRUCT	*cb_ops;		/* User supplied callbacks */
} idata slave_node;

/* Structure for sharing message data with callbacks */
#ifdef PIC_ARCH
	static bank1 CAN_MSG_TYPE current_msg;
#else
	static CAN_MSG_TYPE idata current_msg;
#endif

/* Initialise routine */
int amb_init_slave(void *cb_ops_memory){
/* Point to callback memory */
	slave_node.cb_ops = (CALLBACK_STRUCT *) cb_ops_memory;

/* Initially we have no registered callbacks */
	slave_node.num_cbs = 0;

/* Get the address of this slave from the hardware */
	slave_node.node_address = amb_get_node_address();

/* Calculate the base address */
	slave_node.base_address = ((ulong) (slave_node.node_address + 1)) * 262144;

/* Try to get the serial number from the hardware */
	if (amb_get_serial_number(slave_node.serial_number) != 0) {
		return -1;
	}

	/* Initialise counters and status */
	slave_node.revision_level[0] = PROTOCOL_VERSION_MAJOR;
	slave_node.revision_level[1] = PROTOCOL_VERSION_MINOR;
	slave_node.revision_level[2] = PROTOCOL_VERSION_PATCH;
	slave_node.num_errors = 0;
	slave_node.last_slave_error = 0x0;
	slave_node.num_transactions = 0;

	slave_node.identify_mode = FALSE;
	
/* Setup the CAN hardware */
	return amb_setup_CAN_hw();
}

/* Register callback routine */
int amb_register_function(ulong low_address, ulong high_address, read_or_write_func func){
/* Store callback info */
	slave_node.cb_ops[slave_node.num_cbs].low_address = low_address;
	slave_node.cb_ops[slave_node.num_cbs].high_address = high_address;
	slave_node.cb_ops[slave_node.num_cbs].cb_func = func;

/* Increment the number of callbacks */
	slave_node.num_cbs++;

/* Always succeeds */
	return 0;
}

/* Unregister last registered callback routine */
int amb_unregister_last_function(void){

/* If no function is registered, nothing to be done. */
	if(!slave_node.num_cbs){
		return 0;
	}

/* Decrement the number of callbacks. */
	slave_node.num_cbs--;

	return 0;
}

/* Startup routine */
int amb_start(){
	IEN = 1;

/* Always succeeds */
	return 0;
}

/* Read the slave address */
ubyte amb_get_node_address(){
#ifdef C167_ARCH /* Siemens C167CR */

	#ifdef AMBSI  /* Standard i/f, DIP switch on Port 3.1 to 3.6 */
		return (P3 & 0x7e) >> 1;
	#endif /* AMBSI */

	#ifdef SK167  /* Starter Kit test board, DIP switch on Port 7.1 to 7.6 */
		return (P7 & 0x7e) >> 1;
	#endif /* SK167 */

#endif /* C167_ARCH */

#ifdef PIC_ARCH

	return 0x10; /* What is port the DIP switch is connected to? */

#endif /* PIC_ARCH */
}

/* Read the serial number from the Dallas Semiconductor device */
int amb_get_serial_number(){
	#ifdef C167_ARCH /* Siemens C167CR */

		/* Initialise the 1-wire port (library defined) */
		if (ds1820_init() != 0) {
			slave_node.last_slave_error = NO_DS1820_E;
			return -1;
		}

		/* read the serial number */
		if (ds1820_get_sn(slave_node.serial_number) != 0) {
			slave_node.last_slave_error = NO_SN_E;
			return -1;
		} else {
			return 0;
		}

	#endif /* C167_ARCH */

	#ifdef PIC_ARCH

		/* Set slave_node.serial_number with PIC 1 wire routines */
   		slave_node.serial_number[0] = 0xa5;
		slave_node.serial_number[1] = 0x5a;
		slave_node.serial_number[2] = 0xa5;
		slave_node.serial_number[3] = 0x5a;
   		slave_node.serial_number[4] = 0xa5;
		slave_node.serial_number[5] = 0x5a;
		slave_node.serial_number[6] = 0xa5;
		slave_node.serial_number[7] = 0x5a;

		return 0;

	#endif /* PIC_ARCH */

}

/* Routine to setup an Intel 82527-like controller */
int amb_setup_CAN_hw(){
	#ifdef C167_ARCH

		ulong LAR, UAR;

		/* Set up for the various arbitration registers */

		/* calculate lower mask */
  		LAR = 0x00000000;
  		LAR += (slave_node.base_address & 0x0000001f) << 11;  /* ID  4.. 0 */
  		LAR += (slave_node.base_address & 0x00001fe0) >>  5;  /* ID 12.. 5 */

		/* calculate the upper mask */
  		UAR = 0x00000000;
  		UAR += (slave_node.base_address & 0x001fe000) >>  5;  /* ID 13..20 */
  		UAR += (slave_node.base_address & 0x1fe00000) >> 21;  /* ID 21..28 */

		/*  ------------ CAN Control/Status Register -------------- 
  		 *  start the initialization of the CAN Module 
		 */
  		C1CSR  = 0x0041;  /* set INIT and CCE */

	  	/*  ------------ Bit Timing Register ---------------------
  		 *  baudrate =  1000.000 KBaud
  		 *	 there are 5 time quanta before sample point
  		 *	 there are 4 time quanta after sample point
  		 *	 the (re)synchronization jump width is 2 time quanta
		 */
  		C1BTR  = 0x3440;  /* set Bit Timing Register */
  		C1GMS  = 0xE0FF;  /* set Global Mask Short Register */
  		C1UGML = 0xFFFF;  /* set Upper Global Mask Long Register */
  		C1LGML = 0xF8FF;  /* set Lower Global Mask Long Register */

	  	/*  ------------------------------------------------------------------------
  		 *  ----------------- Configure Message Object 1 ---------------------------
		 *  --- This message object is used to receive the global identify ---------
		 *  --- broadcast message on ID 0x00000000 ---------------------------------
  		 *  ------------------------------------------------------------------------
  		 *  Message object 1 is valid
  		 *  enable receive interrupt
		 */
  		CAN_OBJ[0].MCR  = 0x5599;    /* set Message Control Register */

	  	/*
  	 	 * message direction is receive
  		 * extended 29-bit identifier
    	 */
  		CAN_OBJ[0].MCFG = 0x04;      /* set Message Configuration Register */

	  	CAN_OBJ[0].UAR  = 0x0000;    /* set Upper Arbitration Register */
  		CAN_OBJ[0].LAR  = 0x0000;    /* set Lower Arbitration Register */

	  	/*  ------------------------------------------------------------------------
  		 *  ----------------- Configure Message Object 2 ---------------------------
		 *  --- This message object is used as a transmit object for the serial ----
		 *  --- number.  It can be RTR'd quickly from the master -------------------
  		 *  ------------------------------------------------------------------------
  		 *  Message object 2 is valid
		 */
  		CAN_OBJ[1].MCR  = 0x5695;    /* set Message Control Register */

	  	/* 
		 * message direction is transmit
  		 * extended 29-bit identifier
  		 * 8 valid data bytes
   		 */
  		CAN_OBJ[1].MCFG = 0x8C;      /* set Message Configuration Register */

	  	CAN_OBJ[1].UAR  = UAR;	 /* set Upper Arbitration Register of Object 2 */
  		CAN_OBJ[1].LAR  = LAR;	 /* set Lower Arbitration Register of Object 2 */	

	  	CAN_OBJ[1].Data[0] = slave_node.serial_number[0];   /* set data byte 0 */
  		CAN_OBJ[1].Data[1] = slave_node.serial_number[1];   /* set data byte 1 */
  		CAN_OBJ[1].Data[2] = slave_node.serial_number[2];   /* set data byte 2 */
  		CAN_OBJ[1].Data[3] = slave_node.serial_number[3];   /* set data byte 3 */
  		CAN_OBJ[1].Data[4] = slave_node.serial_number[4];   /* set data byte 4 */
  		CAN_OBJ[1].Data[5] = slave_node.serial_number[5];   /* set data byte 5 */
  		CAN_OBJ[1].Data[6] = slave_node.serial_number[6];   /* set data byte 6 */
  		CAN_OBJ[1].Data[7] = slave_node.serial_number[7];   /* set data byte 7 */

	  	/*  ------------------------------------------------------------------------
  		 *  ----------------- Configure Message Object 3 ---------------------------
		 *  --- This message object is used to transmit all monitor data back to ---
		 *  --- the master. --------------------------------------------------------
  		 *  ------------------------------------------------------------------------
  		 *  Message object 3 is valid
   		 */
	  	CAN_OBJ[2].MCR  = 0x5695;    /* set Message Control Register */

	  	/* 
		 * message direction is transmit
  		 * extended 29-bit identifier
  		 * 0 valid data bytes
      	 */
  		CAN_OBJ[2].MCFG = 0x0C;      /* set Message Configuration Register */

	  	CAN_OBJ[2].UAR  = 0x0000;    /* set Upper Arbitration Register */
  		CAN_OBJ[2].LAR  = 0x0000;    /* set Lower Arbitration Register */

	  	/*  ------------------------------------------------------------------------
  		 *  ----------------- Configure Message Objects 4 to 14 --------------------
		 *  --- These objects are not used at present ------------------------------
  		 *  ------------------------------------------------------------------------
		 */
  		CAN_OBJ[3].MCR  = 0x5555;    /* set Message Control Register */
  		CAN_OBJ[4].MCR  = 0x5555;    /* set Message Control Register */
	  	CAN_OBJ[5].MCR  = 0x5555;    /* set Message Control Register */
  		CAN_OBJ[6].MCR  = 0x5555;    /* set Message Control Register */
  		CAN_OBJ[7].MCR  = 0x5555;    /* set Message Control Register */
  		CAN_OBJ[8].MCR  = 0x5555;    /* set Message Control Register */
  		CAN_OBJ[9].MCR  = 0x5555;    /* set Message Control Register */
  		CAN_OBJ[10].MCR  = 0x5555;    /* set Message Control Register */
  		CAN_OBJ[11].MCR  = 0x5555;    /* set Message Control Register */
  		CAN_OBJ[12].MCR  = 0x5555;    /* set Message Control Register */
  		CAN_OBJ[13].MCR  = 0x5555;    /* set Message Control Register */

	  	/*  ------------------------------------------------------------------------
  		 *  ----------------- Configure Message Object 15 --------------------------
		 *  --- This object is used in Basic CAN mode to receive all M&C requests --
  		 *  ------------------------------------------------------------------------
  		 *  Message object 15 is valid
  		 *  enable receive interrupt
   		 */
  		CAN_OBJ[14].MCR  = 0x5599;    /* set Message Control Register */

	  	/* 
		 * message direction is receive
  		 * extended 29-bit identifier
   	 	 */
  		CAN_OBJ[14].MCFG = 0x04;      /* set Message Configuration Register */

		/* Set the mask so that only the upper 11 bits are compared with the incoming identifier.
		   This ensures that messages are correctly filtered for this slaves range of identifiers */
 	 	C1UMLM = 0xE0FF;           /* set Upper Mask of Last Message */
  		C1LMLM = 0x0000;           /* set Lower Mask of Last Message */

	  	CAN_OBJ[14].UAR  = UAR; /* set Upper Arbitration Register of Basic CAN object */
  		CAN_OBJ[14].LAR  = LAR; /* set Lower Arbitration Register of Basic CAN object */

	    /*
		 *  enable CAN interrupt
  		 *  CAN interrupt priority level(ILVL) = 13
  	 	 *  CAN interrupt group level (GLVL) = 3
    	 */
  		XP0IC = 0x0077;

	  	/* ------------ CAN Control/Status Register --------------
  		 *  reset CCE and INIT
  		 * enable interrupt generation from CAN Module
  		 * enable interrupt generation on a change of bit BOFF or EWARN
		 * No status interrupts!
   		 */
  		C1CSR = 0x000A;

	#endif /* C167_ARCH */

	#ifdef PIC_ARCH

		ubyte AR0, AR1, AR2, AR3;

		/* Initialise the SPI interface to the 82527 */
		SPI_Init();

		/* Set up for the various arbitration registers */

		/* calculate arbitration registers */
  		AR3 = (slave_node.base_address & 0x0000001f) << 3;   /* ID  4.. 0 */
  		AR2 = (slave_node.base_address & 0x00001fe0) >> 5;   /* ID 12.. 5 */
  		AR1 = (slave_node.base_address & 0x001fe000) >> 13;  /* ID 13..20 */
	  	AR0 = (slave_node.base_address & 0x1fe00000) >> 21;  /* ID 21..28 */

		/*  ------------ CAN Control/Status Register -------------- 
  		 *   start the initialization of the CAN Module 
		 */
		SPI_Write(CONTROL, 0x41); /* set INIT and CCE */
		SPI_Write(CPUIF, 0x41);   /* Set DMC to zero: MCLK = SCLK */
		SPI_Write(STATUS, 0x00);  /* Clear status register */
		SPI_Write(BUSCONF, 0x42); /* Disconnect RX1 input, bypass comparator */

	  	/*  ------------ Bit Timing Register ---------------------
  		 *  baudrate =  1000.000 KBaud
  		 *	 there are 5 time quanta before sample point
  		 *	 there are 4 time quanta after sample point
  		 *	 the (re)synchronization jump width is 2 time quanta
		 */
		/*	SPI_Write(BTR0, 0x40);    Set Bit Timing Registers 
			SPI_Write(BTR1, 0x34);    These are for 20MHz XTAL 
 		*/

	   	SPI_Write(BTR0, 0x40);
  		SPI_Write(BTR1, 0x23);    /* These are for 16MHz XTAL */
 

		SPI_Write(GMS0, 0xFF); /* set Global Mask Standard Register */
		SPI_Write(GMS1, 0xE0);

		SPI_Write(GME0, 0xFF); /* Set Global Mask Extended Registers */
		SPI_Write(GME1, 0xFF);
		SPI_Write(GME2, 0xFF);
		SPI_Write(GME3, 0xF8);

	  	/*  ------------------------------------------------------------------------
  		 *  ----------------- Configure Message Object 1 ---------------------------
		 *  --- This message object is used to receive the global identify ---------
		 *  --- broadcast message on ID 0x00000000 ---------------------------------
  		 *  ------------------------------------------------------------------------
  		 *  Message object 1 is valid
  		 *  enable receive interrupt
		 */
  		SPI_Write(MSGCTL0(MSGOBJ1), 0x99);    /* set Message Control Registers */
		SPI_Write(MSGCTL1(MSGOBJ1), 0x55);

	  	/* 
		 * message direction is receive
  		 * extended 29-bit identifier
   		 */
		SPI_Write(MSGCFG(MSGOBJ1), 0x04);  /* set Message Configuration Register */

		SPI_Write(MSGARB0(MSGOBJ1), 0x00); /* set Arbitration Registers */
		SPI_Write(MSGARB1(MSGOBJ1), 0x00);
		SPI_Write(MSGARB2(MSGOBJ1), 0x00);
		SPI_Write(MSGARB3(MSGOBJ1), 0x00);

	  	/*  ------------------------------------------------------------------------
  		 *  ----------------- Configure Message Object 2 ---------------------------
		 *  --- This message object is used as a transmit object for the serial ----
		 *  --- number.  It can be RTR'd quickly from the master -------------------
  		 *  ------------------------------------------------------------------------
  		 *  Message object 2 is valid
		 */
  		SPI_Write(MSGCTL0(MSGOBJ2), 0x95);    /* set Message Control Registers */
		SPI_Write(MSGCTL1(MSGOBJ2), 0x56);

	  	/* 
		 * message direction is transmit
  		 * extended 29-bit identifier
  		 * 8 valid data bytes 
		 */
		SPI_Write(MSGCFG(MSGOBJ2), 0x8C);  /* set Message Configuration Register */

		SPI_Write(MSGARB0(MSGOBJ2), AR0); /* set Arbitration Registers */
		SPI_Write(MSGARB1(MSGOBJ2), AR1);
		SPI_Write(MSGARB2(MSGOBJ2), AR2);
		SPI_Write(MSGARB3(MSGOBJ2), AR3);

		SPI_Write(MSGDATA(MSGOBJ2,0), slave_node.serial_number[0]);   /* set data byte 0 */
		SPI_Write(MSGDATA(MSGOBJ2,1), slave_node.serial_number[1]);   /* set data byte 1 */
		SPI_Write(MSGDATA(MSGOBJ2,2), slave_node.serial_number[2]);   /* set data byte 2 */
		SPI_Write(MSGDATA(MSGOBJ2,3), slave_node.serial_number[3]);   /* set data byte 3 */
		SPI_Write(MSGDATA(MSGOBJ2,4), slave_node.serial_number[4]);   /* set data byte 4 */
		SPI_Write(MSGDATA(MSGOBJ2,5), slave_node.serial_number[5]);   /* set data byte 5 */
		SPI_Write(MSGDATA(MSGOBJ2,6), slave_node.serial_number[6]);   /* set data byte 6 */
		SPI_Write(MSGDATA(MSGOBJ2,7), slave_node.serial_number[7]);   /* set data byte 7 */

	  	/*  ------------------------------------------------------------------------
  		 *  ----------------- Configure Message Object 3 ---------------------------
		 *  --- This message object is used to transmit all monitor data back to ---
		 *  --- the master. --------------------------------------------------------
	  	 *  ------------------------------------------------------------------------
	  	 *  Message object 3 is valid 
	    */
	  	SPI_Write(MSGCTL0(MSGOBJ3), 0x95);    /* set Message Control Registers */
		SPI_Write(MSGCTL1(MSGOBJ3), 0x56);

	  	/* 
		 * message direction is transmit
	  	 * extended 29-bit identifier
	  	 * 0 valid data bytes 
	 	 */
		SPI_Write(MSGCFG(MSGOBJ3), 0x0C);  /* set Message Configuration Register */

		SPI_Write(MSGARB0(MSGOBJ3), 0x00); /* set Arbitration Registers */
		SPI_Write(MSGARB1(MSGOBJ3), 0x00);
		SPI_Write(MSGARB2(MSGOBJ3), 0x00);
		SPI_Write(MSGARB3(MSGOBJ3), 0x00);

	  	/*  ------------------------------------------------------------------------
  		 *  ----------------- Configure Message Objects 4 to 14 --------------------
		 *  --- These objects are not used at present ------------------------------
	  	 *  ------------------------------------------------------------------------
		 */
  		SPI_Write(MSGCTL0(MSGOBJ4), 0x55);    /* set Message Control Registers */
		SPI_Write(MSGCTL1(MSGOBJ4), 0x55);
	  	SPI_Write(MSGCTL0(MSGOBJ5), 0x55);    /* set Message Control Registers */
		SPI_Write(MSGCTL1(MSGOBJ5), 0x55);
	  	SPI_Write(MSGCTL0(MSGOBJ6), 0x55);    /* set Message Control Registers */
		SPI_Write(MSGCTL1(MSGOBJ6), 0x55);
	  	SPI_Write(MSGCTL0(MSGOBJ7), 0x55);    /* set Message Control Registers */
		SPI_Write(MSGCTL1(MSGOBJ7), 0x55);
	  	SPI_Write(MSGCTL0(MSGOBJ8), 0x55);    /* set Message Control Registers */
		SPI_Write(MSGCTL1(MSGOBJ8), 0x55);
	  	SPI_Write(MSGCTL0(MSGOBJ9), 0x55);    /* set Message Control Registers */
		SPI_Write(MSGCTL1(MSGOBJ9), 0x55);
	  	SPI_Write(MSGCTL0(MSGOBJ10), 0x55);    /* set Message Control Registers */
		SPI_Write(MSGCTL1(MSGOBJ10), 0x55);
	  	SPI_Write(MSGCTL0(MSGOBJ11), 0x55);    /* set Message Control Registers */
		SPI_Write(MSGCTL1(MSGOBJ11), 0x55);
	  	SPI_Write(MSGCTL0(MSGOBJ12), 0x55);    /* set Message Control Registers */
		SPI_Write(MSGCTL1(MSGOBJ12), 0x55);
	  	SPI_Write(MSGCTL0(MSGOBJ13), 0x55);    /* set Message Control Registers */
		SPI_Write(MSGCTL1(MSGOBJ13), 0x55);
	  	SPI_Write(MSGCTL0(MSGOBJ14), 0x55);    /* set Message Control Registers */
		SPI_Write(MSGCTL1(MSGOBJ14), 0x55);

	  	/*  ------------------------------------------------------------------------
	  	 *  ----------------- Configure Message Object 15 --------------------------
		 *  --- This object is used in Basic CAN mode to receive all M&C requests --
	  	 *  ------------------------------------------------------------------------
	  	 *  Message object 15 is valid
	  	 *  enable receive interrupt 
	    */
  		SPI_Write(MSGCTL0(MSGOBJ15), 0x99);    /* set Message Control Registers */
		SPI_Write(MSGCTL1(MSGOBJ15), 0x55);

	  	/* 
		 * message direction is receive
  		 * extended 29-bit identifier
	 	 */
		SPI_Write(MSGCFG(MSGOBJ15), 0x04);  /* set Message Configuration Register */

		/* Set the mask so that only the upper 10 bits are compared with the incoming identifier.
		This ensures that messages are correctly filtered for this slaves range of identifiers */
		SPI_Write(MLM0, 0xFF);
		SPI_Write(MLM1, 0xC0);
		SPI_Write(MLM2, 0x00);
		SPI_Write(MLM3, 0x00);

		SPI_Write(MSGARB0(MSGOBJ15), AR0); /* set Arbitration Registers */
		SPI_Write(MSGARB1(MSGOBJ15), AR1);
		SPI_Write(MSGARB2(MSGOBJ15), AR2);
		SPI_Write(MSGARB3(MSGOBJ15), AR3);

		/*  
		 *  enable external CAN interrupt and global enable
	    */

	    INTE = 1;
    	GIE = 1;

	  	/* ------------ CAN Control/Status Register --------------
	  	 *   reset CCE and INIT
	  	 * enable interrupt generation from CAN Module
	  	 * enable interrupt generation on a change of bit BOFF or EWARN
		 * No status interrupts! */
	  	SPI_Write(CONTROL, 0x0A);

	#endif /* PIC_ARCH */

	/* Always succeeds */
	return 0;
}

/*
 ****************************************************************************
 *  This is the interrupt service routine for the CAN controller.
 *  It is executed if:
 *  - the busoff or the error warning status is reached 
 *    (EIE is set)
 *  - a message has been sent or received successfully or a bus 
 *    error occurred (SIE is set)
 *  - the bit INTPND (interrupt pending) in one of the message
 *    object control-registers is set (at Tx or Rx)
 * 
 ****************************************************************************
 */

#ifdef C167_ARCH

	void amb_can_isr(void) interrupt XP0INT{
  	uword uwIntID;
  	uword uwStatus;

	  	while (uwIntID = C1IR & 0x00ff) {
	    	switch (uwIntID & 0x00ff) {
	     		case 1:  /* Status Change Interrupt
    	     	     	 * The CAN controller has updated (not necessarily changed)
        	    	  	 * the status in the Control Register.
						 */

					uwStatus = C1CSR;
		            if (uwStatus & 0x8000) { /* if BOFF */
        		    	/* 
						 * Indicates when the CAN controller is in busoff state.
						 * Increment error counter
					 	 */
						slave_node.num_errors++;
					}

		            if (uwStatus & 0x4000) { /* if EWRN */
        		       /* 
						* Indicates that the least one of the error counters in the
                		* EML has reached the error warning limit of 96.
					 	* Increment the error counter
					 	*/
						slave_node.num_errors++;
            		}

 		           if (uwStatus & 0x0800) { /* if TXOK */
              			/* 
					 	 * Indicates that a message has been transmitted successfully
              	 		 * (error free and acknowledged by at least one other node). 
					 	 */
              			uwStatus &= 0xf7ff;
              			C1CSR = uwStatus;     /* reset TXOK */

				 		/* If we are responding to the identify broadcast, then we are done */
						if (slave_node.identify_mode == TRUE) {
							/* Turn status interrupts off */
							C1CSR = 0x000A;
							slave_node.identify_mode = FALSE;
						}
        		    }

		            if (uwStatus & 0x1000) { /* if RXOK */
        		      	/* Indicates that a message has been received successfully. */
              			uwStatus &= 0xefff;
		              	C1CSR = uwStatus;     /* reset RXOK */
        		    }

		            if (uwStatus & 0x0700) { /* if LEC */
        			    switch ((uwStatus & 0x0700) >> 8) { /* LEC (Last Error Code) */
               				case 1: /* Stuff Error
                        			 * More than 5 equal bits in a sequence have occurred
			                         * in a part of a received message where this is not
                        			 * allowed. 
									 */
								/* Increment error counter */
								slave_node.num_errors++;
	               				break;

			               case 2: /* Form Error
            			            * A fixed format part of a received frame has the
                        			* wrong format. 
									*/
								/* Increment error counter */
				 			 	slave_node.num_errors++;
        			          	break;

			               case 3: /* Ack Error
            			            * The message this CAN controller transmitted was
                        			* not acknowledged by another node. 
									*/
								/* Increment error counter */
								slave_node.num_errors++;
                  				break;

			               case 4: /* Bit1 Error
                        			* During the transmission of a message (with the
                        			* exception of the arbitration field), the device
                        			* wanted to send a recessive level ("1"), but the
                        			* monitored bus value was dominant.
									*/
								/* Increment error counter */
					 			slave_node.num_errors++;

								/* 
								 * If we are responding to an identify request, this means a
								 * duplicate slave address was transmitted simultaneously
								 */
								if (slave_node.identify_mode == TRUE) {
									slave_node.last_slave_error = DUP_SLAVE_ADDR_E;
								}
				                break;

			               case 5: /* Bit0 Error
            			            * During the transmission of a message (or acknow-
                        			* ledge bit, active error flag, or overload flag),
                        			* the device wanted to send a dominant level ("0"),
                        			* but the monitored bus value was recessive. During
                        			* busoff recovery this staus is set each time a
                        			* sequence of 11 recessive bits has been monitored.
                        			* This enables the CPU to monitor the proceeding of
                        			* the busoff recovery sequence (indicating the bus
                        			* is not stuck at dominant or continously disturbed).
									*/
				               	if (uwStatus & 0x8000) { /* if Busoff status */
									/* Increment error counter */
						 			slave_node.num_errors++;
                  				} else {
									/* Increment error counter */
									slave_node.num_errors++;
                  				}
                  				break;

			               case 6: /* CRC Error
                        			* The CRC check sum was incorrect in the message
                        			* received.
									*/
								/* Increment error counter */
					 			slave_node.num_errors++;
                  				break;

			               default:
                  				break;
               			}
            		}
            		break;

				case 2: /* Message Object 15 Interrupt */
    	     		if ((CAN_OBJ[14].MCR & 0x0c00) == 0x0800) { /* if MSGLST set */
	        	    	/* 
						 * Indicates that the CAN controller has stored a new
   		     	    	 * message into object 15, while NEWDAT was still set,
        	    	 	 * ie. the previously stored message is lost.
					 	 */
           			 	CAN_OBJ[14].MCR = 0xf7ff;    /* reset MSGLST */

						/* 
						 * Messages in this object are probably M&C data, so 
						 * do something wih them  Increment error, because we missed
						 * a message 
						 */
						slave_node.num_errors++;

						if (slave_node.last_slave_error != DUP_SLAVE_ADDR_E) {
							amb_handle_transaction();
						}
    	     		} else {
        	       		/* 
						 * The CAN controller has stored a new message
        	   	     	 * into this object.
						 * Messages in this object are probably M&C data, so
						 * do something wih them.
						 */
						if (slave_node.last_slave_error != DUP_SLAVE_ADDR_E) {
							amb_handle_transaction();
						}
            		}
           			CAN_OBJ[14].MCR = 0x7dfd;      /* release buffer */
            		break;

				case 3: /* Message Object 1 Interrupt */
        		 	if ((CAN_OBJ[0].MCR & 0x0300) == 0x0200) {    /* if NEWDAT set */
             		 	if ((CAN_OBJ[0].MCR & 0x0c00) == 0x0800) { /* if MSGLST set */
               				/* 
					 		 * Indicates that the CAN controller has stored a new
               				 * message into this object, while NEWDAT was still set,
               				 * ie. the previously stored message is lost. 
							 */

			               	CAN_OBJ[0].MCR = 0xf7ff;  /* reset MSGLST */

							/* We are responding to the identify broadcast */
							slave_node.identify_mode = TRUE;

							/* Turn status interrupts on */
							C1CSR = 0x000E;

					  		/* Send the serial number in message object 2 */
							slave_node.num_transactions++;
							CAN_OBJ[1].MCR = 0xe7ff;  /* set TXRQ,reset CPUUPD */

							/* This is an error, because we missed a message */
							slave_node.num_errors++;
        		        } else {
                			/* 
							 * The CAN controller has stored a new message
                 		  	 * into this object.
						 	 */

							/* We are responding to the identify broadcast */
							slave_node.identify_mode = TRUE;

							/* Turn status interrupts on */
							C1CSR = 0x000E;

							/* Send the serial number */
							slave_node.num_transactions++;
							CAN_OBJ[1].MCR = 0xe7ff;  /* set TXRQ,reset CPUUPD */
		                }
					
						CAN_OBJ[0].MCR = 0xfdfd;  /* reset NEWDAT, INTPND */
         			}
	            	break;

	     		default:
    		        break;
			}
		}
	}

#endif /* C167_ARCH */

#ifdef PIC_ARCH

	#pragma interrupt_level 1

	static void interrupt
	amb_can_isr(void)
	{
  		ubyte ubIntID;
  		ubyte ubStatus;

		if (INTE && INTF) {

		    /* Disable interrupts while in the ISR */
     		INTE = 0;
			INTF = 0;
	   		GIE = 0;

	  		while (ubIntID = SPI_Read(INTR)) {
    			switch (ubIntID) {
     				case 1:  /* Status Change Interrupt
         				      * The CAN controller has updated (not necessarily changed)
            		  	  	  * the status in the Control Register.
							  */

						ubStatus = SPI_Read(STATUS);
   	        			if (ubStatus & 0x80) { /* if BOFF */
      	      				/* 
						 	 * Indicates when the CAN controller is in busoff state.
						 	 * Increment error counter 
						 	 */
							slave_node.num_errors++;
						}

	            		if (ubStatus & 0x40) { /* if EWRN */
   	            			/* 
						 	 * Indicates that the least one of the error counters in the
         	       			 * EML has reached the error warning limit of 96.
						 	 * Increment the error counter
						 	 */
							slave_node.num_errors++;
	            		}

	            		if (ubStatus & 0x08) { /* if TXOK */
   	           				/* 
						 	 * Indicates that a message has been transmitted successfully
         	     	 		 * (error free and acknowledged by at least one other node).
						 	 */
              				ubStatus &= 0xf7;
              				SPI_Write(STATUS, ubStatus);     /* reset TXOK */

					 		/* If we are responding to the identify broadcast, then we are done */
							if (slave_node.identify_mode == TRUE) {
								/* Turn status interrupts off */
								SPI_Write(CONTROL, 0x0A);

								slave_node.identify_mode = FALSE;
							}
						}

		            	if (ubStatus & 0x10) { /* if RXOK */
	   	           			/* Indicates that a message has been received successfully. */
    	  	        		ubStatus &= 0xef;
        	 	     		SPI_Write(STATUS, ubStatus);     /* reset RXOK */
            			}

		            	if (ubStatus & 0x07) { /* if LEC */
   		         			switch (ubStatus & 0x07) { /* LEC (Last Error Code) */
      		         			case 1: /* Stuff Error
         		               			 * More than 5 equal bits in a sequence have occurred
            		            		 * in a part of a received message where this is not
               	    	     			 * allowed. 
									   	 */
									/* Increment error counter */
									slave_node.num_errors++;
	      	         				break;

								case 2: /* Form Error
   		                     			 * A fixed format part of a received frame has the
      		                  			 * wrong format. 
									     */
									/* Increment error counter */
									slave_node.num_errors++;
			            	      	break;

		               			case 3: /* Ack Error
   		                     			 * The message this CAN controller transmitted was
      		                  			 * not acknowledged by another node. 
									   	 */
									/* Increment error counter */
									slave_node.num_errors++;
	              	   				break;

	               				case 4: /* Bit1 Error
    	                    			 * During the transmission of a message (with the
        	                			 * exception of the arbitration field), the device
            	            			 * wanted to send a recessive level ("1"), but the
                	        			 * monitored bus value was dominant. 
								  		 */
									/* Increment error counter */
						 			slave_node.num_errors++;

									/* 
									 * If we are responding to an identify request, this means a 
									 * duplicate slave address was transmitted simultaneously 
									 */
									if (slave_node.identify_mode == TRUE) {
										slave_node.last_slave_error = DUP_SLAVE_ADDR_E;
									}

									break;

				   	            case 5: /* Bit0 Error
		        		                 * During the transmission of a message (or acknow-
      		             			     * ledge bit, active error flag, or overload flag),
         		        		         * the device wanted to send a dominant level ("0"),
            		           			 * but the monitored bus value was recessive. During
               	    	    			 * busoff recovery this staus is set each time a
                  	    				 * sequence of 11 recessive bits has been monitored.
                     	  				 * This enables the CPU to monitor the proceeding of
                      				   	 * the busoff recovery sequence (indicating the bus
                      			   		 * is not stuck at dominant or continously disturbed).
										 */
			               			if (ubStatus & 0x80) { /* if Busoff status */
										/* Increment error counter */
										slave_node.num_errors++;
            	      				} else {
										/* Increment error counter */
										slave_node.num_errors++;
        	          				}
            	      				break;

				               case 6: /* CRC Error
   		                     			* The CRC check sum was incorrect in the message
      		                  			* received. 
									  	*/
									/* Increment error counter */
							 		slave_node.num_errors++;
                  					break;

		               			default:
   		               				break;
							}
						}
		        	    break;

		  	     	case 2: /* Message Object 15 Interrupt */
						if ((SPI_Read(MSGCTL1(MSGOBJ15)) & 0x0C) == 0x08) { /* if MSGLST set */
	    	        		/* 
						 	 * Indicates that the CAN controller has stored a new
      	       		 		 * message into object 15, while NEWDAT was still set,
         	    		 	 * ie. the previously stored message is lost.
					    	 */
							SPI_Write(MSGCTL1(MSGOBJ15), 0xF7);    /* reset MSGLST */
	
							/* 
							 * Messages in this object are probably M&C data, so
							 * do something wih them  Increment error, because we missed
							 * a message 
							 */
							slave_node.num_errors++;

							if (slave_node.last_slave_error != DUP_SLAVE_ADDR_E) {
								amb_handle_transaction();
							}
         				} else {
            		   		/* 
							 * The CAN controller has stored a new message
	                		 * into this object.
							 * Messages in this object are probably M&C data, so
							 * do something wih them.
							 */
							if (slave_node.last_slave_error != DUP_SLAVE_ADDR_E) {
								amb_handle_transaction();
							}
	            		}

						SPI_Write(MSGCTL0(MSGOBJ15), 0xfd);      /* release buffer */
						SPI_Write(MSGCTL1(MSGOBJ15), 0x7d);
      		      		break;

					case 3: /* Message Object 1 Interrupt */
						if ((SPI_Read(MSGCTL1(MSGOBJ1)) & 0x03) == 0x02) {    /* if NEWDAT set */
							if ((SPI_Read(MSGCTL1(MSGOBJ1)) & 0x0c) == 0x08) { /* if MSGLST set */
         		      			/* 
							 	 * Indicates that the CAN controller has stored a new
               	 				 * message into this object, while NEWDAT was still set,
	                			 * ie. the previously stored message is lost.
						 		 */

								SPI_Write(MSGCTL1(MSGOBJ1), 0xF7);    /* reset MSGLST */
	
								/* We are responding to the identify broadcast */
								slave_node.identify_mode = TRUE;
	
								/* Turn status interrupts on */
								SPI_Write(CONTROL, 0x0E);

								/* Send the serial number in message object 2 */
								slave_node.num_transactions++;
								SPI_Write(MSGCTL1(MSGOBJ2), 0xe7);  /* set TXRQ,reset CPUUPD */

								/* This is an error, because we missed a message */
								slave_node.num_errors++;
      		          		} else {
         		         		/*
								 * The CAN controller has stored a new message
               	    			 * into this object. 
								 */
	
								/* We are responding to the identify broadcast */
								slave_node.identify_mode = TRUE;
	
								/* Turn status interrupts on */
								SPI_Write(CONTROL, 0x0E);

								/* Send the serial number */
								slave_node.num_transactions++;
								SPI_Write(MSGCTL1(MSGOBJ2), 0xe7);  /* set TXRQ,reset CPUUPD */
         		       		}

							SPI_Write(MSGCTL0(MSGOBJ1), 0xfd);  /* reset NEWDAT, INTPND */
							SPI_Write(MSGCTL1(MSGOBJ1), 0xfd);
   			      		}
	      		   		break;

	     			default:
 		   	      		break;
				}
			}

	    	/* Enable interrupts again */
    		INTE = 1;
	   	 	GIE = 1;
		}
	}

#endif  /* PIC_ARCH */

/* Routine to check if a callback should be run */

void amb_handle_transaction(){
	ulong incoming_ID;
	ubyte i;

	/* Get incoming ID from Object 15 */
	incoming_ID = 0x0;
	#ifdef C167_ARCH
		incoming_ID += ((ulong) (CAN_OBJ[14].LAR & 0xf800)) >> 11;  /* ID  4.. 0 */
   		incoming_ID += ((ulong) (CAN_OBJ[14].LAR & 0x00ff)) <<  5;  /* ID 12.. 5 */
   		incoming_ID += ((ulong) (CAN_OBJ[14].UAR & 0xff00)) <<  5;  /* ID 13..20 */
   		incoming_ID += ((ulong) (CAN_OBJ[14].UAR & 0x00ff)) << 21;  /* ID 21..28 */
	#endif /* C167_ARCH */

	#ifdef PIC_ARCH
		incoming_ID += ((ulong) SPI_Read(MSGARB0(MSGOBJ15))) << 21;
		incoming_ID += ((ulong) SPI_Read(MSGARB1(MSGOBJ15))) << 13;
		incoming_ID += ((ulong) SPI_Read(MSGARB2(MSGOBJ15))) << 5;
		incoming_ID += ((ulong) SPI_Read(MSGARB3(MSGOBJ15))) >> 3;
	#endif /* PIC_ARCH */

	/* Calculate relative address from base address */
	current_msg.relative_address = incoming_ID - slave_node.base_address;

	/* Ignore messages that are outside our range */
	if ((current_msg.relative_address > 262143) ||
		(current_msg.relative_address <= 0))
		return;

	/* Get the message length */
	#ifdef C167_ARCH
		current_msg.len = (CAN_OBJ[14].MCFG & 0xf0) >> 4;
	#endif /* C167_ARCH */

	#ifdef PIC_ARCH
		current_msg.len = (SPI_Read(MSGCFG(MSGOBJ15)) & 0xf0) >> 4;	
	#endif /* PIC_ARCH */

	/* This is a monitor request if data length is zero */
	if (current_msg.len != 0) {
		current_msg.dirn = CAN_CONTROL;
		/* Control message: get the data */
		for (i=0; i<current_msg.len; i++)
			#ifdef C167_ARCH
				current_msg.data[i] = CAN_OBJ[14].Data[i];
			#endif /* C167_ARCH */
			#ifdef PIC_ARCH
				current_msg.data[i] = SPI_Read(MSGDATA(MSGOBJ15,i));
			#endif /* PIC_ARCH */
	} else {
		current_msg.dirn = CAN_MONITOR;
		/* Check for common monitor points */
		switch (current_msg.relative_address) {	
			case 0x30000: /* Slave revision level */
				current_msg.len = 3;
				current_msg.data[0] = (ubyte) (slave_node.revision_level[0]);
				current_msg.data[1] = (ubyte) (slave_node.revision_level[1]);
				current_msg.data[2] = (ubyte) (slave_node.revision_level[2]);
				amb_transmit_monitor();
				slave_node.num_transactions++;
				return;
				break;
			case 0x30001: /* Number of errors and last error */
				current_msg.len = 4;
				current_msg.data[0] = (ubyte) (slave_node.num_errors>>8);
				current_msg.data[1] = (ubyte) (slave_node.num_errors);
				current_msg.data[2] = 0x0;
			 	/* LEC from CAN controller */
				#ifdef C167_ARCH
					current_msg.data[3] = C1CSR >> 8;
				#endif /* C167_ARCH */
				#ifdef PIC_ARCH
					current_msg.data[3] = SPI_Read(STATUS);
				#endif /* PIC_ARCH */
				amb_transmit_monitor();
				slave_node.num_transactions++;
				return;
				break;
			case 0x30002: /* Number of transactions */
				current_msg.len = 4;
				current_msg.data[0] = (ubyte) (slave_node.num_transactions>>24);
				current_msg.data[1] = (ubyte) (slave_node.num_transactions>>16);
				current_msg.data[2] = (ubyte) (slave_node.num_transactions>>8);
				current_msg.data[3] = (ubyte) (slave_node.num_transactions);
				amb_transmit_monitor();
				slave_node.num_transactions++;
				return;
				break;
		}
	}

	/* For each registered callback, see if this message was in range */
	for (i=0; i<slave_node.num_cbs; i++) {
		if ((current_msg.relative_address >= slave_node.cb_ops[i].low_address) &&
			(current_msg.relative_address <= slave_node.cb_ops[i].high_address)) {

			/* Increment the transaction counter */
			slave_node.num_transactions++;

			(slave_node.cb_ops[i].cb_func)(&current_msg);

			if (current_msg.dirn == CAN_MONITOR)
				amb_transmit_monitor();
		}	
	}
}

/* Routine to send monitor data back to master using CAN object 3 */
void amb_transmit_monitor(){
  	ubyte i;
  	ulong TX_ID;
	#ifdef C167_ARCH
		ulong v;
	#endif /* C167_ARCH */

	#ifdef PIC_ARCH
		ubyte v;
	#endif /* PIC_ARCH */

	#ifdef C167_ARCH
  		CAN_OBJ[2].MCR = 0xfb7f;     /* set CPUUPD, reset MSGVAL */
	#endif /* C167_ARCH */

	#ifdef PIC_ARCH
		SPI_Write(MSGCTL0(MSGOBJ3), 0x7f);
		SPI_Write(MSGCTL1(MSGOBJ3), 0xfb);
	#endif /* PIC_ARCH */

	/* Recalculate CAN message from relative address */
	TX_ID = slave_node.base_address + current_msg.relative_address;

	/* Calculate the arbitration registers */
	#ifdef C167_ARCH
   		v = 0x00000000;
   		v += (TX_ID & 0x0000001f) << 11;  /* ID  4.. 0 */
   		v += (TX_ID & 0x00001fe0) >>  5;  /* ID 12.. 5 */
   		CAN_OBJ[2].LAR  = v;

	   	v = 0x00000000;
   		v += (TX_ID & 0x001fe000) >>  5;  /* ID 13..20 */
   		v += (TX_ID & 0x1fe00000) >> 21;  /* ID 21..28 */
   		CAN_OBJ[2].UAR  = v;
	#endif /* C167_ARCH */

	#ifdef PIC_ARCH
		v = ((ulong) (TX_ID & 0x1fe00000) >> (ulong) 21);
		SPI_Write(MSGARB0(MSGOBJ3), v);
		v = ((ulong) (TX_ID & 0x001fe000) >> (ulong) 13);
		SPI_Write(MSGARB1(MSGOBJ3), v);
		v = ((ulong) (TX_ID & 0x00001fe0) >> (ulong) 5);
		SPI_Write(MSGARB2(MSGOBJ3), v);
		v = ((ulong) (TX_ID & 0x0000001f) << (ulong) 3);
		SPI_Write(MSGARB3(MSGOBJ3), v);
	#endif /* PIC_ARCH */

	/* set transmit direction and length */
	#ifdef C167_ARCH
   		CAN_OBJ[2].MCFG = 0x0c | (current_msg.len << 4);
	#endif /* C167_ARCH */

	#ifdef PIC_ARCH
		SPI_Write(MSGCFG(MSGOBJ3), 0x0c | (current_msg.len << 4));
	#endif /* PIC_ARCH */
	
	/* Copy data to CAN object 3 */
   	for(i = 0; i < current_msg.len; i++) {
		#ifdef C167_ARCH 
      		CAN_OBJ[2].Data[i] = current_msg.data[i];
		#endif /* C167_ARCH */
		#ifdef PIC_ARCH
			SPI_Write(MSGDATA(MSGOBJ3,i), current_msg.data[i]);
		#endif /* PIC_ARCH */
	}

	#ifdef C167_ARCH
   		CAN_OBJ[2].MCR  = 0xf6bf;  /* set NEWDAT, reset CPUUPD, set MSGVAL */
	
		/* Transmit the object */
  		CAN_OBJ[2].MCR = 0xe7ff;  /* set TXRQ,reset CPUUPD */
	#endif /* C167_ARCH */

	#ifdef PIC_ARCH
		SPI_Write(MSGCTL0(MSGOBJ3), 0xbf);  /* Set MSGVAL */
		SPI_Write(MSGCTL1(MSGOBJ3), 0xf6);  /* set NEWDAT, reset CPUUPD */
	
		/* Transmit the object */
		SPI_Write(MSGCTL1(MSGOBJ3), 0xe7);  /* set TXRQ,reset CPUUPD */
	#endif /* PIC_ARCH */
}

void amb_get_rev_level(ubyte *major, ubyte *minor, ubyte *patch){
	*major = slave_node.revision_level[0];
	*minor = slave_node.revision_level[1];
	*patch = slave_node.revision_level[2];
}

void amb_get_error_status(uword *num_errors, ubyte *last_slave_error){
	*num_errors = slave_node.num_errors;
	*last_slave_error = slave_node.last_slave_error;
}

void amb_get_num_transactions(ulong *num_transactions){
	*num_transactions = slave_node.num_transactions;
}


