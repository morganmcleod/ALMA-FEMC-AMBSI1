/*!	\file	main.c
	\brief	AMBSI1 firmware code

	This is the firmware to be loaded into the AMBSI1 to allow the AMBSI1 to work as a
	bridge between the CAN bus contained in the AMB and the ARCOM embedded controller
	handling the front end hardware.
*/
/* Defines */

//! Longest timeout allowed waiting for acknowledgment from ARCOM board
/*! During each phase of monitoring, a count-down timer counts from \p MAX_TIMEOUT
	down to zero unless an acknowledgment is received. */
#define MAX_TIMEOUT 500    
// about 530 microseconds based on 0xFFFF = 70 ms

//! Is the firmware using the 48 ms pulse?
/*! Defines if the 48ms pulse is used to trigger the corresponding interrupt.
	If yes then P8.0 will not be available for use as a normal I/O pin since
	the assigned pin on the C167 will be jumpered to receive the 48ms pulse.

	\note	To be able to use the 48 ms pulse, it is necessary to program the
	 	  	Xilinx chip to allow the incoming pulse to be passed through. */
// #define USE_48MS

#define MAX_CAN_MSG_PAYLOAD			8		// Max CAN message payload size. Used to determine if error occurred

//! \b 0x20000 -> Base address for the special monitor RCAs
/*! This is the starting relative %CAN address for the special monitor
    requests available in the firmware. */

#define BASE_SPECIAL_MONITOR_RCA    0x20000L
#define GET_AMBSI1_VERSION_INFO     0x20000L    //!< Get the firmware version of this firmware.
#define GET_SETUP_INFO              0x20001L    //!< In versions 1.0.0 and 1.0.1 a monitor request to this initiates communication between the AMBSI1 and the ARCOM.
                                                //!< In version 1.2.x communication is established automatically at power-up.  This request still sends a reply for compatibility with ALMA and FETMS software.
#define GET_ARCOM_VERSION_INFO      0x20002L	//!< Get the ARCOM Pegasus firware version.
#define GET_SPECIAL_MONITOR_RCAS    0x20003L	//!< Get the special monitor RCA range from ARCOM. DEPRECATED
#define GET_SPECIAL_CONTROL_RCAS    0x20004L	//!< Get the special control RCA range from ARCOM. DEPRECATED
#define GET_MONITOR_RCAS            0x20005L	//!< Get the standard monitor RCA range from the ARCOM firmware.
#define GET_CONTROL_RCAS            0x20006L	//!< Get the standard control RCA range from the ARCOM firmware.
#define GET_LO_PA_LIMITS_TABLE_ESN  0x20010L    //!< 0x20010 through 0x20019 return the PA LIMITS table ESNs.

// We carve out some of the special monitor RCAs for timers and debugging of this firmware:
#define BASE_AMBSI1_RESERVED        0x20020L    //!< Lowest special RCA served by this firmware not forwarded to ARCOM.
#define GET_MON_TIMERS1_RCA         0x20020L    //!< Get monitor timing countdown registers 1-4.
#define GET_MON_TIMERS2_RCA         0x20021L    //!< Get monitor timing countdown registers 5-7 and the value of MAX_TIMEOUT.
#define GET_CMD_TIMERS1_RCA         0x20022L    //!< Get command timing countdown registers 1-4.
#define GET_CMD_TIMERS2_RCA         0x20023L    //!< Get command timing countdown registers 5-6 and the value of MAX_TIMEOUT.
#define GET_PPORT_STATE             0x20024L    //!< Get the state of the parallel port lines and other state info
#define LAST_AMBSI1_RESERVED        0x2003FL    //!< Highest special RCA served by this firmware not forwarded to ARCOM.

/* Version Info */
#define VERSION_MAJOR 01	//!< Major Version
#define VERSION_MINOR 03	//!< Minor Revision
#define VERSION_PATCH 00	//!< Patch Level

/* Uses serial port */
#include <reg167.h>
#include <intrins.h>
#include <string.h>

/* include library interface */
#include "..\libraries\amb\amb.h"
#include "..\libraries\ds1820\ds1820.h"

/* Set aside memory for the callbacks in the AMB library
   This is larger than the number of handlers because some handlers get registered for more than one range.
   There should be a slot here for each call to amb_register_function() in this program.
   This being too small caused a buffer overflow in 1.2.0 and before! */
static CALLBACK_STRUCT idata cb_memory[9];

/* A global CAN message used for setting up the parallel port link */
static CAN_MSG_TYPE idata myCANMessage;

/* CAN command queue */
#define CMD_QUEUE_SIZE 32   // not chosen scientifically!
static CAN_MSG_TYPE idata cmdQueue[CMD_QUEUE_SIZE];
static unsigned char idata queueWritePos = 0;
static unsigned char idata queueReadPos = 0;

#define QUEUE_NOT_EMPTY (queueWritePos != queueReadPos)

#define QUEUE_READ (cmdQueue + queueReadPos++)

#define QUEUE_CHECK_BOUNDS(pos) { _atomic_(0); if (pos == CMD_QUEUE_SIZE) pos = 0; _endatomic_(); }

#define QUEUE_WRITE(p_src) { \
    memcpy(cmdQueue + (queueWritePos++), p_src, sizeof(CAN_MSG_TYPE)); \
    QUEUE_CHECK_BOUNDS(queueWritePos); }

/* forward declare CAN message callbacks and helpers */
int ambient_msg(CAN_MSG_TYPE *message);       //!< Called to get the board temperature temperature
int controlMsg(CAN_MSG_TYPE *message);        //!< Called to handle CAN control messages
int implControlSingle(CAN_MSG_TYPE *message); //! helper function
int monitorMsg(CAN_MSG_TYPE *message);        //!< Called to handle CAN monitor messages
int implMonitorSingle(CAN_MSG_TYPE *message, unsigned char sendReply);  //! helper function
int getSetupInfo(CAN_MSG_TYPE *message);      //!< Called to get the AMBSI1 <-> ARCOM link/setup information
int getVersionInfo(CAN_MSG_TYPE *message);    //!< Called to get firmware version informations
int getReservedMsg(CAN_MSG_TYPE *message);    //!< Monitor timers and debugging info from this firmware

/* A global for the last read temperature */
static ubyte idata ambient_temp_data[4];

/* External bus control signal buffer chip enable is on P4.7 */
sbit  DISABLE_EX_BUF	= P4^7;

/* Arcom Parallel port connection lines */
sbit  WRITE				= P2^2;
sbit  DSTROBE			= P2^3;
sbit  WAIT				= P2^8;
sbit  INT				= P2^7;
sbit  SELECT			= P2^10;
sbit  INIT				= P2^5;

/* Separate timers for each phase of monitor and control transaction */
static unsigned int idata monTimer1, monTimer2, monTimer3, monTimer4, monTimer5, monTimer6, monTimer7,
                          cmdTimer1, cmdTimer2, cmdTimer3, cmdTimer4, cmdTimer5, cmdTimer6;

/* Macro to implement handshake with ARCOM board */
// Wait for Data Strobe to go low
#define IMPL_HANDSHAKE(TIMER) for(TIMER = MAX_TIMEOUT; TIMER && DSTROBE; TIMER--) {}

/* Macros to use when debugging timers.
 * Sets the timer countdowns so that a subsequent call to GET_MON_TIMERS1_RCA etc.
 *   will show if a timeout occurred before a transaction phase showing 0xFFFF.
 * Not used in production code.
 */
#define RESET_MON_TIMERS { \
    monTimer1 = monTimer2 = monTimer3 = monTimer4 = monTimer5 = monTimer6 = monTimer7 = 0xFFFF; }

#define RESET_CMD_TIMERS { \
    cmdTimer1 = cmdTimer2 = cmdTimer3 = cmdTimer4 = cmdTimer5 = cmdTimer6 = 0xFFFF; }

/* RCAs address ranges */
static unsigned long idata lowestMonitorRCA,highestMonitorRCA,
						   lowestControlRCA,highestControlRCA,
                           lowestSpecialMonitorRCA,highestSpecialMonitorRCA,
						   lowestSpecialControlRCA,highestSpecialControlRCA;

/* Globals to check for initialization of RCA */
static bit idata ready;			// is the communication between the ARCOM and AMBSI ready?
static bit idata initialized;	// have the RCAs been initialized?

//! MAIN
/*! Takes care of initializing the AMBSI1, the CAN subrutine and globally enables interrupts.
    version 1.2.0: also performs AMBSI1 to ARCOM link setup. */ 
void main(void) {
    unsigned long timer;

	#ifdef USE_48MS
	  // Setup the CAPCOM2 unit to receive the 48ms pulse from the Xilinx
		P8&=0xFE; // Set value of P8.0 to 0
		DP8&=0xFE; // Set INPUT direction for P8.0
		CCM4&=0xFFF0; // Clear setup for CCMOD16
		CCM4|=0x0001; // Set CCMOD16 to trigger on rising edge
		CC16IC=0x0078; // Interrupt: ILVL=14, GLVL=0;
	#endif // USE_48MS

	/* Make sure that external bus control signal buffer is disabled */
	DP4 |= 0x01;
	DISABLE_EX_BUF = 1;

	/* Initialise the slave library */
	if (amb_init_slave((void *) cb_memory) != 0) 
		return;

	/* Register callbacks for CAN events */
	if (amb_register_function(0x30003, 0x30003, ambient_msg) != 0)
		return;

	/* Register callbacks for CAN version information event (RCA -> 0x20000) */
	if (amb_register_function(GET_AMBSI1_VERSION_INFO, GET_AMBSI1_VERSION_INFO, getVersionInfo) != 0)
		return;

	/* Register callback for special monitor requests reserved by this firmware */
	if (amb_register_function(BASE_AMBSI1_RESERVED, LAST_AMBSI1_RESERVED, getReservedMsg) != 0)
        return;

	/* Initialize ports for communication */
    DP7=0x00;   //receive
    DP7=0xFF;   //transmit
    P2=0x0000;
    SELECT=1;
    INIT=0;
    DP2=0x0580;

	/* Register callbacks for CAN events (RCA -> 0x20001) */
	if (amb_register_function(GET_SETUP_INFO, GET_SETUP_INFO, getSetupInfo) != 0)
		return;

	/* globally enable interrupts */
    amb_start();

    /* this gets set true when all our handlers are registered */
  	initialized = FALSE;
  	ready = FALSE;
  	SELECT = 0;

    /* Handshake readiness status with ARCOM board */
    ready = 0;
    while (INIT) { // Wait of init line to go to 0. In the mean time read the temperature
        ds1820_get_temp(&ambient_temp_data[1], &ambient_temp_data[0], &ambient_temp_data[2], &ambient_temp_data[3]);
    }
    ready=1;

    /* Loop until the AMBSI1 to ARCOM link is established */
    while (!initialized) {
        /* Process a fake GET_SETUP_INFO request */
        myCANMessage.dirn=CAN_MONITOR;
        myCANMessage.len=0;
        myCANMessage.relative_address=GET_SETUP_INFO;
        if (getSetupInfo(&myCANMessage)) {
            // if timed out, sleep a bit:
            for (timer = 100000L; timer; timer--) {}   // about 0.1 second
        }
    }

    /* Never return */
    while (1) {
        if (initialized && QUEUE_NOT_EMPTY) {
            implControlSingle(QUEUE_READ);
            QUEUE_CHECK_BOUNDS(queueReadPos);
        } else {
            ds1820_get_temp(&ambient_temp_data[1], &ambient_temp_data[0], &ambient_temp_data[2], &ambient_temp_data[3]);
        }
    }
}

/*! This function will return the firmware version for the AMBSI1 board.
	\param	*message	a CAN_MSG_TYPE 
	\return	0 -	Everything went OK */
int getVersionInfo(CAN_MSG_TYPE *message){
	message->data[0]=VERSION_MAJOR;
	message->data[1]=VERSION_MINOR;
	message->data[2]=VERSION_PATCH;
	message->len=3;
	return 0;
}

int getReservedMsg(CAN_MSG_TYPE *message) {
    switch(message -> relative_address) {
        case GET_MON_TIMERS1_RCA:
            /*! return the timers for phases 1 through 4 of the last monitor request handled. */
            message -> data[1] = (unsigned char) (monTimer1);
            message -> data[0] = (unsigned char) (monTimer1 >> 8);
            message -> data[3] = (unsigned char) (monTimer2);
            message -> data[2] = (unsigned char) (monTimer2 >> 8);
            message -> data[5] = (unsigned char) (monTimer3);
            message -> data[4] = (unsigned char) (monTimer3 >> 8);
            message -> data[7] = (unsigned char) (monTimer4);
            message -> data[6] = (unsigned char) (monTimer4 >> 8);
            message -> len = 8;
            break;
        case GET_MON_TIMERS2_RCA:
            /*! return the timers for phases 5, 6, 7 of the last monitor request handled.
                the fourth byte is the starting countdown value MAX_TIMEOUT */
            message -> data[1] = (unsigned char) (monTimer5);
            message -> data[0] = (unsigned char) (monTimer5 >> 8);
            message -> data[3] = (unsigned char) (monTimer6);
            message -> data[2] = (unsigned char) (monTimer6 >> 8);
            message -> data[5] = (unsigned char) (monTimer7);
            message -> data[4] = (unsigned char) (monTimer7 >> 8);
            message -> data[7] = (unsigned char) (MAX_TIMEOUT);
            message -> data[6] = (unsigned char) (MAX_TIMEOUT >> 8);
            message -> len = 8;
            break;
        case GET_CMD_TIMERS1_RCA:
            /*! return the timers for phases 1 through 4 of the last command handled. */
            message -> data[1] = (unsigned char) (cmdTimer1);
            message -> data[0] = (unsigned char) (cmdTimer1 >> 8);
            message -> data[3] = (unsigned char) (cmdTimer2);
            message -> data[2] = (unsigned char) (cmdTimer2 >> 8);
            message -> data[5] = (unsigned char) (cmdTimer3);
            message -> data[4] = (unsigned char) (cmdTimer3 >> 8);
            message -> data[7] = (unsigned char) (cmdTimer4);
            message -> data[6] = (unsigned char) (cmdTimer4 >> 8);
            message -> len = 8;
            break;
        case GET_CMD_TIMERS2_RCA:
            /*! return the timers for phases 5, 6 of the last command handled.
                the fourth byte is the starting countdown value MAX_TIMEOUT */
            message -> data[1] = (unsigned char) (cmdTimer5);
            message -> data[0] = (unsigned char) (cmdTimer5 >> 8);
            message -> data[3] = (unsigned char) (cmdTimer6);
            message -> data[2] = (unsigned char) (cmdTimer6 >> 8);
            message -> data[5] = (unsigned char) 0;
            message -> data[4] = (unsigned char) 0;
            message -> data[7] = (unsigned char) (MAX_TIMEOUT);
            message -> data[6] = (unsigned char) (MAX_TIMEOUT >> 8);
            message -> len = 8;
            break;
        case GET_PPORT_STATE:
            message -> data[0] = (unsigned char) SELECT;
            message -> data[1] = (unsigned char) INIT;
            message -> data[2] = (unsigned char) INT;
            message -> data[3] = (unsigned char) DP7;
            message -> data[4] = (unsigned char) P7;
            message -> data[5] = (unsigned char) 0;
            message -> data[6] = (unsigned char) ready;
            message -> data[7] = (unsigned char) initialized;
            message -> len = 8;
            break;
        default:
            message -> data[1] = (unsigned char) 0;
            message -> data[0] = (unsigned char) 0;
            message -> data[3] = (unsigned char) 0;
            message -> data[2] = (unsigned char) 0;
            message -> data[5] = (unsigned char) 0;
            message -> data[4] = (unsigned char) 0;
            message -> data[7] = (unsigned char) 0;
            message -> data[6] = (unsigned char) 0;
            message -> len = 0;
    }
    return 0;
}

/*! This function get the RCAs info from the ARCOM board and register the appropriate CAN functions.
	
	This function will return a CAN message with 1 byte (uchar) payload. The meaning of the payload
	are as follows:
		- 0x00 -> No Error	
	   	- 0x05 -> No Error. Previous setup completed successfully.
	   	- 0x06 -> Communication between ARCOM and AMBSI not yet established
		- 0x07 -> Timeout while forwarding CAN message to the ARCOM board

	\param	*message	a CAN_MSG_TYPE
	\return
		- 0  -> Everything went OK
		- -1 -> ERROR */
int getSetupInfo(CAN_MSG_TYPE *message){

	/* The initialization message has to be a monitor message */
	if(message->dirn==CAN_CONTROL){
		return -1;
	}
	
	/* Return message size: 1 byte */
	message->len = 1;

	/* If not ready to communicate, return the message and wait */
	if(!ready){
		message->data[0]=0x06; // Error 0x06: communication between ARCOM and AMBSI not yet established
		return -1;
	}

	/* If already initialized do not inizialize again */
	if(initialized){
		message->data[0]=0x05; // Error 0x05: RCA already initialized
		return -1;
	}

	/* SPECIAL MONITOR RCAs */
	/* Get the information on the available special monitor RCAs from the ARCOM board */
	/* Set up custom can message to perform monitor request */
	myCANMessage.dirn=CAN_MONITOR; // Direction: monitor
	myCANMessage.len=0;	// Size: 0
	myCANMessage.relative_address=GET_SPECIAL_MONITOR_RCAS; // 0x20003 -> RCA: special address to retrieve the special monitor RCAs informations
	if (implMonitorSingle(&myCANMessage, FALSE)) { // Send the monitor request, but don't send response on CAN bus.
		message->data[0]=0x07; // Error 0x07: Timeout while forwarding the message to the ARCOM board
		return -1;
	}
	/* Rebuild highestMonitorRCA */
	highestSpecialMonitorRCA = 0;
	highestSpecialMonitorRCA += ((unsigned long)myCANMessage.data[7])<<24;
	highestSpecialMonitorRCA += ((unsigned long)myCANMessage.data[6])<<16;
	highestSpecialMonitorRCA += ((unsigned long)myCANMessage.data[5])<<8;
	highestSpecialMonitorRCA += ((unsigned long)myCANMessage.data[4]);
	/* Rebuild lowestMonitorRCA */
	lowestSpecialMonitorRCA = 0;
	lowestSpecialMonitorRCA += ((unsigned long)myCANMessage.data[3])<<24;
	lowestSpecialMonitorRCA += ((unsigned long)myCANMessage.data[2])<<16;
	lowestSpecialMonitorRCA += ((unsigned long)myCANMessage.data[1])<<8;
	lowestSpecialMonitorRCA += ((unsigned long)myCANMessage.data[0]);
	/* Register callbacks for special messages */
    amb_register_function(lowestSpecialMonitorRCA, BASE_AMBSI1_RESERVED - 1, monitorMsg);
	amb_register_function(LAST_AMBSI1_RESERVED + 1, highestSpecialMonitorRCA, monitorMsg);
	// 00020002 - 00020FFF

	/* SPECIAL CONTROL RCAs */
	/* Get the information on the available special control RCAs from the ARCOM board */
	/* Set up custom can message to perform monitor request */
	myCANMessage.dirn=CAN_MONITOR; // Direction: monitor
	myCANMessage.len=0;	// Size: 0
	myCANMessage.relative_address=GET_SPECIAL_CONTROL_RCAS; // 0x20004 -> RCA: special address to retrieve the special control RCAs informations
    if (implMonitorSingle(&myCANMessage, FALSE)) { // Send the monitor request, but don't send response on CAN bus.
        message->data[0]=0x07; // Error 0x07: Timeout while forwarding the message to the ARCOM board
        /* Unregister previously succesfully registered functions */
        amb_unregister_last_function(); // SPECIAL MONITOR RCAs
        return -1;
    }
	/* Rebuild highestMonitorRCA */
	highestSpecialControlRCA = 0;
	highestSpecialControlRCA += ((unsigned long)myCANMessage.data[7])<<24;
	highestSpecialControlRCA += ((unsigned long)myCANMessage.data[6])<<16;
	highestSpecialControlRCA += ((unsigned long)myCANMessage.data[5])<<8;
	highestSpecialControlRCA += ((unsigned long)myCANMessage.data[4]);
	/* Rebuild lowestMonitorRCA */
	lowestSpecialControlRCA = 0;
	lowestSpecialControlRCA += ((unsigned long)myCANMessage.data[3])<<24;
	lowestSpecialControlRCA += ((unsigned long)myCANMessage.data[2])<<16;
	lowestSpecialControlRCA += ((unsigned long)myCANMessage.data[1])<<8;
	lowestSpecialControlRCA += ((unsigned long)myCANMessage.data[0]);
	/* Register callbacks for special control RCA messages */
    amb_register_function(lowestSpecialControlRCA, highestSpecialControlRCA, controlMsg);
	// 00021000 - 00021FFF

	/* MONITOR RCAs */
	/* Get the information on the available monitor RCAs from the ARCOM board */
	/* Set up custom can message to perform monitor request */
	myCANMessage.dirn=CAN_MONITOR; // Direction: monitor
	myCANMessage.len=0;	// Size: 0
	myCANMessage.relative_address=GET_MONITOR_RCAS; // 0x20005 -> RCA: special address to retrieve the monitor RCAs informations
    if (implMonitorSingle(&myCANMessage, FALSE)) { // Send the monitor request, but don't send response on CAN bus.
        message->data[0]=0x07; // Error 0x07: Timeout while forwarding the message to the ARCOM board
        /* Unregister previously succesfully registered functions */
        amb_unregister_last_function(); // SPECIAL CONTROL RCAs
        amb_unregister_last_function(); // SPECIAL MONITOR RCAs
        return -1;
    }
	/* Rebuild highestMonitorRCA */
	highestMonitorRCA = 0;
	highestMonitorRCA += ((unsigned long)myCANMessage.data[7])<<24;
	highestMonitorRCA += ((unsigned long)myCANMessage.data[6])<<16;
	highestMonitorRCA += ((unsigned long)myCANMessage.data[5])<<8;
	highestMonitorRCA += ((unsigned long)myCANMessage.data[4]);
	/* Rebuild lowestMonitorRCA */
	lowestMonitorRCA = 0;
	lowestMonitorRCA += ((unsigned long)myCANMessage.data[3])<<24;
	lowestMonitorRCA += ((unsigned long)myCANMessage.data[2])<<16;
	lowestMonitorRCA += ((unsigned long)myCANMessage.data[1])<<8;
	lowestMonitorRCA += ((unsigned long)myCANMessage.data[0]);
	/* Register callbacks for special messages */
	amb_register_function(lowestMonitorRCA, highestMonitorRCA, monitorMsg);
	// 00000001 - 0000FFFF

	/* CONTROL RCAs */
	/* Get the information on the available special monitor RCAs from the ARCOM board */
	/* Set up custom can message to perform monitor request */
	myCANMessage.dirn=CAN_MONITOR; // Direction: monitor
	myCANMessage.len=0;	// Size: 0
	myCANMessage.relative_address=GET_CONTROL_RCAS; // 0x20006 -> RCA: special address to retrieve the special control RCAs informations
    if (implMonitorSingle(&myCANMessage, FALSE)) { // Send the monitor request, but don't send response on CAN bus.
        message->data[0]=0x07; // Error 0x07: Timeout while forwarding the message to the ARCOM board
        /* Unregister previously succesfully registered functions */
        amb_unregister_last_function(); // MONITOR RCAs
        amb_unregister_last_function(); // SPECIAL CONTROL RCAs
        amb_unregister_last_function(); // SPECIAL MONITOR RCAs
        return -1;
    }
	/* Rebuild highestMonitorRCA */
	highestControlRCA = 0;
	highestControlRCA += ((unsigned long)myCANMessage.data[7])<<24;
	highestControlRCA += ((unsigned long)myCANMessage.data[6])<<16;
	highestControlRCA += ((unsigned long)myCANMessage.data[5])<<8;
	highestControlRCA += ((unsigned long)myCANMessage.data[4]);
	/* Rebuild lowestMonitorRCA */
	lowestControlRCA = 0;
	lowestControlRCA += ((unsigned long)myCANMessage.data[3])<<24;
	lowestControlRCA += ((unsigned long)myCANMessage.data[2])<<16;
	lowestControlRCA += ((unsigned long)myCANMessage.data[1])<<8;
	lowestControlRCA += ((unsigned long)myCANMessage.data[0]);
	/* Register callbacks for special messages */
	amb_register_function(lowestControlRCA, highestControlRCA, controlMsg);
	// 00010000 - 0001FFFF

	/* No error */
	initialized=1; // Remember that the RCA have already been initialized
	message->data[0]=0;

    return 0;
}



/*! Return the temperature of the AMBSI as measured by the DS1820 onboard chip.

	\param	*message	a CAN_MSG_TYPE 
	\return	0 -	Everything went OK */
int ambient_msg(CAN_MSG_TYPE *message) {

	if (message->dirn == CAN_MONITOR) {  /* Should only be a monitor requests */
		message->len = 4;
		message->data[0] = ambient_temp_data[0];
		message->data[1] = ambient_temp_data[1];
		message->data[2] = ambient_temp_data[2];
		message->data[3] = ambient_temp_data[3];
	} 

	return 0;
}



/* Triggers every 48ms pulse */
void received_48ms(void) interrupt 0x30{
// Put whatever you want to be execute at the 48ms clock.
// Remember that right now this interrupt has higher priority than the CAN.
// Also to be able to use the 48ms, the Xilinx has to be programmed to connect the
// incoming pulse on (pin31) to the cpu (pin28).
}


/*! Implementation of one command transaction.
    Abstracted out so that commandMsg below use the command queue

    It will start communication with the ARCOM board triggering the parallel port
    interrupt and the sending the CAN message information to the ARCOM board.

    Since a CAN control request doesn't require any acknowledgment, this function
    will then return.

    \param  *message    a CAN_MSG_TYPE
    \return 0 - Everything went OK */
int implControlSingle(CAN_MSG_TYPE *message) {
    unsigned char counter;

    /* Trigger interrupt */
    INT = 1;

    //RESET_CMD_TIMERS Uncomment to debug timers.

    // Set port to transmit data:
    DP7 = 0xFF;

    /* Send RCA */
    IMPL_HANDSHAKE(cmdTimer1)
    P7 = (uword) (message->relative_address);   // Put data on port
    WAIT = 1;   // Acknowledge with Wait going high
    WAIT = 0;   /* Wait down as quick as possible for next message.
                   Any wait state will keep wait high too long and make the ARCOM believe it is an
                   acknowledgment to the following data strobe. */
    IMPL_HANDSHAKE(cmdTimer2)
    P7 = (uword) (message->relative_address>>8);    // Put data on port
    WAIT = 1;   // Acknowledge with Wait going high
    WAIT = 0;   /* Wait down as quick as possible for next message.
                   Any wait state will keep wait high too long and make the ARCOM believe it is an
                   acknowledgment to the following data strobe. */

    IMPL_HANDSHAKE(cmdTimer3)
    P7 = (uword) (message->relative_address>>16);   // Put data on port
    WAIT = 1;   // Acknowledge with Wait going high
    WAIT = 0;   /* Wait down as quick as possible for next message.
                   Any wait state will keep wait high too long and make the ARCOM believe it is an
                   acknowledgment to the following data strobe. */

    IMPL_HANDSHAKE(cmdTimer4)
    P7 = (uword) (message->relative_address>>24);   // Put data on port
    WAIT = 1;   // Acknowledge with Wait going high
    WAIT = 0;   /* Wait down as quick as possible for next message.
                   Any wait state will keep wait high too long and make the ARCOM believe it is an
                   acknowledgment to the following data strobe. */

    /* Send payload size (0 -> monitor message) */
    IMPL_HANDSHAKE(cmdTimer5)
    P7 = message->len;  // Put data on port (0 -> monitor message)
    WAIT = 1;   // Acknowledge with Wait going high
    WAIT = 0;   /* Wait down as quick as possible for next message.
                   Any wait state will keep wait high too long and make the ARCOM believe it is an
                   acknowledgment to the following data strobe. */

    for(counter=0;counter<message->len;counter++){
        IMPL_HANDSHAKE(cmdTimer6)
        P7 = message->data[counter];    // Put data on port (0 -> monitor message)
        WAIT = 1;   // Acknowledge with Wait going high
        WAIT = 0;   /* Wait down as quick as possible for next message.
                       Any wait state will keep wait high too long and make the ARCOM believe it is an
                       acknowledgment to the following data strobe. */
    }

    /* Untrigger interrupt */
    INT = 0;
    return 0;
}

/*! This function will be called in case a CAN control message is received.
	Now queues the command to be handled in the main wait loop and returns immediately.

	\param	*message	a CAN_MSG_TYPE
	\return	0 -	Everything went OK */
int controlMsg(CAN_MSG_TYPE *message) {
    // If this is a monitor request on a control RCA, forward the request to the monitor handler:
    if (message -> dirn == CAN_MONITOR)
        return monitorMsg(message);

    // Currently processing a request or there is a command ahead of this one?
    if (INT || QUEUE_NOT_EMPTY) {
        // Add this command to the queue:
        QUEUE_WRITE(message);
        return 0;
    } else
        return implControlSingle(message);
}


/*! Implementation of one monitor transaction.  
    Abstracted out so that monitorMsg below can retry
    
    \param  *message    a CAN_MSG_TYPE 
    \return
        - 0 -> Everything went OK
        - -1 -> Time out during CAN message forwarding */
int implMonitorSingle(CAN_MSG_TYPE *message, unsigned char sendReply) {
    unsigned char counter;
    unsigned char timeout;

    /* Trigger interrupt */
    INT = 1;

    //RESET_MON_TIMERS Uncomment to debug timers.

    // Set port to transmit data:
    DP7 = 0xFF;

    /* Send RCA */
    IMPL_HANDSHAKE(monTimer1)
    P7 = (uword) (message->relative_address);   // Put data on port
    WAIT = 1;   // Acknowledge with Wait going high
    WAIT = 0;   /* Wait down as quick as possible for next message.
                   Any wait state will keep wait high too long and make the ARCOM believe it is an
                   acknowledgment to the following data strobe. */

    IMPL_HANDSHAKE(monTimer2)
    P7 = (uword) (message->relative_address>>8);    // Put data on port
    WAIT = 1;   // Acknowledge with Wait going high
    WAIT = 0;   /* Wait down as quick as possible for next message.
                   Any wait state will keep wait high too long and make the ARCOM believe it is an
                   acknowledgment to the following data strobe. */

    IMPL_HANDSHAKE(monTimer3)
    P7 = (uword) (message->relative_address>>16);   // Put data on port
    WAIT = 1;   // Acknowledge with Wait going high
    WAIT = 0;   /* Wait down as quick as possible for next message.
                   Any wait state will keep wait high too long and make the ARCOM believe it is an
                   acknowledgment to the following data strobe. */

    IMPL_HANDSHAKE(monTimer4)
    P7 = (uword) (message->relative_address>>24);   // Put data on port
    WAIT = 1;   // Acknowledge with Wait going high
    WAIT = 0;   /* Wait down as quick as possible for next message.
                   Any wait state will keep wait high too long and make the ARCOM believe it is an
                   acknowledgment to the following data strobe. */

    /* Send payload size (0 -> monitor message) */
    IMPL_HANDSHAKE(monTimer5)
    P7 = message->len;  // Put data on port (0 -> monitor message)
    WAIT = 1;   // Acknowledge with Wait going high
    WAIT = 0;   /* Wait down as quick as possible for next message.
                   Any wait state will keep wait high too long and make the ARCOM believe it is an
                   acknowledgment to the following data strobe. */

    /* Set port to receive data */
    DP7 = 0x00;

    /* Receive monitor payload size */
    IMPL_HANDSHAKE(monTimer6)
    message->len = (ubyte) P7;  // Read data from port
    WAIT = 1;   // Acknowledge with Wait going high
    WAIT = 0;   /* Wait down as quick as possible for next message.
                   Any wait state will keep wait high too long and make the ARCOM believe it is an
                   acknowledgment to the following data strobe. */

    /* Detect timeout or error receiving payload size */
    timeout = FALSE;
    if (!monTimer6 || message->len > MAX_CAN_MSG_PAYLOAD)
        timeout = TRUE;

    /* Get the payload */
    for(counter = 0; !timeout && (counter < message -> len); counter++) {
        IMPL_HANDSHAKE(monTimer7)
        message->data[counter] = (ubyte) P7;    // Read data from port
        WAIT = 1;   // Acknowledge with Wait going high
        WAIT = 0;   /* Wait down as quick as possible for next message.
                       Any wait state will keep wait high too long and make the ARCOM believe it is an
                       acknowledgment to the following data strobe. */
        if(!monTimer7)
            timeout = TRUE;
    }
    // Set port to transmit data:
    DP7 = 0xFF;

    /* Untrigger interrupt */
    INT = 0;

    /* Handle timeout */
    if (timeout || !sendReply) {
        // We don't want to send back garbage data (as in earlier versions)
        // but there is no way to return a value which prevents transmitting the buffer.
        
        // Yucky workaround, tell the caller it's actually a control msg:       
        message -> dirn = CAN_CONTROL;
        message -> len=0;
    }
    if (timeout)
        return -1;
    else
        return 0;
}
    

/*! This function will be called in case a CAN monitor message is received.
	It will start communication with the ARCOM board triggering the parallel port
	interrupt and the sending the CAN message information to the ARCOM board.

	Since a CAN monitor request does require response within 150us, this function
	will then wait for data to come back from the ARCOM board.

	\param	*message	a CAN_MSG_TYPE 
	\return
		- 0 -> Everything went OK
	    - -1 -> Time out during CAN message forwarding */
int monitorMsg(CAN_MSG_TYPE *message) {
    int ret = 0;

    // If this is a control message a monitor RCA, forward the request to the control handler
    //  because we want any errors to be detected in the ARCOM firmware.
    if (message -> dirn == CAN_CONTROL)
        return controlMsg(message);

    // Try 1:
    ret = implMonitorSingle(message, TRUE);

    if (ret != 0) {
        // Retry once:
        ret = implMonitorSingle(message, TRUE);
    }
	return ret;
}

