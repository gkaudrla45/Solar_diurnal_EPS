#ifndef _DEFINE_H  
#define _DEFINE_H  
   
#endif

#define OFF 0
#define ON  1

/***************************************
 - pin configuration
***************************************/


/***************************************
 - CONSTANT
***************************************/


/***************************************
 - communication
***************************************/

#define STX     0x02
#define STX_IDX     0
#define CMD_CTRL   0xA7
#define CMD_STATE_REQ  0xA9
#define CMD_IDX     1
#define ETX     0x03

// State
#define STATE_OFF			0x30
#define STATE_ON			0x31
#define STATE_STABLE	0x32
#define STATE_NORMAL	0x33
#define STATE_OVER_CURRENT		0x35
#define STATE_OVER_TEMP			0x36
#define STATE_50			0x38
#define STATE_100			0x6A

#define BUFFER_SIZE 256