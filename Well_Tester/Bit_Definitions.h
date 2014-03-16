/*
 * Bit_Definitions.h
 *
 *  Created on: Oct 17, 2013
 *      Author: Greg
 *
 *      NOTE: All bit definitions are no bigger than 8 bits unsigned or 0-255
 */

#define PCB_MODE true

#ifndef BIT_DEFINITIONS_H_
#define BIT_DEFINITIONS_H_

//bit specifications for input/output on msp
//#define BIT_UNKNOWN					(BIT0)
#define BIT_ECHO						(BIT1)
#define BIT_TXD                         (BIT2)
#define INPUT_CONDUCTIVITY				(BIT3)
#define INPUT_LIGHT						(BIT4)
#define BIT_PUMP						(BIT5)
#define BIT_TRIGGER						(BIT6)
//#define BIT_UNKNOWN					(BIT7)


#if PCB_MODE == true
//Current PCB
#define LA1								(BIT0)
#define LA2								(BIT2)
#define LB1								(BIT1)
#define LB2								(BIT6)

#define RA1								(BIT3)
#define RA2								(BIT4)
#define RB1								(BIT7)
#define RB2								(BIT5)

#else
//Current Breadboard
#define LA1								(BIT0)
#define LA2								(BIT2)
#define LB1								(BIT1)
#define LB2								(BIT3)

#define RA1								(BIT4)
#define RA2								(BIT6)
#define RB1								(BIT5)
#define RB2								(BIT7)
#endif

//turn directions (always forwards, ie cant do a backwards sweep)
#define LEFT			 				(0x0)
#define RIGHT 							(0x1)

//straight directions
#define FORWARD                         (0x0)
#define BACKWARD                        (0x1)

//these are a little shady but put to show relationship with left/right explicitly.
//LEFT/RIGHT have to be 0/1 or 1/0
#define LEFT_MOTOR 					(LEFT+1)
#define RIGHT_MOTOR 		        (RIGHT+1)
#define BOTH_MOTORS 		        (LEFT_MOTOR+RIGHT_MOTOR)
#define CONC_MOTOR					(LEFT_MOTOR)

//numeric constants for input to set_motor function
#define LEFT_FORWARD 		        (0x1)
#define LEFT_BACKWARD 		        (0x2)
#define RIGHT_FORWARD 		        (0x4)
#define RIGHT_BACKWARD 		        (0x8)
#define BOTH_FORWARD		        (RIGHT_FORWARD+LEFT_FORWARD)
#define BOTH_BACKWARD		        (RIGHT_BACKWARD+LEFT_BACKWARD)
#define TURN_LEFT					(LEFT_BACKWARD+RIGHT_FORWARD)
#define TURN_RIGHT					(LEFT_FORWARD+RIGHT_BACKWARD)

//numeric constants for element types
#define SWEEP						(0x1)
#define DIME						(0x2)
#define LINE						(0x4)

//numeric constants for direction
//NOTE: these need to remain the same in order for reorient function to work properly
//Re-Orient relies on unsigned integer subtraction to determine what turns need to be made
#define NORTH						UINT8_C(0)
#define EAST						UINT8_C(64)
#define SOUTH						UINT8_C(128)
#define WEST						UINT8_C(192)

//numeric constants for liquid types (LT's)
#define LT_ORANGE_JUICE				(0x1)
#define LT_APPLE_JUICE				(0x2)
#define LT_COKE						(0x3)
#define LT_MALT_VINEGAR				(0x4)
#define LT_WHITE_VINEGAR			(0x5)
#define LT_MINERAL_OIL				(0x6)
#define LT_VEGETABLE_OIL			(0x7)
#define LT_SUGAR_WATER				(0x8)
#define LT_SALT_WATER				(0x9)
#define LT_DISTILLED_WATER			(0xA)
#define LT_NO_LIQUID				(0xB)



#endif /* BIT_DEFINITIONS_H_ */
