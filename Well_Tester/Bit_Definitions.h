/*
 * Bit_Definitions.h
 *
 *  Created on: Oct 17, 2013
 *      Author: Greg
 *
 *      NOTE: All bit definitions are no bigger than 8 bits
 */

#ifndef BIT_DEFINITIONS_H_
#define BIT_DEFINITIONS_H_

#define BIT_PUMP						(0x01)

#define BIT_TXD                         (0x04)

#define LEFT			 				(0x0)
#define RIGHT 							(0x1)
   
#define FORWARD                         (0x0)
#define BACKWARD                        (0x1)

//these are a little shady but put to show relationship with left/right explicitly.
//LEFT/RIGHT have to be 0/1 or 1/0
#define LEFT_MOTOR 					(LEFT+1)
#define RIGHT_MOTOR 		        (RIGHT+1)
#define BOTH_MOTORS 		        (LEFT_MOTOR+RIGHT_MOTOR)
#define CONC_MOTOR					(LEFT_MOTOR)

#define LEFT_FORWARD 		        (0x1)
#define LEFT_BACKWARD 		        (0x2)
#define RIGHT_FORWARD 		        (0x4)
#define RIGHT_BACKWARD 		        (0x8)
#define BOTH_FORWARD		        (RIGHT_FORWARD+LEFT_FORWARD)
#define BOTH_BACKWARD		        (RIGHT_BACKWARD+LEFT_BACKWARD)
#define TURN_LEFT					(LEFT_BACKWARD+RIGHT_FORWARD)
#define TURN_RIGHT					(LEFT_FORWARD+RIGHT_BACKWARD)

#define BIT_TRIGGER					(BIT6)
#define BIT_ECHO					(BIT1)
//(BIT_LEFT_MOTOR+BIT_RIGHT_MOTOR)

#define SWEEP						(0x1)
#define DIME						(0x2)

#define INPUT_RUN_TYPE				(0x20)
#define INPUT_LIGHT					(0x20)
#define INPUT_CONDUCTIVITY			(0x02)

#define NORTH						(0x1)
#define EAST						(0x2)
#define SOUTH						(0x4)
#define WEST						(0x8)

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



#endif /* BIT_DEFINITIONS_H_ */
