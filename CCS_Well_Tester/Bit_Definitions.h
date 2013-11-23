/*
 * Bit_Definitions.h
 *
 *  Created on: Oct 17, 2013
 *      Author: Greg
 */

#ifndef BIT_DEFINITIONS_H_
#define BIT_DEFINITIONS_H_

#define BIT_EXTRACT                     (0x10)
#define BIT_STARTUP						(0x08)
#define BIT_TXD                         (0x4)
#define LED_RED							(0x40)
#define LED_GREEN						(0x80)

#define LEFT			 				(0x0)
#define RIGHT 							(0x1)
   
#define FORWARD                         (0x0)
#define BACKWARD                        (0x1)

#define LEFT_MOTOR 					(0x1U)
#define RIGHT_MOTOR 		        (0x2U)
#define BOTH_MOTORS 		        (0x3U)

#define LEFT_FORWARD 		        (0x1)
#define LEFT_BACKWARD 		        (0x2)
#define RIGHT_FORWARD 		        (0x4)
#define RIGHT_BACKWARD 		        (0x8)
#define BOTH_FORWARD		        (0x5)
#define BOTH_BACKWARD		        (0xA)
#define TURN_LEFT					(0x6)
#define TURN_RIGHT					(0x9)

#define BIT_MLA         	        (0x2)
#define BIT_MLB         	        (0x4)
#define BIT_MRA         	        (0x8)
#define BIT_MRB         	        (0x10)
#define BIT_LEFT_MOTOR		        BIT_MLA+BIT_MLB
#define BIT_RIGHT_MOTOR		        BIT_MRA+BIT_MRB
#define BIT_ALL_MOTORS		        BIT_LEFT_MOTOR+BIT_RIGHT_MOTOR


#endif /* BIT_DEFINITIONS_H_ */
