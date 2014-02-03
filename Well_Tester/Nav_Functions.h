/*
 * Test.h
 *
 *  Created on: Oct 16, 2013
 *      Author: Greg
 */

#include "cstbool.h"

#ifndef NAV_FUNCTIONS_H_
#define NAV_FUNCTIONS_H_

__interrupt void TIMER0_OTHER_ISR(void);
void Restore_State(unsigned int Motor_ID);
void Set_Timer(void);
void Set_Motor_Outputs(void);
void Start_Motor(unsigned int Motor_ID);
void Straight(	unsigned int Direction,
				unsigned long Steps,
				unsigned int Profile_ID);

void Turn(	unsigned int Direction,
			unsigned int Profile_ID,
			unsigned int Type);

void Create_Nav_Profile(unsigned int Profile_ID,
						unsigned int Start_Speed,
						unsigned int Target_Speed,
						unsigned int End_Speed,
						unsigned int ACC_Rate,
						unsigned int DEC_Rate,
						unsigned int ACC_Period,
						unsigned int DEC_Period);

void Set_Motor(	unsigned int Motor_ID,
				unsigned int Direction,
				unsigned long Steps,
				unsigned int Profile_ID);

void Initialize_Bits(void);

#endif /* NAV_FUNCTIONS_H_ */
