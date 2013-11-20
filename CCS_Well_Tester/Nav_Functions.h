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
void Stop_Motor(unsigned int Motor_ID, bool Do_Save);
void Restore_State(unsigned int Motor_ID);
void Set_Timer(void);
void Set_Motor_Outputs(void);
void Start_Motor(unsigned int Motor_ID);
void Straight(	unsigned int Direction,
				unsigned long Left_Distance,
				unsigned long Right_Distance,
				unsigned int Left_Profile_ID,
				unsigned int Right_Profile_ID);
void Turn(	unsigned int Direction,
			unsigned int Left_Profile_ID,
			unsigned int Right_Profile_ID);

void Create_Nav_Profile(unsigned int Profile_ID,
						unsigned long Target_Speed,
						bool Do_Acc,
						bool Do_Dec);
void Set_Motor(	unsigned int Motor_ID,
				unsigned int Direction,
				unsigned long Left_Distance,
				unsigned long Right_Distance,
				unsigned int Left_Profile_ID,
				unsigned int Right_Profile_ID);

#endif /* NAV_FUNCTIONS_H_ */
