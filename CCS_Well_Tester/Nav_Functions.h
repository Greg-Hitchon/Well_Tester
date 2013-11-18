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
void Straight(unsigned int, unsigned long, unsigned int);
void Turn(unsigned int, unsigned int);
void Create_Nav_Profile(unsigned int Profile_ID,
						unsigned int Target_Speed,
						bool Do_Acc,
						bool Do_Dec);

#endif /* NAV_FUNCTIONS_H_ */
