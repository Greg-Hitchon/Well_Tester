/*
 * Test.h
 *
 *  Created on: Oct 16, 2013
 *      Author: Greg
 */

#include "cstbool.h"

#ifndef NAV_FUNCTIONS_H_
#define NAV_FUNCTIONS_H_

__interrupt void PORT1_ISR(void);
__interrupt void TIMER1_OTHER_ISR(void);
void Restore_State(uint8_t Motor_ID);
void Set_Timer(void);
void Set_Motor_Outputs(void);
void Start_Motor(uint8_t Motor_ID);
void Straight(	uint8_t Direction,
				uint32_t Steps,
				uint8_t Profile_ID);

void Turn(	uint8_t Direction,
			uint8_t Profile_ID,
			uint8_t Type);

void Create_Nav_Profile(uint8_t Profile_ID,
						uint16_t Start_Speed,
						uint16_t Target_Speed,
						uint16_t End_Speed,
						uint16_t ACC_Rate,
						uint16_t DEC_Rate,
						uint16_t ACC_Period,
						uint16_t DEC_Period);

void Set_Motor(	uint8_t Motor_ID,
				uint8_t Direction,
				uint32_t Steps,
				uint8_t Profile_ID);

void Initialize_Bits(void);
void Initialize_Tracking(void);
void Cup_Found(void);
void Go_Home(void);
void Set_Up_Extraction(void);
void Wait_For_Startup(void);
void Update_XY_Coords(uint32_t Steps, uint8_t Direction);

#endif /* NAV_FUNCTIONS_H_ */
