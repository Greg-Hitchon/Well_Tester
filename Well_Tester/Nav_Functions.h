/*
 * Test.h
 *
 *  Created on: Oct 16, 2013
 *      Author: Greg
 */

#include "cstbool.h"

#ifndef NAV_FUNCTIONS_H_
#define NAV_FUNCTIONS_H_

void Set_Timer(void);
void Initialize_Bits(void);
void Initialize_Tracking(void);
void Clear_State(uint8_t Motor_ID);
void Cup_Found(void);
void Go_Home(void);
void Wait_For_Startup(void);
void Hold_Until_Finished(void);
void Start_Motor(uint8_t Motor_ID);
void Final_Run(void);
void Disable_Motors(void);

void Update_Track_Info(uint32_t Steps,
					uint8_t Direction);

void Straight(	uint8_t Direction,
				uint32_t Steps,
				uint8_t Profile_ID);

void Turn(	uint8_t Direction,
			uint8_t Profile_ID,
			uint8_t Type,
			uint32_t Steps);

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

void Execute_Cross(uint8_t Starting_Side,
					uint8_t Straight_ID,
					uint8_t Turn_ID,
					uint8_t Num_Crosses);

void Re_Orient(uint8_t Direction,
			uint8_t Turn_Type,
			uint8_t Profile_ID);


#endif /* NAV_FUNCTIONS_H_ */
