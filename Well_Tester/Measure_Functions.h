/*
 * Measure_Functions.h
 *
 *  Created on: Oct 20, 2013
 *      Author: Greg
 */
#include <stdint.h>
#include "cstbool.h"

#ifndef MEASURE_FUNCTIONS_H_
#define MEASURE_FUNCTIONS_H_

unsigned int Analog_Read(uint8_t Channel,
						uint16_t Sample_Target);

void Get_Result(void);
void Reset_Count(void);
void Initialize_Pulses(void);
bool Get_Pulse_Status(void);
bool Update_Pulse_Tracker(void);
                
#endif /* MEASURE_FUNCTIONS_H_ */
