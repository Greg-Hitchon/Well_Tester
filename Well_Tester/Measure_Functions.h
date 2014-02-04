/*
 * Measure_Functions.h
 *
 *  Created on: Oct 20, 2013
 *      Author: Greg
 */
#include "cstbool.h"

#ifndef MEASURE_FUNCTIONS_H_
#define MEASURE_FUNCTIONS_H_

unsigned int Analog_Read(unsigned int, 
                    unsigned int);

unsigned long Frequency_Read(unsigned int Channel,
                            unsigned long Max_Input_Count,
                            unsigned long Max_Clock_Count,
                            bool Return_Total);

void Get_Result(void);
void Reset_Count(void);
void Initialize_Pulses(void);
bool Get_Pulse_Status(void);
bool Update_Pulse_Tracker(void);
                
#endif /* MEASURE_FUNCTIONS_H_ */
