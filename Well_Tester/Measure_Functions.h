/*
 * Measure_Functions.h
 *
 *  Created on: Oct 20, 2013
 *      Author: Greg
 */

#ifndef MEASURE_FUNCTIONS_H_
#define MEASURE_FUNCTIONS_H_

unsigned int Analog_Read(unsigned int, 
                    unsigned int);

unsigned long Frequency_Read(unsigned int Channel,
                            unsigned long Max_Input_Count,
                            unsigned long Max_Clock_Count,
                            bool Return_Total);

                
#endif /* MEASURE_FUNCTIONS_H_ */