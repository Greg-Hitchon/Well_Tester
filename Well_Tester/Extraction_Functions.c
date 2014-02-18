/*
 * Extraction_Functions.c
 *
 *  Created on: 2014-02-03
 *      Author: greg
 */

//system includes
#include <stdint.h>

//user defined includes
#include "Nav_Functions.h"
#include "Bit_Definitions.h"


//parameters
#define STEPS_ADJUST_FORWARD (0)
#define STEPS_ADJUST_BACKWARD (300)

void Extract_Liquid(void){
	//dependent on placement has to move forward more
	if(STEPS_ADJUST_FORWARD > 0){
		Straight(FORWARD,STEPS_ADJUST_FORWARD,0);
	}
	//backup
	if(STEPS_ADJUST_BACKWARD > 0){
		Straight(BACKWARD,STEPS_ADJUST_BACKWARD,0);
	}

	//just to simulate some time
	__delay_cycles(16000000);
}
