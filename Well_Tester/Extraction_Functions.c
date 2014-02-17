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
#define STEPS_ADJUST (100)

void Extract_Liquid(void){
	//backup
	Straight(BACKWARD,STEPS_ADJUST,0);
	//just to simulate some time
	__delay_cycles(16000000);

	//return forward
	Straight(FORWARD,STEPS_ADJUST,0);
}
