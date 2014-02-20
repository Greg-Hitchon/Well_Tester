/*
 * Extraction_Functions.c
 *
 *  Created on: 2014-02-03
 *      Author: greg
 */

//system includes
#include "Project_Parameters.h"
#include TEST_CHIP
#include <stdint.h>


//user defined includes
#include "Nav_Functions.h"
#include "Bit_Definitions.h"


//parameters
#define STEPS_ADJUST_FORWARD 	(0)
#define STEPS_ADJUST_BACKWARD 	(300)
#define WAIT_CYCLES				(10*16000000)

void Extract_Liquid(void){
	//dependent on placement has to move forward more
	if(STEPS_ADJUST_FORWARD > 0){
		Straight(FORWARD,STEPS_ADJUST_FORWARD,0);
	}
	//backup
	if(STEPS_ADJUST_BACKWARD > 0){
		Straight(BACKWARD,STEPS_ADJUST_BACKWARD,0);
	}

	//pump for some time
	P1DIR |= BIT_PUMP;
	P1OUT |=BIT_PUMP;
	__delay_cycles(WAIT_CYCLES);
	P1OUT &= ~BIT_PUMP;
}
