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
#define STEPS_ADJUST_FORWARD 	(50)
#define STEPS_ADJUST_BACKWARD 	(300)
#define WAIT_CYCLES				(15*16000000)

void Extract_Liquid(void){
	uint16_t ui16_Save_Motors;

	//dependent on placement has to move forward more
	if(STEPS_ADJUST_FORWARD > 0){
		Straight(FORWARD,STEPS_ADJUST_FORWARD,0);
	}
	//backup
	if(STEPS_ADJUST_BACKWARD > 0){
		Straight(BACKWARD,STEPS_ADJUST_BACKWARD,0);
	}

	//make sure no current is going through motors
	ui16_Save_Motors = P2OUT;
	P2OUT = 0;

	//turn on pump
	P1DIR |= BIT_PUMP;
	P1OUT |=BIT_PUMP;

	//run pump for set time
	__delay_cycles(WAIT_CYCLES);

	//Turn off pump
	P1OUT &= ~BIT_PUMP;
	P1DIR &= ~BIT_PUMP;

	//reset motors
	P2OUT = ui16_Save_Motors;


}
