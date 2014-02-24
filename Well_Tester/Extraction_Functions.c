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
#define STEPS_ADJUST_AIR		(300)
#define STEPS_ADJUST_FORWARD 	(50)
#define STEPS_ADJUST_BACKWARD 	(300)
#define LOOP_DELAY				(100)
#define WET_PUMP_CYCLES			(10*16000000)
#define AIR_PUMP_CYCLES			(10*16000000)
#define PAUSE_CYCLES			(5*16000000)

//function declarations
void Save_And_Pump(uint32_t Wait_Cycles);
void Delay_For(uint32_t Wait_Cycles);


void Extract_Liquid(void){
	//dependent on placement has to move forward more
	if(STEPS_ADJUST_FORWARD > 0){
		Straight(FORWARD,STEPS_ADJUST_FORWARD,0);
	}
	//backup
	if(STEPS_ADJUST_BACKWARD > 0){
		Straight(BACKWARD,STEPS_ADJUST_BACKWARD,0);
	}


	//pump out liquid
	Save_And_Pump(WET_PUMP_CYCLES);
	//prepare for air pump
	Straight(FORWARD,STEPS_ADJUST_AIR,0);
	//pump out liquid
	Save_And_Pump(AIR_PUMP_CYCLES);
	//pause to look after some drips
	__delay_cycles(PAUSE_CYCLES);
}


void Save_And_Pump(uint32_t Wait_Cycles){
	uint16_t ui16_Save_Motors;

	//make sure no current is going through motors
	ui16_Save_Motors = P2OUT;
	P2OUT = 0;

	//turn on pump
	P1DIR |= BIT_PUMP;
	P1OUT |=BIT_PUMP;

	//run pump for set time
	Delay_For(Wait_Cycles);

	//Turn off pump
	P1OUT &= ~BIT_PUMP;
	P1DIR &= ~BIT_PUMP;

	//reset motors
	P2OUT = ui16_Save_Motors;
}

void Delay_For(uint32_t Wait_Cycles){
	uint32_t Num_Loops;
	//do loop
	for(Num_Loops = Wait_Cycles/LOOP_DELAY; Num_Loops>0;Num_Loops--){
		__delay_cycles(LOOP_DELAY);
	}
}

