/*
 * Extraction_Functions.c
 *
 *  Created on: Oct 20, 2013
 *      Author: Greg
 *
 * Below info Updated: February 24th 2014
 *
 * The point of this file is to encapsulate all functions related to the actual extraction of the liquid
 */

//**********************************************************************************************************||
//Syestem Headers
//**********************************************************************************************************||
#include "Project_Parameters.h"
#include TEST_CHIP
#include <stdint.h>


//**********************************************************************************************************||
//User Defined Headers
//**********************************************************************************************************||
#include "Nav_Functions.h"
#include "Bit_Definitions.h"


//**********************************************************************************************************||
//Function Prototypes
//**********************************************************************************************************||
void Save_And_Pump(uint32_t Wait_Cycles);
void Delay_For(uint32_t Wait_Cycles);


//**********************************************************************************************************||
//Compile time Constants
//**********************************************************************************************************||
//this is the number of steps to move forward after pumping before pumping air
#define STEPS_ADJUST_AIR		(200)
//this defines the number of steps to move forward to make sure the tube is out of the cup
#define STEPS_NO_TIP_ADJUST		(250)
//moves this number of steps forward after the cup is found
#define STEPS_ADJUST_FORWARD 	(0)
//this is the number of steps the robot moves backwards after cup is found
#define STEPS_ADJUST_BACKWARD 	(250)
//used in the delay loop
#define LOOP_DELAY				(100)
//number of seconds to pump the liquid
#define WET_PUMP_CYCLES			(5*16000000)
//number of seconds to pump the air
#define AIR_PUMP_CYCLES			(15*16000000)
//number of seconds to wait before moving
#define PAUSE_CYCLES			(5*16000000)


//**********************************************************************************************************||
//Functions
//**********************************************************************************************************||

//this is the function called once the cup has been found
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
	//pull hose out of cup
	Straight(FORWARD,STEPS_NO_TIP_ADJUST,0);
	//pback up to original position
	Straight(BACKWARD,STEPS_NO_TIP_ADJUST,0);

}

//saves motor state, turns off motors, pumps, turns on motors
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

//workaround for delays only allowing compile time constants
void Delay_For(uint32_t Wait_Cycles){
	uint32_t Num_Loops;
	//do loop
	for(Num_Loops = Wait_Cycles/LOOP_DELAY; Num_Loops>0;Num_Loops--){
		__delay_cycles(LOOP_DELAY);
	}
}

