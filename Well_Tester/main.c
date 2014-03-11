/*
 * main.c
 *
 * Created on: Oct 16, 2013
 *   Author: Greg
 *
 * Below info Updated: February 24th 2014
 *
 * This is where the program execution begins and ends.  It is broken up into
 * the following files: Navigation, Sensing (cup+liquid), Communication, Extraction,
 * Bit definitions, and Project Parameters
 *
 * The basic flow of the code is to execute a set grid search pattern and wait for an interrupt from the
 * cup sensor.  Once a full cup is detected the liquid is extracted and then the robot returns to the sensing area.
 *
 * The current bit definitions can be seen in the "Bit_Definitions.h" file.
 *
 * Navigation: The actual motor movement is accomplished by outputting pulses to each motor phase as determined by a state array
 *
 * Sensing: Full cups are found by time-averaging an ultrasonic signal and triggering an interrupt at a certain threshold value
 *
 * TO DO LIST:
 * -Finalize sensing
 * -test new navigation algo
 *
 *
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
#include "Measure_Functions.h"
#include "Communication_Functions.h"
#include "cstbool.h"

//**********************************************************************************************************||
//Functions
//**********************************************************************************************************||
//This function section merely contains the main function that sets up different functions then executes
//the main navigation algorithm
//**********************************************************************************************************||

void main(void) {
	//***************************
	//CONFIGURATION NONSENSE HERE
	//***************************
	WDTCTL = WDTPW | WDTHOLD;	// Stop watchdog timer

	//set to 1MHZ
	BCSCTL1 = CALBC1_16MHZ;
	DCOCTL = CALDCO_16MHZ;

	//settle time
	__delay_cycles(32000000);

	//temp
	/*
	//turn off motors
	  	uint16_t Cond_Value, Light_Value, Var_Value;
	 	uint8_t Liquid_Type;
		P2OUT = 0;
		P2SEL = 0;
		P2DIR = 0;
		__enable_interrupt();

		//print to computer
		for(;;){
			//get result from sensing unit
			Liquid_Type = Get_Result(&Cond_Value, &Light_Value, &Var_Value,5);
			__delay_cycles(4000000);
			Output_Result(&Liquid_Type,&Cond_Value,&Light_Value,&Var_Value);
		}
	 */
	//end temp

	//***************************
	//NAVIGATION ALGO HERE
	//***************************
	//make sure interrupts are enabled
	__enable_interrupt();

	//temp
	/*
	//make sure motors are off
	Disable_Motors();
	//start the ultrasonic sensing
	//Initialize_Pulses();
	P1DIR = BIT_PUMP;
	P1OUT = BIT_PUMP;
	for(;;){}
	__bis_SR_register(CPUOFF + GIE);
	*/
	//end temp

	//configure all of port 1 for input (does not do anything of note)
	P1DIR &= ~0xFF;

	//initialize startup bit configuration
	Initialize_Bits();
	//start the ultrasonic sensing
	Initialize_Pulses();
	//initialize the tracking direction + distances
	Initialize_Tracking();
	//create all necessary profiles for navigation purposes
	//main running profile
	Create_Nav_Profile(0,4000,7500,5500,10,10,1,1);
	//turn profile
	Create_Nav_Profile(1,3500,3500,3500,10,10,1,1);
	//adjust profile
	Create_Nav_Profile(2,3000,3000,3000,10,10,1,1);
	//execute algorithm
	Final_Run();
	//____________________________
  
	//***************************
	//Program end, enter infinite loop
	//***************************
	while(1);
}
