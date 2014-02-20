
#include "Project_Parameters.h"
#include TEST_CHIP
#include <stdint.h>

#include "Nav_Functions.h"
#include "Bit_Definitions.h"
#include "Measure_Functions.h"
#include "Communication_Functions.h"
#include "cstbool.h"

//defines
#define DELAY_BETWEEN (5000)

//function prototypes
void Final_Run();
void Square();
void Line();



/*
 * main.c
 *
 *This is where the program execution begins and ends.  It is broken up into 
 *the following sections: Setup, Navigation (interrupted with Extraction), Sensing, Communication
 *
 *The code is fairly procedural and should be easy to follow.
 *
 *The mapping of pinouts is as follows: 
 *PORT 1=1 TXD pin for UART, 1 Interrupt for Extraction, 6 General I/O available for ADC
 *PORT 2=4 Motor outputs, 4 General I/O available for frequency conversion
 *
 *So after the navigation and communication needs are satisfied we have 10 pins left, 6 of which 
 *can be used with ADC (multiple LED inputs for instance) and 4 for frequency (converting physical property to oscillator)
 *
 *TO DO LIST:
 *-clarify set up/execute for ADC, FREQ, NAVIGATION, COMMUNICATION
 *-make printing numbers one function, with parameter for length
 *
 *THINGS DEPENDENT ON CLOCK FREQUENCY:
 *-certain calculations may need to be adjusted (due to integer division/range issues) if clock freq is increased
 *
 *
 *configure for input (high is test, low is run)
 */


void main(void) {
	//***************************
	//CONFIGURATION NONSENSE HERE
	//***************************
	WDTCTL = WDTPW | WDTHOLD;	// Stop watchdog timer

	//set to 1MHZ
	BCSCTL1 = CALBC1_16MHZ;
	DCOCTL = CALDCO_16MHZ;

	//delay .25s for settle
	__delay_cycles(4000000);
	//____________________________

	//***************************
	//NAVIGATION ALGO HERE
	//***************************
	//make sure interrupts are enabled
	__enable_interrupt();

	//start the ultrasonic sensing
	Initialize_Pulses();
	//initialize startup bit configuration
	Initialize_Bits();
	//initialize the tracking direction + distances
	Initialize_Tracking();
	//create all necessary profiles for navigation purposes
	Create_Nav_Profile(0,4000,7500,4000,10,10,1,1);
	Create_Nav_Profile(1,4000,6000,4000,10,10,1,1);
	//execute algorithm
	Final_Run();
	//____________________________
  
	//***************************
	//Program end, enter infinite loop
	//***************************
	while(1);
}

void Line()
{
	uint16_t i_Spd = 1500;

	for(;;){
		//Stright from start then turn left
		Create_Nav_Profile(0,i_Spd,i_Spd,i_Spd,10,10,1,1);
		Straight(FORWARD,TABLE_LENGTH_STEPS,0);
		__delay_cycles(5000000);
		//i_Spd +=500;
	}
}


void Square()
{
	for(;;){
		//Stright from start then turn left
		Straight(FORWARD,TABLE_LENGTH_STEPS,0);
		__delay_cycles(5000000);
		Turn(RIGHT,0,SWEEP,STEPS_PER_SWEEP);
		__delay_cycles(5000000);
	}
}


void Final_Run()
{
	const uint8_t i_Turn_Profile = 1, i_Straight_Profile = 0;

		  //Stright from start then turn left
	    Straight(FORWARD,TABLE_LENGTH_STEPS,i_Straight_Profile);
	    __delay_cycles(DELAY_BETWEEN);
		Turn(LEFT,i_Turn_Profile,SWEEP,STEPS_PER_SWEEP);
		__delay_cycles(DELAY_BETWEEN);

		//straight across top of table then turn left
	    Straight(FORWARD,TABLE_WIDTH_STEPS,i_Straight_Profile);
	    __delay_cycles(DELAY_BETWEEN);
	    Turn(LEFT,i_Turn_Profile,SWEEP,STEPS_PER_SWEEP);
	    __delay_cycles(DELAY_BETWEEN);

	    //Down left hand side then turn left
	    Straight(FORWARD,GRID_STEPS,i_Straight_Profile);
	    __delay_cycles(DELAY_BETWEEN);
	    Turn(LEFT,i_Turn_Profile,DIME,STEPS_PER_DIME);
	    __delay_cycles(DELAY_BETWEEN);

	    //straight back across table then turn right
	    Straight(FORWARD,TABLE_WIDTH_STEPS,i_Straight_Profile);
	    __delay_cycles(DELAY_BETWEEN);
	    Turn(RIGHT,i_Turn_Profile,SWEEP,STEPS_PER_SWEEP);
		__delay_cycles(DELAY_BETWEEN);

		//down right hand side then turn right
		Straight(FORWARD,GRID_STEPS,i_Straight_Profile);
		__delay_cycles(DELAY_BETWEEN);
		Turn(RIGHT,i_Turn_Profile, DIME,STEPS_PER_DIME);
		__delay_cycles(DELAY_BETWEEN);

		//straight across table to left then turn left
		Straight(FORWARD,TABLE_WIDTH_STEPS,i_Straight_Profile);
		__delay_cycles(DELAY_BETWEEN);
		Turn(LEFT,i_Turn_Profile, SWEEP,STEPS_PER_SWEEP);
		__delay_cycles(DELAY_BETWEEN);

		//add in two more crosses

		//Down left hand side then turn left
		Straight(FORWARD,GRID_STEPS,i_Straight_Profile);
		__delay_cycles(DELAY_BETWEEN);
		Turn(LEFT,i_Turn_Profile, DIME,STEPS_PER_DIME);
		__delay_cycles(DELAY_BETWEEN);

		//straight back across table then turn right
		Straight(FORWARD,TABLE_WIDTH_STEPS,i_Straight_Profile);
		__delay_cycles(DELAY_BETWEEN);
		Turn(RIGHT,i_Turn_Profile, SWEEP,STEPS_PER_SWEEP);
		__delay_cycles(DELAY_BETWEEN);

		//down right hand side then turn right
		Straight(FORWARD,GRID_STEPS,i_Straight_Profile);
		__delay_cycles(DELAY_BETWEEN);
		Turn(RIGHT,i_Turn_Profile, DIME,STEPS_PER_DIME);
		__delay_cycles(DELAY_BETWEEN);

		//straight across table to left then turn left
		Straight(FORWARD,TABLE_WIDTH_STEPS,i_Straight_Profile);
		__delay_cycles(DELAY_BETWEEN);
		Turn(LEFT,i_Turn_Profile, SWEEP,STEPS_PER_SWEEP);
		__delay_cycles(DELAY_BETWEEN);

		Straight(FORWARD,TABLE_LENGTH_STEPS-GRID_STEPS*2,i_Straight_Profile);
		__delay_cycles(DELAY_BETWEEN);
}
