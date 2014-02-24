
#include "Project_Parameters.h"
#include TEST_CHIP
#include <stdint.h>

#include "Nav_Functions.h"
#include "Bit_Definitions.h"
#include "Measure_Functions.h"
#include "Communication_Functions.h"
#include "cstbool.h"

/*
 * main.c
 *
 *This is where the program execution begins and ends.  It is broken up into 
 *the following sections: Setup, Navigation (interrupted with Extraction), Sensing, Communication
 *
 *The code is fairly procedural and should be easy to follow.
 *
 *The mapping of pinouts is as follows: 
 *PORT 1=1 TXD pin for UART, 2 Ports for Cup detection, 1 Port for Extraction, 2 For Sensing, 2 Undetermined
 *PORT 2=8 Motor outputs
 *
 *
 *TO DO LIST:
 *
 *
 *THINGS DEPENDENT ON CLOCK FREQUENCY:
 *-certain calculations may need to be adjusted (due to integer division/range issues) if clock freq is increased
 *
 *
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
	//main running profile
	Create_Nav_Profile(0,4000,7500,5500,10,10,1,1);
	//turn profile
	Create_Nav_Profile(1,4000,6000,4000,10,10,1,1);
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
