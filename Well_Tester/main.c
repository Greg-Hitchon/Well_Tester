
#include "Project_Parameters.h"
#include TEST_CHIP

#include "Nav_Functions.h"
#include "Bit_Definitions.h"
#include "Measure_Functions.h"
#include "Communication_Functions.h"
#include "cstbool.h"

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
 */
#define NUM_TEST 15

unsigned long FREQ_VAL[NUM_TEST], ADC_VAL[NUM_TEST];
bool ub_Extract_Ready = false;

void main(void) {

  //***************************
  //CONFIGURATION NONSENSE HERE
  //***************************
  WDTCTL = WDTPW | WDTHOLD;	// Stop watchdog timer
  
  //set to 1MHZ
  BCSCTL1 = CALBC1_16MHZ;
  DCOCTL = CALDCO_16MHZ;
  

  //outputs clock frequency to p1.4
  /*
  P1DIR = BIT4;
  P1SEL = BIT4;

  while(1){}
  */
  
  

  /*
  //test for  number to string conversion
  for (;;){
    Print_UINT(1000UL);
    __delay_cycles(5000000);
  }
  */
  
  /*
  //test for frequency values here
  //int i = 0;
  for(;;){
    //if(i>(NUM_TEST-1)){
    //  i = 0;
    //}
   //FREQ_VAL[i++] = Frequency_Read(BIT0, 1000);   
   Print_ULONG(Frequency_Read(BIT0,20000000,20000000,true));
   __delay_cycles(500000);
   Print_String("\r\n");
   __delay_cycles(500000);
 //_BIS_SR(LPM4_bits + GIE);
  }
  */
  
 /*
  //test for ADC values here
 int i = 0;
  for(;;){
    //if(i>(NUM_TEST-1)){
      //i = 0;
    //}
   //ADC_VAL[i++] = Analog_Read(BIT1, 1000);   
   Print_UINT(Analog_Read(BIT1, 100));
   __delay_cycles(5000000);
   Print_String("\r\n");
   __delay_cycles(5000000);
  }
  */
  
  /*
  //test for printing here
  for (;;){  
    Print_String("\n\n1226!!\r\n\n");
    __delay_cycles(10000000);
    Print_String("\n\nHello Greg!!\r\n\n");
    __delay_cycles(10000000);
    Print_String("\n\nHello World!!\r\n\n");
    __delay_cycles(10000000);
  }
  */
	//enable test led
	P1DIR |= LED_RED | LED_GREEN;
	//signal ready
	P1OUT = LED_RED;

	//*******************************
	//configure startup bit for input
	P1DIR &= ~BIT_STARTUP;
	//rising edge trigger
	P1IES &= ~BIT_STARTUP;
	//enable interrupt
	P1IE |= BIT_STARTUP;
	//*******************************


	//Print_String("\n\nWait for start...\r\n\n");
  
	//turn off cpu
	__bis_SR_register(CPUOFF + GIE);

	//turn off startup interrupt and configure as output
	P1IE &= ~BIT_STARTUP;
	P1DIR |=BIT_STARTUP;

	//indicate active
	P1OUT |= LED_GREEN;

	//set up navigation profiles
	Create_Nav_Profile(0,3250,4500,3250,10,10,1,1);
	//Create_Nav_Profile(1,3100,3100,3100,10,10,1,1);
	//Create_Nav_Profile(0,1200,4000,2000,10,10,1,1);
	//Create_Nav_Profile(1,1200,2000,2000,10,10,1,1);

	//*******************************
	//set extract bit to input
	P1DIR &= ~BIT_EXTRACT;
	//falling edge trigger
	P1IES |= BIT_EXTRACT;
	//enable interrupt
	P1IE |= BIT_EXTRACT;
	//set global flag
	ub_Extract_Ready = true;
	//*******************************

  //***************************
  //NAVIGATION ALGO HERE
  //***************************  
	//Line();
	//Square();
	Final_Run();
  
  //***************************
  //Program end
  //***************************
  while(1);
}


void Line()
{
	for(;;){
		//Stright from start then turn left
		Straight(FORWARD,TABLE_LENGTH_STEPS,TABLE_LENGTH_STEPS,0,0);
		__delay_cycles(5000000);
	}
}


void Square()
{
	for(;;){
		//Stright from start then turn left
		Straight(FORWARD,TABLE_LENGTH_STEPS,TABLE_LENGTH_STEPS,0,0);
		__delay_cycles(5000000);
		Turn(RIGHT,0,0,DIME);
		__delay_cycles(5000000);
	}
}


void Final_Run()
{
	unsigned int i_Left_Profile = 0, i_Right_Profile = 0;

		  //Stright from start then turn left
	    Straight(FORWARD,TABLE_LENGTH_STEPS,TABLE_LENGTH_STEPS,i_Left_Profile,i_Right_Profile);
	    __delay_cycles(5000000);
		Turn(LEFT,i_Left_Profile,i_Right_Profile,SWEEP);
		__delay_cycles(5000000);

		//straight across top of table then turn left
	    Straight(FORWARD,TABLE_WIDTH_STEPS,TABLE_WIDTH_STEPS,i_Left_Profile,i_Right_Profile);
	    __delay_cycles(5000000);
	    Turn(LEFT,i_Left_Profile,i_Right_Profile,SWEEP);
	    __delay_cycles(5000000);

	    //Down left hand side then turn left
	    Straight(FORWARD,GRID_STEPS,GRID_STEPS,i_Left_Profile,i_Right_Profile);
	    __delay_cycles(5000000);
	    Turn(LEFT,i_Left_Profile,i_Right_Profile,DIME);
	    __delay_cycles(5000000);

	    //straight back across table then turn right
	    Straight(FORWARD,TABLE_WIDTH_STEPS,TABLE_WIDTH_STEPS,i_Left_Profile,i_Right_Profile);
	    __delay_cycles(5000000);
	    Turn(RIGHT,i_Left_Profile,i_Right_Profile,SWEEP);
		__delay_cycles(5000000);

		//down right hand side then turn right
		Straight(FORWARD,GRID_STEPS,GRID_STEPS,i_Left_Profile,i_Right_Profile);
		__delay_cycles(5000000);
		Turn(RIGHT,i_Left_Profile,i_Right_Profile, DIME);
		__delay_cycles(5000000);

		//straight across table to left then turn left
		Straight(FORWARD,TABLE_WIDTH_STEPS,TABLE_WIDTH_STEPS,i_Left_Profile,i_Right_Profile);
		__delay_cycles(5000000);
		Turn(LEFT,i_Left_Profile,i_Right_Profile, SWEEP);
		__delay_cycles(5000000);

		//add in two more crosses

		//Down left hand side then turn left
		Straight(FORWARD,GRID_STEPS,GRID_STEPS,i_Left_Profile,i_Right_Profile);
		__delay_cycles(5000000);
		Turn(LEFT,i_Left_Profile,i_Right_Profile, DIME);
		__delay_cycles(5000000);

		//straight back across table then turn right
		Straight(FORWARD,TABLE_WIDTH_STEPS,TABLE_WIDTH_STEPS,i_Left_Profile,i_Right_Profile);
		__delay_cycles(5000000);
		Turn(RIGHT,i_Left_Profile,i_Right_Profile, SWEEP);
		__delay_cycles(5000000);

		//down right hand side then turn right
		Straight(FORWARD,GRID_STEPS,GRID_STEPS,i_Left_Profile,i_Right_Profile);
		__delay_cycles(5000000);
		Turn(RIGHT,i_Left_Profile,i_Right_Profile, DIME);
		__delay_cycles(5000000);

		//straight across table to left then turn left
		Straight(FORWARD,TABLE_WIDTH_STEPS,TABLE_WIDTH_STEPS,i_Left_Profile,i_Right_Profile);
		__delay_cycles(5000000);
		Turn(LEFT,i_Left_Profile,i_Right_Profile, SWEEP);
		__delay_cycles(5000000);

		Straight(FORWARD,TABLE_LENGTH_STEPS-GRID_STEPS*2,TABLE_LENGTH_STEPS-GRID_STEPS*2,i_Left_Profile,i_Right_Profile);
		__delay_cycles(5000000);
}

#pragma vector=PORT1_VECTOR
__interrupt void PORT1_ISR(void){

if((P1IFG & BIT_EXTRACT) && ub_Extract_Ready){
	//enter infinite loop
	P1OUT |= LED_RED;
	P1OUT &= ~LED_GREEN;
	for(;;){
		P1OUT ^= LED_RED | LED_GREEN;
	__delay_cycles(8000000);
	}

	/*
	//toggle led's
	P1OUT |= LED_RED;
	P1OUT &= ~LED_GREEN;

	//have to set up interupt conditions here
	P1IE &= ~BIT_EXTRACT;
	Stop_Motor(BOTH_MOTORS,true);

	//***************************
	//EXTRACTION ALGO HERE
	//***************************
	__enable_interrupt();
	//Print_String("extracting...");
	Straight(BACKWARD,4000UL,2000UL,0,0);
	__delay_cycles(3000000);
	//Print_String("done extracting");

	//restore where we left off
	Restore_State(BOTH_MOTORS);
	Start_Motor(BOTH_MOTORS);


	P1OUT |= LED_GREEN;

	P1OUT &= ~LED_RED;

	//reset the interrupts
	P1IE |= BIT_EXTRACT;

	*/

  }
  else if(P1IFG & BIT_STARTUP){
	  //Print_String("\n\nStarting Program Execution...\r\n\n");
	  P1OUT &= ~LED_RED;
	  __delay_cycles(STARTUP_DELAY_TICKS);
	  __bic_SR_register_on_exit(CPUOFF);
  }
  
  //clear fgs
  P1IFG = 0x0;
}



