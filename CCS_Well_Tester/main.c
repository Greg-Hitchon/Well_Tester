
#include "Project_Parameters.h"
#include TEST_CHIP

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
	Create_Nav_Profile(0,1500,6000,2000,25,25,5,5);
	Create_Nav_Profile(1,1500,4000,2000,10,10,5,5);
	//Create_Nav_Profile(1,17000);

	//*******************************
	//set extract bit to input
	//P1DIR &= ~BIT_EXTRACT;
	//rising edge trigger
	//P1IES &= ~BIT_EXTRACT;
	//enable interrupt
	//P1IE |= BIT_EXTRACT;
	//*******************************

  //***************************
  //NAVIGATION ALGO HERE
  //***************************  
  //set motors to output

  for(;;){
    /*
    Set_Motor_Outputs();
    Print_String("\n\nStart Navigation...\r\n\n");
    Set_Timer();
    Straight(BACKWARD,1000U);
    Turn(LEFT);
    Turn(RIGHT);
    Straight(BACKWARD,1000U);
    Shutdown_Timer();
    Print_String("Done Navigation...\r\n\n");

    //Turn(LEFT,0,0);
    //__delay_cycles(10000000);
	//Print_String("\n\nStart Navigation...\r\n\n");
    */

    Straight(FORWARD,TABLE_LENGTH_STEPS,TABLE_LENGTH_STEPS,0,0);
    __delay_cycles(5000000);
	Turn(RIGHT,1,1);
    __delay_cycles(5000000);
    //Straight(FORWARD,15000UL,15000UL,0,0);
    //__delay_cycles(5000000);
    //Turn(RIGHT,0,0);
    //__delay_cycles(5000000);
    //Straight(FORWARD,15000UL,15000UL,0,0);
  }

  //***************************
  //SENSING ALGO HERE
  //***************************
  Print_String("Start Sensing...\r\n\n");
  
  //FREQ_VAL = Frequency_Read(BIT5, 1000);
  Print_String("Done Sensing...\r\n");

  //***************************
  //COMMUNICATION ALGO HERE
  //***************************
  
  Print_String("Result: Apple Juice");
  
  //***************************
  //Program end
  //***************************
  while(1);
}

#pragma vector=PORT1_VECTOR
__interrupt void PORT1_ISR(void){

if(P1IFG & BIT_EXTRACT){
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



