
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
 *
 *configure for input (high is test, low is run)
 */
#define NUM_LIGHT_TEST 10
#define NUM_COND_TEST 10
#define NUM_VARIABILITY 10

bool ub_Extract_Ready = false;

void main(void) {
	//vars for main
	bool b_Is_Light = false, b_Is_Volatile = false;
	unsigned int ui_Run_Mode, Avg_Cond_Value, Avg_Light_Value, i, ui_Last_Cond, ui_Current_Cond, ui_Max_Diff = 0;

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
	//Determine if test or run
	//***************************
	//configure for input (high is test, low is run)
	P2DIR &= ~INPUT_RUN_TYPE;
	//delay to let settle
	__delay_cycles(4000000);
	//test pin
	if(P2IN & INPUT_RUN_TYPE){
		ui_Run_Mode = TEST_MODE;
	}
	else{
		ui_Run_Mode = RUN_MODE;
	}
	//____________________________


	//***************************
	//Either run TEST MODE or RUN MODE
	//***************************
	if(ui_Run_Mode == TEST_MODE){
		//display test mode
			for(i=0;i<5;i++){
				P1DIR |= LED_GREEN;
				P1OUT ^= LED_GREEN;
				__delay_cycles(4000000);
			}
		//add in a few lines to differentiate
			Print_String("\r\n\r\n");
		//delay for 30s
			for (i=0;i<30;i++){
				//print countdown
					Print_UINT(30-i);
					__delay_cycles(11000000);
					Print_String("\r\n");
					__delay_cycles(5000000);

				//update last value 1s before the test begins
					if(i==(NUM_VARIABILITY-2)){
						ui_Last_Cond = Analog_Read(INPUT_CONDUCTIVITY, 100);
					}
				//only get variability after ~10s
					if(i>(NUM_VARIABILITY-2)){
					//get conductivity measurement
						ui_Current_Cond = Analog_Read(INPUT_CONDUCTIVITY, 100);
					//save maximum diff between readings
						if(ui_Current_Cond>ui_Last_Cond){
							if(ui_Max_Diff<(ui_Current_Cond-ui_Last_Cond)){
								ui_Max_Diff+=(ui_Current_Cond-ui_Last_Cond);
							}
						}
						else{
							if(ui_Max_Diff<(ui_Last_Cond-ui_Current_Cond)){
								ui_Max_Diff+=(ui_Last_Cond-ui_Current_Cond);
							}
						}
					}
			}

		//test for volatility
			if(ui_Max_Diff>VOLATILE_THRESHOLD){
				b_Is_Volatile = true;
			}

		for(;;){
		//reset vars
			Avg_Cond_Value = 0;
			Avg_Light_Value = 0;
			b_Is_Light = false;

		//determine conductivity
			//read analog vals
				for(i=0;i<NUM_COND_TEST;i++){
					Avg_Cond_Value+=Analog_Read(INPUT_CONDUCTIVITY, 100);
				}

				Avg_Cond_Value /= NUM_COND_TEST;

		//determine light dark
			//read analog vals
				for(i=0;i<NUM_LIGHT_TEST;i++){
					Avg_Light_Value+=Analog_Read(INPUT_LIGHT, 100);
				}

				Avg_Light_Value /= NUM_LIGHT_TEST;

			//set bool based on avg value and threshold
				if(Avg_Light_Value > LIGHT_THRESHOLD){
					b_Is_Light = true;
				}

		//perform logic
			//print actual readings
			Print_String("Actual Light Reading: ");
			__delay_cycles(5000000);
			Print_UINT(Avg_Light_Value);
			__delay_cycles(5000000);
			Print_String("\r\n");
			__delay_cycles(5000000);

			Print_String("Actual Cond. Reading: ");
			__delay_cycles(5000000);
			Print_UINT(Avg_Cond_Value);
			__delay_cycles(5000000);
			Print_String("\r\n");
			__delay_cycles(5000000);

			Print_String("Actual Volatility Reading: ");
			__delay_cycles(5000000);
			Print_UINT(ui_Max_Diff);
			__delay_cycles(5000000);
			Print_String("\r\n");
			__delay_cycles(5000000);

			//print choice
			if((b_Is_Light==false) && (b_Is_Volatile) && (Avg_Cond_Value > 450)){
				Print_String("Coke");
			}
			else if(Avg_Cond_Value<500){
				if(b_Is_Light){
					Print_String("White Vinegar");
				}
				else{
					Print_String("Malt Vinegar");
				}
			}
			else if(Avg_Cond_Value<780){
				if(b_Is_Light){
					Print_String("Sugar Water");
				}
				else{
					Print_String("Orange Juice");
				}
			}
			else if(Avg_Cond_Value<994){
				Print_String("Distilled Water");
			}
			else {
				Print_String("Vegetable Oil");
			}

			//print next line
			Print_String("\r\n\r\n");
			__delay_cycles(5000000);
		}
	}
	else{
		//display run mode
		for(i=0;i<5;i++){
			P1DIR |= LED_RED;
			P1OUT ^= LED_RED;
			__delay_cycles(4000000);
		}

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

		//turn off cpu
		__bis_SR_register(CPUOFF + GIE);

		//turn off startup interrupt and configure as output
		P1IE &= ~BIT_STARTUP;
		P1DIR |=BIT_STARTUP;

		//indicate active
		P1OUT |= LED_GREEN;

		//set up navigation profiles
		Create_Nav_Profile(0,3250,4500,3250,10,10,1,1);

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
		Line();
		//Square();
		//Final_Run();
	}
	//____________________________
  
	//***************************
	//Program end
	//***************************
	while(1);
}


void Line()
{
	int i_Spd = 3300;

	for(;;){
		//Stright from start then turn left
		Create_Nav_Profile(0,i_Spd,i_Spd,i_Spd,10,10,1,1);
		Straight(FORWARD,TABLE_LENGTH_STEPS,TABLE_LENGTH_STEPS,0,0);
		__delay_cycles(5000000);
		//i_Spd +=500;
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



