/*
 * Measure_Functions.c
 *
 *  Created on: Oct 20, 2013
 *      Author: Greg
 *
 * Below info Updated: February 24th 2014
 *
 * The goal of these functions is to provide an easy method of getting a consistent ADC
 * value or frequency reading from some input port
 * The ADC function allows for a continuous sample for some number of samples averaging the
 * return value over this interval.
 * The Frequency function allows for some set number of target pulses to be recieved before using
 * the number of timerA cycles in that time period to find the relative frequency
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
#include "Communication_Functions.h"
#include "cstbool.h"


//**********************************************************************************************************||
//Function Prototypes
//**********************************************************************************************************||
unsigned int Analog_Read(uint8_t Channel,
						uint16_t Sample_Target);
uint16_t cstlog2(unsigned int);
void Shutdown_Pulses(void);
void Initialize_Counter(void);
void Shutdown_Counter(void);

//**********************************************************************************************************||
//Compile time Constants
//**********************************************************************************************************||
//Ultrasonic Function
//this is the number of ticks in a period of the ultrasonic
#define PULSE_PERIOD_TICKS 		(200000UL)
//this is the high time of the ticks, corresponds to 10us
#define PULSE_DURATION_TICKS	(40)
//this is the value that the running average has to be less than for the ultrasonic to trigger a cup found
#define CUP_FOUND_TICKS			(20000)
//when keeping track of overflows case may be you have one overflow plus a few ticks that could be missed.  this bumps that up in order to catch all interrupts
#define MIN_LEFTOVER_TICKS		(30)
//this is the number of pulse durations to keep in the running sum array
#define NUM_PULSE_AVG			(10)

//Sensing Unit Function
#define NUM_LIGHT_TEST 			(10)
#define NUM_COND_TEST 			(10)
#define NUM_VARIABILITY 		(10)


//**********************************************************************************************************||
//Variables
//**********************************************************************************************************||
//Mem: ~20 Bytes
//**********************************************************************************************************||
uint32_t gul_ADC_Total;
uint16_t gul_Tick_Count, gui_ADC_Count, gui_ADC_Target, gui_Overflows_Remaining, gui_Overflow_Count, gui_Num_Leftover;
uint8_t gui_Channel;
bool gub_Counter_Running = false, gub_Pulse_Start = false;
   

//**********************************************************************************************************||
//Functions
//**********************************************************************************************************||
uint8_t Get_Result(uint16_t *Cond_Value,
					uint16_t *Light_Value,
					uint16_t *Var_Value,
					uint16_t Wait_Seconds){

	bool b_Is_Light = false, b_Is_Volatile = false;
	uint16_t i, ui_Last_Cond, ui_Current_Cond;

	//clear input used to determine variability
	*Var_Value = 0;
	*Cond_Value = 0;
	*Light_Value = 0;
	b_Is_Light = false;


	//delay for 30s, get variability
	for (i=0;i<Wait_Seconds;i++){
		//delay 1sec
			__delay_cycles(16000000);
		//update last value 1s before the test begins
			if(i==(NUM_VARIABILITY-2)){
				ui_Last_Cond = Analog_Read(INPUT_CONDUCTIVITY, 100);
			}
		//only get variability after ~10s
			if(i>(NUM_VARIABILITY-2)){
			//get conductivity measurement
				ui_Current_Cond = Analog_Read(INPUT_CONDUCTIVITY, 100);
			//save maximum diff between readings
				if(ui_Current_Cond > ui_Last_Cond){
					if(*Var_Value<(ui_Current_Cond-ui_Last_Cond)){
						*Var_Value += (ui_Current_Cond-ui_Last_Cond);
					}
				}
				else{
					if(*Var_Value < (ui_Last_Cond-ui_Current_Cond)){
						*Var_Value += (ui_Last_Cond-ui_Current_Cond);
					}
				}
			}
		}

	//test for volatility
		if(*Var_Value > VOLATILE_THRESHOLD){
			b_Is_Volatile = true;
		}

	//determine conductivity
		//read analog vals
			for(i=0;i<NUM_COND_TEST;i++){
				*Cond_Value += Analog_Read(INPUT_CONDUCTIVITY, 100);
			}

			*Cond_Value /= NUM_COND_TEST;

	//determine light dark
		//read analog vals
			for(i=0;i<NUM_LIGHT_TEST;i++){
				*Light_Value += Analog_Read(INPUT_LIGHT, 100);
			}

			*Light_Value /= NUM_LIGHT_TEST;

		//set bool based on avg value and threshold
			if(*Light_Value > LIGHT_THRESHOLD){
				b_Is_Light = true;
			}

	//perform logic
		//use determined values to get liquid type
		if((b_Is_Light==false) && (b_Is_Volatile) && (*Cond_Value > 450)){
			return LT_COKE;
		}
		else if(*Cond_Value < 500){
			if(b_Is_Light){
				return LT_WHITE_VINEGAR;
			}
			else{
				return LT_MALT_VINEGAR;
			}
		}
		else if(*Cond_Value < 780){
			if(b_Is_Light){
				return LT_SUGAR_WATER;
			}
			else{
				return LT_ORANGE_JUICE;
			}
		}
		else if(*Cond_Value < 994){
			return LT_DISTILLED_WATER;
		}
		else {
			return LT_VEGETABLE_OIL;
		}
}

//Analog_Read performs much like the "Arduino" version with a 10-bit digital 
//value being returned that is scaled to 0-2.5V on a given port
   //NOTE:  internal reference does not seem to be perfect 2.5V so some adjustment is required for accurate performance, temperature dependent
unsigned int Analog_Read(uint8_t Channel,
						uint16_t Sample_Target){

//disable interrupts
__disable_interrupt();

//set global vars
gui_ADC_Count = 0;
gui_ADC_Target = Sample_Target;
gul_ADC_Total = 0;

//set up adc    
//clear channel
ADC10CTL1 &= ~(0xF << 12);  

//set the channel (inch) bits (clear rest)
ADC10CTL1 |= (cstlog2(Channel) << 12);     

//configure input port
ADC10AE0 = Channel;

//on, interrupts enabled ref 0-2.5V 
//ADC10CTL0 = ADC10SHT_2 | ADC10ON | ADC10IE | REFON | SREF_1 | REF2_5V | MSC; // ADC10ON, interrupt enabled
ADC10CTL0 = ADC10SHT_2 | ADC10ON | ADC10IE | MSC; // ADC10ON, interrupt enabled

//let reference settle
__delay_cycles(100);

// Sampling and conversion start
ADC10CTL0 |= ENC | ADC10SC;  
    
//LPM0, wait for completion
 __bis_SR_register(CPUOFF + GIE); 

//clear configuration 
ADC10CTL0 = 0x0;
ADC10CTL1 &= ~(0xF << 12);  
ADC10AE0 = 0x0;

//return value
return UINT16_C(gul_ADC_Total/gui_ADC_Count);
}


//***********************************************************************************************************************************************
//This controls the pulse sending/receiving
//***********************************************************************************************************************************************

void Initialize_Pulses(void){
	//need to have timer running here
	if(!gub_Counter_Running){
		Initialize_Counter();
	}

	//get number of overflows
	gui_Overflow_Count = UINT16_C(PULSE_PERIOD_TICKS / 0x10000UL);
	gui_Num_Leftover = UINT16_C(PULSE_PERIOD_TICKS % 0x10000UL);

	//check and fill remaining overflows
	if(gui_Num_Leftover < MIN_LEFTOVER_TICKS){
		gui_Num_Leftover = MIN_LEFTOVER_TICKS;
	}

	gui_Overflows_Remaining = gui_Overflow_Count;

	//set the pulse width
	TA0CCR1 = gui_Num_Leftover;
	TA0CCTL1 |= CCIE;

	//initialize trigger bit
	P1DIR |= BIT_TRIGGER;
	P1OUT &= ~ BIT_TRIGGER;
}



void Shutdown_Pulses(void){
	//turn off pulse width
	TA0CCTL1 &= ~CCIE;
}
//***********************************************************************************************************************************************


//***********************************************************************************************************************************************
//This controls the timer used to track time
//***********************************************************************************************************************************************
void Initialize_Counter(void){
	//set up pins
	P1OUT &= ~BIT_ECHO;
	P1DIR &= ~BIT_ECHO;
	P1SEL |= BIT_ECHO;

	//set up the compare register
	TA0CCTL0 = CM_3 | CCIS_0 | SCS | CAP | CCIE;

	//set up timera (continuous mode, source: smclk, clear, interrupts enabled, divide by 8)
	//Divide by 8 here is necessary as if not used the taccr1 misses the increment
	//and the frequency is then 1/full clock cycle (65535)
	TA0CTL = TASSEL_2 | MC_2 | TACLR | ID_2;

	//show running
	gub_Counter_Running = true;
}

void Shutdown_Counter(void){
	TA0CTL = 0;
	gub_Counter_Running = false;
}

//***********************************************************************************************************************************************


//log2 function
uint16_t cstlog2 (unsigned int val) {
	uint16_t ret = 65535;
    while (val != 0) {
        val >>= 1;
        ret++;
    }
    return ret;
}

#pragma vector = ADC10_VECTOR
__interrupt void ADC_ISR(void){
  
  if (gui_ADC_Count < gui_ADC_Target)
  {
    //add current regester value to total adc
    gul_ADC_Total += ADC10MEM;
    
    //this keeps track of how many samples we have added
    gui_ADC_Count++;
    
    //start next sample
    ADC10CTL0 |= ENC | ADC10SC;  
  }
  else{
    // Clear CPUOFF bit from 0(SR)
    __bic_SR_register_on_exit(CPUOFF);
  }
}

//**********************************************************************************************************||
//Interrupts
//**********************************************************************************************************||

//this is used for the counter function
#pragma vector=TIMER0_A0_VECTOR
__interrupt void TIMER0_CCR0_ISR(void){
	static uint32_t Time_Sum = 0;
	static uint16_t Time_Track[NUM_PULSE_AVG] = {0}, Last_Time = 0, Tmp_Time;
	static uint8_t Time_Ind=0, Last_Ind;
	static bool Pulse_Start = true, Test_Pulse = false, Valid_Edge = true;
	uint16_t i=0;

		//actualy do store/test
		if(!Pulse_Start){
			//update the current value of the timer right away
			Tmp_Time = TA0CCR0;

			if(Valid_Edge){
				//toggle flab
				Valid_Edge = false;
				//very first thing we do is subtract the current value of the track array from the sum
				Last_Ind = Time_Ind;
				Time_Sum -= UINT32_C(Time_Track[Last_Ind]);

				Time_Track[Last_Ind] = Tmp_Time - Last_Time;
				Time_Sum += UINT32_C(Time_Track[Last_Ind]);


				//get new index
				if(++Time_Ind == NUM_PULSE_AVG){
					//reset the index
					Time_Ind = 0;
					//make sure that we only do a sum if this code has executed at least once
					if(!Test_Pulse){
						Test_Pulse = true;
					}
				}

				//here we update the sum
				if(Test_Pulse){
					//check if done
					if(Time_Sum < CUP_FOUND_TICKS){
						//temp
						/*
						for(i=0;i<NUM_PULSE_AVG; i++){
							Print_UINT((Time_Track[i]));
							Print_String("\r\n");
							__delay_cycles(500000);
						}
						*/
						//end temp

						Shutdown_Pulses();
						Shutdown_Counter();
						Cup_Found();
					}
				}
			}
			else{
				//toggle flag
				Valid_Edge = true;
			}

			//save the last time
			Last_Time = Tmp_Time;
		}
		else{
			//toggle flag
			Pulse_Start = false;
			//update the starting time
			Last_Time = TA0CCR0;
		}
}

//clear flag here
#pragma vector=TIMER0_A1_VECTOR
__interrupt void TIMER0_OTHER_ISR(void){
	//flags reset by reading ta0iv
	switch(__even_in_range(TA0IV,0xA)){
	//pulse period tracker
	case TA0IV_TACCR1:
		if (gub_Pulse_Start){
			if(gui_Overflows_Remaining == 0){
				P1OUT |= BIT_TRIGGER;
				TA0CCR1 += PULSE_DURATION_TICKS;
				gub_Pulse_Start = false;
			}
			else{
				gui_Overflows_Remaining--;
			}
		}
		else{
			P1OUT &= ~BIT_TRIGGER;
			TA0CCR1 += gui_Num_Leftover;
			gub_Pulse_Start = true;
			gui_Overflows_Remaining = gui_Overflow_Count;
		}
	default: break;
	}
	//clear flag
	TA0CTL &= ~TAIFG;
}


