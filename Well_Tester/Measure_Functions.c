/*
 * Measure_Functions.c
 *
 *  Created on: Oct 20, 2013
 *      Author: Greg
 *
 *The goal of these functions is to provide an easy method of getting a consistent ADC
 *value or frequency reading from some input port
 *The ADC function allows for a continuous sample for some number of samples averaging the
 *return value over this interval.  
 *The Frequency function allows for some set number of target pulses to be recieved before using
 *the number of timerA cycles in that time period to find the relative frequency
 */

//system includes
#include "Project_Parameters.h"
#include "Nav_Functions.h"
#include "Bit_Definitions.h"
#include "Communication_Functions.h"
#include TEST_CHIP

#include "cstbool.h"

//macro definitions
#define TICK_FREQUENCY 			(CLOCK_FREQ/8)
#define TICK_RESOLUTION 		(10)
#define PULSE_PERIOD_TICKS 		(100000UL)
#define PULSE_DURATION_TICKS	(500)
#define CUP_FOUND_TICKS			(800)
#define MIN_LEFTOVER_TICKS		(30)


//function definitions
unsigned int cstlog2(unsigned int);
void Shutdown_Pulses(void);
void Initialize_Counter(void);
void Reset_Count(void);
void Shutdown_Counter(void);
unsigned long Get_Count(void);

//global variables
unsigned long gul_ADC_Total;
unsigned int gul_Tick_Count, gui_Channel, gui_ADC_Count, gui_ADC_Target, gui_Overflows_Remaining, gui_Overflow_Count, gui_Num_Leftover;
bool gub_Counter_Running = false, gub_Pulse_Start = false, gub_Pulse_Enabled = false, gub_Start_Count = false;
   

void Get_Result(void){
	__delay_cycles(16000000);
}

//Analog_Read performs much like the "Arduino" version with a 10-bit digital 
//value being returned that is scaled to 0-2.5V on a given port
   //NOTE:  internal reference does not seem to be perfect 2.5V so some adjustment is required for accurate performance, temperature dependent
unsigned int Analog_Read(unsigned int Channel, 
                    unsigned int Sample_Target){


//check valid channel             
if (Channel > BIT7){
  return 0;
}

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
return (unsigned int) (gul_ADC_Total/gui_ADC_Count);
}



//***********************************************************************************************************************************************
//This controls the pulse sending/receiving
//***********************************************************************************************************************************************

void Initialize_Pulses(void){
	//need to have timer running here
	if(!gub_Counter_Running){
		Initialize_Counter();
	}

	//set up interrupts for the echo pin
	P1DIR &= ~BIT_ECHO;
	//rising edge trigger
	P1IES &= ~BIT_ECHO;
	//enable interrupt
	P1IE |= BIT_ECHO;

	//get number of overflows
	gui_Overflow_Count = (unsigned int) (PULSE_PERIOD_TICKS / 0x10000UL);
	gui_Num_Leftover = (unsigned int) (PULSE_PERIOD_TICKS % 0x10000UL);

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
	TA0CTL = TASSEL_2 | MC_2 | TACLR | ID_3;

	//show running
	gub_Counter_Running = true;
}

void Shutdown_Counter(void){
	TA0CTL = 0;
	gub_Counter_Running = false;
}

//***********************************************************************************************************************************************


//log2 function
unsigned int cstlog2 (unsigned int val) {
    unsigned int ret = -1;
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

//this is used for the counter function
#pragma vector=TIMER0_A0_VECTOR
__interrupt void TIMER0_CCR0_ISR(void){
	static unsigned int Last_Time = 0;

	//check if done
	if(TA0CCR0 - Last_Time < CUP_FOUND_TICKS){
		Shutdown_Pulses();
		Shutdown_Counter();
		Cup_Found();
	}
	//save last time
	Last_Time = TA0CCR0;
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
			TA1CCR1 += gui_Num_Leftover;
			gub_Pulse_Start = true;
			gui_Overflows_Remaining = gui_Overflow_Count;
		}
	default: break;
	}
	//clear flag
	TA1CTL &= ~TAIFG;
}


