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
#include TEST_CHIP

#include "cstbool.h"

//macro definitions
#define TICK_FREQUENCY 			(CLOCK_FREQ/8)
#define TICK_RESOLUTION 		(50)
#define PULSE_PERIOD_TICKS 		(0xFFFF)
#define PULSE_DURATION_TICKS	(20)
#define CUP_FOUND_TICKS			(1000000UL)


//function definitions
unsigned int cstlog2(unsigned int);
void Discharge_Cap(unsigned int);

//global variables
unsigned long gul_Tick_Count, gul_ADC_Total;
unsigned int gui_Channel, gui_ADC_Count, gui_ADC_Target;
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


	//set the pulse width
	TA1CCR1 = PULSE_PERIOD_TICKS;
	TA1CCTL1 |= CCIE;

	//initialize trigger bit
	P1DIR |= BIT_TRIGGER;
	P1OUT &= ~ BIT_TRIGGER;

	//this toggles between waiting for a pulse length and a full wait
	gub_Pulse_Start = true;
	gub_Start_Count = true;
	gub_Pulse_Enabled = true;
}

bool Get_Pulse_Status(void){
	return gub_Pulse_Enabled;
}

bool Update_Pulse_Tracker(void){
	if(gub_Start_Count){
		//reset counter
		Reset_Count();
		//configure for falling edge
		P1IES |= BIT_ECHO;
		//toggle flag
		gub_Start_Count = false;
	}
	else{
		if(Get_Count() < CUP_FOUND_TICKS){
			Shutdown_Pulses();
			Shutdown_Counter();
			return true;
		}
		else{
			//rising edge trigger
			P1IES &= ~BIT_ECHO;
			gub_Start_Count = true;
		}
	}
}

void Shutdown_Pulses(void){
	//turn off bool
	gub_Pulse_Enabled = false;

	//turn off interrupts at echo pin
	P1IE &= ~BIT_ECHO;

	//turn off pulse width
	TA1CCTL1 &= ~CCIE;
}
//***********************************************************************************************************************************************


//***********************************************************************************************************************************************
//This controls the timer used to track time
//***********************************************************************************************************************************************
void Initialize_Counter(void){
	//set up timera (continuous mode, source: smclk, clear, interrupts enabled, divide by 8)
	//Divide by 8 here is necessary as if not used the taccr1 misses the increment
	//and the frequency is then 1/full clock cycle (65535)
	TA1CTL = TASSEL_2 | MC_1 | TAIE | TACLR | ID_3;

	//set main counter to 0
	gul_Tick_Count = 0;

	//set up inturrupt period
	TA1CCR0 = TICK_RESOLUTION;
	TA1CCTL0 |= CCIE;

	//indicate counter is running
	gub_Counter_Running = true;
}

void Shutdown_Counter(void){
	//disable interrupts
	TA1CTL &= ~TAIE;

	//indicate counter no longer running
	gub_Counter_Running = false;
}

void Reset_Count(void){
	gul_Tick_Count = 0;
}

unsigned long Get_Count(void){
	return gul_Tick_Count;
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
#pragma vector=TIMER1_A0_VECTOR
__interrupt void TIMER1_CCR0_ISR(void){
	//increment tick count by 10 to increase performacne (slightly lower resolution
	 gul_Tick_Count += (TICK_RESOLUTION + 1);
}

//clear flag here
#pragma vector=TIMER1_A1_VECTOR
__interrupt void TIMER1_OTHER_ISR(void){
	//flags reset by reading ta0iv
	switch(__even_in_range(TA1IV,0xA)){
	//pulse period tracker
	case TA1IV_TACCR1:
		if (gub_Pulse_Start){
			P1OUT |= BIT_TRIGGER;
			TA1CCR1 += PULSE_DURATION_TICKS;
			gub_Pulse_Start = false;
		}
		else{
			P1OUT &= ~ BIT_TRIGGER;
			TA1CCR1 += PULSE_PERIOD_TICKS;
			gub_Pulse_Start = true;
		}
		break;
	case TA1IV_TACCR2:
	default: break;
	}
	//clear flag
	TA1CTL &= ~TAIFG;
}


