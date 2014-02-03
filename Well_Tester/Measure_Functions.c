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
#define TICK_FREQUENCY (CLOCK_FREQ/8)
#define TICK_RESOLUTION 50
#define CAP_DELAY 100

//function definitions
unsigned int cstlog2(unsigned int);
void Discharge_Cap(unsigned int);

//global variables
unsigned long gul_Tick_Count, gul_ADC_Total, gul_Freq_Target, gul_Freq_Count, gul_Max_Ticks;
unsigned int gui_Channel, gui_ADC_Count, gui_ADC_Target;
   

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

//this function uses timera channel 0 to measure the frequency of 
unsigned long Frequency_Read(unsigned int Channel, 
                            unsigned long Max_Input_Count,
                            unsigned long Max_Clock_Count,
                            bool Return_Total){
  //disable all interrupts
  __disable_interrupt();
  
  //clear global vars
  gul_Tick_Count = 0;
  gul_Freq_Count = 0;
  gul_Max_Ticks = Max_Clock_Count;
  gul_Freq_Target = Max_Input_Count;
  gui_Channel = Channel;

  
  //set up timera (continuous mode, source: smclk, clear, interrupts enabled, divide by 8)
  //Divide by 8 here is necessary as if not used the taccr1 misses the increment
  //and the frequency is then 1/full clock cycle (65535)
  TA1CTL = TASSEL_2 | MC_1 | TAIE | TACLR | ID_3;
   
  //set up inturrupt period
  TA1CCR0 = TICK_RESOLUTION;
  TA1CCTL0 |= CCIE;
  
  //set up interrupts on port 2, rising edge needed for cap 
  P2IE |= Channel;
  P2IES &= ~Channel;
  
  //start oscillation
  Discharge_Cap(Channel);
  
  //enter low power mode, wait for completion
   __bis_SR_register(CPUOFF + GIE); 
   
  //trap div0 error
  if(gul_Tick_Count > 0){
    if (Return_Total){
    	//return the total frequency count
    	return gul_Freq_Count;
    }
    else{
    	//returns a simple ratio of the counted frequency/counted clock ticks scaled to the clock frequency
    	return (1000UL*gul_Freq_Count*TICK_FREQUENCY)/gul_Tick_Count;
    }
  }
  else{
    return 0;
  }
     
}

//this funciton clears the cap at a given channel.
//first sets the pin to output, then clears it (sets low)
//last step is to set the pin back to input after a delay
void Discharge_Cap(unsigned int Channel){
  //setup oscillator
  //clear cap
  P2DIR = Channel;
  P2OUT = 0x0;
  
  //allow discharge
  __delay_cycles(CAP_DELAY);
  
  //start oscillation
  P2DIR &= ~Channel;
}

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

#pragma vector=TIMER1_A0_VECTOR
__interrupt void TIMER1_CCR0_ISR(void){
    if (gul_Tick_Count < gul_Max_Ticks){
      //increment tick count by 10 to increase performacne (slightly lower resolution
      gul_Tick_Count += (TICK_RESOLUTION + 1);
    }
    else{
      //disable interrupts
      TA1CTL &= ~TAIE;
       // Clear CPUOFF bit from 0(SR)
      __bic_SR_register_on_exit(CPUOFF);
    }
}

//clear flag here
#pragma vector=TIMER1_A1_VECTOR
__interrupt void TIMER1_OTHER_ISR(void){
  TA1CTL &= ~TAIFG;
}

#pragma vector = PORT2_VECTOR
__interrupt void PORT2_ISR(void){
  
 //start oscillation

	//copied start (inlining)
	//setup oscillator
	//clear cap
	  P2DIR = gui_Channel;
	  P2OUT = 0x0;

	  //allow discharge
	  __delay_cycles(CAP_DELAY);

	  //start oscillation
	  P2DIR &= ~gui_Channel;
	  //copied end (inlining

	//Discharge_Cap(gui_Channel);
   
   //counter
    if (gul_Freq_Count < gul_Freq_Target){
        //increment the frequency counter each timer interrupt is requested
        gul_Freq_Count++;
      }
      else {
        //disable interrupts and set flag
        //P2IE = 0x0;
        TA0CCTL1 &= ~CCIE;
        // Clear CPUOFF bit from 0(SR)
        __bic_SR_register_on_exit(CPUOFF);
      }
  //reset interrupt flag
  P2IFG = 0x0;
}
