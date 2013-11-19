/*
 * Nav_Functions.c
 *
 *  Created on: Oct 16, 2013
 *      Author: Greg
 *
 *This file provides an easy method for running the motors
 *To run a motor the Set_Motor function then the Start_Motor function needs to be called
 *This is simplified by the "Straight" and "Turn" functions
 *
 *This function makes use of timer0 and the CCR0 interrupt and contains the
 *corresponding interrupt handling routine
 *
 *P2 is used for the motor output pins
 */

//syestem headers
#include "Project_Parameters.h"
#include TEST_CHIP

//user defined headers
#include "Bit_Definitions.h"
#include "cstbool.h"

//function prototypes
void Clear_State(unsigned int);
void Save_State(unsigned int);
void Set_Motor(	unsigned int Motor_ID,
				unsigned int Direction,
				unsigned long Distance,
				unsigned int Profile_ID);
void Start_Motor(unsigned int Motor_ID);
void Hold_Until_Finished(void);


//core preprocessor constants (distance in 0.1mm, frequency in KHz, time is in 1/100(seconds))
#define DIST_PER_ROT            (2500)
#define STEPS_PER_ROT           (203UL)
#define MIN_RPM                 (10UL)
#define MAX_RPM                 (100UL)
#define NUM_NAV_PROFILES		(2)

//this constant is used to calculate the initial period in the acceleration profile (Note we are assuming acceleration from 0 and deceleration to 0)
//for a linear ramp this constant can be found usingn the expression (empirically derived): C= 1/(0.6156*N^(-0.476)) where N is the number of steps before the target period is reached
//it is clear why this is not the best to calculate dynamically, a few values are; N=100->C=15, N=203->C=19, N=400->C=28, N=2000->C=60
#define SCALING_CONSTANT		(28)
#define ACC_STEPS				(2000)
#define DEC_STEPS				(203)
#define START_SPEED				(20UL)

//secondary (calculated) values
#define DISTANCE_PER_STEP               ((unsigned  int) (DIST_PER_ROT/STEPS_PER_ROT))
#define MAX_PERIOD				((unsigned long) (((150000UL*CLOCK_FREQ)/((MIN_RPM + (MAX_RPM-MIN_RPM)*(START_SPEED)*STEPS_PER_ROT))*10UL)))

//**********************************************************************************************************||
//constants (calibration and system parameters)
//**********************************************************************************************************||
const unsigned int cad_Distance_Per_90[2] = {16250, 16250};

//**********************************************************************************************************||
//**********************************************************************************************************||

//structs
struct Motor_State {
	unsigned long Step_Target, Step_Count, Tick_Total;
    unsigned int Bit_States, Edge_State, Overflows_Remaining, Profile_ID, Num_Leftover, Num_Overflows, Step_Target_By_2;
	bool Is_Running, Is_Forwards;
};

struct Nav_Profile{
	unsigned long Start_Ticks, Middle_Ticks;
	unsigned int Num_Overflows, Num_Leftover;
	bool Has_ACC, Has_DEC;
};

struct Motor_State s_Cur_Motor_State[2];
struct Motor_State s_Sv_Motor_State[2];
struct Nav_Profile s_Nav_Profiles[NUM_NAV_PROFILES];

//**********************************************************************************************************||
//core functions
//**********************************************************************************************************||
//these are the basic functions that allow the "composite functions" section to be built.
//included functions set up motors parameters, save the current state, and restore the saved state
//**********************************************************************************************************||
//**********************************************************************************************************||

void Create_Nav_Profile(unsigned int Profile_ID,
						unsigned int Target_Speed,
						bool Do_Acc,
						bool Do_Dec){

	//check if there is a valid profile id
	if (Profile_ID < NUM_NAV_PROFILES){

		//get starting ticks
		s_Nav_Profiles[Profile_ID].Middle_Ticks = ((unsigned long) (((150000UL*CLOCK_FREQ)/((MIN_RPM + (MAX_RPM-MIN_RPM)*((unsigned long) Target_Speed))*STEPS_PER_ROT))*10UL));

		if (Do_Acc){
			//scale the ticks value if acceleration is set
			s_Nav_Profiles[Profile_ID].Start_Ticks = s_Nav_Profiles[Profile_ID].Middle_Ticks*SCALING_CONSTANT;

			//check if this is greater than the max period
			if (s_Nav_Profiles[Profile_ID].Start_Ticks > MAX_PERIOD){
				s_Nav_Profiles[Profile_ID].Start_Ticks = MAX_PERIOD;
			}
		}
		else {
			s_Nav_Profiles[Profile_ID].Start_Ticks = s_Nav_Profiles[Profile_ID].Middle_Ticks;
		}

		//get starting overflows
		s_Nav_Profiles[Profile_ID].Num_Overflows = ((unsigned int) ((s_Nav_Profiles[Profile_ID].Start_Ticks)/0x10000UL));

		//get starting leftover
		s_Nav_Profiles[Profile_ID].Num_Leftover = ((unsigned int) ((s_Nav_Profiles[Profile_ID].Start_Ticks) % 0xFFFFUL));

		//this lets the interrupt loop know that period will need adjustment
		s_Nav_Profiles[Profile_ID].Has_DEC = Do_Dec;

		//this lets the interrupt loop know that period will need adjustment
		s_Nav_Profiles[Profile_ID].Has_ACC = Do_Acc;

	}
}


void Set_Timer(void){
  //continuous mode, interrupts enabled, smclk sourcee
  TA0CTL = TASSEL_2 | TAIE | MC_2;
}

void Shutdown_Timer(void){
  TA0CTL = 0x0;
}

void Set_Motor_Outputs(void){
  //set up output (motor) bits
  P2DIR |= BIT_ALL_MOTORS;
  P2OUT &= ~BIT_ALL_MOTORS;
}

void Set_Motor(	unsigned int Motor_ID,
				unsigned int Direction,
				unsigned long Distance,
				unsigned int Profile_ID){
	unsigned int i;
	//unsigned long ul_Steps_ACC, ul_Steps_DEC;

	//reset states here as we are not explicitly filling the entire struct
	Clear_State(Motor_ID);
        
	//here we initialize the number of overflows necessary
	for (i = 0; i<2; i++){
	  s_Cur_Motor_State[i].Overflows_Remaining = s_Nav_Profiles[Profile_ID].Num_Overflows;
	  s_Cur_Motor_State[i].Num_Overflows = s_Nav_Profiles[Profile_ID].Num_Overflows;
	  s_Cur_Motor_State[i].Num_Leftover = s_Nav_Profiles[Profile_ID].Num_Leftover;
	  s_Cur_Motor_State[i].Tick_Total = s_Nav_Profiles[Profile_ID].Start_Ticks;
	  s_Cur_Motor_State[i].Step_Target = ((unsigned long) Distance/DISTANCE_PER_STEP);
	  s_Cur_Motor_State[i].Step_Target_By_2 = s_Cur_Motor_State[i].Step_Target/2;
	  s_Cur_Motor_State[i].Is_Running = true;
	  s_Cur_Motor_State[i].Profile_ID = Profile_ID;
	}

        //set left motor
	if (Motor_ID & LEFT_MOTOR){
		s_Cur_Motor_State[LEFT].Is_Forwards = ((Direction & LEFT_BACKWARD)==0);
		//adjust for backwards bit here
		if (s_Cur_Motor_State[LEFT].Is_Forwards == false){
		   s_Cur_Motor_State[LEFT].Bit_States = BIT_MLB;
		}
	}
	//set right motor
	if (Motor_ID & RIGHT_MOTOR){
		s_Cur_Motor_State[RIGHT].Is_Forwards = ((Direction & RIGHT_BACKWARD)==0);
		//adjust for backwards bit here
		 if (s_Cur_Motor_State[RIGHT].Is_Forwards == false){
		   s_Cur_Motor_State[RIGHT].Bit_States = BIT_MRB;
		}
	}
}

void Stop_Motor(unsigned int Motor_ID, bool Do_Save){
	//save state (saves all of the current values to the global save vars)
	if(Do_Save == true){
          Save_State(Motor_ID);
	}

	//stop motors by resetting the struct to all 0's
	Clear_State(Motor_ID);

	//reset bits
	if (Motor_ID & LEFT_MOTOR){
		P2OUT &= ~BIT_LEFT_MOTOR;
	}

	if (Motor_ID & RIGHT_MOTOR){
		P2OUT &= ~BIT_RIGHT_MOTOR;
	}
}


void Clear_State(unsigned int Motor_ID){
	static const struct Motor_State Empty_Struct = {0};

	if (Motor_ID & LEFT_MOTOR){
		s_Cur_Motor_State[LEFT] = Empty_Struct;
	}

	if (Motor_ID & RIGHT_MOTOR){
		s_Cur_Motor_State[RIGHT] = Empty_Struct;
	}
}

void Save_State(unsigned int Motor_ID){
	if (Motor_ID & LEFT_MOTOR){
		s_Sv_Motor_State[LEFT] = s_Cur_Motor_State[LEFT];
        s_Sv_Motor_State[LEFT].Bit_States = (P2OUT & BIT_LEFT_MOTOR);
	}

	if (Motor_ID & RIGHT_MOTOR){
		s_Sv_Motor_State[RIGHT] = s_Cur_Motor_State[RIGHT];
        s_Sv_Motor_State[RIGHT].Bit_States = (P2OUT & BIT_RIGHT_MOTOR);
	}
}

void Restore_State(unsigned int Motor_ID){
  if (Motor_ID & LEFT_MOTOR){
    s_Cur_Motor_State[LEFT] = s_Sv_Motor_State[LEFT];
  }

  if (Motor_ID & RIGHT_MOTOR){
    s_Cur_Motor_State[RIGHT] = s_Sv_Motor_State[RIGHT];
  }
}

void Start_Motor(unsigned int Motor_ID){
  //here we set the bit states specified by the "settings" struct
  if (Motor_ID & LEFT_MOTOR){
        P2OUT &= ~BIT_LEFT_MOTOR;
        P2OUT |= s_Cur_Motor_State[LEFT].Bit_States;
        TA0CCR1 = s_Cur_Motor_State[LEFT].Num_Leftover;
        //start interrupts
        TA0CCTL1 |= CCIE;
  }

  if (Motor_ID & RIGHT_MOTOR){
        P2OUT &= ~BIT_RIGHT_MOTOR;
        P2OUT |= s_Cur_Motor_State[RIGHT].Bit_States;
        TA0CCR2 = s_Cur_Motor_State[RIGHT].Num_Leftover;
        //start interrupts
        TA0CCTL2 |= CCIE;
  }
}


void Hold_Until_Finished(void){
  __bis_SR_register(CPUOFF + GIE); 
  //while((s_Cur_Motor_State[LEFT].Is_Running == true) || (s_Cur_Motor_State[RIGHT].Is_Running == true)){
  //}
}

//**********************************************************************************************************||
//composite functions
//**********************************************************************************************************||
//these functions just package the basic functions into easier to use forms with common constants supplied
//for typical behaviour (turn (left/right), straight (backwards/forwards)
//**********************************************************************************************************||
//**********************************************************************************************************||

void Turn(unsigned int Direction, unsigned int Profile_ID){
  if(Direction == LEFT){
      Set_Motor(BOTH_MOTORS,TURN_LEFT,cad_Distance_Per_90[LEFT],Profile_ID);
      Start_Motor(BOTH_MOTORS);
      Hold_Until_Finished();
  }
  else{
      Set_Motor(BOTH_MOTORS,TURN_RIGHT,cad_Distance_Per_90[RIGHT],Profile_ID);
      Start_Motor(BOTH_MOTORS);
      Hold_Until_Finished();
  }
}

void Straight(unsigned int Direction, unsigned long Distance, unsigned int Profile_ID){
  if (Direction == FORWARD){
    Set_Motor(BOTH_MOTORS,BOTH_FORWARD,Distance, Profile_ID);
    Start_Motor(BOTH_MOTORS);
    Hold_Until_Finished();
  }
  else{
    Set_Motor(BOTH_MOTORS,BOTH_BACKWARD,Distance, Profile_ID);
    Start_Motor(BOTH_MOTORS);
    Hold_Until_Finished();
  }
}

//**********************************************************************************************************||
//Interrupts
//**********************************************************************************************************||
//this is where the 4 pulse width signals are output.  Each has the same period (so same speed) and each motor has two pulses (A and B)
//This means that there needs to be a Period/4 frequency of interrupts.
//**********************************************************************************************************||
//**********************************************************************************************************||

//#pragma vector=TIMER0_A0_VECTOR
//__interrupt void TIMER0_CCRO_ISR(void){
//}

//clear flag here, also updates the
#pragma vector=TIMER0_A1_VECTOR
__interrupt void TIMER0_OTHER_ISR(void){
  unsigned int i, Motor_ID, INDEX, Profile_ID;
  bool Do_Toggle = false;
  unsigned int volatile *Counter;

  switch(__even_in_range(TA0IV,0xA)){
    case TA0IV_TAIFG:
    	//decrement overflows if needed from left motor
        if (s_Cur_Motor_State[LEFT].Overflows_Remaining > 0){
          s_Cur_Motor_State[LEFT].Overflows_Remaining--;
        }

        //decrement overflows if needed from left motor
		if (s_Cur_Motor_State[RIGHT].Overflows_Remaining > 0){
		  s_Cur_Motor_State[RIGHT].Overflows_Remaining--;
		}
      break;
    case TA0IV_TACCR1:
		//first check if there are still overflows
		if (s_Cur_Motor_State[LEFT].Overflows_Remaining == 0){
			Do_Toggle = true;
			Motor_ID = LEFT_MOTOR;
			Counter = &TA0CCR1;
		}
		break;
    case TA0IV_TACCR2:
		if (s_Cur_Motor_State[RIGHT].Overflows_Remaining == 0){
			Do_Toggle = true;
			Motor_ID = RIGHT_MOTOR;
			Counter = &TA0CCR2;
		}
    default: break;
  }
  
  if(Do_Toggle){
	//motor id's are 1 and 2 to differentiate (can tell either or both) so index is 0, 1
	INDEX = Motor_ID - 1;

	//check whether the motor has reached its termination criteria
	if (s_Cur_Motor_State[INDEX].Step_Count < s_Cur_Motor_State[INDEX].Step_Target){
		//so as to not access the structure all the time save profile ID
		Profile_ID = s_Cur_Motor_State[INDEX].Profile_ID;

		//within this if statement we perform all the actions needed on a per STEP basis such as acceleration
		if (++s_Cur_Motor_State[INDEX].Edge_State == 4){
			//reset state
			s_Cur_Motor_State[INDEX].Edge_State = 0;

			//increment step counter
			s_Cur_Motor_State[INDEX].Step_Count++;

			//check for acceleration
			if (s_Nav_Profiles[Profile_ID].Has_ACC){
				//check for acceleration update here
				if (	(s_Cur_Motor_State[INDEX].Tick_Total > s_Nav_Profiles[Profile_ID].Middle_Ticks) &&
						(s_Cur_Motor_State[INDEX].Step_Count <= ACC_STEPS) &&
						(s_Cur_Motor_State[INDEX].Step_Count <= s_Cur_Motor_State[INDEX].Step_Target_By_2)){
					//adjust the period
					s_Cur_Motor_State[INDEX].Tick_Total -= (2*s_Cur_Motor_State[INDEX].Tick_Total)/(4*s_Cur_Motor_State[INDEX].Step_Count + 1);

					//check for overshoot
					if (s_Cur_Motor_State[INDEX].Tick_Total < s_Nav_Profiles[Profile_ID].Middle_Ticks){
						s_Cur_Motor_State[INDEX].Tick_Total = s_Nav_Profiles[Profile_ID].Middle_Ticks;
					}

					//adjust the overflow total and leftover
					s_Cur_Motor_State[INDEX].Num_Overflows = (s_Cur_Motor_State[INDEX].Tick_Total/0x10000UL);
					s_Cur_Motor_State[INDEX].Num_Leftover = (s_Cur_Motor_State[INDEX].Tick_Total % 0xFFFFUL);
				}
			}

			//check for deceleration
			if (s_Nav_Profiles[Profile_ID].Has_DEC){
				//check for acceleration update here
				if ((	(s_Cur_Motor_State[INDEX].Tick_Total < MAX_PERIOD) &&
						(s_Cur_Motor_State[INDEX].Step_Target - s_Cur_Motor_State[INDEX].Step_Count) <= DEC_STEPS) &&
						(s_Cur_Motor_State[INDEX].Step_Count > s_Cur_Motor_State[INDEX].Step_Target_By_2)){
					//adjust the period between ticks
					s_Cur_Motor_State[INDEX].Tick_Total += (2*s_Cur_Motor_State[INDEX].Tick_Total)/(4*(s_Cur_Motor_State[INDEX].Step_Target - s_Cur_Motor_State[INDEX].Step_Count) + 1);

					//adjust for overshoot here
					if (s_Cur_Motor_State[INDEX].Tick_Total > MAX_PERIOD){
						s_Cur_Motor_State[INDEX].Tick_Total = MAX_PERIOD;
					}

					//adjust the overflow total and leftover
					s_Cur_Motor_State[INDEX].Num_Overflows = (s_Cur_Motor_State[INDEX].Tick_Total/0x10000UL);
					s_Cur_Motor_State[INDEX].Num_Leftover = (s_Cur_Motor_State[INDEX].Tick_Total % 0xFFFFUL);
				}
			}
		}

		//reset the overflow count
		s_Cur_Motor_State[INDEX].Overflows_Remaining = s_Cur_Motor_State[INDEX].Num_Overflows;

		//this sets the provided register (taccr1 or taccr2)
		*Counter += s_Cur_Motor_State[INDEX].Num_Leftover;

		//here we check if we are updating A or B output bits (not sure if % would be better here)
		if ((s_Cur_Motor_State[INDEX].Edge_State == 1) || (s_Cur_Motor_State[INDEX].Edge_State == 3)){
			//update the correct pins on A channel here
			if (INDEX==LEFT){
				P2OUT ^= BIT_MLA;
			}
			else{
				P2OUT ^= BIT_MRA;
			}
		}
		else{
			//update the correct pins on B channel here
			if (INDEX==LEFT){
				P2OUT ^= BIT_MLB;
			}
			else{
				P2OUT ^= BIT_MRB;
			}
		}
	}
	else{
		//never save the state when the motor exits naturally
		if(INDEX==LEFT){
			Stop_Motor(LEFT_MOTOR,false);
			//turn off interrupts
			TA0CCTL1 &= ~CCIE;
		}
		else if(INDEX==RIGHT){
			Stop_Motor(RIGHT_MOTOR,false);
			//turn off interrupts
			TA0CCTL2 &= ~CCIE;
		}
	}
	//check here for exit condition
	//stop interrupts if no motors are running
	if ((s_Cur_Motor_State[LEFT].Is_Running == false) && (s_Cur_Motor_State[RIGHT].Is_Running == false)){
		__bic_SR_register_on_exit(CPUOFF);
	}
  }
}
