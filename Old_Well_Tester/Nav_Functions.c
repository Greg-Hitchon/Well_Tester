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
				unsigned long Left_Distance,
				unsigned long Right_Distance,
				unsigned int Left_Profile_ID,
				unsigned int Right_Profile_ID);
void Start_Motor(unsigned int Motor_ID);
void Hold_Until_Finished(void);


//core preprocessor constants (distance in 0.1mm, frequency in KHz, time is in 1/100(seconds))
#define DIST_PER_ROT            (2500)
#define STEPS_PER_ROT           (203UL)
#define NUM_NAV_PROFILES		(2)
#define MIN_TICK_INCREMENT		(200)

//secondary (calculated) values
#define DISTANCE_PER_STEP               ((unsigned  int) (DIST_PER_ROT/STEPS_PER_ROT))

//**********************************************************************************************************||
//constants (calibration and system parameters)
//**********************************************************************************************************||
const unsigned long cad_Distance_Per_90[2] = {2020, 2020};

//**********************************************************************************************************||
//**********************************************************************************************************||

//structs
struct Motor_State {
	unsigned long Step_Target, Step_Target_By_2, Step_Count, Tick_Total;
    unsigned int Bit_States, Edge_State, Overflows_Remaining, Profile_ID, Num_Leftover, Num_Overflows;
	bool Is_Running, Is_Forwards, Is_B;
};

struct Nav_Profile{
	unsigned long Period;
	unsigned int Num_Overflows, Num_Leftover;
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
						unsigned long Target_Speed){

	//check if there is a valid profile id
	if (Profile_ID < NUM_NAV_PROFILES){

		//get starting ticks
		s_Nav_Profiles[Profile_ID].Period = (unsigned long) ((CLOCK_FREQ*1000000LL)/(4*Target_Speed));

		//get starting overflows
		s_Nav_Profiles[Profile_ID].Num_Overflows = ((unsigned int) ((s_Nav_Profiles[Profile_ID].Period)/ 0x10000UL));

		//get starting leftover
		s_Nav_Profiles[Profile_ID].Num_Leftover = ((unsigned int) ((s_Nav_Profiles[Profile_ID].Period) % 0x10000UL));

		//adjust for minimum
		if (s_Nav_Profiles[Profile_ID].Num_Leftover < MIN_TICK_INCREMENT){
			s_Nav_Profiles[Profile_ID].Num_Leftover = MIN_TICK_INCREMENT;
		}
	}
}


void Set_Timer(void){
  //continuous mode, overflow interrupt disabled, smclk source, no divider
  TA0CTL = TASSEL_2 | MC_2;
}

void Shutdown_Timer(void){
	//this just clears the control register
	TA0CTL = 0x0;
}

void Set_Motor_Outputs(void){
  //set up output (motor) bits
  P2DIR |= BIT_ALL_MOTORS;
  P2OUT &= ~BIT_ALL_MOTORS;
}


//this function sets up the struct used to define running parameters
//could use a better structure here, (put vals in array then loop through) but this is simpler if larger code and harder to maintain
void Set_Motor(	unsigned int Motor_ID,
				unsigned int Direction,
				unsigned long Left_Distance,
				unsigned long Right_Distance,
				unsigned int Left_Profile_ID,
				unsigned int Right_Profile_ID){

	//reset states here as we are not explicitly filling the entire struct
	Clear_State(Motor_ID);

	//set left motor
	if (Motor_ID & LEFT_MOTOR){
		//the "is_forwards" flag is not really necessary but makes code more readable/easy to debug
		s_Cur_Motor_State[LEFT].Is_Forwards = ((Direction & LEFT_BACKWARD)==0);

		//adjust for backwards bit here (by switching this bit we control motor direction
		if (s_Cur_Motor_State[LEFT].Is_Forwards == false){
		   s_Cur_Motor_State[LEFT].Bit_States = BIT_MLB;
		}

		//bring in info from the nav profile here, this initializes the dynamic counters in the motor struct (not really necessary with no acceleration)
		s_Cur_Motor_State[LEFT].Overflows_Remaining = s_Nav_Profiles[Left_Profile_ID].Num_Overflows;
		s_Cur_Motor_State[LEFT].Num_Overflows = s_Nav_Profiles[Left_Profile_ID].Num_Overflows;
		s_Cur_Motor_State[LEFT].Num_Leftover = s_Nav_Profiles[Left_Profile_ID].Num_Leftover;
		s_Cur_Motor_State[LEFT].Tick_Total = s_Nav_Profiles[Left_Profile_ID].Period;

		//get target here
		s_Cur_Motor_State[LEFT].Step_Target = ((unsigned long) Left_Distance/DISTANCE_PER_STEP);
		s_Cur_Motor_State[LEFT].Step_Target_By_2 = s_Cur_Motor_State[LEFT].Step_Target/2;

		//initialize running state and set the profile id which determines accel and speed characteristics
		s_Cur_Motor_State[LEFT].Is_Running = true;
		s_Cur_Motor_State[LEFT].Profile_ID = Left_Profile_ID;

	}
	//set right motor
	if (Motor_ID & RIGHT_MOTOR){
		//the "is_forwards" flag is not really necessary but makes code more readable/easy to debug
		s_Cur_Motor_State[RIGHT].Is_Forwards = ((Direction & RIGHT_BACKWARD)==0);

		//adjust for backwards bit here (by switching this bit we control motor direction
		if (s_Cur_Motor_State[RIGHT].Is_Forwards == false){
		   s_Cur_Motor_State[RIGHT].Bit_States = BIT_MRB;
		}

		//bring in info from the nav profile here, this initializes the dynamic counters in the motor struct
		s_Cur_Motor_State[RIGHT].Overflows_Remaining = s_Nav_Profiles[Right_Profile_ID].Num_Overflows;
		s_Cur_Motor_State[RIGHT].Num_Overflows = s_Nav_Profiles[Right_Profile_ID].Num_Overflows;
		s_Cur_Motor_State[RIGHT].Num_Leftover = s_Nav_Profiles[Right_Profile_ID].Num_Leftover;
		s_Cur_Motor_State[RIGHT].Tick_Total = s_Nav_Profiles[Right_Profile_ID].Period;

		//get target here
		s_Cur_Motor_State[RIGHT].Step_Target = ((unsigned long) Right_Distance/DISTANCE_PER_STEP);
		s_Cur_Motor_State[RIGHT].Step_Target_By_2 = s_Cur_Motor_State[RIGHT].Step_Target/2;

		//initialize running state and set the profile id which determines accel and speed characteristics
		s_Cur_Motor_State[RIGHT].Is_Running = true;
		s_Cur_Motor_State[RIGHT].Profile_ID = Right_Profile_ID;
	}
}

//this will save the current motor state structure or just clear state, not entirely necessary
void Stop_Motor(unsigned int Motor_ID, bool Do_Save){
	//save state (saves all of the current values to the global save vars)
	if(Do_Save == true){
		Save_State(Motor_ID);
	}

	//stop motors by resetting the struct to all 0's
	Clear_State(Motor_ID);

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

//copies over the specified structure to the save motor array
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

//copies the saved structure back to the running structures
void Restore_State(unsigned int Motor_ID){
  if (Motor_ID & LEFT_MOTOR){
    s_Cur_Motor_State[LEFT] = s_Sv_Motor_State[LEFT];
  }

  if (Motor_ID & RIGHT_MOTOR){
    s_Cur_Motor_State[RIGHT] = s_Sv_Motor_State[RIGHT];
  }
}

//before the motors run (and after the running structs have been populated) we need to initialize a few ports/states
void Start_Motor(unsigned int Motor_ID){
	//make sure we are starting with a blank slate here
	Set_Motor_Outputs();

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

//this can be used easily to put the uc into a low power mode waiting for a wakeup call
void Hold_Until_Finished(void){
	//this sets the cpuoff bit as well as interrupts added
  __bis_SR_register(CPUOFF + GIE); 
}

//**********************************************************************************************************||
//composite functions
//**********************************************************************************************************||
//these functions just package the basic functions into easier to use forms with common constants supplied
//for typical behaviour (turn (left/right), straight (backwards/forwards)
//**********************************************************************************************************||
//**********************************************************************************************************||

void Turn(	unsigned int Direction,
			unsigned int Left_Profile_ID,
			unsigned int Right_Profile_ID){

	//do this initialization every time just in case
	Set_Timer();

	//actually set motors
	if(Direction == LEFT){
		Set_Motor(BOTH_MOTORS,TURN_LEFT,cad_Distance_Per_90[LEFT], cad_Distance_Per_90[LEFT], Left_Profile_ID, Right_Profile_ID);
		Start_Motor(BOTH_MOTORS);
		Hold_Until_Finished();
	}
	else{
		Set_Motor(BOTH_MOTORS,TURN_RIGHT,cad_Distance_Per_90[RIGHT], cad_Distance_Per_90[RIGHT], Left_Profile_ID, Right_Profile_ID);
		Start_Motor(BOTH_MOTORS);
		Hold_Until_Finished();
	}
}

void Straight(	unsigned int Direction,
				unsigned long Left_Distance,
				unsigned long Right_Distance,
				unsigned int Left_Profile_ID,
				unsigned int Right_Profile_ID){

	//do this initialization every time just in case
	Set_Timer();

	//actually set motors
	if (Direction == FORWARD){
		Set_Motor(BOTH_MOTORS,BOTH_FORWARD,Left_Distance, Right_Distance, Left_Profile_ID, Right_Profile_ID);
		Start_Motor(BOTH_MOTORS);
		Hold_Until_Finished();
	}
	else{
		Set_Motor(BOTH_MOTORS,BOTH_BACKWARD,Left_Distance, Right_Distance, Left_Profile_ID, Right_Profile_ID);
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

#pragma vector=TIMER0_A0_VECTOR
__interrupt void TIMER0_CCRO_ISR(void){
	//interrupt flags are reset automatically
	__no_operation();
}

//clear flag here, also updates the motors
//note that the motor toggles for left and right are the same, but have seperate code, this is to eliminate all unnecessary code in the interrupt
//however at the frequencies we are running at this is not really needed

#pragma vector=TIMER0_A1_VECTOR
__interrupt void TIMER0_OTHER_ISR(void){
	//flags reset by reading ta0iv
  switch(__even_in_range(TA0IV,0xA)){
    case TA0IV_TACCR1:
    	//"overflow" means one full timer count in this case 2^16
		if (s_Cur_Motor_State[LEFT].Overflows_Remaining == 0){
			//check whether the motor has reached its termination criteria
			if (s_Cur_Motor_State[LEFT].Step_Count < s_Cur_Motor_State[LEFT].Step_Target){
				//increment step counter
				s_Cur_Motor_State[LEFT].Step_Count++;

				//reset the overflow count
				s_Cur_Motor_State[LEFT].Overflows_Remaining = s_Cur_Motor_State[LEFT].Num_Overflows;

				//this keeps track of a or b using a bool in the motor struct
				if (s_Cur_Motor_State[LEFT].Is_B){
					P2OUT ^= BIT_MLB;
					s_Cur_Motor_State[LEFT].Is_B = false;
				}
				else{
					P2OUT ^= BIT_MLA;
					s_Cur_Motor_State[LEFT].Is_B = true;
				}

				//update counter
				TA0CCR1 += s_Cur_Motor_State[LEFT].Num_Leftover;
			}
			else{
				//turn off interrupts
				TA0CCTL1 &= ~CCIE;

				//never save the state when the motor exits naturally
				Clear_State(LEFT_MOTOR);

				//check here for exit condition
				//stop interrupts if no motors are running
				if (s_Cur_Motor_State[RIGHT].Is_Running == false){
					__bic_SR_register_on_exit(CPUOFF);
				}
			}
		}
		else{
			s_Cur_Motor_State[LEFT].Overflows_Remaining--;
		}
		break;
    case TA0IV_TACCR2:
    	//"overflow" means one full timer count in this case 2^16
		if (s_Cur_Motor_State[RIGHT].Overflows_Remaining == 0){
			//check whether the motor has reached its termination criteria
			if (s_Cur_Motor_State[RIGHT].Step_Count < s_Cur_Motor_State[RIGHT].Step_Target){
				//increment step counter
				s_Cur_Motor_State[RIGHT].Step_Count++;

				//reset the overflow count
				s_Cur_Motor_State[RIGHT].Overflows_Remaining = s_Cur_Motor_State[RIGHT].Num_Overflows;

				//this keeps track of a or b using a bool in the motor struct
				if (s_Cur_Motor_State[RIGHT].Is_B){
					P2OUT ^= BIT_MRB;
					s_Cur_Motor_State[RIGHT].Is_B = false;
				}
				else{
					P2OUT ^= BIT_MRA;
					s_Cur_Motor_State[RIGHT].Is_B = true;
				}

				//update counter
				TA0CCR2 += s_Cur_Motor_State[RIGHT].Num_Leftover;
			}
			else{
				//turn off interrupts
				TA0CCTL2 &= ~CCIE;

				//stop motors by resetting the struct to all 0's
				Clear_State(RIGHT_MOTOR);

				//check here for exit condition
				//stop interrupts if no motors are running
				if (s_Cur_Motor_State[LEFT].Is_Running == false){
					__bic_SR_register_on_exit(CPUOFF);
				}
			}
		}
		else{
			s_Cur_Motor_State[RIGHT].Overflows_Remaining--;
		}
    default: break;
  }
}
