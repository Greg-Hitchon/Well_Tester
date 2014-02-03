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
				unsigned long Steps,
				unsigned int Profile_ID);
void Start_Motor(unsigned int Motor_ID);
void Hold_Until_Finished(void);
void Update_State(unsigned int);


//core preprocessor constants (distance in 0.1mm, frequency in KHz, time is in 1/100(seconds))
#define NUM_NAV_PROFILES		(1)
#define MIN_TICK_INCREMENT		(1000)
#define NUM_STATES				(8)
#define NUM_MOTORS				(2)

//secondary (calculated) values
#define ADJ_CLOCK_FREQ					(((unsigned long) CLOCK_FREQ)*12500UL)

//**********************************************************************************************************||
//constants (calibration and system parameters)
//**********************************************************************************************************||
const unsigned int cui_Steps_Per_Sweep = 660;
const unsigned int cui_Steps_Per_Dime = 348;
//This configuration implies:
//Motor 1: a=Bit0, a'=Bit2; b=Bit1, b'=Bit3
//Motor 2: a=Bit4, a'=Bit6; b=Bit5, b'=Bit7
const char cch_State_Map[NUM_MOTORS][NUM_STATES] = {{BIT0,BIT3,BIT1,BIT0,BIT2,BIT1,BIT3,BIT2},{BIT4,BIT7,BIT5,BIT4,BIT6,BIT5,BIT7,BIT6}};


//**********************************************************************************************************||
//**********************************************************************************************************||

//structs
//Note:  step target is needed because we are counting up not down.  This is for code clarity.
struct Motor_State {
	unsigned long Step_Target, Step_Count, Tick_Total;
    unsigned int Direction, Overflows_Remaining, Profile_ID, Num_Leftover, Num_Overflows,
    			Speed, ACC_End, DEC_Start, ACC_Rate, DEC_Rate, ACC_Period, DEC_Period, ACC_Period_Count, DEC_Period_Count;
	bool Is_Running, Has_ACC, Has_DEC, Is_Concurrent;
};

struct Nav_Profile{
	unsigned long Period;
	unsigned int Num_Overflows, Num_Leftover, ACC_Rate, DEC_Rate, ACC_Period, DEC_Period, ACC_Steps, DEC_Steps,
				Start_Speed, Target_Speed, End_Speed;
	bool Has_DEC, Has_ACC;
};

struct Motor_State s_Cur_Motor_State[NUM_MOTORS];
//struct Motor_State s_Sv_Motor_State[2];
struct Nav_Profile s_Nav_Profiles[NUM_NAV_PROFILES];

//variables
unsigned int caui_Last_State[2] ={NUM_STATES+1};
unsigned int caui_State_Direction[2]={FORWARD};

//**********************************************************************************************************||
//core functions
//**********************************************************************************************************||
//these are the basic functions that allow the "composite functions" section to be built.
//included functions set up motors parameters, save the current state, and restore the saved state
//**********************************************************************************************************||
//**********************************************************************************************************||


//sets the state to 0 state
void Initialize_Bits(void){
	//set last state to 0
	caui_Last_State[LEFT]=0;
	caui_Last_State[RIGHT]=0;

	//update motor pins
	P2DIR |= 0xFF;
	P2SEL = 0x0;
	P2IN &= ~0xFF;
	P2IFG = 0x0;
	//for each motor we need to set bits to the initial 0 state
	P2OUT = BIT0 | BIT3 | BIT4 | BIT7;
}

//this dictates the target steps, speed etc.  This is used as a template to construct the navigation
//parameters within a specific motor struct.  The values will change dependent on the step count
void Create_Nav_Profile(unsigned int Profile_ID,
						unsigned int Start_Speed,
						unsigned int Target_Speed,
						unsigned int End_Speed,
						unsigned int ACC_Rate,
						unsigned int DEC_Rate,
						unsigned int ACC_Period,
						unsigned int DEC_Period){

	//check if there is a valid profile id
	if (Profile_ID < NUM_NAV_PROFILES){
		//get starting ticks
		s_Nav_Profiles[Profile_ID].Period = (unsigned long) (ADJ_CLOCK_FREQ/(4*Start_Speed));

		//get starting overflows
		s_Nav_Profiles[Profile_ID].Num_Overflows = ((unsigned int) ((s_Nav_Profiles[Profile_ID].Period)/ 0x10000UL));

		//get starting leftover
		s_Nav_Profiles[Profile_ID].Num_Leftover = ((unsigned int) ((s_Nav_Profiles[Profile_ID].Period) % 0x10000UL));

		//adjust for minimum
		if (s_Nav_Profiles[Profile_ID].Num_Leftover < MIN_TICK_INCREMENT){
			s_Nav_Profiles[Profile_ID].Num_Leftover = MIN_TICK_INCREMENT;
		}

		//**************************************************************************************************
		//Deal with acc/dec here
		//**************************************************************************************************
		s_Nav_Profiles[Profile_ID].Start_Speed = Start_Speed;
		s_Nav_Profiles[Profile_ID].Target_Speed = Target_Speed;
		s_Nav_Profiles[Profile_ID].End_Speed = End_Speed;

		//get the number of ticks that acceleration will take
		s_Nav_Profiles[Profile_ID].ACC_Steps = ((Target_Speed-Start_Speed)*ACC_Period)/ACC_Rate;

		//get number of ticks that decelleration will take
		s_Nav_Profiles[Profile_ID].DEC_Steps = ((Target_Speed-End_Speed)*DEC_Period)/DEC_Rate;

		//here we are just "saving" the inputs to the struct
		s_Nav_Profiles[Profile_ID].ACC_Period = ACC_Period;
		s_Nav_Profiles[Profile_ID].DEC_Period = DEC_Period;

		s_Nav_Profiles[Profile_ID].ACC_Rate = ACC_Rate;
		s_Nav_Profiles[Profile_ID].DEC_Rate = DEC_Rate;

		//save whether of not the profile has an acceleration
		if (Target_Speed > Start_Speed){
			s_Nav_Profiles[Profile_ID].Has_ACC = true;
		}

		if (Target_Speed > End_Speed){
			s_Nav_Profiles[Profile_ID].Has_DEC = true;
		}
	}
}


void Set_Timer(void){
  //continuous mode, overflow interrupt disabled, smclk source, divide by 8
  TA0CTL = TASSEL_2 | MC_2 | ID_3;
}


//this function sets up the struct used to define running parameters
void Set_Motor(	unsigned int Motor_ID,
				unsigned int Direction,
				unsigned long Steps,
				unsigned int Profile_ID){

	unsigned int ui_Temp_Target_Speed = 0, ui_Motor_Index;

	//reset states here as we are not explicitly filling the entire struct
	Clear_State(Motor_ID);

	//adjust immediately for concurrency
	if (Motor_ID==BOTH_MOTORS){
		//NOTE: CHANGING MOTOR ID HERE
		Motor_ID = CONC_MOTOR;
		ui_Motor_Index=CONC_MOTOR - 1;
		//save directionality
		s_Cur_Motor_State[ui_Motor_Index].Direction = Direction;
		//save concurrency bool
		s_Cur_Motor_State[ui_Motor_Index].Is_Concurrent = true;
	}
	else{
		//get the index value (just id-1)
		ui_Motor_Index = Motor_ID-1;
		//have to explicitly get direction as take an int for both, but just forward/back for single
		if (Motor_ID == LEFT_MOTOR){
			if (Direction == FORWARD){
				s_Cur_Motor_State[ui_Motor_Index].Direction = LEFT_FORWARD;
			}
			else{
				s_Cur_Motor_State[ui_Motor_Index].Direction = LEFT_BACKWARD;
			}
		}
		else if (Motor_ID == RIGHT_MOTOR){
			if (Direction == FORWARD){
				s_Cur_Motor_State[ui_Motor_Index].Direction = RIGHT_FORWARD;
			}
			else{
				s_Cur_Motor_State[ui_Motor_Index].Direction = RIGHT_BACKWARD;
			}
		}
	}

	//get target steps here
	s_Cur_Motor_State[ui_Motor_Index].Step_Target = Steps;

	//bring in info from the nav profile here, this initializes the dynamic counters in the motor struct (not really necessary with no acceleration)
	s_Cur_Motor_State[ui_Motor_Index].Overflows_Remaining = s_Nav_Profiles[Profile_ID].Num_Overflows;
	s_Cur_Motor_State[ui_Motor_Index].Num_Overflows = s_Nav_Profiles[Profile_ID].Num_Overflows;
	s_Cur_Motor_State[ui_Motor_Index].Num_Leftover = s_Nav_Profiles[Profile_ID].Num_Leftover;
	s_Cur_Motor_State[ui_Motor_Index].Tick_Total = s_Nav_Profiles[Profile_ID].Period;


	//**********************************************************************************************************
	//acceleration nonsense
	//**********************************************************************************************************

	//here we need to check if the distance is great enough to fit in the acc and dec profiles.
	//if not we reduce the target speed until possible, note the rates dont change
	//this is not a perfect solution
	if ((s_Nav_Profiles[Profile_ID].DEC_Steps + s_Nav_Profiles[Profile_ID].ACC_Steps) > Steps){
		//get new speed
		ui_Temp_Target_Speed = ((Steps +
			s_Nav_Profiles[Profile_ID].Start_Speed/s_Nav_Profiles[Profile_ID].ACC_Rate +
			s_Nav_Profiles[Profile_ID].End_Speed/s_Nav_Profiles[Profile_ID].DEC_Rate) *
			(s_Nav_Profiles[Profile_ID].ACC_Rate*s_Nav_Profiles[Profile_ID].DEC_Rate)) /
			(s_Nav_Profiles[Profile_ID].ACC_Rate + s_Nav_Profiles[Profile_ID].DEC_Rate);

		//calculate new step counts
		if (ui_Temp_Target_Speed > s_Nav_Profiles[Profile_ID].Start_Speed){
			s_Cur_Motor_State[ui_Motor_Index].ACC_End = ((ui_Temp_Target_Speed-s_Nav_Profiles[Profile_ID].Start_Speed)*s_Nav_Profiles[Profile_ID].ACC_Period)/s_Nav_Profiles[Profile_ID].ACC_Rate;
		}
		else{
			s_Cur_Motor_State[ui_Motor_Index].ACC_End = 0;
		}

		if(ui_Temp_Target_Speed > s_Nav_Profiles[Profile_ID].End_Speed){
			s_Cur_Motor_State[ui_Motor_Index].DEC_Start =  1 + Steps - ((ui_Temp_Target_Speed-s_Nav_Profiles[Profile_ID].End_Speed)*s_Nav_Profiles[Profile_ID].DEC_Period)/s_Nav_Profiles[Profile_ID].DEC_Rate;
		}
		else{
			s_Cur_Motor_State[ui_Motor_Index].DEC_Start = 0;
		}
	}
	else{
		s_Cur_Motor_State[ui_Motor_Index].ACC_End = s_Nav_Profiles[Profile_ID].ACC_Steps;
		s_Cur_Motor_State[ui_Motor_Index].DEC_Start = 1 + Steps - s_Nav_Profiles[Profile_ID].DEC_Steps;
	}

	//save all necessary values
	s_Cur_Motor_State[ui_Motor_Index].ACC_Rate = s_Nav_Profiles[Profile_ID].ACC_Rate;
	s_Cur_Motor_State[ui_Motor_Index].DEC_Rate = s_Nav_Profiles[Profile_ID].DEC_Rate;
	s_Cur_Motor_State[ui_Motor_Index].ACC_Period = s_Nav_Profiles[Profile_ID].ACC_Period;
	s_Cur_Motor_State[ui_Motor_Index].DEC_Period = s_Nav_Profiles[Profile_ID].DEC_Period;
	s_Cur_Motor_State[ui_Motor_Index].ACC_Period_Count = s_Nav_Profiles[Profile_ID].ACC_Period;
	s_Cur_Motor_State[ui_Motor_Index].DEC_Period_Count = s_Nav_Profiles[Profile_ID].DEC_Period;
	s_Cur_Motor_State[ui_Motor_Index].Speed = s_Nav_Profiles[Profile_ID].Start_Speed;
	if(s_Cur_Motor_State[ui_Motor_Index].ACC_End > 0){
		s_Cur_Motor_State[ui_Motor_Index].Has_ACC = true;
	}
	if(s_Cur_Motor_State[ui_Motor_Index].DEC_Start > 0){
		s_Cur_Motor_State[ui_Motor_Index].Has_DEC = s_Nav_Profiles[Profile_ID].Has_DEC;
	}

	//initialize running state and set the profile id which determines accel and speed characteristics
	s_Cur_Motor_State[ui_Motor_Index].Profile_ID = Profile_ID;
	s_Cur_Motor_State[ui_Motor_Index].Is_Running = true;
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

/*
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
*/

//updates bits, takes either motor or both
void Update_State(unsigned int Motor_Index){
	unsigned int Bit_Total = 0, i, Motor_ID;
	bool ba_Is_Backwards[2]={false};


	if(s_Cur_Motor_State[Motor_Index].Is_Concurrent){
		Motor_ID = BOTH_MOTORS;
		ba_Is_Backwards[LEFT] = (s_Cur_Motor_State[Motor_Index].Direction & LEFT_BACKWARD);
		ba_Is_Backwards[RIGHT] =  (s_Cur_Motor_State[Motor_Index].Direction & RIGHT_BACKWARD);
	}
	else if (Motor_Index == LEFT) {
		Motor_ID = LEFT_MOTOR;
		ba_Is_Backwards[LEFT] = (s_Cur_Motor_State[LEFT].Direction & LEFT_BACKWARD);
	}
	else{
		Motor_ID= RIGHT_MOTOR;
		ba_Is_Backwards[RIGHT] = (s_Cur_Motor_State[RIGHT].Direction & RIGHT_BACKWARD);
	}

	for(i=0; i<2;i++){
		if(Motor_ID & (i+1)){
			//if backwards then we move the state backwards else forwards
			if(ba_Is_Backwards[i]){
				if(caui_State_Direction[i] == BACKWARD){
					if(caui_Last_State[i] == 0){
						caui_Last_State[i] = (NUM_STATES - 1);
					}
					else{
						caui_Last_State[i]--;
					}
				}
				else{
					caui_State_Direction[i] = BACKWARD;
				}
			}
			else {
				if(caui_State_Direction[i] == FORWARD){
					if(++caui_Last_State[i] > (NUM_STATES-1)){
						caui_Last_State[i] = 0;
					}
				}
				else{
					caui_State_Direction[i] = FORWARD;
				}
			}

			//add to bit total
			Bit_Total += (unsigned int) (cch_State_Map[i][caui_Last_State[i]]);
		}
	}

	//update motor state
	P2OUT ^= Bit_Total;
}

//before the motors run (and after the running structs have been populated) we need to initialize a few ports/states
void Start_Motor(unsigned int Motor_ID){
	//check for concurrent here
	if ((Motor_ID & CONC_MOTOR) && (s_Cur_Motor_State[CONC_MOTOR-1].Is_Concurrent)){
		Motor_ID = CONC_MOTOR;
	}

	//do this initialization every time just in case
	Set_Timer();

	//here we set the bit states specified by the "settings" struct
	if (Motor_ID & LEFT_MOTOR){
		TA0CCR1 = s_Cur_Motor_State[LEFT].Num_Leftover;
		//start interrupts
		TA0CCTL1 |= CCIE;
	}

	if (Motor_ID & RIGHT_MOTOR){
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
			unsigned int Profile_ID,
			unsigned int Type){


	if(Type == SWEEP){
	//actually set motors
		if(Direction == LEFT){
			Set_Motor(RIGHT_MOTOR, FORWARD, cui_Steps_Per_Sweep, Profile_ID);
			Start_Motor(RIGHT_MOTOR);
			Hold_Until_Finished();
		}
		else{
			Set_Motor(LEFT_MOTOR, FORWARD, cui_Steps_Per_Sweep, Profile_ID);
			Start_Motor(LEFT_MOTOR);
			Hold_Until_Finished();
		}
	}
	else{
		//actually set motors
		if(Direction == LEFT){
			Set_Motor(BOTH_MOTORS,TURN_LEFT,cui_Steps_Per_Dime, Profile_ID);
			Start_Motor(BOTH_MOTORS);
			Hold_Until_Finished();
		}
		else{
			Set_Motor(BOTH_MOTORS,TURN_RIGHT,cui_Steps_Per_Dime, Profile_ID);
			Start_Motor(BOTH_MOTORS);
			Hold_Until_Finished();
		}
	}
}

void Straight(	unsigned int Direction,
				unsigned long Steps,
				unsigned int Profile_ID){

	//actually set motors
	if (Direction == FORWARD){
		Set_Motor(BOTH_MOTORS,BOTH_FORWARD,Steps,Profile_ID);
		Start_Motor(BOTH_MOTORS);
		Hold_Until_Finished();
	}
	else{
		Set_Motor(BOTH_MOTORS,BOTH_BACKWARD,Steps,Profile_ID);
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

#pragma vector=TIMER0_A1_VECTOR
__interrupt void TIMER0_OTHER_ISR(void){
	//bit flag
	unsigned int ui_Motor_Index;
	bool b_Do_Update = false;

	//flags reset by reading ta0iv
	switch(__even_in_range(TA0IV,0xA)){
	case TA0IV_TACCR1:
		//flag update needed
		b_Do_Update = true;
		//assign index
		ui_Motor_Index = LEFT;

		break;
	case TA0IV_TACCR2:
		//flag update needed
		b_Do_Update = true;
		//assign index
		ui_Motor_Index = RIGHT;
	default: break;
	}

	//update selected motor
	if(b_Do_Update){
		//"overflow" means one full timer count in this case 2^16
		if (s_Cur_Motor_State[ui_Motor_Index].Overflows_Remaining == 0){
			//check whether the motor has reached its termination criteria
			if (s_Cur_Motor_State[ui_Motor_Index].Step_Count < s_Cur_Motor_State[ui_Motor_Index].Step_Target){
				//increment step counter
				s_Cur_Motor_State[ui_Motor_Index].Step_Count++;

				//actually update pins here
				Update_State(ui_Motor_Index);

				//update period here if necessary
				if ((s_Cur_Motor_State[ui_Motor_Index].Has_ACC) &&
						(s_Cur_Motor_State[ui_Motor_Index].Step_Count <=  s_Cur_Motor_State[ui_Motor_Index].ACC_End) &&
						(--s_Cur_Motor_State[ui_Motor_Index].ACC_Period_Count == 0)){

					//reset period count
					s_Cur_Motor_State[ui_Motor_Index].ACC_Period_Count = s_Cur_Motor_State[ui_Motor_Index].ACC_Period;

					//get starting ticks
					s_Cur_Motor_State[ui_Motor_Index].Speed += s_Cur_Motor_State[ui_Motor_Index].ACC_Rate;

					s_Cur_Motor_State[ui_Motor_Index].Tick_Total = (unsigned long) (ADJ_CLOCK_FREQ/(4*s_Cur_Motor_State[ui_Motor_Index].Speed));

					//get starting overflows
					s_Cur_Motor_State[ui_Motor_Index].Num_Overflows = ((unsigned int) ((s_Cur_Motor_State[ui_Motor_Index].Tick_Total)/ 0x10000UL));

					//get starting leftover
					s_Cur_Motor_State[ui_Motor_Index].Num_Leftover = ((unsigned int) ((s_Cur_Motor_State[ui_Motor_Index].Tick_Total) % 0x10000UL));

					//adjust for minimum
					if (s_Cur_Motor_State[ui_Motor_Index].Num_Leftover < MIN_TICK_INCREMENT){
						s_Cur_Motor_State[ui_Motor_Index].Num_Leftover = MIN_TICK_INCREMENT;
					}
				}
				else if((s_Cur_Motor_State[ui_Motor_Index].Has_DEC) &&
						( s_Cur_Motor_State[ui_Motor_Index].Step_Count >=  s_Cur_Motor_State[ui_Motor_Index].DEC_Start) &&
						(--s_Cur_Motor_State[ui_Motor_Index].DEC_Period_Count == 0)){
					//reset period count
					s_Cur_Motor_State[ui_Motor_Index].DEC_Period_Count = s_Cur_Motor_State[ui_Motor_Index].DEC_Period;

					//get starting ticks
					s_Cur_Motor_State[ui_Motor_Index].Speed -= s_Cur_Motor_State[ui_Motor_Index].DEC_Rate;

					s_Cur_Motor_State[ui_Motor_Index].Tick_Total = (unsigned long) (ADJ_CLOCK_FREQ/(4*s_Cur_Motor_State[ui_Motor_Index].Speed));

					//get starting overflows
					s_Cur_Motor_State[ui_Motor_Index].Num_Overflows = ((unsigned int) ((s_Cur_Motor_State[ui_Motor_Index].Tick_Total)/ 0x10000UL));

					//get starting leftover
					s_Cur_Motor_State[ui_Motor_Index].Num_Leftover = ((unsigned int) ((s_Cur_Motor_State[ui_Motor_Index].Tick_Total) % 0x10000UL));

					//adjust for minimum
					if (s_Cur_Motor_State[ui_Motor_Index].Num_Leftover < MIN_TICK_INCREMENT){
						s_Cur_Motor_State[ui_Motor_Index].Num_Leftover = MIN_TICK_INCREMENT;
					}
				}

				//reset the overflow count
				s_Cur_Motor_State[ui_Motor_Index].Overflows_Remaining = s_Cur_Motor_State[ui_Motor_Index].Num_Overflows;

				//update counter
				if(ui_Motor_Index == LEFT){
					TA0CCR1 += s_Cur_Motor_State[ui_Motor_Index].Num_Leftover;
				}
				else{
					TA0CCR2 += s_Cur_Motor_State[ui_Motor_Index].Num_Leftover;
				}
			}
			else{

				if(s_Cur_Motor_State[ui_Motor_Index].Is_Concurrent){
					//turn off both interrupts
					TA0CCTL1 &= ~CCIE;
					TA0CCTL2 &= ~CCIE;
					//reset motor struct
					Clear_State(BOTH_MOTORS);
				}
				else{
					//reset motor struct
					Clear_State(ui_Motor_Index + 1);
					//turn off interrupts
					if(ui_Motor_Index == LEFT){
						TA0CCTL1 &= ~CCIE;
					}
					else{
						TA0CCTL2 &= ~CCIE;
					}
				}


				//check here for exit condition
				//stop interrupts if no motors are running
				if ((s_Cur_Motor_State[RIGHT].Is_Running == false) && (s_Cur_Motor_State[LEFT].Is_Running == false)){
					__bic_SR_register_on_exit(CPUOFF);
				}
			}
		}
		else{
			s_Cur_Motor_State[ui_Motor_Index].Overflows_Remaining--;
		}
	}
}
