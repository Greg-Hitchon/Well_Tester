/*
 * Nav_Functions.c
 *
 *  Created on: Oct 16, 2013
 *      Author: Greg
 *
 * Below info Updated: February 24th 2014
 *
 * This file provides an easy method for running the motors
 * To run a motor the Set_Motor function then the Start_Motor function needs to be called
 * This is simplified by the "Straight" and "Turn" functions
 *
 * This function makes use of timer0 and the CCR0 interrupt and contains the
 * corresponding interrupt handling routine
 *
 * P2 is used for the motor output pins
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
#include "Bit_Definitions.h"
#include "Extraction_Functions.h"
#include "Measure_Functions.h"
#include "Communication_Functions.h"
#include "cstbool.h"

//**********************************************************************************************************||
//Function Prototypes
//**********************************************************************************************************||
void Set_Motor(	uint8_t Motor_ID,
				uint8_t Direction,
				uint32_t Steps,
				uint8_t Profile_ID);

void Execute_Cross(uint8_t Starting_Side,
					uint8_t Straight_ID,
					uint8_t Turn_ID,
					uint8_t Num_Crosses);

void Re_Orient(uint8_t Direction,
			uint8_t Profile_ID);

void Start_Motor(uint8_t Motor_ID);
void Hold_Until_Finished(void);
void Update_State(uint8_t Motor_ID);
void Go_Home(void);
void Update_XY_Coords(uint32_t Steps, uint8_t Direction);
void Clear_State(uint8_t Motor_ID);


//**********************************************************************************************************||
//Compile time Constants
//**********************************************************************************************************||
//cant have too many nav profiles as memory considerations happen fast
#define NUM_NAV_PROFILES		(3)
//this has to be high enough that we dont miss the next step.  small enough not to make driving rough
#define MIN_TICK_INCREMENT		(1000)
//using half step so the phase count is 8
#define NUM_STATES				(8)
#define NUM_MOTORS				(2)
//this is the number of steps used to make sure we dont run into the wall in the go_home algo
#define STEPS_Y_ADJUST			(150)
//delay between movement in the movement algos
#define DELAY_BETWEEN 			(5000)


//secondary (calculated) values
#define ADJ_CLOCK_FREQ			((UINT32_C(CLOCK_FREQ))*UINT32_C(12500))
//this should be equal to STEPS_PER_SWEEP/PI
#define STEPS_XY_PER_SWEEP		(230)
//this is the number of steps to move forward when running into the wall to straighten up.  try to keep small as possible
#define STEPS_TO_WALL_RUN		(500)
//this is the steps to backup from the wall running in order to be able to execute a dime turn
#define STEPS_TO_BACK_UP_FIRST	(200)
//this is the number of steps to backup to center the robot in the sensing area
#define STEPS_TO_BACK_UP_SECOND	(300)


//**********************************************************************************************************||
//Other Constants
//**********************************************************************************************************||
//This configuration implies:
//Motor 1: a=Bit0, a'=Bit2; b=Bit1, b'=Bit3
//Motor 2: a=Bit4, a'=Bit6; b=Bit5, b'=Bit7
const uint16_t cau16_Orientation[4]={NORTH,EAST,SOUTH,WEST};
const uint8_t cch_State_Map[NUM_MOTORS][NUM_STATES] = {{BIT0,BIT3,BIT1,BIT0,BIT2,BIT1,BIT3,BIT2},{BIT4,BIT7,BIT5,BIT4,BIT6,BIT5,BIT7,BIT6}};



//**********************************************************************************************************||
//Structs
//**********************************************************************************************************||
//Total mem: 2*50+2*30 + 10 = ~200 bytes.  less than half available used, so functions etc will be okay unless declaring large arrays/tons of vars
//**********************************************************************************************************||

//contains info for each motor when given a nav profile as well as a specific step count, number in memory is 2, one for each motor
//Mem: ~50Bytes
struct Motor_State {
	uint32_t Step_Target, Step_Count, Tick_Total;
    uint16_t Overflows_Remaining, Num_Leftover, Num_Overflows, Speed, ACC_End, DEC_Start, ACC_Rate, DEC_Rate, ACC_Period, DEC_Period, ACC_Period_Count, DEC_Period_Count;
    uint8_t Direction,Profile_ID;
	bool Is_Running, Has_ACC, Has_DEC, Is_Concurrent;
};

//contains info about the acc/dec profiles and speed, number is controlled by global definition
//Mem: ~30Bytes
struct Nav_Profile{
	uint32_t Period;
	uint16_t Num_Overflows, Num_Leftover, ACC_Rate, DEC_Rate, ACC_Period, DEC_Period, ACC_Steps, DEC_Steps,
				Start_Speed, Target_Speed, End_Speed;
	bool Has_DEC, Has_ACC;
};

//contains info used to track position of the robot, not necessary as only 1 in memory but makes code clearer
//mem: ~10 bytes
struct Track_Info{
	//X is an axis along the line connecting the sensing area and the starting area, Y is perpindicular to the X axis intersect at leftmost point of X axis, Z is the rotational steps
	uint16_t X_Steps, Y_Steps, Z_Steps;
	uint8_t Orientation_Index, Turn_Type, Turn_Direction, Profile_ID;
	bool Is_Turning;
};

//inititialize the declared structss
struct Motor_State s_Cur_Motor_State[NUM_MOTORS];
struct Nav_Profile s_Nav_Profiles[NUM_NAV_PROFILES];
struct Track_Info s_Track_Info;


//**********************************************************************************************************||
//Variables
//**********************************************************************************************************||
//Mem: ~10 Bytes
//**********************************************************************************************************||

uint16_t caui_Last_State[2] ={NUM_STATES+1};
uint8_t caui_State_Direction[2]={FORWARD};

bool cub_Cup_Found = false, cub_Int_While_Turning = false;

//**********************************************************************************************************||
//Functions
//**********************************************************************************************************||
//These are the basic functions that allow for motor driving and navigation algorithm
//**********************************************************************************************************||

//origin is at finish line with x axis across to start
//north is along positive y axis, east along positive x axis
void Initialize_Tracking(void){
	//set initial x and y coords
	s_Track_Info.X_Steps = TABLE_WIDTH_STEPS + 2*STEPS_XY_PER_SWEEP;
	s_Track_Info.Y_Steps = 0;

	//set initial orientation
	s_Track_Info.Orientation_Index = 0;
}

//sets the state to 0 state
void Initialize_Bits(void){
	//set last state to 0
	caui_Last_State[LEFT]=0;
	caui_Last_State[RIGHT]=0;

	//update motor pins
	P2DIR |= 0xFF;
	P2SEL = 0x0;

	//for each motor we need to set bits to the initial 0 state
	P2OUT = BIT0 | BIT3 | BIT4 | BIT7;
}

//this dictates the target steps, speed etc.  This is used as a template to construct the navigation
//parameters within a specific motor struct.  The values will change dependent on the step count
void Create_Nav_Profile(uint8_t Profile_ID,
						uint16_t Start_Speed,
						uint16_t Target_Speed,
						uint16_t End_Speed,
						uint16_t ACC_Rate,
						uint16_t DEC_Rate,
						uint16_t ACC_Period,
						uint16_t DEC_Period){

	//check if there is a valid profile id
	if (Profile_ID < NUM_NAV_PROFILES){
		//get starting ticks
		s_Nav_Profiles[Profile_ID].Period = UINT16_C((ADJ_CLOCK_FREQ/(4*Start_Speed)));

		//get starting overflows
		s_Nav_Profiles[Profile_ID].Num_Overflows = (UINT16_C((s_Nav_Profiles[Profile_ID].Period)/ 0x10000UL));

		//get starting leftover
		s_Nav_Profiles[Profile_ID].Num_Leftover = (UINT16_C((s_Nav_Profiles[Profile_ID].Period) % 0x10000UL));

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
  TA1CTL = TASSEL_2 | MC_2 | ID_3;
}


//this function sets up the struct used to define running parameters
void Set_Motor(	uint8_t Motor_ID,
				uint8_t Direction,
				uint32_t Steps,
				uint8_t Profile_ID){

	uint32_t ui_Temp_Target_Speed = 0;
	uint8_t ui_Motor_Index;

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


void Clear_State(uint8_t Motor_ID){
	static const struct Motor_State Empty_Struct = {0};

	if (Motor_ID & LEFT_MOTOR){
		s_Cur_Motor_State[LEFT] = Empty_Struct;
	}

	if (Motor_ID & RIGHT_MOTOR){
		s_Cur_Motor_State[RIGHT] = Empty_Struct;
	}
}

//updates bits, takes either motor or both
void Update_State(uint8_t Motor_Index){
	uint8_t Bit_Total = 0, i, Motor_ID = 0;
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
void Start_Motor(uint8_t Motor_ID){
	//check for concurrent here
	if ((Motor_ID & CONC_MOTOR) && (s_Cur_Motor_State[CONC_MOTOR-1].Is_Concurrent)){
		Motor_ID = CONC_MOTOR;
	}

	//do this initialization every time just in case
	Set_Timer();

	//here we set the bit states specified by the "settings" struct
	if (Motor_ID & LEFT_MOTOR){
		TA1CCR1 = s_Cur_Motor_State[LEFT].Num_Leftover;
		//start interrupts
		TA1CCTL1 |= CCIE;
	}

	if (Motor_ID & RIGHT_MOTOR){
		TA1CCR2 = s_Cur_Motor_State[RIGHT].Num_Leftover;
		//start interrupts
		TA1CCTL2 |= CCIE;
	}
}

//this can be used easily to put the uc into a low power mode waiting for a wakeup call
void Hold_Until_Finished(void){
	//this sets the cpuoff bit as well as interrupts added
  __bis_SR_register(CPUOFF + GIE); 
}

//**********************************************************************************************************||
//Composite Functions
//**********************************************************************************************************||
//these functions just package the basic functions into easier to use forms with common constants supplied
//for typical behaviour (turn (left/right), straight (backwards/forwards)
//NOTE: the coords are updated everytime for a full turn...this is because if interrupted the turn is still finished
//with the remaining steps
//**********************************************************************************************************||

void Turn(	uint8_t Direction,
			uint8_t Profile_ID,
			uint8_t Type,
			uint32_t Steps){

	//to reverse turn mid-turn we need the direction and type
	s_Track_Info.Is_Turning = true;
	s_Track_Info.Turn_Direction = Direction;
	s_Track_Info.Turn_Type = Type;
	s_Track_Info.Profile_ID = Profile_ID;


	//Actually do turn
	if(Type == SWEEP){
		//actually set motors
		if(Direction == LEFT){
			Set_Motor(RIGHT_MOTOR, FORWARD, Steps, Profile_ID);
			Start_Motor(RIGHT_MOTOR);
			Hold_Until_Finished();
		}
		else{
			Set_Motor(LEFT_MOTOR, FORWARD, Steps, Profile_ID);
			Start_Motor(LEFT_MOTOR);
			Hold_Until_Finished();
		}

		//For the sweep we need to update the Y direction and X direction, this does one (could be either x or y)
		Update_XY_Coords(STEPS_XY_PER_SWEEP, FORWARD);
	}
	else{
		//actually set motors
		if(Direction == LEFT){
			Set_Motor(BOTH_MOTORS,TURN_LEFT,Steps, Profile_ID);
			Start_Motor(BOTH_MOTORS);
			Hold_Until_Finished();
		}
		else{
			Set_Motor(BOTH_MOTORS,TURN_RIGHT,Steps, Profile_ID);
			Start_Motor(BOTH_MOTORS);
			Hold_Until_Finished();
		}
	}

	//adjust the orientation
	if(Direction == LEFT){
		if(s_Track_Info.Orientation_Index == 0){
			s_Track_Info.Orientation_Index = 3;
		}
		else{
			s_Track_Info.Orientation_Index--;
		}
	}
	else{
		if(s_Track_Info.Orientation_Index == 3){
			s_Track_Info.Orientation_Index = 0;
		}
		else{
			s_Track_Info.Orientation_Index++;
		}
	}

	//a little messy but if we are doing a sweep we need to update xy again after orientation update as it moves in both x and y
	if(Type == SWEEP){
		//this updates the other index (either x or y) after the orienteation has  been updated
		Update_XY_Coords(STEPS_XY_PER_SWEEP, FORWARD);
	}
}

void Straight(	uint8_t Direction,
				uint32_t Steps,
				uint8_t Profile_ID){
	//indicate not turning here so know to update xy not turn info
	s_Track_Info.Is_Turning = false;

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

	//update the coords
	Update_XY_Coords(Steps, Direction);
}

void Update_XY_Coords(uint32_t Steps,
						uint8_t Direction){

	if(Direction == FORWARD){
		//adjust the coords based on steps taken in current movement
		switch (cau16_Orientation[s_Track_Info.Orientation_Index]){
		case NORTH:
			s_Track_Info.Y_Steps += Steps;
			break;
		case EAST:
			s_Track_Info.X_Steps += Steps;
			break;
		case SOUTH:
			if(s_Track_Info.Y_Steps >= Steps){
				s_Track_Info.Y_Steps -= Steps;
			}
			else{
				s_Track_Info.Y_Steps = 0;
			}
			break;
		case WEST:
			if(s_Track_Info.X_Steps >= Steps){
				s_Track_Info.X_Steps -= Steps;
			}
			else{
				s_Track_Info.X_Steps = 0;
			}
		}
	}
	else if (Direction == BACKWARD){
		//adjust the coords based on steps taken in current movement
		switch (cau16_Orientation[s_Track_Info.Orientation_Index]){
		case NORTH:
			if(s_Track_Info.Y_Steps >= Steps){
				s_Track_Info.Y_Steps -= Steps;
			}
			else{
				s_Track_Info.Y_Steps = 0;
			}
			break;
		case EAST:
			if(s_Track_Info.X_Steps >= Steps){
				s_Track_Info.X_Steps -= Steps;
			}
			else{
				s_Track_Info.X_Steps = 0;
			}
			break;
		case SOUTH:
			s_Track_Info.Y_Steps += Steps;
			break;
		case WEST:
			s_Track_Info.X_Steps += Steps;
		}
	}
}

//this function just puts the final algorithm in a function
void Final_Run(void){
	const uint8_t i_Turn_Profile = 1, i_Straight_Profile = 0;
	//go straight up the east wall
	Straight(FORWARD,TABLE_LENGTH_STEPS,i_Straight_Profile);
	//execute 5 crosses
	Execute_Cross(EAST,i_Straight_Profile,i_Turn_Profile,5);
	//go home here if no cup is found
	Go_Home();
}

//this function executes n "crosses" across the board given a starting side (east or west)
//no global tracking of current side, can be determined on calling side by even/odd
void Execute_Cross(uint8_t Starting_Side,
					uint8_t Straight_ID,
					uint8_t Turn_ID,
					uint8_t Num_Crosses){
	uint8_t Side_Tracker, Cross_Tracker;

	//initialize starting side
	Side_Tracker = Starting_Side;

	//do the loop
	for(Cross_Tracker = Num_Crosses; Cross_Tracker > 0; Cross_Tracker--){
		//delay for determined period
		__delay_cycles(DELAY_BETWEEN);

		//orient across the board and toggle the sides
		if(Side_Tracker == EAST){
			Re_Orient(WEST,Turn_ID);
			Side_Tracker = WEST;
		}
		else{
			Re_Orient(EAST,Turn_ID);
			Side_Tracker = EAST;
		}

		//delay for determined period
		__delay_cycles(DELAY_BETWEEN);

		//go straight across the board
		Straight(FORWARD,TABLE_WIDTH_STEPS,Straight_ID);

		//delay for determined period
		__delay_cycles(DELAY_BETWEEN);

		//reorient to the south
		Re_Orient(SOUTH,Turn_ID);

		//delay for determined period
		__delay_cycles(DELAY_BETWEEN);

		//move down the grid steps
		Straight(FORWARD,GRID_STEPS,Straight_ID);
	}
}


//this function takes a direction as an input and truns the robot to that orientation using dime turns
void Re_Orient(uint8_t Direction,
			uint8_t Profile_ID){
	//actually do turns, here we always turn clockwise (right) except for case 3 where we turn couterclockwise (left)
	switch((Direction - cau16_Orientation[s_Track_Info.Orientation_Index])/64){
	case 1:
		//do one turn clockwise
		Turn(RIGHT,Profile_ID,DIME,STEPS_PER_DIME);
		break;
	case 2:
		//do one turn clockwise
		Turn(RIGHT,Profile_ID,DIME,STEPS_PER_DIME);

		//delay for determined period
		__delay_cycles(DELAY_BETWEEN);

		//do one turn clockwise
		Turn(RIGHT,Profile_ID,DIME,STEPS_PER_DIME);
		break;
	case 3:
		//do one turn counter-clockwise
		Turn(LEFT,Profile_ID,DIME,STEPS_PER_DIME);
		break;
	default:
		break;
	}
}

//this just goes hoem based on direction and coordinates
void Go_Home(void){
	uint8_t Liquid_Type = 0;

	//change to false to prevent recursion
	cub_Cup_Found = false;

	//because we need to have sufficient turning radius we need to decrement y by the turning radius
	if(s_Track_Info.Y_Steps>=STEPS_Y_ADJUST){
		s_Track_Info.Y_Steps -= STEPS_Y_ADJUST;
	}

	//if we were turning we need to complete the turn (could adjust to turn back if more efficient but leave it as just finishing the turn for now)
	if(cub_Int_While_Turning && s_Track_Info.Z_Steps > 0){
		Turn(s_Track_Info.Turn_Direction, s_Track_Info.Profile_ID, s_Track_Info.Turn_Type, s_Track_Info.Z_Steps);
	}


	//NOTE: Extra logic here to prevent crash against west side wall, the north and east walls are kept far enough away from to not be a concern
	//to accomplish this we will always turn to the west, and then make adjustments
	Re_Orient(WEST,0);

	//do x translation
	if(s_Track_Info.X_Steps > 0){
		Straight(FORWARD,s_Track_Info.X_Steps,0);
	}
	//make adjustment for straightness
	Straight(FORWARD,STEPS_TO_WALL_RUN,2);
	//back up from wall
	Straight(BACKWARD,STEPS_TO_BACK_UP_FIRST,2);

	//turn to the y direction
	Turn(LEFT,0,DIME,STEPS_PER_DIME);

	//do y translation
	if(s_Track_Info.Y_Steps > 0){
		//do y translation
		Straight(FORWARD, s_Track_Info.Y_Steps,0);
	}
	//make adjustment for straighness
	Straight(FORWARD,STEPS_TO_WALL_RUN,2);
	//back up from wall
	Straight(BACKWARD,STEPS_TO_BACK_UP_SECOND,2);


	//turn off motors
	P2OUT = 0;
	P2DIR = 0;

	//get result from sensing unit
	Liquid_Type = Get_Result();

	//print to computer
	for(;;){
		Output_Result(Liquid_Type);
	}
}


void Cup_Found(void){
	//updates flags and coords
	cub_Cup_Found = true;

	//update the tracking info with current movement so we can get home
	if(!s_Track_Info.Is_Turning){
		//update before going home
		if(s_Cur_Motor_State[CONC_MOTOR-1].Direction & LEFT_FORWARD){
			Update_XY_Coords(s_Cur_Motor_State[CONC_MOTOR-1].Step_Count,FORWARD);
		}
		else{
			Update_XY_Coords(s_Cur_Motor_State[CONC_MOTOR-1].Step_Count,BACKWARD);
		}
	}
	else{
		//here we just need the steps remaining, need to determine which motor has info.
		//if dime then can get from either, if sweep only one is running and has the step info
		if(s_Cur_Motor_State[LEFT].Is_Running){
			s_Track_Info.Z_Steps = s_Cur_Motor_State[LEFT].Step_Target - s_Cur_Motor_State[LEFT].Step_Count;
		}
		else{
			s_Track_Info.Z_Steps = s_Cur_Motor_State[RIGHT].Step_Target - s_Cur_Motor_State[RIGHT].Step_Count;
		}

		//update flag so we know to complete turn
		cub_Int_While_Turning = true;

	}

	//DO EXTRACTION ALGO (can move car here
	Extract_Liquid();

	//after adjustment we can just go home
	Go_Home();
}


//**********************************************************************************************************||
//Interrupts
//**********************************************************************************************************||
//this is where the 4 pulse width signals are output.  Each has the same period (so same speed) and each motor has two pulses (A and B)
//This means that there needs to be a Period/4 frequency of interrupts.
//**********************************************************************************************************||
//**********************************************************************************************************||


//all port 1 interrupts, controls startup and cup locating
#pragma vector=PORT1_VECTOR
__interrupt void PORT1_ISR(void){
	  //clear fgs
	  P1IFG = 0x0;
}


//flag is auto reset
#pragma vector=TIMER1_A0_VECTOR
__interrupt void TIMER1_CCRO_ISR(void){
	//interrupt flags are reset automatically
	__no_operation();
}

//clear flag here, also updates the motors

#pragma vector=TIMER1_A1_VECTOR
__interrupt void TIMER1_OTHER_ISR(void){
	//bit flag
	uint8_t ui_Motor_Index;
	bool b_Do_Update = false;

	//flags reset by reading ta1iv
	switch(__even_in_range(TA1IV,0xA)){
	case TA1IV_TACCR1:
		//flag update needed
		b_Do_Update = true;
		//assign index
		ui_Motor_Index = LEFT;

		break;
	case TA1IV_TACCR2:
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

					s_Cur_Motor_State[ui_Motor_Index].Tick_Total = UINT32_C(ADJ_CLOCK_FREQ/(4*s_Cur_Motor_State[ui_Motor_Index].Speed));

					//get starting overflows
					s_Cur_Motor_State[ui_Motor_Index].Num_Overflows = (UINT16_C((s_Cur_Motor_State[ui_Motor_Index].Tick_Total)/ 0x10000UL));

					//get starting leftover
					s_Cur_Motor_State[ui_Motor_Index].Num_Leftover = (UINT16_C((s_Cur_Motor_State[ui_Motor_Index].Tick_Total) % 0x10000UL));

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

					s_Cur_Motor_State[ui_Motor_Index].Tick_Total = UINT32_C(ADJ_CLOCK_FREQ/(4*s_Cur_Motor_State[ui_Motor_Index].Speed));

					//get starting overflows
					s_Cur_Motor_State[ui_Motor_Index].Num_Overflows = (UINT16_C((s_Cur_Motor_State[ui_Motor_Index].Tick_Total)/ 0x10000UL));

					//get starting leftover
					s_Cur_Motor_State[ui_Motor_Index].Num_Leftover = (UINT16_C((s_Cur_Motor_State[ui_Motor_Index].Tick_Total) % 0x10000UL));

					//adjust for minimum
					if (s_Cur_Motor_State[ui_Motor_Index].Num_Leftover < MIN_TICK_INCREMENT){
						s_Cur_Motor_State[ui_Motor_Index].Num_Leftover = MIN_TICK_INCREMENT;
					}
				}

				//reset the overflow count
				s_Cur_Motor_State[ui_Motor_Index].Overflows_Remaining = s_Cur_Motor_State[ui_Motor_Index].Num_Overflows;

				//update counter
				if(ui_Motor_Index == LEFT){
					TA1CCR1 += s_Cur_Motor_State[ui_Motor_Index].Num_Leftover;
				}
				else{
					TA1CCR2 += s_Cur_Motor_State[ui_Motor_Index].Num_Leftover;
				}
			}
			else{
				//if we get here current command is done so turn off necessary motors
				if(s_Cur_Motor_State[ui_Motor_Index].Is_Concurrent){
					//turn off both interrupts
					TA1CCTL1 &= ~CCIE;
					TA1CCTL2 &= ~CCIE;
					//reset motor struct
					Clear_State(BOTH_MOTORS);
				}
				else{
					//reset motor struct
					Clear_State(ui_Motor_Index + 1);
					//turn off interrupts
					if(ui_Motor_Index == LEFT){
						TA1CCTL1 &= ~CCIE;
					}
					else{
						TA1CCTL2 &= ~CCIE;
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
	//clear flag
	TA1CTL &= ~TAIFG;
}
