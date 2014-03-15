/*
 * Communications_Functions.c
 *
 *  Created on: Oct 20, 2013
 *      Author: Greg
 *
 * Below info Updated: February 24th 2014
 *
 * The goal of these functions is to encapsulate all functions and interrupts used to communicate with the
 * computer via serial protocol.  These functions provide the ability to transmit unsigned ints and strings
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
#include "cstbool.h"
#include "Bit_Definitions.h"


//**********************************************************************************************************||
//Function Prototypes
//**********************************************************************************************************||
void Print_String(char *);
unsigned int strlen(const char *);
char * UINT_TO_STRING(uint16_t i);
char * ULONG_TO_STRING(uint32_t i);
void Print_UINT(uint16_t Value);
void Shutdown_Comms(void);


//**********************************************************************************************************||
//Compile time Constants
//**********************************************************************************************************||
//this constant is from documentation (tabled value given other parameters)
#define UART_DIVISOR (UINT16_C(1666))
//delay between prints to prevent errors.  with no delays errors occur more frequently
#define PRINT_DELAY (500000)


//**********************************************************************************************************||
//Variables
//**********************************************************************************************************||
//Mem: ~10 Bytes
//**********************************************************************************************************||
char *Output_String;
volatile bool vgb_Transmit_Complete = false;
uint16_t i = 0;
bool gb_Comms_Setup = false;


//**********************************************************************************************************||
//Functions
//**********************************************************************************************************||
//initializes the serial registers
void Setup_Comms(void)
{
	//set up the transmit bit
	P1DIR |= BIT_TXD;
	P1OUT &= ~BIT_TXD;
	P1SEL = BIT_TXD;
	P1SEL2 = BIT_TXD;

	//set up the specific registers
	//set clock to smclk
	UCA0CTL1 |= UCSSEL_2;
	//last 8 bits of 1666
	UCA0BR0 = UART_DIVISOR;
	//first 8 bits of
	UCA0BR1 = UART_DIVISOR >> 8;
	//Modulation UCBRSx = 6
	UCA0MCTL =  UCBRS_6;
	//Initialize USCI state machine
	UCA0CTL1 &= ~UCSWRST;
}

//shutdown the transmission by setting the following bit
void Shutdown_Comms(void){
	UCA0CTL1 |= UCSWRST;
}


//uses built in functions to simply print the name of the input result
void Output_Result(uint8_t *Liquid_Type,
					uint16_t *Cond_Value,
					uint16_t *Light_Value){

	//output raw numbers
	Print_String("Actual Light Reading: ");

	Print_UINT(*Light_Value);

	Print_String("\r\n");


	Print_String("Actual Cond. Reading: ");

	Print_UINT(*Cond_Value);

	Print_String("\r\n");

	//this is a simple function to print the liquid type out (not the best way of doing it but it works)
	Print_String("Liquid type: ");

	switch(*Liquid_Type){
	case LT_APPLE_JUICE:
		Print_String("Apple Juice");
		break;
	case LT_COKE:
		Print_String("Coke");
		break;
	case LT_DISTILLED_WATER:
		Print_String("Distilled Water");
		break;
	case LT_MALT_VINEGAR:
		Print_String("Malt Vinegar");
		break;
	case LT_MINERAL_OIL:
		Print_String("Mineral Oil");
		break;
	case LT_ORANGE_JUICE:
		Print_String("Orange Juice");
		break;
	case LT_SALT_WATER:
		Print_String("Salt Water");
		break;
	case LT_SUGAR_WATER:
		Print_String("Sugar Water");
		break;
	case LT_VEGETABLE_OIL:
		Print_String("Vegetable Oil");
		break;
	case LT_WHITE_VINEGAR:
		Print_String("White Vinegar");
		break;
	default:
		Print_String("ERROR 001-Liquid type number unknown.");
		break;
	}

	//print next line
	Print_String("\r\n\r\n");
}

void Print_UINT(uint16_t Value){

  //convert and use print string
  Print_String(UINT_TO_STRING(Value));
}

void Print_ULONG(uint32_t Value){

  //convert and use print string
  Print_String(ULONG_TO_STRING(Value));
}

void Print_String(char *Output){
	//setup communication module (always do this just in case)
	if(!gb_Comms_Setup){
	  Setup_Comms();
	  gb_Comms_Setup = true;
	}

	//always delay for things to settle
	__delay_cycles(PRINT_DELAY);


  //transfer flag initialization
  vgb_Transmit_Complete = false;
  //assign to global var
  Output_String = Output;  
  //send first char
  i=0;
  //enable interrupts
  __enable_interrupt();
  UCA0TXBUF = Output_String[i++];
  IE2 |= UCA0TXIE;                                      // Enable USCI_A0 TX interrupt
  
  while(vgb_Transmit_Complete == false){
  }

  //turn off
  //Shutdown_Comms();
}

//return length of a string (axiom)
unsigned int strlen(const char *str)
{
    const char *s;
    for (s = str; *s; ++s);
    return (s - str);
}

//convert uint to string (axiom)
char * UINT_TO_STRING(uint16_t i)
{
  static char buf[11];
  char* bp = buf+sizeof(buf);
  *--bp = '\0';
  do{
    *--bp = i%10+'0';
  }while (i /= 10);
  
  return bp;
}

//convert ulong to string (axiom)
char * ULONG_TO_STRING(uint32_t i)
{
  static char buf[21];
  char* bp = buf+sizeof(buf);
  *--bp = '\0';
  do{
    *--bp = i%10+'0';
  }while (i /= 10);

  return bp;
}


//**********************************************************************************************************||
//Interrupts
//**********************************************************************************************************||
//Transmit ISR
#pragma vector=USCIAB0TX_VECTOR
__interrupt void USCI0TX_ISR(void)
{
   UCA0TXBUF = Output_String[i++]; 
  
  if (i > strlen(Output_String)){                       // TX over?
    IE2 &= ~UCA0TXIE;                                   // Disable USCI_A0 TX interrupt
    vgb_Transmit_Complete = true;
  }
}

