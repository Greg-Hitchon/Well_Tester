//System includes
#include "Project_Parameters.h"
#include TEST_CHIP
#include <stdint.h>

//User defined includes
#include "cstbool.h"
#include "Bit_Definitions.h"

#define UART_DIVISOR (UINT16_C(1666))
#define PRINT_DELAY (5000000)

void Print_String(char *);
unsigned int strlen(const char *);
char * UINT_TO_STRING(uint16_t i);
char * ULONG_TO_STRING(uint32_t i);


char *Output_String;
volatile bool vgb_Transmit_Complete = false;
uint16_t i = 0;

void Setup_Comms(void)
{
  P1DIR |= BIT_TXD;                                     // All P1.x outputs
  P1OUT &= ~BIT_TXD;                                    // All P1.x reset
  P1SEL = BIT_TXD;                                      // P1.1 = RXD, P1.2=TXD
  P1SEL2 = BIT_TXD;                                     // P1.1 = RXD, P1.2=TXD
  
  UCA0CTL1 |= UCSSEL_2;                                 // CLK = SMCLK
  UCA0BR0 = UART_DIVISOR;                               // last 8 bits of 1666
  UCA0BR1 = UART_DIVISOR >> 8;                          //first 8 bits of
  UCA0MCTL =  UCBRS_6;                                   // Modulation UCBRSx = 6
  UCA0CTL1 &= ~UCSWRST;                                 // **Initialize USCI state machine**
}


void Output_Result(uint8_t Liquid_Type){
	//this is a simple function to print the liquid type out (not the best way of doing it but it works)
	switch(Liquid_Type){
	case LT_APPLE_JUICE:
		Print_String("Apple Juice");
		__delay_cycles(PRINT_DELAY);
		Print_String("\r\n");
		break;
	case LT_COKE:
		Print_String("Coke");
		__delay_cycles(PRINT_DELAY);
		Print_String("\r\n");
		break;
	case LT_DISTILLED_WATER:
		Print_String("Distilled Water");
		__delay_cycles(PRINT_DELAY);
		Print_String("\r\n");
		break;
	case LT_MALT_VINEGAR:
		Print_String("Malt Vinegar");
		__delay_cycles(PRINT_DELAY);
		Print_String("\r\n");
		break;
	case LT_MINERAL_OIL:
		Print_String("Mineral Oil");
		__delay_cycles(PRINT_DELAY);
		Print_String("\r\n");
		break;
	case LT_ORANGE_JUICE:
		Print_String("Orange Juice");
		__delay_cycles(PRINT_DELAY);
		Print_String("\r\n");
		break;
	case LT_SALT_WATER:
		Print_String("Salt Water");
		__delay_cycles(PRINT_DELAY);
		Print_String("\r\n");
		break;
	case LT_SUGAR_WATER:
		Print_String("Sugar Water");
		__delay_cycles(PRINT_DELAY);
		Print_String("\r\n");
		break;
	case LT_VEGETABLE_OIL:
		Print_String("Vegetable Oil");
		__delay_cycles(PRINT_DELAY);
		Print_String("\r\n");
		break;
	case LT_WHITE_VINEGAR:
		Print_String("White Vinegar");
		__delay_cycles(PRINT_DELAY);
		Print_String("\r\n");
		break;
	default:
		break;
	}
}
  //char *c_Tst1;
void Print_UINT(uint16_t Value){

  //c_Tst1 = UINT_TO_STRING(Value);
  //convert and use print string
  Print_String(UINT_TO_STRING(Value));
}

void Print_ULONG(uint32_t Value){

  //c_Tst1 = UINT_TO_STRING(Value);
  //convert and use print string
  Print_String(ULONG_TO_STRING(Value));
}

void Print_String(char *Output){
  //setup communication module (always do this just in case)
  Setup_Comms();
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
}

unsigned int strlen(const char *str)
{
    const char *s;
    for (s = str; *s; ++s);
    return (s - str);
}

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

// USCI A0/B0 Transmit ISR
#pragma vector=USCIAB0TX_VECTOR
__interrupt void USCI0TX_ISR(void)
{
   UCA0TXBUF = Output_String[i++]; 
  
  if (i > strlen(Output_String)){                       // TX over?
    IE2 &= ~UCA0TXIE;                                   // Disable USCI_A0 TX interrupt
    vgb_Transmit_Complete = true;
  }
}

