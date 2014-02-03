#include "Project_Parameters.h"
#include TEST_CHIP

#include "cstbool.h"
#include "Bit_Definitions.h"

#define UART_DIVISOR ((int) 1666)

void Print_String(char *);
unsigned int strlen(const char *);
char * UINT_TO_STRING(unsigned int i);
char * ULONG_TO_STRING(unsigned long i);


char *Output_String;
volatile bool vgb_Transmit_Complete = false;
unsigned int i = 0;

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


void Output_Result(void){
	__delay_cycles(16000000);
}
  //char *c_Tst1;
void Print_UINT(unsigned int Value){

  //c_Tst1 = UINT_TO_STRING(Value);
  //convert and use print string
  Print_String(UINT_TO_STRING(Value));
}

void Print_ULONG(unsigned long Value){

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

char * UINT_TO_STRING(unsigned int i)
{
  static char buf[11];
  char* bp = buf+sizeof(buf);
  *--bp = '\0';
  do{
    *--bp = i%10+'0';
  }while (i /= 10);
  
  return bp;
}

char * ULONG_TO_STRING(unsigned long i)
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

