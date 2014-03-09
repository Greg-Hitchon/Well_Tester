#include <stdint.h>

#ifndef COMMUNICATION_FUNCTIONS_H
#define COMMUNICATION_FUNCTIONS_H

void Output_Result(uint8_t *Liquid_Type,
					uint16_t *Cond_Value,
					uint16_t *Light_Value,
					uint16_t *Var_Value);
void Setup_Comms(void);
void Shutdown_Comms(void);
void Print_String(char *);
void Print_UINT(uint16_t Value);
void Print_ULONG(uint32_t Value);

#endif
