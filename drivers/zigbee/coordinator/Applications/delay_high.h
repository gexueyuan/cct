#ifndef __STM32_DELAY_H__
 #define __STM32_DELAY_H__

 #ifdef __cplusplus
 extern "C" {
 #endif
   
 void __delay_loops(unsigned long int n);

 #ifdef __cplusplus
 }
 #endif

 #define _delay_loops(n)    do{if(n!=0){__delay_loops(n);}}while(0)
#define FREQ_CPU 72000000UL
 #if FREQ_CPU>48000000UL
 #define  _CYCLES_PER_LOOP  6
 #elif F_CPU>24000000UL
 #define  _CYCLES_PER_LOOP  4
 #else
 #define  _CYCLES_PER_LOOP  3
 #endif

 #define delay_us(A)\
   _delay_loops( (unsigned long int) (( (double)(F_CPU) *((A)/1000000.0))/_CYCLES_PER_LOOP+0.5))

 #define _delay_ms(A)\
   _delay_loops( (unsigned long int) (( (double)(F_CPU) *((A)/1000.0))/_CYCLES_PER_LOOP+0.5))

 #define _delay_s(A)\
   _delay_loops( (unsigned long int) (( (double)(F_CPU) *((A)/1.0))/_CYCLES_PER_LOOP+0.5))

 #endif