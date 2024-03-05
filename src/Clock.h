#ifndef CLOCK_H
#define CLOCK_H


#define USE_HSE
//#define USE_HSE_BYPASS
#define HSE_VALUE             8000000U
//#define USE_HSI
//#define USE_HSI_4
#define USE_PLL
#define SYSCLK_FREQ             72000000U

void ClockInit(void);

#endif
