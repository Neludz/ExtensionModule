#ifndef MAIN_H_INCLUDED
#define MAIN_H_INCLUDED


//#include "stm32f1xx_ll_conf.h"

#include "FreeRTOS.h"
#include "timers.h"
#include "clock.h"

#define VERSION_REB			    201

#define WATCH_DOG_TIME_MS		500
#define MAX_DI  				4
#define MAX_DO  				4


#define RESET_VALUE					0xA01	//2561
#define FACTORY_SET_VALUE 			0xB01	//2817

// blink time
#define TIME_BLINK_ON_MS            250

#define pd_US_TO_TICK(x)  (x* (configTICK_RATE_HZ/ 1000LL))

void vBlinker (void *pvParameters);
void Init_IWDG(uint16_t tw);
void IWDG_res(void);
void Error_Handler(void);
//uint32_t Main_Timer_Set(const uint32_t AddTimeMs);
//bool Timer_Is_Expired (const uint32_t Timer);

//=========================================================================

// Отдадочная затычка. Сюда можно вписать код обработки ошибок.
#define	ERROR_ACTION(CODE,POS)		do{}while(1)



#endif /* MAIN_H_INCLUDED */
