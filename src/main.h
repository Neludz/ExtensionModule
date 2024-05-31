#ifndef MAIN_H_INCLUDED
#define MAIN_H_INCLUDED


//#include "stm32f1xx_ll_conf.h"

#include "FreeRTOS.h"
#include "timers.h"
#include "clock.h"
#include <stdbool.h>

#define VERSION_REB			    201

#define WATCH_DOG_TIME_MS		500
#define DI_NUM_CHANNELS  				4
#define DO_NUM_CHANNELS  				4

#define RESET_VALUE					0xA01	//2561
#define FACTORY_SET_VALUE 			0xB01	//2817

// blink time
#define TIME_BLINK_ON_MS            250

// DI_Mode
 #define  DI_MODE_OFF                   0
 #define  DI_MODE_RESET_ALARM           1
 #define  DI_MODE_STARTREK              2
 #define  DI_MODE_EXTERNAL_ALARM        3

// DO_Mode
 #define  DO_MODE_OFF                   0
 #define  DO_MODE_MODBUS                1
 #define  DO_MODE_ALARM                 2

// DO_MODE_ALARM option
#define DO_ALARM_INVERT_STATE           1  //0
#define DO_ALARM_I_MAX                  2  //1
#define DO_ALARM_I_OL                   4  //2
#define DO_ALARM_I_DIDT                 8  //3
#define DO_ALARM_U_MAX_MIN              16 //4
#define DO_ALARM_MEASURE_LAN_ERROR      32 //5
#define DO_ALARM_EXTERNAL_ALARM         64 //6

typedef enum //bit_mask
{
	ACTION_DO_NOT_USE = 1,
	ACTION_DO_ON      = 2,
    ACTION_DO_OFF     = 4,
} do_action_t;

bool get_io(uint32_t number);
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
