
//======================Includes========================================
#include "main.h"
#include <modbus_reg.h>
#include <modbus_hard.h>
#include <optic_current_data.h>

#include "clock.h"

#include <stdbool.h>
#include "inttypes.h"
#include "system_stm32f1xx.h"
#include "stm32f1xx.h"
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"
#include "limits.h"
#include "IO.h"
#include <string.h>
#include "stm32f1xx_hal.h"
#include "usb_device.h"

extern uint16_t MBbuf_main[];
bool X_DI[MAX_DI];

void vRead_DI (void *pvParameters)
{
    uint32_t i = 0;
    bool DI_Check[MAX_DI];
    bool Previous_State[MAX_DI];
    bool Last_State[MAX_DI];

    MBbuf_main[(REG_DI_Bit)]=0;
    for(i = 0; i<MAX_DI; i++)
    {
        X_DI[i] = IO_GetLineActive(IO_DI_1+i);
        Previous_State[i] = X_DI[i];
        Last_State[i] = X_DI[i];
        DI_Check[i] = X_DI[i];
        if (!X_DI[i])
            MBbuf_main[REG_DI_Bit] &= ~(1<<i);
        else
            MBbuf_main[REG_DI_Bit] |=  X_DI[i]<<i;
    }
    vTaskDelay(20/portTICK_RATE_MS);
    while(1)
    {
        for(i = 0; i<MAX_DI; i++)
        {
            Previous_State[i] = Last_State[i];
            Last_State[i] = IO_GetLineActive(IO_DI_1+i);
            if (Last_State[i] == Previous_State[i])
            {
                if(DI_Check[i] != Last_State[i])
                {
                    X_DI[i] = Last_State[i];
                    if (!X_DI[i])
                        MBbuf_main[REG_DI_Bit] &= ~(1<<i);
                    else
                        MBbuf_main[REG_DI_Bit] |=  X_DI[i]<<i;
                    DI_Check[i] = X_DI[i];
                }
            }
        }
        vTaskDelay(71/portTICK_RATE_MS);
    }
}

void vBlinker (void *pvParameters)
{
    while(1)
    {
        IO_InvertLine(IO_LED_STATUS);
        vTaskDelay(TIME_BLINK_ON_MS/portTICK_RATE_MS);

    }
}
/*
**===========================================================================
**
**  Abstract: main program
**
**===========================================================================
*/
int main(void)
{
    ClockInit();
    IO_Init();
#ifndef DEBUG_TARGET
    flash_btock();
#endif
    mh_Buf_Init();
    MX_USB_DEVICE_Init();
#ifndef DEBUG_TARGET
    Init_IWDG(WATCH_DOG_TIME_MS);
#endif
    /*	task	*/
    if(pdTRUE != xTaskCreate(vBlinker,	"Blinker", 	configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL)) ERROR_ACTION(TASK_NOT_CREATE,0);
    if(pdTRUE != xTaskCreate(vRead_DI,	"DI", 		configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL)) ERROR_ACTION(TASK_NOT_CREATE,0);
    mh_Modbus_Init();   //create task for modbus
    oc_optic_current_init();   //create task for optic and current
    /*	start OS	*/
#ifdef DEBUG_TARGET
    printf ( "[ INFO ] Program start now\n" );
#endif
    vTaskStartScheduler();
    return 0;
}

void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}

void vApplicationMallocFailedHook( void )
{
    for( ;; );
}

void vApplicationStackOverflowHook( TaskHandle_t xTask, char * pcTaskName )
{
    ( void ) pcTaskName;
    ( void ) xTask;
    for( ;; );
}

void vApplicationTickHook( void )
{
}

void vApplicationIdleHook( void )
{
}
