
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
bool X_DI[DI_NUM_CHANNELS];

TaskHandle_t RegularTask_Handler;

void vRead_DI (void *pvParameters)
{
    uint32_t i = 0;
    bool DI_Check[DI_NUM_CHANNELS];
    bool Previous_State[DI_NUM_CHANNELS];
    bool Last_State[DI_NUM_CHANNELS];

    MBbuf_main[(REG_DI_STATUS)]=0;
    for(i = 0; i<DI_NUM_CHANNELS; i++)
    {
        X_DI[i] = IO_GetLineActive(IO_DI_1+i);
        Previous_State[i] = X_DI[i];
        Last_State[i] = X_DI[i];
        DI_Check[i] = X_DI[i];
        if (!X_DI[i])
            MBbuf_main[REG_DI_STATUS] &= ~(1<<i);
        else
            MBbuf_main[REG_DI_STATUS] |=  X_DI[i]<<i;
    }
    vTaskDelay(20/portTICK_RATE_MS);
    while(1)
    {
        for(i = 0; i<DI_NUM_CHANNELS; i++)
        {
            Previous_State[i] = Last_State[i];
            Last_State[i] = IO_GetLineActive(IO_DI_1+i);
            if (Last_State[i] == Previous_State[i])
            {
                if(DI_Check[i] != Last_State[i])
                {
                    X_DI[i] = Last_State[i];
                    if (!X_DI[i])
                        MBbuf_main[REG_DI_STATUS] &= ~(1<<i);
                    else
                        MBbuf_main[REG_DI_STATUS] |=  X_DI[i]<<i;
                    DI_Check[i] = X_DI[i];
                }
            }
        }
        vTaskDelay(71/portTICK_RATE_MS);
    }
}

void vRegularTask (void *pvParameters)
{
    uint32_t ulNotifiedValue, Mode_DO[DO_NUM_CHANNELS], alarm_mask, invert_do_on_state;
    BaseType_t notify_ret;
    uint16_t Temp_Mask=0;
    MBbuf_main[REG_DO_STATUS]=0;
    MBbuf_main[REG_DO_ON]=0;
    MBbuf_main[REG_DO_OFF]=0;
    memset(&Mode_DO[0], 0, sizeof(Mode_DO));
    (void) notify_ret;
    while(1)
    {
        // blink led
        IO_InvertLine(IO_LED_STATUS);
        //alarm notify for fast switch
        notify_ret = xTaskNotifyWait(0x00, ULONG_MAX, &ulNotifiedValue, TIME_BLINK_ON_MS/portTICK_RATE_MS);

        for (uint32_t i = 0; i < DO_NUM_CHANNELS; i++)
        {
            // reset if change output mode
            if(Mode_DO[i] != MBbuf_main[REG_DO_1_MODE+i])
            {
                Mode_DO[i] = MBbuf_main[REG_DO_1_MODE+i];
                IO_SetLine((IO_DO_1+i), LOW);
                MBbuf_main[REG_DO_STATUS] &= ~(1<<i);
                MBbuf_main[REG_DO_ON] = 0;
                MBbuf_main[REG_DO_OFF] = 0;
            }
            // mode 0 -> modbus control
            switch (Mode_DO[i])
            {
            case DO_MODE_MODBUS:
                if(MBbuf_main[REG_DO_ON] || MBbuf_main[REG_DO_OFF])
                {
                    Temp_Mask = MBbuf_main[REG_DO_ON] & (~MBbuf_main[REG_DO_OFF]);
                    if ((Temp_Mask>>i) & 1)
                        IO_SetLine((IO_DO_1+i), HIGH);
                    if ((MBbuf_main[REG_DO_OFF]>>i) & 1)
                        IO_SetLine((IO_DO_1+i), LOW);
                }
                break;
            case DO_MODE_ALARM:
                alarm_mask = 0;
                // set
                if ((MBbuf_main[REG_DO_1_ALARM_MSK+i] & DO_ALARM_I_MAX) &&\
                        (MBbuf_main[REG_TRIG_ALARM] & (PR_ALARM_CUR_MAX_P_MSK | PR_ALARM_CUR_MAX_N_MSK)))
                {
                    alarm_mask |= ACTION_DO_ON;
                }
                else if ((MBbuf_main[REG_DO_1_ALARM_MSK+i] & DO_ALARM_I_OL) &&\
                         (MBbuf_main[REG_TRIG_ALARM] & (PR_ALARM_CUR_OL_P_MSK | PR_ALARM_CUR_OL_N_MSK)))
                {
                    alarm_mask |= ACTION_DO_ON;
                }
                else if ((MBbuf_main[REG_DO_1_ALARM_MSK+i] & DO_ALARM_I_DIDT) &&\
                         (MBbuf_main[REG_TRIG_ALARM] & (PR_ALARM_CUR_DIDT_P_MSK | PR_ALARM_CUR_DIDT_N_MSK)))
                {
                    alarm_mask |= ACTION_DO_ON;
                }
                else if ((MBbuf_main[REG_DO_1_ALARM_MSK+i] & DO_ALARM_U_MAX_MIN) &&\
                         (MBbuf_main[REG_TRIG_ALARM] & (PR_ALARM_VOLT_MAX_MSK | PR_ALARM_VOLT_MIN_MSK)))
                {
                    alarm_mask |= ACTION_DO_ON;
                }
                else if ((MBbuf_main[REG_DO_1_ALARM_MSK+i] & DO_ALARM_MEASURE_LAN_ERROR) &&\
                         (MBbuf_main[REG_TRIG_ALARM] & (PR_ALARM_LAN_ERROR_MSK)))
                {
                    alarm_mask |= ACTION_DO_ON;
                }
                else if ((MBbuf_main[REG_DO_1_ALARM_MSK+i] & DO_ALARM_EXTERNAL_ALARM) &&\
                         (MBbuf_main[REG_TRIG_ALARM] & (PR_ALARM_MANUAL_MSK)))
                {
                    alarm_mask |= ACTION_DO_ON;
                }
                //reset
                else
                {
                    alarm_mask |= ACTION_DO_OFF;
                }
                // DO set
                if(MBbuf_main[REG_DO_1_ALARM_MSK+i] & DO_ALARM_INVERT_STATE)
                    invert_do_on_state = 0;
                else
                    invert_do_on_state = 1;
                //
                if(alarm_mask & ACTION_DO_ON)
                {
                    IO_SetLineActiveExtern((IO_DO_1+i), HIGH, invert_do_on_state);
                    MBbuf_main[REG_DO_STATUS] |= (1<<i);
                }
                else if(alarm_mask & ACTION_DO_OFF)
                {
                    IO_SetLineActiveExtern((IO_DO_1+i), LOW, invert_do_on_state);
                    MBbuf_main[REG_DO_STATUS] &= ~(1<<i);
                }
                break;
            default:
                break;
            }
            if(IO_GetLineActive(IO_DO_1+i))	MBbuf_main[REG_DO_STATUS] |= 1<<i;
            else MBbuf_main[REG_DO_STATUS] &= ~(1<<i);
        }
        MBbuf_main[REG_DO_OFF] = 0;
        MBbuf_main[REG_DO_ON] = 0;

        if (MBbuf_main[REG_DATA_SAVE])
        {
            mh_update_all_eeprom();
            MBbuf_main[REG_DATA_SAVE] = 0;
        }
    }
}

bool get_io(uint32_t number)
{
    return X_DI[number];
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
    if(pdTRUE != xTaskCreate(vRegularTask, "Regular", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, &RegularTask_Handler)) ERROR_ACTION(TASK_NOT_CREATE,0);
    if(pdTRUE != xTaskCreate(vRead_DI, "DI", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL)) ERROR_ACTION(TASK_NOT_CREATE,0);
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
