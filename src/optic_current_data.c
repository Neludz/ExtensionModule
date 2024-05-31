#include "main.h"
#include <modbus_reg.h>
#include <modbus_hard.h>

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
#include "optic_current_data.h"
#include <eeprom_emulation.h>

extern uint16_t MBbuf_main[];
uint32_t error=0, count_ok=0, value, value_2;
int16_t Vshunt_mid, Vshunt_inst, Vshunt_didt, V_inst, V_mid, clamped_i_inst;
int32_t I_mid, I_inst, I_didt;
uint32_t alarm_do;
const uint32_t Alarm_Timers[ALARM_NUMBER_TIMER] = ALARM_TIMERS_LIST;
xTimerHandle xAlarm_Timers [ALARM_NUMBER_TIMER], x_reset_alarm_timer;
extern TaskHandle_t RegularTask_Handler;

volatile OpticStruct_t optic_data =
{
    .state = OPTIC_STATE_IDLE,
    .events = OPTIC_EVENTS_NONE,
};
ProtectionStruct_t protection_data =
{
    .trig_alarm = PR_ALARM_NO_ALARM,
    .event = PROTECTION_EVENT_NO_EVENTS,
    .mb_save_index = 0,
};

static uint8_t dallas_crc8(uint32_t buf_size, uint8_t *buf);

void USART1_IRQHandler (void)
{
    uint8_t cnt;
    (void) cnt;
    if (USART1->SR & (USART_SR_FE | USART_SR_ORE | USART_SR_NE))
    {
        optic_data.state = OPTIC_STATE_IDLE;
        cnt = USART1->DR;
    }
    if ((USART1->SR & USART_SR_IDLE))
    {
        if (OPTIC_STATE_RCVE == optic_data.state)
        {
            // optic_data.state = OPTIC_STATE_PARSE;
            if((!dallas_crc8(11, (uint8_t*)optic_data.buf)) && optic_data.index == 11)
            {
                Vshunt_mid = optic_data.buf[0] | (optic_data.buf[1]<<8);
                Vshunt_inst = optic_data.buf[2] | (optic_data.buf[3]<<8);
                Vshunt_didt = (optic_data.buf[4] | (optic_data.buf[5]<<8));
                V_inst = optic_data.buf[6] | (optic_data.buf[7]<<8);
                V_mid = optic_data.buf[8] | (optic_data.buf[9]<<8);
                optic_data.events = OPTIC_EVENTS_UART_FIN;
                count_ok ++;
            }
            else
            {
                error++;
            }
        }
        optic_data.state = OPTIC_STATE_IDLE;
        cnt = USART1->DR;
    }
    else if (USART1->SR & USART_SR_RXNE)
    {
        if( OPTIC_STATE_RCVE == optic_data.state)
        {
            optic_data.buf[(optic_data.index++) & OPTIC_BUFFER_SIZE] = USART1->DR;	 // MAIN DOING: New byte to buffer
        }
        else if(OPTIC_STATE_IDLE == optic_data.state)
        {
            // 1-st symbol come!
            optic_data.buf[0] = USART1->DR;         // Put it to buffer
            optic_data.index = 1;                   // "Clear" the rest of buffer
            optic_data.state = OPTIC_STATE_RCVE;    // MBMachine: begin of receiving the request
        }
        else
        {
            error++;
            cnt =  USART1->DR;
            //USART1->SR = ~USART_SR_RXNE;
        }
    }
}

static void set_alarm (uint32_t alarm)
{
    protection_data.trig_alarm |= alarm;
    protection_data.startreck |= alarm & PR_ALARM_STARTRECK_MSK;
    MBbuf_main[REG_TRIG_ALARM] |= alarm;
    xTaskNotify(RegularTask_Handler, alarm, eSetBits);
}

static uint32_t reset_alarm (void)
{
    if (!protection_data.active_timer_alarm)
    {
        protection_data.trig_alarm = 0;
        protection_data.startreck = 0;
        MBbuf_main[REG_TRIG_ALARM] = 0;
        protection_data.event = PROTECTION_EVENT_NO_EVENTS;
        MBbuf_main[REG_STARTRACK] = 0;
        protection_data.mb_save_index = 0;
        MBbuf_main[REG_GOTRACK] = 0;
        return OPT_RET_OK;
    }
    return OPT_RET_ERROR;
}

static void alarm_service(uint32_t alarm, uint8_t alarm_pos, uint32_t delay_ms)
{
    uint32_t alarm_msk = (1<<alarm_pos);

    if(MBbuf_main[REG_PROTECT_ENABLE_BIT] & alarm_msk)
    {
        if (alarm)
        {
            if(!((protection_data.active_timer_alarm | protection_data.trig_alarm) & alarm_msk))
            {
                if (delay_ms > MIN_ALARM_TIME_MS)
                    xTimerChangePeriod(xAlarm_Timers[alarm_pos], delay_ms/(portTICK_RATE_MS),0);

                protection_data.active_timer_alarm |= alarm_msk;
                xTimerReset(xAlarm_Timers[alarm_pos], 0);
                protection_data.reset_count[alarm_pos] = 0;
            }
        }
        else if(protection_data.active_timer_alarm & alarm_msk)
        {
            protection_data.reset_count[alarm_pos]++;
            if (protection_data.reset_count[alarm_pos] > MIN_ALARM_RESET_AMOUNT)
            {
                xTimerStop(xAlarm_Timers[alarm_pos], 0);
                protection_data.active_timer_alarm &= ~alarm_msk;
            }
        }
    }
    else if(protection_data.active_timer_alarm & alarm_msk)
    {
        protection_data.active_timer_alarm &= ~alarm_msk;
        xTimerStop(xAlarm_Timers[alarm_pos], 0);
    }
}

void vOpticTask (void *pvParameters)
{
    uint8_t scale, scale_count = 0;
    vTaskDelay(50/portTICK_RATE_MS );
    while (1)
    {

        if(optic_data.events == OPTIC_EVENTS_UART_FIN)
        {
            scale = (1<<MBbuf_main[REG_TRECK_SCALE])-1;
            optic_data.events = OPTIC_EVENTS_NONE;
            // mV -> A
            I_mid = ((int16_t)(Vshunt_mid-(int16_t)MBbuf_main[REG_CALIB_MID_VALUE]) * MBbuf_main[REG_CUR_SHUNT_A]) / MBbuf_main[REG_CUR_SHUNT_MV];
            I_inst = ((int16_t)(Vshunt_inst-(int16_t)MBbuf_main[REG_CALIB_INST_VALUE]) * MBbuf_main[REG_CUR_SHUNT_A]) / MBbuf_main[REG_CUR_SHUNT_MV];
            I_didt = (Vshunt_didt * MBbuf_main[REG_CUR_SHUNT_A]) / MBbuf_main[REG_CUR_SHUNT_MV];
            //check alarm
            // REG_CUR_MAX_P
            alarm_service(I_inst >= MBbuf_main[REG_CUR_MAX_P], PR_ALARM_CUR_MAX_P_POS, 0);
            // REG_CUR_MAX_N
            alarm_service(I_inst <= (int16_t)(0-MBbuf_main[REG_CUR_MAX_N]), PR_ALARM_CUR_MAX_N_POS, 0);
            // PR_ALARM_CUR_OL_P
            alarm_service(I_mid >= MBbuf_main[REG_CUR_OL_P_VALUE], PR_ALARM_CUR_OL_P_POS, MBbuf_main[REG_CUR_OL_P_TIME_MS]);
            // PR_ALARM_CUR_OL_N
            alarm_service(I_mid <= (int16_t)(0-MBbuf_main[REG_CUR_OL_N_VALUE]), PR_ALARM_CUR_OL_N_POS, MBbuf_main[REG_CUR_OL_N_TIME_MS]);
            // PR_ALARM_CUR_DIDT_P
            alarm_service(I_didt >= MBbuf_main[REG_CUR_DIDT_P_VALUE], PR_ALARM_CUR_DIDT_P_POS, MBbuf_main[REG_CUR_DIDT_P_TIME_MS]);
            // PR_ALARM_CUR_DIDT_N
            alarm_service(I_didt <= (int16_t)(0-MBbuf_main[REG_CUR_DIDT_N_VALUE]), PR_ALARM_CUR_DIDT_N_POS, MBbuf_main[REG_CUR_DIDT_N_TIME_MS]);
            // PR_ALARM_VOLT_MAX
            alarm_service(V_mid >= (int16_t)MBbuf_main[REG_VOLT_MAX], PR_ALARM_VOLT_MAX_POS, MBbuf_main[REG_VOLT_MAX_TIME_MS]);
            // PR_ALARM_VOLT_MIN
            alarm_service(V_mid <= (int16_t)(MBbuf_main[REG_VOLT_MIN]), PR_ALARM_VOLT_MIN_POS, MBbuf_main[REG_VOLT_MIN_TIME_MS]);
            // overfill protection
            clamped_i_inst = (int16_t)(I_inst > INT16_MAX ? INT16_MAX : (I_inst < INT16_MIN ? INT16_MIN : I_inst));

            // treck part
            scale_count++;
            switch (protection_data.event)
            {
            case PROTECTION_EVENT_NO_EVENTS:
                if (protection_data.startreck)
                {
                    protection_data.event = PROTECTION_EVENT_WRITETRECK;
                    break;
                }
            //fall
            case PROTECTION_EVENT_WAIT_RESET:
                if (!(scale & scale_count))
                {
                    protection_data.treck_pre_cur_buf[TRECK_PRE_BUF & protection_data.startreck_write_pos] = clamped_i_inst;
                    protection_data.treck_pre_volt_buf[TRECK_PRE_BUF & protection_data.startreck_write_pos] = V_inst;
                    protection_data.startreck_write_pos++;
                }
                break;
            case PROTECTION_EVENT_WRITETRECK:
                if(protection_data.mb_save_index < TRACK_MAIN_SIZE)
                {
                    if (!(scale & scale_count))
                    {
                        MBbuf_main[TRACK_CUR_REG_MAIN + protection_data.mb_save_index] = clamped_i_inst;
                        MBbuf_main[TRACK_VOLT_REG_MAIN + protection_data.mb_save_index] = V_inst;
                        if (protection_data.mb_save_index <= TRECK_PRE_BUF)
                        {
                            MBbuf_main[TRACK_CUR_REG_START + protection_data.mb_save_index] = protection_data.treck_pre_cur_buf[TRECK_PRE_BUF & (protection_data.startreck_write_pos + protection_data.mb_save_index)];
                            MBbuf_main[TRACK_VOLT_REG_START + protection_data.mb_save_index] = protection_data.treck_pre_volt_buf[TRECK_PRE_BUF & (protection_data.startreck_write_pos + protection_data.mb_save_index)];
                        }
                        protection_data.mb_save_index++;
                    }
                }
                else
                {
                    protection_data.event = PROTECTION_EVENT_WAIT_RESET;
                    MBbuf_main[REG_STARTRACK] = 3;
                }
                break;

            default:
                break;
            }
            protection_data.lan_error_count = 0;
        }
        taskYIELD();
    }
}

void vCurrentAlarm (void *pvParameters)
{
    uint32_t i, ext_command;
    TickType_t xLastWakeTime;
    vTaskDelay(1500/portTICK_RATE_MS );
    xLastWakeTime = xTaskGetTickCount();
    while(1)
    {
        vTaskDelayUntil( &xLastWakeTime, 25/portTICK_RATE_MS );
        //printf("xPortGetFreeHeapSize=%d\n", xPortGetFreeHeapSize());
        //lan error
        protection_data.lan_error_count++;
        if (protection_data.lan_error_count >= OPTIC_LAN_ERROR_ATTEMPT)
        {
            protection_data.lan_error_count = OPTIC_LAN_ERROR_ATTEMPT;
            protection_data.lan_error = 1;
            protection_data.active_timer_alarm |= MBbuf_main[REG_PROTECT_ENABLE_BIT] & PR_ALARM_LAN_ERROR_MSK;
            IO_SetLine(IO_LED_MEASURE_LAN, LOW);
            if(!(protection_data.trig_alarm & PR_ALARM_LAN_ERROR_MSK) &&\
                    (MBbuf_main[REG_PROTECT_ENABLE_BIT] & PR_ALARM_LAN_ERROR_MSK))
                set_alarm(PR_ALARM_LAN_ERROR_MSK);
        }
        else
        {
            protection_data.lan_error = 0;
            protection_data.active_timer_alarm &= ~PR_ALARM_LAN_ERROR_MSK;
            IO_SetLine(IO_LED_MEASURE_LAN, HIGH);
        }
        //manual error
        ext_command = 0;
        for(i = 0; i < DI_NUM_CHANNELS; i++)
        {
            if (MBbuf_main[REG_DI_1_MODE + i] == DI_MODE_EXTERNAL_ALARM && get_io(i) )
            {
                ext_command = 1;
                break;
            }
        }
        if(!(protection_data.trig_alarm & PR_ALARM_MANUAL_MSK) &&\
                (MBbuf_main[REG_PROTECT_ENABLE_BIT] & PR_ALARM_MANUAL_MSK) &&\
                (ext_command!=0 || MBbuf_main[REG_EXTERNAL_ALARM] != 0))
        {
            protection_data.active_timer_alarm |= MBbuf_main[REG_PROTECT_ENABLE_BIT] & PR_ALARM_MANUAL_MSK;
            MBbuf_main[REG_EXTERNAL_ALARM] = 0;
            set_alarm(PR_ALARM_MANUAL_MSK);
        }
        else
        {
            protection_data.active_timer_alarm &= ~PR_ALARM_MANUAL_MSK;
        }
        // auto reset
        if (MBbuf_main[REG_SET_AUTO_RESET] == ALARM_MODE_AUTO_RESET)
        {
            if(protection_data.trig_alarm &&\
                    !protection_data.active_timer_alarm)
            {
                if(!protection_data.wait_reset_flag)
                {
                    xTimerChangePeriod(x_reset_alarm_timer, (MBbuf_main[REG_AUTO_RESET_S]*1000)/portTICK_RATE_MS, 0);
                    protection_data.wait_reset_flag = 1;
                    xTimerReset(x_reset_alarm_timer, 0);
                }
            }
            else
            {
                xTimerStop(x_reset_alarm_timer, 0);
                protection_data.wait_reset_flag = 0;
            }
        }
    }
}

void vDataFill(void *pvParameters)
{
    uint32_t i, ext_command, ext_go ,ext_command_prev = 0;
    int16_t clamped_value;
    while(1)
    {
        vTaskDelay(200/portTICK_RATE_MS);
        // update mb registers
        MBbuf_main[REG_CUR_MID_MV] = Vshunt_mid-(int16_t)MBbuf_main[REG_CALIB_MID_VALUE];
        clamped_value = (int16_t)(I_mid > INT16_MAX ? INT16_MAX : (I_mid < INT16_MIN ? INT16_MIN : I_mid));
        MBbuf_main[REG_CUR_MID] = clamped_value;
        MBbuf_main[REG_CUR_INST] = clamped_i_inst;
        MBbuf_main[REG_MEASURE_LAN_ERROR] = protection_data.lan_error;
        if((V_mid < (int16_t)MBbuf_main[REG_DELTA_V_VALUE]) && (V_mid > (int16_t)(-MBbuf_main[REG_DELTA_V_VALUE])))
        {
            MBbuf_main[REG_VOLT_MID] = 0;
        }
        else
        {
            MBbuf_main[REG_VOLT_MID] = V_mid;
        }
        MBbuf_main[REG_VOLT_INST] = V_inst;
        clamped_value = (int16_t)(I_didt > INT16_MAX ? INT16_MAX : (I_didt < INT16_MIN ? INT16_MIN : I_didt));
        MBbuf_main[REG_CUR_DIDT] = clamped_value;
        MBbuf_main[REG_ALARM] = protection_data.active_timer_alarm;
        // check DI command
        ext_command = 0;
        for(i = 0; i < DI_NUM_CHANNELS; i++)
        {
            switch (MBbuf_main[REG_DI_1_MODE + i])
            {
            case DI_MODE_RESET_ALARM:
                if (get_io(i) )
                {
                    ext_command |= 1 << DI_MODE_RESET_ALARM;
                    continue;
                }
                break;
            case DI_MODE_STARTREK:
                if (get_io(i))
                {
                    ext_command |= 1 << DI_MODE_STARTREK;
                    continue;
                }
                break;
            default:
                break;
            }
        }
        if (!(ext_command_prev & ext_command))
            ext_go = ext_command;
        else
            ext_go = 0;
        ext_command_prev = ext_command;
        // check mb command and manual command
        if (MBbuf_main[REG_GOTRACK] || (ext_go & (1 << DI_MODE_STARTREK)))
        {
            MBbuf_main[REG_GOTRACK] = 0;
            if (protection_data.event == PROTECTION_EVENT_NO_EVENTS)
            {
                protection_data.event = PROTECTION_EVENT_WRITETRECK;
            }
        }
        if (MBbuf_main[REG_PROTECTION_RESET] == PROTECTION_RESET_VALUE || (ext_go & (1 << DI_MODE_RESET_ALARM)))
        {
            reset_alarm();
            MBbuf_main[REG_PROTECTION_RESET] = 0;
        }
        if( MBbuf_main[REG_CALIB] == CALIBRATION_VALUE)
        {
            shunt_calibration();
            MBbuf_main[REG_CALIB] = 0;
        }
        if (protection_data.event == PROTECTION_EVENT_WAIT_RESET && \
                MBbuf_main[REG_STARTRACK] == 0 && \
                protection_data.trig_alarm == PR_ALARM_NO_ALARM && \
                protection_data.active_timer_alarm == 0)
        {
            reset_alarm();
        }
        //led
        if ((!protection_data.active_timer_alarm) && (!protection_data.trig_alarm))
            IO_SetLine(IO_LED_ALARM, LOW);
        else if ((protection_data.active_timer_alarm) && (!protection_data.trig_alarm))
            IO_InvertLine(IO_LED_ALARM);
        else
            IO_SetLine(IO_LED_ALARM, HIGH);

#ifdef DEBUG_TARGET
        ;// printf ("error=%ld--==--ok_count=%ld, value=%ld, value_2=%ld\n", error, count_ok, value, value_2);
#endif
    }
}

// Init modbus
static void oc_protection_init(void)
{
    //  v_shunt_mV = MBbuf_main[REG_CUR_SHUNT_MV] ;
    //  i_shunt_A = MBbuf_main[REG_CUR_SHUNT_A];
#ifdef DEBUG_TARGET
    printf ("oc_protection_init\n");
#endif
}

void vAlarm_Timers_Function (xTimerHandle xTimer)
{
    uint32_t pxTimerID;
    pxTimerID = (uint32_t) pvTimerGetTimerID(xTimer);
    set_alarm(pxTimerID);
}

void reset_alarm_timer_function (xTimerHandle xTimer)
{
    if (MBbuf_main[REG_SET_AUTO_RESET] == ALARM_MODE_AUTO_RESET)
    {
        reset_alarm();
    }
    protection_data.wait_reset_flag = 0;
}

// Init modbus
void oc_optic_current_init(void)
{
    uint32_t i;
    oc_protection_init();
    //timers
    for(i = 0; i < ALARM_NUMBER_TIMER; i++)
    {
        xAlarm_Timers[i] = xTimerCreate("One_Shot_Timer_n", MIN_ALARM_TIME_MS/portTICK_RATE_MS, pdFALSE, (void *)Alarm_Timers[i], vAlarm_Timers_Function);
    }
    x_reset_alarm_timer = xTimerCreate("reset_timer", (MBbuf_main[REG_AUTO_RESET_S]*1000)/portTICK_RATE_MS, pdFALSE, NULL, reset_alarm_timer_function);

    //  task
    if(pdTRUE != xTaskCreate(vOpticTask, "Optic", 	configMINIMAL_STACK_SIZE*3, NULL, tskIDLE_PRIORITY, NULL)) ERROR_ACTION(TASK_NOT_CREATE,0);
    if(pdTRUE != xTaskCreate(vCurrentAlarm,	"CurAl", configMINIMAL_STACK_SIZE*2, NULL, tskIDLE_PRIORITY, NULL)) ERROR_ACTION(TASK_NOT_CREATE,0);
    if(pdTRUE != xTaskCreate(vDataFill,	"Fill", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL)) ERROR_ACTION(TASK_NOT_CREATE,0);
}

void shunt_calibration()
{
    uint32_t i;
    int32_t mid_tmp = 0, inst_tmp = 0, v_tmp = 0;
    MBbuf_main[REG_CALIB_MID_VALUE] = 0;
    MBbuf_main[REG_CALIB_INST_VALUE] = 0;
    MBbuf_main[REG_CALIB_V_MID_VALUE] = 0;
    vTaskDelay(200/portTICK_RATE_MS);
    for(i=0; i < 64; i++)
    {
        mid_tmp = mid_tmp + Vshunt_mid;
        inst_tmp = inst_tmp + Vshunt_inst;
        v_tmp = v_tmp + V_mid;
        vTaskDelay(3/portTICK_RATE_MS);
    }
    MBbuf_main[REG_CALIB_MID_VALUE] = (mid_tmp >> 6);
    MBbuf_main[REG_CALIB_INST_VALUE] = (inst_tmp >> 6);
    MBbuf_main[REG_CALIB_V_MID_VALUE] = (v_tmp >> 6);

    EE_UpdateVariable(REG_CALIB_MID_VALUE, MBbuf_main[REG_CALIB_MID_VALUE]);
    EE_UpdateVariable(REG_CALIB_INST_VALUE, MBbuf_main[REG_CALIB_INST_VALUE]);
    EE_UpdateVariable(REG_CALIB_V_MID_VALUE, MBbuf_main[REG_CALIB_V_MID_VALUE]);
}

const unsigned char crc_array[256] =
{
    0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83,
    0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
    0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e,
    0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
    0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0,
    0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
    0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d,
    0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
    0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5,
    0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
    0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58,
    0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
    0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6,
    0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
    0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b,
    0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
    0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f,
    0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
    0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92,
    0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
    0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c,
    0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
    0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1,
    0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
    0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49,
    0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
    0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4,
    0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
    0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a,
    0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
    0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7,
    0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
};
//-------------------------------------------------------------------------
static uint8_t dallas_crc8(uint32_t buf_size, uint8_t *buf)
{
    unsigned char crc = 0;
    for ( uint32_t i = 0; i < buf_size; ++i)
    {
        crc = crc_array[buf[i] ^ crc];
    }
    return crc;
}
