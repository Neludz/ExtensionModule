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

extern uint16_t MBbuf_main[];
extern bool X_DI[MAX_DI];
uint32_t error=0, count_ok=0;
uint32_t Imid, Iins, uart_fin=0;
int16_t Vshunt_mid, Vshunt_ins, Vshunt_didt, Vmid;
int32_t Ididt;
uint32_t D_cur_n_ol, D_;
uint16_t v_shunt_mv, i_shunt;


OpticStruct_t optic_data =
{
    .state = OPTIC_STATE_IDLE,
    .events = OPTIC_EVENTS_OK,
};
ProtectionStruct_t protection_data =
{
    .current_alarm_bit = 0,
    .trig_alarm = PROTECTION_ALARM_NO_ALARM,
    .event = PROTECTION_EVENT_NO_EVENTS,
    .mb_save_index = 0,
};

static unsigned char dallas_crc8(const unsigned int size, uint8_t *buf);
static void oc_protection_init(void);

void USART1_IRQHandler (void)
{
    uint8_t cnt;
    (void) cnt;
    if (USART1->SR & USART_SR_RXNE)
    {
        if( OPTIC_STATE_RCVE == optic_data.state)
        {
            optic_data.buf[(optic_data.index++) & OPTIC_BUFFER_SIZE] = USART1->DR;	 // MAIN DOING: New byte to buffer
        }
        else if(OPTIC_STATE_IDLE == optic_data.state)
        {
            // 1-st symbol come!
            optic_data.buf[0] = USART1->DR; 		    // Put it to buffer
            optic_data.index = 1;						// "Clear" the rest of buffer
            optic_data.state = OPTIC_STATE_RCVE;		// MBMachine: begin of receiving the request
        }
        else
        {

            error++;
            USART1->SR = ~USART_SR_RXNE;
        }
    }
    if (USART1->SR & USART_SR_IDLE)
    {
        cnt = USART1->DR;
        if (OPTIC_STATE_RCVE == optic_data.state)
        {
           // optic_data.state = OPTIC_STATE_PARSE;
            if(!dallas_crc8( 8, &optic_data.buf[0]) && optic_data.index == 8)
            {
                Vshunt_mid = (optic_data.buf[0] | optic_data.buf[1]<<8)+8;
                Vshunt_ins = (optic_data.buf[2] | optic_data.buf[3]<<8)+8;
                Vshunt_didt = optic_data.buf[4] | optic_data.buf[5]<<8;
                Vmid = (int8_t)optic_data.buf[6];
                optic_data.events = OPTIC_EVENTS_UART_FIN;
            }
            else
            {
                error++;
            }
        }
        optic_data.state = OPTIC_STATE_IDLE;
    }
}

void vOpticTask (void *pvParameters)
{
    while (1)
    {
        if(optic_data.events == OPTIC_EVENTS_UART_FIN)
        {
             optic_data.events = OPTIC_EVENTS_OK;
            count_ok++;
            switch (protection_data.event)
            {
            case   PROTECTION_EVENT_NO_EVENTS:
                if (Vshunt_ins>protection_data.protection_mvx10_cur_max_p)
                {
                    protection_data.event = PROTECTION_EVENT_STARTRECK;
                    break;
                }
            //fall
            case PROTECTION_EVENT_WAIT_RESET:
                protection_data.treck_pre_buf[TRECK_PRE_BUF & protection_data.startreck_write_pos++] = Vshunt_ins;
                break;

            case PROTECTION_EVENT_STARTRECK:
                if(protection_data.mb_save_index < TRACK_POINT_NUM)
                {
                    MBbuf_main[TRACK_CUR_REG_START + TRECK_PRE_BUF + protection_data.mb_save_index] = Vshunt_ins;
                    if (protection_data.mb_save_index < TRECK_PRE_BUF)
                    {
                        MBbuf_main[TRACK_CUR_REG_START + protection_data.mb_save_index] = protection_data.treck_pre_buf[TRECK_PRE_BUF & (protection_data.startreck_write_pos + protection_data.mb_save_index)];
                    }
                    protection_data.mb_save_index++;
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


        }
        taskYIELD();
    }
}

void vCurrentAlarm (void *pvParameters)
{
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    oc_protection_init();
    while(1)
    {
        vTaskDelayUntil( &xLastWakeTime, 1000/portTICK_RATE_MS );

    }
}

void vDataFill(void *pvParameters)
{
    MBbuf_main[REG_COUNTRACK] = 400;
    while(1)
    {
        vTaskDelay( 400/portTICK_RATE_MS );


        MBbuf_main[REG_CUR_MID] = (Vshunt_mid * v_shunt_mv)/v_shunt_mv;
        MBbuf_main[REG_CUR_INST] = (Vshunt_ins * v_shunt_mv)/v_shunt_mv;
        MBbuf_main[REG_VOLT_INST] = Vmid;
        MBbuf_main[REG_CUR_DIDT] = (Vshunt_didt * v_shunt_mv)/v_shunt_mv;



        if (protection_data.event == PROTECTION_EVENT_NO_EVENTS)
        {
            if (MBbuf_main[REG_GOTRACK])
            {
                protection_data.event = PROTECTION_EVENT_STARTRECK;
            }
        }
        else if (MBbuf_main[REG_RESET])
        {
            protection_data.event = PROTECTION_EVENT_NO_EVENTS;
            MBbuf_main[REG_RESET] = 0;
            MBbuf_main[REG_STARTRACK] = 0;
            protection_data.mb_save_index = 0;
            MBbuf_main[REG_GOTRACK] = 0;
        }



#ifdef DEBUG_TARGET
        printf ("optic_data.buf[6]=%d \n", optic_data.buf[6]);
#endif
    }
}

// Init modbus
static void oc_protection_init(void)
{
    v_shunt_mv = MBbuf_main[REG_CUR_SHUNT_MV] * 10;
    i_shunt = MBbuf_main[REG_CUR_SHUNT_A];
    protection_data.protection_mvx10_cur_max_p = ((int16_t)MBbuf_main[REG_CUR_MAX_P]*v_shunt_mv)/i_shunt;
#ifdef DEBUG_TARGET
    printf ("oc_protection_init = %d  \n", protection_data.protection_mvx10_cur_max_p);
#endif
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
static unsigned char dallas_crc8(const unsigned int size, uint8_t *buf)
{
    unsigned char crc = 0;
    for ( unsigned int i = 0; i < size; ++i )
    {
        crc = crc_array[(0xFF & buf[i]) ^ crc];
    }
    return crc;
}

// Init modbus
void oc_optic_current_init(void)
{
    if(pdTRUE != xTaskCreate(vOpticTask, "Optic", 	configMINIMAL_STACK_SIZE*2, NULL, tskIDLE_PRIORITY, NULL)) ERROR_ACTION(TASK_NOT_CREATE,0);
    if(pdTRUE != xTaskCreate(vCurrentAlarm,	"CurAl", configMINIMAL_STACK_SIZE*2, NULL, tskIDLE_PRIORITY, NULL)) ERROR_ACTION(TASK_NOT_CREATE,0);
    if(pdTRUE != xTaskCreate(vDataFill,	"Fill", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL)) ERROR_ACTION(TASK_NOT_CREATE,0);
}







