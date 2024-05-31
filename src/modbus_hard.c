#include "modbus_hard.h"
#include "modbus.h"
#include "modbus_reg.h"
#include "stm32f1xx_ll_usart.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

#include "IO.h"
#include "main.h"
#include <eeprom_emulation.h>

#include <stdbool.h>
#include "inttypes.h"
#include <stdio.h>
#include <string.h>

#include "usbd_cdc_if.h"

//-----------------------------------------------------------------------
// Variable
//-----------------------------------------------------------------------

uint16_t MBbuf_main[MB_NUM_BUF]= {VERSION_REB};

xQueueHandle xModbusQueue;
TimerHandle_t rs485_timer_handle, eeprom_timer_handle;
TaskHandle_t m_modbus_task_handle;

volatile MBStruct_t MB_RS485;
volatile MBStruct_t MB_USB;

uint8_t RS485_MB_Buf[MB_FRAME_MAX];
uint8_t USB_MB_Buf[MB_FRAME_MAX];

const uint32_t Baud_rate[BAUD_NUMBER]= RS_485_BAUD_LIST;

//-----------------------------------------------------------------------
// Prototype
//-----------------------------------------------------------------------
static void mh_EnableTransmission(const bool Enable);
static void rs485_timer_callback (xTimerHandle xTimer);
static void eeprom_timer_callback (xTimerHandle xTimer);
static void mh_Notify_Eeprom (void *mbb);
static void mh_USB_Init(void);
static void mh_RS485_Init(void);
static void IO_Uart3_Init(void);
static void mh_USB_Transmit_Start (void *mbb);
static void mh_Rs485_Transmit_Start (void *mbb);

//-----------------------------------------------------------------------
// Task function
//-----------------------------------------------------------------------

void mh_task_Modbus (void *pvParameters)
{
    MBStruct_t *st_mb;
    vTaskDelay(3000);
    //  TaskStatus_t xTaskStatus;
    while(1)
    {
        xQueueReceive(xModbusQueue,&st_mb, portMAX_DELAY);
        mb_parsing((MBStruct_t*) st_mb);
        taskYIELD();
        // printf("Heap Size = %d\n",xPortGetFreeHeapSize());
        // vTaskGetInfo(m_modbus_task_handle, &xTaskStatus, pdTRUE, eInvalid);
        // printf("Task stack high water mark (freespace): %d\n", xTaskStatus.usStackHighWaterMark);
    }
}

//-----------------------------------------------------------------------
// irq
//-----------------------------------------------------------------------

void USART3_IRQHandler (void)
{
    uint8_t cnt;
    (void) cnt;
    if (USART3->SR & (USART_SR_FE | USART_SR_ORE | USART_SR_NE))
    {
        MB_RS485.mb_state = MB_STATE_IDLE;
        cnt = USART3->DR;
    }
    else if (USART3->SR & USART_SR_RXNE)
    {
        // xTimerResetFromISR(rs485_timer_handle, NULL);	// Timer reset anyway: received symbol means NO SILENCE
        if( MB_STATE_RCVE == MB_RS485.mb_state)
        {
            if(MB_RS485.mb_index >= MB_FRAME_MAX-1)
            {
                MB_RS485.er_frame_bad = EV_HAPPEND;	                 // This error will be processed later
                USART3->SR = ~USART_SR_RXNE;                         // Nothing more to do in RECEIVE state
            }
            else
            {
                MB_RS485.p_mb_buff[MB_RS485.mb_index++] = USART3->DR;	 // MAIN DOING: New byte to buffer
            }
        }
        else if(mb_instance_idle_check((MBStruct_t*)&MB_RS485)==MB_OK)
        {
            // 1-st symbol come!
            MB_RS485.p_mb_buff[0] = USART3->DR; 		// Put it to buffer
            MB_RS485.mb_index = 1;						// "Clear" the rest of buffer
            MB_RS485.er_frame_bad = EV_NOEVENT;			// New buffer, no old events
            MB_RS485.mb_state=MB_STATE_RCVE;				// MBMachine: begin of receiving the request
        }
        else
        {
            USART3->SR = ~USART_SR_RXNE;
        }
    }
    if (USART3->SR & USART_SR_TC)
    {
        USART3->SR = ~(USART_SR_TC);
        MB_RS485.mb_state = MB_STATE_IDLE;
        mh_EnableTransmission(false);
    }
    if (USART3->SR & USART_SR_TXE)
    {
        if( MB_STATE_SEND == MB_RS485.mb_state)
        {
            if( MB_RS485.mb_index < MB_RS485.response_size)
            {
                //mh_EnableTransmission(true);
                USART3->DR = MB_RS485.p_mb_buff[MB_RS485.mb_index++];   //  sending of the next byte
            }
            else
            {
                MB_RS485.mb_state=MB_STATE_SENT;
                USART3->CR1 &= ~USART_CR1_TXEIE;
            }
        }
        else
        {
            mh_EnableTransmission(false);
            USART3->CR1 &= ~USART_CR1_TXEIE;
        }
    }

    if (USART3->SR & USART_SR_IDLE)
    {
        cnt = USART3->DR;
        if (MB_STATE_RCVE == MB_RS485.mb_state)
        {
            xTimerResetFromISR(rs485_timer_handle, NULL);
        }
    }
}

//-----------------------------------------------------------------------
// Function
//-----------------------------------------------------------------------
// uart delay
static void rs485_timer_callback (xTimerHandle xTimer)
{
    if( MB_STATE_RCVE == MB_RS485.mb_state)
    {
        // If we are receiving, it's the end event: t3.5
        MB_RS485.mb_state=MB_STATE_PARS;					// Begin parsing of a frame.
        MBStruct_t *st_mb=(MBStruct_t*)&MB_RS485;
        xQueueSend(xModbusQueue, &st_mb, 0);
    }
}

//eeprom save delay
static void eeprom_timer_callback (xTimerHandle xTimer)
{
    mh_update_all_eeprom();
}

static void mh_EnableTransmission(const bool Enable)
{
    if (Enable)
        IO_SetLine(IO_RS485_SWITCH,HIGH);
    else
        IO_SetLine(IO_RS485_SWITCH,LOW);
}


static void mh_Notify_Eeprom (void *mbb)
{
    if (!xTimerIsTimerActive(eeprom_timer_handle))
        xTimerReset(eeprom_timer_handle, 0);
}

//void mh_Write_Eeprom (void *mbb)
//{
//    MBStruct_t *st_mb;
//    st_mb = (void*) mbb;
//
//    for (int32_t i = 0; i < (st_mb->cb_index); i++)
//    {
//        if((mb_reg_option_check(i+(st_mb->cb_reg_start), CB_WR) == MB_OK))
//        {
//            EE_UpdateVariable(((st_mb->cb_reg_start)+i), st_mb->p_write[i+(st_mb->cb_reg_start)]);
//        }
//    }
//}

static void mh_USB_Init(void)
{
    MB_USB.p_write = MBbuf_main;
    MB_USB.p_read = MBbuf_main;
    MB_USB.reg_read_last=MB_NUM_BUF-1;
    MB_USB.reg_write_last=MB_NUM_BUF-1;
    MB_USB.cb_state=MB_CB_FREE;
    MB_USB.er_frame_bad=EV_NOEVENT;
    MB_USB.slave_address=MB_ANY_ADDRESS;	//0==any address
    MB_USB.mb_state=MB_STATE_IDLE;
    MB_USB.p_mb_buff=&USB_MB_Buf[0];
    MB_USB.wr_callback = mh_Notify_Eeprom;
    MB_USB.f_start_trans = mh_USB_Transmit_Start;
    MB_USB.f_start_receive = NULL;
}

static void mh_RS485_Init(void)
{
    uint32_t Rs485_Time_ms;

    MB_RS485.p_write = MBbuf_main;
    MB_RS485.p_read = MBbuf_main;
    MB_RS485.reg_read_last=MB_NUM_BUF-1;
    MB_RS485.reg_write_last=MB_NUM_BUF-1;
    MB_RS485.cb_state=MB_CB_FREE;
    MB_RS485.er_frame_bad=EV_NOEVENT;
    MB_RS485.slave_address=MBbuf_main[REG_RS_ADDR];
    MB_RS485.mb_state=MB_STATE_IDLE;
    MB_RS485.p_mb_buff=&RS485_MB_Buf[0];
    MB_RS485.wr_callback = mh_Notify_Eeprom;
    MB_RS485.f_start_trans=mh_Rs485_Transmit_Start;
    MB_RS485.f_start_receive = NULL;

    Rs485_Time_ms = (MBbuf_main[REG_RS_DELAY]);
    rs485_timer_handle = xTimerCreate( "T_RS485", Rs485_Time_ms/portTICK_RATE_MS, pdFALSE, NULL, rs485_timer_callback);
    IO_Uart3_Init();
}

static void IO_Uart3_Init(void)
{
    RCC->APB1ENR	|= RCC_APB1ENR_USART3EN;						//USART3 Clock ON
    // USART3->BRR = Baud_rate[MBbuf_main[Reg_RS485_Baud_Rate]&0x3];	// Bodrate

    LL_USART_SetBaudRate(USART3, UART_FRIQ, Baud_rate[MBbuf_main[REG_RS_BAUD]&0x3]);
    USART3->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_IDLEIE | USART_CR1_TCIE | USART_CR1_RXNEIE;

    switch (MBbuf_main[REG_RS_PARITY_STOP])
    {
    case NO_PARITY_1_STOP:
        break; //default setting

    case NO_PARITY_2_STOP:
        USART3->CR2  |=  USART_CR2_STOP_1;
        break;

    case EVEN_PARITY_1_STOP:
        USART3->CR1  |= USART_CR1_PCE | USART_CR1_M;
        break;

    case ODD_PARITY_1_STOP:
        USART3->CR1  |= USART_CR1_PCE | USART_CR1_M | USART_CR1_PS;
        break;

    default:
        break;
    }
    USART3->CR1 |= USART_CR1_UE;
    NVIC_SetPriority(USART3_IRQn,14);
    NVIC_EnableIRQ (USART3_IRQn);
}

static void mh_USB_Transmit_Start (void *mbb)
{
    MBStruct_t *st_mb;
    st_mb = (void*) mbb;
    CDC_Transmit_FS (st_mb->p_mb_buff, st_mb->response_size);
    MB_USB.mb_state=MB_STATE_IDLE;
}

static void mh_Rs485_Transmit_Start (void *mbb)
{
    mh_EnableTransmission(true);
    USART3->CR1 |= USART_CR1_TXEIE;
}

// Callback for usb com
void mh_USB_Recieve(uint8_t *USB_buf, uint16_t len)	//interrupt	function
{
    if (mb_instance_idle_check((MBStruct_t*)&MB_USB)==MB_OK)
    {
        if(len>MB_FRAME_MAX)
        {
            len=MB_FRAME_MAX;
        }
        MB_USB.mb_state=MB_STATE_PARS;
        MB_USB.mb_index=(len);
        memcpy (MB_USB.p_mb_buff,USB_buf,len);
        MBStruct_t *st_mb=(MBStruct_t*)&MB_USB;
        xQueueSendFromISR(xModbusQueue, &st_mb, 0);
    }
}

// Init modbus
void mh_Modbus_Init(void)
{
    //create queue
    xModbusQueue=xQueueCreate(3,sizeof(MBStruct_t *));

    //create modbus task
    if(pdTRUE != xTaskCreate(mh_task_Modbus, "RS485", MODBUS_TASK_STACK_SIZE, NULL, MODBUS_TASK_PRIORITY, &m_modbus_task_handle)) ERROR_ACTION(TASK_NOT_CREATE, 0);

    //timer
    eeprom_timer_handle = xTimerCreate("ee_timer", EEPROM_SAVE_DELAY_MS/portTICK_RATE_MS, pdFALSE, NULL, eeprom_timer_callback);
    mh_USB_Init();
    mh_RS485_Init();
}

void mh_Factory (void)
{
    taskENTER_CRITICAL();
    for (int32_t i=0; i< MB_NUM_BUF; i++)
    {
        if (mb_reg_option_check(i, CB_WR)==MB_OK)
        {
            MBbuf_main[i] = mb_getRegParam(i).Default_Value;
            EE_UpdateVariable(i, MBbuf_main[i]);
        }
    }
    taskEXIT_CRITICAL();
}

void mh_Buf_Init (void)
{
    int32_t i=0;
    EEPRESULT stat;

    taskENTER_CRITICAL();
    for (i=0; i< MB_NUM_BUF; i++)
    {
        if(mb_reg_option_check(i, CB_WR)==MB_OK)
        {
            stat = EE_ReadVariable(i, &MBbuf_main[i]);
            if((mb_reg_limit_check(i, MBbuf_main[i])==MB_ERROR) || (stat!=RES_OK))
            {
                MBbuf_main[i]=mb_getRegParam(i).Default_Value;
                EE_UpdateVariable(i, MBbuf_main[i]);
            }
        }
    }
    taskEXIT_CRITICAL();
}

void mh_update_all_eeprom ()
{
    taskENTER_CRITICAL();
    for (int32_t i=0; i< MB_NUM_BUF; i++)
    {
        if (mb_reg_option_check(i, CB_WR)==MB_OK)
        {
            EE_UpdateVariable(i, MBbuf_main[i]);
        }
    }
    taskEXIT_CRITICAL();
}

EEPRESULT GetNextVirtAddrData(uint16_t VirtAddressLast, uint16_t *VirtAddressNext, uint16_t *NextData)
{
    uint32_t i;
    VirtAddressLast ++;

    for (i=VirtAddressLast ; i< MB_NUM_BUF ; i++)
    {
        if (mb_reg_option_check(i, CB_WR)==MB_OK)
        {
            *VirtAddressNext = i;
            *NextData = MBbuf_main[i];
            return RES_OK;
        }
    }
    return RES_ERROR;
}

void mh_set_mbbuf_reg (uint16_t value_16, uint16_t reg_addr)
{
    MBbuf_main[reg_addr] = value_16;
}

uint16_t mh_get_mbbuf_reg ( uint16_t reg_addr)
{
    return MBbuf_main[reg_addr];
}
