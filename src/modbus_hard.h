#ifndef MODBUS_HARD_H_INCLUDED
#define MODBUS_HARD_H_INCLUDED

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

#define	UART_FRIQ	36000000LL

#define MODBUS_TASK_PRIORITY               (tskIDLE_PRIORITY + 1)
#define MODBUS_TASK_STACK_SIZE             (configMINIMAL_STACK_SIZE*2)

#define	 BAUD_9600		    9600//0x1D4C
#define	 BAUD_19200		    19200//0xEA6
#define	 BAUD_57600		    57600//0x4E2
#define	 BAUD_115200	    115200//0x271
#define  BAUD_NUMBER        4
#define	 RS_485_BAUD_LIST   {BAUD_9600, BAUD_19200, BAUD_57600, BAUD_115200}

#define	 EEPROM_SAVE_DELAY_MS    60000

typedef enum
{
    NO_PARITY_1_STOP	= 0x00,
    NO_PARITY_2_STOP	= 0x01,
    EVEN_PARITY_1_STOP	= 0x02,
    ODD_PARITY_1_STOP	= 0x03,

} Parity_Stop_Bits_t;

void mh_Modbus_Init(void);
void mh_Factory (void);
void mh_Buf_Init (void);
void mh_update_all_eeprom ();
void mh_USB_Recieve(uint8_t *USB_buf, uint16_t len);
void mh_set_mbbuf_reg (uint16_t value_16, uint16_t reg_addr);
uint16_t mh_get_mbbuf_reg (uint16_t reg_addr);
#endif /* MODBUS_HARD_H_INCLUDED */
