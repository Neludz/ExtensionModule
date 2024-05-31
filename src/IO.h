#ifndef _IO_H
#define _IO_H

#include <stdint.h>
#include <stdbool.h>
#include <stm32f1xx.h>

#define IN	  	  (0x00)    //MODE_1
#define OUT_10MHz (0x01)    //MODE_1
#define OUT_2MHz  (0x02)    //MODE_1
#define OUT_50MHz (0x03)    //MODE_1


#define OUT_PP   (0x00) // MODE_2 - General purpose output push-pull
#define OUT_OD   (0x04) // MODE_2 - General purpose output Open-drain
#define OUT_APP  (0x08) // MODE_2 - Alternate function output Push-pull
#define OUT_AOD  (0x0C) // MODE_2 - Alternate function output Open-drain

#define IN_ADC   (0x00)     //MODE_2
#define IN_HIZ   (0x04)     //MODE_2
#define IN_PULL  (0x08)     //MODE_2

#define OPTIC_BAUD 2000000LL
#define	UART1_FRIQ	72000000LL

typedef struct
{
    GPIO_TypeDef* GPIOx;
    uint16_t GPIO_Pin;
    uint8_t MODE;
    uint8_t DefState;
    uint8_t ActiveState;
} tGPIO_Line;


//------========IO_Start_Table========------
//	 NAME  					GPIOx   GPIO_Pin    MODE_1 		MODE_2	 DefState  ActiveState
#define IO_TABLE\
	X_IO(IO_LED_STATUS,		GPIOA,	15,			OUT_2MHz,	OUT_PP,		LOW,  	HIGH)	\
	X_IO(IO_LED_ALARM,		GPIOB,	3,			OUT_2MHz,	OUT_PP,		LOW,  	HIGH)	\
	X_IO(IO_LED_IN,			GPIOA,	4,			OUT_2MHz,	OUT_PP,		LOW,  	HIGH)	\
	X_IO(IO_LED_MEASURE_LAN,GPIOA,	0,			OUT_2MHz,	OUT_PP,		LOW,  	HIGH)	\
	X_IO(IO_RS485_SWITCH,	GPIOB,  1, 			OUT_2MHz,	OUT_PP,  	LOW,  	HIGH)	\
	X_IO(IO_RX,				GPIOB,  11, 		IN,			IN_HIZ,  	HIGH,  	HIGH)	\
	X_IO(IO_TX,				GPIOB,  10, 		OUT_50MHz,	OUT_APP, 	HIGH,  	HIGH)	\
	X_IO(IO_OPTIC_RX,   	GPIOA,  10, 		IN,	        IN_HIZ, 	HIGH,  	HIGH)	\
	X_IO(IO_DI_1,			GPIOB,  14,			IN,			IN_PULL,	HIGH,  	LOW)	\
	X_IO(IO_DI_2,			GPIOB,  13,  		IN,			IN_PULL,	HIGH,  	LOW)	\
	X_IO(IO_DI_3,			GPIOA,  8,  		IN,			IN_PULL,   	HIGH,  	LOW)	\
	X_IO(IO_DI_4,			GPIOB,  15,  		IN,			IN_PULL,    HIGH,  	LOW)	\
	X_IO(IO_DO_1,			GPIOB,  4,  		OUT_50MHz,	OUT_PP,		LOW,  	HIGH)	\
	X_IO(IO_DO_2,			GPIOB,  5,  		OUT_50MHz,	OUT_PP,		LOW,  	HIGH)	\
	X_IO(IO_DO_3,			GPIOB,  2,  		OUT_50MHz,	OUT_PP,		LOW,  	HIGH)	\
	X_IO(IO_DO_4,			GPIOB,  0,  		OUT_50MHz,	OUT_PP,		LOW,  	HIGH)	\

//USB pins init in default state
//------========IO_End_Table========------

typedef enum
{
#define X_IO(a,b,c,d,e,f,g)	a,
    IO_TABLE
#undef X_IO
    NUM_IO		//count
} tIOLine;

typedef enum
{
    OFF = 0,
    ON = 1,
    LOW = 0,
    HIGH =1,
} tIOState;

void IO_Init(void);
void IO_SetLine(tIOLine Line, bool State);
void IO_InvertLine(tIOLine Line);
bool IO_GetLine(uint8_t Input);
void IO_SetLineActive(tIOLine Line, bool State);
bool IO_GetLineActive(uint8_t Input);
void IO_ConfigLine(tIOLine Line, uint8_t Mode, uint8_t State);
void IO_SPI_Init(void);
void IO_delay_ms(uint32_t ms);
void IO_SetLineActiveExtern(tIOLine Line, bool State, bool Active);

//example
//     	i = IO_GetLine(io_LED3);
//		IO_ConfigLine(io_LED2, OUT_10MHz+OUT_PP, LOW);
//		IO_ConfigLine(io_ADC, IN+IN_ADC, LOW);


#endif /* _IO_H */

