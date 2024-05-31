
#include <stdint.h>
#include <stdbool.h>
#include "stm32f1xx_ll_usart.h"
#include <stm32f1xx.h>
#include <IO.h>
#include <main.h>


static void IO_uart_init();
//--------------X macros---------------------------------------------------------
const tGPIO_Line IOs[NUM_IO] =
{
#define X_IO(a,b,c,d,e,f,g)	{b,c,d+e,f,g},
    IO_TABLE
#undef X_IO
};

//---------------------------------------------------------------------------------
void IO_delay_ms(uint32_t ms)
{
    volatile uint32_t nCount;
//RCC_ClocksTypeDef RCC_Clocks;
//RCC_GetClocksFreq (&RCC_Clocks);
//nCount=(RCC_Clocks.HCLK_Frequency/10000)*ms;
    nCount=(SYSCLK_FREQ/10000)*ms;
    for (; nCount!=0; nCount--);
}
//---------------------------------------------------------------------------------
void IO_SetLine(tIOLine Line, bool State)
{
    if (State)
        IOs[Line].GPIOx->BSRR = 1 << (IOs[Line].GPIO_Pin);
    else
        IOs[Line].GPIOx->BRR = 1 << (IOs[Line].GPIO_Pin);
}
//---------------------------------------------------------------------------------
void IO_InvertLine(tIOLine Line)
{
    IOs[Line].GPIOx->ODR ^= 1 << IOs[Line].GPIO_Pin;
}
//---------------------------------------------------------------------------------
bool IO_GetLine(tIOLine Line)
{
    if (Line < NUM_IO)
        return (((IOs[Line].GPIOx->IDR) & (1<<(IOs[Line].GPIO_Pin))) != 0);
    else
        return false;
}

//---------------------------------------------------------------------------------
bool IO_GetLineActive(tIOLine Line)
{
    if (Line < NUM_IO)
    {
        bool pin_set = (((IOs[Line].GPIOx->IDR) & (1<<(IOs[Line].GPIO_Pin))) ? true : false);
        return (pin_set == ( IOs[Line].ActiveState ? true : false));
    }
    else
        return false;
}
//---------------------------------------------------------------------------------
void IO_SetLineActive(tIOLine Line, bool State)
{
    if (State ^ IOs[Line].ActiveState)
    {
        IOs[Line].GPIOx->BRR = 1 << (IOs[Line].GPIO_Pin);   //reset
    }
    else
    {
        IOs[Line].GPIOx->BSRR = 1 << (IOs[Line].GPIO_Pin);  //set
    }
}

//---------------------------------------------------------------------------------
void IO_SetLineActiveExtern(tIOLine Line, bool State, bool Active)
{
    if (State ^ Active)
    {
        IOs[Line].GPIOx->BRR = 1 << (IOs[Line].GPIO_Pin);   //reset
    }
    else
    {
        IOs[Line].GPIOx->BSRR = 1 << (IOs[Line].GPIO_Pin);  //set
    }
}
//---------------------------------------------------------------------------------
void IO_ConfigLine(tIOLine Line, uint8_t Mode, uint8_t State)
{
    if(IOs[Line].GPIO_Pin < 8)
    {
        IOs[Line].GPIOx->CRL &=   ~(0x0F << (IOs[Line].GPIO_Pin * 4));
        IOs[Line].GPIOx->CRL |=  Mode<<(IOs[Line].GPIO_Pin * 4);
    }
    else
    {
        IOs[Line].GPIOx->CRH &=   ~(0x0F << ((IOs[Line].GPIO_Pin - 8)* 4));
        IOs[Line].GPIOx->CRH |=    Mode<<((IOs[Line].GPIO_Pin - 8)* 4);
    }

    IOs[Line].GPIOx->ODR &= ~(1<<IOs[Line].GPIO_Pin);
    IOs[Line].GPIOx->ODR |= State<<IOs[Line].GPIO_Pin;
}

//---------------------------------------------------------------------------------
void IO_Init(void)
{
//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);

    RCC->APB2ENR	|= RCC_APB2ENR_IOPAEN;
    RCC->APB2ENR 	|= RCC_APB2ENR_IOPBEN;
//RCC->APB2ENR	|= RCC_APB2ENR_IOPCEN;
//RCC->APB2ENR	|= RCC_APB2ENR_IOPDEN;
//RCC->APB2ENR	|= RCC_APB2ENR_IOPEEN;
//RCC->APB2ENR	|= RCC_APB2ENR_IOPFEN;
//RCC->APB2ENR	|= RCC_APB2ENR_IOPGEN;

    RCC->APB2ENR	|= RCC_APB2ENR_AFIOEN;

// for PA15
    AFIO->MAPR|=AFIO_MAPR_SWJ_CFG_JTAGDISABLE;

// Set all pins
    for (int Line = 0; Line < NUM_IO; Line++)
    {
        IO_ConfigLine(Line, IOs[Line].MODE, IOs[Line].DefState);
    }
    IO_uart_init();
}

static void IO_uart_init()
{
    RCC->APB2ENR	|= RCC_APB2ENR_USART1EN;						//USART3 Clock ON
    LL_USART_SetBaudRate(USART1, UART1_FRIQ, OPTIC_BAUD);
    USART1->CR1 |= USART_CR1_RE | USART_CR1_RXNEIE | USART_CR1_IDLEIE;
    USART1->CR1 |= USART_CR1_UE;

    NVIC_SetPriority(USART1_IRQn,1);
    NVIC_EnableIRQ (USART1_IRQn);
}


