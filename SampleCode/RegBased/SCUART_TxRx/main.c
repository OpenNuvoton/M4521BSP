/**************************************************************************//**
 * @file     main.c
 * @version  V2.0
 * $Revision: 5 $
 * $Date: 15/09/02 10:03a $
 * @brief    Show smartcard UART mode by connecting PA.0 and PA.1 pins.
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "M4521.h"

uint8_t au8TxBuf[] = "Hello World!";
#define PLLCTL_SETTING      CLK_PLLCTL_72MHz_HXT
#define PLL_CLOCK       72000000

int32_t g_SCUART_i32ErrCode = 0;       /*!< SCUART global error code */

/*---------------------------------------------------------------------------------------------------------*/
/* The interrupt services routine of smartcard port 0                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void SC0_IRQHandler(void)
{
    // Print SCUART received data to UART port
    // Data length here is short, so we're not care about UART FIFO over flow.
    UART_WRITE(UART0, SCUART_READ(SC0));

    // RDA is the only interrupt enabled in this sample, this status bit
    // automatically cleared after Rx FIFO empty. So no need to clear interrupt
    // status here.

    return;
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock (Internal RC 22.1184MHz) */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;

    /* Waiting for HIRC clock ready */
    while(!(CLK->STATUS & CLK_STATUS_HIRCSTB_Msk));

    /* Select HCLK clock source as HIRC and and HCLK clock divider as 1 */
    CLK->CLKSEL0 &= ~CLK_CLKSEL0_HCLKSEL_Msk;
    CLK->CLKSEL0 |= CLK_CLKSEL0_HCLKSEL_HIRC;
    CLK->CLKDIV0 &= ~CLK_CLKDIV0_HCLKDIV_Msk;
    CLK->CLKDIV0 |= CLK_CLKDIV0_HCLK(1);

    /* Set PLL to Power-down mode and PLLSTB bit in CLK_STATUS register will be cleared by hardware.*/
    CLK->PLLCTL |= CLK_PLLCTL_PD_Msk;

    /* Enable HXT clock (external XTAL 12MHz) */
    CLK->PWRCTL |= CLK_PWRCTL_HXTEN_Msk;

    /* Waiting for HXT clock ready */
    while(!(CLK->STATUS & CLK_STATUS_HXTSTB_Msk));

    /* Set core clock as PLL_CLOCK from PLL */
    CLK->PLLCTL = PLLCTL_SETTING;
    while(!(CLK->STATUS & CLK_STATUS_PLLSTB_Msk));
    CLK->CLKSEL0 &= (~CLK_CLKSEL0_HCLKSEL_Msk);
    CLK->CLKSEL0 |= CLK_CLKSEL0_HCLKSEL_PLL;

    /* Update system core clock */
    PllClock        = PLL_CLOCK;            // PLL
    SystemCoreClock = PLL_CLOCK / 1;        // HCLK
    CyclesPerUs     = PLL_CLOCK / 1000000;  // For CLK_SysTickDelay()

    /* Enable UART module clock */
    CLK->APBCLK0 |= CLK_APBCLK0_UART0CKEN_Msk;

    /* Select UART module clock source as HXT and UART module clock divider as 1 */
    CLK->CLKSEL1 &= ~CLK_CLKSEL1_UARTSEL_Msk;
    CLK->CLKSEL1 |= CLK_CLKSEL1_UARTSEL_HXT;
    CLK->CLKDIV0 &= ~CLK_CLKDIV0_UARTDIV_Msk;
    CLK->CLKDIV0 |= CLK_CLKDIV0_UART(1);

    /* Enable SC0 module clock */
    CLK->APBCLK1 |= CLK_APBCLK1_SC0CKEN_Msk;//CLK_EnableModuleClock(SC0_MODULE);

    /* Select SC0 module clock source */
    CLK->CLKSEL3 &= ~CLK_CLKSEL3_SC0SEL_Msk;
    CLK->CLKSEL3 |= CLK_CLKSEL3_SC0SEL_PLL;
    CLK->CLKDIV1 &= ~CLK_CLKDIV1_SC0DIV_Msk;
    CLK->CLKDIV1 |= CLK_CLKDIV1_SC0(1);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PD multi-function pins for UART0 RXD and TXD */
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD0MFP_Msk | SYS_GPD_MFPL_PD1MFP_Msk);
    SYS->GPD_MFPL |= (SYS_GPD_MFPL_PD0MFP_UART0_RXD | SYS_GPD_MFPL_PD1MFP_UART0_TXD);

    /* Set GPA multi-function pins for SC UART mode */
    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA0MFP_Msk | SYS_GPA_MFPL_PA1MFP_Msk);
    SYS->GPA_MFPL |= (SYS_GPA_MFPL_PA0MFP_SC0_CLK | SYS_GPA_MFPL_PA1MFP_SC0_DAT);

}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART IP */
    SYS->IPRST1 |=  SYS_IPRST1_UART0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_UART0RST_Msk;

    /* Configure UART0 and set UART0 baud rate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HXT, 115200);
    UART0->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}

/*---------------------------------------------------------------------------------------------------------*/
/* Enable smartcard uart interface                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
uint32_t SCUART_Open(SC_T* sc, uint32_t u32baudrate)
{
    uint32_t u32ClkSrc, u32Clk, u32Div;

    u32ClkSrc = (CLK->CLKSEL3 & CLK_CLKSEL3_SC0SEL_Msk) >> CLK_CLKSEL3_SC0SEL_Pos;

    if(u32ClkSrc == 0)
        u32Clk = __HXT;
    else if(u32ClkSrc == 1)
        u32Clk = CLK_GetPLLClockFreq();
    else
        u32Clk = __HIRC;

    u32Clk /= ((CLK->CLKDIV1 & CLK_CLKDIV1_SC0DIV_Msk) + 1);

    // Calculate divider for target baudrate
    u32Div = (u32Clk + (u32baudrate >> 1) - 1) / u32baudrate - 1;

    sc->CTL = SC_CTL_SCEN_Msk | SC_CTL_NSB_Msk;  // Enable smartcard interface and stop bit = 1
    sc->UARTCTL = SCUART_CHAR_LEN_8 | SCUART_PARITY_NONE | SC_UARTCTL_UARTEN_Msk; // Enable UART mode, disable parity and 8 bit per character
    sc->ETUCTL = u32Div;

    return(u32Clk / u32Div);
}

/*---------------------------------------------------------------------------------------------------------*/
/* Get Sc UART clock                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
static uint32_t SCUART_GetClock(SC_T *sc)
{
    uint32_t u32ClkSrc, u32Num, u32Clk;

    if(sc == SC0)
        u32Num = 0;
#if 0 /* M4521 has only one SC interface */
    else if(sc == SC1)
        u32Num = 1;
    else if(sc == SC2)
        u32Num = 2;
    else if(sc == SC3)
        u32Num = 3;
    else if(sc == SC4)
        u32Num = 4;
    else if(sc == SC5)
        u32Intf = 5;
#endif
    else
        return FALSE;

    u32ClkSrc = (CLK->CLKSEL3 >> (2 * u32Num)) & CLK_CLKSEL3_SC0SEL_Msk;

    // Get smartcard module clock
    if(u32ClkSrc == 0)
        u32Clk = __HXT;
    else if(u32ClkSrc == 1)
        u32Clk = CLK_GetPLLClockFreq();
    else if(u32ClkSrc == 2)
    {
        SystemCoreClockUpdate();
        if(CLK->CLKSEL0 & CLK_CLKSEL0_PCLK0SEL_Msk)
            u32Clk = SystemCoreClock / 2;
        else
            u32Clk = SystemCoreClock;
    }
    else
        u32Clk = __HIRC;

#if 0 /* M4521 has only one SC interface */
    if(u32Num < 4)
    {
        u32Clk /= (((CLK->CLKDIV1 >> (8 * u32Num)) & CLK_CLKDIV1_SC0DIV_Msk) + 1);
    }
    else
    {
        u32Clk /= (((CLK->CLKDIV2 >> (8 * (u32Num - 4))) & CLK_CLKDIV2_SC4DIV_Msk) + 1);
    }
#else
    u32Clk /= (((CLK->CLKDIV1 >> (8 * u32Num)) & CLK_CLKDIV1_SC0DIV_Msk) + 1);
#endif

    return u32Clk;
}

/*---------------------------------------------------------------------------------------------------------*/
/* Write data to Sc UART interface                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
uint32_t SCUART_Write(SC_T* sc, uint8_t *pu8TxBuf, uint32_t u32WriteBytes)
{
    uint32_t u32Count;
    /* Baudrate * (start bit + 8-bit data + 1-bit parity + 2-bit stop) */
    uint32_t u32Delay = (SystemCoreClock / SCUART_GetClock(sc)) * sc->ETUCTL * 12, i;

    for(u32Count = 0; u32Count != u32WriteBytes; u32Count++)
    {
        while(SCUART_GET_TX_FULL(sc))  // Wait 'til FIFO not full
        {
            /* Block longer than expected. Maybe some interrupt disable SCUART clock? */
            if(i++ > u32Delay)
            {
                g_SCUART_i32ErrCode = SCUART_TIMEOUT_ERR;
                return u32Count;
            }
        }
        sc->DAT = pu8TxBuf[u32Count];    // Write 1 byte to FIFO
    }
    return u32Count;
}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/

int main(void)
{

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Init UART0 for printf */
    UART0_Init();

    printf("This sample code demos smartcard interface UART mode\n");
    printf("Please connect SC0 CLK pin(PA.0) with SC0 I/O pin(PA.1)\n");
    printf("Hit any key to continue\n");
    getchar();

    // Open smartcard interface 0 in UART mode.
    SCUART_Open(SC0, 115200);
    // Enable receive interrupt
    SCUART_ENABLE_INT(SC0, SC_INTEN_RDAIEN_Msk);
    NVIC_EnableIRQ(SC0_IRQn);

    /* Write data to Sc UART interface */
    SCUART_Write(SC0, au8TxBuf, sizeof(au8TxBuf));

    while(1);
}
/*** (C) COPYRIGHT 2022 Nuvoton Technology Corp. ***/
