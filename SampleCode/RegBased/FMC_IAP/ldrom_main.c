/******************************************************************************
 * @file     LDROM_main.c
 * @version  V1.00
 * $Revision: 5 $
 * $Date: 15/09/02 10:04a $
 * @brief    Show how to reboot to LDROM functions from APROM.
 *           This sample code set VECMAP to LDROM and reset to re-boot to LDROM.
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "M4521.h"

#define PLLCTL_SETTING  CLK_PLLCTL_72MHz_HXT
#define PLL_CLOCK       72000000

int32_t SYS_Init(void)
{
    uint32_t   u32Timeout;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable HIRC clock */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;

    /* Waiting for HIRC clock ready */
    u32Timeout = SystemCoreClock / 10;
    while (!(CLK->STATUS & CLK_STATUS_HIRCSTB_Msk) && (u32Timeout-- > 0));
    if (!(CLK->STATUS & CLK_STATUS_HIRCSTB_Msk))
        return -1;

    /* Switch HCLK clock source to HIRC */
    CLK->CLKSEL0 = CLK_CLKSEL0_HCLKSEL_HIRC;

    /* Set PLL to Power-down mode and PLLSTB bit in CLK_STATUS register will be cleared by hardware.*/
    CLK->PLLCTL |= CLK_PLLCTL_PD_Msk;

    /* Enable HXT */
    CLK->PWRCTL |= CLK_PWRCTL_HXTEN_Msk;

    /* Waiting for HXT ready */
    u32Timeout = SystemCoreClock / 10;
    while (!(CLK->STATUS & CLK_STATUS_HXTSTB_Msk) && (u32Timeout-- > 0));
    if (!(CLK->STATUS & CLK_STATUS_HXTSTB_Msk))
        return -2;

    /* Enable PLL and Set PLL frequency */
    CLK->PLLCTL = PLLCTL_SETTING;

    /* Waiting for PLL ready */
    u32Timeout = SystemCoreClock / 10;
    while (!(CLK->STATUS & CLK_STATUS_PLLSTB_Msk) && (u32Timeout-- > 0));
    if (!(CLK->STATUS & CLK_STATUS_PLLSTB_Msk))
        return -3;

    /* Switch STCLK source to HCLK/2 and HCLK clock source to PLL */
    CLK->CLKSEL0 = CLK_CLKSEL0_STCLKSEL_HCLK_DIV2 | CLK_CLKSEL0_HCLKSEL_PLL;

    /* Enable peripheral clock */
    CLK->AHBCLK |= CLK_AHBCLK_EBICKEN_Msk;
    CLK->APBCLK0 = CLK_APBCLK0_UART0CKEN_Msk;

    /* Peripheral clock source */
    CLK->CLKSEL1 = CLK_CLKSEL1_UARTSEL_PLL;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PD multi-function pins for UART0 RXD, TXD */
    SYS->GPD_MFPL = SYS_GPD_MFPL_PD0MFP_UART0_RXD | SYS_GPD_MFPL_PD1MFP_UART0_TXD;
    return 0;
}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset IP */
    SYS->IPRST1 |=  SYS_IPRST1_UART0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_UART0RST_Msk;

    /* Configure UART0 and set UART0 Baudrate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(PLL_CLOCK, 115200);
    UART0->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}

/**
 * @brief    Routine to get a char
 * @param    None
 * @returns  Get value from UART debug port or semihost
 * @details  Wait UART debug port or semihost to input a char.
 */
static char GetChar(void)
{
    while(1)
    {
        if ((UART0->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk) == 0)
        {
            return (UART0->DAT);
        }
    }
}

extern void SendChar_ToUART(int ch);    /* in retarget_tiny.c */

static void PutString(char *str)
{
    while (*str != '\0')
    {
        SendChar_ToUART(*str++);
    }
}

static __INLINE int32_t set_vector_page_addr(uint32_t u32PageAddr)
{
    uint32_t  tout = FMC_TIMEOUT_WRITE;

    FMC->ISPCMD = FMC_ISPCMD_VECMAP; /* Set ISP Command Code */
    FMC->ISPADDR = u32PageAddr;       /* The address of specified page which will be map to address 0x0. It must be page alignment. */
    FMC->ISPTRG = 0x1;               /* Trigger to start ISP procedure */
#if ISBEN
    __ISB();
#endif                         /* To make sure ISP/CPU be Synchronized */
    while (tout-- > 0)
    {
        if (!FMC->ISPTRG)             /* Waiting for ISP Done */
            return 0;
    }
    return -1;
}

int main()
{
    int32_t   retval;

    /* Unlock protected register */
    SYS_UnlockReg();

    retval = SYS_Init();
    UART0_Init();

    if (retval != 0)
    {
        PutString("SYS_Init failed!\n");
        while (1);
    }

    PutString("\n\n");
    PutString("M4521 FMC IAP Sample Code [LDROM code]\n");

    /* Enable FMC ISP function */
    FMC->ISPCTL |= FMC_ISPCTL_ISPEN_Msk;

    PutString("\n\nPress any key to branch to APROM...\n");
    GetChar();

    PutString("\n\nChange VECMAP and branch to APROM...\n");
    UART_WAIT_TX_EMPTY(UART0);

    /* Mask all interrupt before changing VECMAP to avoid wrong interrupt handler fetched */
    __set_PRIMASK(1);

    /* Change VECMAP for booting to APROM */
    if (set_vector_page_addr(FMC_APROM_BASE) != 0)
    {
        PutString("FMC_SetVectorPageAddr failed!\n");
        while (1);
    }

    /* Reset CPU to boot to APROM */
    SYS->IPRST0 |= SYS_IPRST0_CPURST_Msk;

    while(1);
}
/*** (C) COPYRIGHT 2022 Nuvoton Technology Corp. ***/
