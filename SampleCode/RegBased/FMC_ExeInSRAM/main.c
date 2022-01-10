/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 8 $
 * $Date: 15/09/02 10:03a $
 * @brief
 *           Implement a code and execute in SRAM to program embedded Flash.
 *           (Support KEIL MDK Only)
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "M4521.h"

#define PLLCTL_SETTING      CLK_PLLCTL_72MHz_HXT
#define PLL_CLOCK           72000000


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
        return -1;

    /* Enable PLL and Set PLL frequency */
    CLK->PLLCTL = PLLCTL_SETTING;

    /* Waiting for PLL ready */
    u32Timeout = SystemCoreClock / 10;
    while (!(CLK->STATUS & CLK_STATUS_PLLSTB_Msk) && (u32Timeout-- > 0));
    if (!(CLK->STATUS & CLK_STATUS_PLLSTB_Msk))
        return -1;

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

static int32_t flash_read(uint32_t u32Addr, uint32_t *u32Data)
{
    int32_t  tout = FMC_TIMEOUT_READ;

    FMC->ISPCMD = FMC_ISPCMD_READ;
    FMC->ISPADDR = u32Addr;
    FMC->ISPDAT = 0;
    FMC->ISPTRG = 0x1;
#if ISBEN
    __ISB();
#endif
    while ((tout-- > 0) && (FMC->ISPTRG)) {}
    if (tout <= 0)
    {
        *u32Data = 0xFFFFFFFF;
        return -1;
    }
    *u32Data = FMC->ISPDAT;
    return 0;
}

static int32_t flash_write(uint32_t u32Addr, uint32_t u32Data)
{
    int32_t  tout = FMC_TIMEOUT_WRITE;

    FMC->ISPCMD = FMC_ISPCMD_PROGRAM;
    FMC->ISPADDR = u32Addr;
    FMC->ISPDAT = u32Data;
    FMC->ISPTRG = 0x1;
#if ISBEN
    __ISB();
#endif
    while ((tout-- > 0) && (FMC->ISPTRG)) {}
    if ((tout <= 0) || (FMC->ISPSTS & FMC_ISPSTS_ISPFF_Msk))
    {
        FMC->ISPSTS |= FMC_ISPSTS_ISPFF_Msk;
        return -1;
    }
    return 0;
}

static int32_t flash_erase(uint32_t u32Addr)
{
    int32_t  tout = FMC_TIMEOUT_ERASE;

    FMC->ISPCMD = FMC_ISPCMD_PAGE_ERASE;
    FMC->ISPADDR = u32Addr;
    FMC->ISPTRG = 0x1;
#if ISBEN
    __ISB();
#endif
    while ((tout-- > 0) && (FMC->ISPTRG)) {}
    if ((tout <= 0) || (FMC->ISPCTL & FMC_ISPCTL_ISPFF_Msk))
    {
        FMC->ISPCTL |= FMC_ISPCTL_ISPFF_Msk;
        return -1;
    }
    return 0;
}

int main()
{
    uint32_t u32Data, u32RData;
    uint32_t u32Addr;
    uint32_t i;
    int32_t  retval;

    /* Disable register write-protection function */
    SYS_UnlockReg();

    /* Initial clocks and multi-functions */
    retval = SYS_Init();

    /* Initial UART */
    UART0_Init();

    if (retval != 0)
    {
        printf("SYS_Init failed!\n");
        while (1);
    }

    printf("\n\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|      FMC Write/Read code execute in SRAM Sample Code      |\n");
    printf("+-----------------------------------------------------------+\n");

    /*
       This sample code is used to demonstrate how to implement a code to execute in SRAM.
       By setting scatter loading file (scatter.scf),
       RO code is placed to 0x10000000 ~ 0x10003fff with RW is placed to 0x20004000 ~ 0x20007fff.
    */

    /* Enable FMC ISP functions */
    FMC->ISPCTL |=  FMC_ISPCTL_ISPEN_Msk | FMC_ISPCTL_APUEN_Msk;

    /* The ROM address for erase/write/read demo */
    u32Addr = 0x4000;

    if (flash_erase(u32Addr) != 0) /* Erase page */
    {
        printf("flash_erase address 0x%x failed!\n", u32Addr);
        while (1);
    }

    for(i = 0; i < 0x100; i += 4)
    {
        /* Write Demo */
        u32Data = i + 0x12345678;

        if (flash_write(u32Addr + i, u32Data) != 0)
        {
            printf("flash_write address 0x%x failed!\n", u32Addr + i);
            while (1);
        }

        if ((i & 0xf) == 0)
            printf(".");

        /* Read Demo */
        if (flash_read(u32Addr + i, &u32RData) != 0)
        {
            printf("flash_read address 0x%x failed!\n", u32Addr + i);
            while (1);
        }

        if(u32Data != u32RData)
        {
            printf("[Read/Write FAIL] 0x%x 0x%x\n", u32Data, u32RData);
            while(1);
        }
    }
    /* Disable FMC ISP function */
    FMC->ISPCTL &=  ~FMC_ISPCTL_ISPEN_Msk;

    /* Lock protected registers */
    SYS_LockReg();

    printf("\nFMC Sample Code Completed.\n");

    while(1);
}
/*** (C) COPYRIGHT 2022 Nuvoton Technology Corp. ***/
