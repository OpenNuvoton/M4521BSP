/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * $Revision: 7 $
 * $Date: 15/09/02 10:04a $
 * @brief    Show how to access RTC spare registers.
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "M4521.h"

#define PLL_CLOCK           72000000


void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to HIRC */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable HXT and LXT-32KHz */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk | CLK_PWRCTL_LXTEN_Msk);

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk | CLK_STATUS_LXTSTB_Msk);

    /* Set core clock as PLL_CLOCK from PLL and SysTick source to HCLK/2*/
    CLK_SetCoreClock(PLL_CLOCK);
    CLK_SetSysTickClockSrc(CLK_CLKSEL0_STCLKSEL_HCLK_DIV2);

    /* Enable peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(RTC_MODULE);

    /* Peripheral clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_PLL, CLK_CLKDIV0_UART(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PD multi-function pins for UART0 RXD, TXD */
    SYS->GPD_MFPL = SYS_GPD_MFPL_PD0MFP_UART0_RXD | SYS_GPD_MFPL_PD1MFP_UART0_TXD;
}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART module */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    uint32_t i, u32ReadData;
    uint32_t u32TimeOutCount = SystemCoreClock; // 1 second timeout

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("+------------------------------------------+\n");
    printf("|    RTC Spare Register R/W Sample Code    |\n");
    printf("+------------------------------------------+\n\n");

    /* Initial RTC and stay in normal state */
    if(RTC->INIT != RTC_INIT_ACTIVE_Msk)
    {
        RTC->INIT = RTC_INIT_KEY;
        while(RTC->INIT != RTC_INIT_ACTIVE_Msk)
        {
            if(u32TimeOutCount == 0)
            {
                printf("\n RTC initial fail!!");
                printf("\n Please check h/w setting!!");
                while(1);
            }
            u32TimeOutCount--;
        }
    }

    /* Enable Spare registers access function */
    RTC_EnableSpareAccess();

    for(i = 0; i < 20; i++)
    {
        printf("Write Spare register-%02d to 0x%08X ... ", i, (0x5A5A0000 + i));
        RTC_WaitAccessEnable();
        RTC_WRITE_SPARE_REGISTER(i, 0x5A5A0000 + i);
        u32TimeOutCount = SystemCoreClock;
        while(!(RTC->SPRCTL & RTC_SPRCTL_SPRRWRDY_Msk))
        {
            if(u32TimeOutCount == 0) break;
            u32TimeOutCount--;
        }
        printf("DONE\n");
    }

    printf("\n");
    for(i = 0; i < 20; i++)
    {
        printf("Read Spare register-%02d value is ", i);
        RTC_WaitAccessEnable();
        u32ReadData = RTC_READ_SPARE_REGISTER(i);
        if(u32ReadData == (0x5A5A0000 + i))
        {
            printf("0x%08X ... PASS.\n", u32ReadData);
        }
        else
        {
            printf("0x%08X ... FAIL.\n", u32ReadData);
            while(1);
        }

        u32TimeOutCount = SystemCoreClock;
        while(!(RTC->SPRCTL & RTC_SPRCTL_SPRRWRDY_Msk))
        {
            // wait spare register stable
            if(u32TimeOutCount == 0) break;
            u32TimeOutCount--;
        }
    }

    while(1);
}
/*** (C) COPYRIGHT 2022 Nuvoton Technology Corp. ***/
