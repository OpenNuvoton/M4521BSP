/**************************************************************************//**
 * @file     main.c
 * @brief
 *           Show how to implement a USB micro printer device.
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "M4521.h"
#include "micro_printer.h"


#define CRYSTAL_LESS        1
#define TRIM_INIT           (GCR_BASE+0x12C)
/*--------------------------------------------------------------------------*/
void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal RC 22.1184 MHz clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

#if (!CRYSTAL_LESS)
    /* Enable external XTAL 12 MHz clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Waiting for external XTAL clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set Flash Access Delay */
    FMC->FTCTL |= FMC_FTCTL_FOM_Msk;

    /* Set core clock */
    CLK_SetCoreClock(72000000);

    /* Select module clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HIRC, CLK_CLKDIV0_UART(1));
    CLK_SetModuleClock(USBD_MODULE, 0, CLK_CLKDIV0_USB(3));
#else
    /* Set core clock */
    CLK_SetCoreClock(72000000);
    CLK_EnableXtalRC(CLK_PWRCTL_HIRC48MEN_Msk);
    CLK->CLKSEL0 |= CLK_CLKSEL0_USBCKSEL_Msk;

    /* Select module clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HIRC, CLK_CLKDIV0_UART(1));
#endif

    /* Enable module clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(USBD_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set GPD multi-function pins for UART0 RXD, TXD  */
    SYS->GPD_MFPL = SYS_GPD_MFPL_PD0MFP_UART0_RXD | SYS_GPD_MFPL_PD1MFP_UART0_TXD;

}


void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset IP */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
}



/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
#if CRYSTAL_LESS
    uint32_t u32TrimInit;
#endif
    uint8_t Str[9];

    /* Unlock protected registers */
    SYS_UnlockReg();

    SYS_Init();
    UART0_Init();

    printf("\n");
    printf("+--------------------------------------------------------+\n");
    printf("|         NuMicro USB Micro Printer Sample Code          |\n");
    printf("+--------------------------------------------------------+\n");

    USBD_Open(&gsInfo, PTR_ClassRequest, NULL);

    /* Endpoint configuration */
    PTR_Init();
    USBD_Start();

#if CRYSTAL_LESS
    /* Backup init trim */
    u32TrimInit = M32(TRIM_INIT);

    /* Clear SOF */
    USBD->INTSTS = USBD_INTSTS_SOFIF_Msk;
#endif

    NVIC_EnableIRQ(USBD_IRQn);

    PB->MODE = 0x5000;   // PB.6, PB.7 output mode

    while(1)
    {
#if CRYSTAL_LESS
        /* Start USB trim if it is not enabled */
        if ((SYS->IRC48MTCTL & SYS_IRC48MTCTL_FREQSEL_Msk) != 1)
        {
            /* Start USB trim only when SOF */
            if (USBD->INTSTS & USBD_INTSTS_SOFIF_Msk)
            {
                /* Clear SOF */
                USBD->INTSTS = USBD_INTSTS_SOFIF_Msk;
                /* re-enable crystal-less */
                SYS->IRC48MTCTL |= (SYS_IRC48MTCTL_REFCKSEL_Msk | 0x1);
            }
        }

        /* Disable USB trim when error */
        if (SYS->IRC48MTISTS & (SYS_IRC48MTISTS_TFAILIF_Msk | SYS_IRC48MTISTS_CLKERRIF_Msk))
        {
            /* Init TRIM */
            M32(TRIM_INIT) = u32TrimInit;
            /* disable crystall-less */
            SYS->IRC48MTCTL = 0;
            /* clear error flags */
            SYS->IRC48MTISTS = SYS_IRC48MTISTS_TFAILIF_Msk | SYS_IRC48MTISTS_CLKERRIF_Msk;
            /* clear SOF */
            USBD->INTSTS = USBD_INTSTS_SOFIF_Msk;
        }
#endif

        CLK_SysTickDelay(2000);   // delay
        if(++Str[1] > 0x39)
            Str[1] = 0x30;      // increase 1 to 10 than reset to 0
        PB->DOUT ^= 0x40; // PB.6
    }
}
/*** (C) COPYRIGHT 2022 Nuvoton Technology Corp. ***/
