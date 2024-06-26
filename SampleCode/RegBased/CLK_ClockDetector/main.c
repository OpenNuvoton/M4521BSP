/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * $Revision: 11 $
 * $Date: 15/09/02 10:03a $
 * @brief    Show the usage of clock fail detector and clock frequency monitor function.
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include "stdio.h"
#include "M4521.h"

#define PLLCTL_SETTING  CLK_PLLCTL_72MHz_HXT
#define PLL_CLOCK       72000000


extern char GetChar(void);

/*---------------------------------------------------------------------------------------------------------*/
/*  Clock Fail Detector IRQ Handler                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
void CLKFAIL_IRQHandler(void)
{
    uint32_t u32Reg;
    uint32_t u32TimeOutCnt;

    /* Unlock protected registers */
    SYS_UnlockReg();

    u32Reg = CLK->CLKDSTS;

    if(u32Reg & CLK_CLKDSTS_HXTFIF_Msk)
    {
        /* HCLK is switched to HIRC automatically if HXT clock fail interrupt is happened */
        printf("HXT Clock is stopped!      HCLK is switched to HIRC.\n");

        /* Disable HXT clock fail interrupt */
        CLK->CLKDCTL &= ~(CLK_CLKDCTL_HXTFDEN_Msk | CLK_CLKDCTL_HXTFIEN_Msk);

        /* Write 1 to clear HXT Clock fail interrupt flag */
        CLK->CLKDSTS = CLK_CLKDSTS_HXTFIF_Msk;
    }

    if(u32Reg & CLK_CLKDSTS_LXTFIF_Msk)
    {
        /* LXT clock fail interrupt is happened */
        printf("LXT Clock is stopped!\n");

        /* Disable HXT clock fail interrupt */
        CLK->CLKDCTL &= ~(CLK_CLKDCTL_LXTFIEN_Msk | CLK_CLKDCTL_LXTFDEN_Msk);

        /* Write 1 to clear LXT Clock fail interrupt flag */
        CLK->CLKDSTS = CLK_CLKDSTS_LXTFIF_Msk;
    }

    if(u32Reg & CLK_CLKDSTS_HXTFQIF_Msk)
    {
        /* HCLK should be switched to HIRC if HXT clock frequency monitor interrupt is happened */
        CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;

        /* Waiting for HIRC clock ready */
        u32TimeOutCnt = SystemCoreClock / 2;
        while((CLK->STATUS & CLK_STATUS_HIRCSTB_Msk) != CLK_STATUS_HIRCSTB_Msk)
        {
            if(--u32TimeOutCnt == 0)
            {
                printf("HCLK switched to HIRC fail!\n");
                break;
            }
        }

        CLK->CLKSEL0 &= (~CLK_CLKSEL0_HCLKSEL_Msk) ;
        printf("HXT Frequency is abnormal! HCLK is switched to HIRC.\n");

        /* Disable HXT clock frequency monitor interrupt */
        CLK->CLKDCTL &= ~(CLK_CLKDCTL_HXTFQDEN_Msk | CLK_CLKDCTL_HXTFQIEN_Msk);

        /* Write 1 to clear HXT Clock frequency monitor interrupt */
        CLK->CLKDSTS = CLK_CLKDSTS_HXTFQIF_Msk;
    }

    /* Lock protected registers */
    SYS_LockReg();

}

int32_t SYS_Init(void)
{
    uint32_t u32TimeOutCnt;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable HIRC clock */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;

    /* Waiting for HIRC clock ready */
    u32TimeOutCnt = SystemCoreClock / 2;
    while((CLK->STATUS & CLK_STATUS_HIRCSTB_Msk) != CLK_STATUS_HIRCSTB_Msk)
    {
        if(--u32TimeOutCnt == 0)
            return CLK_TIMEOUT_ERR;
    }

    /* Enable HXT clock */
    CLK->PWRCTL |= CLK_PWRCTL_HXTEN_Msk;

    /* Waiting for HXT clock ready */
    u32TimeOutCnt = SystemCoreClock / 2;
    while((CLK->STATUS & CLK_STATUS_HXTSTB_Msk) != CLK_STATUS_HXTSTB_Msk)
    {
        if(--u32TimeOutCnt == 0)
            return CLK_TIMEOUT_ERR;
    }

    /* Enable LXT clock */
    CLK->PWRCTL |= CLK_PWRCTL_LXTEN_Msk;

    /* Waiting for LXT clock ready */
    u32TimeOutCnt = SystemCoreClock / 2;
    while((CLK->STATUS & CLK_STATUS_LXTSTB_Msk) != CLK_STATUS_LXTSTB_Msk)
    {
        if(--u32TimeOutCnt == 0)
            return CLK_TIMEOUT_ERR;
    }

    /* Select HCLK clock source as HIRC and HCLK clock divider as 1 */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | CLK_CLKSEL0_HCLKSEL_HIRC;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_HCLKDIV_Msk)) | CLK_CLKDIV0_HCLK(1);

    /* Set PLL to Power-down mode */
    CLK->PLLCTL |= CLK_PLLCTL_PD_Msk;

    /* Set core clock as PLL_CLOCK from PLL */
    CLK->PLLCTL = PLLCTL_SETTING;

    /* Waiting for PLL_clock ready */
    u32TimeOutCnt = SystemCoreClock / 2;
    while((CLK->STATUS & CLK_STATUS_PLLSTB_Msk) != CLK_STATUS_PLLSTB_Msk)
    {
        if(--u32TimeOutCnt == 0)
            return CLK_TIMEOUT_ERR;
    }

    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | CLK_CLKSEL0_HCLKSEL_PLL;

    /* Update System Core Clock */
    PllClock        = PLL_CLOCK;            // PLL
    SystemCoreClock = PLL_CLOCK / 1;        // HCLK
    CyclesPerUs     = PLL_CLOCK / 1000000;  // For CLK_SysTickDelay()

    /* Enable UART module clock */
    CLK->APBCLK0 |= CLK_APBCLK0_UART0CKEN_Msk;

    /* Select UART module clock source as HIRC and UART module clock divider as 1 */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & (~CLK_CLKSEL1_UARTSEL_Msk)) | CLK_CLKSEL1_UARTSEL_HIRC;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_UARTDIV_Msk)) | CLK_CLKDIV0_UART(1);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PD multi-function pins for UART0 RXD(PD.0) and TXD(PD.1) */
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD0MFP_Msk | SYS_GPD_MFPL_PD1MFP_Msk);
    SYS->GPD_MFPL |= (SYS_GPD_MFPL_PD0MFP_UART0_RXD | SYS_GPD_MFPL_PD1MFP_UART0_TXD);

    /* Set PC multi-function pins for CLKO(PC.1) */
    SYS->GPC_MFPL &= ~SYS_GPC_MFPL_PC1MFP_Msk;
    SYS->GPC_MFPL |= SYS_GPC_MFPL_PC1MFP_CLKO;

    /* Set PF multi-function pins for X32_OUT(PF.0) and X32_IN(PF.1) */
    SYS->GPF_MFPL &= ~(SYS_GPF_MFPL_PF0MFP_Msk | SYS_GPF_MFPL_PF1MFP_Msk);
    SYS->GPF_MFPL |= SYS_GPF_MFPL_PF0MFP_X32_OUT | SYS_GPF_MFPL_PF1MFP_X32_IN;

    return 0;
}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART0 */
    SYS->IPRST1 |=  SYS_IPRST1_UART0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_UART0RST_Msk;

    /* Configure UART0 and set UART0 baud rate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC, 115200);
    UART0->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    int32_t  retval;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    retval = SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    if (retval != 0)
    {
        printf("SYS_Init failed!\n");
        while (1);
    }

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+-------------------------------------------------------------+\n");
    printf("|               M4521 Clock Detector Sample Code              |\n");
    printf("+-------------------------------------------------------------+\n");
    printf("| 1. HXT clock fail interrupt will happen if HXT is stopped.  |\n");
    printf("|    HCLK clock source will be switched from HXT to HIRC.     |\n");
    printf("| 2. LXT clock fail interrupt will happen if LXT is stopped.  |\n");
    printf("+-------------------------------------------------------------+\n");
    printf("\nStop HXT or LXT to test.\n\n");

    /* Enable clock output, select CLKO clock source as HCLK and set clock output frequency is HCLK/4.
       HCLK clock source will be switched to HIRC if HXT stop and HCLK clock source is from HXT.
       You can check if HCLK clock source is switched to HIRC by clock output pin output frequency.
    */

    /* Enable CKO clock source */
    CLK->APBCLK0 |= CLK_APBCLK0_CLKOCKEN_Msk;

    /* CKO = clock source / 2^(1 + 1) */
    CLK->CLKOCTL = CLK_CLKOCTL_CLKOEN_Msk | (1);

    /* Select CLKO clock source as HCLK */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & (~CLK_CLKSEL1_CLKOSEL_Msk)) | CLK_CLKSEL1_CLKOSEL_HCLK;

    /* Set the HXT clock frequency monitor upper and lower boundary value.
       The upper boundary value should be more than 512*(HXT/HIRC).
       The low boundary value should be less than 512*(HXT/HIRC).
    */
    CLK->CDUPB = 278;
    CLK->CDLOWB = 276;

    /* Set clock fail detector function enabled and interrupt enabled */
    CLK->CLKDCTL = CLK_CLKDCTL_HXTFDEN_Msk |
                   CLK_CLKDCTL_HXTFIEN_Msk |
                   CLK_CLKDCTL_LXTFDEN_Msk |
                   CLK_CLKDCTL_LXTFIEN_Msk |
                   CLK_CLKDCTL_HXTFQDEN_Msk |
                   CLK_CLKDCTL_HXTFQIEN_Msk;

    /* Enable clock fail detector interrupt */
    NVIC_EnableIRQ(CKFAIL_IRQn);

    /* Wait for clock fail detector interrupt happened */
    while(1);

}
/*** (C) COPYRIGHT 2022 Nuvoton Technology Corp. ***/
