/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 15/09/02 10:03a $
 * @brief
 *           Use PDMA to implement Ping-Pong buffer by scatter-gather mode(memory to memory).
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "M4521.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Macro, type and constant definitions                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
#define PLLCTL_SETTING  CLK_PLLCTL_72MHz_HXT
#define PLL_CLOCK           72000000

uint32_t PDMA_TEST_COUNT = 50;
uint32_t g_au32SrcArray0[1] = {0x55555555};
uint32_t g_au32SrcArray1[1] = {0xAAAAAAAA};
uint32_t g_au32DestArray[1];
uint32_t volatile g_u32IsTestOver = 0;
uint32_t volatile g_u32TransferredCount = 0;
uint32_t g_u32DMAConfig = 0;
static uint32_t s_u32TableIndex = 0;

typedef struct dma_desc_t
{
    uint32_t ctl;
    uint32_t src;
    uint32_t dest;
    uint32_t offset;
} DMA_DESC_T;

DMA_DESC_T DMA_DESC[2];

/**
 * @brief       DMA IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The DMA default IRQ, declared in startup_M4521.s.
 */
void PDMA_IRQHandler(void)
{
    //if (PDMA->TDSTS == 0x20)
    {
        /* Reload PDMA description table configuration */
        DMA_DESC[s_u32TableIndex].ctl = g_u32DMAConfig;
        s_u32TableIndex ^= 1;
        g_u32TransferredCount++;

        if (g_u32TransferredCount >= PDMA_TEST_COUNT)
        {
            /* Set PDMA into stop mode by description table */
            DMA_DESC[0].ctl &= ~PDMA_DSCT_CTL_OPMODE_Msk;
            DMA_DESC[1].ctl &= ~PDMA_DSCT_CTL_OPMODE_Msk;
            g_u32IsTestOver = 1;
            PDMA_CLR_EMPTY_FLAG(PDMA_SCATSTS_TEMPTYFn_Msk);
        }
        PDMA_CLR_TD_FLAG(PDMA_TDSTS_TDIFn_Msk);
    }
}

int32_t SYS_Init(void)
{
    uint32_t u32Timeout;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock (Internal RC 22.1184MHz) */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;

    /* Waiting for HIRC clock ready */
    u32Timeout = SystemCoreClock / 10;
    while (!(CLK->STATUS & CLK_STATUS_HIRCSTB_Msk) && (u32Timeout-- > 0));
    if (!(CLK->STATUS & CLK_STATUS_HIRCSTB_Msk))
        return -1;

    /* Select HCLK clock source as HIRC and and HCLK clock divider as 1 */
    CLK->CLKSEL0 &= ~CLK_CLKSEL0_HCLKSEL_Msk;
    CLK->CLKSEL0 |= CLK_CLKSEL0_HCLKSEL_HIRC;
    CLK->CLKDIV0 &= ~CLK_CLKDIV0_HCLKDIV_Msk;
    CLK->CLKDIV0 |= CLK_CLKDIV0_HCLK(1);

    /* Enable HXT clock (external XTAL 12MHz) */
    CLK->PWRCTL |= CLK_PWRCTL_HXTEN_Msk;

    /* Waiting for HXT clock ready */
    u32Timeout = SystemCoreClock / 10;
    while (!(CLK->STATUS & CLK_STATUS_HXTSTB_Msk) && (u32Timeout-- > 0));
    if (!(CLK->STATUS & CLK_STATUS_HXTSTB_Msk))
        return -1;

    /* Set core clock as PLL_CLOCK from PLL */
    CLK->PLLCTL = PLLCTL_SETTING;
    u32Timeout = SystemCoreClock / 10;
    while (!(CLK->STATUS & CLK_STATUS_PLLSTB_Msk) && (u32Timeout-- > 0));
    if (!(CLK->STATUS & CLK_STATUS_PLLSTB_Msk))
        return -1;

    CLK->CLKSEL0 &= (~CLK_CLKSEL0_HCLKSEL_Msk);
    CLK->CLKSEL0 |= CLK_CLKSEL0_HCLKSEL_PLL;

    /* Update System Core Clock */
    PllClock        = PLL_CLOCK;            // PLL
    SystemCoreClock = PLL_CLOCK / 1;        // HCLK
    CyclesPerUs     = PLL_CLOCK / 1000000;  // For CLK_SysTickDelay()

    /* Enable UART module clock */
    CLK->APBCLK0 |= CLK_APBCLK0_UART0CKEN_Msk;

    /* Select UART module clock source as HXT and UART module clock divider as 1 */
    CLK->CLKSEL1 &= ~CLK_CLKSEL1_UARTSEL_Msk;
    CLK->CLKSEL1 |= CLK_CLKSEL1_UARTSEL_HXT;

    /* IP clock source */
    CLK->AHBCLK |= CLK_AHBCLK_PDMACKEN_Msk;


    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PD multi-function pins for UART0 RXD, TXD and */
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD0MFP_Msk | SYS_GPD_MFPL_PD1MFP_Msk);
    SYS->GPD_MFPL |= (SYS_GPD_MFPL_PD0MFP_UART0_RXD | SYS_GPD_MFPL_PD1MFP_UART0_TXD);

    return 0;
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
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    uint32_t u32Timeout;
    int32_t retval;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    retval = SYS_Init();

    /* Lock protected registers */
    /* If user want to write protected register, please issue SYS_UnlockReg() to unlock protected register. */
    SYS_LockReg();

    /* Init UART for printf */
    UART0_Init();

    if (retval != 0)
    {
        printf("SYS_Init failed!\n");
        while (1);
    }

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);

    printf("+-----------------------------------------------------------------------+ \n");
    printf("|    M4521 PDMA Driver Ping-Pong Buffer Sample Code (Scatter-gather)     | \n");
    printf("+-----------------------------------------------------------------------+ \n");

    /* Scatter-Gather description table configuration in SRAM */
    g_u32DMAConfig = (0 << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_32 | PDMA_SAR_FIX | PDMA_DAR_FIX | PDMA_BURST_1 | PDMA_REQ_BURST | PDMA_OP_SCATTER;
    /*-----------------------------------------------------------------------------------------------------------------------------------------------------------*/
    /* Note: PDMA_REQ_BURST is only supported in memory-to-memory transfer mode.
             PDMA transfer type should be set as PDMA_REQ_SINGLE in memory-to-peripheral and peripheral-to-memory transfer mode,
             then above code will be modified as follows:
             g_u32DMAConfig = (0 << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_32 | PDMA_SAR_FIX | PDMA_DAR_FIX | PDMA_BURST_1 | PDMA_REQ_SINGLE | PDMA_OP_SCATTER; */
    /*-----------------------------------------------------------------------------------------------------------------------------------------------------------*/

    /* Description table 1 */
    DMA_DESC[0].ctl = g_u32DMAConfig;
    DMA_DESC[0].src = (uint32_t)g_au32SrcArray0; /* Ping-Pong buffer 1 */
    DMA_DESC[0].dest = (uint32_t)&g_au32DestArray[0];
    DMA_DESC[0].offset = (uint32_t)&DMA_DESC[1] - (PDMA->SCATBA); /* next operation table is table 2 */

    /* Description table 2 */
    DMA_DESC[1].ctl = g_u32DMAConfig;
    DMA_DESC[1].src = (uint32_t)g_au32SrcArray1; /* Ping-Pong buffer 2 */
    DMA_DESC[1].dest = (uint32_t)&g_au32DestArray[0];
    DMA_DESC[1].offset = (uint32_t)&DMA_DESC[0] - (PDMA->SCATBA); /* next operation table is table 1 */

    /* The operation of the two description table will transfer data to the same destination buffer form different source in sequence.
       And operation sequence will be table 1 -> table 2-> table 1 -> table 2 -> table 1 -> ... -> until PDMA configuration doesn't be reloaded. */

    /* Open Channel 5 */
    PDMA->CHCTL |= BIT5;

    /* Set transfer mode as memory to memory */
    PDMA->REQSEL4_7 = (PDMA->REQSEL4_7 & ~PDMA_REQSEL4_7_REQSRC5_Msk) | (PDMA_MEM << PDMA_REQSEL4_7_REQSRC5_Pos);

    /* Enable Scatter Gather mode and set the description table address in SRAM */
    PDMA->DSCT[5].CTL = PDMA_OP_SCATTER;
    PDMA->DSCT[5].NEXT = (uint32_t)&DMA_DESC[0] - (PDMA->SCATBA);

    /* Enable transfer done interrupt */
    PDMA->INTEN |= BIT5;
    NVIC_EnableIRQ(PDMA_IRQn);
    g_u32IsTestOver = 0;

    /* Start PDMA operatin */
    PDMA->SWREQ = BIT5;

    u32Timeout = SystemCoreClock;
    while(1)
    {
        u32Timeout--;
        if(g_u32IsTestOver == 1)
        {
            g_u32IsTestOver = 0;
            printf("test done...\n");

            /* Close channel 5 */
            PDMA->CHCTL &= ~BIT5;
            while(1);
        }

        if(u32Timeout == 0)
        {
            printf("transfer timeout\n");
            while(1);
        }
    }
}
/*** (C) COPYRIGHT 2022 Nuvoton Technology Corp. ***/
