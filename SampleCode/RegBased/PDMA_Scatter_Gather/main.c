/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 10 $
 * $Date: 15/11/24 3:17p $
 * @brief
 *           Use PDMA channel 5 to transfer data from memory to memory by scatter-gather mode.
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

uint32_t PDMA_TEST_LENGTH = 64;
uint8_t SrcArray[256];
uint8_t DestArray0[256];
uint8_t DestArray1[256];

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
    while (!(CLK->STATUS & CLK_STATUS_HIRCSTB_Msk) && (u32Timeout-- > 0));
    if (!(CLK->STATUS & CLK_STATUS_HIRCSTB_Msk))
        return -1;

    /* Set core clock as PLL_CLOCK from PLL */
    CLK->PLLCTL = PLLCTL_SETTING;
    u32Timeout = SystemCoreClock / 10;
    while (!(CLK->STATUS & CLK_STATUS_PLLSTB_Msk) && (u32Timeout-- > 0));
    if (!(CLK->STATUS & CLK_STATUS_PLLSTB_Msk))
        return -1;

    CLK->CLKSEL0 &= (~CLK_CLKSEL0_HCLKSEL_Msk);
    CLK->CLKSEL0 |= CLK_CLKSEL0_HCLKSEL_HXT;

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
    uint32_t u32Src, u32Dst0, u32Dst1;

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
    printf("|    M4521 PDMA Memory to Memory Driver Sample Code (Scatter-gather)     | \n");
    printf("+-----------------------------------------------------------------------+ \n");

    u32Src = (uint32_t)SrcArray;
    u32Dst0 = (uint32_t)DestArray0;
    u32Dst1 = (uint32_t)DestArray1;

    /* This sample will transfer data by finished two descriptor table in sequence.(descriptor table 1 -> descriptor table 2) */

    /* Descriptor table 1 */
    /* Configure transfer count is PDMA_TEST_LENGTH, transfer width, source and destination increment size are one word(32 bit),
       transfer type is burst transfer type and the burst size is 128,
       and operation mode is scatter-gather mode */
    DMA_DESC[0].ctl = ((PDMA_TEST_LENGTH - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_32 | PDMA_SAR_INC | PDMA_DAR_INC | PDMA_REQ_BURST | PDMA_OP_SCATTER;
    DMA_DESC[0].src = u32Src;
    DMA_DESC[0].dest = u32Dst0;
    DMA_DESC[0].offset = (uint32_t)&DMA_DESC[1] - (PDMA->SCATBA); /* next operation table is table 2 */

    /* Descriptor table 2 */
    /* Configure transfer count is PDMA_TEST_LENGTH, transfer width, source and destination increment size are one word(32 bit),
       transfer type is burst transfer type and the burst size is 128,
       and operation mode is basic mode */
    DMA_DESC[1].ctl = ((PDMA_TEST_LENGTH - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_32 | PDMA_SAR_INC | PDMA_DAR_INC | PDMA_REQ_BURST | PDMA_OP_BASIC;
    DMA_DESC[1].src = u32Dst0;
    DMA_DESC[1].dest = u32Dst1;
    DMA_DESC[1].offset = 0; /* no next operation table */

    /* Open Channel 5 */
    PDMA->CHCTL |= (1 << 5);

    /* Set transfer mode as memory to memory */
    PDMA->REQSEL4_7 = (PDMA->REQSEL4_7 & ~PDMA_REQSEL4_7_REQSRC5_Msk) | (0x1F << PDMA_REQSEL4_7_REQSRC5_Pos);

    /* Enable Scatter Gather mode */
    PDMA->DSCT[5].CTL = (PDMA->DSCT[5].CTL & ~PDMA_DSCT_CTL_OPMODE_Msk) | PDMA_OP_SCATTER;

    /* Assign the first scatter-gather description table is table 1 */
    PDMA->DSCT[5].NEXT = (uint32_t)&DMA_DESC[0] - (PDMA->SCATBA);

    /* Generate a software request to trigger transfer with PDMA channel 5 */
    PDMA->SWREQ = (1 << 5);

    /* Waiting for transfer done */
    u32Timeout = SystemCoreClock;
    while( (PDMA->TRGSTS & (1 << 5)) && (u32Timeout-- > 0));
    if((PDMA->TRGSTS & (1 << 5)))
    {
        printf("transfer timeout\n");
        while(1);
    }

    printf("test done...\n");

    /* Close Channel 5 */
    PDMA->CHCTL = 0;

    while(1);
}
/*** (C) COPYRIGHT 2022 Nuvoton Technology Corp. ***/
