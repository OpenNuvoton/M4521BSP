/**************************************************************************//**
 * @file     main.c
 * @brief
 *           Demonstrate how to implement a USB audio class device.
 *           NAU8822 is used in this sample code to play the audio data from Host.
 *           It also supports to record data from NAU8822 to Host.
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "M4521.h"
#include "usbd_audio.h"

#define CRYSTAL_LESS        1
#define TRIM_INIT           (GCR_BASE+0x12C)

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal RC 22.1184MHz clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

#if (!CRYSTAL_LESS)
    /* Enable external XTAL 12MHz clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Waiting for external XTAL clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set Flash Access Delay */
    FMC->FTCTL |= FMC_FTCTL_FOM_Msk;

    /* Set core clock */
    CLK_SetCoreClock(72000000);

    /* Select module clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HXT, CLK_CLKDIV0_UART(1));
    CLK_SetModuleClock(USBD_MODULE, 0, CLK_CLKDIV0_USB(3));
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HXT, 0);
    CLK_SetModuleClock(SPI1_MODULE, CLK_CLKSEL2_SPI1SEL_PLL, 0);

#else
    /* Set core clock */
    CLK_SetCoreClock(72000000);
    CLK_EnableXtalRC(CLK_PWRCTL_HIRC48MEN_Msk);
    CLK->CLKSEL0 |= CLK_CLKSEL0_USBCKSEL_Msk;

    /* Select module clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HIRC, CLK_CLKDIV0_UART(1));
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HIRC, 0);
    CLK_SetModuleClock(SPI1_MODULE, CLK_CLKSEL2_SPI1SEL_PLL, 0);
#endif

    /* Enable module clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(USBD_MODULE);
    CLK_EnableModuleClock(TMR0_MODULE);
    CLK_EnableModuleClock(I2C0_MODULE);
    CLK_EnableModuleClock(SPI1_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PA2, PA3 as I2C0 SDA and SCL */
    SYS->GPA_MFPL = SYS_GPA_MFPL_PA3MFP_I2C0_SCL | SYS_GPA_MFPL_PA2MFP_I2C0_SDA;

    /* Set PD.0, PD.1 as UART0 TXD, UART0 RXD */
    SYS->GPD_MFPL = SYS_GPD_MFPL_PD1MFP_UART0_TXD | SYS_GPD_MFPL_PD0MFP_UART0_RXD;

    /* GPE[13:9] : SPI1_CLK (I2S1_BCLK), SPI1_MISO (I2S1_DI), SPI1_MOSI (I2S1_DO), SPI1_SS (I2S1_LRCLK), SPI1_I2SMCLK */
    SYS->GPE_MFPH = SYS_GPE_MFPH_PE10MFP_SPI1_MISO | SYS_GPE_MFPH_PE11MFP_SPI1_MOSI | SYS_GPE_MFPH_PE12MFP_SPI1_SS | SYS_GPE_MFPH_PE13MFP_SPI1_CLK | SYS_GPE_MFPH_PE9MFP_SPI1_I2SMCLK;

    /* Set PC.1 as CLKO pin */
    SYS->GPC_MFPL = SYS_GPC_MFPL_PC1MFP_CLKO;

    /* Enable CLKO (PC.1) for monitor HCLK. CLKO = HCLK/8 Hz */

}

void UART0_Init(void)
{
    /* Reset IP */
    SYS_ResetModule(UART0_RST);
    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
}

void I2C0_Init(void)
{
    /* Open I2C0 and set clock to 100k */
    I2C_Open(I2C0, 100000);

    /* Get I2C0 Bus Clock */
    printf("I2C clock %d Hz\n", I2C_GetBusClockFreq(I2C0));
}



/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
#if CRYSTAL_LESS
    uint32_t u32TrimInit;
#endif
    int32_t i;

    /*
        This sample code is used to demo USB Audio Class + NAU8822.
        User can define PLAY_RATE in usbd_audio.h to support 48000Hz, 32000Hz, 16000Hz and 8000Hz.

        The audio is input from NAU8822 AUXIN.
        The audio is output by NAU8822 Headphone output.

        NAU8822 is connect with I2S1 (PA.4~PA.7) and controlled by I2C0 (PD.4, PD.5).
        NAU8822 clock source is also come from I2S1 (MCLK, PD.0).
        PC.1 is used to output clock (HCLK/8) to check HCLK frequency.

    */


    /* Unlock Protected Regsiter */
    SYS_UnlockReg();

    /* Initial system & multi-function */
    SYS_Init();

    /* Initial UART0 for debug message */
    UART0_Init();

    printf("\n");
    printf("+-------------------------------------------------------+\n");
    printf("|          NuMicro USB Audio CODEC Sample Code          |\n");
    printf("+-------------------------------------------------------+\n");

    /* Init I2C0 to access NAU8822 */
    I2C0_Init();

    /* Initialize NAU8822 codec */
    WAU8822_Setup();

    I2S_Open(SPI1, I2S_MODE_SLAVE, PLAY_RATE, I2S_DATABIT_16, I2S_STEREO, I2S_FORMAT_I2S);

    /* Set MCLK and enable MCLK */
    I2S_EnableMCLK(SPI1, 12000000);

    /* Fill dummy data to I2S TX for start I2S iteration */
    for(i = 0; i < 4; i++)
        I2S_WRITE_TX_FIFO(SPI1, 0);

    /* Start I2S play iteration */
    I2S_EnableInt(SPI1, I2S_FIFO_TXTH_INT_MASK | I2S_FIFO_RXTH_INT_MASK);

    USBD_Open(&gsInfo, UAC_ClassRequest, (SET_INTERFACE_REQ)UAC_SetInterface);
    /* Endpoint configuration */
    UAC_Init();
    USBD_Start();
#if CRYSTAL_LESS
    /* Backup init trim */
    u32TrimInit = M32(TRIM_INIT);

    /* Waiting for USB bus stable */
    USBD->INTSTS = USBD_INTSTS_SOFIF_Msk;
    while((USBD->INTSTS & USBD_INTSTS_SOFIF_Msk) == 0);

    /* Enable USB crystal-less */
    SYS->IRC48MTCTL |= (SYS_IRC48MTCTL_REFCKSEL_Msk | 0x1);
#endif
    NVIC_EnableIRQ(USBD_IRQn);
    NVIC_EnableIRQ(SPI1_IRQn);

    /* SPI (I2S) interrupt has higher frequency then USBD interrupt.
       Therefore, we need to set SPI (I2S) with higher priority to avoid
       SPI (I2S) interrupt pending too long time when USBD interrupt happen. */
    NVIC_SetPriority(USBD_IRQn, 3);
    NVIC_SetPriority(SPI1_IRQn, 2);


    while(SYS->PDID)
    {
        uint8_t ch;
        uint32_t u32Reg, u32Data;
        extern int32_t kbhit(void);

#if CRYSTAL_LESS
        /* Re-start crystal-less when any error found */
        if (SYS->IRC48MTISTS & (SYS_IRC48MTISTS_TFAILIF_Msk | SYS_IRC48MTISTS_CLKERRIF_Msk))
        {
            SYS->IRC48MTISTS = SYS_IRC48MTISTS_TFAILIF_Msk | SYS_IRC48MTISTS_CLKERRIF_Msk;

            /* Init TRIM */
            M32(TRIM_INIT) = u32TrimInit;

            /* Waiting for USB bus stable */
            USBD->INTSTS = USBD_INTSTS_SOFIF_Msk;
            while((USBD->INTSTS & USBD_INTSTS_SOFIF_Msk) == 0);

            /* Re-enable crystal-less */
            SYS->IRC48MTCTL |= (SYS_IRC48MTCTL_REFCKSEL_Msk | 0x1);
            //printf("USB trim fail. Just retry. SYS->HIRCTSTS = 0x%x, SYS->HIRCTCTL = 0x%x\n", SYS->HIRCTSTS, SYS->HIRCTCTL);
        }
#endif
        /* Adjust codec sampling rate to synch with USB. The adjustment range is +-0.005% */
        AdjFreq();

        /* Set audio volume according USB volume control settings */
        VolumnControl();

        /* User can change audio codec settings by I2C at run-time if necessary */
        if(!kbhit())
        {
            printf("\nEnter codec setting:\n");
            // Get Register number
            ch = getchar();
            u32Reg = ch - '0';
            ch = getchar();
            u32Reg = u32Reg * 10 + (ch - '0');
            printf("%d\n", u32Reg);

            // Get data
            ch = getchar();
            u32Data = (ch >= '0' && ch <= '9') ? ch - '0' : ch - 'a' + 10;
            ch = getchar();
            u32Data = u32Data * 16 + ((ch >= '0' && ch <= '9') ? ch - '0' : ch - 'a' + 10);
            ch = getchar();
            u32Data = u32Data * 16 + ((ch >= '0' && ch <= '9') ? ch - '0' : ch - 'a' + 10);
            printf("%03x\n", u32Data);
            I2C_WriteWAU8822(u32Reg,  u32Data);
        }

    }
}
/*** (C) COPYRIGHT 2022 Nuvoton Technology Corp. ***/
