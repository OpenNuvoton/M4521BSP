/**************************************************************************//**
 * @file     main.c
 * @brief
 *           Show how to implement a USB keyboard device.
 *           This sample code supports to use GPIO to simulate key input.
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "M4521.h"
#include "hid_kb.h"

#define CRYSTAL_LESS        1
#define TRIM_INIT           (GCR_BASE+0x12C)
/*--------------------------------------------------------------------------*/
uint8_t volatile g_u8EP2Ready = 0;
uint8_t g_u8InitialBuffer = 0;

extern uint8_t Led_Status[8];
uint32_t LED_SATUS = 0;

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


void HID_UpdateKbData(void)
{
    int32_t i;
    uint8_t *buf;
    uint32_t key = 0xF;
    static uint32_t preKey;

    if(g_u8EP2Ready)
    {
        buf = (uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP2));

        /* If PB.5 = 0, just report it is key 'a' */
        key = (PB->PIN & (1 << 5)) ? 0 : 1;

        if(key == 0)
        {
            if(g_u8InitialBuffer == 0)
            {
                for(i = 0; i < 8; i++)
                {
                    buf[i] = 0;
                }
                g_u8InitialBuffer = 1;
            }
            else
            {
                buf[2] = 0x0;
            }

            if(key != preKey)
            {
                /* Trigger to note key release */
                USBD_SET_PAYLOAD_LEN(EP2, 8);
            }
        }
        else
        {
            preKey = key;
            buf[2] = 0x04; /* Key A */
            USBD_SET_PAYLOAD_LEN(EP2, 8);
        }
    }

    if(Led_Status[0] != LED_SATUS)
    {
        if((Led_Status[0] & HID_LED_ALL) != (LED_SATUS & HID_LED_ALL))
        {
            if(Led_Status[0] & HID_LED_NumLock)
                printf("NmLK  ON, ");
            else
                printf("NmLK OFF, ");

            if(Led_Status[0] & HID_LED_CapsLock)
                printf("CapsLock  ON, ");
            else
                printf("CapsLock OFF, ");

            if(Led_Status[0] & HID_LED_ScrollLock)
                printf("ScrollLock  ON, ");
            else
                printf("ScrollLock OFF, ");

            if(Led_Status[0] & HID_LED_Compose)
                printf("Compose  ON, ");
            else
                printf("Compose OFF, ");

            if(Led_Status[0] & HID_LED_Kana)
                printf("Kana  ON\n");
            else
                printf("Kana OFF\n");
        }
        LED_SATUS = Led_Status[0];
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
#if CRYSTAL_LESS
    uint32_t u32TrimInit;
#endif
    /* Unlock protected registers */
    SYS_UnlockReg();

    SYS_Init();
    UART0_Init();

    printf("\n");
    printf("+--------------------------------------------------------+\n");
    printf("|          NuMicro USB HID Keyboard Sample Code          |\n");
    printf("+--------------------------------------------------------+\n");
    printf("If PB.5 = 0, just report it is key 'a'.\n");

    USBD_Open(&gsInfo, HID_ClassRequest, NULL);

    /* Endpoint configuration */
    HID_Init();
    USBD_Start();

#if CRYSTAL_LESS
    /* Backup init trim */
    u32TrimInit = M32(TRIM_INIT);

    /* Clear SOF */
    USBD->INTSTS = USBD_INTSTS_SOFIF_Msk;
#endif

    NVIC_EnableIRQ(USBD_IRQn);

    /* start to IN data */
    g_u8EP2Ready = 1;

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

        HID_UpdateKbData();
    }
}
/*** (C) COPYRIGHT 2022 Nuvoton Technology Corp. ***/
