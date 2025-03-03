/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 16/08/30 11:47a $
 * @brief    This sample shows how to use USB Host driver to handle HID keyboard devices.
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "M4521.h"
#include "usbh_core.h"
#include "usbh_hid.h"

uint8_t  desc_buff[1024];

extern int  kbd_parse_report(HID_DEV_T *hdev, uint8_t *buf, int len);

static HID_DEV_T  *hdev_kbd = NULL;
static uint8_t    kbd_dat[8];


void Delay(uint32_t delayCnt)
{
    while(delayCnt--)
    {
        __NOP();
        __NOP();
    }
}


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

    /* Enable external XTAL 12MHz clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Waiting for external XTAL clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set Flash Access Delay */
    FMC->FTCTL |= FMC_FTCTL_FOM_Msk;

    /* Set core clock */
    CLK_SetCoreClock(72000000);

    SYS->USBPHY = (0x1 << SYS_USBPHY_USBROLE_Pos);     /* USB Host Role */

    /* Enable module clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(USBH_MODULE);

    /* Select module clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HXT, CLK_CLKDIV0_UART(1));
    CLK_SetModuleClock(USBH_MODULE, 0, CLK_CLKDIV0_USB(3));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set GPD multi-function pins for UART0 RXD and TXD */
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD0MFP_Msk | SYS_GPD_MFPL_PD1MFP_Msk);
    SYS->GPD_MFPL |= (SYS_GPD_MFPL_PD0MFP_UART0_RXD | SYS_GPD_MFPL_PD1MFP_UART0_TXD);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init USB VBUS_EN and VBUS_ST multi-function pin                                                         */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Below settings is use power switch IC to enable/disable USB Host power.
       Set PC.4 is VBUS_EN function pin and PC.3 VBUS_ST function pin             */
    SYS->GPC_MFPL &= ~(SYS_GPC_MFPL_PC4MFP_Msk | SYS_GPC_MFPL_PC3MFP_Msk);
    SYS->GPC_MFPL |=  (SYS_GPC_MFPL_PC3MFP_USB_VBUS_ST | SYS_GPC_MFPL_PC4MFP_USB_VBUS_EN);

    /* Below settings is use power switch IC to enable/disable USB Host power.
       Set PA.2 is VBUS_EN function pin and PA.3 VBUS_ST function pin             */
    // SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA2MFP_Msk | SYS_GPA_MFPL_PA3MFP_Msk);
    // SYS->GPA_MFPL |= (SYS_GPA_MFPL_PA3MFP_USB_VBUS_ST | SYS_GPA_MFPL_PA2MFP_USB_VBUS_EN);
}


void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART IP */
    SYS->IPRST1 |=  SYS_IPRST1_UART0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_UART0RST_Msk;

    /* Configure UART0 and set UART0 Baudrate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HXT, 115200);
    UART0->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}


void  int_read_callback(HID_DEV_T *hdev, uint8_t *rdata, int data_len)
{
    int  i;

    if ((hdev->bSubClassCode == 1) && (hdev->bProtocolCode == 1))
    {
        hdev_kbd = hdev;
        memcpy(kbd_dat, rdata, sizeof(kbd_dat));
    }
    else
    {
        printf("Device [0x%x,0x%x] %d bytes received =>\n",
               hdev->udev->descriptor.idVendor, hdev->udev->descriptor.idProduct, data_len);
        for(i = 0; i < data_len; i++)
            printf("0x%02x ", rdata[i]);
        printf("\n");
    }
}


int  init_hid_device(HID_DEV_T *hdev)
{
    int   i, ret;

    printf("\n\n==================================\n");
    printf("  Init HID device : 0x%x\n", (int)hdev);
    printf("  VID: 0x%x, PID: 0x%x\n\n", hdev->udev->descriptor.idVendor, hdev->udev->descriptor.idProduct);

    ret = HID_HidGetReportDescriptor(hdev, desc_buff, 1024);
    if(ret > 0)
    {
        printf("\nDump report descriptor =>\n");
        for(i = 0; i < ret; i++)
        {
            if((i % 16) == 0)
                printf("\n");
            printf("%02x ", desc_buff[i]);
        }
        printf("\n\n");
    }

    /*
     *  SET_REPORT with report ID 0, report type RT_OUTPUT.
     *  Some keyboard device require this command to trigger.
     */
    desc_buff[0] = 0;
    HID_HidSetReport(hdev, RT_OUTPUT, 0, desc_buff, 1);

    printf("\nUSBH_HidStartIntReadPipe...\n");
    if(USBH_HidStartIntReadPipe(hdev, int_read_callback) == HID_RET_OK)
    {
        printf("Interrupt in transfer started...\n");
    }
    return 0;
}

uint32_t CLK_GetUSBFreq(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Get USB Peripheral Clock                                                                                */
    /*---------------------------------------------------------------------------------------------------------*/
    /* USB Peripheral clock = PLL_CLOCK/USBDIV+1) */
    return CLK_GetPLLClockFreq()/(((CLK->CLKDIV0 & CLK_CLKDIV0_USBDIV_Msk)>>CLK_CLKDIV0_USBDIV_Pos)+1);
}

/*----------------------------------------------------------------------------
  MAIN function
 *----------------------------------------------------------------------------*/
int32_t main(void)
{
    HID_DEV_T    *hdev;

    /* Lock protected registers */
    if(SYS->REGLCTL == 1) // In end of main function, program issued CPU reset and write-protection will be disabled.
        SYS_LockReg();

    /* Unlock protected registers */
    SYS_UnlockReg();

    SYS_Init(); //In the end of SYS_Init() will issue SYS_LockReg() to lock protected register. If user want to write protected register, please issue SYS_UnlockReg() to unlock protected register.

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\n");
    printf(" System clock:   %d Hz.\n", SystemCoreClock);
    printf(" USB Host clock: %d Hz.\n", CLK_GetUSBFreq());
    printf("+---------------------------------------+\n");
    printf("|                                       |\n");
    printf("|  M4521 USB Host HID sample program    |\n");
    printf("|                                       |\n");
    printf("+---------------------------------------+\n");

    USBH_Open();

    USBH_HidInit();

    printf("Wait until any HID devices connected...\n");

    while(1)
    {
        if(USBH_ProcessHubEvents())              /* USB Host port detect polling and management */
        {
            hdev = USBH_HidGetDeviceList();
            if(hdev == NULL)
                continue;

            while(hdev != NULL)
            {
                init_hid_device(hdev);

                if(hdev != NULL)
                    hdev = hdev->next;
            }
        }

        if (hdev_kbd != NULL)
        {
            kbd_parse_report(hdev_kbd, kbd_dat, 8);
            hdev_kbd = NULL;
        }
    }
}
/*** (C) COPYRIGHT 2022 Nuvoton Technology Corp. ***/
