/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 15/09/02 10:05a $
 * @brief
 *           Show how to implement a USB Host with a file system to read/write a file on USB Mass Storage.
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "M4521.h"
#include "diskio.h"
#include "ff.h"
#include "usbh_core.h"
#include "usbh_umas.h"

#define DiskTest


FATFS FatFs[_VOLUMES];      /* File system object for logical drive */

#ifdef __ICCARM__
#pragma data_alignment=32
BYTE Buff[4096] ;       /* Working buffer */
#else
BYTE Buff[4096] __attribute__((aligned(32)));       /* Working buffer */
#endif

void Delay(uint32_t delayCnt)
{
    while(delayCnt--)
    {
        __NOP();
        __NOP();
    }
}

void put_rc(FRESULT rc)
{
    const TCHAR *p =
        _T("OK\0DISK_ERR\0INT_ERR\0NOT_READY\0NO_FILE\0NO_PATH\0INVALID_NAME\0")
        _T("DENIED\0EXIST\0INVALID_OBJECT\0WRITE_PROTECTED\0INVALID_DRIVE\0")
        _T("NOT_ENABLED\0NO_FILE_SYSTEM\0MKFS_ABORTED\0TIMEOUT\0LOCKED\0")
        _T("NOT_ENOUGH_CORE\0TOO_MANY_OPEN_FILES\0");
    //FRESULT i;
    uint32_t i;
    for(i = 0; (i != (UINT)rc) && *p; i++)
    {
        while(*p++) ;
    }
    printf(_T("rc=%u FR_%s\n"), (UINT)rc, p);
}

/*---------------------------------------------------------*/
/* User Provided RTC Function for FatFs module             */
/*---------------------------------------------------------*/
/* This is a real time clock service to be called from     */
/* FatFs module. Any valid time must be returned even if   */
/* the system does not support an RTC.                     */
/* This function is not required in read-only cfg.         */

unsigned long get_fattime(void)
{
    unsigned long tmr;

    tmr = 0x00000;

    return tmr;
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

static FIL file1;        /* File objects */
uint8_t u8Disk = 0;
/*----------------------------------------------------------------------------
  MAIN function
 *----------------------------------------------------------------------------*/

int32_t main(void)
{
    FRESULT res;
    UINT s2, cnt;
    uint32_t i;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init(); //In the end of SYS_Init() will issue SYS_LockReg() to lock protected register. If user want to write protected register, please issue SYS_UnlockReg() to unlock protected register.

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    /* Init USB host and prepare a root hub*/
    USBH_Open();

    /* Set USB host to support USB mass storage device*/
    USBH_MassInit();

    /*Delay for power stable*/
    Delay(0x500000);

    printf("+------------------------------------------------+\n");
    printf("|                                                |\n");
    printf("|  M4521 USB Host Mass Storage sample program    |\n");
    printf("|                                                |\n");
    printf("+------------------------------------------------+\n");

    while(1)
    {
        USBH_ProcessHubEvents();//Process hub events that include resume, suspense, plug-in, unplug and so on.
        Delay(0x500000);

        if(disk_status(0)&&u8Disk) //Check mass storage device status
        {
            put_rc(f_mount(0, NULL));//No disk
            u8Disk = 0;
            printf("\n======= No disk found =======\n\n");
        }

        if((disk_status(0)==0)&&(u8Disk==0))
        {
            put_rc(f_mount(0, &FatFs[0]));//disk found
            u8Disk = 1;
            printf("\n======= Disk found =======\n");

#ifdef DiskTest
            putchar('\n');
            res = f_open(&file1, (TCHAR*)"Test.txt", FA_OPEN_EXISTING | FA_READ);// Open "Test.txt" file for read operation
            if(res)//Check the result of FatFs function
            {
                //Fail --> Maybe connection fail or Not find "Text.txt" file

                put_rc(res);//Print fail message

                res = f_open(&file1, (TCHAR*)"Test.txt", FA_CREATE_ALWAYS | FA_WRITE);// Open "Test.txt" file for write operation
                if(res)
                {
                    //Fail
                    put_rc(res);//Print fail message
                }
                else
                {
                    //Pass
                    cnt = 512;//Set data lenght
                    for(i=0; i<cnt; i++)
                        Buff[i] = i;//Prepare data for write

                    res = f_write(&file1, Buff, cnt, &s2);//Write data into file.
                    if(res != FR_OK)
                    {
                        //Fail
                        put_rc(res);//Print fail message
                    }
                    f_close(&file1);//Close "Test.txt" file
                    printf("\nData lenght:%d\n",s2);//Print how many wrote data
                }
            }
            else
            {
                //Pass
                cnt = 512;//Set data lenght
                res = f_read(&file1, Buff, cnt, &s2);//Read data from file.
                if(res != FR_OK)
                {
                    //Fail
                    put_rc(res);//Print fail message
                }
                else
                {
                    //Pass
                    printf("Data size:%d\n",s2);//Print how many read data
                    for(i=0; i<s2; i++)
                    {
                        if((i%8) == 0 )
                            printf("\n");
                        printf("0x%2x ",Buff[i]);//Print read data
                    }
                }
                f_close(&file1);//Close "Test.txt" file
                printf("\n\nData lenght:%d\n",s2);//Print how many wrote data
                printf("Test finish.\n");//Print how many wrote data
            }
#endif
        }
    }
}
/*** (C) COPYRIGHT 2022 Nuvoton Technology Corp. ***/
