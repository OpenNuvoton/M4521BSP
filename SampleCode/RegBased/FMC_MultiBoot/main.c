/******************************************************************************
 * @file     main.c
 * @version  V3.00
 * $Revision: 7 $
 * $Date: 15/09/02 10:04a $
 * @brief    Show how to read/program embedded flash by ISP function.
 * @note
 * Copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "M4521.h"


#define PLLCTL_SETTING      CLK_PLLCTL_72MHz_HXT
#define PLL_CLOCK           72000000


#if !defined(__ICCARM__) && !defined(__GNUC__)
extern uint32_t Image$$RO$$Base;
#endif


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

void UART_Init()
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

static __INLINE int32_t set_vector_page_addr(uint32_t u32PageAddr)
{
    uint32_t  tout = FMC_TIMEOUT_WRITE;

    FMC->ISPCMD = FMC_ISPCMD_VECMAP; /* Set ISP Command Code */
    FMC->ISPADDR = u32PageAddr;       /* The address of specified page which will be map to address 0x0. It must be page alignment. */
    FMC->ISPTRG = 0x1;               /* Trigger to start ISP procedure */
#if ISBEN
    __ISB();
#endif                         /* To make sure ISP/CPU be Synchronized */
    while (tout-- > 0)
    {
        if (!FMC->ISPTRG)             /* Waiting for ISP Done */
            return 0;
    }
    return -1;
}

int main()
{
    uint8_t   u8Ch;
    int32_t   retval;

    /* Unlock protected registers */
    SYS_UnlockReg();

    retval = SYS_Init();
    UART_Init();

    if (retval != 0)
    {
        printf("SYS_Init failed!\n");
        while (1);
    }

    /*
        This sample code shows how to boot with different firmware images in APROM.
        In the code, VECMAP is used to implement multi-boot function. Software set VECMAP
        to remap page of VECMAP to 0x0~0x1ff.
        NOTE: VECMAP only valid when CBS = 00'b or 10'b.

        To use this sample code, please:
        1. Build all targets and download to device individually. The targets are:
            FMC_MultiBoot, RO=0x0
            FMC_Boot0, RO=0x2000
            FMC_Boot1, RO=0x4000
            FMC_Boot2, RO=0x6000
            FMC_Boot3, RO=0x8000
        2. Reset MCU to execute FMC_MultiBoot.

    */

    printf("\n\n");
    printf("+---------------------------------------------+\n");
    printf("|     Multi-Boot Sample Code(0x%08X)      |\n", FMC_GetVECMAP());
    printf("+---------------------------------------------+\n");

    /* Enable FMC ISP function */
    /* Enable ISP function */
    FMC->ISPCTL |= FMC_ISPCTL_ISPEN_Msk;

    /* Check if boot mode is "APROM with IAP" */
    if (((FMC_Read(FMC_CONFIG_BASE) & 0xC0) != 0x80) || g_FMC_i32ErrCode)
    {
        printf("\n\nPlease set boot mode as [APROM with IAP]!\n");
        while (1);
    }

#if defined(__BASE__)
    printf("Boot from 0\n");
#endif
#if defined(__BOOT0__)
    printf("Boot from 0x2000\n");
#endif
#if defined(__BOOT1__)
    printf("Boot from 0x4000\n");
#endif
#if defined(__BOOT2__)
    printf("Boot from 0x6000\n");
#endif
#if defined(__BOOT3__)
    printf("Boot from 0x8000\n");
#endif
#if defined(__LDROM__)
    printf("Boot from 0x100000\n");
#endif

#if defined(__ICCARM__) || defined(__GNUC__)
    printf("VECMAP = 0x%x\n", FMC_GetVECMAP());
#else
    printf("Current RO Base = 0x%x, VECMAP = 0x%x\n", (uint32_t)&Image$$RO$$Base, FMC_GetVECMAP());
#endif

    printf("Select one boot image: \n");
    printf("[0] Boot 0, base = 0x2000\n");
    printf("[1] Boot 1, base = 0x4000\n");
    printf("[2] Boot 2, base = 0x6000\n");
    printf("[3] Boot 3, base = 0x8000\n");
#if !defined(__GNUC__)
    printf("[4] Boot 4, base = 0x100000\n");
#endif
    printf("[Others] Boot, base = 0x0\n");

    u8Ch = getchar();

    switch (u8Ch)
    {
    case '0':
        retval = set_vector_page_addr(0x2000);
        break;

    case '1':
        retval = set_vector_page_addr(0x4000);
        break;

    case '2':
        retval = set_vector_page_addr(0x6000);
        break;

    case '3':
        retval = set_vector_page_addr(0x8000);
        break;

#if !defined(__GNUC__)
    case '4':
        retval = set_vector_page_addr(0x100000);
        break;
#endif

    default:
        retval = set_vector_page_addr(0x0);
        break;
    }

    if (retval != 0)
    {
        printf("set_vector_page_addr failed!\n");
        while (1);
    }

    /* Reset CPU only to reset to new vector page */
    SYS_ResetCPU();

    /* Reset System to reset to new vector page. */
    //NVIC_SystemReset();

    /* Disable ISP function */
    FMC_Close();

    /* Lock protected registers */
    SYS_LockReg();

    printf("\nDone\n");

    while (SYS->PDID) __WFI();
}

/*** (C) COPYRIGHT 2014~2015 Nuvoton Technology Corp. ***/
