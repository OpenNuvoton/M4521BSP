/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 5 $
 * $Date: 15/09/02 10:03a $
 * @brief    Show how to read/program embedded flash by ISP function.
 * @note
 * Copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "M4521.h"

#define PLLCTL_SETTING      CLK_PLLCTL_72MHz_HXT
#define PLL_CLOCK           72000000


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

void UART0_Init(void)
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

static int32_t flash_read(uint32_t u32Addr, uint32_t *u32Data)
{
    int32_t  tout = FMC_TIMEOUT_READ;

    FMC->ISPCMD = FMC_ISPCMD_READ;
    FMC->ISPADDR = u32Addr;
    FMC->ISPDAT = 0;
    FMC->ISPTRG = 0x1;
#if ISBEN
    __ISB();
#endif
    while ((tout-- > 0) && (FMC->ISPTRG)) {}
    if (tout <= 0)
    {
        *u32Data = 0xFFFFFFFF;
        return -1;
    }
    *u32Data = FMC->ISPDAT;
    return 0;
}

int32_t fmc_read_uid(uint32_t u32Index, uint32_t *u32Data)
{
    uint32_t  tout = FMC_TIMEOUT_READ;

    FMC->ISPCMD = FMC_ISPCMD_READ_UID;          /* Set ISP Command Code */
    FMC->ISPADDR = (0x04 * u32Index) + 0x10;     /* The UCID is at offset 0x10 with word alignment. */
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;         /* Trigger to start ISP procedure */
#if ISBEN
    __ISB();
#endif                                     /* To make sure ISP/CPU be Synchronized */
    while (tout-- > 0)
    {
        if (!(FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk))  /* Waiting for ISP Done */
        {
            *u32Data = FMC->ISPDAT;
            return 0;
        }
    }
    return -1;
}

static int32_t flash_write(uint32_t u32Addr, uint32_t u32Data)
{
    int32_t  tout = FMC_TIMEOUT_WRITE;

    FMC->ISPCMD = FMC_ISPCMD_PROGRAM;
    FMC->ISPADDR = u32Addr;
    FMC->ISPDAT = u32Data;
    FMC->ISPTRG = 0x1;
#if ISBEN
    __ISB();
#endif
    while ((tout-- > 0) && (FMC->ISPTRG)) {}
    if ((tout <= 0) || (FMC->ISPSTS & FMC_ISPSTS_ISPFF_Msk))
    {
        FMC->ISPSTS |= FMC_ISPSTS_ISPFF_Msk;
        return -1;
    }
    return 0;
}

static int32_t flash_erase(uint32_t u32Addr)
{
    int32_t  tout = FMC_TIMEOUT_ERASE;

    FMC->ISPCMD = FMC_ISPCMD_PAGE_ERASE;
    FMC->ISPADDR = u32Addr;
    FMC->ISPTRG = 0x1;
#if ISBEN
    __ISB();
#endif
    while ((tout-- > 0) && (FMC->ISPTRG)) {}
    if ((tout <= 0) || (FMC->ISPCTL & FMC_ISPCTL_ISPFF_Msk))
    {
        FMC->ISPCTL |= FMC_ISPCTL_ISPFF_Msk;
        return -1;
    }
    return 0;
}

int main()
{
    char *cBootMode[] = {"LDROM+IAP", "LDROM", "APROM+IAP", "APROM"};
    uint32_t u32CBS;
    uint32_t u32Data, u32RData;
    uint32_t u32Addr;
    uint32_t u32Cfg0, u32Cfg1;
    int32_t  retval;

    /* Disable register write-protection function */
    SYS_UnlockReg();

    /* Initial clocks and multi-functions */
    retval = SYS_Init();

    /* Initial UART */
    UART0_Init();

    if (retval != 0)
    {
        printf("SYS_Init failed!\n");
        while (1);
    }

    printf("\n\n");
    printf("+----------------------------------------+\n");
    printf("|          M4521 FMC Sample Code          |\n");
    printf("+----------------------------------------+\n");

    /* Enable FMC ISP functions */
    FMC->ISPCTL |=  FMC_ISPCTL_ISPEN_Msk | FMC_ISPCTL_APUEN_Msk | FMC_ISPCTL_LDUEN_Msk | FMC_ISPCTL_CFGUEN_Msk;

    /* Check current boot mode */
    u32CBS = (FMC->ISPSTS & FMC_ISPSTS_CBS_Msk) >> FMC_ISPSTS_CBS_Pos;
    printf("  Current Boot Mode is ....................... [%s]\n", cBootMode[u32CBS]);

    /* Show the user configuration */
    if (flash_read(FMC_CONFIG_BASE, &u32Cfg0) != 0)
    {
        printf("flash_read Config0 failed!\n");
        goto err_exit;
    }
    if (flash_read(FMC_CONFIG_BASE + 4, &u32Cfg1) != 0)
    {
        printf("flash_read Config1 failed!\n");
        goto err_exit;
    }
    printf("  CFG0 ....................................... [0x%08x]\n", u32Cfg0);
    printf("  CFG1 ....................................... [0x%08x]\n", u32Cfg1);

    /* Read UID */
    if (fmc_read_uid(0, &u32RData) != 0)
    {
        printf("fmc_read_uid 0 failed!\n");
        goto err_exit;
    }
    printf("  UID[31: 0] ................................. [0x%08x]\n", u32RData);

    if (fmc_read_uid(1, &u32RData) != 0)
    {
        printf("fmc_read_uid 0 failed!\n");
        goto err_exit;
    }
    printf("  UID[63:32] ................................. [0x%08x]\n", u32RData);

    if (fmc_read_uid(2, &u32RData) != 0)
    {
        printf("fmc_read_uid 0 failed!\n");
        goto err_exit;
    }
    printf("  UID[95:64] ................................. [0x%08x]\n", u32RData);

    /* The ROM address for erase/write/read demo */
    u32Addr = 0x4000;

    /* Erase Demo */
    if (flash_erase(u32Addr) != 0)
    {
        printf("flash_erase address 0x%x failed!\n", u32Addr);
        goto err_exit;
    }

    /* Write Demo */
    u32Data = 0x12345678;
    if (flash_write(u32Addr, u32Data) != 0)
    {
        printf("flash_write address 0x%x failed!\n", u32Addr);
        goto err_exit;
    }

    /* Read Demo */
    if (flash_read(u32Addr, &u32RData) != 0)
    {
        printf("flash_read address 0x%x failed!\n", u32Addr);
        goto err_exit;
    }

    printf("  Write %08x to 0x%08x ............... ", u32Addr, u32Data);
    if(u32Data == u32RData)
        printf("[OK]\n");
    else
        printf("[FAIL]\n");

    /* Disable FMC ISP function */
    FMC->ISPCTL &=  ~FMC_ISPCTL_ISPEN_Msk;

    /* Lock protected registers */
    SYS_LockReg();

    printf("\nFMC Sample Code Completed.\n");

    while(SYS->PDID);

err_exit:
    printf("FMC error!\n");
    while(SYS->PDID);
}


