/******************************************************************************
 * @file     APROM_main.c
 * @version  V1.00
 * $Revision: 6 $
 * $Date: 15/09/02 10:04a $
 * @brief    Show how to reboot to LDROM functions from APROM.
 *           This sample code set VECMAP to LDROM and reset to re-boot to LDROM.
 * @note
 * Copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "M4521.h"

#define PLLCTL_SETTING       CLK_PLLCTL_72MHz_HXT
#define PLL_CLOCK            72000000

#define ERR_FLASH_READ       -1
#define ERR_FLASH_WRITE      -2
#define ERR_FLASH_ERASE      -3

typedef void (FUNC_PTR)(void);

extern uint32_t  loaderImage1Base;


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

static int32_t fmc_read_cid(uint32_t *u32Data)
{
    uint32_t  tout = FMC_TIMEOUT_READ;

    FMC->ISPCMD = FMC_ISPCMD_READ_CID;           /* Set ISP Command Code */
    FMC->ISPADDR = 0x0;                          /* Must keep 0x0 when read CID */
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;          /* Trigger to start ISP procedure */
#if ISBEN
    __ISB();
#endif                                    /* To make sure ISP/CPU be Synchronized */
    while (tout-- > 0)
    {
        if (!(FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk))  /* Waiting for ISP Done */
        {
            *u32Data = FMC->ISPDAT;
            if (*u32Data != 0xDA)
                return -1;
            return 0;
        }
    }
    return -1;
}

static int32_t fmc_read_pid(uint32_t *u32Data)
{
    uint32_t  tout = FMC_TIMEOUT_READ;

    FMC->ISPCMD = FMC_ISPCMD_READ_DID;          /* Set ISP Command Code */
    FMC->ISPADDR = 0x04;                        /* Must keep 0x4 when read PID */
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;         /* Trigger to start ISP procedure */
#if ISBEN
    __ISB();
#endif                                           /* To make sure ISP/CPU be Synchronized */
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

static int SetIAPBoot(void)
{
    uint32_t  au32Config[2];
    uint32_t  u32CBS;

    /* Read current boot mode */
    u32CBS = (FMC->ISPSTS & FMC_ISPSTS_CBS_Msk) >> FMC_ISPSTS_CBS_Pos;
    if(u32CBS & 1)
    {
        /* Modify User Configuration when it is not in IAP mode */

        printf("Modify boot mode to be \"Boot from APROM with IAP\" mode.\n");

        if (flash_read(FMC_CONFIG_BASE, &au32Config[0]) != 0)
        {
            printf("flash_read Config0 failed!\n");
            return -1;
        }
        if (flash_read(FMC_CONFIG_BASE + 4, &au32Config[1]) != 0)
        {
            printf("flash_read Config1 failed!\n");
            return -1;
        }

        if(au32Config[0] & 0x40)
        {
            FMC->ISPCTL |= FMC_ISPCTL_CFGUEN_Msk;    /* enable update of User Configuration */
            au32Config[0] &= ~0x40;

            /*  Erase and program User Configuration 0/1 */
            if (flash_erase(FMC_CONFIG_BASE) != 0)
                return ERR_FLASH_ERASE;

            if (flash_write(FMC_CONFIG_BASE, au32Config[0]) != 0)
                return ERR_FLASH_WRITE;

            if (flash_write(FMC_CONFIG_BASE+4, au32Config[1]) != 0)
                return ERR_FLASH_WRITE;

            // Perform chip reset to make new User Config take effect
            SYS->IPRST0 |= SYS_IPRST0_CHIPRST_Msk;
        }
    }
    return 0;
}

static int  LoadImage(uint32_t u32ImageBase, uint32_t u32FlashAddr, uint32_t u32MaxSize)
{
    uint32_t   i, j, u32Data, u32ImageSize, *pu32Loader;

    u32ImageSize = u32MaxSize;

    printf("Program image to flash address 0x%x...", u32FlashAddr);
    pu32Loader = (uint32_t *)u32ImageBase;
    for(i = 0; i < u32ImageSize; i += FMC_FLASH_PAGE_SIZE)
    {
        if (flash_erase(u32FlashAddr + i) != 0)
        {
            printf("flash_erase address 0x%x failed!\n", u32FlashAddr + i);
            return ERR_FLASH_ERASE;
        }

        for(j = 0; j < FMC_FLASH_PAGE_SIZE; j += 4)
        {
            if (flash_write(u32FlashAddr + i + j, pu32Loader[(i + j) / 4]) != 0)
            {
                printf("flash_write address 0x%x failed!\n", u32FlashAddr + i + j);
                return ERR_FLASH_WRITE;
            }
        }
    }
    printf("OK.\n");

    printf("Verify ...");

    /* Verify loader */
    for(i = 0; i < u32ImageSize; i += FMC_FLASH_PAGE_SIZE)
    {
        for(j = 0; j < FMC_FLASH_PAGE_SIZE; j += 4)
        {
            if (flash_read(u32FlashAddr + i + j, &u32Data) != 0)
            {
                printf("flash_read address 0x%x failed!\n", u32FlashAddr + i + j);
                return -1;
            }

            if(u32Data != pu32Loader[(i + j) / 4])
            {
                printf("data mismatch on 0x%x, [0x%x], [0x%x]\n", u32FlashAddr + i + j, u32Data, pu32Loader[(i + j) / 4]);
                return -1;
            }

            if(i + j >= u32ImageSize)
                break;
        }
    }
    printf("OK.\n");
    return 0;
}


int main()
{
    uint8_t   u8Item;
    uint32_t  u32Data;
    char *acBootMode[] = {"LDROM+IAP", "LDROM", "APROM+IAP", "APROM"};
    uint32_t  u32CBS;
    int32_t   retval;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init system clock and multi-function I/O */
    retval = SYS_Init();

    /* Init UART */
    UART0_Init();

    if (retval != 0)
    {
        printf("SYS_Init failed!\n");
        while (1);
    }

    printf("\n\n");
    printf("+----------------------------------------+\n");
    printf("|      M4521 FMC IAP Sample Code         |\n");
    printf("|           [APROM code]                 |\n");
    printf("+----------------------------------------+\n");

    /* Enable FMC ISP function */
    FMC->ISPCTL |= FMC_ISPCTL_ISPEN_Msk;

    if (SetIAPBoot() != 0)
    {
        printf("Failed to set IAP boot mode!\n");
        goto lexit;
    }

    /* Get boot mode */
    printf("  Boot Mode ............................. ");
    u32CBS = (FMC->ISPSTS & FMC_ISPSTS_CBS_Msk) >> FMC_ISPSTS_CBS_Pos;
    printf("[%s]\n", acBootMode[u32CBS]);

    if (fmc_read_cid(&u32Data) != 0)
        goto lexit;
    printf("  Company ID ............................ [0x%08x]\n", u32Data);

    if (fmc_read_pid(&u32Data) != 0)
        goto lexit;
    printf("  Product ID ............................ [0x%08x]\n", u32Data);

    /* Read User Configuration */
    if (flash_read(FMC_CONFIG_BASE, &u32Data) != 0)
        goto lexit;
    printf("  User Config 0 ......................... [0x%08x]\n", u32Data);

    if (flash_read(FMC_CONFIG_BASE + 4, &u32Data) != 0)
        goto lexit;
    printf("  User Config 1 ......................... [0x%08x]\n", u32Data);

    do
    {
        printf("\n\n\n");
        printf("+----------------------------------------+\n");
        printf("|               Select                   |\n");
        printf("+----------------------------------------+\n");
        printf("| [0] Load IAP code to LDROM             |\n");
        printf("| [1] Run IAP program (in LDROM)         |\n");
        printf("+----------------------------------------+\n");
        printf("Please select...");
        u8Item = getchar();
        printf("%c\n", u8Item);

        switch(u8Item)
        {
        case '0':
            FMC->ISPCTL |= FMC_ISPCTL_LDUEN_Msk;    /* enable update of LDROM */
            if (LoadImage((uint32_t)&loaderImage1Base,
                          FMC_LDROM_BASE, FMC_LDROM_SIZE) != 0)
            {
                printf("Load image to LDROM failed!\n");
                goto lexit;
            }
            FMC->ISPCTL &= ~FMC_ISPCTL_LDUEN_Msk;    /* disable update of LDROM */
            break;

        case '1':
            printf("\n\nChange VECMAP and branch to LDROM...\n");
            UART_WAIT_TX_EMPTY(UART0); /* To make sure all message has been print out */

            /* Mask all interrupt before changing VECMAP to avoid wrong interrupt handler fetched */
            __set_PRIMASK(1);

            /* Set VECMAP to LDROM for booting from LDROM */
            if (set_vector_page_addr(FMC_LDROM_BASE) != 0)
            {
                printf("FMC_SetVectorPageAddr failed!\n");
                goto lexit;
            }

            /* Reset CPU to boot to LDROM */
            SYS->IPRST0 |= SYS_IPRST0_CPURST_Msk;

            break;

        default :
            break;
        }
    }
    while(1);

lexit:

    /* Disable FMC ISP function */
    FMC->ISPCTL &= ~FMC_ISPCTL_ISPEN_Msk;

    /* Lock protected registers */
    SYS_LockReg();

    while(1);
}

/*** (C) COPYRIGHT 2018~2020 Nuvoton Technology Corp. ***/
