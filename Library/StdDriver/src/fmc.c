/**************************************************************************//**
 * @file     fmc.c
 * @version  V3.00
 * $Revision: 7 $
 * $Date: 15/08/11 10:26a $
 * @brief    M4521 FMC driver source file
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "M4521.h"

/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup FMC_Driver FMC Driver
  @{
*/


/** @addtogroup FMC_EXPORTED_FUNCTIONS FMC Exported Functions
  @{
*/

int32_t  g_FMC_i32ErrCode;

/**
  * @brief      Set boot source from LDROM or APROM after next software reset
  * @param[in]  i32BootSrc
  *                         1: Boot from LDROM,
  *                         0: Boot from APROM
  * @return   None
  * @details  This function is used to switch APROM boot or LDROM boot. User need to call
  *           FMC_SetBootSource to select boot source first, then use CPU reset or
  *           System Reset Request to reset system.
  */
void FMC_SetBootSource(int32_t i32BootSrc)
{
    if (i32BootSrc)
        FMC->ISPCTL |= FMC_ISPCTL_BS_Msk; /* Boot from LDROM */
    else
        FMC->ISPCTL &= ~FMC_ISPCTL_BS_Msk;/* Boot from APROM */
}

/**
  * @brief    Disable ISP Functions
  * @param    None
  * @return   None
  * @details  This function will clear ISPEN bit of ISPCTL to disable ISP function
  */
void FMC_Close(void)
{
    FMC->ISPCTL &= ~FMC_ISPCTL_ISPEN_Msk;
}

/**
  * @brief    Disable APROM update function
  * @param    None
  * @return   None
  *
  * @details  Disable APROM update function will forbid APROM programming when boot form APROM.
  *           APROM update is default to be disable.
  */
void FMC_DisableAPUpdate(void)
{
    FMC->ISPCTL &= ~FMC_ISPCTL_APUEN_Msk;
}

/**
  * @brief    Disable User Configuration update function
  * @param    None
  * @return   None
  *
  * @details  Disable User Configuration update function will forbid User Configuration programming.
  *           User Configuration update is default to be disable.
  */
void FMC_DisableConfigUpdate(void)
{
    FMC->ISPCTL &= ~FMC_ISPCTL_CFGUEN_Msk;
}

/**
  * @brief    Disable LDROM update function
  * @param    None
  * @return   None

  * @details  Disable LDROM update function will forbid LDROM programming.
  *           LDROM update is default to be disable.
  */
void FMC_DisableLDUpdate(void)
{
    FMC->ISPCTL &= ~FMC_ISPCTL_LDUEN_Msk;
}

/**
  * @brief    Enable APROM update function
  * @param    None
  * @return   None
  *
  * @details  Enable APROM to be able to program when boot from APROM.
  */
void FMC_EnableAPUpdate(void)
{
    FMC->ISPCTL |= FMC_ISPCTL_APUEN_Msk;
}

/**
  * @brief    Enable User Configuration update function
  * @param    None
  * @return   None
  *
  * @details  Enable User Configuration to be able to program.
  */
void FMC_EnableConfigUpdate(void)
{
    FMC->ISPCTL |= FMC_ISPCTL_CFGUEN_Msk;
}

/**
  * @brief    Enable LDROM update function
  * @param    None
  * @return   None
  *
  * @details  Enable LDROM to be able to program.
  */
void FMC_EnableLDUpdate(void)
{
    FMC->ISPCTL |= FMC_ISPCTL_LDUEN_Msk;
}

/**
  * @brief    Get the current boot source
  * @param    None
  * @retval   0 This chip is currently booting from APROM
  * @retval   1 This chip is currently booting from LDROM
  *
  * @note     This function only show the boot source.
  *           User need to read ISPSTA register to know if IAP mode supported or not in relative boot.
  */
int32_t FMC_GetBootSource(void)
{
    if (FMC->ISPCTL & FMC_ISPCTL_BS_Msk)
        return 1;
    else
        return 0;
}

/**
  * @brief    Enable FMC ISP function
  * @param    None
  * @return   None
  * @details  ISPEN bit of ISPCTL must be set before we can use ISP commands.
  *           Therefore, To use all FMC function APIs, user needs to call FMC_Open() first to enable ISP functions.
  * @note     ISP functions are write-protected. user also needs to unlock it by calling SYS_UnlockReg() before using all ISP functions.
  */
void FMC_Open(void)
{
    FMC->ISPCTL |=  FMC_ISPCTL_ISPEN_Msk;
}

/**
  * @brief    Get the base address of Data Flash if enabled.
  * @param    None
  * @return   The base address of Data Flash
  * @details  This function is used to return the base address of Data Flash.
  */
uint32_t FMC_ReadDataFlashBaseAddr(void)
{
    return FMC->DFBA;
}

/**
 * @brief       Read 32-bit Data from specified address of flash
 * @param[in]   u32Addr  Flash address include APROM, LDROM, Data Flash, and CONFIG
 * @return      The word data read from specified flash address.
 *              Return 0xFFFFFFFF if read failed.
 * @note        Global error code g_FMC_i32ErrCode
 *              -1  Read time-out
 * @details     To read word data from Flash include APROM, LDROM, Data Flash, and CONFIG.
 */
uint32_t FMC_Read(uint32_t u32Addr)
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
        g_FMC_i32ErrCode = -1;
        return 0xFFFFFFFF;
    }
    return FMC->ISPDAT;
}

/**
 * @brief       Read Unique ID
 * @param[in]   u8Index  UID index. 0 = UID[31:0], 1 = UID[63:32], 2 = UID[95:64]
 * @return      The 32-bit unique ID data of specified UID index.
 * @details     To read out 96-bit Unique ID.
 */
uint32_t FMC_ReadUID(uint8_t u8Index)
{
    uint32_t  tout = FMC_TIMEOUT_READ;

    g_FMC_i32ErrCode = 0;
    FMC->ISPCMD = FMC_ISPCMD_READ_UID;
    FMC->ISPADDR = (u8Index << 2);
    FMC->ISPDAT = 0;
    FMC->ISPTRG = 0x1;
#if ISBEN
    __ISB();
#endif
    while (tout-- > 0)
    {
        if (!(FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk))  /* Waiting for ISP Done */
            return FMC->ISPDAT;
    }
    g_FMC_i32ErrCode = -1;
    return 0xFFFFFFFF;
}

/**
  * @brief    Read company ID
  * @param    None
  * @return   The company ID (32-bit)
  * @details  The company ID of Nuvoton is fixed to be 0xDA
  *
  * @note     Global error code g_FMC_i32ErrCode
  *           -1  Read time-out
  */
uint32_t FMC_ReadCID(void)
{
    uint32_t  tout = FMC_TIMEOUT_READ;

    g_FMC_i32ErrCode = 0;
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
            if (FMC->ISPDAT != 0xDA)
                g_FMC_i32ErrCode = -1;
            return FMC->ISPDAT;
        }
    }
    g_FMC_i32ErrCode = -1;
    return 0xFFFFFFFF;
}

/**
  * @brief    Read product ID
  * @param    None
  * @return   The product ID (32-bit)
  * @details  This function is used to read product ID.
  *
  * @note     Global error code g_FMC_i32ErrCode
  *           -1  Read time-out
  */
uint32_t FMC_ReadPID(void)
{
    uint32_t  tout = FMC_TIMEOUT_READ;

    g_FMC_i32ErrCode = 0;
    FMC->ISPCMD = FMC_ISPCMD_READ_DID;          /* Set ISP Command Code */
    FMC->ISPADDR = 0x04;                        /* Must keep 0x4 when read PID */
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;         /* Trigger to start ISP procedure */
#if ISBEN
    __ISB();
#endif                                           /* To make sure ISP/CPU be Synchronized */
    while (tout-- > 0)
    {
        if (!(FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk))  /* Waiting for ISP Done */
            return FMC->ISPDAT;
    }
    g_FMC_i32ErrCode = -1;
    return 0xFFFFFFFF;
}

/**
  * @brief      To read UCID
  * @param[in]  u32Index    Index of the UCID to read. u32Index must be 0, 1, 2, or 3.
  * @return     The UCID of specified index
  * @details    This function is used to read unique chip ID (UCID).
  *
  * @note     Global error code g_FMC_i32ErrCode
  *           -1  Read time-out
  */
uint32_t FMC_ReadUCID(uint32_t u32Index)
{
    uint32_t  tout = FMC_TIMEOUT_READ;

    g_FMC_i32ErrCode = 0;
    FMC->ISPCMD = FMC_ISPCMD_READ_UID;          /* Set ISP Command Code */
    FMC->ISPADDR = (0x04 * u32Index) + 0x10;     /* The UCID is at offset 0x10 with word alignment. */
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;         /* Trigger to start ISP procedure */
#if ISBEN
    __ISB();
#endif                                     /* To make sure ISP/CPU be Synchronized */
    while (tout-- > 0)
    {
        if (!(FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk))  /* Waiting for ISP Done */
            return FMC->ISPDAT;
    }
    g_FMC_i32ErrCode = -1;
    return 0xFFFFFFFF;
}

/**
 * @brief      Program 32-bit data into specified address of flash
 * @param[in]  u32Addr  Flash address include APROM, LDROM, Data Flash, and CONFIG
 * @param[in]  u32Data  32-bit Data to program
 * @return     0   Success
 * @return     -1  Program Failed
 *
 * @note       Global error code g_FMC_i32ErrCode
 *             -1  Program failed or time-out
 *
 * @details    To program word data into Flash include APROM, LDROM, Data Flash, and CONFIG.
 *             The corresponding functions in CONFIG are listed in FMC section of Technical Reference Manual.
 *
 */
int32_t FMC_Write(uint32_t u32Addr, uint32_t u32Data)
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
    if (tout <= 0)
    {
        g_FMC_i32ErrCode = -1;
        return -1;
    }
    if (FMC->ISPSTS & FMC_ISPSTS_ISPFF_Msk)
    {
        FMC->ISPSTS |= FMC_ISPSTS_ISPFF_Msk;
        g_FMC_i32ErrCode = -1;
        return -1;
    }
    return 0;
}

/**
 * @brief      Program 64-bit data into specified address of flash
 * @param[in]  u32Addr  Flash address include APROM, LDROM, Data Flash, and CONFIG
 * @param[in]  u32Data0 32-bit Data to program
 * @param[in]  u32Data1 32-bit Data to program
 * @return     0   Success
 * @return     -1  Program Failed
 *
 * @note       Global error code g_FMC_i32ErrCode
 *             -1  Program failed or time-out
 *
 * @details    To program two words data into Flash include APROM, LDROM, Data Flash, and CONFIG.
 *             The corresponding functions in CONFIG are listed in FMC section of Technical Reference Manual.
 *
 */
int32_t FMC_Write8(uint32_t u32Addr, uint32_t u32Data0, uint32_t u32Data1)
{
    int32_t  tout = FMC_TIMEOUT_WRITE;

    FMC->ISPCMD = FMC_ISPCMD_WRITE_8;
    FMC->ISPADDR = u32Addr;
    FMC->MPDAT0 = u32Data0;
    FMC->MPDAT1 = u32Data1;
    FMC->ISPTRG = 0x1;
#if ISBEN
    __ISB();
#endif
    while ((tout-- > 0) && (FMC->ISPTRG)) {}
    if (tout <= 0)
    {
        g_FMC_i32ErrCode = -1;
        return -1;
    }
    if (FMC->ISPSTS & FMC_ISPSTS_ISPFF_Msk)
    {
        FMC->ISPSTS |= FMC_ISPSTS_ISPFF_Msk;
        g_FMC_i32ErrCode = -1;
        return -1;
    }
    return 0;
}

/**
 * @brief      Program Multi-Word data into specified address of flash
 * @param[in]  u32Addr  Flash address include APROM, LDROM, Data Flash, and CONFIG
 * @param[in]  pu32Buf  A data pointer is point to a data buffer start address;
 * @return     0   Success
 * @return     -1  Program Failed
 *
 * @note       Global error code g_FMC_i32ErrCode
 *             -1  Program failed or time-out
 *
 * @details    To program multi-words data into Flash include APROM, LDROM, Data Flash, and CONFIG.
 *             The corresponding functions in CONFIG are listed in FMC section of Technical Reference Manual.
 *
 */
int32_t FMC_Write256(uint32_t u32Addr, uint32_t *pu32Buf)
{
    int32_t i, idx;
    volatile uint32_t *pu32IspData;
    int32_t  tout;

    g_FMC_i32ErrCode = 0;
    idx = 0;
    FMC->ISPCMD = FMC_ISPCMD_MULTI_PROG;
    FMC->ISPADDR = u32Addr;

retrigger:

    FMC->MPDAT0 = pu32Buf[idx + 0];
    FMC->MPDAT1 = pu32Buf[idx + 1];
    FMC->MPDAT2 = pu32Buf[idx + 2];
    FMC->MPDAT3 = pu32Buf[idx + 3];
    FMC->ISPTRG = 0x1;

    pu32IspData = &FMC->MPDAT0;
    idx += 4;

    for(i = idx; i < 256 / 4; i += 4) // Max data length is 256 bytes (256/4 words)
    {
        __set_PRIMASK(1); // Mask interrupt to avoid status check coherence error
        tout = FMC_TIMEOUT_WRITE;
        do
        {
            if ((FMC->MPSTS & FMC_MPSTS_MPBUSY_Msk) == 0)
            {
                __set_PRIMASK(0);
                FMC->ISPADDR = FMC->MPADDR & (~0xful);
                idx = (FMC->ISPADDR - u32Addr) / 4;
                goto retrigger;
            }
        }
        while ((tout-- > 0) && (FMC->MPSTS & (3 << FMC_MPSTS_D0_Pos)));

        if (tout <= 0)
        {
            g_FMC_i32ErrCode = -1;
            __set_PRIMASK(0);
            return -1;
        }

        tout = FMC_TIMEOUT_WRITE;
        // Update new data for D0
        pu32IspData[0] = pu32Buf[i];
        pu32IspData[1] = pu32Buf[i + 1];
        do
        {
            if ((FMC->MPSTS & FMC_MPSTS_MPBUSY_Msk) == 0)
            {
                __set_PRIMASK(0);
                FMC->ISPADDR = FMC->MPADDR & (~0xful);
                idx = (FMC->ISPADDR - u32Addr) / 4;
                goto retrigger;
            }
        }
        while ((tout-- > 0) && (FMC->MPSTS & (3 << FMC_MPSTS_D2_Pos)));

        if (tout <= 0)
        {
            g_FMC_i32ErrCode = -1;
            __set_PRIMASK(0);
            return -1;
        }

        // Update new data for D2
        pu32IspData[2] = pu32Buf[i + 2];
        pu32IspData[3] = pu32Buf[i + 3];
        __set_PRIMASK(0);
    }

    tout = FMC_TIMEOUT_WRITE;
    while ((tout-- > 0) && (FMC->ISPSTS & FMC_ISPSTS_ISPBUSY_Msk)) { }
    if (tout <= 0)
    {
        g_FMC_i32ErrCode = -1;
        return -1;
    }
    return 0;
}

/**
 * @brief      Flash page erase
 *
 * @param[in]  u32Addr  Flash address including APROM, LDROM, Data Flash, and CONFIG
 * @details    To do flash page erase. The target address could be APROM, LDROM, Data Flash, or CONFIG.
 *             The page size is 2048 bytes.
 *
 * @retval     0   Success
 * @retval     -1  Erase/program/read/verify failed
 *
 * @note       Global error code g_FMC_i32ErrCode
 *             < 0  Errors caused by erase/program/read failed or time-out
 *
 */
int32_t FMC_Erase(uint32_t u32Addr)
{
    int32_t  tout = FMC_TIMEOUT_ERASE;

    g_FMC_i32ErrCode = 0;
    FMC->ISPCMD = FMC_ISPCMD_PAGE_ERASE;
    FMC->ISPADDR = u32Addr;
    FMC->ISPTRG = 0x1;
#if ISBEN
    __ISB();
#endif
    while ((tout-- > 0) && (FMC->ISPTRG)) {}
    if (tout <= 0)
    {
        g_FMC_i32ErrCode = -1;
        return -1;
    }

    if (FMC->ISPCTL & FMC_ISPCTL_ISPFF_Msk)
    {
        FMC->ISPCTL |= FMC_ISPCTL_ISPFF_Msk;
        g_FMC_i32ErrCode = -1;
        return -1;
    }
    return 0;
}

/**
  * @brief       Read the User Configuration words.
  * @param[out]  u32Config  The word buffer to store the User Configuration data.
  * @param[in]   u32Count   The word count to be read.
  * @retval       0 Success
  * @retval      -1 Failed
  *
  * @details     This function is used to read the settings of user configuration.
  *              if u32Count = 1, Only CONFIG0 will be returned to the buffer specified by u32Config.
  *              if u32Count = 2, Both CONFIG0 and CONFIG1 will be returned.
  */
int32_t FMC_ReadConfig(uint32_t *u32Config, uint32_t u32Count)
{
    int32_t i;

    for(i = 0; i < u32Count; i++)
        u32Config[i] = FMC_Read(FMC_CONFIG_BASE + i * 4);

    return 0;
}


/**
  * @brief      Write User Configuration
  *
  * @param[in]  u32Config The word buffer to store the User Configuration data.
  * @param[in]  u32Count The word count to program to User Configuration.
  * @retval     0 Success
  * @retval    -1 Failed
  * @details    User must enable User Configuration update before writing it.
  *             User must erase User Configuration before writing it.
  *             User Configuration is also be page erase. User needs to backup necessary data
  *             before erase User Configuration.
  */
int32_t FMC_WriteConfig(uint32_t *u32Config, uint32_t u32Count)
{
    int32_t i;

    for(i = 0; i < u32Count; i++)
    {
        FMC_Write(FMC_CONFIG_BASE + i * 4, u32Config[i]);
        if(FMC_Read(FMC_CONFIG_BASE + i * 4) != u32Config[i])
            return -1;
    }

    return 0;
}

/**
 * @brief       Get Flash Checksum
 * @param[in]   u32Addr    Specific flash start address
 * @param[in]   i32Size    Specific a size of Flash area
 * @return      A checksum value of a flash block.
 *
 * @details     To get VECMAP value which is the page address for remapping to vector page (0x0).
 *
 * @note        Global error code g_FMC_i32ErrCode
 *              -1  Command time-out
 */
uint32_t FMC_GetCheckSum(uint32_t u32Addr, int32_t i32Size)
{
    uint32_t  tout;

    g_FMC_i32ErrCode = 0;
    FMC->ISPCMD = FMC_ISPCMD_CAL_CHECKSUM;
    FMC->ISPADDR = u32Addr;
    FMC->ISPDAT = i32Size;
    FMC->ISPTRG = 0x1;
#if ISBEN
    __ISB();
#endif
    tout = FMC_TIMEOUT_CHKSUM;
    while (tout-- > 0)
    {
        if (!FMC->ISPTRG)             /* Waiting for ISP Done */
            goto chksum;
    }
    g_FMC_i32ErrCode = -1;
    return 0xFFFFFFFF;

chksum:
    FMC->ISPCMD = FMC_ISPCMD_CHECKSUM;
    FMC->ISPTRG = 0x1;
#if ISBEN
    __ISB();
#endif
    tout = FMC_TIMEOUT_CHKSUM;
    while (tout-- > 0)
    {
        if (!FMC->ISPTRG)             /* Waiting for ISP Done */
            return FMC->ISPDAT;
    }
    g_FMC_i32ErrCode = -1;
    return 0xFFFFFFFF;
}

/**
 * @brief      Enable Flash Access Frequency  Optimization Mode
 * @param[in]  u32Mode   Optimize flash access cycle mode
 *             - \ref FMC_FTCTL_OPTIMIZE_DISABLE
 *             - \ref FMC_FTCTL_OPTIMIZE_12MHZ
 *             - \ref FMC_FTCTL_OPTIMIZE_36MHZ
 *             - \ref FMC_FTCTL_OPTIMIZE_60MHZ
 *             - \ref FMC_FTCTL_OPTIMIZE_72MHZ
 * @return     None
 * @details    This function will set FOM bit fields of FTCTL register to set flash access frequency optimization mode.
 * @note       The flash optimization mode (FOM) bits are write protect.
 */
void FMC_EnableFreqOptimizeMode(uint32_t u32Mode)
{
    FMC->FTCTL &= ~FMC_FTCTL_FOM_Msk;
    FMC->FTCTL |= (u32Mode << FMC_FTCTL_FOM_Pos);
}

/**
 * @brief      Disable Flash Access Frequency  Optimization Mode
 * @param      None
 * @return     None
 * @details    This function will clear FOM bit fields of FTCTL register to disable flash access frequency optimization mode.
 * @note       The flash optimization mode (FOM) bits are write protect.
 */
void FMC_DisableFreqOptimizeMode(void)
{
    FMC->FTCTL &= ~FMC_FTCTL_FOM_Msk;
}

/*@}*/ /* end of group FMC_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group FMC_Driver */

/*@}*/ /* end of group Standard_Driver */

/*** (C) COPYRIGHT 2022 Nuvoton Technology Corp. ***/

