/**************************************************************************//**
 * @file     FMC.h
 * @version  V2.1
 * $Revision: 19 $
 * $Date: 15/08/11 10:26a $
 * @brief    M4521 Flash Memory Controller Driver Header File
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#ifndef __FMC_H__
#define __FMC_H__

#include "M4521.h"

#ifdef __cplusplus
extern "C"
{
#endif


/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup FMC_Driver FMC Driver
  @{
*/

/** @addtogroup FMC_EXPORTED_CONSTANTS FMC Exported Constants
  @{
*/

/*---------------------------------------------------------------------------------------------------------*/
/* Global constant definitions                                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
#define ISBEN   0

/*---------------------------------------------------------------------------------------------------------*/
/* Define Base Address                                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
#define FMC_APROM_BASE          0x00000000UL    /*!< APROM  Base Address         */
#define FMC_LDROM_BASE          0x00100000UL    /*!< LDROM  Base Address         */
#define FMC_SPROM_BASE          0x00200000UL    /*!< SPROM  Base Address         */
#define FMC_CONFIG_BASE         0x00300000UL    /*!< CONFIG Base Address         */

#define FMC_CONFIG0_ADDR        (FMC_CONFIG_BASE)       /*!< CONFIG 0 Address */
#define FMC_CONFIG1_ADDR        (FMC_CONFIG_BASE + 4)   /*!< CONFIG 1 Address */


#define FMC_FLASH_PAGE_SIZE     0x800           /*!< Flash Page Size (2048 Bytes) */
#define FMC_LDROM_SIZE          0x1000          /*!< LDROM Size (4 kBytes)       */

/*---------------------------------------------------------------------------------------------------------*/
/*  ISPCTL constant definitions                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
#define FMC_ISPCTL_BS_LDROM     0x2     /*!< ISPCTL setting to select to boot from LDROM */
#define FMC_ISPCTL_BS_APROM     0x0     /*!< ISPCTL setting to select to boot from APROM */

/*---------------------------------------------------------------------------------------------------------*/
/*  ISPCMD constant definitions                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
#define FMC_ISPCMD_READ         0x00     /*!< ISP Command: Read Flash               */
#define FMC_ISPCMD_PROGRAM      0x21     /*!< ISP Command: 32-bit Program Flash     */
#define FMC_ISPCMD_WRITE_8      0x61     /*!< ISP Command: 64-bit program Flash     */
#define FMC_ISPCMD_PAGE_ERASE   0x22     /*!< ISP Command: Page Erase Flash         */
#define FMC_ISPCMD_READ_CID     0x0B     /*!< ISP Command: Read Company ID          */
#define FMC_ISPCMD_READ_UID     0x04     /*!< ISP Command: Read Unique ID           */
#define FMC_ISPCMD_READ_DID     0x0C     /*!< ISP Command: Read Device ID           */
#define FMC_ISPCMD_VECMAP       0x2E     /*!< ISP Command: Set vector mapping       */
#define FMC_ISPCMD_CHECKSUM     0x0D     /*!< ISP Command: Read Checksum            */
#define FMC_ISPCMD_CAL_CHECKSUM 0x2D     /*!< ISP Command: Run Check Calculation    */
#define FMC_ISPCMD_MULTI_PROG   0x27     /*!< ISP Command: Flash Multi-Word Program */

/*---------------------------------------------------------------------------------------------------------*/
/*  FTCTL constant definitions                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
#define FMC_FTCTL_OPTIMIZE_DISABLE      0x00       /*!< Frequency Optimize Mode disable */
#define FMC_FTCTL_OPTIMIZE_12MHZ        0x01       /*!< Frequency Optimize Mode <= 12Mhz */
#define FMC_FTCTL_OPTIMIZE_36MHZ        0x02       /*!< Frequency Optimize Mode <= 36Mhz */
#define FMC_FTCTL_OPTIMIZE_60MHZ        0x04       /*!< Frequency Optimize Mode <= 60Mhz */
#define FMC_FTCTL_OPTIMIZE_72MHZ        0x05       /*!< Frequency Optimize Mode <= 72Mhz */

#define FMC_TIMEOUT_READ        ((SystemCoreClock/10)*2) /*!< Read command time-out 100 ms         \hideinitializer */
#define FMC_TIMEOUT_WRITE       ((SystemCoreClock/10)*2) /*!< Write command time-out 100 ms        \hideinitializer */
#define FMC_TIMEOUT_ERASE       ((SystemCoreClock/10)*4) /*!< Erase command time-out 200 ms        \hideinitializer */
#define FMC_TIMEOUT_CHKSUM      (SystemCoreClock*2)      /*!< Get checksum command time-out 2 s    \hideinitializer */

/*@}*/ /* end of group FMC_EXPORTED_CONSTANTS */

/*---------------------------------------------------------------------------------------------------------*/
/*  Global variables                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
extern int32_t  g_FMC_i32ErrCode;

/** @addtogroup FMC_EXPORTED_FUNCTIONS FMC Exported Functions
  @{
*/

/*---------------------------------------------------------------------------------------------------------*/
/*  FMC Macro Definitions                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
/**
 * @brief      Enable ISP Function
 * @param      None
 * @return     None
 * @details    This function will set ISPEN bit of ISPCTL control register to enable ISP function.
 */
#define FMC_ENABLE_ISP()          (FMC->ISPCTL |=  FMC_ISPCTL_ISPEN_Msk)  /*!< Enable ISP Function  */

/**
 * @brief      Disable ISP Function
 * @param      None
 * @return     None
 * @details    This function will clear ISPEN bit of ISPCTL control register to disable ISP function.
 */
#define FMC_DISABLE_ISP()         (FMC->ISPCTL &= ~FMC_ISPCTL_ISPEN_Msk)  /*!< Disable ISP Function */

/**
 * @brief      Enable LDROM Update Function
 * @param      None
 * @return     None
 * @details    This function will set LDUEN bit of ISPCTL control register to enable LDROM update function.
 *             User needs to set LDUEN bit before they can update LDROM.
 */
#define FMC_ENABLE_LD_UPDATE()    (FMC->ISPCTL |=  FMC_ISPCTL_LDUEN_Msk)  /*!< Enable LDROM Update Function   */

/**
 * @brief      Disable LDROM Update Function
 * @param      None
 * @return     None
 * @details    This function will set ISPEN bit of ISPCTL control register to disable LDROM update function.
 */
#define FMC_DISABLE_LD_UPDATE()   (FMC->ISPCTL &= ~FMC_ISPCTL_LDUEN_Msk)  /*!< Disable LDROM Update Function  */

/**
 * @brief      Enable User Configuration Update Function
 * @param      None
 * @return     None
 * @details    This function will set CFGUEN bit of ISPCTL control register to enable User Configuration update function.
 *             User needs to set CFGUEN bit before they can update User Configuration area.
 */
#define FMC_ENABLE_CFG_UPDATE()   (FMC->ISPCTL |=  FMC_ISPCTL_CFGUEN_Msk) /*!< Enable CONFIG Update Function  */

/**
 * @brief      Disable User Configuration Update Function
 * @param      None
 * @return     None
 * @details    This function will clear CFGUEN bit of ISPCTL control register to disable User Configuration update function.
 */
#define FMC_DISABLE_CFG_UPDATE()  (FMC->ISPCTL &= ~FMC_ISPCTL_CFGUEN_Msk) /*!< Disable CONFIG Update Function */


/**
 * @brief      Enable APROM Update Function
 * @param      None
 * @return     None
 * @details    This function will set APUEN bit of ISPCTL control register to enable APROM update function.
 *             User needs to set APUEN bit before they can update APROM in APROM boot mode.
 */
#define FMC_ENABLE_AP_UPDATE()    (FMC->ISPCTL |=  FMC_ISPCTL_APUEN_Msk)  /*!< Enable APROM Update Function   */

/**
 * @brief      Disable APROM Update Function
 * @param      None
 * @return     None
 * @details    This function will clear APUEN bit of ISPCTL control register to disable APROM update function.
 */
#define FMC_DISABLE_AP_UPDATE()   (FMC->ISPCTL &= ~FMC_ISPCTL_APUEN_Msk)  /*!< Disable APROM Update Function  */

/**
 * @brief      Next Booting Selection function
 * @param[in]  x   Booting from APROM(0)/LDROM(1)
 * @return     None
 * @details    This function will set MCU next booting from LDROM/APROM.
 * @note       When use this macro, the Boot Loader booting selection MBS(CONFIG0[5]) must be set.
 */
#define FMC_SELECT_NEXT_BOOT(x)   (FMC->ISPCTL = (FMC->ISPCTL & ~FMC_ISPCTL_BS_Msk) | ((x) << FMC_ISPCTL_BS_Pos)) /*!< Select Next Booting, x = 0 or 1 */

/**
 * @brief      Get MCU Booting Status
 * @param      None
 * @return     None
 * @details    This function will get status of chip next booting from LDROM/APROM.
 */
#define FMC_GET_BOOT_STATUS()     ((FMC->ISPCTL & FMC_ISPCTL_BS_Msk)?1:0) /*!< Get MCU Booting Status */


/**
 * @brief       Set vector mapping address
 * @param[in]   u32PageAddr  The page address to remap to address 0x0. The address must be page alignment.
 * @return      To set VECMAP to remap specified page address to 0x0.
 * @retval      0   Success
 * @retval      -1  Failed
 * @details     This function is used to set VECMAP to map specified page to vector page (0x0).
 *
 * @note        VECMAP only valid when new IAP function is enabled. (CBS = 10'b or 00'b)
 * @note        Global error code g_FMC_i32ErrCode
 *              -1  Command time-out
 */
static __INLINE int32_t FMC_SetVectorPageAddr(uint32_t u32PageAddr)
{
    uint32_t  tout = FMC_TIMEOUT_WRITE;

    g_FMC_i32ErrCode = 0;
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
    g_FMC_i32ErrCode = -1;
    return -1;
}

/**
 * @brief       Get current vector mapping address.
 * @param       None
 * @return      The current vector mapping address.
 * @details     To get VECMAP value which is the page address for remapping to vector page (0x0).
 *
 * @note
 *              VECMAP only valid when new IAP function is enabled. (CBS = 10'b or 00'b)
 *
 */
static __INLINE uint32_t FMC_GetVECMAP(void)
{
    return (FMC->ISPSTS & FMC_ISPSTS_VECMAP_Msk);
}

void FMC_Open(void);
void FMC_Close(void);
void FMC_EnableAPUpdate(void);
void FMC_DisableAPUpdate(void);
void FMC_EnableConfigUpdate(void);
void FMC_DisableConfigUpdate(void);
void FMC_EnableLDUpdate(void);
void FMC_DisableLDUpdate(void);
uint32_t FMC_Read(uint32_t u32Addr);
uint32_t FMC_ReadUID(uint8_t u8Index);
uint32_t FMC_ReadCID(void);
uint32_t FMC_ReadPID(void);
uint32_t FMC_ReadUCID(uint32_t u32Index);
int32_t FMC_Write(uint32_t u32Addr, uint32_t u32Data);
int32_t FMC_Write8(uint32_t u32Addr, uint32_t u32Data0, uint32_t u32Data1);
int32_t FMC_Write256(uint32_t u32Addr, uint32_t *pu32Buf);
int32_t FMC_Erase(uint32_t u32Addr);
int32_t FMC_ReadConfig(uint32_t *u32Config, uint32_t u32Count);
int32_t FMC_WriteConfig(uint32_t *u32Config, uint32_t u32Count);
void FMC_SetBootSource(int32_t i32BootSrc);
int32_t FMC_GetBootSource(void);
uint32_t FMC_ReadDataFlashBaseAddr(void);
uint32_t FMC_GetCheckSum(uint32_t u32Addr, int32_t i32Size);
void FMC_EnableFreqOptimizeMode(uint32_t u32Mode);
void FMC_DisableFreqOptimizeMode(void);
/*@}*/ /* end of group FMC_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group FMC_Driver */

/*@}*/ /* end of group Standard_Driver */

#ifdef __cplusplus
}
#endif


#endif

