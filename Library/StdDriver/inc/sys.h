/**************************************************************************//**
 * @file     SYS.h
 * @version  V3.0
 * $Revision  1 $
 * $Date: 15/08/11 10:26a $
 * @brief    M4521 SYS Header File
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#ifndef __SYS_H__
#define __SYS_H__

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup SYS_Driver SYS Driver
  @{
*/

/** @addtogroup SYS_EXPORTED_CONSTANTS SYS Exported Constants
  @{
*/


/*---------------------------------------------------------------------------------------------------------*/
/*  Module Reset Control Resister constant definitions.                                                    */
/*---------------------------------------------------------------------------------------------------------*/
#define PDMA_RST    ((0x0<<24) | SYS_IPRST0_PDMARST_Pos )   /*!< Reset PDMA */
#define EBI_RST     ((0x0<<24) | SYS_IPRST0_EBIRST_Pos )    /*!< Reset EBI */
#define USBH_RST    ((0x0<<24) | SYS_IPRST0_USBHRST_Pos )   /*!< Reset USBH */
#define CRC_RST     ((0x0<<24) | SYS_IPRST0_CRCRST_Pos )    /*!< Reset CRC */

#define GPIO_RST    ((0x4<<24) | SYS_IPRST1_GPIORST_Pos )   /*!< Reset GPIO */
#define TMR0_RST    ((0x4<<24) | SYS_IPRST1_TMR0RST_Pos )   /*!< Reset TMR0 */
#define TMR1_RST    ((0x4<<24) | SYS_IPRST1_TMR1RST_Pos )   /*!< Reset TMR1 */
#define TMR2_RST    ((0x4<<24) | SYS_IPRST1_TMR2RST_Pos )   /*!< Reset TMR2 */
#define TMR3_RST    ((0x4<<24) | SYS_IPRST1_TMR3RST_Pos )   /*!< Reset TMR3 */
#define I2C0_RST    ((0x4<<24) | SYS_IPRST1_I2C0RST_Pos )   /*!< Reset I2C0 */
#define I2C1_RST    ((0x4<<24) | SYS_IPRST1_I2C1RST_Pos )   /*!< Reset I2C1 */
#define SPI0_RST    ((0x4<<24) | SYS_IPRST1_SPI0RST_Pos )   /*!< Reset SPI0 */
#define SPI1_RST    ((0x4<<24) | SYS_IPRST1_SPI1RST_Pos )   /*!< Reset SPI1 */
#define UART0_RST   ((0x4<<24) | SYS_IPRST1_UART0RST_Pos )  /*!< Reset UART0 */
#define UART1_RST   ((0x4<<24) | SYS_IPRST1_UART1RST_Pos )  /*!< Reset UART1 */
#define UART2_RST   ((0x4<<24) | SYS_IPRST1_UART2RST_Pos )  /*!< Reset UART2 */
#define UART3_RST   ((0x4<<24) | SYS_IPRST1_UART3RST_Pos )  /*!< Reset UART3 */
#define USBD_RST    ((0x4<<24) | SYS_IPRST1_USBDRST_Pos )   /*!< Reset USBD */
#define EADC_RST    ((0x4<<24) | SYS_IPRST1_EADCRST_Pos )   /*!< Reset EADC */

#define SC0_RST     ((0x8<<24) | SYS_IPRST2_SC0RST_Pos )    /*!< Reset SC0 */
#define PWM0_RST    ((0x8<<24) | SYS_IPRST2_PWM0RST_Pos )   /*!< Reset PWM0 */
#define PWM1_RST    ((0x8<<24) | SYS_IPRST2_PWM1RST_Pos )   /*!< Reset PWM1 */


/*---------------------------------------------------------------------------------------------------------*/
/*  Brown Out Detector Threshold Voltage Selection constant definitions.                                   */
/*---------------------------------------------------------------------------------------------------------*/
#define SYS_BODCTL_BOD_RST_EN           (1UL<<SYS_BODCTL_BODRSTEN_Pos)    /*!< Brown-out Reset Enable */
#define SYS_BODCTL_BOD_INTERRUPT_EN     (0UL<<SYS_BODCTL_BODRSTEN_Pos)    /*!< Brown-out Interrupt Enable */
#define SYS_BODCTL_BODVL_4_5V           (3UL<<SYS_BODCTL_BODVL_Pos)       /*!< Setting Brown Out Detector Threshold Voltage as 4.5V */
#define SYS_BODCTL_BODVL_3_7V           (2UL<<SYS_BODCTL_BODVL_Pos)       /*!< Setting Brown Out Detector Threshold Voltage as 3.7V */
#define SYS_BODCTL_BODVL_2_7V           (1UL<<SYS_BODCTL_BODVL_Pos)       /*!< Setting Brown Out Detector Threshold Voltage as 2.7V */
#define SYS_BODCTL_BODVL_2_2V           (0UL<<SYS_BODCTL_BODVL_Pos)       /*!< Setting Brown Out Detector Threshold Voltage as 2.2V */


/*---------------------------------------------------------------------------------------------------------*/
/*  VREFCTL constant definitions. (Write-Protection Register)                                              */
/*---------------------------------------------------------------------------------------------------------*/
#define SYS_VREFCTL_VREF_2_56V      (0x3UL<<SYS_VREFCTL_VREFCTL_Pos)    /*!< VOUT = 2.56V */
#define SYS_VREFCTL_VREF_2_048V     (0x7UL<<SYS_VREFCTL_VREFCTL_Pos)    /*!< VOUT = 2.048V */
#define SYS_VREFCTL_VREF_3_072V     (0xBUL<<SYS_VREFCTL_VREFCTL_Pos)    /*!< VOUT = 3.072V */
#define SYS_VREFCTL_VREF_4_096V     (0xFUL<<SYS_VREFCTL_VREFCTL_Pos)    /*!< VOUT = 4.096V */


/*---------------------------------------------------------------------------------------------------------*/
/*  Multi-Function constant definitions.                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
/* How to use below #define?
Example 1: If user want to set PA.0 as SC0_CLK in initial function,
           user can issue following command to achieve it.

           SYS->GPA_MFPL  = (SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA0MFP_Msk) ) | SYS_GPA_MFPL_PA0_MFP_SC0_CLK  ;

*/
//PA0 MFP
#define SYS_GPA_MFPL_PA0MFP_GPIO               (0ul << SYS_GPA_MFPL_PA0MFP_Pos)        /*!< GPA_MFPL PA0 setting for GPIO*/
#define SYS_GPA_MFPL_PA0MFP_UART1_nCTS         (1ul << SYS_GPA_MFPL_PA0MFP_Pos)        /*!< GPA_MFPL PA0 setting for UART1_nCTS*/
#define SYS_GPA_MFPL_PA0MFP_UART1_TXD          (3ul << SYS_GPA_MFPL_PA0MFP_Pos)        /*!< GPA_MFPL PA0 setting for UART1_TXD*/
#define SYS_GPA_MFPL_PA0MFP_SC0_CLK            (5ul << SYS_GPA_MFPL_PA0MFP_Pos)        /*!< GPA_MFPL PA0 setting for SC0_CLK*/
#define SYS_GPA_MFPL_PA0MFP_PWM1_CH5           (6ul << SYS_GPA_MFPL_PA0MFP_Pos)        /*!< GPA_MFPL PA0 setting for PWM1_CH5*/
#define SYS_GPA_MFPL_PA0MFP_EBI_AD0            (7ul << SYS_GPA_MFPL_PA0MFP_Pos)        /*!< GPA_MFPL PA0 setting for EBI_AD0*/
#define SYS_GPA_MFPL_PA0MFP_INT0               (8ul << SYS_GPA_MFPL_PA0MFP_Pos)        /*!< GPA_MFPL PA0 setting for INT0*/
#define SYS_GPA_MFPL_PA0MFP_SPI1_I2SMCLK       (9ul << SYS_GPA_MFPL_PA0MFP_Pos)        /*!< GPA_MFPL PA0 setting for SPI1_I2SMCLK*/

//PA1 MFP
#define SYS_GPA_MFPL_PA1MFP_GPIO               (0ul << SYS_GPA_MFPL_PA1MFP_Pos)        /*!< GPA_MFPL PA1 setting for GPIO*/
#define SYS_GPA_MFPL_PA1MFP_UART1_nRTS         (1ul << SYS_GPA_MFPL_PA1MFP_Pos)        /*!< GPA_MFPL PA1 setting for UART1_nRTS*/
#define SYS_GPA_MFPL_PA1MFP_UART1_RXD          (3ul << SYS_GPA_MFPL_PA1MFP_Pos)        /*!< GPA_MFPL PA1 setting for UART1_RXD*/
#define SYS_GPA_MFPL_PA1MFP_SC0_DAT            (5ul << SYS_GPA_MFPL_PA1MFP_Pos)        /*!< GPA_MFPL PA1 setting for SC0_DAT*/
#define SYS_GPA_MFPL_PA1MFP_PWM1_CH4           (6ul << SYS_GPA_MFPL_PA1MFP_Pos)        /*!< GPA_MFPL PA1 setting for PWM1_CH4*/
#define SYS_GPA_MFPL_PA1MFP_EBI_AD1            (7ul << SYS_GPA_MFPL_PA1MFP_Pos)        /*!< GPA_MFPL PA1 setting for EBI_AD1*/
#define SYS_GPA_MFPL_PA1MFP_STADC              (10ul << SYS_GPA_MFPL_PA1MFP_Pos)       /*!< GPA_MFPL PA1 setting for STADC*/

//PA2 MFP
#define SYS_GPA_MFPL_PA2MFP_GPIO               (0ul << SYS_GPA_MFPL_PA2MFP_Pos)        /*!< GPA_MFPL PA2 setting for GPIO*/
#define SYS_GPA_MFPL_PA2MFP_USB_VBUS_EN        (1ul << SYS_GPA_MFPL_PA2MFP_Pos)        /*!< GPA_MFPL PA2 setting for USB_VBUS_EN*/
#define SYS_GPA_MFPL_PA2MFP_UART0_TXD          (2ul << SYS_GPA_MFPL_PA2MFP_Pos)        /*!< GPA_MFPL PA2 setting for UART0_TXD*/
#define SYS_GPA_MFPL_PA2MFP_UART0_nCTS         (3ul << SYS_GPA_MFPL_PA2MFP_Pos)        /*!< GPA_MFPL PA2 setting for UART0_nCTS*/
#define SYS_GPA_MFPL_PA2MFP_I2C0_SDA           (4ul << SYS_GPA_MFPL_PA2MFP_Pos)        /*!< GPA_MFPL PA2 setting for I2C0_SDA*/
#define SYS_GPA_MFPL_PA2MFP_SC0_RST            (5ul << SYS_GPA_MFPL_PA2MFP_Pos)        /*!< GPA_MFPL PA2 setting for SC0_RST*/
#define SYS_GPA_MFPL_PA2MFP_PWM1_CH3           (6ul << SYS_GPA_MFPL_PA2MFP_Pos)        /*!< GPA_MFPL PA2 setting for PWM1_CH3*/
#define SYS_GPA_MFPL_PA2MFP_EBI_AD2            (7ul << SYS_GPA_MFPL_PA2MFP_Pos)        /*!< GPA_MFPL PA2 setting for EBI_AD2*/

//PA3 MFP
#define SYS_GPA_MFPL_PA3MFP_GPIO               (0ul << SYS_GPA_MFPL_PA3MFP_Pos)        /*!< GPA_MFPL PA3 setting for GPIO*/
#define SYS_GPA_MFPL_PA3MFP_USB_VBUS_ST       (1ul << SYS_GPA_MFPL_PA3MFP_Pos)        /*!< GPA_MFPL PA3 setting for USB_VBUS_ST*/
#define SYS_GPA_MFPL_PA3MFP_UART0_RXD          (2ul << SYS_GPA_MFPL_PA3MFP_Pos)        /*!< GPA_MFPL PA3 setting for UART0_RXD*/
#define SYS_GPA_MFPL_PA3MFP_UART0_nRTS         (3ul << SYS_GPA_MFPL_PA3MFP_Pos)        /*!< GPA_MFPL PA3 setting for UART0_nRTS*/
#define SYS_GPA_MFPL_PA3MFP_I2C0_SCL           (4ul << SYS_GPA_MFPL_PA3MFP_Pos)        /*!< GPA_MFPL PA3 setting for I2C0_SCL*/
#define SYS_GPA_MFPL_PA3MFP_SC0_PWR            (5ul << SYS_GPA_MFPL_PA3MFP_Pos)        /*!< GPA_MFPL PA3 setting for SC0_PWR*/
#define SYS_GPA_MFPL_PA3MFP_PWM1_CH2           (6ul << SYS_GPA_MFPL_PA3MFP_Pos)        /*!< GPA_MFPL PA3 setting for PWM1_CH2*/
#define SYS_GPA_MFPL_PA3MFP_EBI_AD3            (7ul << SYS_GPA_MFPL_PA3MFP_Pos)        /*!< GPA_MFPL PA3 setting for EBI_AD3*/

//PB0 MFP
#define SYS_GPB_MFPL_PB0MFP_GPIO               (0ul << SYS_GPB_MFPL_PB0MFP_Pos)        /*!< GPB_MFPL PB0 setting for GPIO*/
#define SYS_GPB_MFPL_PB0MFP_EADC_CH0           (1ul << SYS_GPB_MFPL_PB0MFP_Pos)        /*!< GPB_MFPL PB0 setting for EADC_CH0*/
#define SYS_GPB_MFPL_PB0MFP_SPI0_MOSI1         (2ul << SYS_GPB_MFPL_PB0MFP_Pos)        /*!< GPB_MFPL PB0 setting for SPI0_MOSI1*/
#define SYS_GPB_MFPL_PB0MFP_UART2_RXD          (3ul << SYS_GPB_MFPL_PB0MFP_Pos)        /*!< GPB_MFPL PB0 setting for UART2_RXD*/
#define SYS_GPB_MFPL_PB0MFP_T2                 (4ul << SYS_GPB_MFPL_PB0MFP_Pos)        /*!< GPB_MFPL PB0 setting for T2*/
#define SYS_GPB_MFPL_PB0MFP_EBI_nWRL           (7ul << SYS_GPB_MFPL_PB0MFP_Pos)        /*!< GPB_MFPL PB0 setting for EBI_nWRL*/
#define SYS_GPB_MFPL_PB0MFP_INT1               (8ul << SYS_GPB_MFPL_PB0MFP_Pos)        /*!< GPB_MFPL PB0 setting for INT1*/

//PB1 MFP
#define SYS_GPB_MFPL_PB1MFP_GPIO               (0ul << SYS_GPB_MFPL_PB1MFP_Pos)        /*!< GPB_MFPL PB1 setting for GPIO*/
#define SYS_GPB_MFPL_PB1MFP_EADC_CH1           (1ul << SYS_GPB_MFPL_PB1MFP_Pos)        /*!< GPB_MFPL PB1 setting for EADC_CH1*/
#define SYS_GPB_MFPL_PB1MFP_SPI0_MISO1         (2ul << SYS_GPB_MFPL_PB1MFP_Pos)        /*!< GPB_MFPL PB1 setting for SPI0_MISO1*/
#define SYS_GPB_MFPL_PB1MFP_UART2_TXD          (3ul << SYS_GPB_MFPL_PB1MFP_Pos)        /*!< GPB_MFPL PB1 setting for UART2_TXD*/
#define SYS_GPB_MFPL_PB1MFP_T3                 (4ul << SYS_GPB_MFPL_PB1MFP_Pos)        /*!< GPB_MFPL PB1 setting for T3*/
#define SYS_GPB_MFPL_PB1MFP_SC0_RST            (5ul << SYS_GPB_MFPL_PB1MFP_Pos)        /*!< GPB_MFPL PB1 setting for SC0_RST*/
#define SYS_GPB_MFPL_PB1MFP_PWM0_SYNC_OUT      (6ul << SYS_GPB_MFPL_PB1MFP_Pos)        /*!< GPB_MFPL PB1 setting for PWM0_SYNC_OUT*/
#define SYS_GPB_MFPL_PB1MFP_EBI_nWRH           (7ul << SYS_GPB_MFPL_PB1MFP_Pos)        /*!< GPB_MFPL PB1 setting for EBI_nWRH*/

//PB2 MFP
#define SYS_GPB_MFPL_PB2MFP_GPIO               (0ul << SYS_GPB_MFPL_PB2MFP_Pos)        /*!< GPB_MFPL PB2 setting for GPIO*/
#define SYS_GPB_MFPL_PB2MFP_EADC_CH2           (1ul << SYS_GPB_MFPL_PB2MFP_Pos)        /*!< GPB_MFPL PB2 setting for EADC_CH2*/
#define SYS_GPB_MFPL_PB2MFP_SPI0_CLK           (2ul << SYS_GPB_MFPL_PB2MFP_Pos)        /*!< GPB_MFPL PB2 setting for SPI0_CLK*/
#define SYS_GPB_MFPL_PB2MFP_SPI1_CLK           (3ul << SYS_GPB_MFPL_PB2MFP_Pos)        /*!< GPB_MFPL PB2 setting for SPI1_CLK*/
#define SYS_GPB_MFPL_PB2MFP_UART1_RXD          (4ul << SYS_GPB_MFPL_PB2MFP_Pos)        /*!< GPB_MFPL PB2 setting for UART1_RXD*/
#define SYS_GPB_MFPL_PB2MFP_SC0_CD             (5ul << SYS_GPB_MFPL_PB2MFP_Pos)        /*!< GPB_MFPL PB2 setting for SC0_CD*/
#define SYS_GPB_MFPL_PB2MFP_UART3_RXD          (9ul << SYS_GPB_MFPL_PB2MFP_Pos)        /*!< GPB_MFPL PB2 setting for UART3_RXD*/
#define SYS_GPB_MFPL_PB2MFP_T2_EXT             (11ul << SYS_GPB_MFPL_PB2MFP_Pos)       /*!< GPB_MFPL PB2 setting for T2_EXT*/

//PB3
#define SYS_GPB_MFPL_PB3MFP_GPIO               (0ul << SYS_GPB_MFPL_PB3MFP_Pos)        /*!< GPB_MFPL PB3 setting for GPIO*/
#define SYS_GPB_MFPL_PB3MFP_EADC_CH3           (1ul << SYS_GPB_MFPL_PB3MFP_Pos)        /*!< GPB_MFPL PB3 setting for EADC_CH3*/
#define SYS_GPB_MFPL_PB3MFP_SPI0_MISO0         (2ul << SYS_GPB_MFPL_PB3MFP_Pos)        /*!< GPB_MFPL PB3 setting for SPI0_MISO0*/
#define SYS_GPB_MFPL_PB3MFP_SPI1_MISO          (3ul << SYS_GPB_MFPL_PB3MFP_Pos)        /*!< GPB_MFPL PB3 setting for SPI1_MISO*/
#define SYS_GPB_MFPL_PB3MFP_UART1_TXD          (4ul << SYS_GPB_MFPL_PB3MFP_Pos)        /*!< GPB_MFPL PB3 setting for UART1_TXD*/
#define SYS_GPB_MFPL_PB3MFP_EBI_ALE            (7ul << SYS_GPB_MFPL_PB3MFP_Pos)        /*!< GPB_MFPL PB3 setting for EBI_ALE*/
#define SYS_GPB_MFPL_PB3MFP_UART3_TXD          (9ul << SYS_GPB_MFPL_PB3MFP_Pos)        /*!< GPB_MFPL PB3 setting for UART3_TXD*/
#define SYS_GPB_MFPL_PB3MFP_T0_EXT             (11ul << SYS_GPB_MFPL_PB3MFP_Pos)       /*!< GPB_MFPL PB3 setting for T0_EXT*/

//PB4
#define SYS_GPB_MFPL_PB4MFP_GPIO               (0ul << SYS_GPB_MFPL_PB4MFP_Pos)        /*!< GPB_MFPL PB4 setting for GPIO*/
#define SYS_GPB_MFPL_PB4MFP_EADC_CH4           (1ul << SYS_GPB_MFPL_PB4MFP_Pos)        /*!< GPB_MFPL PB4 setting for EADC_CH4*/
#define SYS_GPB_MFPL_PB4MFP_SPI0_SS            (2ul << SYS_GPB_MFPL_PB4MFP_Pos)        /*!< GPB_MFPL PB4 setting for SPI0_SS*/
#define SYS_GPB_MFPL_PB4MFP_SPI1_SS            (3ul << SYS_GPB_MFPL_PB4MFP_Pos)        /*!< GPB_MFPL PB4 setting for SPI1_SS*/
#define SYS_GPB_MFPL_PB4MFP_UART1_nCTS         (4ul << SYS_GPB_MFPL_PB4MFP_Pos)        /*!< GPB_MFPL PB4 setting for UART1_nCTS*/
#define SYS_GPB_MFPL_PB4MFP_EBI_AD7            (7ul << SYS_GPB_MFPL_PB4MFP_Pos)        /*!< GPB_MFPL PB4 setting for EBI_AD7*/
#define SYS_GPB_MFPL_PB4MFP_UART2_TXD          (9ul << SYS_GPB_MFPL_PB4MFP_Pos)        /*!< GPB_MFPL PB4 setting for UART2_TXD*/
#define SYS_GPB_MFPL_PB4MFP_T1_EXT             (11ul << SYS_GPB_MFPL_PB4MFP_Pos)       /*!< GPB_MFPL PB4 setting for T1_EXT*/

//PB5
#define SYS_GPB_MFPL_PB5MFP_GPIO               (0ul << SYS_GPB_MFPL_PB5MFP_Pos)        /*!< GPB_MFPL PB5 setting for GPIO*/
#define SYS_GPB_MFPL_PB5MFP_EADC_CH13          (1ul << SYS_GPB_MFPL_PB5MFP_Pos)        /*!< GPB_MFPL PB5 setting for EADC_CH13*/
#define SYS_GPB_MFPL_PB5MFP_SPI0_MOSI0         (2ul << SYS_GPB_MFPL_PB5MFP_Pos)        /*!< GPB_MFPL PB5 setting for SPI0_MOSI0*/
#define SYS_GPB_MFPL_PB5MFP_SPI1_MOSI          (3ul << SYS_GPB_MFPL_PB5MFP_Pos)        /*!< GPB_MFPL PB5 setting for SPI1_MOSI*/
#define SYS_GPB_MFPL_PB5MFP_EBI_AD6            (7ul << SYS_GPB_MFPL_PB5MFP_Pos)        /*!< GPB_MFPL PB5 setting for EBI_AD6*/
#define SYS_GPB_MFPL_PB5MFP_UART2_RXD          (9ul << SYS_GPB_MFPL_PB5MFP_Pos)        /*!< GPB_MFPL PB5 setting for UART2_RXD*/

//PB6
#define SYS_GPB_MFPL_PB6MFP_GPIO               (0ul << SYS_GPB_MFPL_PB6MFP_Pos)        /*!< GPB_MFPL PB6 setting for GPIO*/
#define SYS_GPB_MFPL_PB6MFP_EADC_CH14          (1ul << SYS_GPB_MFPL_PB6MFP_Pos)        /*!< GPB_MFPL PB6 setting for EADC_CH14*/
#define SYS_GPB_MFPL_PB6MFP_SPI0_MISO0         (2ul << SYS_GPB_MFPL_PB6MFP_Pos)        /*!< GPB_MFPL PB6 setting for SPI0_MISO0*/
#define SYS_GPB_MFPL_PB6MFP_SPI1_MISO          (3ul << SYS_GPB_MFPL_PB6MFP_Pos)        /*!< GPB_MFPL PB6 setting for SPI1_MISO*/
#define SYS_GPB_MFPL_PB6MFP_EBI_AD5            (7ul << SYS_GPB_MFPL_PB6MFP_Pos)        /*!< GPB_MFPL PB6 setting for EBI_AD5*/

//PB7
#define SYS_GPB_MFPL_PB7MFP_GPIO               (0ul << SYS_GPB_MFPL_PB7MFP_Pos)        /*!< GPB_MFPL PB7 setting for GPIO*/
#define SYS_GPB_MFPL_PB7MFP_EADC_CH15          (1ul << SYS_GPB_MFPL_PB7MFP_Pos)        /*!< GPB_MFPL PB7 setting for EADC_CH15*/
#define SYS_GPB_MFPL_PB7MFP_SPI0_CLK           (2ul << SYS_GPB_MFPL_PB7MFP_Pos)        /*!< GPB_MFPL PB7 setting for SPI0_CLK*/
#define SYS_GPB_MFPL_PB7MFP_SPI1_CLK           (3ul << SYS_GPB_MFPL_PB7MFP_Pos)        /*!< GPB_MFPL PB7 setting for SPI1_CLK*/
#define SYS_GPB_MFPL_PB7MFP_EBI_AD4            (7ul << SYS_GPB_MFPL_PB7MFP_Pos)        /*!< GPB_MFPL PB7 setting for EBI_AD4*/
#define SYS_GPB_MFPL_PB7MFP_STADC              (10ul << SYS_GPB_MFPL_PB7MFP_Pos)       /*!< GPB_MFPL PB7 setting for STADC*/

//PB8
#define SYS_GPB_MFPH_PB8MFP_GPIO               (0ul << SYS_GPB_MFPH_PB8MFP_Pos)        /*!< GPB_MFPH PB8 setting for GPIO*/
#define SYS_GPB_MFPH_PB8MFP_EADC_CH5           (1ul << SYS_GPB_MFPH_PB8MFP_Pos)        /*!< GPB_MFPH PB8 setting for EADC_CH5*/
#define SYS_GPB_MFPH_PB8MFP_UART1_nRTS         (4ul << SYS_GPB_MFPH_PB8MFP_Pos)        /*!< GPB_MFPH PB8 setting for UART1_nRTS*/
#define SYS_GPB_MFPH_PB8MFP_PWM0_CH2           (6ul << SYS_GPB_MFPH_PB8MFP_Pos)        /*!< GPB_MFPH PB8 setting for PWM0_CH2*/

//PB11
#define SYS_GPB_MFPH_PB11MFP_GPIO              (0ul << SYS_GPB_MFPH_PB11MFP_Pos)        /*!< GPB_MFPH_ PB11 setting for GPIO*/
#define SYS_GPB_MFPH_PB11MFP_EADC_CH8          (1ul << SYS_GPB_MFPH_PB11MFP_Pos)        /*!< GPB_MFPH_ PB11 setting for EADC_CH8*/

//PB12
#define SYS_GPB_MFPH_PB12MFP_GPIO              (0ul << SYS_GPB_MFPH_PB12MFP_Pos)        /*!< GPB_MFPH_ PB12 setting for GPIO*/
#define SYS_GPB_MFPH_PB12MFP_EADC_CH9          (1ul << SYS_GPB_MFPH_PB12MFP_Pos)        /*!< GPB_MFPH_ PB12 setting for EADC_CH9*/

//PB15
#define SYS_GPB_MFPH_PB15MFP_GPIO              (0ul << SYS_GPB_MFPH_PB15MFP_Pos)        /*!< GPB_MFPH PB15 setting for GPIO*/
#define SYS_GPB_MFPH_PB15MFP_EADC_CH12         (1ul << SYS_GPB_MFPH_PB15MFP_Pos)        /*!< GPB_MFPH PB15 setting for EADC_CH12*/
#define SYS_GPB_MFPH_PB15MFP_EBI_nCS1          (7ul << SYS_GPB_MFPH_PB15MFP_Pos)        /*!< GPB_MFPH PB15 setting for EBI_nCS1*/

//PC0
#define SYS_GPC_MFPL_PC0MFP_GPIO               (0ul << SYS_GPC_MFPL_PC0MFP_Pos)        /*!< GPC_MFPL PC0 setting for GPIO*/
#define SYS_GPC_MFPL_PC0MFP_SPI1_CLK           (2ul << SYS_GPC_MFPL_PC0MFP_Pos)        /*!< GPC_MFPL PC0 setting for SPI1_CLK*/
#define SYS_GPC_MFPL_PC0MFP_UART2_nCTS         (3ul << SYS_GPC_MFPL_PC0MFP_Pos)        /*!< GPC_MFPL PC0 setting for UART2_nCTS*/
#define SYS_GPC_MFPL_PC0MFP_PWM0_CH0           (6ul << SYS_GPC_MFPL_PC0MFP_Pos)        /*!< GPC_MFPL PC0 setting for PWM0_CH0*/
#define SYS_GPC_MFPL_PC0MFP_EBI_AD8            (7ul << SYS_GPC_MFPL_PC0MFP_Pos)        /*!< GPC_MFPL PC0 setting for EBI_AD8*/
#define SYS_GPC_MFPL_PC0MFP_INT2               (8ul << SYS_GPC_MFPL_PC0MFP_Pos)        /*!< GPC_MFPL PC0 setting for INT2*/
#define SYS_GPC_MFPL_PC0MFP_UART3_TXD          (9ul << SYS_GPC_MFPL_PC0MFP_Pos)        /*!< GPC_MFPL PC0 setting for UART3_TXD*/
#define SYS_GPC_MFPL_PC0MFP_T3_EXT             (11ul << SYS_GPC_MFPL_PC0MFP_Pos)       /*!< GPC_MFPL PC0 setting for T3_EXT*/

//PC1
#define SYS_GPC_MFPL_PC1MFP_GPIO               (0ul << SYS_GPC_MFPL_PC1MFP_Pos)        /*!< GPC_MFPL PC1 setting for GPIO*/
#define SYS_GPC_MFPL_PC1MFP_CLKO               (1ul << SYS_GPC_MFPL_PC1MFP_Pos)        /*!< GPC_MFPL PC1 setting for CLKO*/
#define SYS_GPC_MFPL_PC1MFP_UART2_nRTS         (3ul << SYS_GPC_MFPL_PC1MFP_Pos)        /*!< GPC_MFPL PC1 setting for UART2_nRTS*/
#define SYS_GPC_MFPL_PC1MFP_PWM0_CH1           (6ul << SYS_GPC_MFPL_PC1MFP_Pos)        /*!< GPC_MFPL PC1 setting for PWM0_CH1*/
#define SYS_GPC_MFPL_PC1MFP_EBI_AD9            (7ul << SYS_GPC_MFPL_PC1MFP_Pos)        /*!< GPC_MFPL PC1 setting for EBI_AD9*/
#define SYS_GPC_MFPL_PC1MFP_UART3_RXD          (9ul << SYS_GPC_MFPL_PC1MFP_Pos)        /*!< GPC_MFPL PC1 setting for UART3_RXD*/

//PC2
#define SYS_GPC_MFPL_PC2MFP_GPIO               (0ul << SYS_GPC_MFPL_PC2MFP_Pos)        /*!< GPC_MFPL PC2 setting for GPIO*/
#define SYS_GPC_MFPL_PC2MFP_SPI1_SS            (2ul << SYS_GPC_MFPL_PC2MFP_Pos)        /*!< GPC_MFPL PC2 setting for SPI1_SS*/
#define SYS_GPC_MFPL_PC2MFP_UART2_TXD          (3ul << SYS_GPC_MFPL_PC2MFP_Pos)        /*!< GPC_MFPL PC2 setting for UART2_TXD*/
#define SYS_GPC_MFPL_PC2MFP_PWM0_CH2           (6ul << SYS_GPC_MFPL_PC2MFP_Pos)        /*!< GPC_MFPL PC2 setting for PWM0_CH2*/
#define SYS_GPC_MFPL_PC2MFP_EBI_AD10           (7ul << SYS_GPC_MFPL_PC2MFP_Pos)        /*!< GPC_MFPL PC2 setting for EBI_AD10*/

//PC3
#define SYS_GPC_MFPL_PC3MFP_GPIO               (0ul << SYS_GPC_MFPL_PC3MFP_Pos)        /*!< GPC_MFPL PC3 setting for GPIO*/
#define SYS_GPC_MFPL_PC3MFP_SPI1_MOSI          (2ul << SYS_GPC_MFPL_PC3MFP_Pos)        /*!< GPC_MFPL PC3 setting for SPI1_MOSI*/
#define SYS_GPC_MFPL_PC3MFP_UART2_RXD          (3ul << SYS_GPC_MFPL_PC3MFP_Pos)        /*!< GPC_MFPL PC3 setting for UART2_RXD*/
#define SYS_GPC_MFPL_PC3MFP_USB_VBUS_ST        (4ul << SYS_GPC_MFPL_PC3MFP_Pos)        /*!< GPC_MFPL PC3 setting for USB_VBUS_ST*/
#define SYS_GPC_MFPL_PC3MFP_PWM0_CH3           (6ul << SYS_GPC_MFPL_PC3MFP_Pos)        /*!< GPC_MFPL PC3 setting for PWM0_CH3*/
#define SYS_GPC_MFPL_PC3MFP_EBI_AD11           (7ul << SYS_GPC_MFPL_PC3MFP_Pos)        /*!< GPC_MFPL PC3 setting for EBI_AD11*/

//PC4
#define SYS_GPC_MFPL_PC4MFP_GPIO               (0ul << SYS_GPC_MFPL_PC4MFP_Pos)        /*!< GPC_MFPL PC4 setting for GPIO*/
#define SYS_GPC_MFPL_PC4MFP_SPI1_MISO          (2ul << SYS_GPC_MFPL_PC4MFP_Pos)        /*!< GPC_MFPL PC4 setting for SPI1_MISO*/
#define SYS_GPC_MFPL_PC4MFP_I2C1_SCL           (3ul << SYS_GPC_MFPL_PC4MFP_Pos)        /*!< GPC_MFPL PC4 setting for I2C1_SCL*/
#define SYS_GPC_MFPL_PC4MFP_USB_VBUS_EN        (4ul << SYS_GPC_MFPL_PC4MFP_Pos)        /*!< GPC_MFPL PC4 setting for USB_VBUS_EN*/
#define SYS_GPC_MFPL_PC4MFP_PWM0_CH4           (6ul << SYS_GPC_MFPL_PC4MFP_Pos)        /*!< GPC_MFPL PC4 setting for PWM0_CH4*/
#define SYS_GPC_MFPL_PC4MFP_EBI_AD12           (7ul << SYS_GPC_MFPL_PC4MFP_Pos)        /*!< GPC_MFPL PC4 setting for EBI_AD12*/

//PC5
#define SYS_GPC_MFPL_PC5MFP_GPIO               (0ul << SYS_GPC_MFPL_PC5MFP_Pos)        /*!< GPC_MFPL PC5 setting for GPIO*/
#define SYS_GPC_MFPL_PC5MFP_PWM0_CH5           (6ul << SYS_GPC_MFPL_PC5MFP_Pos)        /*!< GPC_MFPL PC5 setting for PWM0_CH5*/
#define SYS_GPC_MFPL_PC5MFP_EBI_AD13           (7ul << SYS_GPC_MFPL_PC5MFP_Pos)        /*!< GPC_MFPL PC5 setting for EBI_AD13*/

//PC6
#define SYS_GPC_MFPL_PC6MFP_GPIO               (0ul << SYS_GPC_MFPL_PC6MFP_Pos)        /*!< GPC_MFPL PC6 setting for GPIO*/
#define SYS_GPC_MFPL_PC6MFP_I2C1_SMBAL         (3ul << SYS_GPC_MFPL_PC6MFP_Pos)        /*!< GPC_MFPL PC6 setting for I2C1_SMBAL*/
#define SYS_GPC_MFPL_PC6MFP_PWM1_CH0           (6ul << SYS_GPC_MFPL_PC6MFP_Pos)        /*!< GPC_MFPL PC6 setting for PWM1_CH0*/
#define SYS_GPC_MFPL_PC6MFP_EBI_AD14           (7ul << SYS_GPC_MFPL_PC6MFP_Pos)        /*!< GPC_MFPL PC6 setting for EBI_AD14*/
#define SYS_GPC_MFPL_PC6MFP_UART0_TXD          (9ul << SYS_GPC_MFPL_PC6MFP_Pos)        /*!< GPC_MFPL PC6 setting for UART0_TXD*/

//PC7
#define SYS_GPC_MFPL_PC7MFP_GPIO               (0ul << SYS_GPC_MFPL_PC7MFP_Pos)        /*!< GPC_MFPL PC7 setting for GPIO*/
#define SYS_GPC_MFPL_PC7MFP_I2C1_SMBSUS        (3ul << SYS_GPC_MFPL_PC7MFP_Pos)        /*!< GPC_MFPL PC7 setting for I2C1_SMBSUS*/
#define SYS_GPC_MFPL_PC7MFP_PWM1_CH1           (6ul << SYS_GPC_MFPL_PC7MFP_Pos)        /*!< GPC_MFPL PC7 setting for PWM1_CH1*/
#define SYS_GPC_MFPL_PC7MFP_EBI_AD15           (7ul << SYS_GPC_MFPL_PC7MFP_Pos)        /*!< GPC_MFPL PC7 setting for EBI_AD15*/
#define SYS_GPC_MFPL_PC7MFP_UART0_RXD          (9ul << SYS_GPC_MFPL_PC7MFP_Pos)        /*!< GPC_MFPL PC7 setting for UART0_RXD*/

//PD0
#define SYS_GPD_MFPL_PD0MFP_GPIO            (0ul << SYS_GPD_MFPL_PD0MFP_Pos)        /*!< GPD_MFPL PD0 setting for GPIO*/
#define SYS_GPD_MFPL_PD0MFP_EADC_CH6        (1ul << SYS_GPD_MFPL_PD0MFP_Pos)        /*!< GPD_MFPL PD0 setting for EADC_CH6*/
#define SYS_GPD_MFPL_PD0MFP_SPI1_I2SMCLK    (2ul << SYS_GPD_MFPL_PD0MFP_Pos)        /*!< GPD_MFPL PD0 setting for SPI1_I2SMCLK*/
#define SYS_GPD_MFPL_PD0MFP_UART0_RXD       (3ul << SYS_GPD_MFPL_PD0MFP_Pos)        /*!< GPD_MFPL PD0 setting for UART0_RXD*/
#define SYS_GPD_MFPL_PD0MFP_INT3            (8ul << SYS_GPD_MFPL_PD0MFP_Pos)        /*!< GPD_MFPL PD0 setting for INT3*/
#define SYS_GPD_MFPL_PD0MFP_T3              (11ul << SYS_GPD_MFPL_PD0MFP_Pos)       /*!< GPD_MFPL PD0 setting for T3*/

//PD1
#define SYS_GPD_MFPL_PD1MFP_GPIO            (0ul << SYS_GPD_MFPL_PD1MFP_Pos)        /*!< GPD_MFPL PD1 setting for GPIO*/
#define SYS_GPD_MFPL_PD1MFP_EADC_CH11       (1ul << SYS_GPD_MFPL_PD1MFP_Pos)        /*!< GPD_MFPL PD1 setting for EADC_CH11*/
#define SYS_GPD_MFPL_PD1MFP_PWM0_SYNC_IN    (2ul << SYS_GPD_MFPL_PD1MFP_Pos)        /*!< GPD_MFPL PD1 setting for PWM0_SYNC_IN*/
#define SYS_GPD_MFPL_PD1MFP_UART0_TXD       (3ul << SYS_GPD_MFPL_PD1MFP_Pos)        /*!< GPD_MFPL PD1 setting for UART0_TXD*/
#define SYS_GPD_MFPL_PD1MFP_T0              (6ul << SYS_GPD_MFPL_PD1MFP_Pos)        /*!< GPD_MFPL PD1 setting for T0*/
#define SYS_GPD_MFPL_PD1MFP_EBI_nRD         (7ul << SYS_GPD_MFPL_PD1MFP_Pos)        /*!< GPD_MFPL PD1 setting for EBI_nRD*/

//PD2
#define SYS_GPD_MFPL_PD2MFP_GPIO            (0ul << SYS_GPD_MFPL_PD2MFP_Pos)        /*!< GPD_MFPL PD2 setting for GPIO*/
#define SYS_GPD_MFPL_PD2MFP_STADC           (1ul << SYS_GPD_MFPL_PD2MFP_Pos)        /*!< GPD_MFPL PD2 setting for STADC*/
#define SYS_GPD_MFPL_PD2MFP_T0_EXT          (3ul << SYS_GPD_MFPL_PD2MFP_Pos)        /*!< GPD_MFPL PD2 setting for T0_EXT*/
#define SYS_GPD_MFPL_PD2MFP_PWM0_BRAKE0     (6ul << SYS_GPD_MFPL_PD2MFP_Pos)        /*!< GPD_MFPL PD2 setting for PWM0_BRAKE0*/
#define SYS_GPD_MFPL_PD2MFP_EBI_nWR         (7ul << SYS_GPD_MFPL_PD2MFP_Pos)        /*!< GPD_MFPL PD2 setting for EBI_nWR*/
#define SYS_GPD_MFPL_PD2MFP_INT0            (8ul << SYS_GPD_MFPL_PD2MFP_Pos)        /*!< GPD_MFPL PD2 setting for INT0*/

//PD3
#define SYS_GPD_MFPL_PD3MFP_GPIO            (0ul << SYS_GPD_MFPL_PD3MFP_Pos)        /*!< GPD_MFPL PD3 setting for GPIO*/
#define SYS_GPD_MFPL_PD3MFP_T2              (1ul << SYS_GPD_MFPL_PD3MFP_Pos)        /*!< GPD_MFPL PD3 setting for T2*/
#define SYS_GPD_MFPL_PD3MFP_T1_EXT          (3ul << SYS_GPD_MFPL_PD3MFP_Pos)        /*!< GPD_MFPL PD3 setting for T1_EXT*/
#define SYS_GPD_MFPL_PD3MFP_PWM0_BRAKE1     (6ul << SYS_GPD_MFPL_PD3MFP_Pos)        /*!< GPD_MFPL PD3 setting for PWM0_BRAKE1*/
#define SYS_GPD_MFPL_PD3MFP_EBI_MCLK        (7ul << SYS_GPD_MFPL_PD3MFP_Pos)        /*!< GPD_MFPL PD3 setting for EBI_MCLK*/
#define SYS_GPD_MFPL_PD3MFP_INT1            (8ul << SYS_GPD_MFPL_PD3MFP_Pos)        /*!< GPD_MFPL PD3 setting for INT1*/

//PD7
#define SYS_GPD_MFPL_PD7MFP_GPIO           (0ul << SYS_GPD_MFPL_PD7MFP_Pos)        /*!< GPD_MFPL PD7 setting for GPIO*/
#define SYS_GPD_MFPL_PD7MFP_PWM0_SYNC_IN   (3ul << SYS_GPD_MFPL_PD7MFP_Pos)        /*!< GPD_MFPL PD7 setting for PWM0_SYNC_IN*/
#define SYS_GPD_MFPL_PD7MFP_T1             (4ul << SYS_GPD_MFPL_PD7MFP_Pos)        /*!< GPD_MFPL PD7 setting for T1*/
#define SYS_GPD_MFPL_PD7MFP_PWM0_CH5       (6ul << SYS_GPD_MFPL_PD7MFP_Pos)        /*!< GPD_MFPL PD7 setting for PWM0_CH5*/
#define SYS_GPD_MFPL_PD7MFP_EBI_nRD        (7ul << SYS_GPD_MFPL_PD7MFP_Pos)        /*!< GPD_MFPL PD7 setting for EBI_nRD*/

//PD8
#define SYS_GPD_MFPH_PD8MFP_GPIO               (0ul << SYS_GPD_MFPH_PD8MFP_Pos)        /*!< GPD_MFPH PD8 setting for GPIO*/
#define SYS_GPD_MFPH_PD8MFP_EADC_CH7           (1ul << SYS_GPD_MFPH_PD8MFP_Pos)        /*!< GPD_MFPH PD8 setting for EADC_CH7*/
#define SYS_GPD_MFPH_PD8MFP_EBI_nCS0           (7ul << SYS_GPD_MFPH_PD8MFP_Pos)        /*!< GPD_MFPH PD8 setting for EBI_nCS0*/

//PD9
#define SYS_GPD_MFPH_PD9MFP_GPIO               (0ul << SYS_GPD_MFPH_PD9MFP_Pos)        /*!< GPD_MFPH PD9 setting for GPIO*/
#define SYS_GPD_MFPH_PD9MFP_EADC_CH10          (1ul << SYS_GPD_MFPH_PD9MFP_Pos)        /*!< GPD_MFPH PD9 setting for EADC_CH10*/
#define SYS_GPD_MFPH_PD9MFP_EBI_ALE            (7ul << SYS_GPD_MFPH_PD9MFP_Pos)        /*!< GPD_MFPH PD9 setting for EBI_ALE*/

//PD12
#define SYS_GPD_MFPH_PD12MFP_GPIO              (0ul << SYS_GPD_MFPH_PD12MFP_Pos)        /*!< GPD_MFPH PD12 setting for GPIO*/
#define SYS_GPD_MFPH_PD12MFP_UART3_TXD         (3ul << SYS_GPD_MFPH_PD12MFP_Pos)        /*!< GPD_MFPH PD12 setting for UART3_TXD*/
#define SYS_GPD_MFPH_PD12MFP_PWM1_CH0          (6ul << SYS_GPD_MFPH_PD12MFP_Pos)        /*!< GPD_MFPH PD12 setting for PWM1_CH0*/
#define SYS_GPD_MFPH_PD12MFP_EBI_ADR16         (7ul << SYS_GPD_MFPH_PD12MFP_Pos)        /*!< GPD_MFPH PD12 setting for EBI_ADR16*/

//PD13
#define SYS_GPD_MFPH_PD13MFP_GPIO              (0ul << SYS_GPD_MFPH_PD13MFP_Pos)        /*!< GPD_MFPH PD13 setting for GPIO*/
#define SYS_GPD_MFPH_PD13MFP_UART3_RXD         (3ul << SYS_GPD_MFPH_PD13MFP_Pos)        /*!< GPD_MFPH PD13 setting for UART3_RXD*/
#define SYS_GPD_MFPH_PD13MFP_PWM1_CH1          (6ul << SYS_GPD_MFPH_PD13MFP_Pos)        /*!< GPD_MFPH PD13 setting for PWM1_CH1*/
#define SYS_GPD_MFPH_PD13MFP_EBI_ADR17         (7ul << SYS_GPD_MFPH_PD13MFP_Pos)        /*!< GPD_MFPH PD13 setting for EBI_ADR17*/

//PD14
#define SYS_GPD_MFPH_PD14MFP_GPIO              (0ul << SYS_GPD_MFPH_PD14MFP_Pos)        /*!< GPD_MFPH_ PD14 setting for GPIO*/
#define SYS_GPD_MFPH_PD14MFP_UART3_nCTS        (3ul << SYS_GPD_MFPH_PD14MFP_Pos)        /*!< GPD_MFPH_ PD14 setting for UART3_nCTS*/
#define SYS_GPD_MFPH_PD14MFP_PWM1_CH2          (6ul << SYS_GPD_MFPH_PD14MFP_Pos)        /*!< GPD_MFPH_ PD14 setting for PWM1_CH2*/
#define SYS_GPD_MFPH_PD14MFP_EBI_ADR18         (7ul << SYS_GPD_MFPH_PD14MFP_Pos)        /*!< GPD_MFPH_ PD14 setting for EBI_ADR18*/

//PD15
#define SYS_GPD_MFPH_PD15MFP_GPIO              (0ul << SYS_GPD_MFPH_PD15MFP_Pos)        /*!< GPD_MFPH_ PD15 setting for GPIO*/
#define SYS_GPD_MFPH_PD15MFP_UART3_nRTS        (3ul << SYS_GPD_MFPH_PD15MFP_Pos)        /*!< GPD_MFPH_ PD15 setting for UART3_nRTS*/
#define SYS_GPD_MFPH_PD15MFP_PWM1_CH3          (6ul << SYS_GPD_MFPH_PD15MFP_Pos)        /*!< GPD_MFPH_ PD15 setting for PWM1_CH3*/
#define SYS_GPD_MFPH_PD15MFP_EBI_ADR19         (7ul << SYS_GPD_MFPH_PD15MFP_Pos)        /*!< GPD_MFPH_ PD15 setting for EBI_ADR19*/

//PE0
#define SYS_GPE_MFPL_PE0MFP_GPIO               (0ul << SYS_GPE_MFPL_PE0MFP_Pos)        /*!< GPE_MFPL PE0 setting for GPIO*/
#define SYS_GPE_MFPL_PE0MFP_I2C1_SDA           (3ul << SYS_GPE_MFPL_PE0MFP_Pos)        /*!< GPE_MFPL PE0 setting for I2C1_SDA*/
#define SYS_GPE_MFPL_PE0MFP_T2_EXT             (4ul << SYS_GPE_MFPL_PE0MFP_Pos)        /*!< GPE_MFPL PE0 setting for T2_EXT*/
#define SYS_GPE_MFPL_PE0MFP_SC0_CD             (5ul << SYS_GPE_MFPL_PE0MFP_Pos)        /*!< GPE_MFPL PE0 setting for SC0_CD*/
#define SYS_GPE_MFPL_PE0MFP_PWM0_CH0           (6ul << SYS_GPE_MFPL_PE0MFP_Pos)        /*!< GPE_MFPL PE0 setting for PWM0_CH0*/
#define SYS_GPE_MFPL_PE0MFP_EBI_nCS1           (7ul << SYS_GPE_MFPL_PE0MFP_Pos)        /*!< GPE_MFPL PE0 setting for EBI_nCS1*/
#define SYS_GPE_MFPL_PE0MFP_INT4               (8ul << SYS_GPE_MFPL_PE0MFP_Pos)        /*!< GPE_MFPL PE0 setting for INT4*/

//PE8
#define SYS_GPE_MFPH_PE8MFP_GPIO               (0ul << SYS_GPE_MFPH_PE8MFP_Pos)        /*!< GPE_MFPH PE8 setting for GPIO*/
#define SYS_GPE_MFPH_PE8MFP_UART1_TXD          (1ul << SYS_GPE_MFPH_PE8MFP_Pos)        /*!< GPE_MFPH PE8 setting for UART1_TXD*/
#define SYS_GPE_MFPH_PE8MFP_SPI0_MISO1         (2ul << SYS_GPE_MFPH_PE8MFP_Pos)        /*!< GPE_MFPH PE8 setting for SPI0_MISO1*/
#define SYS_GPE_MFPH_PE8MFP_I2C1_SCL           (4ul << SYS_GPE_MFPH_PE8MFP_Pos)        /*!< GPE_MFPH PE8 setting for I2C1_SCL*/
#define SYS_GPE_MFPH_PE8MFP_SC0_PWR            (5ul << SYS_GPE_MFPH_PE8MFP_Pos)        /*!< GPE_MFPH PE8 setting for SC0_PWR*/
#define SYS_GPE_MFPH_PE8MFP_CLKO               (9ul << SYS_GPE_MFPH_PE8MFP_Pos)        /*!< GPE_MFPH PE8 setting for CLKO*/
#define SYS_GPE_MFPH_PE8MFP_PWM0_BRAKE0        (10ul << SYS_GPE_MFPH_PE8MFP_Pos)       /*!< GPE_MFPH PE8 setting for PWM0_BRAKE0*/
#define SYS_GPE_MFPH_PE8MFP_T1                 (11ul << SYS_GPE_MFPH_PE8MFP_Pos)       /*!< GPE_MFPH PE8 setting for T1*/

//PE9
#define SYS_GPE_MFPH_PE9MFP_GPIO               (0ul << SYS_GPE_MFPH_PE9MFP_Pos)        /*!< GPE_MFPH PE9 setting for GPIO*/
#define SYS_GPE_MFPH_PE9MFP_UART1_RXD          (1ul << SYS_GPE_MFPH_PE9MFP_Pos)        /*!< GPE_MFPH PE9 setting for UART1_RXD*/
#define SYS_GPE_MFPH_PE9MFP_SPI0_MOSI1         (2ul << SYS_GPE_MFPH_PE9MFP_Pos)        /*!< GPE_MFPH PE9 setting for SPI0_MOSI1*/
#define SYS_GPE_MFPH_PE9MFP_I2C1_SDA           (4ul << SYS_GPE_MFPH_PE9MFP_Pos)        /*!< GPE_MFPH PE9 setting for I2C1_SDA*/
#define SYS_GPE_MFPH_PE9MFP_SC0_RST            (5ul << SYS_GPE_MFPH_PE9MFP_Pos)        /*!< GPE_MFPH PE9 setting for SC0_RST*/
#define SYS_GPE_MFPH_PE9MFP_SPI1_I2SMCLK       (9ul << SYS_GPE_MFPH_PE9MFP_Pos)        /*!< GPE_MFPH PE9 setting for SPI1_I2SMCLK*/
#define SYS_GPE_MFPH_PE9MFP_PWM1_BRAKE1        (10ul << SYS_GPE_MFPH_PE9MFP_Pos)       /*!< GPE_MFPH PE9 setting for PWM1_BRAKE1*/
#define SYS_GPE_MFPH_PE9MFP_T2                 (11ul << SYS_GPE_MFPH_PE9MFP_Pos)       /*!< GPE_MFPH PE9 setting for T2*/

//PE10
#define SYS_GPE_MFPH_PE10MFP_GPIO              (0ul << SYS_GPE_MFPH_PE10MFP_Pos)        /*!< GPE_MFPH PE10 setting for GPIO*/
#define SYS_GPE_MFPH_PE10MFP_SPI1_MISO         (1ul << SYS_GPE_MFPH_PE10MFP_Pos)        /*!< GPE_MFPH PE10 setting for SPI1_MISO*/
#define SYS_GPE_MFPH_PE10MFP_SPI0_MISO0        (2ul << SYS_GPE_MFPH_PE10MFP_Pos)        /*!< GPE_MFPH PE10 setting for SPI0_MISO0*/
#define SYS_GPE_MFPH_PE10MFP_UART1_nCTS        (3ul << SYS_GPE_MFPH_PE10MFP_Pos)        /*!< GPE_MFPH PE10 setting for UART1_nCTS*/
#define SYS_GPE_MFPH_PE10MFP_I2C0_SMBAL        (4ul << SYS_GPE_MFPH_PE10MFP_Pos)        /*!< GPE_MFPH PE10 setting for I2C0_SMBAL*/
#define SYS_GPE_MFPH_PE10MFP_SC0_DAT           (5ul << SYS_GPE_MFPH_PE10MFP_Pos)        /*!< GPE_MFPH PE10 setting for SC0_DAT*/
#define SYS_GPE_MFPH_PE10MFP_UART3_TXD         (9ul << SYS_GPE_MFPH_PE10MFP_Pos)        /*!< GPE_MFPH PE10 setting for UART3_TXD*/
#define SYS_GPE_MFPH_PE10MFP_I2C1_SCL          (11ul << SYS_GPE_MFPH_PE10MFP_Pos)       /*!< GPE_MFPH PE10 setting for I2C1_SCL*/

//PE11
#define SYS_GPE_MFPH_PE11MFP_GPIO              (0ul << SYS_GPE_MFPH_PE11MFP_Pos)        /*!< GPE_MFPH PE11 setting for GPIO*/
#define SYS_GPE_MFPH_PE11MFP_SPI1_MOSI         (1ul << SYS_GPE_MFPH_PE11MFP_Pos)        /*!< GPE_MFPH PE11 setting for SPI1_MOSI*/
#define SYS_GPE_MFPH_PE11MFP_SPI0_MOSI0        (2ul << SYS_GPE_MFPH_PE11MFP_Pos)        /*!< GPE_MFPH PE11 setting for SPI0_MOSI0*/
#define SYS_GPE_MFPH_PE11MFP_UART1_nRTS        (3ul << SYS_GPE_MFPH_PE11MFP_Pos)        /*!< GPE_MFPH PE11 setting for UART1_nRTS*/
#define SYS_GPE_MFPH_PE11MFP_I2C0_SMBSUS       (4ul << SYS_GPE_MFPH_PE11MFP_Pos)        /*!< GPE_MFPH PE11 setting for I2C0_SMBSUS*/
#define SYS_GPE_MFPH_PE11MFP_SC0_CLK           (5ul << SYS_GPE_MFPH_PE11MFP_Pos)        /*!< GPE_MFPH PE11 setting for SC0_CLK*/
#define SYS_GPE_MFPH_PE11MFP_UART3_RXD         (9ul << SYS_GPE_MFPH_PE11MFP_Pos)        /*!< GPE_MFPH PE11 setting for UART3_RXD*/
#define SYS_GPE_MFPH_PE11MFP_I2C1_SDA          (11ul << SYS_GPE_MFPH_PE11MFP_Pos)       /*!< GPE_MFPH PE11 setting for I2C1_SDA*/

//PE12
#define SYS_GPE_MFPH_PE12MFP_GPIO              (0ul << SYS_GPE_MFPH_PE12MFP_Pos)        /*!< GPE_MFPH PE12 setting for GPIO*/
#define SYS_GPE_MFPH_PE12MFP_SPI1_SS           (1ul << SYS_GPE_MFPH_PE12MFP_Pos)        /*!< GPE_MFPH PE12 setting for SPI1_SS*/
#define SYS_GPE_MFPH_PE12MFP_SPI0_SS           (2ul << SYS_GPE_MFPH_PE12MFP_Pos)        /*!< GPE_MFPH PE12 setting for SPI0_SS*/
#define SYS_GPE_MFPH_PE12MFP_UART1_TXD         (3ul << SYS_GPE_MFPH_PE12MFP_Pos)        /*!< GPE_MFPH PE12 setting for UART1_TXD*/
#define SYS_GPE_MFPH_PE12MFP_I2C0_SCL          (4ul << SYS_GPE_MFPH_PE12MFP_Pos)        /*!< GPE_MFPH PE12 setting for I2C0_SCL*/

//PE13
#define SYS_GPE_MFPH_PE13MFP_GPIO              (0ul << SYS_GPE_MFPH_PE13MFP_Pos)        /*!< GPE_MFPH PE13 setting for GPIO*/
#define SYS_GPE_MFPH_PE13MFP_SPI1_CLK          (1ul << SYS_GPE_MFPH_PE13MFP_Pos)        /*!< GPE_MFPH PE13 setting for SPI1_CLK*/
#define SYS_GPE_MFPH_PE13MFP_SPI0_CLK          (2ul << SYS_GPE_MFPH_PE13MFP_Pos)        /*!< GPE_MFPH PE13 setting for SPI0_CLK*/
#define SYS_GPE_MFPH_PE13MFP_UART1_RXD         (3ul << SYS_GPE_MFPH_PE13MFP_Pos)        /*!< GPE_MFPH PE13 setting for UART1_RXD*/
#define SYS_GPE_MFPH_PE13MFP_I2C0_SDA          (4ul << SYS_GPE_MFPH_PE13MFP_Pos)        /*!< GPE_MFPH PE13 setting for I2C0_SDA*/

//PF0
#define SYS_GPF_MFPL_PF0MFP_GPIO               (0ul << SYS_GPF_MFPL_PF0MFP_Pos)        /*!< GPF_MFPL PF0 setting for GPIO*/
#define SYS_GPF_MFPL_PF0MFP_X32_OUT            (1ul << SYS_GPF_MFPL_PF0MFP_Pos)        /*!< GPF_MFPL PF0 setting for X32_OUT*/
#define SYS_GPF_MFPL_PF0MFP_INT5               (8ul << SYS_GPF_MFPL_PF0MFP_Pos)        /*!< GPF_MFPL PF0 setting for INT5*/

//PF1
#define SYS_GPF_MFPL_PF1MFP_GPIO               (0ul << SYS_GPF_MFPL_PF1MFP_Pos)        /*!< GPF_MFPL PF1 setting for GPIO*/
#define SYS_GPF_MFPL_PF1MFP_X32_IN             (1ul << SYS_GPF_MFPL_PF1MFP_Pos)        /*!< GPF_MFPL PF1 setting for X32_IN*/

//PF2
#define SYS_GPF_MFPL_PF2MFP_GPIO               (0ul << SYS_GPF_MFPL_PF2MFP_Pos)        /*!< GPF_MFPL PF2 setting for GPIO*/
#define SYS_GPF_MFPL_PF2MFP_TAMPER             (1ul << SYS_GPF_MFPL_PF2MFP_Pos)        /*!< GPF_MFPL PF2 setting for TAMPER*/

//PF3
#define SYS_GPF_MFPL_PF3MFP_GPIO               (0ul << SYS_GPF_MFPL_PF3MFP_Pos)        /*!< GPF_MFPL PF3 setting for GPIO*/
#define SYS_GPF_MFPL_PF3MFP_XT1_OUT            (1ul << SYS_GPF_MFPL_PF3MFP_Pos)        /*!< GPF_MFPL PF3 setting for XT1_OUT*/
#define SYS_GPF_MFPL_PF3MFP_I2C1_SCL           (3ul << SYS_GPF_MFPL_PF3MFP_Pos)        /*!< GPF_MFPL PF3 setting for I2C1_SCL*/

//PF4
#define SYS_GPF_MFPL_PF4MFP_GPIO               (0ul << SYS_GPF_MFPL_PF4MFP_Pos)        /*!< GPF_MFPL PF4 setting for GPIO*/
#define SYS_GPF_MFPL_PF4MFP_XT1_IN             (1ul << SYS_GPF_MFPL_PF4MFP_Pos)        /*!< GPF_MFPL PF4 setting for XT1_IN*/
#define SYS_GPF_MFPL_PF4MFP_I2C1_SDA           (3ul << SYS_GPF_MFPL_PF4MFP_Pos)        /*!< GPF_MFPL PF4 setting for I2C1_SDA*/

//PF5
#define SYS_GPF_MFPL_PF5MFP_GPIO               (0ul << SYS_GPF_MFPL_PF5MFP_Pos)        /*!< GPF_MFPL PF5 setting for GPIO*/
#define SYS_GPF_MFPL_PF5MFP_ICE_CLK            (1ul << SYS_GPF_MFPL_PF5MFP_Pos)        /*!< GPF_MFPL PF5 setting for ICE_CLK*/

//PF6
#define SYS_GPF_MFPL_PF6MFP_GPIO               (0ul << SYS_GPF_MFPL_PF6MFP_Pos)        /*!< GPF_MFPL PF6 setting for GPIO*/
#define SYS_GPF_MFPL_PF6MFP_ICE_DAT            (1ul << SYS_GPF_MFPL_PF6MFP_Pos)        /*!< GPF_MFPL PF6 setting for ICE_DAT*/

//PF7
#define SYS_GPF_MFPL_PF7MFP_GPIO               (0ul << SYS_GPF_MFPL_PF7MFP_Pos)        /*!< GPF_MFPL PF7 setting for GPIO*/


/*@}*/ /* end of group SYS_EXPORTED_CONSTANTS */


/** @addtogroup SYS_EXPORTED_FUNCTIONS SYS Exported Functions
  @{
*/


/**
  * @brief      Clear Brown-out detector interrupt flag
  * @param      None
  * @return     None
  * @details    This macro clear Brown-out detector interrupt flag.
  */
#define SYS_CLEAR_BOD_INT_FLAG()        (SYS->BODCTL |= SYS_BODCTL_BODIF_Msk)

/**
  * @brief      Set Brown-out detector function to normal mode
  * @param      None
  * @return     None
  * @details    This macro set Brown-out detector to normal mode.
  *             The register write-protection function should be disabled before using this macro.
  */
#define SYS_CLEAR_BOD_LPM()             (SYS->BODCTL &= ~SYS_BODCTL_BODLPM_Msk)

/**
  * @brief      Disable Brown-out detector function
  * @param      None
  * @return     None
  * @details    This macro disable Brown-out detector function.
  *             The register write-protection function should be disabled before using this macro.
  */
#define SYS_DISABLE_BOD()               (SYS->BODCTL &= ~SYS_BODCTL_BODEN_Msk)

/**
  * @brief      Enable Brown-out detector function
  * @param      None
  * @return     None
  * @details    This macro enable Brown-out detector function.
  *             The register write-protection function should be disabled before using this macro.
  */
#define SYS_ENABLE_BOD()                (SYS->BODCTL |= SYS_BODCTL_BODEN_Msk)

/**
  * @brief      Get Brown-out detector interrupt flag
  * @param      None
  * @retval     0   Brown-out detect interrupt flag is not set.
  * @retval     >=1 Brown-out detect interrupt flag is set.
  * @details    This macro get Brown-out detector interrupt flag.
  */
#define SYS_GET_BOD_INT_FLAG()          (SYS->BODCTL & SYS_BODCTL_BODIF_Msk)

/**
  * @brief      Get Brown-out detector status
  * @param      None
  * @retval     0   System voltage is higher than BOD threshold voltage setting or BOD function is disabled.
  * @retval     >=1 System voltage is lower than BOD threshold voltage setting.
  * @details    This macro get Brown-out detector output status.
  *             If the BOD function is disabled, this function always return 0.
  */
#define SYS_GET_BOD_OUTPUT()            (SYS->BODCTL & SYS_BODCTL_BODOUT_Msk)

/**
  * @brief      Enable Brown-out detector interrupt function
  * @param      None
  * @return     None
  * @details    This macro enable Brown-out detector interrupt function.
  *             The register write-protection function should be disabled before using this macro.
  */
#define SYS_DISABLE_BOD_RST()           (SYS->BODCTL &= ~SYS_BODCTL_BODRSTEN_Msk)

/**
  * @brief      Enable Brown-out detector reset function
  * @param      None
  * @return     None
  * @details    This macro enable Brown-out detect reset function.
  *             The register write-protection function should be disabled before using this macro.
  */
#define SYS_ENABLE_BOD_RST()            (SYS->BODCTL |= SYS_BODCTL_BODRSTEN_Msk)

/**
  * @brief      Set Brown-out detector function low power mode
  * @param      None
  * @return     None
  * @details    This macro set Brown-out detector to low power mode.
  *             The register write-protection function should be disabled before using this macro.
  */
#define SYS_SET_BOD_LPM()               (SYS->BODCTL |= SYS_BODCTL_BODLPM_Msk)

/**
  * @brief      Set Brown-out detector voltage level
  * @param[in]  u32Level is Brown-out voltage level. Including :
  *             - \ref SYS_BODCTL_BODVL_4_5V
  *             - \ref SYS_BODCTL_BODVL_3_7V
  *             - \ref SYS_BODCTL_BODVL_2_7V
  *             - \ref SYS_BODCTL_BODVL_2_2V
  * @return     None
  * @details    This macro set Brown-out detector voltage level.
  *             The write-protection function should be disabled before using this macro.
  */
#define SYS_SET_BOD_LEVEL(u32Level)     (SYS->BODCTL = (SYS->BODCTL & ~SYS_BODCTL_BODVL_Msk) | (u32Level))

/**
  * @brief      Get reset source is from Brown-out detector reset
  * @param      None
  * @retval     0   Previous reset source is not from Brown-out detector reset
  * @retval     >=1 Previous reset source is from Brown-out detector reset
  * @details    This macro get previous reset source is from Brown-out detect reset or not.
  */
#define SYS_IS_BOD_RST()                (SYS->RSTSTS & SYS_RSTSTS_BODRF_Msk)

/**
  * @brief      Get reset source is from CPU reset
  * @param      None
  * @retval     0   Previous reset source is not from CPU reset
  * @retval     >=1 Previous reset source is from CPU reset
  * @details    This macro get previous reset source is from CPU reset.
  */
#define SYS_IS_CPU_RST()                (SYS->RSTSTS & SYS_RSTSTS_CPURF_Msk)

/**
  * @brief      Get reset source is from LVR Reset
  * @param      None
  * @retval     0   Previous reset source is not from Low-Voltage-Reset
  * @retval     >=1 Previous reset source is from Low-Voltage-Reset
  * @details    This macro get previous reset source is from Low-Voltage-Reset.
  */
#define SYS_IS_LVR_RST()                (SYS->RSTSTS & SYS_RSTSTS_LVRF_Msk)

/**
  * @brief      Get reset source is from Power-on Reset
  * @param      None
  * @retval     0   Previous reset source is not from Power-on Reset
  * @retval     >=1 Previous reset source is from Power-on Reset
  * @details    This macro get previous reset source is from Power-on Reset.
  */
#define SYS_IS_POR_RST()                (SYS->RSTSTS & SYS_RSTSTS_PORF_Msk)

/**
  * @brief      Get reset source is from reset pin reset
  * @param      None
  * @retval     0   Previous reset source is not from reset pin reset
  * @retval     >=1 Previous reset source is from reset pin reset
  * @details    This macro get previous reset source is from reset pin reset.
  */
#define SYS_IS_RSTPIN_RST()             (SYS->RSTSTS & SYS_RSTSTS_PINRF_Msk)

/**
  * @brief      Get reset source is from system reset
  * @param      None
  * @retval     0   Previous reset source is not from system reset
  * @retval     >=1 Previous reset source is from system reset
  * @details    This macro get previous reset source is from system reset.
  */
#define SYS_IS_SYSTEM_RST()             (SYS->RSTSTS & SYS_RSTSTS_SYSRF_Msk)

/**
  * @brief      Get reset source is from window watch dog reset
  * @param      None
  * @retval     0   Previous reset source is not from window watch dog reset
  * @retval     >=1 Previous reset source is from window watch dog reset
  * @details    This macro get previous reset source is from window watch dog reset.
  */
#define SYS_IS_WDT_RST()                (SYS->RSTSTS & SYS_RSTSTS_WDTRF_Msk)

/**
  * @brief      Disable Low-Voltage-Reset function
  * @param      None
  * @return     None
  * @details    This macro disable Low-Voltage-Reset function.
  *             The register write-protection function should be disabled before using this macro.
  */
#define SYS_DISABLE_LVR()               (SYS->BODCTL &= ~SYS_BODCTL_LVREN_Msk)

/**
  * @brief      Enable Low-Voltage-Reset function
  * @param      None
  * @return     None
  * @details    This macro enable Low-Voltage-Reset function.
  *             The register write-protection function should be disabled before using this macro.
  */
#define SYS_ENABLE_LVR()                (SYS->BODCTL |= SYS_BODCTL_LVREN_Msk)

/**
  * @brief      Disable Power-on Reset function
  * @param      None
  * @return     None
  * @details    This macro disable Power-on Reset function.
  *             The register write-protection function should be disabled before using this macro.
  */
#define SYS_DISABLE_POR()               (SYS->PORCTL = 0x5AA5)

/**
  * @brief      Enable Power-on Reset function
  * @param      None
  * @return     None
  * @details    This macro enable Power-on Reset function.
  *             The register write-protection function should be disabled before using this macro.
  */
#define SYS_ENABLE_POR()                (SYS->PORCTL = 0)

/**
  * @brief      Clear reset source flag
  * @param[in]  u32RstSrc is reset source. Including :
  *             - \ref SYS_RSTSTS_PORF_Msk
  *             - \ref SYS_RSTSTS_PINRF_Msk
  *             - \ref SYS_RSTSTS_WDTRF_Msk
  *             - \ref SYS_RSTSTS_LVRF_Msk
  *             - \ref SYS_RSTSTS_BODRF_Msk
  *             - \ref SYS_RSTSTS_SYSRF_Msk
  *             - \ref SYS_RSTSTS_CPURF_Msk
  *             - \ref SYS_RSTSTS_CPULKRF_Msk
  * @return     None
  * @details    This macro clear reset source flag.
  */
#define SYS_CLEAR_RST_SOURCE(u32RstSrc) ((SYS->RSTSTS) = (u32RstSrc) )


/*---------------------------------------------------------------------------------------------------------*/
/* static inline functions                                                                                 */
/*---------------------------------------------------------------------------------------------------------*/


/**
  * @brief      Disable register write-protection function
  * @param      None
  * @return     None
  * @details    This function disable register write-protection function.
  *             To unlock the protected register to allow write access.
  */
__STATIC_INLINE void SYS_UnlockReg(void)
{
    do
    {
        SYS->REGLCTL = 0x59;
        SYS->REGLCTL = 0x16;
        SYS->REGLCTL = 0x88;
    }
    while(SYS->REGLCTL == 0);
}

/**
  * @brief      Enable register write-protection function
  * @param      None
  * @return     None
  * @details    This function is used to enable register write-protection function.
  *             To lock the protected register to forbid write access.
  */
__STATIC_INLINE void SYS_LockReg(void)
{
    SYS->REGLCTL = 0;
}


void SYS_ClearResetSrc(uint32_t u32Src);
uint32_t SYS_GetBODStatus(void);
uint32_t SYS_GetResetSrc(void);
uint32_t SYS_IsRegLocked(void);
uint32_t SYS_ReadPDID(void);
void SYS_ResetChip(void);
void SYS_ResetCPU(void);
void SYS_ResetModule(uint32_t u32ModuleIndex);
void SYS_EnableBOD(int32_t i32Mode, uint32_t u32BODLevel);
void SYS_DisableBOD(void);


/*@}*/ /* end of group SYS_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group SYS_Driver */

/*@}*/ /* end of group Standard_Driver */


#ifdef __cplusplus
}
#endif

#endif  //__SYS_H__
