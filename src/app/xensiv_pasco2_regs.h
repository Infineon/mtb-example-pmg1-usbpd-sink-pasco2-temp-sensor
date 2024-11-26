/******************************************************************************
* File Name: xensiv_pasco2_regs.h
*
* Description: This file contains the register definitions
*              for interacting with the XENSIV™ PAS CO2 sensor.
*
* Related Document: See README.md
*
*******************************************************************************
* Copyright 2024, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

#ifndef XENSIV_PASCO2_REGS_H_
#define XENSIV_PASCO2_REGS_H_

/**
 * \addtogroup group_board_libs XENSIV™ PAS CO2 sensor
 * \{
 */

#define XENSIV_PASCO2_REG_PROD_ID                       (0x00U)                                                 /*!< REG_PROD: Address */
#define XENSIV_PASCO2_REG_SENS_STS                      (0x01U)                                                 /*!< SENS_STS: Address */
#define XENSIV_PASCO2_REG_MEAS_RATE_H                   (0x02U)                                                 /*!< MEAS_RATE_H: Address */
#define XENSIV_PASCO2_REG_MEAS_RATE_L                   (0x03U)                                                 /*!< MEAS_RATE_L: Address */
#define XENSIV_PASCO2_REG_MEAS_CFG                      (0x04U)                                                 /*!< MEAS_CFG: Address */
#define XENSIV_PASCO2_REG_CO2PPM_H                      (0x05U)                                                 /*!< CO2PPM_H: Address */
#define XENSIV_PASCO2_REG_CO2PPM_L                      (0x06U)                                                 /*!< CO2PPM_L: Address */
#define XENSIV_PASCO2_REG_MEAS_STS                      (0x07U)                                                 /*!< MEAS_STS: Address */
#define XENSIV_PASCO2_REG_INT_CFG                       (0x08U)                                                 /*!< INT_CFG: Address */
#define XENSIV_PASCO2_REG_ALARM_TH_H                    (0x09U)                                                 /*!< ALARM_TH_H: Address */
#define XENSIV_PASCO2_REG_ALARM_TH_L                    (0x0aU)                                                 /*!< ALARM_TH_L: Address */
#define XENSIV_PASCO2_REG_PRESS_REF_H                   (0x0bU)                                                 /*!< PRESS_REF_H: Address */
#define XENSIV_PASCO2_REG_PRESS_REF_L                   (0x0cU)                                                 /*!< PRESS_REF_L: Address */
#define XENSIV_PASCO2_REG_CALIB_REF_H                   (0x0dU)                                                 /*!< CALIB_REF_H: Address */
#define XENSIV_PASCO2_REG_CALIB_REF_L                   (0x0eU)                                                 /*!< CALIB_REF_L: Address */
#define XENSIV_PASCO2_REG_SCRATCH_PAD                   (0x0fU)                                                 /*!< SCRATCH_PAD: Address */
#define XENSIV_PASCO2_REG_SENS_RST                      (0x10U)                                                 /*!< SENS_RST: Address */

#define XENSIV_PASCO2_REG_PROD_ID_REV_POS               (0U)                                                    /*!< REG_PROD: ID_REV position */
#define XENSIV_PASCO2_REG_PROD_ID_REV_MSK               (0x1fU << XENSIV_PASCO2_REG_PROD_ID_REV_POS)            /*!< REG_PROD: ID_REV mask */
#define XENSIV_PASCO2_REG_PROD_ID_PROD_POS              (5U)                                                    /*!< REG_PROD: ID_PROD position */
#define XENSIV_PASCO2_REG_PROD_ID_PROD_MSK              (0x07U << XENSIV_PASCO2_REG_PROD_ID_PROD_POS)           /*!< REG_PROD: ID_PROD mask */

#define XENSIV_PASCO2_REG_SENS_STS_ICCER_CLR_POS        (0U)                                                    /*!< SENS_STS: ICCER_CLR position */
#define XENSIV_PASCO2_REG_SENS_STS_ICCER_CLR_MSK        (0x01U << XENSIV_PASCO2_REG_SENS_STS_ICCER_CLR_POS)     /*!< SENS_STS: ICCER_CLR mask */
#define XENSIV_PASCO2_REG_SENS_STS_ORVS_CLR_POS         (1U)                                                    /*!< SENS_STS: ORVS_CLR position */
#define XENSIV_PASCO2_REG_SENS_STS_ORVS_CLR_MSK         (0x01U << XENSIV_PASCO2_REG_SENS_STS_ORVS_CLR_POS)      /*!< SENS_STS: ORVS_CLR mask */
#define XENSIV_PASCO2_REG_SENS_STS_ORTMP_CLR_POS        (2U)                                                    /*!< SENS_STS: ORTMP_CLR position */
#define XENSIV_PASCO2_REG_SENS_STS_ORTMP_CLR_MSK        (0x01U << XENSIV_PASCO2_REG_SENS_STS_ORTMP_CLR_POS)     /*!< SENS_STS: ORTMP_CLR mask */
#define XENSIV_PASCO2_REG_SENS_STS_ICCER_POS            (3U)                                                    /*!< SENS_STS: ICCER position */
#define XENSIV_PASCO2_REG_SENS_STS_ICCER_MSK            (0x01U << XENSIV_PASCO2_REG_SENS_STS_ICCER_POS)         /*!< SENS_STS: ICCER mask */
#define XENSIV_PASCO2_REG_SENS_STS_ORVS_POS             (4U)                                                    /*!< SENS_STS: ORVS position */
#define XENSIV_PASCO2_REG_SENS_STS_ORVS_MSK             (0x01U << XENSIV_PASCO2_REG_SENS_STS_ORVS_POS)          /*!< SENS_STS: ORVS mask */
#define XENSIV_PASCO2_REG_SENS_STS_ORTMP_POS            (5U)                                                    /*!< SENS_STS: ORTMP position */
#define XENSIV_PASCO2_REG_SENS_STS_ORTMP_MSK            (0x01U << XENSIV_PASCO2_REG_SENS_STS_ORTMP_POS)         /*!< SENS_STS: ORTMP mask */
#define XENSIV_PASCO2_REG_SENS_STS_PWM_DIS_ST_POS       (6U)                                                    /*!< SENS_STS: PWM_DIS_ST position */
#define XENSIV_PASCO2_REG_SENS_STS_PWM_DIS_ST_MSK       (0x01U << XENSIV_PASCO2_REG_SENS_STS_PWM_DIS_ST_POS)    /*!< SENS_STS: PWM_DIS_ST mask */
#define XENSIV_PASCO2_REG_SENS_STS_SEN_RDY_POS          (7U)                                                    /*!< SENS_STS: SEN_RDY position */
#define XENSIV_PASCO2_REG_SENS_STS_SEN_RDY_MSK          (0x01U << XENSIV_PASCO2_REG_SENS_STS_SEN_RDY_POS)       /*!< SENS_STS: SEN_RDY mask */

#define XENSIV_PASCO2_REG_MEAS_RATE_H_VAL_POS           (0U)                                                    /*!< MEAS_RATE_H: VAL position */
#define XENSIV_PASCO2_REG_MEAS_RATE_H_VAL_MSK           (0xffU << XENSIV_PASCO2_REG_MEAS_RATE_H_VAL_POS)        /*!< MEAS_RATE_H: VAL mask */

#define XENSIV_PASCO2_REG_MEAS_RATE_L_VAL_POS           (0U)                                                    /*!< MEAS_RATE_L: VAL position */
#define XENSIV_PASCO2_REG_MEAS_RATE_L_VAL_MSK           (0xffU << XENSIV_PASCO2_REG_MEAS_RATE_L_VAL_POS)        /*!< MEAS_RATE_L: VAL mask */

#define XENSIV_PASCO2_REG_MEAS_CFG_OP_MODE_POS          (0U)                                                    /*!< MEAS_CFG: OP_MODE position */
#define XENSIV_PASCO2_REG_MEAS_CFG_OP_MODE_MSK          (0x03U << XENSIV_PASCO2_REG_MEAS_CFG_OP_MODE_POS)       /*!< MEAS_CFG: OP_MODE mask */
#define XENSIV_PASCO2_REG_MEAS_CFG_BOC_CFG_POS          (2U)                                                    /*!< MEAS_CFG: BOC_CFG position */
#define XENSIV_PASCO2_REG_MEAS_CFG_BOC_CFG_MSK          (0x03U << XENSIV_PASCO2_REG_MEAS_CFG_BOC_CFG_POS)       /*!< MEAS_CFG: BOC_CFG mask */
#define XENSIV_PASCO2_REG_MEAS_CFG_PWM_MODE_POS         (4U)                                                    /*!< MEAS_CFG: PWM_MODE position */
#define XENSIV_PASCO2_REG_MEAS_CFG_PWM_MODE_MSK         (0x01U << XENSIV_PASCO2_REG_MEAS_CFG_PWM_MODE_POS)      /*!< MEAS_CFG: PWM_MODE mask */
#define XENSIV_PASCO2_REG_MEAS_CFG_PWM_OUTEN_POS        (5U)                                                    /*!< MEAS_CFG: PWM_OUTEN position */
#define XENSIV_PASCO2_REG_MEAS_CFG_PWM_OUTEN_MSK        (0x01U << XENSIV_PASCO2_REG_MEAS_CFG_PWM_OUTEN_POS)     /*!< MEAS_CFG: PWM_OUTEN mask */

#define XENSIV_PASCO2_REG_CO2PPM_H_VAL_POS              (0U)                                                    /*!< CO2PPM_H: VAL position */
#define XENSIV_PASCO2_REG_CO2PPM_H_VAL_MSK              (0xffU << XENSIV_PASCO2_REG_CO2PPM_H_VAL_POS)           /*!< CO2PPM_H: VAL mask */

#define XENSIV_PASCO2_REG_CO2PPM_L_VAL_POS              (0U)                                                    /*!< CO2PPM_L: VAL position */
#define XENSIV_PASCO2_REG_CO2PPM_L_VAL_MSK              (0xffU << XENSIV_PASCO2_REG_CO2PPM_L_VAL_POS)           /*!< CO2PPM_L: VAL mask */

#define XENSIV_PASCO2_REG_MEAS_STS_ALARM_CLR_POS        (0U)                                                    /*!< MEAS_STS: ALARM_CLR position */
#define XENSIV_PASCO2_REG_MEAS_STS_ALARM_CLR_MSK        (0x01U << XENSIV_PASCO2_REG_MEAS_STS_ALARM_CLR_POS)     /*!< MEAS_STS: ALARM_CLR mask */
#define XENSIV_PASCO2_REG_MEAS_STS_INT_STS_CLR_POS      (1U)                                                    /*!< MEAS_STS: INT_STS_CLR position */
#define XENSIV_PASCO2_REG_MEAS_STS_INT_STS_CLR_MSK      (0x01U << XENSIV_PASCO2_REG_MEAS_STS_INT_STS_CLR_POS)   /*!< MEAS_STS: INT_STS_CLR mask */
#define XENSIV_PASCO2_REG_MEAS_STS_ALARM_POS            (2U)                                                    /*!< MEAS_STS: ALARM position */
#define XENSIV_PASCO2_REG_MEAS_STS_ALARM_MSK            (0x01U << XENSIV_PASCO2_REG_MEAS_STS_ALARM_POS)         /*!< MEAS_STS: ALARM mask */
#define XENSIV_PASCO2_REG_MEAS_STS_INT_STS_POS          (3U)                                                    /*!< MEAS_STS: INT_STS position */
#define XENSIV_PASCO2_REG_MEAS_STS_INT_STS_MSK          (0x01U << XENSIV_PASCO2_REG_MEAS_STS_INT_STS_POS)       /*!< MEAS_STS: INT_STS mask */
#define XENSIV_PASCO2_REG_MEAS_STS_DRDY_POS             (4U)                                                    /*!< MEAS_STS: DRDY position */
#define XENSIV_PASCO2_REG_MEAS_STS_DRDY_MSK             (0x01U << XENSIV_PASCO2_REG_MEAS_STS_DRDY_POS)          /*!< MEAS_STS: DRDY mask */

#define XENSIV_PASCO2_REG_INT_CFG_ALARM_TYP_POS         (0U)                                                    /*!< INT_CFG: ALARM_TYP position */
#define XENSIV_PASCO2_REG_INT_CFG_ALARM_TYP_MSK         (0x01U << XENSIV_PASCO2_REG_INT_CFG_ALARM_TYP_POS)      /*!< INT_CFG: ALARM_TYP mask */
#define XENSIV_PASCO2_REG_INT_CFG_INT_FUNC_POS          (1U)                                                    /*!< INT_CFG: INT_FUNC position */
#define XENSIV_PASCO2_REG_INT_CFG_INT_FUNC_MSK          (0x07U << XENSIV_PASCO2_REG_INT_CFG_INT_FUNC_POS)       /*!< INT_CFG: INT_FUNC mask */
#define XENSIV_PASCO2_REG_INT_CFG_INT_TYP_POS           (4U)                                                    /*!< INT_CFG: INT_TYP position */
#define XENSIV_PASCO2_REG_INT_CFG_INT_TYP_MSK           (0x01U << XENSIV_PASCO2_REG_INT_CFG_INT_TYP_POS)        /*!< INT_CFG: INT_TYP mask */

#define XENSIV_PASCO2_REG_ALARM_TH_H_VAL_POS            (0U)                                                    /*!< ALARM_TH_H: VAL position */
#define XENSIV_PASCO2_REG_ALARM_TH_H_VAL_MSK            (0xffU << XENSIV_PASCO2_REG_ALARM_TH_H_VAL_POS)         /*!< ALARM_TH_H: VAL mask */

#define XENSIV_PASCO2_REG_ALARM_TH_L_VAL_POS            (0U)                                                    /*!< ALARM_TH_L: VAL position */
#define XENSIV_PASCO2_REG_ALARM_TH_L_VAL_MSK            (0xffU << XENSIV_PASCO2_REG_ALARM_TH_L_VAL_POS)         /*!< ALARM_TH_L: VAL mask */

#define XENSIV_PASCO2_REG_PRESS_REF_H_VAL_POS           (0U)                                                    /*!< PRESS_REF_H: VAL position */
#define XENSIV_PASCO2_REG_PRESS_REF_H_VAL_MSK           (0xffU << XENSIV_PASCO2_REG_PRESS_REF_H_VAL_POS)        /*!< PRESS_REF_H: VAL mask */

#define XENSIV_PASCO2_REG_PRESS_REF_L_VAL_POS           (0U)                                                    /*!< PRESS_REF_L: VAL position */
#define XENSIV_PASCO2_REG_PRESS_REF_L_VAL_MSK           (0xffU << XENSIV_PASCO2_REG_PRESS_REF_L_VAL_POS)        /*!< PRESS_REF_L: VAL mask */

#define XENSIV_PASCO2_REG_CALIB_REF_H_VAL_POS           (0U)                                                    /*!< CALIB_REF_H: VAL position */
#define XENSIV_PASCO2_REG_CALIB_REF_H_VAL_MSK           (0xffU << XENSIV_PASCO2_REG_CALIB_REF_H_VAL_POS)        /*!< CALIB_REF_H: VAL mask */

#define XENSIV_PASCO2_REG_CALIB_REF_L_VAL_POS           (0U)                                                    /*!< CALIB_REF_L: VAL position */
#define XENSIV_PASCO2_REG_CALIB_REF_L_VAL_MSK           (0xffU << XENSIV_PASCO2_REG_CALIB_REF_L_VAL_POS)        /*!< CALIB_REF_L: VAL mask */

#define XENSIV_PASCO2_REG_SCRATCH_PAD_VAL_POS           (0U)                                                    /*!< SCRATCH_PAD: VAL position */
#define XENSIV_PASCO2_REG_SCRATCH_PAD_VAL_MSK           (0xffU << XENSIV_PASCO2_REG_SCRATCH_PAD_VAL_POS)        /*!< SCRATCH_PAD: VAL mask */

#define XENSIV_PASCO2_REG_SENS_RST_SRTRG_POS            (0U)                                                    /*!< SENS_RST: SRTRG position */
#define XENSIV_PASCO2_REG_SENS_RST_SRTRG_MSK            (0xffU << XENSIV_PASCO2_REG_SENS_RST_SRTRG_POS)         /*!< SENS_RST: SRTRG mask */

/** \} group_board_libs */

#endif

/* End of File */
