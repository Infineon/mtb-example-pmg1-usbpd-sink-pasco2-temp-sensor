/******************************************************************************
* File Name:  I2CMaster.h
*
* Description:  This file provides constants and parameter values for the I2C
*               Master block which reads from PASCO2 I2C Slave sensor.
*
* Related Document: See Readme.md
*
********************************************************************************
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

#ifndef SOURCE_I2CMASTER_H_
#define SOURCE_I2CMASTER_H_

#include "cy_pdl.h"
#include "cybsp.h"
#include "cy_result.h"
#include "xensiv_pasco2.h"

/*******************************************************************************
* Macros
*******************************************************************************/
#define I2C_SUCCESS             (0UL)
#define I2C_FAILURE             (1UL)
#define WRITE_BUFF_SIZE         (17u)

#define TRANSFER_CMPLT          (0x00UL)
#define READ_CMPLT              (TRANSFER_CMPLT)

/* Start address of slave buffer */
#define EZI2C_BUFFER_ADDRESS    (0x00)

/* Address of I2C PASCO2 slave */
#define I2C_PASCO2_SLAVE_ADDR   (0x28U)

#ifndef CY_RSLT_MODULE_BOARD_HARDWARE_XENSIV_PASCO2
#define CY_RSLT_MODULE_BOARD_HARDWARE_XENSIV_PASCO2 0x01CA
#endif

/** Result code indicating a communication error */
#define XENSIV_PASCO2_RSLT_ERR_COMM\
    (CY_RSLT_CREATE(CY_RSLT_TYPE_ERROR, CY_RSLT_MODULE_BOARD_HARDWARE_XENSIV_PASCO2, XENSIV_PASCO2_ERR_COMM))

/** Result code indicating that an unexpectedly large I2C write was requested which is not supported */
#define XENSIV_PASCO2_RSLT_ERR_WRITE_TOO_LARGE\
    (CY_RSLT_CREATE(CY_RSLT_TYPE_ERROR, CY_RSLT_MODULE_BOARD_HARDWARE_XENSIV_PASCO2, XENSIV_PASCO2_ERR_WRITE_TOO_LARGE))

/** Result code indicating that the sensor is not yet ready after reset */
#define XENSIV_PASCO2_RSLT_ERR_NOT_READY\
    (CY_RSLT_CREATE(CY_RSLT_TYPE_ERROR, CY_RSLT_MODULE_BOARD_HARDWARE_XENSIV_PASCO2, XENSIV_PASCO2_ERR_NOT_READY))

/** Result code indicating whether an invalid command has been received by the serial communication interface */
#define XENSIV_PASCO2_RSLT_ICCERR\
    (CY_RSLT_CREATE(CY_RSLT_TYPE_ERROR, CY_RSLT_MODULE_BOARD_HARDWARE_XENSIV_PASCO2, XENSIV_PASCO2_ICCERR))

/** Result code indicating whether a condition where VDD12V has been outside the specified valid range has been detected */
#define XENSIV_PASCO2_RSLT_ORVS\
    (CY_RSLT_CREATE(CY_RSLT_TYPE_ERROR, CY_RSLT_MODULE_BOARD_HARDWARE_XENSIV_PASCO2, XENSIV_PASCO2_ORVS))

/** Result code indicating whether a condition where the temperature has been outside the specified valid range has been detected */
#define XENSIV_PASCO2_RSLT_ORTMP\
    (CY_RSLT_CREATE(CY_RSLT_TYPE_ERROR, CY_RSLT_MODULE_BOARD_HARDWARE_XENSIV_PASCO2, XENSIV_PASCO2_ORTMP))

/** Result code indicating that a new CO2 value is not yet ready */
#define XENSIV_PASCO2_RSLT_READ_NRDY\
    (CY_RSLT_CREATE(CY_RSLT_TYPE_ERROR, CY_RSLT_MODULE_BOARD_HARDWARE_XENSIV_PASCO2, XENSIV_PASCO2_READ_NRDY))

/*******************************************************************************
* Function Prototypes
*******************************************************************************/

typedef struct
{
    uint8_t regAddress;
    uint8_t length;
} RegBlock_t;

/*******************************************************************************
 * Global Function Declaration
 ******************************************************************************/

/**
 * @brief This function initializes and enables master SCB.
 *
 * @return uint32_t indicating status of initialization
 */
uint32_t InitI2CMaster(void);

/**
 * @brief This function initializes I2C block for PASCO2 sensor.
 *
 * @param[in] dev Pointer to the XENSIV™ PAS CO2 sensor device
 * @return cy_rslt_t XENSIV_PASCO2_OK if writing to the register was successful; an error indicating what went wrong otherwise
 */
cy_rslt_t xensiv_pasco2_my_init_i2c(xensiv_pasco2_t * dev);

/**
 * @brief This function gets result from the CO2 register of PASCO2 sensor.
 *
 * @param[in] dev Pointer to the XENSIV™ PAS CO2 sensor device
 * @param[in] press_ref New pressure reference value to apply
 * @param[out] co2_ppm_val Pointer to populate with the CO2 ppm value
 * @return cy_rslt_t XENSIV_PASCO2_OK if writing to the register was successful; an error indicating what went wrong otherwise
 */
cy_rslt_t xensiv_pasco2_my_read(const xensiv_pasco2_t * dev, uint16_t press_ref, uint16_t * co2_ppm_val);

#endif /* SOURCE_I2CMASTER_H_ */
