/******************************************************************************
* File Name: xensiv_pasco2_platform.h
*
* Description: XENSIV PAS CO2 sensor driver platform dependencies
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

#ifndef XENSIV_PASCO2_PLATFORM_H_
#define XENSIV_PASCO2_PLATFORM_H_

/**
 * \addtogroup group_board_libs_platform XENSIV PAS CO2 Sensor platform functions
 * \{
 * XENSIV PAS CO2 Sensor platform functions.
 * 
 * To adapt the driver to your platform you need to provide an implementation for the functions
 * declared in this file. See example implementation in xensiv_pasco2_mtb.c using the PSoC6 HAL.
 *
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Target platform-specific functions to perform I2C write/read transfer.
 * Synchronously writes a block of data and optionally receive a block of data.
 * @param[in] ctx Target platform object
 * @param[in] dev_addr device address (7-bit)
 * @param[in] tx_buffer I2C send data
 * @param[in] tx_len I2C send data size
 * @param[in] rx_buffer I2C receive data @note Can be NULL to indicate no read access.
 * @param[in] rx_len I2C receive data size
 * @return XENSIV_PASCO2_OK if the I2C transfer was successful; an error indicating what went wrong otherwise
 */

int32_t xensiv_pasco2_plat_i2c_read(uint8_t regAddr, uint8_t * rx_buffer, size_t rx_len);
int32_t xensiv_pasco2_plat_i2c_write(const uint8_t * tx_buffer, size_t tx_len);

/**
 * @brief Target platform-specific function that waits for a specified time period in milliseconds
 *
 * @param[in] ms Number of miliseconds to wait for
 */
void xensiv_pasco2_plat_delay(uint32_t ms);

/**
 * @brief Target platform-specific function to reverse the byte order (16-bit)
 * A sample implementation would look like 
 * \code
 *  return ((uint16_t)(((x & 0x00ffU) << 8) |
 *                     ((x & 0xff00U) >> 8)));
 * \endcode
 * @param[in] x Value to reverse
 * @return Reversed value
 */
uint16_t xensiv_pasco2_plat_htons(uint16_t x);

/**
 * @brief Target platform-specific function that implements a runtime assertion; used to verify the assumptions made by the program and take appropiate actions if assumption is false
 *
 * @param[in] expr Expression to be verified
 */
void xensiv_pasco2_plat_assert(int expr);

#ifdef __cplusplus
}
#endif

/** \} group_board_libs_platform */

#endif // ifndef XENSIV_PASCO2_PLATFORM_H_

/* End of File */
