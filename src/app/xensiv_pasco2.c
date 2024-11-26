/******************************************************************************
* File Name: xensiv_pasco2.c
*
* Description: This file contains the functions for interacting with the
*              XENSIV™ PAS CO2 sensor.
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

#include "xensiv_pasco2.h"
#include "xensiv_pasco2_platform.h"

#define XENSIV_PASCO2_COMM_DELAY_MS             (5U)
#define XENSIV_PASCO2_COMM_TEST_VAL             (0xA5U)

#define XENSIV_PASCO2_SOFT_RESET_DELAY_MS       (2000U)

#define XENSIV_PASCO2_FCS_MEAS_RATE_S           (10)

#define XENSIV_PASCO2_I2C_WRITE_BUFFER_LEN      (17U)

static int32_t xensiv_pasco2_i2c_read(const xensiv_pasco2_t * dev, uint8_t reg_addr, uint8_t * data, uint8_t len)
{
    if(dev == NULL){
        xensiv_pasco2_plat_assert(0);
        return 0;
    }
    xensiv_pasco2_plat_assert(dev->ctx != NULL);
    xensiv_pasco2_plat_assert(reg_addr <= XENSIV_PASCO2_REG_SENS_RST);
    if(data == NULL){
        xensiv_pasco2_plat_assert(0);
        return 0;
    }

    return xensiv_pasco2_plat_i2c_read(reg_addr, data, len);
}

static int32_t xensiv_pasco2_i2c_write(const xensiv_pasco2_t * dev, uint8_t reg_addr, const uint8_t * data, uint8_t len)
{
    if(dev == NULL){
        xensiv_pasco2_plat_assert(0);
        return 0;
    }
    xensiv_pasco2_plat_assert(dev->ctx != NULL);
    xensiv_pasco2_plat_assert(reg_addr <= XENSIV_PASCO2_REG_SENS_RST);
    if(data == NULL){
        xensiv_pasco2_plat_assert(0);
        return 0;
    }
    xensiv_pasco2_plat_assert((len + 1U) < XENSIV_PASCO2_I2C_WRITE_BUFFER_LEN);

    uint8_t w_data[XENSIV_PASCO2_I2C_WRITE_BUFFER_LEN];
    w_data[0] = reg_addr;
    for (uint8_t i = 0; i < len; ++i)
    {
        w_data[i + 1U] = data[i];
    }

    uint16_t w_len = (uint16_t)((uint16_t)len + 1U);

    return xensiv_pasco2_plat_i2c_write(w_data, w_len);
}

static int32_t xensiv_pasco2_init(const xensiv_pasco2_t * dev)
{
    xensiv_pasco2_plat_assert(dev != NULL);

    /* Check communication */
    uint8_t data = XENSIV_PASCO2_COMM_TEST_VAL;

    int32_t res = xensiv_pasco2_set_reg(dev, (uint8_t)XENSIV_PASCO2_REG_SCRATCH_PAD, &data, 1U);

    if (XENSIV_PASCO2_OK == res)
    {
        res = xensiv_pasco2_get_reg(dev, (uint8_t)XENSIV_PASCO2_REG_SCRATCH_PAD, &data, 1U);
    }

    if ((XENSIV_PASCO2_OK == res) && (XENSIV_PASCO2_COMM_TEST_VAL == data))
    {
        /* Soft reset */
        res = xensiv_pasco2_cmd(dev, XENSIV_PASCO2_CMD_SOFT_RESET);
        xensiv_pasco2_plat_delay(XENSIV_PASCO2_SOFT_RESET_DELAY_MS);

        if (XENSIV_PASCO2_OK == res)
        {
            /* Read the sensor status and verify if the sensor is ready */
            res = xensiv_pasco2_get_reg(dev, (uint8_t)XENSIV_PASCO2_REG_SENS_STS, &data, 1U);
        }

        if (XENSIV_PASCO2_OK == res)
        {
            if ((data & XENSIV_PASCO2_REG_SENS_STS_ICCER_MSK) != 0U)
            {
                res = XENSIV_PASCO2_ICCERR;
            }
            else if ((data & XENSIV_PASCO2_REG_SENS_STS_ORVS_MSK) != 0U)
            {
                res = XENSIV_PASCO2_ORVS;
            }
            else if ((data & XENSIV_PASCO2_REG_SENS_STS_ORTMP_MSK) != 0U)
            {
                res = XENSIV_PASCO2_ORTMP;
            }
            else if ((data & XENSIV_PASCO2_REG_SENS_STS_SEN_RDY_MSK) == 0U)
            {
                res = XENSIV_PASCO2_ERR_NOT_READY;
            }
            else
            {
                res = XENSIV_PASCO2_OK;
            }
        }
    }
    else
    {
        res = XENSIV_PASCO2_ERR_COMM;
    }

    return res;
}

int32_t xensiv_pasco2_init_i2c(xensiv_pasco2_t * dev, void * ctx)
{
    if((dev == NULL) || (ctx == NULL)){
        xensiv_pasco2_plat_assert(0);
        return 0;
    }

    dev->ctx = ctx;
    dev->read = xensiv_pasco2_i2c_read;
    dev->write = xensiv_pasco2_i2c_write;

    return xensiv_pasco2_init(dev);
}

int32_t xensiv_pasco2_set_reg(const xensiv_pasco2_t * dev, uint8_t reg_addr, const uint8_t * data, uint8_t len)
{
    if((dev == NULL) || (data == NULL)){
        xensiv_pasco2_plat_assert(0);
        return 0;
    }

    int32_t res = dev->write(dev, reg_addr, data, len);
    xensiv_pasco2_plat_delay(XENSIV_PASCO2_COMM_DELAY_MS);

    return res;
}

int32_t xensiv_pasco2_get_reg(const xensiv_pasco2_t * dev, uint8_t reg_addr, uint8_t * data, uint8_t len)
{
    if((dev == NULL) || (data == NULL)){
        xensiv_pasco2_plat_assert(0);
        return 0;
    }

    int32_t res = dev->read(dev, reg_addr, data, len);
    xensiv_pasco2_plat_delay(XENSIV_PASCO2_COMM_DELAY_MS);

    return res;
}

int32_t xensiv_pasco2_get_id(const xensiv_pasco2_t * dev, xensiv_pasco2_id_t * id)
{
    if((dev == NULL) || (id == NULL)){
        xensiv_pasco2_plat_assert(0);
        return 0;
    }

    return xensiv_pasco2_get_reg(dev, (uint8_t)XENSIV_PASCO2_REG_PROD_ID, &(id->u), 1U);
}

int32_t xensiv_pasco2_get_status(const xensiv_pasco2_t * dev, xensiv_pasco2_status_t * status)
{
    if((dev == NULL) || (status == NULL)){
        xensiv_pasco2_plat_assert(0);
        return 0;
    }

    return xensiv_pasco2_get_reg(dev, (uint8_t)XENSIV_PASCO2_REG_SENS_STS, &(status->u), 1U);
}

int32_t xensiv_pasco2_clear_status(const xensiv_pasco2_t * dev, uint8_t mask)
{
    if(dev == NULL){
        xensiv_pasco2_plat_assert(0);
        return 0;
    }

    return xensiv_pasco2_set_reg(dev, (uint8_t)XENSIV_PASCO2_REG_SENS_STS, &mask, 1U);
}

int32_t xensiv_pasco2_get_interrupt_config(const xensiv_pasco2_t * dev, xensiv_pasco2_interrupt_config_t * int_config)
{
    xensiv_pasco2_plat_assert(dev != NULL);
    xensiv_pasco2_plat_assert(int_config != NULL);

    return xensiv_pasco2_get_reg(dev, (uint8_t)XENSIV_PASCO2_REG_INT_CFG, &(int_config->u), 1U);
}

int32_t xensiv_pasco2_set_interrupt_config(const xensiv_pasco2_t * dev, xensiv_pasco2_interrupt_config_t int_config)
{
    xensiv_pasco2_plat_assert(dev != NULL);

    return xensiv_pasco2_set_reg(dev, (uint8_t)XENSIV_PASCO2_REG_INT_CFG, &(int_config.u), 1U);
}

int32_t xensiv_pasco2_get_measurement_config(const xensiv_pasco2_t * dev, xensiv_pasco2_measurement_config_t * meas_config)
{
    xensiv_pasco2_plat_assert(dev != NULL);
    xensiv_pasco2_plat_assert(meas_config != NULL);

    return xensiv_pasco2_get_reg(dev, (uint8_t)XENSIV_PASCO2_REG_MEAS_CFG, &(meas_config->u), 1U);
}

int32_t xensiv_pasco2_set_measurement_config(const xensiv_pasco2_t * dev, xensiv_pasco2_measurement_config_t meas_config)
{
    xensiv_pasco2_plat_assert(dev != NULL);

    return xensiv_pasco2_set_reg(dev, (uint8_t)XENSIV_PASCO2_REG_MEAS_CFG, &(meas_config.u), 1U);
}

int32_t xensiv_pasco2_get_result(const xensiv_pasco2_t * dev, uint16_t * val)
{
    if((dev == NULL) || (val == NULL)){
        xensiv_pasco2_plat_assert(0);
        return 0;
    }

    xensiv_pasco2_meas_status_t meas_status;
    int32_t res = xensiv_pasco2_get_measurement_status(dev, &meas_status);

    if (XENSIV_PASCO2_OK == res)
    {
        if (meas_status.b.drdy != 0U)
        {
            res = xensiv_pasco2_get_reg(dev, (uint8_t)XENSIV_PASCO2_REG_CO2PPM_H, (uint8_t *)val, 2U);
            *val = xensiv_pasco2_plat_htons(*val);
        }
        else
        {
            res =  XENSIV_PASCO2_READ_NRDY;
        }
    }

    return res;
}

int32_t xensiv_pasco2_set_measurement_rate(const xensiv_pasco2_t * dev, uint16_t val)
{
    xensiv_pasco2_plat_assert(dev != NULL);
    xensiv_pasco2_plat_assert((val >= XENSIV_PASCO2_MEAS_RATE_MIN) && (val <= XENSIV_PASCO2_MEAS_RATE_MAX));

    val = xensiv_pasco2_plat_htons(val);
    return xensiv_pasco2_set_reg(dev, (uint8_t)XENSIV_PASCO2_REG_MEAS_RATE_H, (uint8_t *)&val, 2U);
}

int32_t xensiv_pasco2_get_measurement_status(const xensiv_pasco2_t * dev, xensiv_pasco2_meas_status_t * status)
{
    xensiv_pasco2_plat_assert(dev != NULL);
    xensiv_pasco2_plat_assert(status != NULL);

    return xensiv_pasco2_get_reg(dev, (uint8_t)XENSIV_PASCO2_REG_MEAS_STS, &(status->u), 1U);
}

int32_t xensiv_pasco2_clear_measurement_status(const xensiv_pasco2_t * dev, uint8_t mask)
{
    xensiv_pasco2_plat_assert(dev != NULL);

    return xensiv_pasco2_set_reg(dev, (uint8_t)XENSIV_PASCO2_REG_MEAS_STS, &mask, 1U);
}

int32_t xensiv_pasco2_set_alarm_threshold(const xensiv_pasco2_t * dev, uint16_t val)
{
    xensiv_pasco2_plat_assert(dev != NULL);

    val = (uint16_t)xensiv_pasco2_plat_htons(val);
    return xensiv_pasco2_set_reg(dev, (uint8_t)XENSIV_PASCO2_REG_ALARM_TH_H, (uint8_t *)&val, 2U);
}

int32_t xensiv_pasco2_set_pressure_compensation(const xensiv_pasco2_t * dev, uint16_t val)
{
    xensiv_pasco2_plat_assert(dev != NULL);

    val = (uint16_t)xensiv_pasco2_plat_htons(val);
    return xensiv_pasco2_set_reg(dev, (uint8_t)XENSIV_PASCO2_REG_PRESS_REF_H, (uint8_t *)&val, 2U);
}

int32_t xensiv_pasco2_set_offset_compensation(const xensiv_pasco2_t * dev, uint16_t val)
{
    xensiv_pasco2_plat_assert(dev != NULL);

    val = (uint16_t)xensiv_pasco2_plat_htons(val);
    return xensiv_pasco2_set_reg(dev, (uint8_t)XENSIV_PASCO2_REG_CALIB_REF_H, (uint8_t *)&val, 2U);
}

int32_t xensiv_pasco2_set_scratch_pad(const xensiv_pasco2_t * dev, uint8_t val)
{
    xensiv_pasco2_plat_assert(dev != NULL);

    return xensiv_pasco2_set_reg(dev, (uint8_t)XENSIV_PASCO2_REG_SCRATCH_PAD, &val, 1U);
}

int32_t xensiv_pasco2_get_scratch_pad(const xensiv_pasco2_t * dev, uint8_t * val)
{
    xensiv_pasco2_plat_assert(dev != NULL);
    xensiv_pasco2_plat_assert(val != NULL);

    return xensiv_pasco2_get_reg(dev, (uint8_t)XENSIV_PASCO2_REG_SCRATCH_PAD, val, 1U);
}

int32_t xensiv_pasco2_cmd(const xensiv_pasco2_t * dev, xensiv_pasco2_cmd_t cmd)
{
    xensiv_pasco2_plat_assert(dev != NULL);

    return xensiv_pasco2_set_reg(dev, (uint8_t)XENSIV_PASCO2_REG_SENS_RST, (const uint8_t * )&cmd, 1U);
}

int32_t xensiv_pasco2_start_single_mode(const xensiv_pasco2_t * dev)
{
    xensiv_pasco2_plat_assert(dev != NULL);

    xensiv_pasco2_measurement_config_t meas_config;
    int32_t res = xensiv_pasco2_get_measurement_config(dev, &meas_config);

    if (XENSIV_PASCO2_OK == res)
    {
        if (meas_config.b.op_mode != XENSIV_PASCO2_OP_MODE_IDLE)
        {
            meas_config.b.op_mode = XENSIV_PASCO2_OP_MODE_IDLE;
            res = xensiv_pasco2_set_measurement_config(dev, meas_config);
        }
    }

    if (XENSIV_PASCO2_OK == res)
    {
        meas_config.b.op_mode = XENSIV_PASCO2_OP_MODE_SINGLE;
        meas_config.b.boc_cfg = XENSIV_PASCO2_BOC_CFG_AUTOMATIC;
        res = xensiv_pasco2_set_measurement_config(dev, meas_config);
    }

    return res;
}

int32_t xensiv_pasco2_start_continuous_mode(const xensiv_pasco2_t * dev, uint16_t val)
{
    xensiv_pasco2_plat_assert(dev != NULL);
    xensiv_pasco2_plat_assert((val >= XENSIV_PASCO2_MEAS_RATE_MIN) && (val <= XENSIV_PASCO2_MEAS_RATE_MAX));

    xensiv_pasco2_measurement_config_t meas_config;
    int32_t res = xensiv_pasco2_get_measurement_config(dev, &meas_config);

    if (XENSIV_PASCO2_OK == res)
    {
        if (meas_config.b.op_mode != XENSIV_PASCO2_OP_MODE_IDLE)
        {
            meas_config.b.op_mode = XENSIV_PASCO2_OP_MODE_IDLE;
            res = xensiv_pasco2_set_measurement_config(dev, meas_config);
        }
    }

    if (XENSIV_PASCO2_OK == res)
    {
        val = xensiv_pasco2_plat_htons(val);
        res = xensiv_pasco2_set_reg(dev, (uint8_t)XENSIV_PASCO2_REG_MEAS_RATE_H, (uint8_t *)&val, 2U);
    }

    if (XENSIV_PASCO2_OK == res)
    {
        meas_config.b.op_mode = XENSIV_PASCO2_OP_MODE_CONTINUOUS;
        meas_config.b.boc_cfg = XENSIV_PASCO2_BOC_CFG_AUTOMATIC;
        res = xensiv_pasco2_set_measurement_config(dev, meas_config);
    }

    return res;
}

int32_t xensiv_pasco2_perform_forced_compensation(const xensiv_pasco2_t * dev, uint16_t co2_ref)
{
    if(dev == NULL){
        xensiv_pasco2_plat_assert(0);
        return 0;
    }

    xensiv_pasco2_measurement_config_t meas_config;
    int32_t res = xensiv_pasco2_get_measurement_config(dev, &meas_config);

    if (XENSIV_PASCO2_OK == res)
    {
        meas_config.b.op_mode = XENSIV_PASCO2_OP_MODE_IDLE;
        res = xensiv_pasco2_set_measurement_config(dev, meas_config);
    }

    if (XENSIV_PASCO2_OK == res)
    {
        res = xensiv_pasco2_set_measurement_rate(dev, XENSIV_PASCO2_FCS_MEAS_RATE_S);
    }

    if (XENSIV_PASCO2_OK == res)
    {
        res = xensiv_pasco2_set_offset_compensation(dev, co2_ref);
    }

    if (XENSIV_PASCO2_OK == res)
    {
        meas_config.b.op_mode = XENSIV_PASCO2_OP_MODE_CONTINUOUS;
        meas_config.b.boc_cfg = XENSIV_PASCO2_BOC_CFG_FORCED;
        res = xensiv_pasco2_set_measurement_config(dev, meas_config);
    }

    if (XENSIV_PASCO2_OK == res)
    {
        /* wait until the FCS is finished */
        do
        {
            res = xensiv_pasco2_get_measurement_config(dev, &meas_config);
        } while ((XENSIV_PASCO2_OK != res) || (XENSIV_PASCO2_BOC_CFG_FORCED == meas_config.b.boc_cfg));
    }

    if (XENSIV_PASCO2_OK == res)
    {
        meas_config.b.op_mode = XENSIV_PASCO2_OP_MODE_IDLE;
        res = xensiv_pasco2_set_measurement_config(dev, meas_config);
    }

    if (XENSIV_PASCO2_OK == res)
    {
        res = xensiv_pasco2_cmd(dev, XENSIV_PASCO2_CMD_SAVE_FCS_CALIB_OFFSET);
    }

    return res;
}

/* End of File */
