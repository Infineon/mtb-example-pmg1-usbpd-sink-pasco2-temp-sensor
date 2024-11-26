/******************************************************************************
* File Name:  I2CMaster.c
*
* Description:  This file contains all the functions and variables required for
*               proper operation of I2C Master block to read the values
*               from PASCO2 I2C Slave sensor.
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

/* Header file includes */
#include "i2c_master.h"
#include "stdio.h"
#include "config.h"
#include "cy_pdutils_sw_timer.h"
#include "cy_pdstack_common.h"


/*******************************************************************************
* Macros
*******************************************************************************/
#if ENABLE_PASCO2_I2C_INTERFACE
/* PASCO2 Measure Rate*/
#define XENSIV_PASCO2_MEAS_RATE_S               (10)
/* PASCO2 Result Error*/
#define XENSIV_PASCO2_ERROR(x)                  (((x) == XENSIV_PASCO2_OK) ? CY_RSLT_SUCCESS :\
                                                CY_RSLT_CREATE(CY_RSLT_TYPE_ERROR, CY_RSLT_MODULE_BOARD_HARDWARE_XENSIV_PASCO2, (x)))

/* I2C master interrupt macros */
#define I2C_INTR_NUM                            CYBSP_I2C_IRQ
#define I2C_INTR_PRIORITY                       (3UL)

/* I2C transfer delay */
#define CY_SCB_WAIT_DELAY                       (100UL)

extern cy_stc_pdutils_sw_timer_t gl_TimerCtx;
extern cy_stc_pdstack_context_t gl_PdStackPort0Ctx;

/* Timer used to ensure I2C transfers to the MPS complete on time. */
#define SCB_I2C_TIMER_ID                        (0xC3u)
/* The MPS transfer timeout is set to 10 ms timeout period. */
#define SCB_I2C_TIMER_PERIOD                    (10u)



/* MUX access timeout indication. */
static volatile bool i2c_xfer_timeout = false;

/* Timer callback used for I2C transactions to the MUX. */
static void i2c_xfer_timer_cb(
        cy_timer_id_t id,            /**< Timer ID for which callback is being generated. */
        void *callbackContext)       /**< Timer module Context. */
{
    /*
     * I2C transmission to MUX continues longer than timeout. Slave doesn't
     * respond.
     */
    i2c_xfer_timeout = true;
}


/*******************************************************************************
* Global variables
*******************************************************************************/
/* Structure for master transfer configuration */
cy_stc_scb_i2c_master_xfer_config_t masterTransferCfg =
{
    .slaveAddress = XENSIV_PASCO2_I2C_ADDR, //I2C_PASCO2_SLAVE_ADDR
    .buffer       = NULL,
    .bufferSize   = 0U,
    .xferPending  = false
};


/** The instance-specific context structure.
 * It is used by the driver for internal configuration and
 * data keeping for the I2C. Do not modify anything in this structure.
 */
static cy_stc_scb_i2c_context_t CYBSP_I2C_context;

/*******************************************************************************
* Function Declaration
*******************************************************************************/
void CYBSP_I2C_Interrupt(void);
/*******************************************************************************
* Function Name: CYBSP_I2C_Interrupt
****************************************************************************//**
*
* Summary:
*   Invokes the Cy_SCB_I2C_Interrupt() PDL driver function.
*
*******************************************************************************/
void CYBSP_I2C_Interrupt(void)
{
    Cy_SCB_I2C_Interrupt(CYBSP_I2C_HW, &CYBSP_I2C_context);
}

/*******************************************************************************
* Function Name: InitI2CMaster
********************************************************************************
*
* Summary:
*   This function initializes and enables master SCB
*
* Return:
*   Status of initialization
*
*******************************************************************************/
uint32_t InitI2CMaster(void)
{
    cy_en_scb_i2c_status_t initStatus;
    cy_en_sysint_status_t sysStatus;
    cy_stc_sysint_t CYBSP_I2C_SCB_IRQ_cfg =
    {
            /*.intrSrc =*/ I2C_INTR_NUM,
            /*.intrPriority =*/ I2C_INTR_PRIORITY
    };

    /*Initialize and enable the I2C in master mode*/
    initStatus = Cy_SCB_I2C_Init(CYBSP_I2C_HW, &CYBSP_I2C_config, &CYBSP_I2C_context);
    if(initStatus != CY_SCB_I2C_SUCCESS)
    {
        return I2C_FAILURE;
    }

    /* Hook interrupt service routine */
    sysStatus = Cy_SysInt_Init(&CYBSP_I2C_SCB_IRQ_cfg, &CYBSP_I2C_Interrupt);
    if(sysStatus != CY_SYSINT_SUCCESS)
    {
        return I2C_FAILURE;
    }
    NVIC_EnableIRQ((IRQn_Type) CYBSP_I2C_SCB_IRQ_cfg.intrSrc);
    Cy_SCB_I2C_Enable(CYBSP_I2C_HW, &CYBSP_I2C_context);
    return I2C_SUCCESS;
}

/*******************************************************************************
* Function Name: xensiv_pasco2_my_init_i2c
********************************************************************************
*
* Summary:
*   This function initializes I2C block for PASCO2 sensor
*
* Return:
*   Status of initialization
*
*******************************************************************************/
cy_rslt_t xensiv_pasco2_my_init_i2c(xensiv_pasco2_t * dev)
{
    CY_ASSERT(dev != NULL);

    int32_t res = xensiv_pasco2_init_i2c(dev, &CYBSP_I2C_context);
    if (XENSIV_PASCO2_OK == res)
    {
        res = xensiv_pasco2_start_continuous_mode(dev, XENSIV_PASCO2_MEAS_RATE_S);
    }
    return XENSIV_PASCO2_ERROR(res);
}

/*******************************************************************************
* Function Name: xensiv_pasco2_plat_i2c_read
********************************************************************************
*
* Summary:
*   This function performs I2C read for PASCO2 sensor
*
* Return:
*   Status of read
*
*******************************************************************************/
int32_t xensiv_pasco2_plat_i2c_read (uint8_t regAddr, uint8_t * rx_buffer, size_t rx_len)
{
    cy_en_scb_i2c_status_t status;
    uint32_t masterStatus =0;
    uint32_t sts =I2C_SUCCESS;

    /* Clear the timeout flag and start a timer. */
    i2c_xfer_timeout = false;
    Cy_PdUtils_SwTimer_Start  (&gl_TimerCtx, (void *)&gl_PdStackPort0Ctx, (cy_timer_id_t)SCB_I2C_TIMER_ID,
            SCB_I2C_TIMER_PERIOD, i2c_xfer_timer_cb);


    uint8_t PASCO2_read_buff = regAddr;
    /* Setup transfer specific parameters */
    masterTransferCfg.buffer     = &PASCO2_read_buff;
    masterTransferCfg.bufferSize = 0x01;

    /* Initiate write pointer value transaction */
    status = Cy_SCB_I2C_MasterWrite(CYBSP_I2C_HW, &masterTransferCfg, &CYBSP_I2C_context);
    if(status == CY_SCB_I2C_SUCCESS)
    {
        /* Wait until master completes write transfer*/
        do
        {
            masterStatus  = Cy_SCB_I2C_MasterGetStatus(CYBSP_I2C_HW, &CYBSP_I2C_context);

        } while ((0UL != (masterStatus & CY_SCB_I2C_MASTER_BUSY)) && (!i2c_xfer_timeout));

        /* Setup transfer specific parameters */
        masterTransferCfg.buffer     = rx_buffer;
        masterTransferCfg.bufferSize = rx_len;

        /* Initiate read register transaction */
        status = Cy_SCB_I2C_MasterRead(CYBSP_I2C_HW, &masterTransferCfg, &CYBSP_I2C_context);
        if(status == CY_SCB_I2C_SUCCESS)
        {
            /* Wait until master completes read transfer */
            do
            {
                masterStatus  = Cy_SCB_I2C_MasterGetStatus(CYBSP_I2C_HW, &CYBSP_I2C_context);

            } while ((0UL != (masterStatus & CY_SCB_I2C_MASTER_BUSY)) && (!i2c_xfer_timeout));

        }
    }
    if((status != CY_SCB_I2C_SUCCESS) || (i2c_xfer_timeout == true) ||
        (0UL != (masterStatus & CY_SCB_I2C_MASTER_ADDR_NAK)))
            sts =  I2C_FAILURE;



    return (sts == I2C_SUCCESS)
    ? XENSIV_PASCO2_OK
    : XENSIV_PASCO2_ERR_COMM;

}

/*******************************************************************************
* Function Name: xensiv_pasco2_plat_i2c_write
********************************************************************************
*
* Summary:
*   This function performs I2C write for PASCO2 sensor
*
* Return:
*   Status of write
*
*******************************************************************************/
int32_t xensiv_pasco2_plat_i2c_write (const uint8_t * tx_buffer, size_t tx_len)
{
    cy_en_scb_i2c_status_t status;
    uint32_t masterStatus =0;
    uint32_t sts =I2C_SUCCESS;

    /* Clear the timeout flag and start a timer. */
    i2c_xfer_timeout = false;
    Cy_PdUtils_SwTimer_Start  (&gl_TimerCtx, (void *)&gl_PdStackPort0Ctx, (cy_timer_id_t)SCB_I2C_TIMER_ID,
                SCB_I2C_TIMER_PERIOD, i2c_xfer_timer_cb);


    uint8_t PASCO2_write_reg[WRITE_BUFF_SIZE] = {0};

    for(uint8_t i=0 ; i<tx_len ;  i++)
    {
        PASCO2_write_reg[i] = tx_buffer[i];
    }


    if(tx_len > WRITE_BUFF_SIZE)
        tx_len = WRITE_BUFF_SIZE;

    /* Setup transfer specific parameters */
    masterTransferCfg.buffer     = PASCO2_write_reg;
    masterTransferCfg.bufferSize = tx_len;

    /* Initiate write pointer value transaction */
    status = Cy_SCB_I2C_MasterWrite(CYBSP_I2C_HW, &masterTransferCfg, &CYBSP_I2C_context);
    if(status == CY_SCB_I2C_SUCCESS)
    {
        /* Wait until master completes write transfer*/
        do
        {
            masterStatus  = Cy_SCB_I2C_MasterGetStatus(CYBSP_I2C_HW, &CYBSP_I2C_context);

        } while ((0UL != (masterStatus & CY_SCB_I2C_MASTER_BUSY))&&  (!i2c_xfer_timeout));
    }

    if((status != CY_SCB_I2C_SUCCESS) || (i2c_xfer_timeout == true) ||
        (0UL != (masterStatus & CY_SCB_I2C_MASTER_ADDR_NAK)))
            sts =  I2C_FAILURE;



    return (sts == I2C_SUCCESS)
    ? XENSIV_PASCO2_OK
    : XENSIV_PASCO2_ERR_COMM;
}

/*******************************************************************************
* Function Name: xensiv_pasco2_my_read
********************************************************************************
*
* Summary:
*   This function gets result from the CO2 register of PASCO2 sensor
*
* Return:
*   Status of get result
*
*******************************************************************************/
cy_rslt_t xensiv_pasco2_my_read(const xensiv_pasco2_t * dev, uint16_t press_ref, uint16_t * co2_ppm_val)
{
    CY_ASSERT(dev != NULL);
    CY_ASSERT(co2_ppm_val != NULL);


    int32_t res = xensiv_pasco2_get_result(dev, co2_ppm_val);
    if (XENSIV_PASCO2_OK == res)
    {
        res = xensiv_pasco2_set_pressure_compensation(dev, press_ref);
    }

    return XENSIV_PASCO2_ERROR(res);
}

void xensiv_pasco2_plat_delay(uint32_t ms)
{
    Cy_SysLib_Delay(ms);
}

uint16_t xensiv_pasco2_plat_htons(uint16_t x)
{
    return (uint16_t)__REV16(x);
}

void xensiv_pasco2_plat_assert(int expr)
{
    CY_ASSERT(expr);
    (void)expr; /* make release build */
}

#endif /* ENABLE_PASCO2_I2C_INTERFACE  */
