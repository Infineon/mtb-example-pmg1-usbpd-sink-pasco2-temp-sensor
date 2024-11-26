/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the USBPD Sink PAS CO2 and
*              Temperature Sensor Example for ModusToolbox.
*
* Related Document: See README.md
*
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


#include "cy_pdl.h"
#include "cybsp.h"
#include "config.h"
#include "i2c_master.h"

#include "cy_pdutils_sw_timer.h"
#include "cy_usbpd_common.h"
#include "cy_pdstack_common.h"
#include "cy_usbpd_typec.h"
#include "cy_pdstack_dpm.h"
#include "cy_usbpd_vbus_ctrl.h"
#include "cy_usbpd_phy.h"
#include "instrumentation.h"
#include "app.h"
#include "pdo.h"
#include "psink.h"
#include "swap.h"
#include "vdm.h"
#include "charger_detect.h"
#include "mtbcfg_ezpd.h"
#include <stdio.h>
#include <inttypes.h>

/* CY ASSERT failure */
#define CY_ASSERT_FAILED                            (0u)

/* PAS CO2 sensor macros */
#if ENABLE_PASCO2_I2C_INTERFACE
#define WAIT_SENSOR_RDY_MS                          (2000U)     /* Wait time in milliseconds for the sensor to be ready */
#define DEFAULT_PRESSURE_REF_HPA                    (0x3F7)     /* Default atmospheric pressure to compensate for (hPa) */
#define PASCO2_INIT_RETRY_COUNT                     (3u)
#define TIMER_PERIOD_MSEC                           (10000U)    /* Sensor Measurement time period in milliseconds (Minimum value is 10000U) */
#define TIMER_OFFSET_MSEC                           (499U)      /* This offset is added to the timer period to avoid conflict between record
                                                                 * and read of PAS CO2 sensor measurements. The XENSIV PAS CO2 sensor records
                                                                 * CO2 data every 10 seconds */

volatile bool pasco2_sensor_read_flag = false;                  /* Read flag for PAS CO2 sensor */
volatile bool pasco2_sensor_init_flag = false;                  /* Init flag for PAS CO2 sensor */
volatile bool is_pasco2_initialized = false;                    /* Flag for PAS CO2 sensor initialization status*/
volatile bool is_12V_contract_established = false;              /* Flag for 12 V PD contract establishment status*/
uint8_t init_retry_counter = PASCO2_INIT_RETRY_COUNT;           /* Retry counter for PAS CO2 sensor initialization*/
volatile uint32_t pasco2_current_timer_val;                     /* Variable to store current timer value*/
#endif /* ENABLE_PASCO2_I2C_INTERFACE */

/* Die-Temperature read macros */
#define TEMP_SLOPE                                  (-742)      /* This value can be adjusted to calibrate the slope of the temperature correlation graph*/
#define TEMP_OFFSET                                 (493)       /* This value can be adjusted to calibrate the offset of the temperature correlation graph*/

/*******************************************************************************
* Global Variables
*******************************************************************************/
static xensiv_pasco2_t xensiv_pasco2;

/* LED blink rate in milliseconds */
static uint16_t gl_LedBlinkRate = LED_TIMER_PERIOD_DETACHED;

cy_stc_pdutils_sw_timer_t        gl_TimerCtx;
cy_stc_usbpd_context_t   gl_UsbPdPort0Ctx;

cy_stc_pdstack_context_t gl_PdStackPort0Ctx;
#if PMG1_PD_DUALPORT_ENABLE
cy_stc_usbpd_context_t gl_UsbPdPort1Ctx;
cy_stc_pdstack_context_t gl_PdStackPort1Ctx;
#endif /* PMG1_PD_DUALPORT_ENABLE */

/* Structure for UART Context */
cy_stc_scb_uart_context_t CYBSP_UART_context;

cy_stc_syspm_callback_params_t callback_params =
{
    .base = CYBSP_UART_HW,
    .context = &CYBSP_UART_context
};

cy_stc_syspm_callback_t uart_deep_sleep_cb =
{
    Cy_SCB_UART_DeepSleepCallback,
    CY_SYSPM_DEEPSLEEP,
    0,
    &callback_params,
    NULL,
    NULL
};

const cy_stc_pdstack_dpm_params_t pdstack_port0_dpm_params =
{
        .dpmSnkWaitCapPeriod = 400,
        .dpmRpAudioAcc = CY_PD_RP_TERM_RP_CUR_DEF,
        .dpmDefCableCap = 300,
        .muxEnableDelayPeriod = 0,
        .typeCSnkWaitCapPeriod = 0,
        .defCur = 90
};

#if PMG1_PD_DUALPORT_ENABLE
const cy_stc_pdstack_dpm_params_t pdstack_port1_dpm_params =
{
        .dpmSnkWaitCapPeriod = 400,
        .dpmRpAudioAcc = CY_PD_RP_TERM_RP_CUR_DEF,
        .dpmDefCableCap = 300,
        .muxEnableDelayPeriod = 0,
        .typeCSnkWaitCapPeriod = 0,
        .defCur = 90
};
#endif /* PMG1_PD_DUALPORT_ENABLE */
cy_stc_pdstack_context_t * gl_PdStackContexts[NO_OF_TYPEC_PORTS] =
{
        &gl_PdStackPort0Ctx,
#if PMG1_PD_DUALPORT_ENABLE
        &gl_PdStackPort1Ctx
#endif /* PMG1_PD_DUALPORT_ENABLE */
};

bool mux_ctrl_init(uint8_t port)
{
    /* No MUXes to be controlled on the PMG1 proto kits. */
    CY_UNUSED_PARAMETER(port);
    return true;
}

const cy_stc_sysint_t wdt_interrupt_config =
{
    .intrSrc = (IRQn_Type)srss_interrupt_wdt_IRQn,
    .intrPriority = 0U,
};

const cy_stc_sysint_t usbpd_port0_intr0_config =
{
    .intrSrc = (IRQn_Type)mtb_usbpd_port0_IRQ,
    .intrPriority = 0U,
};

const cy_stc_sysint_t usbpd_port0_intr1_config =
{
    .intrSrc = (IRQn_Type)mtb_usbpd_port0_DS_IRQ,
    .intrPriority = 0U,
};

#if PMG1_PD_DUALPORT_ENABLE
const cy_stc_sysint_t usbpd_port1_intr0_config =
{
    .intrSrc = (IRQn_Type)mtb_usbpd_port1_IRQ,
    .intrPriority = 0U,
};

const cy_stc_sysint_t usbpd_port1_intr1_config =
{
    .intrSrc = (IRQn_Type)mtb_usbpd_port1_DS_IRQ,
    .intrPriority = 0U,
};
#endif /* PMG1_PD_DUALPORT_ENABLE */

cy_stc_pdstack_context_t *get_pdstack_context(uint8_t portIdx)
{
    return (gl_PdStackContexts[portIdx]);
}

/* Solution PD event handler */
void sln_pd_event_handler(cy_stc_pdstack_context_t* ctx, cy_en_pdstack_app_evt_t evt, const void *data)
{
    (void)ctx;

    if(evt == APP_EVT_HANDLE_EXTENDED_MSG)
    {
        cy_stc_pd_packet_extd_t * ext_mes = (cy_stc_pd_packet_extd_t * )data;
        if ((ext_mes->msg != CY_PDSTACK_EXTD_MSG_SECURITY_RESP) && (ext_mes->msg != CY_PDSTACK_EXTD_MSG_FW_UPDATE_RESP))
        {
            /* Send Not supported message */
            Cy_PdStack_Dpm_SendPdCommand(ctx, CY_PDSTACK_DPM_CMD_SEND_NOT_SUPPORTED, NULL, true, NULL);
        }
    }
    if(evt == APP_EVT_PD_CONTRACT_NEGOTIATION_COMPLETE)
    {
        /*
         * Validate 12V contract
         */
        if (ctx->dpmConfig.attach)
        {
            if (ctx->dpmConfig.contractExist)
            {
                if(ctx ->dpmStat.contract.minVolt != CY_PD_VSAFE_12V)
                {
                    char_t contract_string[60];
                    sprintf(contract_string, "\n\r 12V contract not established ");
                    Cy_SCB_UART_PutString(CYBSP_UART_HW, contract_string);
                    is_12V_contract_established = false;
                }
                else
                {
                    is_12V_contract_established = true;
                }
            }
        }
    }
}

void instrumentation_cb(uint8_t port, inst_evt_t evt)
{
    uint8_t evt_offset = APP_TOTAL_EVENTS;
    evt += evt_offset;
    sln_pd_event_handler(&gl_PdStackPort0Ctx, (cy_en_pdstack_app_evt_t)evt, NULL);
}

static void wdt_interrupt_handler(void)
{
    /* Clear WDT pending interrupt */
    Cy_WDT_ClearInterrupt();

#if (CY_PDUTILS_TIMER_TICKLESS_ENABLE == 0)
    /* Load the timer match register. */
    Cy_WDT_SetMatch((Cy_WDT_GetCount() + gl_TimerCtx.multiplier));
#endif /* (TIMER_TICKLESS_ENABLE == 0) */

    /* Invoke the timer handler. */
    Cy_PdUtils_SwTimer_InterruptHandler (&(gl_TimerCtx));
}

static void cy_usbpd0_intr0_handler(void)
{
    Cy_USBPD_Intr0Handler(&gl_UsbPdPort0Ctx);
}

static void cy_usbpd0_intr1_handler(void)
{
    Cy_USBPD_Intr1Handler(&gl_UsbPdPort0Ctx);
}

#if PMG1_PD_DUALPORT_ENABLE
static void cy_usbpd1_intr0_handler(void)
{
    Cy_USBPD_Intr0Handler(&gl_UsbPdPort1Ctx);
}

static void cy_usbpd1_intr1_handler(void)
{
    Cy_USBPD_Intr1Handler(&gl_UsbPdPort1Ctx);
}
#endif /* PMG1_PD_DUALPORT_ENABLE */

#if APP_FW_LED_ENABLE
void led_timer_cb (
        cy_timer_id_t id,            /**< Timer ID for which callback is being generated. */
        void *callbackContext)       /**< Timer module Context. */
{
    cy_stc_pdstack_context_t *stack_ctx = (cy_stc_pdstack_context_t *)callbackContext;
#if BATTERY_CHARGING_ENABLE
    const chgdet_status_t    *chgdet_stat;
#endif /* #if BATTERY_CHARGING_ENABLE */

    /* Toggle the User LED and re-start timer to schedule the next toggle event. */
    Cy_GPIO_Inv(CYBSP_USER_LED_PORT, CYBSP_USER_LED_PIN);

    /* Calculate the desired LED blink rate based on the correct Type-C connection state. */
    if (stack_ctx->dpmConfig.attach)
    {
        if (stack_ctx->dpmConfig.contractExist)
        {
            gl_LedBlinkRate = LED_TIMER_PERIOD_PD_SRC;
        }
        else
        {
#if BATTERY_CHARGING_ENABLE
            chgdet_stat = chgdet_get_status(stack_ctx);
            if (chgdet_stat->chgdet_fsm_state == CHGDET_FSM_SINK_DCP_CONNECTED)
            {
                gl_LedBlinkRate = LED_TIMER_PERIOD_DCP_SRC;
            }
            else if (chgdet_stat->chgdet_fsm_state == CHGDET_FSM_SINK_CDP_CONNECTED)
            {
                gl_LedBlinkRate = LED_TIMER_PERIOD_CDP_SRC;
            }
            else
#endif /* BATTERY_CHARGING_ENABLE */
            {
                gl_LedBlinkRate = LED_TIMER_PERIOD_TYPEC_SRC;
            }
        }
    }
    else
    {
        gl_LedBlinkRate = LED_TIMER_PERIOD_DETACHED;
    }

    Cy_PdUtils_SwTimer_Start (&gl_TimerCtx, callbackContext, id, gl_LedBlinkRate, led_timer_cb);
}
#endif /* APP_FW_LED_ENABLE */

cy_stc_pd_dpm_config_t* get_dpm_connect_stat(void)
{
    return &(gl_PdStackPort0Ctx.dpmConfig);
}

#if PMG1_PD_DUALPORT_ENABLE
cy_stc_pd_dpm_config_t* get_dpm_port1_connect_stat()
{
    return &(gl_PdStackPort1Ctx.dpmConfig);
}
#endif /* PMG1_PD_DUALPORT_ENABLE */

/*
 * Application callback functions for the DPM. Since this application
 * uses the functions provided by the stack, loading the stack defaults.
 */
const cy_stc_pdstack_app_cbk_t app_callback =
{
    app_event_handler,
    vconn_enable,
    vconn_disable,
    vconn_is_present,
    vbus_is_present,
    vbus_discharge_on,
    vbus_discharge_off,
    psnk_set_voltage,
    psnk_set_current,
    psnk_enable,
    psnk_disable,
    eval_src_cap,
    eval_dr_swap,
    eval_pr_swap,
    eval_vconn_swap,
    eval_vdm,
    vbus_get_value,
};

cy_stc_pdstack_app_cbk_t* app_get_callback_ptr(cy_stc_pdstack_context_t * context)
{
    (void)context;
    /* Solution callback pointer is same for all ports */
    return ((cy_stc_pdstack_app_cbk_t *)(&app_callback));
}

#if ENABLE_PASCO2_I2C_INTERFACE
/* This function handles the timer interrupt that set the flag for the sensor to be read.*/
void timer_interrupt_handler(void)
{
    /* Clear the terminal count interrupt */
    Cy_TCPWM_ClearInterrupt(CYBSP_TIMER_HW, CYBSP_TIMER_NUM, CY_TCPWM_INT_ON_TC );

#if ENABLE_PASCO2_I2C_INTERFACE
    if (pasco2_current_timer_val == TIMER_PERIOD_MSEC)
    {
        pasco2_sensor_read_flag = true;
    }
    else if(pasco2_current_timer_val == WAIT_SENSOR_RDY_MS)
    {
        pasco2_sensor_init_flag = true;
    }
#endif /* ENABLE_PASCO2_I2C_INTERFACE */
}



/* This function handles the initialization of the Timer modules used to measure the temperature data periodically */
void init_sensor_timer_module (uint32_t timer_period)
{
    /* Start the TCPWM component in timer/counter mode. The return value of the
     * function indicates whether the arguments are valid or not. It is not used
     * here for simplicity.
     */
    cy_rslt_t result;

    result = Cy_TCPWM_Counter_Init(CYBSP_TIMER_HW, CYBSP_TIMER_NUM, &CYBSP_TIMER_config);
    if(result != CY_TCPWM_SUCCESS)
    {
        CY_ASSERT(CY_ASSERT_FAILED);
    }

    /* Set the timer period in milliseconds. To count N cycles, period should be
     * set to N-1.
     */
    Cy_TCPWM_Counter_SetPeriod(CYBSP_TIMER_HW, CYBSP_TIMER_NUM, timer_period+TIMER_OFFSET_MSEC);

    /* Check if the desired interrupt is enabled prior to triggering */
    if (0UL != (CY_TCPWM_INT_ON_TC & Cy_TCPWM_GetInterruptMask(CYBSP_TIMER_HW, CYBSP_TIMER_NUM)))
    {
       Cy_TCPWM_SetInterrupt(CYBSP_TIMER_HW, CYBSP_TIMER_NUM, CY_TCPWM_INT_ON_TC);
    }

     /* Clear any pending interrupt */
    Cy_TCPWM_ClearInterrupt(CYBSP_TIMER_HW, CYBSP_TIMER_NUM, CY_TCPWM_INT_ON_CC_OR_TC );

    /* Enable the Timer Module */
    Cy_TCPWM_Counter_Enable(CYBSP_TIMER_HW, CYBSP_TIMER_NUM);

    /* Trigger a software start on the counter instance. This is required when
     * no other hardware input signal is connected to the component to act as
     * a trigger source.
     */
    Cy_TCPWM_TriggerStart(CYBSP_TIMER_HW, CYBSP_TIMER_MASK);

}

/* Below function disables the Timer module */
void deinit_sensor_timer_module(void)
{
    Cy_TCPWM_Counter_Disable(CYBSP_TIMER_HW, CYBSP_TIMER_NUM);
}
#endif /* ENABLE_PASCO2_I2C_INTERFACE */

int main(void)
{
    /* Variables to store ADC outputs */
    uint8_t BJT_val;
    uint8_t BANDGAP;

    /* Variables to store temperature data */
    int8_t temp;
    char_t temp_string[30];

    /* Variables to store CO2 level data */
    uint16_t ppm;
    char_t co2_string[50];

    cy_rslt_t result;
    cy_en_sysint_status_t intr_result;
    cy_stc_pdutils_timer_config_t timerConfig;

    /* Initialize the device and board peripherals */
    result = cybsp_init();

    /* Configure and enable the UART peripheral */
    Cy_SCB_UART_Init(CYBSP_UART_HW, &CYBSP_UART_config, &CYBSP_UART_context);
    Cy_SysPm_RegisterCallback(&uart_deep_sleep_cb);
    Cy_SCB_UART_Enable(CYBSP_UART_HW);

#if DEBUG_PRINT
     /* Sequence to clear screen */
     Cy_SCB_UART_PutString(CYBSP_UART_HW, "\x1b[2J\x1b[;H");

     /* Print "CO2 Sensor" */
     Cy_SCB_UART_PutString(CYBSP_UART_HW, "****************** ");
     Cy_SCB_UART_PutString(CYBSP_UART_HW, "\n\nPAS CO2 and Temperature Sensor\n\n");
     Cy_SCB_UART_PutString(CYBSP_UART_HW, "****************** \r\n\n");
#endif

    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
        #if DEBUG_PRINT
                /* Print Debug messages over UART */
                Cy_SCB_UART_PutString(CYBSP_UART_HW, "\n\r CYBSP Init Failed \0" );
        #endif
    }

    /*
     * Register the interrupt handler for the watchdog timer. This timer is used to
     * implement the soft timers required by the USB-PD Stack.
     */
    Cy_SysInt_Init(&wdt_interrupt_config, &wdt_interrupt_handler);
    NVIC_EnableIRQ(wdt_interrupt_config.intrSrc);

    timerConfig.sys_clk_freq = Cy_SysClk_ClkSysGetFrequency();
    timerConfig.hw_timer_ctx = NULL;

    /* Initialize the soft timer module. */
    Cy_PdUtils_SwTimer_Init(&gl_TimerCtx, &timerConfig);

#if ENABLE_PASCO2_I2C_INTERFACE
    /* Enable TCPWM interrupt */
    cy_stc_sysint_t intrCfg =
    {
       /*.intrSrc =*/ CYBSP_TIMER_IRQ, /* Interrupt source is Timer interrupt */
       /*.intrPriority =*/ 3UL   /* Interrupt priority is 3 */
    };
    intr_result = Cy_SysInt_Init(&intrCfg, timer_interrupt_handler);
    if (intr_result != CY_SYSINT_SUCCESS)
    {
        CY_ASSERT(CY_ASSERT_FAILED);
    }
    /* Enable Timer Interrupt */
    NVIC_EnableIRQ(intrCfg.intrSrc);
    /* Initialize I2C master SCB */
    result = InitI2CMaster();
    if (result != I2C_SUCCESS)
    {
        #if DEBUG_PRINT
            /* Print Debug messages over UART */
            Cy_SCB_UART_PutString(CYBSP_UART_HW, "\n\r I2C Master Init Failed \0" );
        #endif
    }
#endif /* ENABLE_PASCO2_I2C_INTERFACE */

    /* The PMG1-S0 kit does not have PULL on the I2C lines used for this project.
     * Configure the GPIO drive mode to pull up using internal pull up for I2C lines.
     * PMG1-S1, PMG1-S2 and PMG1-S3 kit have external pull up on the I2C lines used for this project.
     */
#if ENABLE_PASCO2_I2C_INTERFACE && PMG1_S0
    Cy_GPIO_SetDrivemode(CYBSP_I2C_SCL_PORT,CYBSP_I2C_SCL_PIN, CY_GPIO_DM_PULLUP);
    Cy_GPIO_SetDrivemode(CYBSP_I2C_SDA_PORT,CYBSP_I2C_SDA_PIN, CY_GPIO_DM_PULLUP);
#endif /* ENABLE_PASCO2_I2C_INTERFACE && PMG1_S0 */

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize the instrumentation related data structures. */
    instrumentation_init();

    /* Register callback function to be executed when instrumentation fault occurs. */
    instrumentation_register_cb((instrumentation_cb_t)instrumentation_cb);

    /* Configure and enable the USBPD interrupts */
    Cy_SysInt_Init(&usbpd_port0_intr0_config, &cy_usbpd0_intr0_handler);
    NVIC_EnableIRQ(usbpd_port0_intr0_config.intrSrc);

    Cy_SysInt_Init(&usbpd_port0_intr1_config, &cy_usbpd0_intr1_handler);
    NVIC_EnableIRQ(usbpd_port0_intr1_config.intrSrc);

#if PMG1_PD_DUALPORT_ENABLE
    /* Configure and enable the USBPD interrupts for Port #1. */
    Cy_SysInt_Init(&usbpd_port1_intr0_config, &cy_usbpd1_intr0_handler);
    NVIC_EnableIRQ(usbpd_port1_intr0_config.intrSrc);

    Cy_SysInt_Init(&usbpd_port1_intr1_config, &cy_usbpd1_intr1_handler);
    NVIC_EnableIRQ(usbpd_port1_intr1_config.intrSrc);
#endif /* PMG1_PD_DUALPORT_ENABLE */

    /* Initialize the USBPD driver */
#if defined(CY_DEVICE_CCG3)
    Cy_USBPD_Init(&gl_UsbPdPort0Ctx, 0, mtb_usbpd_port0_HW, NULL,
            (cy_stc_usbpd_config_t *)&mtb_usbpd_port0_config, get_dpm_connect_stat);
#else
    Cy_USBPD_Init(&gl_UsbPdPort0Ctx, 0, mtb_usbpd_port0_HW, mtb_usbpd_port0_HW_TRIM,
            (cy_stc_usbpd_config_t *)&mtb_usbpd_port0_config, get_dpm_connect_stat);
#if PMG1_PD_DUALPORT_ENABLE
    Cy_USBPD_Init(&gl_UsbPdPort1Ctx, 1, mtb_usbpd_port1_HW, mtb_usbpd_port1_HW_TRIM,
            (cy_stc_usbpd_config_t *)&mtb_usbpd_port1_config, get_dpm_port1_connect_stat);
#endif /* PMG1_PD_DUALPORT_ENABLE */
#endif

    /* Initialize the Device Policy Manager. */
    Cy_PdStack_Dpm_Init(&gl_PdStackPort0Ctx,
                       &gl_UsbPdPort0Ctx,
                       &mtb_usbpd_port0_pdstack_config,
                       app_get_callback_ptr(&gl_PdStackPort0Ctx),
                       &pdstack_port0_dpm_params,
                       &gl_TimerCtx);

#if PMG1_PD_DUALPORT_ENABLE
    Cy_PdStack_Dpm_Init(&gl_PdStackPort1Ctx,
                       &gl_UsbPdPort1Ctx,
                       &mtb_usbpd_port1_pdstack_config,
                       app_get_callback_ptr(&gl_PdStackPort1Ctx),
                       &pdstack_port1_dpm_params,
                       &gl_TimerCtx);
#endif /* PMG1_PD_DUALPORT_ENABLE */

    /* Perform application level initialization. */
    app_init(&gl_PdStackPort0Ctx);
#if PMG1_PD_DUALPORT_ENABLE
    app_init(&gl_PdStackPort1Ctx);
#endif /* PMG1_PD_DUALPORT_ENABLE */

    /* Initialize the fault configuration values */
    fault_handler_init_vars(&gl_PdStackPort0Ctx);
#if PMG1_PD_DUALPORT_ENABLE
    fault_handler_init_vars(&gl_PdStackPort1Ctx);
#endif /* PMG1_PD_DUALPORT_ENABLE */

    /* Start any timers or tasks associated with application instrumentation. */
    instrumentation_start();

    /* Start the device policy manager operation. This will initialize the USB-PD block and enable connect detection. */
    Cy_PdStack_Dpm_Start(&gl_PdStackPort0Ctx);
#if PMG1_PD_DUALPORT_ENABLE
    Cy_PdStack_Dpm_Start(&gl_PdStackPort1Ctx);
#endif /* PMG1_PD_DUALPORT_ENABLE */


#if APP_FW_LED_ENABLE
    /* Start a timer that will blink the FW ACTIVE LED. */
    Cy_PdUtils_SwTimer_Start (&gl_TimerCtx, (void *)&gl_PdStackPort0Ctx, (cy_timer_id_t)LED_TIMER_ID,
            gl_LedBlinkRate, led_timer_cb);
#endif /* APP_FW_LED_ENABLE */

    /*
     * After the initialization is complete, keep processing the USB-PD device policy manager task in a loop.
     */

    for (;;)
    {
        /* Handle the device policy tasks for each PD port. */
        Cy_PdStack_Dpm_Task(&gl_PdStackPort0Ctx);
#if PMG1_PD_DUALPORT_ENABLE
        Cy_PdStack_Dpm_Task(&gl_PdStackPort1Ctx);
#endif /* PMG1_PD_DUALPORT_ENABLE */

        /* Perform any application level tasks. */
        app_task(&gl_PdStackPort0Ctx);
#if PMG1_PD_DUALPORT_ENABLE
        app_task(&gl_PdStackPort1Ctx);
#endif /* PMG1_PD_DUALPORT_ENABLE */

#if ENABLE_PASCO2_I2C_INTERFACE
        if(is_12V_contract_established == true)
        {
            if(is_pasco2_initialized == false)
            {
                /* Check the initialization counter for the PAS CO2 Sensor */
                if(init_retry_counter>0)
                {
                    /* Initializing the timer module to start timer for WAIT_SENSOR_RDY_MS
                     * to wait for the sensor to be ready*/
                    if (0UL == (CY_TCPWM_COUNTER_STATUS_COUNTER_RUNNING &
                                    Cy_TCPWM_Counter_GetStatus(CYBSP_TIMER_HW, CYBSP_TIMER_NUM)))
                    {
                        pasco2_current_timer_val = WAIT_SENSOR_RDY_MS;
                        init_sensor_timer_module(pasco2_current_timer_val);
                        pasco2_sensor_init_flag = false;
                    }
                    if(pasco2_sensor_init_flag == true)
                    {
                        /* Initialize the PAS CO2 Sensor */
                        result = xensiv_pasco2_my_init_i2c(&xensiv_pasco2);
                        pasco2_sensor_init_flag = false;
                        /* Check if the sensor initialization was successful */
                        if (result == CY_RSLT_SUCCESS)
                        {
                            is_pasco2_initialized = true;

                            /* Denitializing the timer module to stop timer after successful initialization of sensor*/
                            deinit_sensor_timer_module();

                    #if DEBUG_PRINT
                            /* Print Debug messages over UART. */
                            Cy_SCB_UART_PutString(CYBSP_UART_HW, "\n\r PAS CO2 device initialization successful \0" );
                    #endif

                            /* Initializing the timer module to start timer for TIMER_PERIOD_MSEC for periodic
                            * CO2 and temperature data measurements*/
                            if(0UL == (CY_TCPWM_COUNTER_STATUS_COUNTER_RUNNING &
                                    Cy_TCPWM_Counter_GetStatus(CYBSP_TIMER_HW, CYBSP_TIMER_NUM)))
                            {
                                pasco2_current_timer_val = TIMER_PERIOD_MSEC;
                                init_sensor_timer_module(pasco2_current_timer_val);
                                pasco2_sensor_read_flag = false;
                            }
                        }
                        else
                        {
                            is_pasco2_initialized = false;
                            init_retry_counter--;
                            if(init_retry_counter==0)
                            {
                            #if DEBUG_PRINT
                                /* Print Debug messages over UART. */
                                char_t fail_string[60];
                                sprintf(fail_string, "\n\r PAS CO2 device initialization failed after %d attempts", PASCO2_INIT_RETRY_COUNT);
                                Cy_SCB_UART_PutString(CYBSP_UART_HW, fail_string);
                            #endif
                            }
                        }
                    }
                }
            }


            if ((pasco2_sensor_read_flag == true) &&
                (is_pasco2_initialized == true))
            {
                /* Read the CO2 sensor data */
                result = xensiv_pasco2_my_read(&xensiv_pasco2, DEFAULT_PRESSURE_REF_HPA, &ppm);

                /* Read the BJT-based temperature sensor data in the ADC block of PMG1 device
                 * BJT VBE voltage is the input to the 8-bit ADC and the sampled bandgap reference voltage
                 * are used to calculate the temperature using the conversion formula
                 * */
                BJT_val = Cy_USBPD_Adc_Sample(&gl_UsbPdPort0Ctx, CY_USBPD_ADC_ID_0, CY_USBPD_ADC_INPUT_BJT);
                BANDGAP = Cy_USBPD_Adc_Sample(&gl_UsbPdPort0Ctx, CY_USBPD_ADC_ID_0, CY_USBPD_ADC_INPUT_BANDGAP);

                /* Calibration formula: Slope:-742, Offset: 493 (from characterization data) */
                temp = (((TEMP_SLOPE) * ((BJT_val << 16) / BANDGAP)) >> 16) + TEMP_OFFSET;

                if (result == CY_RSLT_SUCCESS)
                {
                    /* Conversion from int to char for UART transmit */
                    sprintf(co2_string,"\n\r CO2: %d ppm", ppm);
                    sprintf(temp_string, "\n\r Temp: %d C", temp);

                    /* Send a string over serial terminal */
                    Cy_SCB_UART_PutString(CYBSP_UART_HW, co2_string);
                    Cy_SCB_UART_PutString(CYBSP_UART_HW, temp_string);
                }
                /* Set the read flag to false to wait till timer expires again */
                pasco2_sensor_read_flag = false;
            }
        }
#endif /* ENABLE_PASCO2_I2C_INTERFACE */


        /* Perform tasks associated with instrumentation. */
        instrumentation_task();

#if SYS_DEEPSLEEP_ENABLE
        /* If possible, enter deep sleep mode for power saving. */
        if(is_12V_contract_established == false)
        {
            system_sleep(&gl_PdStackPort0Ctx,
    #if PMG1_PD_DUALPORT_ENABLE
                    &gl_PdStackPort1Ctx
    #else
                    NULL
    #endif /* PMG1_PD_DUALPORT_ENABLE */
                    );
        }
#endif /* SYS_DEEPSLEEP_ENABLE */
    }
}

/* [] END OF FILE */
