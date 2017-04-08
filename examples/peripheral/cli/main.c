/**
 * Copyright (c) 2016 - 2017, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "nrf.h"
#include "nrf_drv_clock.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_drv_power.h"
#include "nrf_cli.h"

#include "nrf_cli_uart.h"
#include "nrf_cli_rtt.h"
#include "nrf_queue.h"

#include "app_error.h"
#include "app_util.h"

#ifdef NRF52840_XXAA
#include "nrf_drv_usbd.h"
#include "app_usbd_core.h"
#include "app_usbd.h"
#include "app_usbd_string_desc.h"
#include "app_usbd_cdc_acm.h"
#include "nrf_cli_cdc_acm.h"
#endif

#include "boards.h"

/**@file
 * @defgroup CLI_example main.c
 *
 * @{
 *
 */
#define LED_USB_RESUME     (BSP_BOARD_LED_0)
#define LED_CDC_ACM_OPEN   (BSP_BOARD_LED_1)
#define LED_CDC_ACM_RX     (BSP_BOARD_LED_2)
#define LED_CDC_ACM_TX     (BSP_BOARD_LED_3)


#ifdef BOARD_PCA10056
/**
 * @brief Enable power USB detection
 *
 * Configure if example supports USB port connection
 */
#ifndef USBD_POWER_DETECTION
#define USBD_POWER_DETECTION true
#endif

/**
 * @brief  USB connection status
 * */
static bool m_usb_connected = false;

static void power_usb_event_handler(nrf_drv_power_usb_evt_t event)
{
    switch(event)
    {
        case NRF_DRV_POWER_USB_EVT_DETECTED:

            if (!nrf_drv_usbd_is_enabled())
            {
                app_usbd_enable();
            }
            break;
        case NRF_DRV_POWER_USB_EVT_REMOVED:
            m_usb_connected = false;
            break;
        case NRF_DRV_POWER_USB_EVT_READY:
            m_usb_connected = true;
            break;
        default:
            ASSERT(false);
    }
}

static void usb_start(void)
{
    if (USBD_POWER_DETECTION)
    {
        static const nrf_drv_power_usbevt_config_t config =
        {
            .handler = power_usb_event_handler
        };
        ret_code_t ret;

        ret = nrf_drv_power_usbevt_init(&config);
        APP_ERROR_CHECK(ret);
    }
    else
    {
        app_usbd_enable();
        app_usbd_start();
        m_usb_connected = true;
    }
}

static bool usb_connection_handle(bool last_usb_conn_status)
{
    if (last_usb_conn_status != m_usb_connected)
    {
        last_usb_conn_status = m_usb_connected;
        m_usb_connected ? app_usbd_start() : app_usbd_disable();
    }

    return last_usb_conn_status;
}
#endif

static void nrf_cli_cmd_print(nrf_cli_t const * p_cli, size_t argc, char **argv)
{
    if (argc != 2)
    {
        nrf_cli_fprintf(p_cli,
                        NRF_CLI_ERROR,
                        "print: bad param count\r\n");
        return;
    }

    nrf_cli_fprintf(p_cli, NRF_CLI_NORMAL, "%s\r\n", argv[1]);
}

static void nrf_cli_cmd_pyt(nrf_cli_t const * p_cli, size_t argc, char **argv)
{
    nrf_cli_fprintf(p_cli, NRF_CLI_ERROR, "Nice joke;)\r\n");
}

/**
 * @brief Command set array
 * */
static const nrf_cli_cmd_t m_cmd_set[] = {
        NRF_CLI_BUILTIN_CMD_SET,
        NRF_CLI_CMD("print", "command history", nrf_cli_cmd_print),
        NRF_CLI_CMD("python", "python interpreter", nrf_cli_cmd_pyt),
};


/**
 * @brief Command line interface instance
 * */
#ifdef BOARD_PCA10056
NRF_CLI_DEF(m_cli_cdc_acm, "nrf_cli:~$ ", nrf_cli_cdc_acm_transport, m_cmd_set);
#endif
NRF_CLI_DEF(m_cli_uart,    "nrf_cli:~$ ", nrf_cli_uart_transport, m_cmd_set);
NRF_CLI_DEF(m_cli_rtt,     "nrf_cli:~$ ", nrf_cli_rtt_transport, m_cmd_set);

static void cli_start(void)
{
    ret_code_t ret;
#ifdef BOARD_PCA10056
    ret = nrf_cli_start(&m_cli_cdc_acm);
    APP_ERROR_CHECK(ret);
#endif
    ret = nrf_cli_start(&m_cli_uart);
    APP_ERROR_CHECK(ret);
    ret = nrf_cli_start(&m_cli_rtt);
    APP_ERROR_CHECK(ret);
}

int main(void)
{
    ret_code_t ret;

    ret = nrf_drv_clock_init();
    APP_ERROR_CHECK(ret);
    ret = nrf_drv_power_init(NULL);
    APP_ERROR_CHECK(ret);

    bsp_board_leds_init();
    bsp_board_buttons_init();

#ifdef BOARD_PCA10056
    ret = nrf_cli_init(&m_cli_cdc_acm);
    APP_ERROR_CHECK(ret);
#endif
    ret = nrf_cli_init(&m_cli_uart);
    APP_ERROR_CHECK(ret);
    ret = nrf_cli_init(&m_cli_rtt);
    APP_ERROR_CHECK(ret);

#ifdef BOARD_PCA10056
    ret = app_usbd_init(NULL);
    APP_ERROR_CHECK(ret);

    app_usbd_class_inst_t const * class_cdc_acm =
            app_usbd_cdc_acm_class_inst_get(&nrf_cli_cdc_acm);
    ret = app_usbd_class_append(class_cdc_acm);
    APP_ERROR_CHECK(ret);

    bool last_usb_conn_status = false;
    bool last_port_status = false;
    usb_start();
#endif
    cli_start();
    while (true)
    {
#ifdef BOARD_PCA10056
        last_usb_conn_status = usb_connection_handle(last_usb_conn_status);

        if (last_port_status != nrf_cli_cdc_acm_port_is_open())
        {
            last_port_status = nrf_cli_cdc_acm_port_is_open();
        }
        nrf_cli_process(&m_cli_cdc_acm);
#endif
        nrf_cli_process(&m_cli_uart);
        nrf_cli_process(&m_cli_rtt);
    }
}

/** @} */
