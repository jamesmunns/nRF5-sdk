/**
 * Copyright (c) 2015 - 2017, Nordic Semiconductor ASA
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
/**
 * @file
 * @brief File with example code presenting usage of drivers for TWIS slave and TWI in master mode
 *
 * @sa twi_master_with_twis_slave_example
 */

/**
 * @defgroup twi_master_with_twis_slave_example Example code presenting usage of drivers for TWIS slave and TWI in master mode
 *
 * This code presents the usage of two drivers:
 * - @ref nrf_twi_drv (in synchronous mode)
 * - @ref nrf_twis_drv (in asynchronous mode)
 *
 * On the slave, EEPROM memory is simulated.
 * For simulated EEPROM, the AT24C01C device produced by ATMEL was selected.
 * This device has 128 bytes of memory and it is simulated using internal RAM.
 * This RAM area is accessed only by simulated EEPROM so the rest of the application can access it
 * only using TWI commands via hardware configured pins.
 *
 * The selected memory chip uses a 7-bit constant address. Word to access is selected during
 * a write operation: first byte sent is used as the current address pointer.
 *
 * A maximum of an 8-byte page can be written in a single access.
 * The whole memory can be read in a single access.
 *
 * Differences between real chip and simulated one:
 * 1. Simulated chip has practically 0 ms write time.
 *    This example does not poll the memory for readiness.
 * 2. During sequential read, when memory end is reached, zeroes are sent.
 *    There is no support for roll-over.
 * 3. It is possible to write a maximum of 8 bytes in a single sequential write.
 *    However, in simulated EEPROM the whole address pointer is incremented.
 *    In a real memory chip, only the 3 lowest bits chang during writing.
 *    In a real device, writing would roll-over in memory page.
 *
 * On the master side, we communicate with that memory and allow write and read.
 * Simple commands via UART can be used to check the memory.
 *
 * Pins to short:
 * - @ref TWI_SCL_M - @ref EEPROM_SIM_SCL_S
 * - @ref TWI_SDA_M - @ref EEPROM_SIM_SDA_S
 *
 * Supported commands are always listed in the welcome message.
 * @{
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <ctype.h>
#include <string.h>
#include "config.h"
#include "eeprom_simulator.h"
#include "nrf_drv_twi.h"
#include "nrf_gpio.h"
#include "app_error.h"
#include "nrf.h"
#include "bsp.h"
#include "app_util_platform.h"
#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

/**
 * @brief Repeated part of the help string.
 *
 * This string contains list of supported commands together with description.
 * It is used in a welcome message and in a help message if an unsupported commmand is detected.
 */
static const char m_cmd_help_str1[] =
        "   p - Print the EEPROM contents in a form: address, 8 bytes of code, ASCII form.\r\n";
static const char m_cmd_help_str2[] =
        "   w - Write a string starting from address 0.\r\n";
static const char m_cmd_help_str3[] =
        "   c - Clear the memory (write 0xff)\r\n";
static const char m_cmd_help_str4[] =
        "   x - Get transmission error byte.\r\n";

/**
 * @brief TWI master instance.
 *
 * Instance of TWI master driver that will be used for communication with simulated
 * EEPROM memory.
 */
static const nrf_drv_twi_t m_twi_master = NRF_DRV_TWI_INSTANCE(MASTER_TWI_INST);


/**
 * @brief Write data to simulated EEPROM.
 *
 * Function uses the TWI interface to write data into EEPROM memory.
 *
 * @param     addr  Start address to write.
 * @param[in] pdata Pointer to data to send.
 * @param     size  Byte count of data to send.
 * @attention       Maximum number of bytes that may be written is @ref EEPROM_SIM_SEQ_WRITE_MAX.
 *                  In sequential write, all data must be in the same page
 *                  (higher address bits do not change).
 *
 * @return NRF_SUCCESS or reason of error.
 *
 * @attention If you wish to communicate with real EEPROM memory chip, check its readiness
 * after writing the data.
 */
static ret_code_t eeprom_write(uint16_t addr, uint8_t const * pdata, size_t size)
{
    ret_code_t ret;
    /* Memory device supports only a limited number of bytes written in sequence */
    if (size > (EEPROM_SIM_SEQ_WRITE_MAX_BYTES))
    {
        return NRF_ERROR_INVALID_LENGTH;
    }
    /* All written data must be in the same page */
    if ((addr / (EEPROM_SIM_SEQ_WRITE_MAX_BYTES)) != ((addr + size - 1) / (EEPROM_SIM_SEQ_WRITE_MAX_BYTES)))
    {
        return NRF_ERROR_INVALID_ADDR;
    }
    do
    {
        uint8_t buffer[EEPROM_SIM_ADDRESS_LEN_BYTES + EEPROM_SIM_SEQ_WRITE_MAX_BYTES]; /* Addr + data */

        memcpy(buffer, &addr, EEPROM_SIM_ADDRESS_LEN_BYTES);
        memcpy(buffer + EEPROM_SIM_ADDRESS_LEN_BYTES, pdata, size);
        ret = nrf_drv_twi_tx(&m_twi_master, EEPROM_SIM_ADDR, buffer, size + EEPROM_SIM_ADDRESS_LEN_BYTES, false);
    }while (0);
    return ret;
}


/**
 * @brief Read data from simulated EEPROM.
 *
 * Function uses the TWI interface to read data from EEPROM memory.
 *
 * @param     addr  Start address to read.
 * @param[in] pdata Pointer to the buffer to fill with data.
 * @param     size  Byte count of data to read.
 *
 * @return NRF_SUCCESS or reason of error.
 */
static ret_code_t eeprom_read(uint16_t addr, uint8_t * pdata, size_t size)
{
    ret_code_t ret;
    if (size > (EEPROM_SIM_SIZE))
    {
        return NRF_ERROR_INVALID_LENGTH;
    }
    do
    {
       uint16_t addr16 = addr;
       ret = nrf_drv_twi_tx(&m_twi_master, EEPROM_SIM_ADDR, (uint8_t *)&addr16, EEPROM_SIM_ADDRESS_LEN_BYTES, true);
       if (NRF_SUCCESS != ret)
       {
           break;
       }
       ret = nrf_drv_twi_rx(&m_twi_master, EEPROM_SIM_ADDR, pdata, size);
    }while (0);
    return ret;
}

/**
 * @brief Pretty-print the EEPROM content.
 *
 * Respond on memory print command.
 */
static void do_print_data(void)
{
    uint16_t addr;
    uint8_t buff[IN_LINE_PRINT_CNT];
    for (addr=0; addr<(EEPROM_SIM_SIZE); addr+=(IN_LINE_PRINT_CNT))
    {
        ret_code_t err_code;
        err_code = eeprom_read(addr, buff, IN_LINE_PRINT_CNT);
        if (NRF_SUCCESS != err_code)
        {
            NRF_LOG_WARNING("Communication error\r\n");
            return;
        }

        NRF_LOG_RAW_INFO("%.2x: ", addr);
        NRF_LOG_RAW_HEXDUMP_INFO(buff, IN_LINE_PRINT_CNT);

    }
    NRF_LOG_FLUSH();
}


/**
 * @brief Safely get a string from stdin.
 *
 * Function reads character by character into the given buffer.
 * Maximum @em nmax number of characters are read.
 *
 * Function ignores all nonprintable characters.
 * String may be finished by CR or NL.
 *
 * @attention
 * Remember that after characters are read, zero will be added to mark the string end.
 * Thee given buffer should be no smaller than @em nmax + 1.
 *
 * @param[out] str  Buffer for the string.
 * @param      nmax Maximum number of characters to be read.
 */
static void safe_gets(char * str, size_t nmax)
{
    int c;
    char cstr[2] = {0, 0};
    while (1)
    {
        c = NRF_LOG_GETCHAR();
        if (isprint(c))
        {
            *str++ = (char)c;
            cstr[0] = c;
            NRF_LOG_RAW_INFO("%s", nrf_log_push(cstr));
            NRF_LOG_FLUSH();
            if (0 == --nmax)
                break;
        }
        else if ('\n' == c || '\r' == c)
        {
            break;
        }
    }
    *str = '\0';
    cstr[0] = '\n';
    NRF_LOG_RAW_INFO("%s", nrf_log_push(cstr));
    NRF_LOG_FLUSH();
}

/**
 * @brief Check the string length but no more than the given number of characters.
 *
 * Function iterates through the string searching for the zero character.
 * Even when it is not found it stops iterating after maximum of @em nmax characters.
 *
 * @param[in] str  String to check.
 * @param     nmax Maximum number of characters to check.
 *
 * @return String length or @em nmax if no zero character is found.
 */
static size_t safe_strlen(char const * str, size_t nmax)
{
    size_t n=0;
    while ('\0' != *str++)
    {
        ++n;
        if (0 == --nmax)
            break;
    }
    return n;
}

/**
 * @brief Function that performs the command of writing a string to EEPROM.
 *
 * Function gets user input and writes the given string to EEPROM starting from address 0.
 * It is accessing EEPROM using maximum allowed number of bytes in sequence.
 */
static void do_string_write(void)
{
    char str[(EEPROM_SIM_SIZE) + 1];
    uint16_t addr = 0;

    NRF_LOG_RAW_INFO("Waiting for string to write:\r\n");
    NRF_LOG_FLUSH();
    safe_gets(str, sizeof(str) - 1);
    while (1)
    {
        ret_code_t err_code;
        size_t to_write = safe_strlen(str + addr, EEPROM_SIM_SEQ_WRITE_MAX_BYTES);
        if (0 == to_write)
            break;
        err_code = eeprom_write(addr, (uint8_t const *)str + addr, to_write);
        if (NRF_SUCCESS != err_code)
        {
            NRF_LOG_WARNING("Communication error\r\n");
            return;
        }
        addr += to_write;
    }

    NRF_LOG_RAW_INFO("OK\r\n");
}

/**
 * @brief Function that performs simulated EEPROM clearing.
 *
 * Function fills the EEPROM with 0xFF value.
 * It is accessing the EEPROM writing only one byte at once.
 */
static void do_clear_eeprom(void)
{
    uint8_t clear_val = 0xff;
    size_t addr;
    for (addr=0; addr<(EEPROM_SIM_SIZE); ++addr)
    {
        ret_code_t err_code;
        err_code = eeprom_write(addr, &clear_val, 1);
        if (NRF_SUCCESS != err_code)
        {
            NRF_LOG_WARNING("Communication error\r\n");
            return;
        }
    }
    NRF_LOG_RAW_INFO("Memory erased\r\n");
}

/**
 * @brief Initialize the master TWI.
 *
 * Function used to initialize the master TWI interface that would communicate with simulated EEPROM.
 *
 * @return NRF_SUCCESS or the reason of failure.
 */
static ret_code_t twi_master_init(void)
{
    ret_code_t ret;
    const nrf_drv_twi_config_t config =
    {
       .scl                = TWI_SCL_M,
       .sda                = TWI_SDA_M,
       .frequency          = NRF_TWI_FREQ_400K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    ret = nrf_drv_twi_init(&m_twi_master, &config, NULL, NULL);

    if (NRF_SUCCESS == ret)
    {
        nrf_drv_twi_enable(&m_twi_master);
    }

    return ret;
}

static void help_print(void)
{
    NRF_LOG_RAW_INFO("%s", (uint32_t)m_cmd_help_str1);
    NRF_LOG_RAW_INFO("%s", (uint32_t)m_cmd_help_str2);
    NRF_LOG_RAW_INFO("%s", (uint32_t)m_cmd_help_str3);
    NRF_LOG_RAW_INFO("%s", (uint32_t)m_cmd_help_str4);
}

/**
 *  The beginning of the journey
 */
int main(void)
{
    ret_code_t err_code;
    /* Initialization of UART */
    bsp_board_leds_init();

    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));

    /* Initializing simulated EEPROM */
    err_code = eeprom_simulator_init();
    APP_ERROR_CHECK(err_code);

    /* Initializing TWI master interface for EEPROM */
    err_code = twi_master_init();
    APP_ERROR_CHECK(err_code);


    /* Welcome message */
    NRF_LOG_RAW_INFO(
            "This is TWIS and TWI usage example\r\n"
            "You can access simulated EEPROM memory using following commands:\r\n"
    );
    help_print();

    NRF_LOG_FLUSH();

    /* Main loop */
    while (1)
    {
        uint8_t c = NRF_LOG_GETCHAR();

        switch ((char)c)
        {
        case '\n':
        case '\r':
            break;
        case 'p':
            do_print_data();
            break;
        case 'w':
            do_string_write();
            break;
        case 'c':
            do_clear_eeprom();
            break;
        case 'x':
            {
                uint32_t error = eeprom_simulator_error_get_and_clear();
                NRF_LOG_WARNING("Error word: %x\r\n", (unsigned int)error);
            }
            break;
        default:
            NRF_LOG_RAW_INFO("You selected %c\r\n", (char)c);
            NRF_LOG_RAW_INFO("Unknown command, try one of the following:\r\n");
            help_print();
            break;
        }
        if (eeprom_simulator_error_check())
        {
            NRF_LOG_RAW_INFO(
                    "WARNING: EEPROM transmission error detected.\r\n"
                    "Use 'x' command to read error word.\r\n"
            );
        }
        NRF_LOG_FLUSH();
    }
}

/** @} */ /* End of group twi_master_with_twis_slave_example */
