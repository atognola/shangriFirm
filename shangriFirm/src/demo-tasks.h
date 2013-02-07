/**
 *
 * \file
 *
 * \brief Generic FreeRTOS peripheral control functions
 *
 *
 * Copyright (c) 2012 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

#ifndef DEMO_TASKS_H
#define DEMO_TASKS_H

#include "conf_example.h"
#include "freertos_usart_serial.h"
#include "usart.h"
#include "uart.h"

/* The size of the buffer provided to the USART driver for storage of received
 * bytes. */
#define RX_BUFFER_SIZE_BYTES    (50)

#define COMMAND_HEADER		'#'
#define SIM_PWR_SEQUENCE	1000/portTICK_RATE_MS
#define SIM_RES_SEQUENCE	10/portTICK_RATE_MS
#define MAX_PWR_COMMANDS	2
#define COMMAND_SIZE		sizeof(uint32_t)
#define ON_COMMAND			11
#define OFF_COMMAND			12
#define RES_COMMAND			13

void create_usart_uart_tunnel_tasks(Usart *pxUsart,uint16_t usart_stack_depth_words,
									uint16_t uart_stack_depth_words,unsigned portBASE_TYPE task_priority);
portBASE_TYPE are_tunnel_tasks_still_running(void);

typedef	struct{
	xQueueHandle sim_pwr_commands_queue;
	freertos_usart_if myUsart;
	} tunnel_params_t;

#if (defined confINCLUDE_SPI_FLASH_TASK)
#include "spi.h"
void create_spi_flash_test_task(Spi *spi_base, uint16_t stack_depth_words,
		unsigned portBASE_TYPE task_priority,
		portBASE_TYPE set_asynchronous_api);
portBASE_TYPE did_spi_flash_test_pass(void);

#endif

#if (defined confINCLUDE_TWI_EEPROM_TASK)
#include "freertos_twi_master.h"
void create_twi_eeprom_test_task(Twi *twi_base, uint16_t stack_depth_words,
		unsigned portBASE_TYPE task_priority,
		portBASE_TYPE set_asynchronous_api);
portBASE_TYPE did_twi_eeprom_test_pass(void);

#endif

#endif /* DEMO_TASKS_H */
