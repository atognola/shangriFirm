/*
 * sim900.c
 *
 * Created: 05/02/2013 01:22:54 p.m.
 *  Author: Ale
 */ 

/* Standard includes. */
#include <stdio.h>
#include <string.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "FreeRTOSConfig.h"

/* Atmel library includes. */
#include "freertos_usart_serial.h"

/* Demo includes. */
#include <gpio.h>
#include "sam3n_ek.h"

/* Debug includes. */
#include "sim900.h"
#include "simComAtCommands.h"

//////////////////////////////////////////////////////////////////////////////////////////////
//Memory reservations																		//
//////////////////////////////////////////////////////////////////////////////////////////////
uint8_t command_buff[MAX_COMMAND_LENGTH];
static uint8_t usart_receive_buffer[RX_BUFFER_SIZE] = {0};
//static uint8_t uart_receive_buffer[RX_BUFFER_SIZE] = {0};
static xQueueHandle sim_pwr_commands_queue;

//////////////////////////////////////////////////////////////////////////////////////////////
//Sends an AT command																		//
//////////////////////////////////////////////////////////////////////////////////////////////
bool sendAtCommand(sim_command_t *command,sim900_t *sim_instance){
	
	status_code_t			returned_status;
	const portTickType		time_out_definition = (100UL / portTICK_RATE_MS);
	freertos_usart_if		usart;
	
	usart=sim_instance->usart_port;
	
	strcpy(command_buff,command->command);											//Must send a string stored in ram, not flash!
	strcpy(command_buff+strlen(command_buff),STRING_TERM);
	
	returned_status = freertos_usart_write_packet(usart,
					command_buff, strlen(command_buff),time_out_definition);
	configASSERT(returned_status == STATUS_OK);
	
	return (returned_status==STATUS_OK)?pdTRUE:pdFALSE;
}

//////////////////////////////////////////////////////////////////////////////////////////////
//Waits for a valid AT command response														//
//////////////////////////////////////////////////////////////////////////////////////////////
static bool waitAtResponse(sim_command_t *command,sim900_t *sim_instance){
	return pdTRUE;
}

//////////////////////////////////////////////////////////////////////////////////////////////
//Starts up SIM900 module																	//
//////////////////////////////////////////////////////////////////////////////////////////////
bool bootSim(Usart *usart_base,sim900_t *sim_instance){
	create_sim900_driver_tasks(usart_base,								//BOARD_USART points to the USART peripheral
							mainUSART_TASK_STACK_SIZE,
							mainUART_TUNNEL_TASK_STACK_SIZE,
							mainUSART_TASK_PRIORITY);
// 	freertos_usart_if myUsart=(sim_instance->usart_port);
// 	/* Initialise the USART interface. */
// 	freertos_peripheral_options_t usart_driver_options = {
// 		usart_receive_buffer,							/* The buffer used internally by the USART driver to store incoming characters. */
// 		RX_BUFFER_SIZE,									/* The size of the buffer provided to the USART driver to store incoming characters. */
// 		configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY,	/* The priority used by the USART interrupts. */
// 		USART_RS232,									/* Configure the USART for RS232 operation. */
// 		(USE_TX_ACCESS_MUTEX | USE_RX_ACCESS_MUTEX)
// 	};
// 	const sam_usart_opt_t usart_settings = {
// 		USART_BAUD_RATE,
// 		US_MR_CHRL_8_BIT,
// 		US_MR_PAR_NO,
// 		US_MR_NBSTOP_1_BIT,
// 		US_MR_CHMODE_NORMAL,
// 		0 /* Only used in IrDA mode. */
// 	}; ///*_RB_ TODO This is not SAM specific, not a good thing. */
// 	myUsart = freertos_usart_serial_init(usart_base,&usart_settings, &usart_driver_options);
// 	configASSERT(myUsart);
}

//////////////////////////////////////////////////////////////////////////////////////////////
//Sim900 command parser/interpreter															//
//////////////////////////////////////////////////////////////////////////////////////////////
static void usart_rx_task(void *pvParameters)
{
	freertos_usart_if		usart_port;
	static uint8_t			rx_buffer[RX_BUFFER_SIZE],i;
	uint32_t				received;
	const portTickType		time_out_definition = (100UL / portTICK_RATE_MS),
							short_delay = (10UL / portTICK_RATE_MS);

	/* The (already open) USART port is passed in as the task parameter. */
	usart_port = (freertos_usart_if)pvParameters;
	
	/* Clear memory buffer */
	memset(rx_buffer, 0x00, sizeof(rx_buffer));
	
	for (;;) {		
		//Recibe en la USART y forwardea por la UART
		received = freertos_usart_serial_read_packet(usart_port, rx_buffer,RX_BUFFER_SIZE,short_delay);
		if(received!=0)
		{
			/* Start send. */
			for (i=0;(i<received)&&(i<RX_BUFFER_SIZE);i++)
			{
				putchar(rx_buffer[i]);
			}
		} else {
			vTaskDelay(short_delay);
		}

		/* Increment a loop counter as an indication that this task is still
		actually receiving strings. */
		usart_rx_task_loops++;
	}
}

//////////////////////////////////////////////////////////////////////////////////////////////
//For debug purpouses, to let computer issue AT commands									//
//////////////////////////////////////////////////////////////////////////////////////////////
static void uart_tunnel_rx_task(void *pvParameters)
{
	freertos_usart_if		usart_port;
	static uint8_t			rx_buffer[RX_BUFFER_SIZE];
	static uint8_t			rx_char;
	uint32_t				pwr_command,i=0;
	const portTickType		time_out_definition = (100UL / portTICK_RATE_MS),
							short_delay = (10UL / portTICK_RATE_MS);
	status_code_t			returned_status;
	
	//Para debug nomás!
	sim900_t				sim_instance;
	sim_command_t			a_command;
	
	/* The (already open) USART port is passed in as the task parameter. */
	usart_port = (freertos_usart_if *) pvParameters;
	sim_instance.usart_port=usart_port;
	/* Create the semaphore to be used to get notified of end of
	transmissions. */
// 	vSemaphoreCreateBinary(notification_semaphore);
// 	configASSERT(notification_semaphore);
	/* Clear memory buffer */
	memset(rx_buffer, 0x00, sizeof(rx_buffer));
	
	/* Start with the semaphore in the expected state - no data has been sent
	yet.  A block time of zero is used as the semaphore is guaranteed to be
	there as it has only just been created. */
	//xSemaphoreTake(notification_semaphore, 0);
	
	for (;;) {
		//Recibe en la UART y forwardea por la USART
		if((rx_char=getchar())!=0)
		{
			rx_buffer[i]=rx_char;
			i=i+1;
			if(rx_char==10)										//Newline character received, therefore, must send!
			{
				if(rx_buffer[0]==TUNNEL_COMMAND_HEADER) {
					//Terminal command received
					if((rx_buffer[1]=='O')&&(rx_buffer[2]=='N')) {
						pwr_command = ON_COMMAND;									//Pasar comando en queue
						xQueueSend(sim_pwr_commands_queue,&pwr_command,time_out_definition);
					} else if((rx_buffer[1]=='R')&&(rx_buffer[2]=='E')&&(rx_buffer[3]=='S')) {
						pwr_command = RES_COMMAND;									//Pasar comando en queue
						xQueueSend(sim_pwr_commands_queue,&pwr_command,time_out_definition);
					} else if((rx_buffer[1]=='A')&&(rx_buffer[2]=='T')) {
						a_command.command=AT;
						sendAtCommand(&a_command,&sim_instance);
					}else {
						putchar('N'); putchar('o'); putchar('C'); putchar('m'); putchar('d'); putchar(13); putchar(10);	//ResSim message
					}
					i=0;
				} else {
					//Text received
					returned_status = freertos_usart_write_packet(usart_port,						/* Start send. */
					rx_buffer, i,time_out_definition);
					configASSERT(returned_status == STATUS_OK);
					i=0;
				}				
			}
		}
		
		/* Increment a loop counter as an indication that this task is still
		actually receiving strings. */
		uart_rx_task_loops++;
	}
}

//////////////////////////////////////////////////////////////////////////////////////////////
//Handles sim900 hardware pins																//
//////////////////////////////////////////////////////////////////////////////////////////////
void turn_on_sim_task(void *pvParameters)
{
	uint32_t command;
	const portTickType		time_out_definition = (100UL / portTICK_RATE_MS),
	short_delay = (10UL / portTICK_RATE_MS);
	
	for(;;) {
		/*if(xQueueReceive(sim_pwr_commands_queue,&command,short_delay)) {*/
		if(xQueueReceive(sim_pwr_commands_queue,&command,portMAX_DELAY)) {
			if(command==ON_COMMAND) {
				putchar('O'); putchar('n'); putchar('S'); putchar('i'); putchar('m'); putchar(13); putchar(10);	//OnSim message
				#if SIM_PWR_IDLE_LEVEL==0
				gpio_set_pin_high(SIM_PWR_GPIO);							//set pin high
				#else
				gpio_set_pin_low(SIM_PWR_GPIO);								//set pin low
				#error La ShangriBoard invierte la logica
				#endif
				vTaskDelay(SIM_PWR_SEQUENCE);									//corresponding bit bang time
				#if SIM_PWR_IDLE_LEVEL==0
				gpio_set_pin_low(SIM_PWR_GPIO);								//set pin low
				#else
				gpio_set_pin_high(SIM_PWR_GPIO);							//set pin high
				#error La ShangriBoard invierte la logica
				#endif
			} else if (command==RES_COMMAND)
			{
				putchar('R'); putchar('e'); putchar('s'); putchar('S'); putchar('i'); putchar('m'); putchar(13); putchar(10);	//ResSim message
				#if SIM_NRST_IDLE_LEVEL==0
				gpio_set_pin_high(SIM_NRST_GPIO);							//set pin high
				#else
				gpio_set_pin_low(SIM_NRST_GPIO);							//set pin low
				#error La ShangriBoard invierte la logica
				#endif
				vTaskDelay(SIM_RES_SEQUENCE);									//corresponding bit bang time
				#if SIM_NRST_IDLE_LEVEL==0
				gpio_set_pin_low(SIM_NRST_GPIO);							//set pin low
				#else
				gpio_set_pin_high(SIM_NRST_GPIO);							//set pin high
				#error La ShangriBoard invierte la logica
				#endif
			}
		} /*else {
			vTaskDelay(time_out_definition);
		}*/
	}
}

//////////////////////////////////////////////////////////////////////////////////////////////
//Checks whether usart driver functions are still running									//
//////////////////////////////////////////////////////////////////////////////////////////////
portBASE_TYPE are_sim900_driver_tasks_still_running(void)
{
	portBASE_TYPE return_value = pdPASS;
	static uint32_t usart_rx_last_loop_count = 0;
	static uint32_t uart_rx_last_loop_count = 0;

	/* Ensure the count of Rx loops is still incrementing. */
	if (usart_rx_last_loop_count == usart_rx_task_loops || uart_rx_last_loop_count == uart_rx_task_loops) {
		/* The Rx task has somehow stalled, set the error LED. */
		return_value = pdFAIL;
	}

	usart_rx_last_loop_count = usart_rx_task_loops;
	uart_rx_last_loop_count = uart_rx_task_loops;

	return return_value;
}

//////////////////////////////////////////////////////////////////////////////////////////////
//Starts up freertos USART driver functions													//
//////////////////////////////////////////////////////////////////////////////////////////////
freertos_usart_if create_sim900_driver_tasks(Usart *usart_base,
		uint16_t usart_stack_depth_words,
		uint16_t uart_stack_depth_words,
		unsigned portBASE_TYPE task_priority)
{
	freertos_usart_if myUsart;
	/* Initialise the USART interface. */
	freertos_peripheral_options_t usart_driver_options = {
		usart_receive_buffer,							/* The buffer used internally by the USART driver to store incoming characters. */
		RX_BUFFER_SIZE,									/* The size of the buffer provided to the USART driver to store incoming characters. */
		configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY,	/* The priority used by the USART interrupts. */
		USART_RS232,									/* Configure the USART for RS232 operation. */
		(USE_TX_ACCESS_MUTEX | USE_RX_ACCESS_MUTEX)
	};
	const sam_usart_opt_t usart_settings = {
		USART_BAUD_RATE,
		US_MR_CHRL_8_BIT,
		US_MR_PAR_NO,
		US_MR_NBSTOP_1_BIT,
		US_MR_CHMODE_NORMAL,
		0 /* Only used in IrDA mode. */
	}; ///*_RB_ TODO This is not SAM specific, not a good thing. */
	myUsart = freertos_usart_serial_init(usart_base,&usart_settings, &usart_driver_options);
	configASSERT(myUsart);
	
	sim_pwr_commands_queue = xQueueCreate(MAX_PWR_COMMANDS,COMMAND_SIZE);
	
	/* Success: Create the two tasks as described above. */
	/*xTaskCreate(uart_tunnel_tx_task, (const signed char *const) "UartTx",
	uart_stack_depth_words, (void *) myUart,
	task_priority, NULL);*/
	xTaskCreate(uart_tunnel_rx_task,					/* One of the tasks that implement the tunnel. */
				(const signed char *const) "UartRx",	/* Text name assigned to the task.  This is just to assist debugging.  The kernel does not use this name itself. */
				uart_stack_depth_words,					/* The size of the stack allocated to the task. */
				(void *) myUsart,						/* The parameter is used to pass the already configured UART port into the task. */
				task_priority + 2,						/* The priority allocated to the task. */
				NULL);									/* Used to store the handle to the created task - in this case the handle is not required. */
	/*xTaskCreate(usart_tunnel_tx_task, (const signed char *const) "UsartTx",
	usart_stack_depth_words, (void *) freertos_usart,
	task_priority, NULL);*/
	xTaskCreate(usart_rx_task,							/* One of the tasks that implement the tunnel. */
				(const signed char *const) "UsartRx",	/* Text name assigned to the task.  This is just to assist debugging.  The kernel does not use this name itself. */
				usart_stack_depth_words,				/* The size of the stack allocated to the task. */
				(void *) myUsart,						/* The parameter is used to pass the already configured USART+UART ports into the task. */
				task_priority + 2,						/* The priority allocated to the task. */
				NULL);									/* Used to store the handle to the created task - in this case the handle is not required. */
	/* Task used to power on and power off Sim900 GPRS module */
	xTaskCreate(turn_on_sim_task,							/* One of the tasks that implement the tunnel. */
				(const signed char *const) "SimPowerOn",	/* Text name assigned to the task.  This is just to assist debugging.  The kernel does not use this name itself. */
				configMINIMAL_STACK_SIZE,					/* The size of the stack allocated to the task. */
				(void *) &sim_pwr_commands_queue,			/* The parameter is used to pass the already configured USART+UART ports into the task. */
				tskIDLE_PRIORITY+2,							/* The priority allocated to the task. */
				NULL);										/* Used to store the handle to the created task - in this case the handle is not required. */
	
	return myUsart;
}

//////////////////////////////////////////////////////////////////////////////////////////////
//Patch to avoid using regular printf														//
//////////////////////////////////////////////////////////////////////////////////////////////
void myPrintf(uint8_t *buff){
	uint32_t i=0;
	
	for (i=0;(i<strlen(buff))&&(*(buff+i)!=0);i++)
	{
		putchar(*(buff+i));
	}
}