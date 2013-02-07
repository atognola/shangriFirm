/*
 * sim900.h
 *
 * Created: 05/02/2013 01:23:07 p.m.
 *  Author: Ale
 */ 


#ifndef SIM900_H_
#define SIM900_H_

#define SIM_900_DEBUG

#include "stdint.h"
#include "freertos_usart_serial.h"
#include "queue.h"
#include "semphr.h"
#include "sim900/simComAtCommands.h"

/* The size of the buffer provided to the USART driver for storage of received
 * bytes. */
#define RX_BUFFER_SIZE_BYTES					(50)

/* The baud rate to use. */
#define USART_BAUD_RATE							(115200)

/* The size of the buffer used to receive characters from the USART driver.
 * This equals the length of the longest string used in this file. */
#define RX_BUFFER_SIZE							(100)

/* The priorities at which various tasks will get created. */
#define mainUSART_TASK_PRIORITY					(TASK_NORMAL_PRIORITY)

/* The stack sizes allocated to the various tasks. */
#define mainUSART_TASK_STACK_SIZE				(configMINIMAL_STACK_SIZE*3)
#define mainUART_TUNNEL_TASK_STACK_SIZE			(configMINIMAL_STACK_SIZE*3)

/* Definitions used to manage sim900's power on/off. */
#define MAX_PWR_COMMANDS		2
#define SIM_PWR_SEQUENCE		1000/portTICK_RATE_MS
#define SIM_RES_SEQUENCE		10/portTICK_RATE_MS

/* For debug purpouses, commands have been implemented */
#define TUNNEL_COMMAND_HEADER	'#'
#define COMMAND_SIZE			sizeof(uint32_t)
#define ON_COMMAND				11
#define OFF_COMMAND				12
#define RES_COMMAND				13

typedef struct {
	uint8_t				*command;							//Actual command issued
	uint8_t				*expected_response;					//Response expected upon success
	uint8_t				*failure_response;					//Response expected upon failure
	portTickType		time_of_expiry;						//Time when the command will expire in case of no valid response from Sim
	uint8_t				must_reissue_upon_failure;			//Amount of times to reissue command until success 
} sim_command_t;

typedef	struct {
	uint8_t				*response;							//Actual response received
	portTickType		time;								//Time when the response was received
} sim_response_t;

typedef uint8_t* sim_state_t;								//Current sim state

typedef	struct {
	freertos_usart_if	usart_port;							//Freertos USART structure pointer
	sim_command_t		last_command_issued;				//Last issued command
	sim_state_t			actual_sim_state;					//Actual Sim state
	xQueueHandle		tcp_data;							//Pointer to the queue used to retrieve tcp data
	xSemaphoreHandle	sim_available;						//Semaphore used to access sim module
} sim900_t;

//Sim 900 driver functions
bool bootSim(Usart *usart_base,sim900_t *sim_instance);
bool bringUpGprsService(sim900_t *sim_instance);
bool connectTcpSocket(uint8_t *ipAddress,uint8_t *port, sim900_t *sim_instance);
bool sendAtCommand(sim_command_t *command,sim900_t *sim_instance);				//For debug only!
//static bool sendAtCommand(sim_command_t *command,sim900_t *sim_instance);
static bool waitAtResponse(sim_command_t *command,sim900_t *sim_instance);
//Task related ones
freertos_usart_if create_sim900_driver_tasks(Usart *pxUsart,uint16_t usart_stack_depth_words,
									uint16_t uart_stack_depth_words,unsigned portBASE_TYPE task_priority);
portBASE_TYPE are_sim900_driver_tasks_still_running(void);

/*
 * Tasks used to develop the USART drivers.  One task sends out a series of
 * strings, the other task expects to receive the same series of strings.  An
 * error is latched if any characters are missing.  A loopback connector is
 * required to ensure the transmitted characters are also received.
 */
static void usart_rx_task(void *pvParameters);
static void uart_tunnel_rx_task(void *pvParameters);
static void sim_pin_handler_task(void *pvParameters);

/* Counts the number of times the Rx task receives a string.  The count is used
to ensure the task is still executing. */
static uint32_t usart_rx_task_loops = 0UL;
static uint32_t uart_rx_task_loops = 0UL;

//Miscellaneous
void myPrintf(uint8_t *buff);

#endif /* SIM900_H_ */