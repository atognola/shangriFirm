/*
 * sim900.h
 *
 * Created: 05/02/2013 01:23:07 p.m.
 *  Author: Ale
 */ 


#ifndef SIM900_H_
#define SIM900_H_

#include "stdint.h"
#include "freertos_usart_serial.h"
#include "queue.h"
#include "semphr.h"

typedef struct {
	uint8_t				*command;							//Actual command issued
	uint8_t				*expected_response;					//Response expected upon success
	uint8_t				*failure_response;					//Response expected upon failure
	portTickType		time_of_issue;						//Time when the command was issued
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

bool bootSim(sim900_t *sim_instance);
bool bringUpGprsService(sim900_t *sim_instance);
bool connectTcpSocket(uint8_t *ipAddress,uint8_t *port, sim900_t *sim_instance);
static bool sendAtCommand(sim_command_t *command,sim900_t *sim_instance);
static bool waitAtResponse(sim_command_t *command,sim900_t *sim_instance);

#endif /* SIM900_H_ */