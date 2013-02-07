/*
 * sim900.c
 *
 * Created: 05/02/2013 01:22:54 p.m.
 *  Author: Ale
 */ 

#include "sim900.h"
#include "simComAtCommands.h"

bool sendAtCommand(sim_command_t *command,sim900_t *sim_instance){
	
	printf(ATplus);
	
	// 	char *buff;
	// 	int i=0;
	// 	/* Data cannot be sent from Flash, so copy the string to RAM. */
	// 	//strcpy((char *) local_buffer,(const char *) echo_strings[string_index]);
	// 	buff=echo_strings[2];
	// 	
	// 	for (i=0;*(buff+i)!=0;i++)
	// 	{
	// 		putchar(*(buff+i));
	// 	}
	// 	return *(buff+i);
	
	return false;
}