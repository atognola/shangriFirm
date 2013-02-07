/*
 * simComAtCommands.h
 *
 * Created: 05/02/2013 01:22:30 p.m.
 *  Author: Ale
 */ 


#ifndef SIMCOMATCOMMANDS_H_
#define SIMCOMATCOMMANDS_H_

#define	MAX_COMMAND_LENGTH		15

/* Hayes AT command header */
#define AT						"AT"
#define ATplus					"AT+"
#define STRING_TERM				"\r\n"

/* Possible set commands */
#define SET_BAUDARATE_COMM		"IPR"									//"ATE+IPR=[baudrate]<CR><LF>"
#define SET_NO_ECHO_COMM		"ATE0"									//"ATE0<CR><LF>"
#define SET_APN_COMM			"CSTT"									//"AT+CSTT="[apn]","[user]","[pass]"<CR><LF>"
#define DIAL_UP_GPRS_COMM		"CIICR"									//"AT+CIICR"
#define DIAL_DOWN_GPRS_COMM		"CIPSHUT"								//"AT+CIPSHUT<CR><LF>"
//#define SET_GPRS_ATTACH_COMM	"CGATT"									//no recuerdo
#define ENABLE_DATA_HEADER_COMM	"CIPHEAD=1"								//"AT+CIPHEAD<CR><LF>"
#define OPEN_SOCKET_COMM		"CIPSTART"								//"AT+CIPSTART="[TCP/UDP]","[url]","[port]"<CR><LF>"
#define SEND_DATA_COMM			"CIPSEND"								//"AT+CIPSEND=[number]<CR><LF>"
#define CLOSE_SOCKET_COMM		"CIPCLOSE"								//"AT+CIPCLOSE<CR><LF>"
#define HANG_UP_COMM			"ATH"									//"ATH<CR><LF>"
#define DELETE_SMS_COMM			"AT+CMGD"								//"AT+CMGD=[number]<CR><LF>"

/* Possible query commands */
#define QUERY_STATUS			"CIPSTATUS"
#define QUERY_SIGNAL			"CSQ"
#define QUERY_NETW_REG			"CREG?"
#define QUERY_PIN_PRESENCE		"CPIN?"
#define QUERY_GPRS_ATTACH		"CGATT?"
#define QUERY_IP_ADDRESS		"CIFSR"

/* Possible types of connection */
#define TCP						"TCP"
#define UDP						"UDP"

/* Possible responses */
#define OK_MSG					"OK"									//"<CR><LF>OK<CR><LF>"
#define ERROR_MSG				"ERROR"									//"<CR><LF>ERROR<CR><LF>"
#define RDY_MSG					"RDY"									//"<CR><LF>RDY<CR><LF>"
#define PIN_READY_MSG			"+CPIN: READY"							//"<CR><LF>+CPIN: READY<CR><LF>"
#define NETW_REG_OK_MSG			"+CREG: 0,1"							//"<CR><LF>+CREG: 0,1<CR><LF><CR><LF>OK<CR><LF>"
#define GPRS_ATTACH_OK_MSG		"+CGATT: 1"								//"<CR><LF>+CGATT: 1<CR><LF><CR><LF>OK<CR><LF>"
#define LOST_GPRS_MSG			"+PDP DEACT"							//"<CR><LF>+PDP: DEACT<CR><LF>"
//#define IP_NUMBER_MSG			"[number]"								//"<CR><LF>10.181.5.233<CR><LF>"
#define CONNECT_OK_MSG			"CONNECT OK"							//"<CR><LF>CONNECT OK<CR><LF>"
#define CONNECT_FAIL_MSG		"CONNECT FAIL"							//"<CR><LF>CONNECT FAIL<CR><LF>"
#define SHUT_OK_MSG				"SHUT OK"								//"<CR><LF>SHUT OK<CR><LF>"
#define INCOMING_CALL_MSG		"RING"									//"<CR><LF>RING<CR><LF>"
#define INCOMING_SMS_MSG		"+CMTI: "								//"<CR><LF>+CMTI: "SM",[number]<CR><LF>"		[number]=4
#define INCOMING_DATA_MSG		"+IPD,"									//"<CR><LF>+IPD,[number]:[DATA]"
#define POWER_OFF_MSG			"NORMAL POWER DOWN"						//"<CR><LF>NORMAL POWER DOWN<CR><LF>"
#define CALL_ENDED_MSG			"NO CARRIER"							//"<CR><LF>NO CARRIER<CR><LF>"
#define CLOSED_SOCKET_MSG		"CLOSED"								//"<CR><LF>CLOSED<CR><LF>"
#define ALREADY_CONNECT_MSG		"ALREADY CONNECT"						//"<CR><LF>ALREADY CONNECT<CR><LF>"

/* Possible states */
#define STATE_HEADER			"STATE: "								//"<CR><LF>STATE: "
#define IP_INITIAL_STATE		"IP INITIAL"							//"IP INITIAL<CR><LF>"   "<CR><LF>STATE: IP INITIAL<CR><LF>"
#define TCP_CLOSED_STATE		"TCP CLOSED"							//"TCP CLOSED<CR><LF>"


//TurnOnMessage: #CR#LFRDY#CR#LF#CR#LF+CFUN: 1#CR#LF#CR#LF+CPIN: READY#CR#LF#CR#LFCall Ready#CR#LF
//Only received when fixed baudrate is chosen!

#endif /* SIMCOMATCOMMANDS_H_ */