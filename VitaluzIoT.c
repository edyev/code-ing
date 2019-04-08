/*
 * VitaluzIoT.c
 *
 *  Created on: 05/03/2019
 *      Author: Eddie Vazquez Hernandez
 */
#include "VitaluzIoT.h"

uint8_t VitaluzIoT_Send(package* payload, pdp_cfg* pdp, socket_cfg* socket){
	char json[150] = "{\"k\":\"";
	char f_buff[40] = {0};
	/*pdp_cfg pdp;

	pdp.apn = "HOLOGRAM";
	pdp.user = "";
	pdp.pwd = "";
	pdp.id = 1;
	pdp.auth = 0;
	*/


	strcat(json, DEVICE_KEY);
	strcat(json, "\", \"d\":\"");
	parse_package(payload, f_buff); //to save data todo:uncomment in final ASAP!!
	//f_buff[0] = '1';//todo:delete after testing only for saving data
	strcat(json, f_buff);
	strcat(json, "\",\"t\":\"TOPIC1\"}");

	uint8_t uh = BG96_SocketInit(BG_UART, socket);
	BG96_SendData(BG_UART, socket, json);
	BG96_SocketClose(BG_UART, socket);
	//BG96_PDPDeinit(BG_UART, &pdp);
	return 0;
}

uint8_t VitaluzIoT_Write(uint8_t outputs, Pin* r1, Pin* r2, Pin* v){
	if(outputs & R1_MASK)
		Relay_Set(r1);
	else
		Relay_Clear(r1);

	if(outputs & R2_MASK)
		Relay_Set(r2);
	else
		Relay_Clear(r2);

	if(outputs & V_MASK)
		Relay_Set(v);
	else
		Relay_Clear(v);

	return 0;
}
/*In buffer mode, send request data cmd and fetch bytes into char* data*/
uint8_t VitaluzIoT_FetchData(socket_cfg* socket, char* data ,uint8_t size){
	/*AT+QIRD=11\r*/
	char cmd[12] = {0}, aux[2] = {0};
	strcat(cmd, "AT+QIRD=");
	sprintf(aux, "%d", socket->id);
	strcat(cmd, aux);
	strcat(cmd, "\r");
	BG96_SendCmd(BG_UART, data, 0, '[');

	return 0;
}

uint8_t VitaluzIoT_ListenEnable(pdp_cfg* pdp, socket_cfg* socket){



	/*tcp server cmd: AT+QIOPEN=1,0,"TCP LISTENER","127.0.0.1",0, 4010,1*/

	//BG96_PDPInit(BG_UART, &pdp);//we are using a global pdp init which works fine
	uint8_t err = BG96_SocketInit(BG_UART, socket);
	BG96_Listen(BG_UART);
	return 0;
}

void VitaluzIoT_Deafen(pdp_cfg* pdp, socket_cfg* socket){

	/*tcp server cmd: AT+QIOPEN=1,0,"TCP LISTENER","127.0.0.1",0, 4010,1*/
	/*Identical to ListenEnable*/
	BG96_SocketClose(BG_UART, socket);
	BG96_Deafen(BG_UART);
}



/* !
 * brief Converts float to string
 *
 * param base USART base address
 * param data array to store every char read
 * param token char that stops function
 * param length pointer that stores number of read char
 * return number of chars in float string */
uint8_t float2str(float num, char* str, uint8_t frac_dig){
    uint8_t n_char = 0;
    float aux;
    uint8_t digit, count=0;
    /*Get the number of required integer digits*/
    n_char = int_digits(num);
    char digit_str[1];
    /*Initialize str as empty string*/
    for(uint8_t i = 0; i < strlen(str); i++)
    	str[i] = 0;
    aux = num;
    /*Check if negative*/
    if(aux < 0){
        str[count++] = '-';
        /*after adding minus symbol we need absolute*/
        aux *= -1.0f;
        n_char--;
    }
    /*Loop to convert integer part*/
    for(uint8_t i = n_char; i > 0; i--){
    	/*Obtains digit according to tenth position*/
        digit = (uint8_t)(aux / pow(10, i - 1));
        /*Subtract digit weight from number*/
        aux -= digit*pow(10, i - 1);
        /*Convert digit to ascii*/
        sprintf(digit_str, "%d", digit);
        /*Concatenate digit to string*/
        strcat(str, digit_str);
        /*Count correspond to number of chars*/
        count++;
    }
    count++;
    /*Add decimal point*/
    strcat(str, ".");

    /*Loop for fractional part. */
    for(uint8_t i = 0; i < frac_dig; i++){
    	/*Make first decimal one digit integer*/
        aux *=10;
        /*Discard decimals*/
        digit = (uint8_t)(aux);
        /*Discard digit by substracting*/
        aux -= (float)digit;
        /*Convert to char*/
        sprintf(digit_str, "%d", digit);
        /*Concatenate*/
        strcat(str, digit_str);

        count++;
    }

    /*We add three chars counting point and two fractional digits*/
    return count;

}

uint8_t int_digits(float x){
    float aux;
    uint8_t count = 0;
    aux = x;
    if(aux < 0){
        count++;
        aux *= -1.0f;
    }
    do{
        count++;
        aux /= 10;
    }while(aux > 1);
    return count;
}
/*t:-203,v:2.25,i:15,o:15,r:3,s:1*/
void parse_package(package* payload, char* str){
	raw_float_t volume;
	char aux_buff[10];
	char f_buff[10];
	for(uint8_t i = 0; i < strlen(str); i++)
		str[i] = 0;
	strcat(str,"t:");
	float2str(payload->temperature, aux_buff, 3);
	strcat(str, aux_buff);
	strcat(str, ",v:");
	float2str(payload->volume, f_buff, 3);
	strcat(str, f_buff);
	strcat(str, ",i:");
	itoa(payload->inputs, aux_buff,10);
	strcat(str, aux_buff);
	strcat(str, ",o:");
	itoa(payload->outputs, aux_buff,10);
	strcat(str, aux_buff);
	strcat(str, ",r:");
	itoa(payload->relays, aux_buff,10);
	strcat(str, aux_buff);
	strcat(str, ",s:");
	itoa(payload->valve, aux_buff,10);
	strcat(str, aux_buff);
}

uint8_t VitaluzIoT_Receive(pdp_cfg* pdp, socket_cfg* incoming_socket){
	uint8_t data[100] = {0}, incoming_cmd[4] = {0} ;
	/*If function returns with 0xFF code it means it timed out*/
	if(BG96_StoreTimeout(BG_UART, data,',',  45, 3000000) == 255){
		USART_Deinit(BG_UART);
		return 255;
	}

	 /*Parse socket id to integer*/
	 if(data[1] == ',')
		 for(uint8_t i = strlen(data); i > 0 ; i--)
			 data[i] = 0;
	 else if(data[2] == ',')
		 for(uint8_t i = strlen(data); i > 1 ; i--)
			 data[i] = 0;
	 else{

	     for(uint8_t i = 0; i < strlen(data); i++)
	    	 data[i] = 0;
	     return 10; //todo: create enum error
	 }
	 incoming_socket->id = (uint8_t)strtol(data, NULL, 10);
	 char cmd[17] = {0};
	 char aux[2] = {0};
	 /*Command to fetch data received via socket*/
	 strcat(cmd, "AT+QIRD=");
	 sprintf(aux, "%d", incoming_socket->id);
	 strcat(cmd, aux);
	 strcat(cmd,"\r");
	 /*Sends command without changing uart initialization*/
	 BG96_SendCmd(BG_UART, cmd, no_init ,0 );
	 /*rx_buffer reset*/
	 for(uint8_t i = 0; i < strlen(data); i++){
		 data[i] = 0;
	 }
	 BG96_Store(BG_UART, incoming_cmd, '[', ']', 3000000); //todo: consider adding another arg to force function to return after n chars
	 /* Important!! ~700ms pause required before closing socket.*/
     halt(700000);
	 /*Close socket command. Closing incoming_socket avoids*/
	 /*BG96 restriction of only 12 available sockets */
	 char cmd2[17] = {0};
	 strcat(cmd2, "AT+QICLOSE=");
	 strcat(cmd2, aux);
	 strcat(cmd2, "\r");
	 USART_Write(BG_UART, cmd2);
	 uint16_t bla ;
	 BG96_StoreAfter(BG_UART,  cmd2, '\n',  1);


}

void VitaluzIoT_RefreshServer(pdp_cfg* pdp, socket_cfg* server){
	 char cmd2[17] = {0}, aux[3] = {0};
	 strcat(cmd2, "AT+QICLOSE=");
	 sprintf(aux, "%d", server->id);
	 strcat(cmd2, aux);
	 strcat(cmd2, "\r");
	 USART_Write(BG_UART, cmd2);
	 BG96_StoreAfter(BG_UART,  cmd2, '\n',  1);
	 USART_Deinit(BG_UART);
	 halt(200000);
	 VitaluzIoT_ListenEnable(pdp, server);
}












