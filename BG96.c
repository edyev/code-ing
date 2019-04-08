/*
 * BG96.c
 *
 *  Created on: 11/02/2019
 *      Author: Eddie Vazquez Hernandez
 */


#include <BG96.h>

void BG96_Init(USART_Type* base){


	 BG96_PinSetup();
	 CLOCK_Select(kUART1_Clk_From_MainClk);


	    /* Default config by using USART_GetDefaultConfig():
	     * config->baudRate_Bps = 9600U;
	     * config->parityMode = kUSART_ParityDisabled;
	     * config->stopBitCount = kUSART_OneStopBit;
	     * config->bitCountPerChar = kUSART_8BitsPerChar;
	     * config->loopback = false;
	     * config->enableRx = false;
	     * config->enableTx = false;
	     * config->syncMode = kUSART_SyncModeDisabled;
	     */

	 USART_GetDefaultConfig(&usart_cfg);
	 usart_cfg.enableRx = true;
	 usart_cfg.enableTx = true;
	 usart_cfg.baudRate_Bps = 115200;

	 USART_Init(base, &usart_cfg, CLOCK_GetFreq(kCLOCK_MainClk));
	 char* buff = "BG Ready!\n";
	// USART_WriteBlocking(base, buff, strlen(buff));

}

void BG96_PinSetup(void){

	/* Configures Switch matrix for TX=P1_20 and RX=P1_21 */
	/* Enables clock for IOCON.: enable */
	CLOCK_EnableClock(kCLOCK_Iocon);
	/* Enables clock for switch matrix.: enable */
	CLOCK_EnableClock(kCLOCK_Swm);

	const uint32_t pio52_config = (/* Selects pull-up function */
	                               IOCON_PIO_MODE_PULLUP |
	                               /* Enable hysteresis */
	                               IOCON_PIO_HYS_EN |
	                               /* Input not invert */
	                               IOCON_PIO_INV_DI |
	                               /* Disables Open-drain function */
	                               IOCON_PIO_OD_DI |
	                               /* Bypass input filter */
	                               IOCON_PIO_SMODE_BYPASS |
	                               /* IOCONCLKDIV0 */
	                               IOCON_PIO_CLKDIV0);
	  /* PORT4 PIN4 (coords: ) is configured as  */
	  IOCON_PinMuxSet(IOCON, IOCON_INDEX_PIO1_20, pio52_config);

	  const uint32_t pio53_config = (/* Selects pull-up function */
	                                 IOCON_PIO_MODE_PULLUP |
	                                 /* Enable hysteresis */
	                                 IOCON_PIO_HYS_EN |
	                                 /* Input not invert */
	                                 IOCON_PIO_INV_DI |
	                                 /* Disables Open-drain function */
	                                 IOCON_PIO_OD_DI |
	                                 /* Bypass input filter */
	                                 IOCON_PIO_SMODE_BYPASS |
	                                 /* IOCONCLKDIV0 */
	                                 IOCON_PIO_CLKDIV0);
	  /* PORT4 PIN5 (coords: ) is configured as  */
	  IOCON_PinMuxSet(IOCON, IOCON_INDEX_PIO1_2, pio53_config);

	  /*
	  * tx: P1_20->0x0D0 (datasheet p. 152) = 208 ->208/4= 52
		rx: P1_21->0xd4 = 212 -> 212/4 =53
	   */
	  /* USART0_TXD connect to P1_20 */
	  SWM_SetMovablePinSelect(SWM0, kSWM_USART1_TXD, kSWM_PortPin_P1_20);

	  /* USART0_RXD connect to P1_21 */
	  SWM_SetMovablePinSelect(SWM0, kSWM_USART1_RXD, kSWM_PortPin_P1_21);

	  /* Disable clock for switch matrix. */
	  CLOCK_DisableClock(kCLOCK_Swm);

}


/* !
 * brief intializes and activates PDP
 * Every command inits and deinits USART
 *
 *
 * param base USART base address
 * param apn Access Point Name
 * param user
 * param pwd Password for user
 * aut	 type of authentication
 */
uint8_t BG96_PDPInit(USART_Type *base, pdp_cfg* cfg){
	/*
	 * Sequence to initialize and activate PDP
	 * AT+QICSGP=1,1,"apn_name","user","pwd",0\r
	 * AT+QIACT=1\r
	 */
	char cmd[100] = {0};
	char aux[2];
	uint32_t err = 0;

	strcat(cmd, "AT+QICSGP=");
	uitoa(cfg->id, aux, 10);
	strcat(cmd, aux);
	/*IPV4*/
	strcat(cmd, ",1,\"");
	strcat(cmd, cfg->apn);
	strcat(cmd, "\",\"");
	strcat(cmd, cfg->user);
	strcat(cmd, "\",\"");
	strcat(cmd, cfg->pwd);
	strcat(cmd, "\",");
	uitoa(cfg->auth, aux, 10);
	strcat(cmd, aux);
	strcat(cmd, "\r");
	/*Check for response required otherwise this cmd */
	/*response can be read by the next cmd*/
	err = BG96_SendCmd(base, cmd, 1, 0);
	halt(1000);
	for(uint8_t i = 0; i < 100; i++)
		cmd[i] = 0;

	strcat(cmd,"AT+QIACT=");
	uitoa(cfg->id, aux, 10);
	strcat(cmd,aux);
	strcat(cmd,"\r");
	err = BG96_SendCmd(base, cmd, 1, 0);
	return err;
}

/* !
 * brief Socket initialization
 *
 *
 * param base USART base address
 * param cfg socket configuration
 */
uint8_t BG96_SocketInit(USART_Type* base, socket_cfg* cfg){
	//AT+QIOPEN=1,0,"TCP","cloudsocket.hologram.io",9999,0,1<CR>
	char cmd[100] = {0}, resp_code[15] = {0};
	char aux[5] = {0};
	uint8_t err = 0, count = 0;

	strcat(cmd, "AT+QIOPEN=");
	sprintf(aux, "%d", cfg->pdp_id);
	strcat(cmd,aux);
	strcat(cmd, ",");
	sprintf(aux, "%d", cfg->id);
	strcat(cmd,aux);
	strcat(cmd, ",\"");
	/*
	 * tcp_client,
	udp_client,
	tcp_server,
	udp_service
	 *
	 *
	 * */
	switch(cfg->mode){
	case tcp_client: strcat(cmd, "TCP");
		break;
	case udp_client: strcat(cmd, "UDP");
		break;
	case tcp_server: strcat(cmd, "TCP LISTENER");
		break;
	case udp_service: strcat(cmd, "UDP SERVICE");
		break;
	default:
		break;
	}
	strcat(cmd, "\",\"");
	strcat(cmd, cfg->addr);
	strcat(cmd, "\",");
	sprintf(aux, "%d", cfg->remote_port);
	strcat(cmd,aux);
	strcat(cmd, ",");
	sprintf(aux, "%d", cfg->local_port);
	strcat(cmd,aux);
	strcat(cmd, ",");
	sprintf(aux, "%d", cfg->access);
	strcat(cmd,aux);
	strcat(cmd, "\r");
	/*Send command without deinitializing*/
	BG96_SendCmd(base, cmd, 101, ',');
	/*Store response, which include error flag*/
	BG96_Store(base, resp_code, ':', '\r', 10000000);
	for(uint8_t i = 0; i < strlen(aux); i++)
		aux[i] = 0;
	for(uint8_t i = 0; i < strlen(resp_code); i++){
		if(err)
			aux[count++] = resp_code[i];
		if(resp_code[i] == ',')
			err = 1;
	}
	BG96_Deafen(base);
	return (uint8_t)strtol(aux, NULL, 10);
}
/* !
 * brief Sends string of data via socket pointed by cfg
 * A PDP context and socket has to be already configured
 *
 * param base USART base address
 * param cfg sending socket
 * param payload data to be sent
 */
uint8_t BG96_SendData(USART_Type* base, socket_cfg* cfg, char* payload){
	char cmd[100] = {0};
	char aux[2];
	uint8_t err = 0;

	strcat(cmd, "AT+QISEND=");
	sprintf(aux, "%d", cfg->id);
	strcat(cmd, aux);
	strcat(cmd,"\r");
	BG96_SendCmd(base, cmd, 0, 0);
	halt(1000);
	/*Wait for token before sending data*/
	for(uint8_t i = 0; i < 100; i++)
		cmd[i] = 0;

	strcat(cmd, payload);
	strcat(cmd, "\r\r\r\x1A");
	/*To do: validate > char */
	halt(100000);
	err = BG96_SendCmd(base, cmd, 0, 0);
	halt(7000000);
	return err;

}
/* !
 * brief Deactivates context PDP and hence disconnects from network
 *
 *
 *
 * param base USART base address
 * param cfg context PDP
 */
uint8_t BG96_PDPDeinit(USART_Type* base, pdp_cfg* cfg){
	char cmd[15];
	char aux[2];
	uint8_t err = 0;

	strcat(cmd, "AT+QIDEACT=");
	sprintf(aux, "%d", cfg->id);
	strcat(cmd, aux);
	strcat(cmd, "\r");
	//err = BG96_SendCmd(base, cmd,2, 'K');
	err = BG96_SendCmd(base, cmd, 0,0);
	halt(5000000);
	return err;
}

/* !
 * brief Closes socket
 *
 *
 *
 * param base USART base address
 * param cfg pointer to socket config
 */

uint8_t BG96_SocketClose(USART_Type* base, socket_cfg* cfg){
	char cmd[15] = {0};
	char aux[2];
	uint8_t err = 0;
	strcat(cmd, "AT+QICLOSE=");
	sprintf(aux, "%d", cfg->id);
    strcat(cmd, aux);
	strcat(cmd, "\r");
	err = BG96_SendCmd(base, cmd,1, 0);
	return err;
}

/* !
 * brief Enables USART to receive data
 * No SendCmd should be used otherwise unknown behaviour might occur
 * BG96_Deaf must be called afterwards everytime this function is used.
 *
 *
 * param base USART base address
 */
uint8_t BG96_Listen(USART_Type* base){
	return USART_Init(base, &usart_cfg, CLOCK_GetFreq(kCLOCK_MainClk));
}

/* !
 * brief Disables USART.
 *
 *
 * param base USART base address
 */
void BG96_Deafen(USART_Type* base){
	USART_Deinit(base);
}

/* !
 * brief atomic function to send a command
 * Every command inits and deinits USART
 *
 *
 * param base USART base address
 * param cmd AT command string.
 * param response_type type of processing of response to command
 * param lines flexible variable to control flow of function
 * 		 lines can also work as a char token if size == 3
 * Todo: implement error code (return) and rename size
 */
uint8_t BG96_SendCmd(USART_Type* base, char* cmd, uint8_t size, char lines){

	uint8_t resp_code = 0;
	uint16_t msg_size;
	char data[100] = {0} ;
	char aux[3] = {0};
	size_t msg;
	/*if size == 100 skip USART deinit and init*/
	if(size != 100)
		USART_Init(base, &usart_cfg, CLOCK_GetFreq(kCLOCK_MainClk));

	USART_Write(base, cmd);

	/*If size == 0 no response check */
	if(!size){
		USART_Deinit(base);
		return 0;
	}
	for(uint8_t i = 0; i < strlen(data); i++)
		data[i] = 0;
	for(uint8_t i = 0; i < strlen(cmd); i++)
		cmd[i] = 0;
	/*If 1 simple processing after first \n for cmds with OK basic response */
	/*If 2 wait until token is found. No timeout yet*/
	/*If*/
	if(size == 1)
		BG96_StoreAfter(base, data, '\n', 2);
	else if (size == wait_resp){
		BG96_WaitUntil(base, (char)lines, &msg_size);
		USART_Deinit(base);
		return 0;
	}
	else if (size == store_resp){ //todo: move to another function it is not good modify cmd
		BG96_StoreAfter(base, aux, (char)lines, 1);
		USART_Deinit(base);
		return (uint8_t)strtol(aux, NULL, 10);
	}
	else if (size >= 100){
		return 0;
	}

	/* USART deinit. for flushing buffer
	 * Buffer flush needed to discard useless data in
	 * verification mode.
	 * If not flushed data for next operation will correspond
	 * to previous receptions and hence be corrupted
	 * */

	USART_Deinit(base);
	//simple ok check
	if(data[0]=='O')
		return 0;
	else
		return 1;

}

/*	Todo: only for reference, delete later.
 *
 * 	Function must be called right after issuing a command
 * 	This function performs a basic check of responses
 * 	from BG96
 * 	*/


/*To do: rename, this is BG96 AT cmd read based on USART read blocking
 * brief keeps reading rx until a token char is found
 *
 * param base USART base address
 * param data array to store every char read
 * param token char that stops function
 * param length pointer that stores number of read char */


status_t USART_ReadUntil(USART_Type *base, char* data, char token, uint16_t* length)
{
    uint32_t status;
    uint8_t condition = 0;


    /* Check arguments */
    assert(!((NULL == base) || (NULL == data)));
    *length = 0;

   do
    {
        /* loop until receive is ready to read */
        while (!(base->STAT & USART_STAT_RXRDY_MASK))
        {
        }

        data[*length] = base->RXDAT;

        *length = *length + 1;

        /* Check receive status */
        status = base->STAT;

        if (status & USART_STAT_FRAMERRINT_MASK)
        {
            base->STAT |= USART_STAT_FRAMERRINT_MASK;
            return kStatus_USART_FramingError;
        }
        if (status & USART_STAT_PARITYERRINT_MASK)
        {
            base->STAT |= USART_STAT_PARITYERRINT_MASK;
            return kStatus_USART_ParityError;
        }
        if (status & USART_STAT_RXNOISEINT_MASK)
        {
            base->STAT |= USART_STAT_RXNOISEINT_MASK;
            return kStatus_USART_NoiseError;
        }
        if (base->STAT & USART_STAT_OVERRUNINT_MASK)
        {
            base->STAT |= USART_STAT_OVERRUNINT_MASK;
            return kStatus_USART_HardwareOverrun;
        }

        condition = (data[*length-1]) != token;

        if(data[*length-1] == token && (*length-1) < 2)
        	condition = 1;

    } while(condition);

   	/* length - 1 accounts for current char as it is
   	 * increased right after reading
   	 *
   	 * Condition accounts for BG96 response that includes a token
   	 * at the beginning and one at the end. Ignores first one.
   	 */
    return kStatus_Success;
}
/*
 * brief Reads buffer without storing bytes until token is found
 *
 * param base USART base address
 * param token char that stops function
 * param length pointer that stores number of read char */

uint8_t BG96_WaitUntil(USART_Type *base, char token, uint16_t* length)
{
    uint32_t status;
    char _buffer;
    uint8_t condition = 0;
    uint32_t error = kStatus_Success;

    /* Check arguments */
    assert(!((NULL == base)));
    *length = 0;

   do
    {
        /* loop until receive is ready to read */
        while (!(base->STAT & USART_STAT_RXRDY_MASK))
        {
        }

        _buffer= base->RXDAT;

        *length = *length + 1;

        /* Check receive status */
        status = base->STAT;

        if (status & USART_STAT_FRAMERRINT_MASK)
        {
            base->STAT |= USART_STAT_FRAMERRINT_MASK;
            error = kStatus_USART_FramingError;
        }
        if (status & USART_STAT_PARITYERRINT_MASK)
        {
            base->STAT |= USART_STAT_PARITYERRINT_MASK;
            error =  kStatus_USART_ParityError;
        }
        if (status & USART_STAT_RXNOISEINT_MASK)
        {
            base->STAT |= USART_STAT_RXNOISEINT_MASK;
            error =  kStatus_USART_NoiseError;
        }
        if (base->STAT & USART_STAT_OVERRUNINT_MASK)
        {
            base->STAT |= USART_STAT_OVERRUNINT_MASK;
            error =  kStatus_USART_HardwareOverrun;
        }

        condition = (_buffer) != token;


    } while(condition);

   	/* length - 1 accounts for current char as it is
   	 * increased right after reading
   	 *
   	 * Condition accounts for BG96 response that includes a token
   	 * at the beginning and one at the end. Ignores first one.
   	 */
    return (uint8_t)kStatus_Success;
}

/*
 * brief Reads buffer and stores into data n characters after token is read
 *
 * param base USART base address
 * param data pointer to array where buffer chars will be stored
 * param token char that begins storage
 * param n number of bytes to store in data */

uint8_t BG96_StoreTimeout(USART_Type* base, char* data, char token, uint16_t n,
						  uint32_t timeout){
    uint32_t status;
    char _buffer;
    uint8_t condition = 0, store = 0;
    uint32_t error = kStatus_Success;
    uint32_t length;
    uint32_t count = 0;
    /* Check arguments */
    assert(!((NULL == base)));
    length = 0;

   do
    {
        /* loop until receive is ready to read */
        while (!(base->STAT & USART_STAT_RXRDY_MASK))
        {
        	if(count++ > timeout)
        		return 255;

        }


        _buffer= base->RXDAT;

        if(store){
        	*data = _buffer;
        	data++;
        	n--;
        }



        /* Check receive status */
        status = base->STAT;

        if (status & USART_STAT_FRAMERRINT_MASK)
        {
            base->STAT |= USART_STAT_FRAMERRINT_MASK;
            error = kStatus_USART_FramingError;
        }
        if (status & USART_STAT_PARITYERRINT_MASK)
        {
            base->STAT |= USART_STAT_PARITYERRINT_MASK;
            error =  kStatus_USART_ParityError;
        }
        if (status & USART_STAT_RXNOISEINT_MASK)
        {
            base->STAT |= USART_STAT_RXNOISEINT_MASK;
            error =  kStatus_USART_NoiseError;
        }
        if (base->STAT & USART_STAT_OVERRUNINT_MASK)
        {
            base->STAT |= USART_STAT_OVERRUNINT_MASK;
            error =  kStatus_USART_HardwareOverrun;
        }
        /*Once token is reached always 1*/
        store = store ? 1 : !(_buffer != token);
        if(!n)
        	break;

    } while(1);

   	/* length - 1 accounts for current char as it is
   	 * increased right after reading
   	 *
   	 * Condition accounts for BG96 response that includes a token
   	 * at the beginning and one at the end. Ignores first one.
   	 */
    return length;

}
/*
 * brief Reads buffer and stores into data n characters after token is read
 *
 * param base USART base address
 * param data pointer to array where buffer chars will be stored
 * param token char that begins storage
 * param n number of bytes to store in data */

uint8_t BG96_StoreAfter(USART_Type* base, char* data, char token, uint16_t n){
    uint32_t status;
    char _buffer;
    uint8_t condition = 0, store = 0;
    uint32_t error = kStatus_Success;
    uint32_t length;
    /* Check arguments */
    assert(!((NULL == base)));
    length = 0;

   do
    {
        /* loop until receive is ready to read */
        while (!(base->STAT & USART_STAT_RXRDY_MASK))
        {

        }


        _buffer= base->RXDAT;

        if(store){
        	*data = _buffer;
        	data++;
        	n--;
        }



        /* Check receive status */
        status = base->STAT;

        if (status & USART_STAT_FRAMERRINT_MASK)
        {
            base->STAT |= USART_STAT_FRAMERRINT_MASK;
            error = kStatus_USART_FramingError;
        }
        if (status & USART_STAT_PARITYERRINT_MASK)
        {
            base->STAT |= USART_STAT_PARITYERRINT_MASK;
            error =  kStatus_USART_ParityError;
        }
        if (status & USART_STAT_RXNOISEINT_MASK)
        {
            base->STAT |= USART_STAT_RXNOISEINT_MASK;
            error =  kStatus_USART_NoiseError;
        }
        if (base->STAT & USART_STAT_OVERRUNINT_MASK)
        {
            base->STAT |= USART_STAT_OVERRUNINT_MASK;
            error =  kStatus_USART_HardwareOverrun;
        }
        /*Once token is reached always 1*/
        store = store ? 1 : !(_buffer != token);
        if(!n)
        	break;

    } while(1);

   	/* length - 1 accounts for current char as it is
   	 * increased right after reading
   	 *
   	 * Condition accounts for BG96 response that includes a token
   	 * at the beginning and one at the end. Ignores first one.
   	 */
    return length;

}
/*
 * brief Reads chars received between start_token and end_token
 *
 * param base USART base address
 * param data pointer to array where buffer chars will be stored
 * param start_token char that begins storage
 * param end_token char that finished storage and finishes function */

uint8_t BG96_Store(USART_Type* base, char* data, char start_token, char end_token, uint32_t timeout ){
    uint32_t status;
    char _buffer;
    uint8_t store = 0;
    uint32_t error = kStatus_Success;
    uint32_t count = 0;
    uint32_t length;
    /* Check arguments */
    assert(!((NULL == base)));
    length = 0;

   do
    {
        /* loop until receive is ready to read */
        while (!(base->STAT & USART_STAT_RXRDY_MASK))
        {
        	if(count++ > timeout)
        		return 255;
        }


        _buffer= base->RXDAT;

        if(store){
        	*data = _buffer;
        	data++;

        }



        /* Check receive status */
        status = base->STAT;

        if (status & USART_STAT_FRAMERRINT_MASK)
        {
            base->STAT |= USART_STAT_FRAMERRINT_MASK;
            error = kStatus_USART_FramingError;
        }
        if (status & USART_STAT_PARITYERRINT_MASK)
        {
            base->STAT |= USART_STAT_PARITYERRINT_MASK;
            error =  kStatus_USART_ParityError;
        }
        if (status & USART_STAT_RXNOISEINT_MASK)
        {
            base->STAT |= USART_STAT_RXNOISEINT_MASK;
            error =  kStatus_USART_NoiseError;
        }
        if (base->STAT & USART_STAT_OVERRUNINT_MASK)
        {
            base->STAT |= USART_STAT_OVERRUNINT_MASK;
            error =  kStatus_USART_HardwareOverrun;
        }
        /*Once token is reached always 1*/
        store = store ? 1 : !(_buffer != start_token);
        if(_buffer == end_token & store){
        	*(--data) = 0;
        	break;
        }

    } while(1);

   	/* length - 1 accounts for current char as it is
   	 * increased right after reading
   	 *
   	 * Condition accounts for BG96 response that includes a token
   	 * at the beginning and one at the end. Ignores first one.
   	 */
    return length;

}



uint8_t BG96_ShutDown(USART_Type* base){
	return BG96_SendCmd(base, "AT+QPOWD\r", 0, 0);
}



static void halt(uint32_t nops){
	for(uint32_t i = 0; i < nops; i++)
		__asm("nop");
}


