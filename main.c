/*
 *      Coded by Eddie Vazquez Hernandez
 *        for Vitaluz IoT
 *
 *
 */

#include <VitaluzIoT.h>

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void vErrorTask(void*);
void vSendTask(void*);
void vVolumeTask(void*);
void vPollTask(void*);
void vAlarmTask(void*);
void vTemperatureTask(void*);
void vEEPROMTask(void*);
void vServerKeeper(void*);
void vResetTask(void* pV);
/*******************************************************************************
 * Global Variables
 ******************************************************************************/
//4debugging
volatile float dbg_flag = 0;

/*For setting TCP reports period*/
volatile uint32_t send_period_sec;

/*Pins for external hardware*/
IO_Port port;
relay_pin relay[2];
valve_pin valve;
Pin output[4], input[4];
Pin ds18_pin = {1, 14, 0};
pdp_cfg pdp;
wwdt_config_t wwdt_cfg;
/*Report periods are stored in this array (seconds)*/
uint32_t cfg_periods[10] = {SECONDS(60), MINUTES(10), MINUTES(30), HOURS(1),
                            HOURS(2), HOURS(3), HOURS(4),
                            HOURS(6), HOURS(8), HOURS(12)};
/*uint32_t cfg_periods[10] = {10, 20, 30 , 40,
                            50, 60 , 60 * 120,
                            180 * 60, 240 * 60, 60 * 60 * 24};*///fast testing values

/*Socket declaration*/
socket_cfg send_socket, server_socket;
/*Queues declaration*/
QueueHandle_t xVolQueue;
QueueHandle_t xTempQueue;
QueueHandle_t xFlagQueue;
QueueHandle_t xServerTimeoutQueue;
QueueHandle_t xReportPeriodQueue;
QueueHandle_t xRespQueue;
/*Task handles*/
TaskHandle_t xAlarmTaskHandle;
TaskHandle_t xPollTaskHandle;
TaskHandle_t xResetHandle;
/*Mutex to protect USART*/
SemaphoreHandle_t xUsartMux;
/*******************************************************************************
 * Code
 ******************************************************************************/
//debug: for checking state before reset
void WDT_IRQHandler(void){
    uint32_t wdtStatus = WWDT_GetStatusFlags(WWDT);



    /* The chip will reset before this happens */


    /* Handle warning interrupt */


            while (WWDT->TV >= WWDT->WARNINT)
            {
            }
            /* Feed only for the first 5 warnings then allow for a WDT reset to occur */
            WWDT_Refresh(WWDT);



        while (WWDT->TV < WWDT->WARNINT)
        {
        }
        /* A watchdog feed didn't occur prior to warning timeout */
        WWDT_ClearStatusFlags(WWDT, kWWDT_WarningFlag);
}
/*Callback for movement detection interrupt*/
//maybe itd be better if we implement right in the IRQ handler, reduces overhead
void pint_intr_callback(pint_pin_int_t pintr, uint32_t pmatch_status)
{
    /*On interruption alarm task (max priority) is resumed*/
    xTaskResumeFromISR(xAlarmTaskHandle);
}

/*!
 * @brief Main function
 */
int main(void)
{

    USART_cfg exp_usart_cfg;
    I2C_cfg exp_i2c_cfg;
    SPI_cfg exp_spi_cfg;
    /*PDP context config*/
    pdp.apn = "HOLOGRAM";
    pdp.user = "";
    pdp.pwd = "";
    pdp.id = 1;
    pdp.auth = 0;
    /*Sockets configuration*/
    /*Socket to send data to Hologram*/
    send_socket.addr = "cloudsocket.hologram.io" ;
    send_socket.mode = tcp_client;
    send_socket.access = direct_push;
    send_socket.local_port = 0;
    send_socket.remote_port = 9999;
    send_socket.pdp_id = pdp.id;
    send_socket.id = 0;
    /*Socket for listening for commands (server, incoming)*/
    /*tcp server cmd: AT+QIOPEN=1,0,"TCP LISTENER","127.0.0.1",0, 4010,1*/
    server_socket.addr = "10.59.22.103";
    server_socket.mode = tcp_server;
    server_socket.access = buffer;
    server_socket.local_port = 4010;
    server_socket.remote_port = 0;
    server_socket.pdp_id = pdp.id;
    server_socket.id = 1;
    /*Define I/O*/
    relay[0].pin = 12; //red
    relay[0].port = 0;
    relay[1].pin = 0; //green
    relay[1].port = 0;
    valve.pin = 15;//blue
    valve.port = 1;
    /*Expansion port usaer cfg. todo: do we need it?*/
    exp_usart_cfg.base = USART2;
    exp_i2c_cfg.base = I2C1;
    exp_spi_cfg.base = SPI0;
    /*Short pause before starting up everything*/
    for(uint32_t i = 0; i < 80000; i++)
            __asm("nop");
    /* Select 30 MHz clock */
    BOARD_BootClockFRO30M();
    /*Enable WD clock*/
    CLOCK_InitWdtOsc(kCLOCK_WdtAnaFreq600KHZ, 4U);
    //NVIC_EnableIRQ(WDT_IRQn);
    WWDT_GetDefaultConfig(&wwdt_cfg);
    /*35000U * SECONDS*/
    wwdt_cfg.timeoutValue = 35000U * 240U;
    wwdt_cfg.warningValue = 1023;
    wwdt_cfg.windowValue = 0xFFFFFFFF;
    /* Configure WWDT to reset on timeout */
    wwdt_cfg.enableWatchdogReset = true;
    /* Setup watchdog clock frequency(Hz). */
    wwdt_cfg.clockFreq_Hz = CLOCK_GetFreq(kCLOCK_WdtOsc);
    /*Set up alarm(interrupt) signal*/
    SYSCON_AttachSignal(SYSCON, kPINT_PinInt0, kSYSCON_GpioPort0Pin28ToPintsel);
    /*Init expansion port*/
    ExpIO_Init(&exp_usart_cfg, &exp_i2c_cfg, &exp_spi_cfg);
    Relay_Init(&relay[0]);
    Relay_Init(&relay[1]);
    Relay_Init(&valve);
    /*Init. communications Quectel BG96 via UART1 */
    BG96_Init(BG_UART);
    /*Init. flow sensor counter and input*/
    FlowSensor_Init();
    /*Init. accelerometer LIS2H via I2C0*/
    LIS2H_Init(I2C0);
    /*Init. DS18B20 One Wire temperature sensor. Flow not allowed if sensor not present*/
    while(!DS18_Init(&ds18_pin));
    /*Requests first conversion. If we don't request it we get default temp*/
    DS18_RequestConv(&ds18_pin);
    /*Configures and sets up PDP context needed for TCP connection*/
    /*Once set up we can create a server or client connection */
    uint8_t pdp_err = 0;
    /*Setup PDP. Zero means no error. Try indefinitely to connect*/
    do{
        pdp_err = BG96_PDPInit(BG_UART, &pdp);
    }
    while(pdp_err != 0);
    /*Setup pin interrupt that will be sourced by accelerometer*/
    /*Interrupt can only occur after PDP was set*/
    PINT_Init(PINT);
    PINT_PinInterruptConfig(PINT, kPINT_PinInt0, kPINT_PinIntEnableRiseEdge,
                            pint_intr_callback);
    PINT_EnableCallbackByIndex(PINT, kPINT_PinInt0);
    /*Init watchdog*/
    //WWDT_Init(WWDT, &wwdt_cfg);
    /*Configures data report period in seconds. Max: 2^32 sec*/
    send_period_sec = 60 * 10 ;
    /*Create queues*/
    /*Queues are used to provide with an RTOS safe mechanism for task */
    /*intercommunication.Size 1 as last value always overwrites previous*/
    /*Queue to accumulate volume*/
    xVolQueue = xQueueCreate( 1, sizeof( float ) );
    /*Queue for temperature samples*/
    xTempQueue = xQueueCreate(1, sizeof(float));
    /*This queue attempts to set a flag when a close of pdp is required*/
    xFlagQueue = xQueueCreate(1, sizeof(uint8_t));
    /*Queue that keeps track of how much time has passed without incoming*/
    /*connections so the server won't timeout*/
    xServerTimeoutQueue = xQueueCreate(1, sizeof(uint32_t));
    /*This queue stores and updates report period*/
    xReportPeriodQueue = xQueueCreate(1, sizeof(uint32_t));
    /*Mutex for protecting communication with BG96*/
    xUsartMux = xSemaphoreCreateMutex();
    if( xVolQueue == NULL || xTempQueue == NULL || xFlagQueue == NULL ||
             xServerTimeoutQueue == NULL ||  xReportPeriodQueue == NULL){
        USART_Write(USART2, (uint8_t*)"One or more queue was not created!\n");
    }
    if( xUsartMux == NULL)
        USART_Write(USART2, (uint8_t*)"Mutex was not created!\n");
    /*Create tasks*/
    xTaskCreate(vSendTask, "SendTask", 450, NULL, 5, NULL);
    xTaskCreate(vVolumeTask, "VolumeTask", 48, NULL, 2, NULL);
    xTaskCreate(vTemperatureTask, "TemperatureTask", 75, NULL, 3, NULL);
    xTaskCreate(vPollTask, "PollTask", 490, NULL, 2, &xPollTaskHandle);
    xTaskCreate(vAlarmTask, "AlarmTask", 300, NULL, 7, &xAlarmTaskHandle) ;
    xTaskCreate(vResetTask, "ResetTask", 80, NULL, 8, &xResetHandle) ;

    /*Start Kernel aka executing tasks*/
    vTaskStartScheduler();
    while (1)
    {

    }
}


void vSendTask(void* pV){
    /*Task that will send Hologram compliant json over tcp.*/
    /*Variables to read queues*/
    uint8_t close_flag = 0;
    volatile uint32_t period, timeout;
    float volume;
    /*Package that stores all variables to be sent*/
    package payload;
    /*Use of volume ONLY for first read of temperature*/
    /*as this is 1st task to be executed and queue is empty */
    /*1st Conversion requested in setup*/
    volume = DS18_ReadTemp(&ds18_pin); //todo: if memory available at final implementation create own var.
    xQueueOverwrite(xTempQueue, (void*)&volume);
    /*Initialize FlagQueue to zero, there is no need to close pdp*/
    xQueueOverwrite(xFlagQueue, (void*)&close_flag);
    /*Default period goes here*/
    period = 30 * 60;
    xQueueOverwrite(xReportPeriodQueue, (void*)&period);
    for(;;){
        /*Mutex with infinite waiting in blocked state time*/
        /*To account for any possible delays when mutex is blocked*/
        xSemaphoreTake( xUsartMux, (TickType_t) portMAX_DELAY );
        /*Verifies if PDP close is needed*/
        xQueuePeek(xFlagQueue, (void*)&close_flag,(TickType_t) 1  );
        if(close_flag)
            VitaluzIoT_Deafen(&pdp, &server_socket);
        volume = 0;
        /*Fetch volume from queue*/
        xQueuePeek(xVolQueue, (void*)&volume, (TickType_t) 10);
        /*Fetch package data*/
        payload.volume = volume;
        xQueuePeek(xTempQueue, (void*)&payload.temperature, (TickType_t) 1 );
        payload.relays = Pin_Read(&relay[1]) << 1 | Pin_Read(&relay[0]);
        payload.valve = Pin_Read(&valve);
        /*Send data packages through BG96*/
        VitaluzIoT_Send(&payload, &pdp, &send_socket);
        /*Socket init and UART enabling so incoming connection can be heard*/
        VitaluzIoT_ListenEnable(&pdp, &server_socket);
        /*Every time listen enable is called we have to close server socket*/
        close_flag = 1;
        xQueueOverwrite(xFlagQueue, (void*)&close_flag);
        /*Reset xServerTimeout as this task when executed resets server*/
        timeout = 0;
        xQueueOverwrite(xServerTimeoutQueue, (void*)&timeout);
        /*Reset accumulated volume*/
        volume = 0;
        xQueueOverwrite(xVolQueue, (void*)&volume);
        xSemaphoreGive( xUsartMux );
        /*Watchdog reset*/
        WWDT->FEED = 0xAA;
        WWDT->FEED = 0x55;
        for(uint32_t i = 0; i < period;++i){
            vTaskDelay(pdMS_TO_TICKS(1000));
            /*Fetch most recent report period from queue*/
            xQueuePeek(xReportPeriodQueue, (void*)&period, (TickType_t) 10);
        }

    }


}


/*This task gather volume readings and queues values*/
void vVolumeTask(void* pV){
    float volume, vol_aux; //todo: delete vol_aux
    for(;;){

        Relay_Toggle(&relay[0]);//delete just 4 debug, useful to know program hasnt hang
        xQueuePeek(xVolQueue, (void*)&volume, (TickType_t) 10);
        //volume += FlowSensor_getVolume();
        vol_aux = FlowSensor_getVolume();
        volume += vol_aux;
        xQueueOverwrite(xVolQueue, (void*)&volume);
        FlowSensor_Reset();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
void vPollTask(void* pV){
    /*
     * <CR><LF>
       +QIURC: "incoming",9,0,"10.176.100.3",51257<CR><LF>
       <CR><LF>
       +QIURC: "recv",9,1<CR><LF>
       &<CR><LF>
     */
    //+QIURC: "incoming",11,0,"10.176.100.3",38090<CR>
    package payload;
    uint8_t  close_flag, selector = 0, err_count = 0;
    uint32_t timeout = 0, period = 0, err = 0;
    /*String to store incoming data*/
    char rx_buffer[100] = {0};
    /*String for parsing received command*/
    char incoming_cmd[4] = {0} ;
    /*Incoming connection socket. Different to server socket*/
    socket_cfg incoming_socket;
    float temp, volume;
    for(;;){
        /*Once executed this task, it is implied a server socket is opened*/
        /*and hence we have to notify this to SendTask so it will close it*/
        close_flag = 1;
        xQueueOverwrite(xFlagQueue, (void*)&close_flag);
        /*This means data is being sent from BG96*/
        if(BG96_Check4Data(BG_UART)){
            /*Lock UART to prevent another task from using it*/
            xSemaphoreTake( xUsartMux, (TickType_t) 10 );
            /*Every time an incoming connection opens a new socket is opened*/
            /*The following line shows up when an incoming connection is prompted:*/
            /*+QIURC: "incoming",11,0,"10.176.100.3",5523*/
            /*Store chars after first ',' . Digits after it*/
            /*represent the socket id. Store 45 chars after ',' assures */
            /*we wait enough before issuing next cmd. Timeout is included*/
            /*If function times out (returns 0xFF) no further processing */
            /*This disregards unexpected data or messages*/
            if(BG96_StoreTimeout(BG_UART, rx_buffer, ',',  45, 3000000) != 0xFF){
                /*Parse socket id to integer*/
                if(rx_buffer[1] == ',')
                    for(uint8_t i = strlen(rx_buffer); i > 0 ; i--)
                        rx_buffer[i] = 0;
                else if(rx_buffer[2] == ',')
                    for(uint8_t i = strlen(rx_buffer); i > 1 ; i--)
                        rx_buffer[i] = 0;
                else{
                    err = 1;
                    for(uint8_t i = 0; i < strlen(rx_buffer); i++)
                        rx_buffer[i] = 0;
                }
                incoming_socket.id = (uint8_t)strtol(rx_buffer, NULL, 10);
                char cmd[17] = {0};
                char aux[2] = {0};
                /*Command to fetch data received via socket*/
                strcat(cmd, "AT+QIRD=");
                sprintf(aux, "%d", incoming_socket.id);
                strcat(cmd, aux);
                strcat(cmd,"\r");
                /*Sends command without changing uart initialization*/
                BG96_SendCmd(BG_UART, cmd, no_init ,0 );
                /*rx_buffer reset*/
                for(uint8_t i = 0; i < strlen(rx_buffer); i++){
                    rx_buffer[i] = 0;
                }
                /*Specific format to receive data is [data]*/
                /*Parse byte received*/
                uint32_t to = 0;
                to = BG96_Store(BG_UART, incoming_cmd, '[', ']', haltSEC(10));
                /* Important!! ~700ms pause required before closing socket.*/
                to++;
                halt(haltMS(100));
                /*Close socket command. Closing incoming_socket avoids*/
                /*BG96 restriction of only 12 available sockets */
                char cmd2[17] = {0};
                strcat(cmd2, "AT+QICLOSE=");
                strcat(cmd2, aux);
                strcat(cmd2, "\r");
                USART_Write(BG_UART, cmd2);
                BG96_StoreTimeout(BG_UART,  cmd2, '\n',  1, haltSEC(11));
                volume = 0;
                /*Fetch accumulated volume*/
                xQueuePeek(xVolQueue, (void*)&volume, (TickType_t)10);
                payload.volume = volume;
                xQueuePeek(xTempQueue, (void*)&temp, (TickType_t) 1 );
                payload.temperature = temp;
                payload.relays = Pin_Read(&relay[1]) << 1 | Pin_Read(&relay[0]);
                payload.valve = Pin_Read(&valve);
                /*Based on received cmd send data or write to I/O*/
                /*MSb defines operation. 1->send(read request) */
                /*0->Write operation to I/O. bit[6:0] maps to I/O*/
                selector = (uint8_t)strtol(incoming_cmd, NULL, 10);
                if(selector & RW_MASK){
                    /*Discard R/W bit only ls nibble has info.*/
                    if(selector == ON_DEMAND_MASK){
                        /*Send requires uninitialized USART*/
                        BG96_Deafen(BG_UART);
                        VitaluzIoT_Send(&payload, &pdp, &send_socket);
                        halt(400000);
                        BG96_Listen(BG_UART);
                    }
                    /*Condition prevents accessing undefined values of cfg_periods */
                    else if (selector < 138) {
                        send_period_sec = cfg_periods[selector & 0x0F];
                        period = cfg_periods[selector & 0x0F];
                        xQueueOverwrite(xReportPeriodQueue, (void*)&period);
                    }
                }
                else{
                    VitaluzIoT_Write(selector, &relay[0], &relay[1], &valve);
                }
                for(uint8_t i = 0; i < strlen(cmd); i++)
                    cmd[i] = 0;
                for(uint8_t i = 0; i < strlen(rx_buffer); i++)
                    rx_buffer[i] = 0;
             }
             /*End of condition*/
             USART_Deinit(BG_UART);
             halt(1000);
             BG96_Listen(BG_UART);
             timeout = 0;
             xQueueOverwrite(xServerTimeoutQueue, (void*)&timeout);
             xSemaphoreGive(xUsartMux);

         }

         xQueuePeek(xServerTimeoutQueue, &timeout, (TickType_t)10);
         ///*Reset server socket to prevent timeout and hence disconnection
         if(timeout > (2 * 60)){
             xSemaphoreTake(xUsartMux,(TickType_t) 10  );
             err = VitaluzIoT_RefreshServer(&pdp, &server_socket);
             if(err == CONNECTION_ERROR)
            	 err_count++;
             else
            	 err_count = 0;
             if(err_count > 4)
            	 vTaskResume(xResetHandle);

             timeout = 0;
             xQueueOverwrite(xServerTimeoutQueue, (void*)&timeout);
             BG96_Listen(BG_UART);
             xSemaphoreGive(xUsartMux);
         }
         WWDT->FEED = 0xAA;
         WWDT->FEED = 0x55;
    }

}

void vAlarmTask(void* pV){
    /*Highest priority, suspends immediately, resumed by ISR*/
    vTaskSuspend(NULL);
    for(;;){
        //evaluate how feasible is that the alarm has to wait
        //for ongoing communication with BG96.
        //delays will be present only if there is an ongoing
        //send or reception; and also if another socket is a good idea.
        /*We do not close incoming_socket as it might take up to 10 sec*/
        /*If incoming data arrives at the same time this is executed, it*/
        /*will be discarded*/
        xSemaphoreTake(xUsartMux, (TickType_t) portMAX_DELAY);
        /*We deinit as Alarm is atomic and opens and closes in every cmd*/
        USART_Deinit(BG_UART);
        VitaluzIoT_Alarm(&pdp, &send_socket);
        BG96_Listen(BG_UART);
        xSemaphoreGive(xUsartMux);
        vTaskSuspend(NULL);
    }
}
/*In this task we also keep track of server timeout*/
void vTemperatureTask(void* pV){
    /*Overflow of count doesn't affect our purpose*/
    uint8_t count = 0;
    uint32_t server_timeout = 0;
    float temp = 0;
    /*Temperature fetch requires 1s delay between request and read*/
    for(;;){
        /*Condition that switches between requests and fetch between every call*/
        if(count % 2){
            DS18_RequestConv(&ds18_pin);
        }
        else{
            temp = DS18_ReadTemp(&ds18_pin);
            /*Overwrite as we only want most recent value*/
            xQueueOverwrite(xTempQueue, (void*)&temp);
        }
    /*increment variable to switch cmd in next call*/
    xQueueReceive(xServerTimeoutQueue, (void*)&server_timeout ,(TickType_t)10);
    server_timeout++;
    /*BUG!: count increment was above xQueueReceive(xServer...*/
    /*but it would not increment and hence would not update temp.*/
    /*count moved below increments correctly*/
    count++;
    xQueueSendToFront(xServerTimeoutQueue, (void*)&server_timeout, (TickType_t)10);
    vTaskDelay(pdMS_TO_TICKS(1000));
    }

}

void vResetTask(void* pV){


    /*Suspend Task on creation*/
    vTaskSuspend(NULL);
    for(;;){
        /*Delete every task? do we need to do it*/
        WWDT_Init(WWDT, &wwdt_cfg);
        /*Reset device via WatchDog*/
        WWDT->FEED = 0xAA;
        WWDT->FEED = 0xBB;

    }
}
