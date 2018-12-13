/* Includes ------------------------------------------------------------------*/
#include <Platform/platform.h>
#include <FreeRTOS/FreeRTOS.h>

#include <Platform/Drivers/leds.h>

__attribute__((noreturn)) int main(void)
{

 	platformInit();

 	ledsTest();

	vTaskStartScheduler();

	/* We should never get here as control is now taken by the scheduler */
	for(;;)
		;
}

/****END OF FILE****/
