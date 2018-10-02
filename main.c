 /*
 * @file    main.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MKL25Z4.h"
#include "fsl_debug_console.h"
/* TODO: insert other include files here. */
#include "Schdlr_SAB.h"

static void svfnRedLedTask(void *vpTaskParams);
static void svfnGreenLedTask(void *vpTaskParams);
static void svfnCounterTask(void *vpTaskParams);

/* TODO: insert other definitions and declarations here. */

/*
 * @brief   Application entry point.
 */
int main(void) {

	static uint32_t uwRedLedStack[TASK_STACK_SIZE];
	static uint32_t uwGreenLedStack[TASK_STACK_SIZE];
	static uint32_t uwCounter1Stack[TASK_STACK_SIZE];

	uint32_t StatusDebug = 0;
  	/* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
  	/* Init FSL debug console. */
    BOARD_InitDebugConsole();

    //PRINTF("Hello World\n");
    Schdlr_xfnInit();
    StatusDebug = Schdlr_xfnTaskCreate(svfnRedLedTask, NULL, uwRedLedStack, sizeof(uwRedLedStack), Task_Priority_4);
    StatusDebug = Schdlr_xfnTaskCreate(svfnGreenLedTask, NULL, uwGreenLedStack, sizeof(uwGreenLedStack), Task_Priority_31);
    StatusDebug = Schdlr_xfnTaskCreate(svfnCounterTask, NULL, uwCounter1Stack, sizeof(uwCounter1Stack), Task_Priority_0);
    if(StatusDebug)
    {
    	Schdlr_xfnStart();
    }
    while(1);

    return 0;
}

static void svfnRedLedTask(void *vpTaskParams)
{
	LED_RED_INIT(LOGIC_LED_OFF);
	while(1)
	{
		LED_RED_TOGGLE();
		Schdlr_xfnTaskBlock(Task_Priority_4);
	}
}

static void svfnGreenLedTask(void *vpTaskParams)
{
	LED_GREEN_INIT(LOGIC_LED_OFF);
	while(1)
	{
		LED_GREEN_TOGGLE();
		Schdlr_xfnTaskYield();
	}
}

static void svfnCounterTask(void *vpTaskParams)
{
	uint32_t uwCounter = 0;
	while(1)
	{
		uwCounter++;
		Schdlr_xfnTaskBlock(Task_Priority_31);
	}
}
