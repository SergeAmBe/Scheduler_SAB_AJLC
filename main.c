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

/* TODO: insert other definitions and declarations here. */

/*
 * @brief   Application entry point.
 */
int main(void) {

	static uint32_t uwRedLedStack[TASK_STACK_SIZE];
	static uint32_t uwGreenLedStack[TASK_STACK_SIZE];
  	/* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
  	/* Init FSL debug console. */
    BOARD_InitDebugConsole();

    //PRINTF("Hello World\n");
    Schdlr_xfnInit();
    Schdlr_xfnTaskCreate(svfnRedLedTask, NULL, uwRedLedStack, sizeof(uwRedLedStack), 1);
    Schdlr_xfnTaskCreate(svfnGreenLedTask, NULL, uwGreenLedStack, sizeof(uwGreenLedStack), 2);
    Schdlr_xfnStart();

    while(1);

    return 0;
}

static void svfnRedLedTask(void *vpTaskParams)
{
	LED_RED_INIT(LOGIC_LED_OFF);
	while(1)
	{
		LED_RED_TOGGLE();
		Schdlr_xfnTaskYield();
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
