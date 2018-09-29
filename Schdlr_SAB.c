/*
 * SAB_OS.c
 *
 *  Created on: 19/09/2018
 *      Author: Sergio Amador Benet
 */

#include <string.h>
#include "Schdlr_SAB.h"
#include "MKL25Z4.h"

/*******************************************************************************
* Prototypes
******************************************************************************/
void Schdlr_xfnSchdlr(void);

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*!
 * @brief Tasks current status
 */
enum SchdlrTaskStatus{
	Schdlr_Task_Status_Ready = 1,
	Schdlr_Task_Status_Runnig,
	Schdlr_Task_Status_Blocked
};

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*!
 * @brief Schdlr States, only purpose is to use Schdlr function correctly
 */
enum{
	Schdlr_State_Default = 1,
	Schdlr_State_Initialized,
	Schdlr_State_Tasks_Initialized,
	Schdlr_State_Started
}SchdlrState = Schdlr_State_Default;

/*!
 * @brief All the parameters that a task needs to exist and work properly,
 * It's the structure of the Task Control Descriptor
 */
struct xTaskCntrlDescriptor{
	volatile uint32_t uwSP;
	void (*handler)(void *vpParams);
	void *pParams;
	enum SchdlrTaskStatus xStatus;
	SchdlrPriority uwTaskPriorty;
};

/*!
 * @brief Scheduler management structure
 */
static struct{
	struct xTaskCntrlDescriptor xaTasks[SCHDLR_CONFIG_MAX_TASKS];
	uint32_t uwCurrentTask;
	uint32_t uwSize;
}xSchdlrQueue;

volatile struct xTaskCntrlDescriptor *xpSchdlrCurrTask;
volatile struct xTaskCntrlDescriptor *xpSchdlrNextTask;

/*******************************************************************************
 * Codes
 ******************************************************************************/

/*!
 * @brief It Initiates the Schdlr by clearing the TCD memory and
 * sets the schdlr status to init so other schdlr function can do their
 * job properly.
 *
 * @param none
 *
 * @return Returns a state if initiated its job (true) or not (false).
 */
SchdlrRetStatus_t Schdlr_xfnInit(void)
{
	if(SchdlrState != Schdlr_State_Default)
	{
		return Schdlr_Ret_False;
	}
	memset(&xSchdlrQueue, 0, sizeof(xSchdlrQueue));
	SchdlrState = Schdlr_State_Initialized;
	return Schdlr_Ret_True;
}

/*!
 * @brief Creates a Schdlr Task in ready state
 *
 * @param Pointer to task function to be created
 * @param A parameter that the task is going to use (char, int, struct, etc)
 * @param A pointer to stack array (usually a char array)
 * @param The size of the stack array.
 * @param Priority of the task
 *
 * @return True if task was created successfully or false otherwise
 */
SchdlrRetStatus_t Schdlr_xfnTaskCreate(void (*vpfnhandler)(void *vpParams),
									   void *vpTaskParams,
									   uint32_t *uwpStack,
									   uint32_t uwStackSize,
									   SchdlrPriority uwPriority)
{
	if(SchdlrState != Schdlr_State_Initialized && SchdlrState != Schdlr_State_Tasks_Initialized)
	{
		return Schdlr_Ret_False;
	}
	if(xSchdlrQueue.uwSize >= SCHDLR_CONFIG_MAX_TASKS - 1)
	{
		return Schdlr_Ret_False;
	}
	if(uwStackSize % sizeof(uint32_t) != 0)
	{
		return Schdlr_Ret_False;
	}

	uint32_t uwStackOffset = (uwStackSize/sizeof(uint32_t));

	struct xTaskCntrlDescriptor *pxTask = &xSchdlrQueue.xaTasks[xSchdlrQueue.uwSize];
	pxTask->handler = vpfnhandler;
	pxTask->pParams = vpTaskParams;
	pxTask->uwSP = (uint32_t)(uwpStack + uwStackOffset - 16);
	pxTask->xStatus = Schdlr_Task_Status_Ready;
	pxTask->uwTaskPriorty = uwPriority;

	SchdlrState = Schdlr_State_Tasks_Initialized;
	xSchdlrQueue.uwSize++;

	return Schdlr_Ret_True;
}

/*!
 * @brief Creates a Schdlr Task in blocked state
 *
 * @param Pointer to task function to be created
 * @param A parameter that the task is going to use (char, int, struct, etc)
 * @param A pointer to stack array (usually a char array)
 * @param The size of the stack array.
 * @param Priority of the task
 *
 * @return True if task was created successfully or false otherwise
 */
SchdlrRetStatus_t Schdlr_xfnTaskCreateBlocked(void (*vpfnhandler)(void *vpParams),
									   void *vpTaskParams,
									   uint32_t *uwpStack,
									   uint32_t uwStackSize,
									   SchdlrPriority uwPriority)
{
	if(SchdlrState != Schdlr_State_Initialized && SchdlrState != Schdlr_State_Tasks_Initialized)
	{
		return Schdlr_Ret_False;
	}
	if(xSchdlrQueue.uwSize >= SCHDLR_CONFIG_MAX_TASKS - 1)
	{
		return Schdlr_Ret_False;
	}
	if(uwStackSize % sizeof(uint32_t) != 0)
	{
		return Schdlr_Ret_False;
	}

	uint32_t uwStackOffset = (uwStackSize/sizeof(uint32_t));

	struct xTaskCntrlDescriptor *pxTask = &xSchdlrQueue.xaTasks[xSchdlrQueue.uwSize];
	pxTask->handler = vpfnhandler;
	pxTask->pParams = vpTaskParams;
	pxTask->uwSP = (uint32_t)(uwpStack + uwStackOffset - 16);
	pxTask->xStatus = Schdlr_Task_Status_Blocked;
	pxTask->uwTaskPriorty = uwPriority;

	SchdlrState = Schdlr_State_Tasks_Initialized;
	xSchdlrQueue.uwSize++;

	return Schdlr_Ret_True;
}

/*!
 * @brief Starts the Schdlr with all the tasks created
 *
 * @param None
 *
 * @return True if Schdlr started successfully or false otherwise
 */
SchdlrRetStatus_t Schdlr_xfnStart(void)
{
	if(SchdlrState != Schdlr_State_Tasks_Initialized)
	{
		return Schdlr_Ret_False;
	}

	//NVIC_SetPriority(PendSV_IRQn, 0xff); /* Lowest possible priority */

	xpSchdlrCurrTask = &xSchdlrQueue.xaTasks[xSchdlrQueue.uwCurrentTask];
	SchdlrState = Schdlr_State_Started;

	__set_PSP(xpSchdlrCurrTask->uwSP+64);
	__set_CONTROL(0x03);
	__ISB();

	xpSchdlrCurrTask->handler(xpSchdlrCurrTask->pParams);

	return Schdlr_Ret_True;
}

/*!
 * @brief This function manages the task queue and indicates what task is
 * next to run, its a FIFO.
 *
 * @param None
 *
 * @return None
 */
void Schdlr_xfnSchdlr(void)
{
	xpSchdlrCurrTask = &xSchdlrQueue.xaTasks[xSchdlrQueue.uwCurrentTask];
	xpSchdlrCurrTask->xStatus = Schdlr_Task_Status_Ready;

	xSchdlrQueue.uwCurrentTask++;
	if(xSchdlrQueue.uwCurrentTask >= xSchdlrQueue.uwSize)
	{
		xSchdlrQueue.uwCurrentTask = 0;
	}
	xpSchdlrNextTask = &xSchdlrQueue.xaTasks[xSchdlrQueue.uwCurrentTask];
	xpSchdlrNextTask->xStatus = Schdlr_Task_Status_Runnig;
}

/*!
 * @brief This function calls the scheduler to see which is the next task to run
 * and forces a Context Switch.
 *
 * @param None
 *
 * @return None
 */
#define SCB_ICSR_PENDSV_ADDRESS 0xe000ed04
volatile uint32_t * ScbIcsr = (volatile uint32_t *)SCB_ICSR_PENDSV_ADDRESS;
void Schdlr_xfnTaskYield(void)
{
	Schdlr_xfnSchdlr();
	*ScbIcsr |= SCB_ICSR_PENDSVSET_Msk;
	//SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
}

/*!
 * @brief This ISR handles the Context Switch from the current task to the next.
 * Naked attribute so that it does not mess the current task proper context.
 *
 * @param None
 *
 * @return None
 */
__attribute__ (( naked )) void PendSV_Handler(void)
{
	__asm volatile
	(
		"	.syntax unified						\n"
		"	cpsid	i							\n"
		"	push 	{r4-r7}						\n"
		"	mov		r4,r8						\n"
		"   mov		r5,r9						\n"
		"   mov		r6,r10						\n"
		"   mov		r7,r11						\n"
		"   push 	{r4-r7}						\n"
		"   mrs 	r0,psp						\n"
		"   ldr		r2,=AsmxpSchdlrCurrTask		\n"
		"   ldr		r1,[r2]						\n"
		"   str		r0,[r1]						\n"
		"   ldr		r2,=AsmxpSchdlrNextTask		\n"
		"   ldr		r1,[r2]						\n"
		"   ldr		r0,[r1]						\n"
		"   msr		psp,r0						\n"
		"   pop		{r4-r7}						\n"
		"	mov		r8,r4						\n"
		"   mov		r9,r5						\n"
		"   mov		r10,r6						\n"
		"   mov		r11,r7						\n"
		"   pop		{r4-r7}						\n"
		"   ldr		r0, =0xFFFFFFFD				\n"
		"   cpsie	i							\n"
		"   bx		r0							\n"
		"	.align 4							\n"
		"	AsmxpSchdlrCurrTask: .word xpSchdlrCurrTask	\n"
		"	.align 4							\n"
		"	AsmxpSchdlrNextTask: .word xpSchdlrNextTask	\n"
	);
}
