/*
 * SAB_OS.c
 *
 *  Created on: 19/09/2018
 *      Author: Sergio Amador Benet
 */

#include "Schdlr_SAB.h"

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void Schdlr_xfnSchdlr(void);
uint32_t Schdlr_uwfnFindFirstSet(uint32_t uwTaskPriorty);
void PendSV_Handler(void);
void SysTick_Handler(void);
static void Schdlr_vfnTaskFinished(void);
void Schdlr_vfnTaskIdle(void *vpTaskParams);
/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define ALL_STATUS_BITS_MASK 	(0xFFFFFFFF)
#define ERROR_TASKPRIORITY		(0xFFFFFFFF)
#define HIGHEST_TIME			(0xFFFFFFFF)
#define XPSR_DEFAULT_MASK 		(0x01000000)
#define IDLE_TASK_NO			31
/*******************************************************************************
 * Variables
 ******************************************************************************/
/*!
 * @brief Schdlr States, only purpose is to use Schdlr function correctly
 */
enum {
	Schdlr_State_Default = 1,
	Schdlr_State_Initialized,
	Schdlr_State_Tasks_Initialized,
	Schdlr_State_Started
} SchdlrState = Schdlr_State_Default;

/*!
 * @brief All the parameters that a task needs to exist and work properly,
 * It's the structure of the Task Control Descriptor
 */
struct xTaskCntrlDescriptor {
	volatile uint32_t uwSP;
	void (*handler)(void *vpParams);
	void *pParams;
};

/*!
 * @brief Scheduler management structure
 */
static struct {
	struct xTaskCntrlDescriptor xaTasks[SCHDLR_CONFIG_MAX_TASKS];
	volatile uint32_t uwCurrentTask;
	volatile uint32_t uwTaskPriorty;
} xSchdlrQueue;

/*!
 * @brief Pointers to the Current Task and the Next Task
 */
volatile struct xTaskCntrlDescriptor *xpSchdlrCurrTask;
volatile struct xTaskCntrlDescriptor *xpSchdlrNextTask;

static uint32_t uwaIdleTaskStack[TASK_STACK_SIZE];

/*!
 * @brief Software Timer for Delay by SysTick
 */
#ifdef	SYSTICK_FEAT
volatile static uint32_t uwaDelayTimer[MAX_TIMERS];
#endif
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
	if (SchdlrState != Schdlr_State_Default)
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
		void *vpTaskParams, uint32_t *uwpStack, uint32_t uwStackSize, uint32_t uwPriority)
{
	if (SchdlrState != Schdlr_State_Initialized && SchdlrState != Schdlr_State_Tasks_Initialized)
	{/* Schdlr must be initialized before calling this function Or Still creating Tasks */
		return Schdlr_Ret_False;
	}
	if(uwPriority & xSchdlrQueue.uwTaskPriorty)
	{/* There cannot be tasks with the same priority */
		return Schdlr_Ret_False;
	}
	if (uwStackSize % sizeof(uint32_t) != 0)
	{/* Stack size must 4 byte aligned */
		return Schdlr_Ret_False;
	}

	uint32_t uwStackOffset = (uwStackSize / sizeof(uint32_t));
	xSchdlrQueue.uwCurrentTask = Schdlr_uwfnFindFirstSet(uwPriority);
	struct xTaskCntrlDescriptor *pxTask = &xSchdlrQueue.xaTasks[xSchdlrQueue.uwCurrentTask];
	pxTask->handler = vpfnhandler;
	pxTask->pParams = vpTaskParams;
	pxTask->uwSP = (uint32_t) (uwpStack + uwStackOffset - 16);
	xSchdlrQueue.uwTaskPriorty |= (uwPriority);

	uwpStack[uwStackOffset - 1] = XPSR_DEFAULT_MASK;
	uwpStack[uwStackOffset - 2] = (uint32_t)vpfnhandler;
	uwpStack[uwStackOffset - 3] = (uint32_t)&Schdlr_vfnTaskFinished;
	uwpStack[uwStackOffset - 8] = (uint32_t)vpTaskParams;

	SchdlrState = Schdlr_State_Tasks_Initialized;

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
SchdlrRetStatus_t Schdlr_xfnTaskCreateBlocked(void (*vpfnhandler)(void *vpParams), void *vpTaskParams,
											uint32_t *uwpStack, uint32_t uwStackSize, uint32_t uwPriority)
{
	if (SchdlrState != Schdlr_State_Initialized && SchdlrState != Schdlr_State_Tasks_Initialized)
	{/* Schdlr must be initialized before calling this function Or Still creating Tasks */
		return Schdlr_Ret_False;
	}
	if(uwPriority & xSchdlrQueue.uwTaskPriorty)
	{/* There cannot be tasks with the same priority */
		return Schdlr_Ret_False;
	}
	if (uwStackSize % sizeof(uint32_t) != 0)
	{/* Stack size must 4 byte aligned */
		return Schdlr_Ret_False;
	}

	uint32_t uwStackOffset = (uwStackSize / sizeof(uint32_t));

	xSchdlrQueue.uwCurrentTask = Schdlr_uwfnFindFirstSet(uwPriority);
	struct xTaskCntrlDescriptor *pxTask = &xSchdlrQueue.xaTasks[xSchdlrQueue.uwCurrentTask];
	pxTask->handler = vpfnhandler;
	pxTask->pParams = vpTaskParams;
	pxTask->uwSP = (uint32_t) (uwpStack + uwStackOffset - 16);
	xSchdlrQueue.uwTaskPriorty &= ~(uwPriority);

	uwpStack[uwStackOffset - 1] = XPSR_DEFAULT_MASK;
	uwpStack[uwStackOffset - 2] = (uint32_t)vpfnhandler;
	uwpStack[uwStackOffset - 3] = (uint32_t)&Schdlr_vfnTaskFinished;
	uwpStack[uwStackOffset - 8] = (uint32_t)vpTaskParams;

	SchdlrState = Schdlr_State_Tasks_Initialized;

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
	if (SchdlrState != Schdlr_State_Tasks_Initialized)
	{
		return Schdlr_Ret_False;
	}

	Schdlr_xfnTaskCreate(Schdlr_vfnTaskIdle, NULL, uwaIdleTaskStack, sizeof(uwaIdleTaskStack), Task_Priority_31);
	NVIC_SetPriority(PendSV_IRQn, 0x3);

#ifdef SYSTICK_FEAT
	SysTick_Config(SystemCoreClock / SYSTICK_FREQ);
	NVIC_SetPriority(SysTick_IRQn, 0x0);
#endif

	xSchdlrQueue.uwCurrentTask = Schdlr_uwfnFindFirstSet(xSchdlrQueue.uwTaskPriorty);
	xpSchdlrCurrTask = &xSchdlrQueue.xaTasks[xSchdlrQueue.uwCurrentTask];
	SchdlrState = Schdlr_State_Started;

	__set_PSP(xpSchdlrCurrTask->uwSP + 64);
	//__set_CONTROL(0x03);
	__set_CONTROL(0x02);
	__ISB();

	xpSchdlrCurrTask->handler(xpSchdlrCurrTask->pParams);

	return Schdlr_Ret_True;
}

/*!
 * @brief This function calls the scheduler to see which is the next task to run
 * and forces a Context Switch.
 *
 * @param None
 *
 * @return None
 */
//#define SCB_ICSR_PENDSV_ADDRESS 0xe000ed04
//volatile uint32_t * ScbIcsr = (volatile uint32_t *)SCB_ICSR_PENDSV_ADDRESS;
void Schdlr_xfnTaskYield(void)
{
	//__set_CONTROL(0x02);
	SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
	//__set_CONTROL(0x03);
}

/*!
 * @brief It blocks a task by clearing its bit mask
 *
 * @param None
 *
 * @return None
 */
void Schdlr_xfnTaskBlocked(void)
{
	uint32_t uwTaskMask;
	uwTaskMask = Schdlr_uwfnGetTaskMask(xSchdlrQueue.uwCurrentTask);
	xSchdlrQueue.uwTaskPriorty &= ~(uwTaskMask);
	Schdlr_xfnTaskYield();
}

/*!
 * @brief It readies a task by setting its bit mask
 *
 * @param The task to ready using TaskPriority_t mask
 *
 * @return None
 */
void Schdlr_xfnTaskReady(uint32_t uwTaskToReady)
{
	xSchdlrQueue.uwTaskPriorty |= (uwTaskToReady);
}

/*!
 * @brief This function manages the task queue and indicates what task is
 * next to run, priority based.
 *
 * @param None
 *
 * @return None
 */
static void Schdlr_xfnSchdlr(void)
{
	xpSchdlrCurrTask = &xSchdlrQueue.xaTasks[xSchdlrQueue.uwCurrentTask];
	if((xSchdlrQueue.uwTaskPriorty & ALL_STATUS_BITS_MASK) == Task_Priority_31)
	{
		xpSchdlrNextTask = &xSchdlrQueue.xaTasks[IDLE_TASK_NO];
	}else
	{
		xSchdlrQueue.uwCurrentTask = Schdlr_uwfnFindFirstSet(xSchdlrQueue.uwTaskPriorty);
		xpSchdlrNextTask = &xSchdlrQueue.xaTasks[xSchdlrQueue.uwCurrentTask];
	}
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
//void PendSV_Handler(void)
{
	__asm volatile
	(
			"	.syntax unified						\n"
			"	cpsid 	i							\n"
			"	mrs		r0, psp						\n"
			"	subs	r0, #16						\n"
			"	stmia	r0!,{r4-r7}					\n"
			"	mov		r4,r8						\n"
			"   mov		r5,r9						\n"
			"   mov		r6,r10						\n"
			"   mov		r7,r11						\n"
			"	subs	r0, #32						\n"
			"	stmia	r0!,{r4-r7}					\n"
			"	subs	r0, #16						\n");
			Schdlr_xfnSchdlr();
	__asm volatile
		(
			"   ldr		r2,=xpSchdlrCurrTask		\n"
			"   ldr		r1,[r2]						\n"
			"   str		r0,[r1]						\n"
			"   ldr		r2,=xpSchdlrNextTask		\n"
			"   ldr		r1,[r2]						\n"
			"   ldr		r0,[r1]						\n"
			"	ldmia	r0!,{r4-r7}					\n"
			"	mov		r8,r4						\n"
			"   mov		r9,r5						\n"
			"   mov		r10,r6						\n"
			"   mov		r11,r7						\n"
			"	ldmia	r0!,{r4-r7}					\n"
			"	msr		psp, r0						\n"
			"   ldr		r0, =0xFFFFFFFD				\n"
			"   cpsie	i							\n"
			"	isb									\n"
			"   bx		r0							"
	);
}

/*!
 * @brief Finds the first LSB that reference to task priority(1),
 * also works for general purpose FindFirstSet algorithm
 *
 * @param 32bit variable to find first set
 *
 * @return Counter to the first set bit
 */
uint32_t Schdlr_uwfnFindFirstSet(uint32_t uwTaskPriorty)
{
	if (!uwTaskPriorty)
	{
		return ERROR_TASKPRIORITY;
	}
	uint32_t uwTempMask = Task_Priority_0;
	uint32_t uwReturnNxtTask = 0;
	while (!(uwTaskPriorty & uwTempMask))
	{
		uwTempMask <<= 1;
		uwReturnNxtTask++;
	}
	return uwReturnNxtTask;
}

/*!
 * @brief Gets the Task Mask by its number in the Task array
 *
 * @param Number of the Task in the xSchdlrQueue.xaTasks[] array
 *
 * @return The mask representing its priority (xSchdlrQueue.uwTaskPriorty) of the task
 * @return ERROR_TASKPRIORITY if the taskNum is greater than the Max task allowed
 */
uint32_t Schdlr_uwfnGetTaskMask(uint32_t uwTaskNumber)
{
	if (uwTaskNumber > (SCHDLR_CONFIG_MAX_TASKS - 1))
	{
		return ERROR_TASKPRIORITY;
	}
	uint32_t uwTempMask = 1;
	while (uwTaskNumber--)
	{
		uwTempMask <<= 1;
	}
	return uwTempMask;
}

/*!
 * @brief Blocks a Task for uwDelay_ms in (1/SYSTICK_FREQ)(5msecs),
 * the time you want to block is uwDelay_ms * (1/SYSTICK_FREQ).
 *
 * @param Number of milisecs to block that task
 *
 * @return none
 */
void Schdlr_vfnTaskDelay(uint32_t uwDelay_ms)
{
	uint32_t uwTaskMask;
	uwTaskMask = Schdlr_uwfnGetTaskMask(xSchdlrQueue.uwCurrentTask);
	xSchdlrQueue.uwTaskPriorty &= ~(uwTaskMask);
	uwaDelayTimer[xSchdlrQueue.uwCurrentTask] = uwDelay_ms;
	Schdlr_xfnTaskYield();
}

/*!
 * @brief Schdlr system Tick, handles software timers
 *
 * @param none
 *
 * @return none
 */
void SysTick_Handler(void)
{
	uint8_t ubIndex = 31;
	uint32_t uwTaskMask = Task_Priority_31;
	do
	{
		if(uwaDelayTimer[ubIndex])
		{
			uwaDelayTimer[ubIndex]--;
			if(!uwaDelayTimer[ubIndex])
			{
				Schdlr_xfnTaskReady(uwTaskMask);
			}
		}
		uwTaskMask >>= 1;
	}while(ubIndex--);
	//Schdlr_xfnTaskYield();
}

/*!
 * @brief When every other task is blocked, TaskIdle
 * runs to keep track when APP layer task is ready to run.
 *
 * @param It always takes a NULL pointer
 *
 * @return none
 */
void Schdlr_vfnTaskIdle(void *vpTaskParams)
{
	while(1)
	{
		while((xSchdlrQueue.uwTaskPriorty & ALL_STATUS_BITS_MASK) == Task_Priority_31);
		Schdlr_xfnTaskYield();
	}
}

/*!
 * @brief This function is entered when some task handler returns.
 * It should not happen in any normal application but its here
 * because:
 * 1)Debugging purposes
 * 2)It's the address value of LR of every stack frame of a task.
 *
 * @param none
 *
 * @return none
 */
static void Schdlr_vfnTaskFinished(void)
{
	volatile uint32_t i = 0;
	while (1)
	{
		i++;
	}
}
