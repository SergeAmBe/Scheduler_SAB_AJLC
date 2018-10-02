/*
 * SAB_OS.h
 *
 *  Created on: 19/09/2018
 *      Author: Sergio Amador Benet
 */

#ifndef SCHDLR_SAB_H_
#define SCHDLR_SAB_H_

#include "MKL25Z4.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define SCHDLR_CONFIG_MAX_TASKS	32
#define TASK_STACK_SIZE			64

/*! @brief  Schdlr status return codes. */
typedef enum{
	Schdlr_Ret_False = 0,
	Schdlr_Ret_True = 1
}SchdlrRetStatus_t;

/*! @brief  Use this to set task priority or task status. */
typedef enum TaskPriority{
	Task_Priority_0 = (1<<0),
	Task_Priority_1 = (1<<1),
	Task_Priority_2 = (1<<2),
	Task_Priority_3 = (1<<3),
	Task_Priority_4 = (1<<4),
	Task_Priority_5 = (1<<5),
	Task_Priority_6 = (1<<6),
	Task_Priority_7 = (1<<7),
	Task_Priority_8 = (1<<8),
	Task_Priority_9 = (1<<9),
	Task_Priority_10 = (1<<10),
	Task_Priority_11 = (1<<11),
	Task_Priority_12 = (1<<12),
	Task_Priority_13 = (1<<13),
	Task_Priority_14 = (1<<14),
	Task_Priority_15 = (1<<15),
	Task_Priority_16 = (1<<16),
	Task_Priority_17 = (1<<17),
	Task_Priority_18 = (1<<18),
	Task_Priority_19 = (1<<19),
	Task_Priority_20 = (1<<20),
	Task_Priority_21 = (1<<21),
	Task_Priority_22 = (1<<22),
	Task_Priority_23 = (1<<23),
	Task_Priority_24 = (1<<24),
	Task_Priority_25 = (1<<25),
	Task_Priority_26 = (1<<26),
	Task_Priority_27 = (1<<27),
	Task_Priority_28 = (1<<28),
	Task_Priority_29 = (1<<29),
	Task_Priority_30 = (1<<30),
	Task_Priority_31 = (1<<31)
}TaskPriority_t;

typedef unsigned long SchdlrPriority;
/*******************************************************************************
 * API
 ******************************************************************************/
SchdlrRetStatus_t Schdlr_xfnInit(void);
SchdlrRetStatus_t Schdlr_xfnStart(void);
SchdlrRetStatus_t Schdlr_xfnTaskCreate(void (*vpfnhandler)(void *vpParams),
									   void *vpTaskParams,
									   uint32_t *uwpStack,
									   uint32_t uwStackSize,
									   uint32_t uwPriority);
SchdlrRetStatus_t Schdlr_xfnTaskCreateBlocked(void (*vpfnhandler)(void *vpParams),
									   void *vpTaskParams,
									   uint32_t *uwpStack,
									   uint32_t uwStackSize,
									   uint32_t uwPriority);
void Schdlr_xfnTaskYield(void);
void Schdlr_xfnTaskBlock(uint32_t uwTaskToBlock);

#endif /* SCHDLR_SAB_H_ */
