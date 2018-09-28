/*
 * SAB_OS.h
 *
 *  Created on: 19/09/2018
 *      Author: Sergio Amador Benet
 */

#ifndef SCHDLR_SAB_H_
#define SCHDLR_SAB_H_

#include <string.h>
#include "MKL25Z4.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define SCHDLR_CONFIG_MAX_TASKS	32
#define TASK_STACK_SIZE			256

/*! @brief  Schdlr status return codes. */
typedef enum{
	Schdlr_Ret_False = 0,
	Schdlr_Ret_True = 1
}SchdlrRetStatus_t;

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
									   SchdlrPriority uwPriority);
SchdlrRetStatus_t Schdlr_xfnTaskCreateBlocked(void (*vpfnhandler)(void *vpParams),
									   void *vpTaskParams,
									   uint32_t *uwpStack,
									   uint32_t uwStackSize,
									   SchdlrPriority uwPriority);
void Schdlr_xfnTaskYield(void);


#endif /* SCHDLR_SAB_H_ */
