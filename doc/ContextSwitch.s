/*
 * ContextSwitch.s
 *
 *  Created on: 28/09/2018
 *      Author: Sergio Amador Benet
 */

/*!
 * @brief This ISR handles the Context Switch from the current task to the next.
 * Naked attribute so that it does not mess the current task proper context.
 *
 * @param None
 *
 * @return None
 */
 /*
.syntax unified
.cpu cortex-m0
.fpu softvfp

.thumb

.global PendSV_Handler
.type PendSV_Handler, %function
PendSV_Handler:

	cpsid	i
	push 	{r4-r7}
	mov		r4,r8
	mov		r5,r9
	mov		r6,r10
	mov		r7,r11
	push 	{r4-r7}
	mrs 	r0,psp
	ldr		r2,=AsmxpSchdlrCurrTask
	ldr		r1,[r2]
	str		r0,[r1]
	ldr		r2,=AsmxpSchdlrNextTask
	ldr		r1,[r2]
	ldr		r0,[r1]
	msr		psp,r0
	pop		{r4-r7}
	mov		r8,r4
	mov		r9,r5
	mov		r10,r6
	mov		r11,r7
	pop		{r4-r7}
	ldr		r0, =0xFFFFFFFD
	cpsie	i
	bx		r0
	.align 4
	AsmxpSchdlrCurrTask: .word xpSchdlrCurrTask
	.align 4
	AsmxpSchdlrNextTask: .word xpSchdlrNextTask

.size PendSV_Handler, .-PendSV_Handler
*/
