/*******************************************************************************
Copyright (C) Marvell International Ltd. and its affiliates

This software file (the "File") is owned and distributed by Marvell
International Ltd. and/or its affiliates ("Marvell") under the following
alternative licensing terms.  Once you have made an election to distribute the
File under one of the following license alternatives, please (i) delete this
introductory statement regarding license alternatives, (ii) delete the two
license alternatives that you have not elected to use and (iii) preserve the
Marvell copyright notice above.

********************************************************************************
Marvell Commercial License Option

If you received this File from Marvell and you have entered into a commercial
license agreement (a "Commercial License") with Marvell, the File is licensed
to you under the terms of the applicable Commercial License.

********************************************************************************
Marvell GPL License Option

If you received this File from Marvell, you may opt to use, redistribute and/or
modify this File in accordance with the terms and conditions of the General
Public License Version 2, June 1991 (the "GPL License"), a copy of which is
available along with the File in the license.txt file or by writing to the Free
Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 or
on the worldwide web at http://www.gnu.org/licenses/gpl.txt.

THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE IMPLIED
WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY
DISCLAIMED.  The GPL License provides additional details about this warranty
disclaimer.
********************************************************************************
Marvell BSD License Option

If you received this File from Marvell, you may opt to use, redistribute and/or
modify this File under the following licensing terms.
Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

    *   Redistributions of source code must retain the above copyright notice,
	this list of conditions and the following disclaimer.

    *   Redistributions in binary form must reproduce the above copyright
	notice, this list of conditions and the following disclaimer in the
	documentation and/or other materials provided with the distribution.

    *   Neither the name of Marvell nor the names of its contributors may be
	used to endorse or promote products derived from this software without
	specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*******************************************************************************/

#ifndef __INCmvTmrWtdgh
#define __INCmvTmrWtdgh

#ifdef __cplusplus
extern "C" {
#endif

/* includes */
#include "mvOs.h"
#include "cntmr/mvCntmrRegs.h"
#include "ctrlEnv/mvCtrlEnvSpec.h"
#include "mvSysCntmrConfig.h"

typedef struct {
	MV_U16		ctrlModel;
	MV_U16		ctrlRev;
	MV_U32		ctrlFamily;
} MV_CNTMR_HAL_DATA;

/* This enumerator describe counters\watchdog numbers       */
	typedef enum _mvCntmrID {
		TIMER0 = 0,		/* Global counter 0 */
		TIMER1,			/* Global counter 1 */
		TIMER2,			/* Global counter 2 */
		TIMER3,			/* Global counter 3 */
		TIMER4,			/* Global Watchdog 0*/
		TIMER5,			/* CPU0 Timer 0   for A0 this is private CPU timer 0  */
		TIMER6, 		/* CPU0 Timer 1   for A0 this is private CPU timer 1   */
		TIMER7, 		/* CPU0 Watchdog  for A0 this is private CPU WD  */
	} MV_CNTMR_ID;

#define MAX_GLOBAL_TIMER	TIMER4
#define FIRST_PRIVATE_TIMER TIMER5

	typedef enum _mvCntmrRatio {
		MV_RATIO_1  = 0,  /*  0 = 1: Timer tic occurs every source clock        */
		MV_RATIO_2,       /*  1 = 2: Timer tic occurs every 2 source clocks     */
		MV_RATIO_4,       /*  2 = 4: Timer tic occurs every 4 source clocks     */
		MV_RATIO_8,       /*  3 = 8: Timer tic occurs every 8 source clocks     */
		MV_RATIO_16,      /*  4 = 16: Timer tic occurs every 16 source clocks   */
		MV_RATIO_32,      /*  5 = 32: Timer tic occurs every 32 source clocks   */
		MV_RATIO_64,      /*  6 = 64: Timer tic occurs every 64 source clocks   */
		MV_RATIO_128      /*  7 = 128: Timer tic occurs every 128 source clocks */
	} MV_CNTMR_RATIO_ID;

/* Counter / Timer control structure */
	typedef struct _mvCntmrCtrl {
		MV_BOOL enable;	/* enable */
		MV_BOOL autoEnable;	/* counter/Timer  */

		MV_CNTMR_RATIO_ID	Ratio;
		MV_BOOL enable_25Mhz;	/* enable timer count frequency is to 25Mhz*/

	} MV_CNTMR_CTRL;

/* Functions */

	MV_STATUS   mvCntmrHalInit(MV_CNTMR_HAL_DATA *halData);

/* Load an init Value to a given counter/timer */
	MV_STATUS mvCntmrLoad(MV_U32 countNum, MV_U32 value);

/* Returns the value of the given Counter/Timer */
	MV_U32 mvCntmrRead(MV_U32 countNum);

/* Returns 0xffffffff minus the value of the given Counter/Timer */
	MV_U32 mvCntmrReadDiff(MV_U32 countNum);

/* Write a value of the given Counter/Timer */
	void mvCntmrWrite(MV_U32 countNum, MV_U32 countVal);

/* Set the Control to a given counter/timer */
	MV_STATUS mvCntmrCtrlSet(MV_U32 countNum, MV_CNTMR_CTRL *pCtrl);

/* Get the value of a given counter/timer */
	MV_STATUS mvCntmrCtrlGet(MV_U32 countNum, MV_CNTMR_CTRL *pCtrl);

/* Set the Enable-Bit to logic '1' ==> starting the counter. */
	MV_STATUS mvCntmrEnable(MV_U32 countNum);

/* Stop the counter/timer running, and returns its Value. */
	MV_STATUS mvCntmrDisable(MV_U32 countNum);

/* Combined all the sub-operations above to one function: Load,setMode,Enable */
	MV_STATUS mvCntmrStart(MV_U32 countNum, MV_U32 value, MV_CNTMR_CTRL *pCtrl);

/*	Clear an Counter/Timer interrupt (Ack) */
	MV_STATUS mvCntmrIntClear(MV_U32 cntmrNum);

/*	get Counter/Timer Frequency */
	MV_U32 mvCntmrFrqGet(MV_U32 cntmrNum);


#ifdef __cplusplus
}
#endif
#endif				/* __INCmvTmrWtdgh */
