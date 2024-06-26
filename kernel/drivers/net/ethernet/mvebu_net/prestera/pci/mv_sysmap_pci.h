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
********************************************************************************
* mv_sysmap_pci.h
*
* DESCRIPTION:
*       Includes defines needed by the PCI PP device driver
*
* DEPENDENCIES:
*       None.
*
*******************************************************************************/
#ifndef __MV_SYSMAP_PCI_H
#define __MV_SYSMAP_PCI_H

/* win nr for BAR2 */
#define DFXW				1
#define SCPUW				2
#define DITCMW				3
#define DDTCMW				4
#define SCPUDRAMW			5

#define DRAGONITE_DTCM_OFFSET		0x04000000

/* common BAR 2 sizes */
#define DFX_SIZE			_1M
#define SCPUDRAM_SIZE			_16M
#define ITCM_SIZE			_64K
#define DTCM_SIZE			_64K

/* common BAR 2 bases */
#define DFX_BASE			0x0
#define SCPU_BASE			(DFX_BASE + DFX_SIZE + _1M)/* keep aligned to the window size */
#define SCPUDRAM_BASE			(_16M)/* keep aligned to the window size */

/* AC3 BAR 2 sizes */
#define SCPU_SIZE_AC3			_2M

/* AC3 BAR 2 bases */
#define ITCM_BASE_AC3			(SCPU_BASE + SCPU_SIZE_AC3)
#define DTCM_BASE_AC3			(ITCM_BASE_AC3 + ITCM_SIZE)

/* BC2 BAR 2 sizes */
#define SCPU_SIZE_BC2			_2M

/* BOBK BAR 2 sizes */
#define SCPU_SIZE_BOBK			_512K

/* BOBK BAR 2 bases */
#define ITCM_BASE_BOBK			(SCPU_BASE + SCPU_SIZE_BOBK)
#define DTCM_BASE_BOBK			(ITCM_BASE_BOBK + ITCM_SIZE)

/* ALDRIN-A0 BAR 2 bases */
#define ITCM_BASE_ALDR			(DFX_BASE + DFX_SIZE)
#define DTCM_BASE_ALDR			(ITCM_BASE_ALDR + ITCM_SIZE)

#endif /* __MV_SYSMAP_PCI_H */
