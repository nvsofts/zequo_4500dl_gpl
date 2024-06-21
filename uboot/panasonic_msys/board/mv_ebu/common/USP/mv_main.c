/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */
/*******************************************************************************
Copyright (C) Marvell International Ltd. and its affiliates

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
*******************************************************************************/

#include <linux/ctype.h>
#include <common.h>
#include "mvTypes.h"
#include "mvBoardEnvLib.h"
#include "mvCpuIf.h"
#include "mvCtrlEnvLib.h"
#include "mvDebug.h"
#include "device/mvDevice.h"
#include "twsi/mvTwsi.h"
#include "ctrlEnv/mvCtrlEnvLib.h"
#include "eth-phy/mvEthPhyXsmi.h"
#if defined(MV_ETH_LEGACY)
#include "eth/mvEth.h"
#include "mv_eth_legacy.h"
#elif defined(MV_ETH_NETA)
#include "neta/gbe/mvNeta.h"
#include "mv_egiga_neta.h"
#else
#include "pp2/gbe/mvPp2Gbe.h"
#include "mv_egiga_pp2.h"
#endif /* MV_ETH_LEGACY or MV_ETH_NETA */
#include "pex/mvPex.h"
#include "gpp/mvGpp.h"
#include "gpp/mvGppRegs.h"
#include "mv_phy.h"
#include "ddr2_3/mvDramIfRegs.h"
#ifdef MV_INCLUDE_RTC
#include "rtc/integ_rtc/mvRtc.h"
#include "rtc.h"
#elif CONFIG_RTC_DS1338_DS1339
#include "rtc/ext_rtc/mvDS133x.h"
#endif
#include "mvSysCntmrApi.h"
#if defined(MV_INCLUDE_XOR)
#include "xor/mvXor.h"
#include "mvSysXorApi.h"
#endif
#if defined(MV_INCLUDE_IDMA)
#include "sys/mvSysIdma.h"
#include "idma/mvIdma.h"
#endif
#if defined(MV_INCLUDE_USB)
#include "usb/mvUsb.h"
#include "mvSysUsbApi.h"
#endif
#ifdef CONFIG_AMP_SUPPORT
#include "mv_amp.h"
#endif
#include "cpu/mvCpu.h"
#include "nand.h"
#include "spi_flash.h"
#ifdef CONFIG_PCI
#include <pci.h>
#endif
#include <net.h>
#include <netdev.h>
#include <command.h>
#include "mvCommon.h"
#include "uart/mvUart.h"
#include "ctrlEnv/mvCtrlEnvAddrDec.h"

#ifdef CONFIG_ROL
#include "mvRol.h"
#endif

/* #define MV_DEBUG */
#ifdef MV_MAIN_DEBUG
#define DB(x) x
#else
#define DB(x)
#endif

extern MV_TARGET_ATTRIB *mvTargetDefaultsArray;
extern MV_TARGET_ATTRIB mvTargetMsysArray[];
extern MV_TARGET_ATTRIB mvTargetAxpArray[];


extern int display_dram_config(int print);
int late_print_cpuinfo(void);
/* CPU address decode table. */
MV_CPU_DEC_WIN mvMsysWinMap[] = MSYS_WIN_MAP_TBL;
MV_CPU_DEC_WIN mvAxpWinMap[] = AXP_WIN_MAP_TBL;
#if defined(CONFIG_CMD_RCVR)
extern void recoveryDetection(void);
#endif
void mv_cpu_init(void);
#if defined(MV_INCLUDE_CLK_PWR_CNTRL)
void mv_set_power_scheme(void);
#endif
extern nand_info_t nand_info[];       /* info for NAND chips */
extern struct spi_flash *flash;
extern const char version_string[];
#ifdef CONFIG_MRVL_MMC
int mrvl_mmc_initialize(bd_t *bis);
#endif
#ifdef MV_NAND_BOOT
extern MV_U32 nandEnvBase;
#endif
char* strToLower(char * st);
void envVerifyAndSet(char* envName, char* value1, char* value2, int defaultValue);
void envSetDefault(char* envName, char* defaultValue);
int mv_get_arch_number(void);
void setBoardEnv(void);

DECLARE_GLOBAL_DATA_PTR;


// set of variables needed to access alleyCat3 XSMI PHY reg
typedef struct
{
	MV_U32 devAddr;
	MV_U16 regAddr;
	MV_U16 data;
}mv_alleyCat3_xsmi_reg_read_write_STC;

// SKU with PHY XSMI interface
typedef enum
{
	PHY_SXMI_SKU_NUM_OF
}XSMI_PHY_SKU_ENUM;

// struct to write values to set of ports with XSMI PHY
typedef struct  
{
	XSMI_PHY_SKU_ENUM                       sku_num;
	MV_U8									pp_dev_id;     //0-local 1- remote
	MV_U8								    xsmi_bus_num;
	mv_alleyCat3_xsmi_reg_read_write_STC	xsmi_reg_struct;
	MV_U32								    first_phy_addr;
	MV_U32								    last_phy_addr;
}mv_alleyCat3_SXMI_phy_ports_write_STC;


// decliration
void mv_alleyCat3_XSMI_phy_ports_write(XSMI_PHY_SKU_ENUM sku_num, mv_alleyCat3_SXMI_phy_ports_write_STC *arr);

// array of reg values to turn on leds of all SKU ports with XSMI PHY 
mv_alleyCat3_SXMI_phy_ports_write_STC alleyCat3_xsmi_phy_leds_on_array[] =
{
	{ PHY_SXMI_SKU_NUM_OF,0,0,{0,0,0},0,0 }   // LAST Line
};

// array of reg values to turn off leds of all SKU ports with xsmi PHY
mv_alleyCat3_SXMI_phy_ports_write_STC alleyCat3_xsmi_phy_leds_off_array[] =
{
    { PHY_SXMI_SKU_NUM_OF,0,0,{ 0,0,0 },0,0 }   // LAST Line
};

// write to XSMI PHY set of ports according to input array
void mv_alleyCat3_XSMI_phy_ports_write( XSMI_PHY_SKU_ENUM sku_num, mv_alleyCat3_SXMI_phy_ports_write_STC *arr)
{

    MV_U32 phy_add;
	MV_U32 array_idx = 0;
	MV_U32 arr_size = 0;

	while (arr[arr_size].sku_num != PHY_SXMI_SKU_NUM_OF) 
	{
		arr_size++;
	}

	for (array_idx = 0 ; array_idx < arr_size; ++array_idx)
	{
		if (arr[array_idx].sku_num == sku_num)
		{
			// init Xsmi hal 
			MV_ETHPHY_XSMI_HAL_DATA halData;
			halData.ctrlModel = 0;
			halData.ppDevId = arr[array_idx].pp_dev_id;
			halData.ethPhyXsmiRegOff = (arr[array_idx].xsmi_bus_num == 0) ? 0x40000 : 0x42000;
			mvEthPhyXsmiHalInit(&halData);

			phy_add = arr[array_idx].first_phy_addr;

			// set register to each port from first to last
			for (; phy_add <= arr[array_idx].last_phy_addr; phy_add++)
			{
				DB(printf("XSMI_WRITE: PhyAddress:0x%X DevAddr:0x%X RegAdd:0x%X writeval:0x%X\n",
					phy_add,
					arr[array_idx].xsmi_reg_struct.devAddr,
					arr[array_idx].xsmi_reg_struct.regAddr,
					arr[array_idx].xsmi_reg_struct.data));

				mvEthPhyXsmiRegWrite(phy_add,
					arr[array_idx].xsmi_reg_struct.devAddr,
					arr[array_idx].xsmi_reg_struct.regAddr,
					arr[array_idx].xsmi_reg_struct.data);
			}
		}
	}
}



void mv_print_map(void)
{
#ifdef DB_78X60_PCAC
		return 0;
#endif
	printf("\nMap:   Code:\t\t0x%08x:0x%08x\n", (unsigned int)gd->reloc_off, (unsigned int)(gd->reloc_off + _bss_start_ofs));
	printf("       BSS:\t\t0x%08x\n", (unsigned int)(gd->reloc_off + _bss_end_ofs));
	printf("       Stack:\t\t0x%08x\n", (unsigned int)gd->start_addr_sp);
	printf("       Heap:\t\t0x%08x:0x%08x\n\n",(unsigned int)(gd->relocaddr - TOTAL_MALLOC_LEN), (unsigned int)gd->relocaddr);
}

void print_mvBanner(void)
{
#ifdef CONFIG_SILENT_CONSOLE
	DECLARE_GLOBAL_DATA_PTR;
	gd->flags |= GD_FLG_SILENT;
#endif

#ifdef CONFIG_ROL
	mvRolBanner();
#endif
	printf("\n");
	printf(" __   __                      _ _\n");
	printf("|  \\/  | __ _ _ ____   _____| | |\n");
	printf("| |\\/| |/ _` | '__\\ \\ / / _ \\ | |\n");
	printf("| |  | | (_| | |   \\ V /  __/ | |\n");
	printf("|_|  |_|\\__,_|_|    \\_/ \\___|_|_|\n");
	printf("         _   _     ____              _\n");
	printf("        | | | |   | __ )  ___   ___ | |_ \n");
	printf("        | | | |___|  _ \\ / _ \\ / _ \\| __| \n");
	printf("        | |_| |___| |_) | (_) | (_) | |_ \n");
	printf("         \\___/    |____/ \\___/ \\___/ \\__| \n");
	printf(" ** LOADER **\n");
	return;
}

void maskAllInt(void) {
	int i;
	/* for all interrupts (0-115) reset bit 0:3 and 8:11 to disable IRQ and FIQ */
	for (i=0; i < MV_IRQ_NR; i++)
		MV_REG_WRITE(CPU_INT_SOURCE_CONTROL_REG(i), MV_REG_READ(CPU_INT_SOURCE_CONTROL_REG(i)) & ~(0xF0F));

}

/*******************************************************************************
* enable_caches - Platform hook for enabling caches
*
* DESCRIPTION:	This function is called by main u-boot init function
* 				If caches are required they can be enabled here. Currently
* 				this is a stub doing nothing
* INPUT:
*		None
* OUTPUT:
*       None*
* RETURN:
* 		None
*******************************************************************************/
void enable_caches(void)
{
	return;
}

/* init for the Master*/
void misc_init_r_dec_win(void)
{
	char *env;
	mvSysCntmrInit();

#if defined(MV_INCLUDE_USB)
	mvSysUsbInit();	/* initialize USB2.0 */
#endif

#if defined(MV_INCLUDE_XOR)
	// mvSysXorInit();
#endif


#if defined(MV_INCLUDE_CLK_PWR_CNTRL)
	env = getenv("enaClockGating");
	if( env && ((strcmp(env,"yes") == 0) || (strcmp(env,"Yes") == 0)) )
		mv_set_power_scheme();
#endif

    return;
}

/*
 * Miscellaneous platform dependent initializations
 */

/* Device ids for alleycat3 PP */
/* GT_98DX1235 */
#define USPP_device_id_DX1235_CNS  0xf409
/* GT_98DX3236 */
#define USPP_device_id_DX3236_CNS  0xf410 
/* GT_98DX3336 */
#define USPP_device_id_DX3336_CNS  0xf400 
/* GT_98DX1336 */
#define USPP_device_id_DX1336_CNS  0xf41a
/* GT_98DX3233 */
#define USPP_device_id_DX3233_CNS  0xf413 
/* GT_98DX1233 */
#define USPP_device_id_DX1233_CNS  0xf408 
/* Generic device id  */
#define USPP_device_id_User_Define_CNS  0xfc00
/* Device id  mask */
#define USPP_device_id_mask 		 0xffff0

extern int scanDevicesById(MV_U32 deviceId,MV_U32 *num_of_pp);

extern	MV_STATUS mvEthPhyRegRead(MV_U32 phyAddr, MV_U32 regOffs, MV_U16 *data);
extern	MV_STATUS mvEthPhyRegWrite(MV_U32 phyAddr, MV_U32 regOffs, MV_U16 data);

/* golabal mac address for yukon EC */
unsigned char yuk_enetaddr[6];

extern int timer_init(void );
extern void i2c_init(int speed, int slaveaddr);

/* Definitions and arrays for blinking ports leds for Alleycat3 */
#define	  ADDR_REGION(x)  	      (0x01000000 * x)
#define   SMI_VAL(x)      	      (0x02C0C000 + x * 0x10000)
#define	  SMI_BUSY        	      28
#define   SMI_CTRL_REG    	      0x4054
#define   MISC_CONF_REG   	      0x4200
#define   MISC_CONF_VAL   	      0x270000
#define	  PP0_BASE        	      SWITCH_REGS_BASE
#define	  PP0_ADDRESS(x)  	      PP0_BASE + ADDR_REGION(x)
#define   PP1_ADDR_COM_REGION     0xe4000124
#define   PP1_MISC_CONF_REG       0xe4084200
#define   PP1_SMI_CTRL_REG        0xe4084054

/* milliseconds the led will be on */
/*6000-100*/
#define   BLINK_TIME              5900

#define   SWITCH_DEVICE_ID_REG    0x4c
#define	  CPU1_ADDRESS    	      PEX0_MEM_BASE
#define   GPIO_OUT_REG			  0x18100
#define   GPIO_OUT_ENABLED_REG	  0x18104
#define   GPIO_BLINK_ENABLED_REG  0x18108
#define   GPIO_IN_REG	          0x18110

static ulong portLedOnValues[5] = {/* Matrix Mode */ 0x02C0C004, 0x03603fa0, 0x02C0C000,
						           /* LED On 	  */ 0x02C0C003, 0x02009999 };
static ulong portLedOffValues[2] = {0x02001188, 0x02C0C000};

void SMI_WRITE(ulong addr, ulong writeval){
	/* Wait until SMI is ready */
	while ( *((ulong  *)addr) & (1 << SMI_BUSY) )
		mvOsSleep(1);
	*((ulong  *)addr) = (ulong )writeval;
}

/* Memory read\write macros */
#define MEM_WRITE_WITH_MASK(addr, mask, writeval)			*((ulong  *)addr) = ( (*((ulong  *)addr) & ~((ulong)mask)) |  \
															( (ulong)writeval & (ulong)mask ) );

#define MEM_READ_WITH_MASK(addr, mask) 		( (*((ulong *)addr)) & mask )

#define MW_MAC(addr_, writeval_)		*((ulong  *)addr_) = ((ulong)writeval_);


void project_axp_specific_init(MV_VOID){
	  ulong addr, writeval, mask;

	  /* Initialization for all hooper PPs  */

	  mask=0x00000600;
	  /* Configuring green led blinking */
	  addr=0xf1018004;
	  writeval=0x0;
	  *((ulong  *)addr) = ( (*((ulong  *)addr) & ~((ulong)mask)) |
		  ( (ulong)writeval & (ulong)mask ) );
	  addr=0xf1018100;
	  writeval=0x00000400;
	  *((ulong  *)addr) = ( (*((ulong  *)addr) & ~((ulong)mask)) |
	      ( (ulong)writeval & (ulong)mask ) );
	  addr=0xf1018104;
	  writeval=0x00000000;
	  *((ulong  *)addr) = ( (*((ulong  *)addr) & ~((ulong)mask)) |
	      ( (ulong)writeval & (ulong)mask ) );
	  addr=0xf1018110;
	  writeval=0x00000600;
	  *((ulong  *)addr) = ( (*((ulong  *)addr) & ~((ulong)mask)) |
	      ( (ulong)writeval & (ulong)mask ) );
	  addr=0xf1018108;
	  writeval=0x00000400;
	  *((ulong  *)addr) = ( (*((ulong  *)addr) & ~((ulong)mask)) |
	      ( (ulong)writeval & (ulong)mask ) );

}


/* 
* Below code can be used as infrastructure for manipulating PP registers, by providing
* actual register address and value to be written.
* Address Completion method used is the Legacy one.
* Code was copied from cpss/freeRTOS/FreeRTOSV7.3.0/FreeRTOS/Demo/ARMv7_AXP_GCC/labservice/hwServices/mvStub.c
* Code is useful for BC3 and AC3.
*/

/*******************************************************************************
*                    Access Methods Definitions
*******************************************************************************/

#define MV_MEMIO32_WRITE(addr, data) ((*((volatile unsigned int*)(addr))) = ((unsigned int)(data)))
#define MV_MEMIO32_READ(addr)        ((*((volatile unsigned int*)(addr))))

#ifndef MV_32BIT_LE
#define MV_32BIT_LE(X) (X)
#endif

/* No Fast Swap implementation (in assembler) for ARM */
#define MV_32BIT_LE_FAST(val) MV_32BIT_LE(val)

/* 32bit write in little endian mode */
#define MV_MEMIO_LE32_WRITE(addr, data) MV_MEMIO32_WRITE(addr, MV_32BIT_LE_FAST(data))

/*******************************************************************************
* SWITCH_WIN_BASE_ADDR_GET
*******************************************************************************/
static __inline ulong SWITCH_WIN_BASE_ADDR_GET(GT_VOID)
{
    static ulong baseAddr = 0;

    if (baseAddr == 0)
        baseAddr = 0xa8000000/*MV_REG_READ(AHB_TO_MBUS_WIN_BASE_REG(SWITCH_WIN_ID))*/;

    return baseAddr;
}

/*******************************************************************************
* BC2/AC3 SWITCH_LEGACY_ADDR_COMPL_SET (Legacy mode)
*******************************************************************************/

/* In Legacy address completion mode:
        bits [25:24] are the address completion region ID.
        bits [31:24] are copied to the address completion register.

           the address is assembled from:
                   [23:0]  - original bits from the address
                   [25:24] - address completion region ID
                   [31:26] - switch window base address
*/

#define SWITCH_LEGACY_REGS_BASE_ADDR_MASK             0xFC000000
#define SWITCH_LEGACY_REGS_INDEX_OFF                  24
#define SWITCH_LEGACY_ADDR_COMPL_MSB_VAL(addr) ((addr >> SWITCH_LEGACY_REGS_INDEX_OFF) & 0xFF)
#define SWITCH_LEGACY_ADDR_COMPL_SHIFT(addr)   ((addr >> SWITCH_LEGACY_REGS_INDEX_OFF) & 0x3)
#define SWITCH_LEGACY_BUS_ADDR(addr)           ((~SWITCH_LEGACY_REGS_BASE_ADDR_MASK & addr) |\
                                                                                           (SWITCH_WIN_BASE_ADDR_GET() & \
                                                                                            SWITCH_LEGACY_REGS_BASE_ADDR_MASK))

static __inline ulong SWITCH_LEGACY_ADDR_COMPL_SET(ulong addr)
{
    ulong rVal;
    ulong addrCompOffset;
    ulong addrCompReg      = SWITCH_WIN_BASE_ADDR_GET();
    ulong addrCompIndex    = SWITCH_LEGACY_ADDR_COMPL_SHIFT(addr);

    if((addr & 0xFF000000) != 0 && addrCompIndex == 0)
    {
        addrCompIndex = 3;
    }

    addrCompOffset = (addrCompIndex << 3); /* multiple by 8 */

    /* Configure address completion region REG using SERDES memory window */
    rVal  = MV_MEMIO_LE32_READ(addrCompReg);
    rVal &= ~(0xFF << addrCompOffset);
    rVal |= SWITCH_LEGACY_ADDR_COMPL_MSB_VAL(addr) << addrCompOffset;

    MV_MEMIO_LE32_WRITE((unsigned int *)(addrCompReg), rVal);

    return (SWITCH_LEGACY_BUS_ADDR(addr) | (addrCompIndex << SWITCH_LEGACY_REGS_INDEX_OFF));
}

/*******************************************************************************
* genSwitchRegisterSet
*
* DESCRIPTION:
*       Implement write access to device registers.
*******************************************************************************/
void genSwitchRegisterSet(ulong address, ulong data)
{
    address = SWITCH_LEGACY_ADDR_COMPL_SET(address);

    MV_MEMIO_LE32_WRITE(address, data);

}

/* 
* Read board id from PP MSYS CPU GPIOs 
*/
ulong   msys_board_id(MV_VOID) {
    ulong addr, writeval, readval, mask, board_id;

    /* Initialize Data Out Enable registers so we can read Board ID
        MPP[9] = board ID buffer output enable should be set as output and drive '1'.
        MPP[5],MPP[7],MPP[8],MPP[18] = board ID should be set as input and read */
    addr= CPU1_ADDRESS + GPIO_OUT_ENABLED_REG;
    mask = ~(0x00000200);   
    writeval = 0;
    MEM_WRITE_WITH_MASK(addr, mask, writeval);  /* Set MPP[9] Output Enabled */

    addr= CPU1_ADDRESS + GPIO_OUT_ENABLED_REG;
    mask = (1 << 5) | (1 << 7) | (1 << 8) | (1 << 18);
    writeval = mask;
    MEM_WRITE_WITH_MASK(addr, mask, writeval);  /* Set MPP[5],MPP[7],MPP[8],MPP[18] as input */

    addr= CPU1_ADDRESS + GPIO_OUT_REG;
    mask = (1 << 9);
    writeval = mask;
    MEM_WRITE_WITH_MASK(addr, mask, writeval);  /* Set Board ID buffer OE MPP[9] Active HIGH so Board ID can be read */

    /* Read Board ID */
    addr= CPU1_ADDRESS + GPIO_IN_REG;
    readval = MV_REG_READ(addr);
    board_id = ((readval >> 5) & 0xD) | ((readval >> 17) & 0x2);

    return (board_id);
}

void project_msys_specific_init(MV_VOID){
	  ulong addr, writeval, readval, mask, port, region, addrComRegionOne, portLedValueIndex, idx;
/*************************************************************************/
/* Init LEDs */
/*************************************************************************/
//MPP control
MW_MAC(0xf1018000,0x00042222);
MW_MAC(0xf1018004,0x11000000);/*[PEGA 20190715] : modify mpp_reg_1_PTR to 0x11000000 due to redundant power LED display issue*/
MW_MAC(0xf1018008,0x44444004);
MW_MAC(0xf101800C,0x14444444);
MW_MAC(0xf1018010,0x00000000);

//for 7-segment
MW_MAC(0xa8000388,1<<7);
MW_MAC(0xf1018184,0);

//data_out
//0xD0011E80 (MPP[12]=1,MPP[6]=0 for Status Green)
/*MPP[8]=0 (LEDs power),
MPP[12]=0,MPP[6]=1 (Status Amber)*/
MW_MAC(0xf1018100,0xD00106C0);/*[PEGA 20190715] : modify  gpio_0_data_out to 0xD00106C0 due to redundant power LED display issue*/

//data_out_en
MW_MAC(0xf1018104,0x9fecc022);

//DFX (for 7-segment GPP[7] control)
MW_MAC(0xf60f8284,0xaff809fe);

//Serial LEDs
//Init
MW_MAC(0xa8000140,0x8102);
MW_MAC(0xa8000124,0xe0);
MW_MAC(0xa8000128,0x100);
MW_MAC(0xa800012c,0x120);
MW_MAC(0xa8000130,0x140);
MW_MAC(0xa8000134,0x1200);

MW_MAC(0xa8084100,0x738cefc0);
MW_MAC(0xa8104104,0x4b41ee40);
MW_MAC(0xa8184100,0x738cefc0);
MW_MAC(0xa8204104,0x4b41ee40);
MW_MAC(0xa808410c,0x0000e000);
MW_MAC(0xa818410c,0x0000e000);

MW_MAC(0xa8085110,0x00000032);
MW_MAC(0xa8185110,0x00000032);
                 
MW_MAC(0xa8085104,0xe0000000);
MW_MAC(0xa8105100,0x03800000);
MW_MAC(0xa8185104,0xe0000000);
MW_MAC(0xa8205100,0x03800000);

MW_MAC(0xa8280090,0x1a);
MW_MAC(0xa8281090,0x41a);
MW_MAC(0xa8282090,0x81a);
MW_MAC(0xa8283090,0xc1a);
MW_MAC(0xa8284090,0x101a);
MW_MAC(0xa8285090,0x141a);
MW_MAC(0xa8286090,0x181a);
MW_MAC(0xa8287090,0x1c1a);
MW_MAC(0xa8288090,0x201a);
MW_MAC(0xa8289090,0x241a);
MW_MAC(0xa828a090,0x281a);
MW_MAC(0xa828b090,0x2c1a);
MW_MAC(0xa828c090,0x301a);
MW_MAC(0xa828d090,0x341a);
MW_MAC(0xa828e090,0x381a);
MW_MAC(0xa828f090,0x3c1a);
MW_MAC(0xa8290090,0x401a);
MW_MAC(0xa8291090,0x441a);
MW_MAC(0xa8292090,0x481a);
MW_MAC(0xa8293090,0x4c1a);
MW_MAC(0xa8294090,0x501a);
MW_MAC(0xa8295090,0x541a);
MW_MAC(0xa8296090,0x581a);
MW_MAC(0xa8297090,0x5c1a);

//7-segment - "18"
  addr = 0xf1018180;
  *((ulong  *)addr) = 0x0;

  addr = 0xa8000384;
  *((ulong  *)addr) |= (1<<7);

  addr = 0xf1018100;
  *((ulong  *)addr) &= ~(1<<7);

  mvOsSleep(1);

  addr = 0xa8000384;
  *((ulong  *)addr) &= ~(1<<7);

  mvOsSleep(1);

  for(idx=0;idx<9;idx++)
  {
    addr = 0xf1018100;
    *((ulong  *)addr) |= (1<<7);

    mvOsSleep(1);

    *((ulong  *)addr) &= ~(1<<7);
  }
  
  mvOsSleep(1);

  addr = 0xa8000384;
  *((ulong  *)addr) |= 1<<7;

#if 0
//AlexK>According to latest LEDs spec - on power up LEDs should be amber without green
//SD-card
//Solid green
  writeval = 0xFF;
  i2c_write(0x22,0x07,1,&writeval,1);

  writeval = 0xFF;
  i2c_write(0x22,0x03,1,&writeval,1);

//Serial LEDs - Green
MW_MAC(0xa808410c,0x00001fff);
MW_MAC(0xa818410c,0x00001fff);
MW_MAC(0xa8084108,0x00001fff);
MW_MAC(0xa8184108,0x00001fff);

//XG LEDs - Green
  writeval = 0x55;
  i2c_write(0x22,0x02,1,&writeval,1);
#endif

//SD-card
//Solid amber
  writeval = 0xEF;
  i2c_write(0x22,0x07,1,&writeval,1);

  writeval = 0xEF;
  i2c_write(0x22,0x03,1,&writeval,1);

//Serial LEDs - Amber
MW_MAC(0xa808410c,0x00001fff);
MW_MAC(0xa818410c,0x00001fff);
MW_MAC(0xa8084108,0x1fff0000);
MW_MAC(0xa8184108,0x1fff0000);

//XG LEDs - Amber
  writeval = 0xAA;
  i2c_write(0x22,0x02,1,&writeval,1);

mvOsSleep(100);

  writeval = 0x00;
  i2c_write(0x22,0x06,1,&writeval,1);

mvOsSleep(BLINK_TIME); /*6000-100*/
/**************************************************************************/
/*  Final state delay  */
/**************************************************************************/
//7-segment - OFF
  addr = 0xf1018180;
  *((ulong  *)addr) = 0x1;

  addr = 0xa8000384;
  *((ulong  *)addr) &= ~(1<<7);

  mvOsSleep(1);

  for(idx=0;idx<9;idx++)
  {
    addr = 0xf1018100;
    *((ulong  *)addr) |= (1<<7);

    mvOsSleep(1);

    *((ulong  *)addr) &= ~(1<<7);
  }
  
  mvOsSleep(1);

  addr = 0xa8000384;
  *((ulong  *)addr) |= 1<<7;

//SD-card
//HW control
  writeval = 0xE7;
  i2c_write(0x22,0x07,1,&writeval,1);

  writeval = 0xE7;
  i2c_write(0x22,0x03,1,&writeval,1);

//Serial LEDs - OFF
//AlexK>Do not turn off according to latest spec
//MW_MAC(0xa808410c,0x0);
//MW_MAC(0xa818410c,0x0);
//MW_MAC(0xa8084108,0x0);
//MW_MAC(0xa8184108,0x0);

//XG LEDs - OFF
//AlexK>Do not turn XG off according to latest spec
//  writeval = 0x00;
//  i2c_write(0x22,0x02,1,&writeval,1);
}

int board_init (void)
{
	DECLARE_GLOBAL_DATA_PTR;
	int clock_divisor;
	unsigned int i;
	MV_GPP_HAL_DATA gppHalData;
	clock_divisor = (CONFIG_SYS_TCLK / 16)/9600;

	/* muti-core support, initiate each Uart to each cpu */
	mvUartInit(whoAmI(), clock_divisor, mvUartBase(whoAmI()));
	if (whoAmI() != 0)
		return 0;

	if (isBoardMsys())
		mvTargetDefaultsArray = mvTargetMsysArray;
	else
		mvTargetDefaultsArray = mvTargetAxpArray;

#if defined(MV_INCLUDE_TWSI)
	MV_TWSI_ADDR slave;
#endif
	maskAllInt();

	/* must initialize the int in order for udelay to work */
	/* interrupt_init(); - no interrupt handling in u-boot */
	timer_init();

	/* Init the Board environment module (device bank params init) */
	mvBoardEnvInit();
	
#ifdef CONFIG_ROL
	mvRolInitParams();
#endif

#if defined(MV_INCLUDE_TWSI)
	slave.type = ADDR7_BIT;
	slave.address = 0;
	mvTwsiInit(0, CONFIG_SYS_I2C_SPEED, CONFIG_SYS_TCLK, &slave, 0);
#endif

	/* Init the Controlloer environment module (MPP init) */
	mvCtrlEnvInit();

#if defined(CONFIG_DISPLAY_CPUINFO)
	late_print_cpuinfo();          /* display cpu info (and speed) */
#endif
	mvBoardDebugLed(2);

	/* Init the Controller CPU interface */
	if (isBoardMsys())
		mvCpuIfInit(mvMsysWinMap);
	else
		mvCpuIfInit(mvAxpWinMap);
#if defined(MV_NOR_BOOT)
	env_init();
#endif

	/* Init the GPIO sub-system */
	gppHalData.ctrlRev = mvCtrlRevGet();
	mvGppInit(&gppHalData);

	/* arch number of Integrator Board */
	gd->bd->bi_arch_number=mv_get_arch_number();


	/* adress of boot parameters */
	gd->bd->bi_boot_params = 0x00000100;

	/* relocate the exception vectors */
	/* U-Boot is running from DRAM at this stage */
	for(i = 0; i < 0x100; i+=4) {
		*(unsigned int *)(0x0 + i) = *(unsigned int*)(CONFIG_SYS_TEXT_BASE + i);
	}
	mvBoardDebugLed(4);
	mv_print_map();
	
	/* MTL: get board ID */
	i = MV_REG_READ(0x18000);
	i = (i&0xFF0FFFFF);
	MV_REG_WRITE(0x18000, i);
	i = MV_REG_READ(0x18110);
	printf("MTL: boardId = %u %u %u %u\n", ((i>>8)&1),((i>>7)&1), ((i>>18)&1), ((i>>5)&1));
	
	
	return 0;
}

void misc_init_r_env(void){
	char *env;
	char tmp_buf[10];
	unsigned int malloc_len;
	int i;

	envSetDefault("console", "console=ttyS0,9600");
#if defined(MV_NAND) && defined(MV_INCLUDE_SPI)
	envSetDefault("mtdparts", "'mtdparts=armada-nand:8m(boot)ro,8m@8m(kernel),-(rootfs);mtdparts=spi_flash:4m(boot),-(spi-rootfs)'");
	envSetDefault("mtdids", "nand0=armada-nand;spi0=spi_flash");
#elif defined(MV_NAND)
	envSetDefault("mtdparts", "mtdparts=armada-nand:8m(boot)ro,8m@8m(kernel),-(rootfs)");
	envSetDefault("mtdids", "nand0=armada-nand");
#elif defined(MV_INCLUDE_SPI)
	envSetDefault("mtdparts", "mtdparts=spi_flash:4m(boot),-(spi-rootfs)");
	envSetDefault("mtdids", "spi0=spi_flash");
#endif

	env = getenv("nandEcc");
	if (!env) {
#if defined(MV_NAND)
		MV_NFC_ECC_MODE nandEccMode = mvBoardNandECCModeGet();
		switch (nandEccMode) {
		case MV_NFC_ECC_BCH_1K:			/* 8 bit */
			setenv("nandEcc", "nfcConfig=8bitecc");
			break;
		case MV_NFC_ECC_BCH_704B:		/* 12 bit */
			setenv("nandEcc", "nfcConfig=12bitecc");
			break;
		case MV_NFC_ECC_BCH_512B:		/* 16 bit */
			setenv("nandEcc", "nfcConfig=16bitecc");
			break;
		case MV_NFC_ECC_BCH_2K:			/* 4 bit */
		default:
			setenv("nandEcc", "nfcConfig=4bitecc");
			break;
		}
#endif
	}

	/* update the CASset env parameter */
#ifdef MV_MIN_CAL
	envSetDefault("CASset", "min");
#else
	envSetDefault("CASset", "max");
#endif

#if defined (MV_INC_BOARD_NOR_FLASH)
	envVerifyAndSet("enaFlashBuf", "no", "yes",2);
#endif

	setBoardEnv();
	/* Write allocation */
	envVerifyAndSet("enaWrAllo", "no", "yes",1);
	envVerifyAndSet("disL2Cache", "yes", "no",1);
	envVerifyAndSet("cacheShare", "no", "yes",1);
	envVerifyAndSet("pexMode", "EP", "RC",2);
#if defined(MV_INCLUDE_CLK_PWR_CNTRL)
	/* Clock Gating */
	envVerifyAndSet("enaClockGating", "no", "yes",1);
#endif
	envSetDefault("pxefile_addr_r", "3100000");
	envSetDefault("initrd_name", "uInitrd");


	envVerifyAndSet("sata_dma_mode", "no", "yes",2);
	envSetDefault("sata_delay_reset", "0");

	/* Malloc length */
	env = getenv("MALLOC_len");
	if(env)
		malloc_len =  simple_strtoul(env, NULL, 10) << 20;
	else
		malloc_len	= 0;
	if(malloc_len == 0){
		sprintf(tmp_buf,"%d",CONFIG_SYS_MALLOC_LEN>>20);
		setenv("MALLOC_len",tmp_buf);
	}

	/* primary network interface */
	envSetDefault("ethprime", ENV_ETH_PRIME);

	/* image/script addr */
#if defined (CONFIG_CMD_STAGE_BOOT)
	envSetDefault("fdt_addr", "2040000");
	envSetDefault("kernel_addr_r", "2080000");
	envSetDefault("ramdisk_addr_r", "2880000");
	envSetDefault("device_partition", "0:1");
	envSetDefault("boot_order", "hd_scr usb_scr mmc_scr hd_img usb_img mmc_img pxe net_img net_scr");
	envSetDefault("script_name", "boot.scr");
	envSetDefault("ide_path", "/");
	envSetDefault("script_addr_r", "3000000");
	envSetDefault("bootargs_dflt", "$console $nandEcc $mtdparts $bootargs_root nfsroot=$serverip:$rootpath "
				  "ip=$ipaddr:$serverip$bootargs_end $mvNetConfig video=dovefb:lcd0:$lcd0_params "
				  "clcd.lcd0_enable=$lcd0_enable clcd.lcd_panel=$lcd_panel");
	envSetDefault("bootcmd_auto", "stage_boot $boot_order");
	envSetDefault("bootcmd_lgcy", "tftpboot 0x2000000 $image_name; setenv bootargs $bootargs_dflt; bootm 0x2000000; ");
#endif /* #if defined (CONFIG_CMD_STAGE_BOOT) */

	/* netbsd boot arguments */
	env = getenv("netbsd_en");
	if( !env || ( ((strcmp(env,"no") == 0) || (strcmp(env,"No") == 0) ))) {
		setenv("netbsd_en","no");
	} else {
		setenv("netbsd_en","yes");
		envSetDefault("netbsd_gw", "192.168.0.254");
		envSetDefault("netbsd_mask", "255.255.255.0");
		envSetDefault("netbsd_fs", "nfs");
		envSetDefault("netbsd_server", "192.168.0.1");
		envSetDefault("netbsd_ip", getenv("ipaddr"));
		envSetDefault("netbsd_rootdev", "mgi0");
		envSetDefault("netbsd_add", "0x800000");
		envSetDefault("netbsd_get", "tftpboot $netbsd_add $image_name");
		envSetDefault("netbsd_set_args", "setenv bootargs nfsroot=$netbsd_server:$rootpath fs=$netbsd_fs \
                    ip=$netbsd_ip serverip=$netbsd_server mask=$netbsd_mask gw=$netbsd_gw rootdev=$netbsd_rootdev \
                    ethaddr=$ethaddr eth1addr=$eth1addr ethmtu=$ethmtu eth1mtu=$eth1mtu $netbsd_netconfig");
		envSetDefault("netbsd_boot", "bootm $netbsd_add $bootargs");
		envSetDefault("netbsd_bootcmd", "run netbsd_get ; run netbsd_set_args ; run netbsd_boot");
	}

	/* vxWorks boot arguments */
	env = getenv("vxworks_en");
	if( !env || ( ((strcmp(env,"no") == 0) || (strcmp(env,"No") == 0) )))
		setenv("vxworks_en","no");
	else {
		char* buff = (char *)0x1100;
		setenv("vxworks_en","yes");
		sprintf(buff,"mgi(0,0) host:vxWorks.st");
		env = getenv("serverip");
		strcat(buff, " h=");
		strcat(buff,env);
		env = getenv("ipaddr");
		strcat(buff, " e=");
		strcat(buff,env);
		strcat(buff, ":ffff0000 u=anonymous pw=target ");
		setenv("vxWorks_bootargs",buff);
		setenv("bootaddr", "0x1100");
	}

	/* linux boot arguments */
	envSetDefault("bootargs_root", "root=/dev/nfs rw");

	/* For open Linux we set boot args differently */
	env = getenv("mainlineLinux");
	if(env && ((strcmp(env,"yes") == 0) ||  (strcmp(env,"Yes") == 0)))
		envSetDefault("bootargs_end", ":::orion:eth0:none");
	else {
		envSetDefault("bootargs_end", MV_BOOTARGS_END);
	}
	envSetDefault("image_name", "uImage");

#if CONFIG_OF_LIBFDT
	char bootcmd_fdt[] = "tftpboot 0x2000000 $image_name;tftpboot $fdtaddr $fdtfile;"
		"setenv bootargs $console $nandEcc $mtdparts $bootargs_root nfsroot=$serverip:$rootpath "
		"ip=$ipaddr:$serverip$bootargs_end $mvNetConfig video=dovefb:lcd0:$lcd0_params "
		"clcd.lcd0_enable=$lcd0_enable clcd.lcd_panel=$lcd_panel;  bootz 0x2000000 - $fdtaddr;";
	env = getenv("fdtaddr");
	if (!env)
		setenv("fdtaddr", "0x1000000");

	env = getenv("fdtfile");
	if (!env)
		setenv("fdtfile", "bobcat2-db.dtb");
	env = getenv("bootcmd_fdt");
	if (!env)
		setenv("bootcmd_fdt",bootcmd_fdt);
#endif

#if CONFIG_AMP_SUPPORT
	env = getenv("amp_enable");
	if(!env || ( ((strcmp(env,"no") == 0) || (strcmp(env,"No") == 0) ))){
		setenv("amp_enable","no");
	}
	else{
		envSetDefault("amp_groups", "0");
		envSetDefault("amp_shared_mem", "0x80000000:0x100000");
		setenv("bootcmd", "amp_boot");
		envSetDefault("amp_verify_boot", "yes");
	}
#endif

#ifdef CONFIG_ARM_LPAE
	/* LPAE support */
	envSetDefault("enaLPAE", "no");;
#endif

#if (CONFIG_BOOTDELAY >= 0)
	env = getenv("bootcmd");
	if(!env)
#if defined(CONFIG_OF_LIBFDT) && defined (CONFIG_OF_LIBFDT_IS_DEFAULT)
		setenv("bootcmd",bootcmd_fdt);
#elif defined(CONFIG_CMD_STAGE_BOOT)
//	setenv("bootcmd","stage_boot $boot_order");
// Temporary workaround till stage_boot gets stable.
/*	setenv("bootcmd", "tftpboot 0x2000000 $image_name;"
		   "setenv bootargs $console $nandEcc $mtdparts $bootargs_root nfsroot=$serverip:$rootpath "
		   "ip=$ipaddr:$serverip$bootargs_end  video=dovefb:lcd0:$lcd0_params "
		   "clcd.lcd0_enable=$lcd0_enable clcd.lcd_panel=$lcd_panel;  bootm $loadaddr; ");*/
	setenv("bootcmd","setenv bootargs console=ttyS0,9600;nand read 2000000 200000 C00000;bootm 2000000");
#endif
#endif /* (CONFIG_BOOTDELAY >= 0) */
	envSetDefault("standalone", "fsload 0x2000000 $image_name;setenv bootargs $console $nandEcc $mtdparts "
				  "root=/dev/mtdblock0 rw ip=$ipaddr:$serverip$bootargs_end; bootm 0x2000000;");

	/* Disable PNP config of Marvell memory controller devices. */
	envSetDefault("disaMvPnp", "no");
	envSetDefault("bootdelay", "-1");

#if (defined(MV_INCLUDE_GIG_ETH) || defined(MV_INCLUDE_UNM_ETH))
	/* Generate random ip and mac address */
	/* Read RTC to create pseudo-random data for enc */
	unsigned int rand[4] = {0x1, 0x2, 0x3, 0x4};
	char addr_env[20]="ethaddr" , mtu_env[20]="ethmtu";
	char ethaddr_all[30];
#if defined(MV_INCLUDE_RTC)
	struct rtc_time tm;
	rtc_get(&tm);

	rand[0] = ((tm.tm_yday + tm.tm_sec) % 254);
	/* No valid ip with one of the fileds has the value 0 */
	if (rand[0] == 0)
		rand[0]+=2;
	rand[1] = ((tm.tm_yday + tm.tm_min) % 254);
	/* No valid ip with one of the fileds has the value 0 */
	if (rand[1] == 0)
		rand[1]+=2;
	/* Check if the ip address is the same as the server ip */
	if ((rand[1] == 1) && (rand[0] == 11))
		rand[0]+=2;

	rand[2] = (tm.tm_min * tm.tm_sec) % 254;
	rand[3] = (tm.tm_hour * tm.tm_sec) % 254;
#else
	rand[1] = (get_timer(0)) % 254;
	rand[0] = (get_timer(0)) % 254;
	rand[2] = (get_timer(0)) % 254;
	rand[3] = (get_timer(0)) % 254;
#endif

	/* MAC addresses */
	for (i = 0; i < MV_ETH_MAX_PORTS; i++) {
		sprintf(ethaddr_all, "00:50:43:%02x:%02x:%02x", rand[(0 + i) % 4], rand[(1 + i) % 4], rand[(2 + i) % 4]);
		envSetDefault(addr_env, ethaddr_all);
		envSetDefault(mtu_env, "1500");
		sprintf(addr_env, "eth%daddr", i + 1);
		sprintf(mtu_env, "eth%dmtu", i + 1);
	}
	sprintf(ethaddr_all,"00:50:43:%02x:%02x:%02x", rand[3], rand[2], rand[0]);
	envSetDefault("mv_pon_addr", ethaddr_all);

	/* Set mvNetConfig env parameter */
#if defined(MV_INCLUDE_USB)
	/* USB Host */
	envSetDefault("usbType", 2);
#endif  /* (MV_INCLUDE_USB) */

#endif /*  (MV_INCLUDE_GIG_ETH) || defined(MV_INCLUDE_UNM_ETH) */

#if defined(YUK_ETHADDR)
	envSetDefault("yuk_ethaddr", YUK_ETHADDR);
	{
		int i;
		char *tmp = getenv ("yuk_ethaddr");
		char *end;

		for (i=0; i<6; i++) {
			yuk_enetaddr[i] = tmp ? simple_strtoul(tmp, &end, 16) : 0;
			if (tmp)
				tmp = (*end) ? end+1 : end;
		}
	}
#endif /* defined(YUK_ETHADDR) */

#if defined(CONFIG_CMD_RCVR)
	envSetDefault("netretry", "no");
	envSetDefault("rcvrip", RCVR_IP_ADDR);
	envSetDefault("loadaddr", RCVR_LOAD_ADDR);
	envSetDefault("autoload", "no");
	/* Check the recovery trigger */
/*	recoveryDetection(); */
#endif
	envSetDefault("eeeEnable", "no");

#if defined(CONFIG_MV_SCATTERED_SPINUP)
	envSetDefault("spinup_config", "0,0");
#endif /* CONFIG_MV_SCATTERED_SPINUP */

	return;
}


#ifdef BOARD_LATE_INIT
int board_late_init (void)
{
	mvBoardDebugLed(0);
#ifdef CONFIG_ROL
	mvRolLateInit();
#endif
	return 0;
}
#endif

void pcie_tune(void)
{
	MV_REG_WRITE(0xF1041AB0, 0x100);
	MV_REG_WRITE(0xF1041A20, 0x78000801);
	MV_REG_WRITE(0xF1041A00, 0x4014022F);
	MV_REG_WRITE(0xF1040070, 0x18110008);
	return;
}

int board_eth_init(bd_t *bis)
{
#if defined(MV_INCLUDE_GIG_ETH) || defined(MV_INCLUDE_UNM_ETH)
	/* move to the begining so in case we have a PCI NIC it will
	read the env mac addresses correctlly. */
	mv_eth_initialize(bis);
#endif
#if defined(CONFIG_SK98)
	skge_initialize(bis);
#endif
#if defined(CONFIG_E1000)
	e1000_initialize(bis);
#endif
	return 0;
}

#ifdef CONFIG_MMC
int board_mmc_init(bd_t *bis)
{
#ifdef CONFIG_MRVL_MMC
	mrvl_mmc_initialize(bis);
#endif
	return 0;
}
#endif

int print_cpuinfo (void)
{
	return 0;
}

/*
* late_print_cpuinfo - marvell U-Boot print function - used after code relocation
*
* DESCRIPTION:
*       This function is called by board_init_r (after code relocation).
*       all global variables limitations(bss) are off at this state
 */
int late_print_cpuinfo(void)
{
	char name[50];
	mvBoardNameGet(name);
	printf("Board: %s\n",  name);
	mvCtrlModelRevNameGet(name);
	printf("SoC:   %s\n", name);
	if (mvCtrlGetCpuNum())
		printf("       running %d CPUs\n", mvCtrlGetCpuNum()+1);
	if (!mvCtrlIsValidSatR())
		printf("       Custom configuration\n");
	mvCpuNameGet(name);
	printf("CPU:   %s",  name);
#ifdef MV_CPU_LE
	printf(" LE\n");
#else
	printf(" BE\n");
#endif
if (mvCtrlGetCpuNum())
	printf("       CPU %d\n",  whoAmI());

	if (isBoardMsys()) 
		printf("       CPU    @ %d [MHz]\n",  mvCpuPclkGet()/1000000);

	printf("       L2     @ %d [MHz]\n", mvCpuL2ClkGet()/1000000);
	printf("       TClock @ %d [MHz]\n", mvTclkGet()/1000000);
	if (isBoardMsys()) 
		printf("       DDR    @ %d [MHz]\n", CONFIG_SYS_BUS_CLK/1000000);

	printf("       DDR %dBit Width, %s Memory Access, DLB %s\n",
	   mvCtrlDDRBudWidth(), mvCtrlDDRThruXbar()?"XBAR":"FastPath",
	   mvCtrlIsDLBEnabled() ? "Enabled" : "Disabled");
#if defined(CONFIG_ECC_SUPPORT)
		printf("       DDR ECC %s\n", mvCtrlDDRECC()?"Enabled":"Disabled");
#endif
	display_dram_config(1);
	return 0;
}

int board_early_init_f (void)
{
	if (isBoardMsys()) {
		/* Open window to DFX registers
		 * DFX registers are needed in early stage of U-Boot boot sequence
		 * This is done here once, before U-Boot sets its window configuration*/
		MV_REG_WRITE(AHB_TO_MBUS_WIN_CTRL_REG(1), 0x000f0081);
		MV_REG_WRITE(AHB_TO_MBUS_WIN_BASE_REG(1), DFX_REGS_BASE);
	}
	return 0;
}

int misc_init_r (void)
{
	char *env;
	mvBoardDebugLed(5);

	/* init special env variables */
	misc_init_r_env();
#ifdef CONFIG_AMP_SUPPORT
	amp_init();
#endif

	mv_cpu_init();

	/* init the units decode windows */
	misc_init_r_dec_win();

	/* Clear old kernel images which remained stored in memory */
	memset ((void *)CONFIG_SYS_LOAD_ADDR, 0, CONFIG_SYS_MIN_HDR_DEL_SIZE);
	mvBoardDebugLed(6);

	mvBoardDebugLed(7);
	/* pcie fine tunning */
	env = getenv("pcieTune");
	if(env && ((strcmp(env,"yes") == 0) || (strcmp(env,"yes") == 0)))
		pcie_tune();
	else
		setenv("pcieTune","no");

#if defined(MV_INCLUDE_UNM_ETH) || defined(MV_INCLUDE_GIG_ETH)

	/* Init the PHY or Switch of the board */
	mvBoardEgigaPhyInit();
#endif /* #if defined(MV_INCLUDE_UNM_ETH) || defined(MV_INCLUDE_GIG_ETH) */

	return 0;
}

MV_U32 mvTclkGet(void)
{
	DECLARE_GLOBAL_DATA_PTR;
	/* get it only on first time */
	if(gd->tclk == 0)
		gd->tclk = mvBoardTclkGet();

	return gd->tclk;
}

MV_U32 mvSysClkGet(void)
{
	DECLARE_GLOBAL_DATA_PTR;
	/* get it only on first time */
	if(gd->bus_clk == 0)
		gd->bus_clk = mvBoardSysClkGet();

	return gd->bus_clk;
}

/* exported for EEMBC */
#if defined(MV_INCLUDE_RTC) || defined(CONFIG_RTC_DS1338_DS1339)
MV_U32 mvGetRtcSec(void)
{
	MV_RTC_TIME time;
#ifdef MV_INCLUDE_RTC
	mvRtcTimeGet(&time);
#elif CONFIG_RTC_DS1338_DS1339
	mvRtcDS133xTimeGet(&time);
#endif
	return (time.minutes * 60) + time.seconds;
}
#endif

void reset_cpu (ulong addr)
{
	mvBoardReset();
}

void envSetDefault(char *envName, char *defaultValue)
{
	char *env;
	env = getenv(envName);
	if (!env)
		setenv(envName,defaultValue);
}

void envVerifyAndSet(char *envName, char *value1, char *value2, int defaultValue)
{
	char *val1 = strToLower(value1), *val2 = strToLower(value2);
	char *env = strToLower(getenv(envName));
	if (!env) {
		if (defaultValue == 1)
			setenv(envName,val1);
		else
			setenv(envName,val2);
		return;
	}

	if (strcmp(env, val1) == 0)
		setenv(envName, val1);
	else
		setenv(envName, val2);
}

char *strToLower(char *st)
{
	int i;
	for (i = 0; st[i] != '\0'; i++)
		st[i] = tolower(st[i]);
	return st;
}


