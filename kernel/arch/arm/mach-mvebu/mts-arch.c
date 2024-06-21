/* MTS specific arch , for example overwrite pin*/

#include <linux/io.h>
#include <linux/of.h>
#include "mvebu-soc-id.h"

#include <linux/of.h>
#include <linux/of_fdt.h>

/* by model name in dts file we classify SKU to project */
#define DTS_TESLA_MSYS_STR 		"tesla_msys.dts"
#define DTS_AXP_STR				"tesla_axp.dts"

#define	 MV_GPP(_pinId)  	(1<<(_pinId))

/* GT_98DX1233 */
#define MV_DEV_ID_DX1233_CNS                    (0xf408 )
/* Tesla MSYS SF350-08 board id */
#define MV_TESLA_MSYS_SF350_08_BOARD_ID_CNS     (1)

/* AC3 GPP reg offsets */
#define AC3_GPP_DATA_OUT_REG_OFFSET			0x00
#define AC3_GPP_DATA_OUT_EN_REG				0x04
#define AC3_GPP_BLINK_EN_REG				0x08
#define AC3_GPP_DATA_IN_REG					0x10
#define AC3_GPP_DATA_IN_POL_REG				0x0C
#define AC3_GPP_INT_CAUSE_REG				0x14
#define AC3_GPP_INT_MASK_REG				0x18
#define AC3_GPP_INT_LVL_REG					0x1C

typedef enum
{
	MTS_PROJ_TESLA,

	MTS_PROJ_NUM_OF,
}MTS_PROJ_TYPR_EN;

/*******************************************************************************
* mvMsysBoardIdGet
*
* DESCRIPTION:
*	get board Id - relevant for boards that the board id pins are MPP[5],MPP[7],MPP[8],MPP[18]
*
* INPUT:
*	mpp_base_addr       - MPP base address (VA of 0X18100)
*
* RETURN: board Id.
*******************************************************************************/
/* */
static u32 mvTeslaBoardIdGet(void __iomem * mpp_base_addr)
{
	u32 writeval, readval, mask, board_id;
	void __iomem *gpp_base_addr = mpp_base_addr + 0x100;

	/*Initialize Data Out Enable registers so we can read Board ID
	MPP[9] = board ID buffer output enable should be set as output and drive '1'.
	MPP[5],MPP[7],MPP[8],MPP[18] = board ID should be set as input and read */

	readval = readl(gpp_base_addr + AC3_GPP_DATA_OUT_EN_REG);
	writeval = readval & ~(MV_GPP(9));
	writel(writeval, gpp_base_addr + AC3_GPP_DATA_OUT_EN_REG);

	readval = readl(gpp_base_addr + AC3_GPP_DATA_OUT_EN_REG);
	mask = (MV_GPP(5)) | (MV_GPP(7)) | (MV_GPP(8)) | (MV_GPP(18));
	writeval = readval | mask;
	writel(writeval, gpp_base_addr + AC3_GPP_DATA_OUT_EN_REG);				/* Set MPP[5],MPP[7],MPP[8],MPP[18] as input */

    readval = readl(gpp_base_addr);
    writeval = readval | (MV_GPP(9));
    writel(writeval, gpp_base_addr );					/* Set Board ID buffer OE MPP[9] Active HIGH so Board ID can be read */

    /* Read Board ID */
    readval = readl(gpp_base_addr + AC3_GPP_DATA_IN_REG);
    board_id = ((readval >> 5) & 0xD) | ((readval >> 17) & 0x2);
    pr_devel("board_id is %d\n",board_id);

    return (board_id);
}

/*******************************************************************************
* mvMsys_System_LED_Init_SF350_08_board
*
* DESCRIPTION:
*	Set System LED init (Green blinking) for SF350-08 board
*
* INPUT:
*	mpp_base_addr       - MPP base address (VA of 0X18100)
*******************************************************************************/
static void mvMsys_System_LED_Init_SF350_08_board(void __iomem * mpp_base_addr)
{
	u32 readval, writeval, mask;
	void __iomem *gpp_base_addr = mpp_base_addr + 0x100;

	readval = readl(mpp_base_addr);
    mask = 0x00000F0F;      /* Select MPPs[0,2] as GPIOs */
    writeval = readval & ~(mask);
    writel(writeval, mpp_base_addr);

    /*
    * Initialize Data Out Enable registers so we can set System LED Green blinking:
    * MPP[0] = System LED green; MPP[2] = System LED Amber
    */
	readval = readl(gpp_base_addr + AC3_GPP_DATA_OUT_EN_REG);
    mask = (MV_GPP(0)) | (MV_GPP(2));
    writeval = readval & ~(mask);
    writel(writeval, gpp_base_addr + AC3_GPP_DATA_OUT_EN_REG);

    /* Set MPP[2] System LED Green */
    readval = readl(gpp_base_addr);
    writeval = readval  & ~(MV_GPP(0));
    writeval = writeval |  (MV_GPP(2));
    writel(writeval, gpp_base_addr);    /* MPP[2], Green ON,  Amber OFF */

    /* Set blinking */
    readval = readl(gpp_base_addr+AC3_GPP_BLINK_EN_REG);
    writeval = readval & ~(MV_GPP(0));
    writeval = readval |  (MV_GPP(2));
    writel(writeval, gpp_base_addr+AC3_GPP_BLINK_EN_REG);      /* MPP[2], Green ON,  Amber OFF */
}
/*******************************************************************************
* mts_handle_mpp
*
* DESCRIPTION: classify project by dts file model field and handle MPP
*
* INPUT:
*	mpp_base_addr       - MPP base address (VA of 0X18100)
*	pin_id				- pin number
*
*	RETURN: 0 if pin special use for MTS else !=0 (will be configured by kernel).
*******************************************************************************/
int mts_handle_mpp(void __iomem *mpp_base_addr, unsigned pin_id )
{
	u32 dev,rev;
	int ret = -1; /*not over-writen*/
	static MTS_PROJ_TYPR_EN project = MTS_PROJ_NUM_OF;
	struct device_node *projectNd;
    static int init_file_name = 0;   /* 0 means not initialized*/

	/* find project by looking on the "dts_file" field inside the dts file and classify to project
       in case no such field ignore it, do this once since this function called for each pin*/
    if (!init_file_name)
    {
    	init_file_name = 1;
    	projectNd = of_find_node_by_name(NULL, "dts_file");
    	if (projectNd)
    	{
    		const char *str = of_get_property(projectNd, "file_name", NULL);
    		if (str && (!strstr(str, DTS_TESLA_MSYS_STR) || !strstr(str, DTS_AXP_STR)))
    		{
    			project =  MTS_PROJ_TESLA;
    		}
    	}
    }


    /* code relevant for tesla project only */
    if (project == MTS_PROJ_TESLA)
    {
    	/* read PP id*/
		if (mvebu_get_mts_PP_id(&dev, &rev))
		{
			pr_err("%s: failed to get PP id\n", __func__); /* Error*/
		}

		/* Identify id SKU type by PP id and board id */
		if (dev == MV_DEV_ID_DX1233_CNS)
		{
			/* Device ID 0xF408 - suitable for SF350-08 and SF352-08x boards */
			if (mvTeslaBoardIdGet(mpp_base_addr) == MV_TESLA_MSYS_SF350_08_BOARD_ID_CNS)
			{
				/* Set System LED for this board for pins 0,2 */
				if ((pin_id == 0 || pin_id == 2) )
				{
					pr_info("PP Id is 0x%X revision 0x%X, overwrite pin id %d\n",dev, rev, pin_id);
					mvMsys_System_LED_Init_SF350_08_board(mpp_base_addr);
					ret = 0;
				}
			}
		}
    }

    return ret;
}

