/*******************************************************************************
* presteraPpDriver.c
*
* DESCRIPTION:
*       PCI/PEX/PexMbus driver, 2, 4 or 8 regions
*
* DEPENDENCIES:
*
* COMMENTS:
*   Please note: this file is shared for:
*       axp_lsp_3.4.69
*       msys_lsp_3_4
*       msys_lsp_2_6_32
*
*******************************************************************************/
#include <linux/uaccess.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/cdev.h>
#include <linux/proc_fs.h>
#include <linux/interrupt.h>
#include <linux/mm.h>
#include <linux/spinlock.h>
#include <linux/io.h>

#ifdef CONFIG_OF
#include <linux/slab.h>
#endif

#include "mv_prestera.h"
#include "mv_prestera_pp_driver_glob.h"

/* offset of the address completion for PEX 8 completion regions */
#define PEX_MBUS_ADDR_COMP_REG_MAC(_index)	(0x120 + (4 * _index))
/* bits of address passed as is through PCI window */
#define NOT_ADDRESS_COMPLETION_BITS_NUM_CNS	19
/* bits of address extracted from address completion registers */
#define ADDRESS_COMPLETION_BITS_MASK_CNS	(0xFFFFFFFF << NOT_ADDRESS_COMPLETION_BITS_NUM_CNS)

#define ADDR_COMP_REG   0
#define REG_ACCESS_ADDRESS(_regAddr, _compIdx) \
	(((_compIdx) << 24) | ((_regAddr) & 0x00ffffff))

#define REG_ACCESS_ADDRESS_MBUS(_regAddr, _compIdx) \
	(((_compIdx) << NOT_ADDRESS_COMPLETION_BITS_NUM_CNS) | \
	 ((_regAddr) & (~ADDRESS_COMPLETION_BITS_MASK_CNS)))

struct ppDriverData_STC {
	struct pp_dev      *dev;
	uintptr_t           ppRegsBase;
	uintptr_t           pciRegsBase;
	uintptr_t           dfxRegsBase;
	uint32_t            *addrCompletShadow;
	uint8_t             addrCompletionSize;
	spinlock_t          hwComplSem;
	uint8_t             compIdx;
	uint8_t		    completion_shift;
	void (*hwCompletion)(struct ppDriverData_STC *drv, uint32_t regAddr,
		uint8_t *compIdxPtr, uintptr_t *addressPtr);
} ppDriverData_STC;


static void hwCompletionPci(struct ppDriverData_STC *drv, uint32_t regAddr,
				uint8_t *compIdxPtr, uintptr_t *addressPtr)
{
	uint8_t     addrRegion; /* 8 bit MST value of PP internal address*/
	uintptr_t   address;  /*physical access address for PCI access */
	uintptr_t   compIdx; /* address completion register field index 0-3*/
	uint8_t     i;      /* count iterator for the completion index compare loop*/
	uint32_t    data;   /* data to be write to memory */

	/* check if addrRegion is 0 */
	if ((regAddr & 0xFF000000) == 0) {
		compIdx = 0;
	} else {
		spin_lock(&(drv->hwComplSem));

		addrRegion = (uint8_t)(regAddr >> 24);
		/* compare addr region to existing Address regions*/
		for (i = 3; (i > 0) && (addrRegion != drv->addrCompletShadow[i]); i--)
			;/*  */
		if (i == 0) {   /* Set addrRegion in AddrCompletion register */

			/*round robin on Region index : 1,2,3*/
			drv->compIdx++;
			if (drv->compIdx > 3)
				drv->compIdx = 1;
			compIdx = drv->compIdx;

			/*update Address Completion shadow*/
			drv->addrCompletShadow[compIdx] = addrRegion;

			/* update Hw Address Completion - using completion region 0 */
			address = drv->ppRegsBase + ADDR_COMP_REG;
			data = (drv->addrCompletShadow[1]<<8) |
				(drv->addrCompletShadow[2]<<16) |
				(drv->addrCompletShadow[3]<<24);

			/*write the address completion 3 times.
			because the PP have a 2 entry write buffer
			so, the 3 writes will make sure we do get
			to the hardware register itself */
			write_u32(data, address);
			write_u32(data, address);
			write_u32(data, address);
		} else {
			compIdx = i;
		}
	}

	address = drv->ppRegsBase + (uintptr_t)REG_ACCESS_ADDRESS(regAddr, compIdx);
	*compIdxPtr = compIdx;
	*addressPtr = address;
}

static void hwCompletionPexMbus(struct ppDriverData_STC *drv, uint32_t regAddr,
					uint8_t *compIdxPtr, uintptr_t *addressPtr)
{
	uint32_t    addrRegion;  /* 13 bit MSB value of PP internal address */
	uintptr_t   address;  /*physical access address for PCI access */
	uint32_t    compIdx; /* address completion register field index 0-7*/
	uint8_t     i;      /* count iterator for the completion index compare loop*/

	/* check if addrRegion is 0 */
	if ((regAddr & ADDRESS_COMPLETION_BITS_MASK_CNS) == 0) {
		compIdx = 0;
	} else {
		spin_lock(&(drv->hwComplSem));

		addrRegion = (regAddr >> NOT_ADDRESS_COMPLETION_BITS_NUM_CNS);
		/* compare addr region to existing Address regions*/
		for (i = 1; i < 8; i++) {
			if (addrRegion == drv->addrCompletShadow[i])
				break;
		}
		if (i == 8) {
			/* Set addrRegion in AddrCompletion register */

			/*round robin on Region index : 1,2,3*/
			drv->compIdx++;
			if (drv->compIdx > 7)
				drv->compIdx = 1;
			compIdx = drv->compIdx;

			/*update Address Completion shadow*/
			drv->addrCompletShadow[compIdx] = addrRegion;

			/* update Hw Address Completion - using completion region 0 */
			address = drv->ppRegsBase + PEX_MBUS_ADDR_COMP_REG_MAC(compIdx);
			write_u32(addrRegion, address);
		} else
			compIdx = i;
	}

	address = drv->ppRegsBase + (uintptr_t)REG_ACCESS_ADDRESS_MBUS(regAddr, compIdx);
	*compIdxPtr = compIdx;
	*addressPtr = address;
}


static void hwCompletionPciHalf(struct ppDriverData_STC *drv, uint32_t regAddr,
					uint8_t *compIdxPtr, uintptr_t *addressPtr)
{
	uint8_t     addrRegion; /* 8 bit MST value of PP internal address*/
	uintptr_t   address;  /*physical access address for PCI access */
	uintptr_t   compIdx; /* address completion register field index 0-3*/
	uint32_t    data;   /* data to be write to memory */

	/* check if addrRegion is 0 */
	if ((regAddr & 0xFF000000) == 0)
		compIdx = 0;
	else {
		spin_lock(&(drv->hwComplSem));

		compIdx = 1;
		addrRegion = (uint8_t)(regAddr >> 24);
		if (drv->addrCompletShadow[0] != addrRegion) {
			/* Set addrRegion in AddrCompletion register */

			/*update Address Completion shadow*/
			drv->addrCompletShadow[0] = addrRegion;

			/* update Hw Address Completion - using completion region 0 */
			address = drv->ppRegsBase + ADDR_COMP_REG;
			data = (drv->addrCompletShadow[0]<<8);

			/*write the address completion 3 times.
			  because the PP have a 2 entry write buffer
			  so, the 3 writes will make sure we do get
			  to the hardware register itself */
			write_u32(data, address);
			write_u32(data, address);
			write_u32(data, address);
		}
	}

	address = drv->ppRegsBase + (uintptr_t)REG_ACCESS_ADDRESS(regAddr, compIdx);
	*compIdxPtr = compIdx;
	*addressPtr = address;
}


static int hwReadWrite(struct ppDriverData_STC *drv, int isReadOp,
		       uint32_t regAddr, uint32_t length, uint32_t *dataPtr)
{
	uintptr_t address; /*physical address for PCI access */
	uint8_t   compIdx; /* address completion register field index 0-3*/
	uint32_t  j = 0;   /* count iterator for the write loop*/
	uint32_t  nextRegionAddr; /* address of the next region after the one
								 currently used */
	uint32_t  loopLength = 0; /* when length exceeds region addr, Set to end of
								 region range */
	uint32_t  data;

	drv->hwCompletion(drv, regAddr, &compIdx, &address);

	/* check whether completion region boundaries exceeded*/
	nextRegionAddr =  (uint32_t)(drv->addrCompletShadow[compIdx] + 1) << drv->completion_shift;
	loopLength = length;
	if ((uintptr_t)(regAddr + length * 4) > nextRegionAddr)
		loopLength = (nextRegionAddr - regAddr) / 4;

	for (j = 0; j < loopLength; j++) {
		if (isReadOp) {
			data = read_u32(address);
			if (put_user(data, dataPtr+j)) {
				if (compIdx != 0)
					spin_unlock(&(drv->hwComplSem));
				return -EFAULT;
			}
		} else {
			if (get_user(data, dataPtr+j)) {
				if (compIdx != 0)
					spin_unlock(&(drv->hwComplSem));
				return -EFAULT;
			}

			write_u32(data, address);
		}

		address += 4;
	}
	if (compIdx != 0)
		spin_unlock(&(drv->hwComplSem));

	if (loopLength < length) {
		/* Recursive call for rest of data in next region.  */
		return hwReadWrite(drv, isReadOp, nextRegionAddr, length-loopLength,
				dataPtr+loopLength);
	}
	return 0;
}


int presteraPpDriverRead(uintptr_t base, uint32_t  size,
			       uint32_t  regAddr, uint32_t *dataPtr)
{
	uint32_t data;
	if (base == 0 || regAddr >= size)
		return -EFAULT;

	base += regAddr;

	data = read_u32(base);

	if (put_user(data, dataPtr))
		return -EFAULT;

	return 0;
}

int presteraPpDriverWrite(uintptr_t base, uint32_t size,
				uint32_t regAddr, uint32_t *dataPtr)
{
	uint32_t data;
	if (base == 0 || regAddr >= size)
		return -EFAULT;

	if (get_user(data, dataPtr))
		return -EFAULT;

	base += regAddr;
	write_u32(data, base);

	return 0;
}

static int presteraPpDriverReset(struct ppDriverData_STC *drv)
{
	uintptr_t address;
	uint32_t  data;
	int i;

	spin_lock(&(drv->hwComplSem));

	if (drv->dev->ppdriverType == mvPpDrvDriverType_PexMbus_E) {
		/* set 8-region mode: regAddr = 0x140, set bit16 to 0 */
		address = drv->ppRegsBase + 0x140;
		data = read_u32(address);
		data &= (~(1 << 16));
		write_u32(data, address);
	}


	/* Update Address Completion shadow */
	for (i = 0; i < drv->addrCompletionSize; i++) {
		drv->addrCompletShadow[i] = 0;
		if (drv->dev->ppdriverType == mvPpDrvDriverType_PexMbus_E) {
			/* Reset Hw Address Completion          */
			address = drv->ppRegsBase+PEX_MBUS_ADDR_COMP_REG_MAC(i);
			write_u32(0, address);
		}
	}

	if (drv->dev->ppdriverType == mvPpDrvDriverType_Pci_E ||
	    drv->dev->ppdriverType == mvPpDrvDriverType_PexMbus_E)
		drv->compIdx = 1;


	if (drv->dev->ppdriverType == mvPpDrvDriverType_Pci_E ||
	    drv->dev->ppdriverType == mvPpDrvDriverType_PciHalf_E) {
		/* Reset Hw Address Completion */
		address = drv->ppRegsBase + ADDR_COMP_REG;
		write_u32(0, address);
		write_u32(0, address);
		write_u32(0, address);
	}

	spin_unlock(&(drv->hwComplSem));

	return 0;
}

static int presteraPpDriverDestroy(struct ppDriverData_STC *drv)
{
	struct pp_dev *dev = drv->dev;

	if (dev->ppregs.base == 0)
		iounmap((void *)drv->ppRegsBase);
	if (dev->config.base == 0)
		iounmap((void *)drv->pciRegsBase);
	if (drv->dfxRegsBase && dev->dfx.base == 0)
		iounmap((void *)drv->dfxRegsBase);

	dev->ppdriver = (PP_DRIVER_FUNC)NULL;
	dev->ppdriverType = 0;
	dev->ppdriverData = NULL;

	kfree(drv->addrCompletShadow);
	kfree(drv);

	return 0;
}

static int presteraPpDriverIo(struct ppDriverData_STC *drv, struct mvPpDrvDriverIo_STC *io)
{
	if (io == NULL)
		return presteraPpDriverDestroy(drv);

	switch (io->op) {
	case mvPpDrvDriverIoOps_PpRegRead_E:
		return hwReadWrite(drv, 1, io->regAddr, 1, (uint32_t *)(io->dataPtr));
	case mvPpDrvDriverIoOps_PpRegWrite_E:
		return hwReadWrite(drv, 0, io->regAddr, 1, (uint32_t *)(io->dataPtr));
	case mvPpDrvDriverIoOps_RamRead_E:
		return hwReadWrite(drv, 1, io->regAddr, io->length, (uint32_t *)(io->dataPtr));
	case mvPpDrvDriverIoOps_RamWrite_E:
		return hwReadWrite(drv, 0, io->regAddr, io->length, (uint32_t *)(io->dataPtr));
	case mvPpDrvDriverIoOps_Reset_E:
		return presteraPpDriverReset(drv);
	case mvPpDrvDriverIoOps_Destroy_E:
		return presteraPpDriverDestroy(drv);
	case mvPpDrvDriverIoOps_PciRegRead_E:
		return presteraPpDriverRead(
				drv->pciRegsBase, drv->dev->config.size,
				io->regAddr, (uint32_t *)(io->dataPtr));
	case mvPpDrvDriverIoOps_PciRegWrite_E:
		return presteraPpDriverWrite(
				drv->pciRegsBase, drv->dev->config.size,
				io->regAddr, (uint32_t *)(io->dataPtr));
	case mvPpDrvDriverIoOps_DfxRegRead_E:
		return presteraPpDriverRead(
				drv->dfxRegsBase, drv->dev->dfx.size,
				io->regAddr, (uint32_t *)(io->dataPtr));
	case mvPpDrvDriverIoOps_DfxRegWrite_E:
		return presteraPpDriverWrite(
				drv->dfxRegsBase, drv->dev->dfx.size,
				io->regAddr, (uint32_t *)(io->dataPtr));
	}
	return -EFAULT;
}

int presteraPpDriverCreate(struct pp_dev *dev, enum mvPpDrvDriverType_ENT type)
{
	struct ppDriverData_STC *drv;
	size_t regsSize;

	drv = kmalloc(sizeof(struct ppDriverData_STC), GFP_KERNEL);
	if (!drv)
		return -ENOMEM;

	memset(drv, 0, sizeof(*drv));

	switch (type) {
	case mvPpDrvDriverType_Pci_E:
		drv->addrCompletionSize = 4;
		drv->hwCompletion = hwCompletionPci;
		drv->completion_shift = 24;
		regsSize = 64 * 1024 * 1024;
		break;
	case mvPpDrvDriverType_PciHalf_E:
		drv->addrCompletionSize = 1;
		drv->hwCompletion = hwCompletionPciHalf;
		drv->completion_shift = 24;
		regsSize = 32 * 1024 * 1024;
		break;
	case mvPpDrvDriverType_PexMbus_E:
		drv->addrCompletionSize = 8;
		drv->hwCompletion = hwCompletionPexMbus;
		drv->completion_shift = NOT_ADDRESS_COMPLETION_BITS_NUM_CNS;
		regsSize = 4 * 1024 * 1024;
		break;
	}


	drv->addrCompletShadow = kmalloc(drv->addrCompletionSize * sizeof(drv->addrCompletShadow), GFP_KERNEL);
	if (!drv->addrCompletShadow) {
		kfree(drv);
		return -ENOMEM;
	}

	memset(drv->addrCompletShadow, 0, drv->addrCompletionSize * sizeof(drv->addrCompletShadow));

	drv->dev = dev;

	if (dev->ppregs.base == 0)
		drv->ppRegsBase = (uintptr_t)ioremap_nocache(dev->ppregs.phys, regsSize);
	else
		drv->ppRegsBase = dev->ppregs.base;
	if (dev->config.base == 0)
		drv->pciRegsBase = (uintptr_t)ioremap_nocache(dev->config.phys, dev->config.size);
	else
		drv->pciRegsBase = dev->config.base;
	if (dev->dfx.phys) {
		if (dev->dfx.base == 0)
			drv->dfxRegsBase = (uintptr_t)ioremap_nocache(dev->dfx.phys, dev->dfx.size);
		else
			drv->dfxRegsBase = dev->dfx.base;
	}

	spin_lock_init(&(drv->hwComplSem));
	if (type == mvPpDrvDriverType_Pci_E || type == mvPpDrvDriverType_PexMbus_E)
		drv->compIdx = 1;

	dev->ppdriver = (PP_DRIVER_FUNC)presteraPpDriverIo;
	dev->ppdriverType = (int)type;
	dev->ppdriverData = drv;

	return 0;
}
