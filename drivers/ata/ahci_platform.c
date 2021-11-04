// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2016 Stefan Roese <sr@denx.de>
 */

#include <common.h>
#include <ahci.h>
#include <dm.h>
#include <asm/io.h>

/*
 * Dummy implementation that can be overwritten by a board
 * specific function
 */
__weak int board_ahci_enable(void)
{
	return 0;
}

static int platform_ahci_bind(struct udevice *dev)
{
	struct udevice *scsi_dev;
	int ret;

	ret = ahci_bind_scsi(dev, &scsi_dev);
	if (ret) {
		debug("%s: Failed to bind (err=%d\n)", __func__, ret);
		return ret;
	}

	return 0;
}

static int platform_ahci_probe(struct udevice *dev)
{
	/*
	 * Board specific SATA / AHCI enable code, e.g. enable the
	 * AHCI power or deassert reset
	 */
	ulong base = map_physmem(devfdt_get_addr(dev), sizeof(void *),
			MAP_NOCACHE);
	board_ahci_enable();
	ahci_probe_scsi(dev, base);

	return 0;
}

static const struct udevice_id platform_ahci_ids[] = {
	{ .compatible = "snps,spear-ahci", },
	{ }
};

U_BOOT_DRIVER(ahci_platform_drv) = {
	.name		= "ahci_platform",
	.id		= UCLASS_AHCI,
	.of_match	= platform_ahci_ids,
	.bind		= platform_ahci_bind,
	.probe		= platform_ahci_probe,
};
