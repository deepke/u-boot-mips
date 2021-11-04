/*
 * Copyright (C) 2007 Atmel Corporation
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
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
#include <common.h>
#include <dm.h>
#include <spi.h>
#include <malloc.h>
#include <asm/io.h>
#include <linux/kernel.h>
#define	SPCR	0x00
#define	SPSR	0x01
#define FIFO	0x02
#define	SPER	0x03
#define	PARA	0x04
#define	SFCS	0x05
#define	TIMI	0x06


#define spi_readb(as, reg)					\
	readb(as->regs + reg)
#define spi_writeb(as, reg, value)				\
	writeb(value, as->regs + reg)

DECLARE_GLOBAL_DATA_PTR;

struct ls_spi_priv {
	void		*regs;
	int max_hz;
	uint mode;
	uint hz;
	uint spcr, sper;
};

struct ls_slave_data {
	unsigned int cs;
	uint max_hz;
	uint mode;
};

static void ls_spi_cs_activate(struct ls_spi_priv *as, struct ls_slave_data *slave)
{
	int cs;
	spi_writeb(as, PARA, spi_readb(as, PARA)&~1);
	cs = spi_readb(as, SFCS)&~(0x11 << slave->cs);
	spi_writeb(as, SFCS, (0x1 << slave->cs)|cs);
	spi_readb(as, SFCS);
}

static void ls_spi_cs_deactivate(struct ls_spi_priv *as, struct ls_slave_data *slave)
{
	int cs = spi_readb(as, SFCS)&~(0x11 << slave->cs);
	spi_writeb(as, SFCS, (0x11 << slave->cs)|cs);
	spi_writeb(as, PARA, spi_readb(as, PARA)|1);
	spi_readb(as, SFCS);
}

static int ls_spi_setup(struct ls_spi_priv *as,  uint hz)
{
	unsigned int div, div_tmp;
	unsigned int bit;
	unsigned long clk;
	unsigned char val, spcr, sper;
	const char rdiv[12] = {0, 1, 4, 2, 3, 5, 6, 7, 8, 9, 10, 11};

	if ( hz && as->hz != hz) {
		clk = 100000000;
		div = DIV_ROUND_UP(clk, hz);

		if (div < 2)
			div = 2;

		if (div > 4096)
			div = 4096;

		bit = fls(div) - 1;
		if((1<<bit) == div) bit--;
		div_tmp = rdiv[bit];
		as->hz = hz;
		spcr = div_tmp & 3;
		sper = (div_tmp >> 2) & 3;

		val = spi_readb(as, SPCR);
		spi_writeb(as, SPCR, (val & ~3) | spcr);
		val = spi_readb(as, SPER);
		spi_writeb(as, SPER, (val & ~3) | sper);

	}
	return 0;
}

static int ls_spi_update_state(struct ls_spi_priv *as)
{
	unsigned int hz;
	unsigned int div, div_tmp;
	unsigned int bit;
	unsigned long clk;
	unsigned char val;
	const char rdiv[12] = {0, 1, 4, 2, 3, 5, 6, 7, 8, 9, 10, 11};

	hz  = as->hz;


	clk = 100000000;
	div = DIV_ROUND_UP(clk, hz);

	if (div < 2)
		div = 2;

	if (div > 4096)
		div = 4096;

	bit = fls(div) - 1;
	if ((1<<bit) == div)
		bit--;
	div_tmp = rdiv[bit];

	//dev_dbg(&spi->dev, "clk = %ld hz = %d div_tmp = %d bit = %d\n", clk, hz, div_tmp, bit);

	as->spcr = div_tmp & 3;
	as->sper = (div_tmp >> 2) & 3;

	val = spi_readb(as, SPCR);
	val &= ~0xc;
	if (as->mode & SPI_CPOL)
		val |= 8;
	if (as->mode & SPI_CPHA)
		val |= 4;
	spi_writeb(as, SPCR, (val & ~3) | as->spcr);
	val = spi_readb(as, SPER);
	spi_writeb(as, SPER, (val & ~3) | as->sper);

	return 0;
}


int ls_spi_xfer_internal(struct ls_spi_priv *as, struct ls_slave_data *slave, unsigned int bitlen,
		const void *dout, void *din, unsigned long flags)
{
	unsigned int	len_tx;
	unsigned int	len;
	int		ret;
	u32		status;
	const u8	*txp = dout;
	u8		*rxp = din;
	u8		value;

	ret = 0;
	if (bitlen == 0)
		/* Finish any previously submitted transfers */
		goto out;

	/*
	 * TODO: The controller can do non-multiple-of-8 bit
	 * transfers, but this driver currently doesn't support it.
	 *
	 * It's also not clear how such transfers are supposed to be
	 * represented as a stream of bytes...this is a limitation of
	 * the current SPI interface.
	 */
	if (bitlen % 8) {
		/* Errors always terminate an ongoing transfer */
		flags |= SPI_XFER_END;
		goto out;
	}

	len = bitlen / 8;

	/*
	 * The controller can do automatic CS control, but it is
	 * somewhat quirky, and it doesn't really buy us much anyway
	 * in the context of U-Boot.
	 */
	if (flags & SPI_XFER_BEGIN)
		ls_spi_cs_activate(as, slave);

	for (len_tx = 0; len_tx < len; len_tx++) {

		if (txp)
			value = *txp++;
		else
			value = 0;
		spi_writeb(as, FIFO, value);
		while((spi_readb(as, SPSR) & 0x1) == 1);

		value = spi_readb(as, FIFO);
		if (rxp)
			*rxp++ = value;
	}

out:
	if (flags & SPI_XFER_END) {
		/*
		 * Wait until the transfer is completely done before
		 * we deactivate CS.
		 */
		ls_spi_cs_deactivate(as, slave);
	}

	return 0;
}

static int ls_spi_claim_bus_internal(struct ls_spi_priv *as, struct ls_slave_data *slave)
{
	spi_writeb(as, SPCR, 0x51);
	spi_writeb(as, SPER, 0x04);

	spi_writeb(as, TIMI, 0x01);
	spi_writeb(as, PARA, 0x46);
	return 0;

}

#ifndef CONFIG_DM_SPI
struct ls_spi_slave {
	struct spi_slave slave;
	struct ls_spi_priv  priv;
};

void spi_init(void)
{
}

static inline struct ls_spi_priv *to_ls_spi(struct spi_slave *slave)
{
	struct ls_spi_slave *lss;
	lss = container_of(slave, struct ls_spi_priv, slave);
	return &lss->priv;
}

int spi_cs_is_valid(unsigned int bus, unsigned int cs)
{
	return bus == 0 && cs == 0;
}

void spi_cs_activate(struct spi_slave *slave)
{
	struct ls_spi_priv *as = to_ls_spi(slave);
	ls_spi_cs_activate(as);
}

void spi_cs_deactivate(struct spi_slave *slave)
{
	struct ls_spi_priv *as = to_ls_spi(slave);
	ls_spi_cs_deactivate(as);
}

struct spi_slave *spi_setup_slave(unsigned int bus, unsigned int cs,
			unsigned int max_hz, unsigned int mode)
{
	struct ls_spi_slave	*lss;
	struct ls_spi_priv	*as;
	unsigned int		scbr;
	u32			csrx;
	void			*regs;

	if (cs > 3 || !spi_cs_is_valid(bus, cs))
		return NULL;

	switch (bus) {
	case 0:
		regs = (void *)LS2H_SPI_REG_BASE;
		break;
	default:
		return NULL;
	}


	lss = malloc(sizeof(struct ls_spi_slave));
	if (!lss)
		return NULL;
	lss->slave.bus = bus;
	lss->slave.cs = cs;
	lss->slave.max_hz = max_hz;
	lss->slave.mode = mode;
	as = &lss->priv;
	as->regs = regs;
	
	struct ls_slave_data slave = {
	.cs = cs, 
	.max_hz = max_hz,
	.mode = mode
	};
	
	ls_spi_claim_bus_internal(as, &slave);

	return &lss->slave;
}

void spi_free_slave(struct spi_slave *slave)
{
	struct ls_spi_slave *lss;
	lss = container_of(slave, struct ls_spi_priv, slave);

	free(lss);
}

int spi_claim_bus(struct spi_slave *slave)
{
	return 0;
}

void spi_release_bus(struct spi_slave *slave)
{
}


int spi_xfer(struct spi_slave *slave, unsigned int bitlen,
		const void *dout, void *din, unsigned long flags)
{
	struct ls_spi_priv *as = to_ls_spi(slave);
	struct ls_slave_data slave = {
	.cs = slave->cs, 
	.max_hz = slave->max_hz,
	.mode = slave->mode
	};
	return ls_spi_xfer_internal(as, &slave, bitlen, dout, din, flags);
}
#else

static int ls_spi_probe(struct udevice *bus)
{
	struct ls_spi_priv *as = dev_get_plat(bus);
	int node = dev_of_offset(bus);
	const void *blob = gd->fdt_blob;
	int ret;

	as->regs = map_physmem(devfdt_get_addr(bus), 0, MAP_NOCACHE);

	as->max_hz = fdtdec_get_int(blob, node, "spi-max-frequency",
				      20000000);

	return 0;
}

static int ls_spi_xfer(struct udevice *dev, unsigned int bitlen,
		const void *dout, void *din, unsigned long flags)
{
	struct ls_spi_priv *as = dev_get_plat(dev_get_parent(dev));
	struct dm_spi_slave_plat *slave_plat = dev_get_parent_plat(dev);
	struct ls_slave_data slave = {
	.cs = slave_plat->cs, 
	.max_hz = slave_plat->max_hz,
	.mode = slave_plat->mode
	};
	return ls_spi_xfer_internal(as, &slave, bitlen, dout, din, flags);
}

static int ls_spi_claim_bus(struct udevice *dev)
{
	struct ls_spi_priv *as = dev_get_plat(dev->parent);
	struct dm_spi_slave_plat *slave_plat = dev_get_parent_plat(dev);
	struct ls_slave_data slave = {
	.cs = slave_plat->cs, 
	.max_hz = slave_plat->max_hz,
	.mode = slave_plat->mode
	};

	return ls_spi_claim_bus_internal(as, &slave);
}

static int ls_spi_release_bus(struct udevice *dev)
{
	return 0;
}

static int ls_spi_set_speed(struct udevice *bus, uint speed)
{
	struct ls_spi_priv *as = dev_get_plat(bus);
	if (as->hz != speed)
	{
		as->hz = speed;
		ls_spi_update_state(as);
	}
	return 0;
}

static int ls_spi_set_mode(struct udevice *bus, uint mode)
{
	struct ls_spi_priv *as = dev_get_plat(bus);

	if (as->mode != mode)
	{
		as->mode = mode;
		ls_spi_update_state(as);
	}

	return 0;
}

static const struct dm_spi_ops ls_spi_ops = {
	.claim_bus	= ls_spi_claim_bus,
	.release_bus	= ls_spi_release_bus,
	.xfer		= ls_spi_xfer,
	.set_speed	= ls_spi_set_speed,
	.set_mode	= ls_spi_set_mode,
};

static const struct udevice_id ls_spi_ids[] = {
	{ .compatible = "loongson,ls-spi" },
	{ }
};

U_BOOT_DRIVER(ls_spi) = {
	.name	= "ls_spi",
	.id	= UCLASS_SPI,
	.of_match = ls_spi_ids,
	.ops	= &ls_spi_ops,
	.plat_auto = sizeof(struct ls_spi_priv),
	.probe	= ls_spi_probe,
};
#endif
