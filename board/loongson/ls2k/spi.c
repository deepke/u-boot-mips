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
#include <spi.h>
#include <malloc.h>
#include <asm/io.h>
#include "ls2k.h"
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

struct ls2h_spi_slave {
	struct spi_slave slave;
	void		*regs;
	u32		mr;
};

void spi_init(void)
{
}

static inline struct ls2h_spi_slave *to_ls2h_spi(struct spi_slave *slave)
{
	return container_of(slave, struct ls2h_spi_slave, slave);
}

int spi_cs_is_valid(unsigned int bus, unsigned int cs)
{
	return bus == 0 && cs == 0;
}

void spi_cs_activate(struct spi_slave *slave)
{
	struct ls2h_spi_slave *as = to_ls2h_spi(slave);

	int cs;
	spi_writeb(as, PARA, spi_readb(as, PARA)&~1);
	cs = spi_readb(as, SFCS)&~(0x11 << slave->cs);
	spi_writeb(as, SFCS, (0x1 << slave->cs)|cs);
	spi_readb(as, SFCS);
}

void spi_cs_deactivate(struct spi_slave *slave)
{
	struct ls2h_spi_slave *as = to_ls2h_spi(slave);

	int cs = spi_readb(as, SFCS)&~(0x11 << slave->cs);
	spi_writeb(as, SFCS, (0x11 << slave->cs)|cs);
	spi_writeb(as, PARA, spi_readb(as, PARA)|1);
	spi_readb(as, SFCS);
}

struct spi_slave *spi_setup_slave(unsigned int bus, unsigned int cs,
			unsigned int max_hz, unsigned int mode)
{
	struct ls2h_spi_slave	*as;
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


	as = malloc(sizeof(struct ls2h_spi_slave));
	if (!as)
		return NULL;

	as->slave.bus = bus;
	as->slave.cs = cs;
	as->regs = regs;
	as->mr = (~(1 << cs) & 0xf);

	spi_writeb(as, SPCR, 0x51);
	spi_writeb(as, SPER, 0x04);

	spi_writeb(as, TIMI, 0x01);
	spi_writeb(as, PARA, 0x46);

	return &as->slave;
}

void spi_free_slave(struct spi_slave *slave)
{
	struct ls2h_spi_slave *as = to_ls2h_spi(slave);

	free(as);
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
	struct ls2h_spi_slave *as = to_ls2h_spi(slave);
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
		spi_cs_activate(slave);

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
		spi_cs_deactivate(slave);
	}

	return 0;
}
