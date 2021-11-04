/*
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
 *
 */

#include <common.h>
#include <asm/io.h>

int LS2H_I2C_BASE=0xbfe01000;

#define ls2h_i2c_writeb(val, addr)	writeb(val, LS2H_I2C_BASE + addr)
#define ls2h_i2c_readb(addr)		readb(LS2H_I2C_BASE + addr)


/* All transfers are described by this data structure */
struct i2c_msg {
	__u16 addr;	/* slave address			*/
	__u16 flags;
#define I2C_M_TEN		0x0010	/* this is a ten bit chip address */
#define I2C_M_RD		0x0001	/* read data, from slave to master */
#define I2C_M_NOSTART		0x4000	/* if I2C_FUNC_PROTOCOL_MANGLING */
#define I2C_M_REV_DIR_ADDR	0x2000	/* if I2C_FUNC_PROTOCOL_MANGLING */
#define I2C_M_IGNORE_NAK	0x1000	/* if I2C_FUNC_PROTOCOL_MANGLING */
#define I2C_M_NO_RD_ACK		0x0800	/* if I2C_FUNC_PROTOCOL_MANGLING */
#define I2C_M_RECV_LEN		0x0400	/* length will be first received byte */
	__u16 len;		/* msg length				*/
	__u8 *buf;		/* pointer to msg data			*/
};


#define	CR_START			0x80
#define	CR_STOP				0x40
#define	CR_READ				0x20
#define	CR_WRITE			0x10
#define	CR_ACK				0x8
#define	CR_IACK				0x1

#define	SR_NOACK			0x80
#define	SR_BUSY				0x40
#define	SR_AL				0x20
#define	SR_TIP				0x2
#define	SR_IF				0x1

#define LS2H_I2C_PRER_LO_REG	0x0
#define LS2H_I2C_PRER_HI_REG	0x1
#define LS2H_I2C_CTR_REG    	0x2
#define LS2H_I2C_TXR_REG    	0x3
#define LS2H_I2C_RXR_REG    	0x3
#define LS2H_I2C_CR_REG     	0x4
#define LS2H_I2C_SR_REG     	0x4

#define ls2h_i2c_debug(fmt, args...)	printf(fmt, ##args)
#define pr_info printf

static void ls2h_i2c_stop(void)
{
again:
        ls2h_i2c_writeb(CR_STOP, LS2H_I2C_CR_REG);
        ls2h_i2c_readb(LS2H_I2C_SR_REG);
        while (ls2h_i2c_readb(LS2H_I2C_SR_REG) & SR_BUSY)
                goto again;
}

static int ls2h_i2c_start(int dev_addr, int flags)
{
	int retry = 5;
	unsigned char addr = (dev_addr & 0x7f) << 1;
	addr |= (flags & I2C_M_RD)? 1:0;

start:
	ls2h_i2c_writeb(addr, LS2H_I2C_TXR_REG);
	ls2h_i2c_debug("%s <line%d>: i2c device address: 0x%x\n",
			__func__, __LINE__, addr);
	ls2h_i2c_writeb((CR_START | CR_WRITE), LS2H_I2C_CR_REG);
	while (ls2h_i2c_readb(LS2H_I2C_SR_REG) & SR_TIP) ;

	if (ls2h_i2c_readb(LS2H_I2C_SR_REG) & SR_NOACK) {
		ls2h_i2c_stop();
		while (retry--)
			goto start;
		pr_info("There is no i2c device ack\n");
		return 0;
	}
	return 1;
}

static int ls2h_i2c_read(unsigned char *buf, int count)
{
        int i;

        for (i = 0; i < count; i++) {
                ls2h_i2c_writeb((i == count - 1)?
				(CR_READ | CR_ACK) : CR_READ,
				LS2H_I2C_CR_REG);
                while (ls2h_i2c_readb(LS2H_I2C_SR_REG) & SR_TIP) ;
                buf[i] = ls2h_i2c_readb(LS2H_I2C_RXR_REG);
		ls2h_i2c_debug("%s <line%d>: read buf[%d] <= %02x\n",
				__func__, __LINE__, i, buf[i]);
        }

        return i;
}

static int ls2h_i2c_write(unsigned char *buf, int count)
{
        int i;

        for (i = 0; i < count; i++) {
		ls2h_i2c_writeb(buf[i], LS2H_I2C_TXR_REG);
		ls2h_i2c_debug("%s <line%d>: write buf[%d] => %02x\n",
				__func__, __LINE__, i, buf[i]);
		ls2h_i2c_writeb(CR_WRITE, LS2H_I2C_CR_REG);
		while (ls2h_i2c_readb(LS2H_I2C_SR_REG) & SR_TIP) ;

		if (ls2h_i2c_readb(LS2H_I2C_SR_REG) & SR_NOACK) {
			ls2h_i2c_debug("%s <line%d>: device no ack\n",
					__func__, __LINE__);
			ls2h_i2c_stop();
			return 0;
		}
        }

        return i;
}


static int i2c_transfer(struct i2c_msg *msgs, int num)
{
	struct i2c_msg *m = msgs;
	int i;

	for(i = 0; i < num; i++) {
		if (!(m->flags & I2C_M_NOSTART) && !ls2h_i2c_start(m->addr, m->flags)) {
			return 0;
		}
		if (m->flags & I2C_M_RD)
			ls2h_i2c_read(m->buf, m->len);
		else
			ls2h_i2c_write(m->buf, m->len);
		++m;
	}

	ls2h_i2c_stop();

	return i;
}

/* ------------------------------------------------------------------------ */
/* API Functions                                                            */
/* ------------------------------------------------------------------------ */

void i2c_init(int speed, int slaveaddr)
{
        ls2h_i2c_writeb(0, LS2H_I2C_CTR_REG);
        ls2h_i2c_writeb(0x2c, LS2H_I2C_PRER_LO_REG);
        ls2h_i2c_writeb(0x1, LS2H_I2C_PRER_HI_REG);
        ls2h_i2c_writeb(0x80, LS2H_I2C_CTR_REG);
}


/**
 * i2c_probe: - Test if a chip answers for a given i2c address
 *
 * @chip:	address of the chip which is searched for
 * @return:	0 if a chip was found, -1 otherwhise
 */

int i2c_probe(uchar chip)
{
	if(ls2h_i2c_start(chip, 0))
        {
		ls2h_i2c_stop();
                return 0;
        }
        else
          return -1;
}


/**
 * i2c_read: - Read multiple bytes from an i2c device
 *
 * The higher level routines take into account that this function is only
 * called with len < page length of the device (see configuration file)
 *
 * @chip:	address of the chip which is to be read
 * @addr:	i2c data address within the chip
 * @alen:	length of the i2c data address (1..2 bytes)
 * @buffer:	where to write the data
 * @len:	how much byte do we want to read
 * @return:	0 in case of success
 */

int i2c_read(uchar chip, uint addr, int alen, uchar *buffer, int len)
{
	struct i2c_msg msg[2] = { { chip, 0, alen, (__u8 *)&addr },
	                          { chip, I2C_M_RD, len, buffer }
	                        };

	ls2h_i2c_debug(("i2c_read(chip=0x%02x, addr=0x%02x, alen=0x%02x, len=0x%02x)\n",chip,addr,alen,len));

	i2c_transfer(msg, 2);
	return 0;
}


/**
 * i2c_write: -  Write multiple bytes to an i2c device
 *
 * The higher level routines take into account that this function is only
 * called with len < page length of the device (see configuration file)
 *
 * @chip:	address of the chip which is to be written
 * @addr:	i2c data address within the chip
 * @alen:	length of the i2c data address (1..2 bytes)
 * @buffer:	where to find the data to be written
 * @len:	how much byte do we want to read
 * @return:	0 in case of success
 */

int i2c_write(uchar chip, uint addr, int alen, uchar *buffer, int len)
{
	struct i2c_msg msg[2] = { { chip, 0, alen, (__u8 *)&addr },
	                          { chip, I2C_M_NOSTART, len, buffer }
	                        };
	ls2h_i2c_debug(("i2c_write(chip=0x%02x, addr=0x%02x, alen=0x%02x, len=0x%02x)\n",chip,addr,alen,len));

	i2c_transfer(msg, 2);
	return 0;
}

