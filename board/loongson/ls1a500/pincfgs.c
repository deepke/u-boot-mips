#include <common.h>
#include <command.h>
#include "pincfgs.h"
pin_cfgs_t default_pin_cfgs[] = {
	//0:GPIO , 5:main
	{  0, 0},	//0:GPIO0
	{  1, 0},	//0:GPIO1
	{  2, 5},	//5:vga_hsync
	{  3, 5},	//5:vga_vsync
	{  4, 5},	//5:lcd_clk
	{  5, 5},	//5:lcd_vsync
	{  6, 5},	//5:lcd_hsync
	{  7, 5},	//5:lcd_en
	{  8, 5},	//5:lcd_dat_b[0]
	{  9, 5},	//5:lcd_dat_b[1]
	{ 10, 5},	//5:lcd_dat_b[2]
	{ 11, 5},	//5:lcd_dat_b[3]
	{ 12, 5},	//5:lcd_dat_b[4]
	{ 13, 5},	//5:lcd_dat_b[5]
	{ 14, 5},	//5:lcd_dat_b[6]
	{ 15, 5},	//5:lcd_dat_b[7]
	{ 16, 5},	//5:lcd_dat_g[0]
	{ 17, 5},	//5:lcd_dat_g[1]
	{ 18, 5},	//5:lcd_dat_g[2]
	{ 19, 5},	//5:lcd_dat_g[3]
	{ 20, 5},	//5:lcd_dat_g[4]
	{ 21, 5},	//5:lcd_dat_g[5]
	{ 22, 5},	//5:lcd_dat_g[6]
	{ 23, 5},	//5:lcd_dat_g[7]
	{ 24, 5},	//5:lcd_dat_r[0]
	{ 25, 5},	//5:lcd_dat_r[1]
	{ 26, 5},	//5:lcd_dat_r[2]
	{ 27, 5},	//5:lcd_dat_r[3]
	{ 28, 5},	//5:lcd_dat_r[4]
	{ 29, 5},	//5:lcd_dat_r[5]
	{ 30, 5},	//5:lcd_dat_r[6]
	{ 31, 5},	//5:lcd_dat_r[7]
	{ 32, 5},	//5:kb_clk
	{ 33, 5},	//5:kb_dat
	{ 34, 5},	//5:ms_clk
	{ 35, 5},	//5:ms_dat
	{ 36, 5},	//5:ac97_datai
	{ 37, 5},	//5:ac97_datao
	{ 38, 5},	//5:ac97_sync
	{ 39, 5},	//5:ac97_reset
	{ 40, 5},	//5:spi0_clk
	{ 41, 5},	//5:spi0_miso
	{ 42, 5},	//5:spi0_mosi
	{ 43, 5},	//5:spi0_cs[0]
	{ 44, 0},	//5:spi1_clk	0:GPIO44
	{ 45, 0},	//5:spi1_miso	0:GPIO45
	{ 46, 0},	//5:spi1_mosi	0:GPIO46
	{ 47, 0},	//5:spi1_cs[0]	0:GPIO47
	{ 48, 1},	//5:uart0_rx	1:gmac1_rx_ctl	
	{ 49, 1},	//5:uart0_tx	1:gmac1_rx[0]	
	{ 50, 1},	//5:uart0_rts	1:gmac1_rx[1]	3:scl1
	{ 51, 1},	//5:uart0_cts	1:gmac1_rx[2]	3:sda1
	{ 52, 1},	//5:uart0_dsr	1:gmac1_rx[3]	
	{ 53, 1},	//5:uart0_dtr	1:gmac1_tx_ctl	
	{ 54, 1},	//5:uart0_dcd	1:gmac1_tx[0]	
	{ 55, 1},	//5:uart0_ri	1:gmac1_tx[1]	
	{ 56, 1},	//1:gmac1_tx[2]
	{ 57, 1},	//1:gmac1_tx[3]
	{ 58, 1},	//1:gmac1_mdck
	{ 59, 1},	//1:gmac1_mdio
	{ 60, 5},	//5:uart2_tx
	{ 61, 5},	//5:uart2_rx
	{ 62, 1},	//5:uart3_tx	1:pix1_scl
	{ 63, 1},	//5:uart3_rx	1:pix1_sda	2:pwm[13]
	{ 64, 1},	//5:scl0	1:nand_rdy[1]	3:spi0_cs[3]
	{ 65, 1},	//5:sda0	1:nand_ce[1]	3:spi0_cs[2]
	{ 66, 1},	//5:can0_rx	1:nand_rdy[2]	2:sda2		3:spi0_cs[1]
	{ 67, 1},	//5:can0_tx	1:nand_ce[2]	2:scl2		3:spi1_cs[3]
	{ 68, 1},	//5:can1_rx	1:nand_rdy[3]	2:sda3		3:spi1_cs[2]
	{ 69, 1},	//5:can1_tx	1:nand_ce[3]	2:scl3		3:spi1_cs[1]
	{ 70, 5},	//5:lpc_ad[0]	1:nand_d[0]
	{ 71, 5},	//5:lpc_ad[1]	1:nand_d[1]
	{ 72, 5},	//5:lpc_ad[2]   1:nand_d[2]
	{ 73, 5},	//5:lpc_ad[3]   1:nand_d[3]
	{ 74, 5},	//5:lpc_frame	1:nand_d[4]
	{ 75, 5},	//5:lpc_serirq	1:nand_d[5]
	{ 76, 5},	//5:nand_cle
	{ 77, 5},	//5:nand_ale
	{ 78, 5},	//5:nand_rd
	{ 79, 5},	//5:nand_wr
	{ 80, 5},	//5:nand_ce[0]
	{ 81, 5},	//5:nand_rdy[0]
	{ 82, 5},	//5:nand_d[6]
	{ 83, 5},	//5:nand_d[7]
	{ 84, 5},	//5:pwm[0]
	{ 85, 5},	//5:pwm[1]
	{ 86, 5},	//5:pwm[2]	0:GPIO86
	{ 87, 5},	//5:pwm[3]
	{ 88, 5},	//5:gmac0_rx_ctl
	{ 89, 5},	//5:gmac0_rx[0]
	{ 90, 5},	//5:gmac0_rx[1]
	{ 91, 5},	//5:gmac0_rx[2]
	{ 92, 5},	//5:gmac0_rx[3]
	{ 93, 5},	//5:gmac0_tx_ctl
	{ 94, 5},	//5:gmac0_tx[0]
	{ 95, 5},	//5:gmac0_tx[1]
	{ 96, 5},	//5:gmac0_tx[2]
	{ 97, 5},	//5:gmac0_tx[3]
	{ 98, 5},	//5:gmac0_mdck
	{ 99, 5},	//5:gmac0_mdio
	{100, 5},	//1:pr_int	2:lioa[0]
	{101, 5},	//1:pr0_clk	2:lioa[1]
	{102, 5},	//1:pr0_start	2:lioa[2]
	{103, 5},	//1:pr0_ready	2:lioa[3]
	{104, 5},	//1:pr0_enable	2:lioa[4]
	{105, 5},	//1:pr0_shold	2:lioa[5]	3:pwm[5]
	{106, 5},	//1:pr0_data	2:lioa[6]	3:pwm[6]
	{107, 5},	//1:pr0_hsync	2:lioa[7]	3:pwm[7]
	{108, 5},	//1:pr1_enable	2:lioa[8]	3:pwm[8]
	{109, 5},	//1:pr1_shold	2:lioa[9]	3:pwm[9]
	{110, 5},	//1:pr1_data	2:lioa[10]	3:pwm[10]
	{111, 5},	//1:pr2_clk	2:lioa[11]	3:pwm[11]
	{112, 5},	//1:pr2_start	2:lioa[12]	3:pwm[12]
	{113, 5},	//1:pr2_ready	2:lioa[13]	3:pwm[13]	4:spi2_clk
	{114, 5},	//1:pr2_enable	2:lioa[14]	3:pwm[14]	4:spi2_miso
	{115, 5},	//1:pr2_shold	2:lioa[15]	3:pwm[15]	4:spi2_mosi
	{116, 5},	//1:pr2_data	2:lio_data[0]	3:uart1_rx	4:spi2_cs
	{117, 5},	//1:pr2_hsync	2:lio_data[1]	3:uart1_tx	4:spi3_clk
	{118, 5},	//1:pr3_enable	2:lio_data[2]	3:uart1_rts	4:spi3_miso
	{119, 5},	//1:pr3_shold	2:lio_data[3]	3:uart1_cts	4:spi3_mosi
	{120, 5},	//1:pr3_data	2:lio_data[4]	3:uart1_dsr	4:spi3_cs
	{121, 5},	//1:pr4_clk	2:lio_data[5]	3:uart1_dtr	4:spi4_clk
	{122, 5},	//1:pr4_start	2:lio_data[6]	3:uart1_dcd	4:spi4_miso
	{123, 5},	//1:pr4_ready	2:lio_data[7]	3:uart1_ri	4:spi4_mosi
	{124, 5},	//1:pr4_enable	2:lio_data[8]	4:spi4_cs
	{125, 5},	//1:pr4_shold	2:lio_data[9]	4:spi5_clk
	{126, 5},	//1:pr4_data	2:lio_data[10]	4:spi5_miso
	{127, 5},	//1:pr4_hsync	2:lio_data[11]	4:spi5_mosi
	{128, 5},	//1:pr5_enable	2:lio_data[12]	4:spi5_cs
	{129, 5},	//1:pr5_shold	2:lio_data[13]
	{130, 5},	//1:pr5_data	2:lio_data[14]
	{131, 5},	//1:pr6_clk	2:lio_data[15]
	{132, 5},	//1:pr6_start	2:lioa[16]
	{133, 5},	//1:pr6_ready	2:lioa[17]	4:can2_rx
	{134, 5},	//1:pr6_enable	2:lioa[18]	4:can2_tx
	{135, 5},	//1:pr6_shold	2:lioa[19]	4:can3_rx
	{136, 5},	//1:pr6_data	2:lioa[20]	4:can3_tx
	{137, 5},	//1:pr6_hsync	2:lioa[21]
	{138, 5},	//1:pr7_enable	2:lioa[22]
	{139, 5},	//1:pr7_shold	2:liocsn[0]
	{140, 5},	//1:pr7_data	2:liocsn[1]
	{141, 5},	//1:pix0_scl	2:liowrn
	{142, 5},	//1:pix0_sda	2:liordn
	{143, 5},	//4:sdio1_clk
	{144, 5},	//4:sdio1_cmd
	{145, 5},	//4:sdio1_d[0]
	{146, 5},	//4:sdio1_d[1]
	{147, 5},	//4:sdio1_d[2]
	{148, 5},	//4:sdio1_d[3]
	{149, 5},	//5:sdio_clk
	{150, 5},	//5:sdio_cmd
	{151, 5},	//5:sdio_d[0]
	{152, 5},	//5:sdio_d[1]
	{153, 5},	//5:sdio_d[2]
	{154, 5}	//5:sdio_d[3]
};

/* add all pins that you want to cfg. just like this, then call cfg_all_pin_multi() */
pin_cfgs_t can0_pin_cfgs[] = {
	{ 66, 5 },	//5:can0_rx	1:nand_rdy[2]	2:sda2		3:spi0_cs[1]
	{ 67, 5 },	//5:can0_tx	1:nand_ce[2]	2:scl2		3:spi1_cs[3]
};

#define USB0_AS_OTG	1	//USB0 cfg as OTG
#define LIO_16_BIT	1	//LIO cfg as 16bit mode
void ls1a500_devices_fixup(void)
{
	/* cfg all i2c as master */
	readb(LS1A500_I2C0_REG_BASE + 2) |= 0x20;
	readb(LS1A500_I2C0_REG_BASE + 0x802) |= 0x20;
	readb(LS1A500_I2C0_REG_BASE + 0x1002) |= 0x20;
	readb(LS1A500_I2C0_REG_BASE + 0x1802) |= 0x20;
	readb(LS1A500_I2C0_REG_BASE + 0x2002) |= 0x20;
	readb(LS1A500_I2C0_REG_BASE + 0x2802) |= 0x20;

	readl(LS1A500_GENERAL_CFG0) |= (1 << 9);	//hda pins use ac97

	readl(LS1A500_GENERAL_CFG1) |= (USB0_AS_OTG << 1) | (LIO_16_BIT << 28);
}


void ls1a500_gpio_out(int gpio_num, int val)
{
        set_pin_mode(gpio_num, 0);
        uint32_t addr_dir = LS1A500_GPIO_00_63_DIR + gpio_num / 64 * GPIO_SKIP_64_OFFSET;
        uint32_t addr_out = LS1A500_GPIO_00_63_OUT + gpio_num / 64 * GPIO_SKIP_64_OFFSET;
        int bit = gpio_num % 64;
        readq(addr_dir) &= ~(1ULL << bit);
        readq(addr_out) =  readq(addr_out) & ~(1ULL << bit) | ((unsigned long long)!!val << bit);
}

int ls1a500_gpio_in(int gpio_num)
{
        set_pin_mode(gpio_num, 0);
        uint32_t addr_dir = LS1A500_GPIO_00_63_DIR + gpio_num / 64 * GPIO_SKIP_64_OFFSET;
        uint32_t addr_in = LS1A500_GPIO_00_63_IN + gpio_num / 64 * GPIO_SKIP_64_OFFSET;
        int bit = gpio_num % 64;
        readq(addr_dir) |= (1ULL << bit);
	return !!(readq(addr_in) & (1ULL << bit));
}

static int cmd_ls1a500_gpio_out(struct cmd_tbl *cmdtp, int flag, int ac, char * const av[])
{
	if(ac != 3) {
		printf("gpio_out <gpio_num> <output_val>\n");
		return 0;
	}
	ls1a500_gpio_out(strtoul(av[1], NULL, 0), strtoul(av[2], NULL, 0));
	return 0;
}

int cmd_ls1a500_gpio_in(struct cmd_tbl *cmdtp, int flag, int ac, char * const av[])
{
	if(ac != 2) {
		printf("gpio_in <gpio_num>\n");
		return 0;
	}
	printf("gpio read val: %d\n", ls1a500_gpio_in(strtoul(av[1], NULL, 0)));
	return 0;
}

char pin_set[156] = {0};
int loop_set_pin(int from, int to, int val, int skip)
{
	int i;
	for (i = from; i <= to; i++) {	//check
		if (pin_set[i]) {
			if(skip)
				return -1;
		}
	}
	for (i = from; i <= to; i++) {
		set_pin_mode(i, val);
		pin_set[i] = 1;
	}
	return 0;
}

int cfg_func_multi(const char * name, int skip)
{
	int ret = 0;
	if (strstr(name, "can0")) {
		ret = loop_set_pin(66, 67, 5, skip);
	} else if (strstr(name, "can1")) {
		ret = loop_set_pin(68, 69, 5, skip);
	} else if (strstr(name, "can2")) {
		ret = loop_set_pin(133, 134, 4, skip);
	} else if (strstr(name, "can3")) {
		ret = loop_set_pin(135, 136, 4, skip);
	} else if (strstr(name, "gmac0")) {
		ret = loop_set_pin(88, 99, 5, skip);
	} else if (strstr(name, "gmac1")) {
		ret = loop_set_pin(48, 59, 1, skip);
	} else if (strstr(name, "hda")) {
		ret = loop_set_pin(36, 39, 5, skip);
		readl(LS1A500_GENERAL_CFG0) |= (1 << 9);	//hda pins use ac97
	} else if (strstr(name, "nand")) {
		ret = loop_set_pin(64, 75, 1, skip);
		ret = loop_set_pin(76, 83, 5, skip);
	} else if (strstr(name, "pwm")) {
		ret = loop_set_pin(84, 87, 5, skip);
		ret = loop_set_pin(105, 115, 3, skip);
	} else if (strstr(name, "sdio0")) {
		ret = loop_set_pin(149, 154, 5, skip);
	} else if (strstr(name, "sdio1")) {
		ret = loop_set_pin(143, 148, 4, skip);
	} else if (strstr(name, "serial")) {
		ret = loop_set_pin(60, 61, 5, skip);
	} else if (strstr(name, "uart0")) {
		ret = loop_set_pin(48, 55, 5, skip);
	} else if (strstr(name, "uart1")) {
		ret = loop_set_pin(116, 123, 3, skip);
	} else if (strstr(name, "uart3")) {
		ret = loop_set_pin(62, 63, 5, skip);
	} else if (strstr(name, "spi0")) {
		ret = loop_set_pin(40, 43, 5, skip);
		ret = loop_set_pin(64, 66, 3, skip);
	} else if (strstr(name, "spi1")) {
		ret = loop_set_pin(44, 47, 5, skip);
		ret = loop_set_pin(67, 69, 3, skip);
	} else if (strstr(name, "spi2")) {
		ret = loop_set_pin(113, 116, 4, skip);
	} else if (strstr(name, "spi3")) {
		ret = loop_set_pin(117, 120, 4, skip);
	} else if (strstr(name, "spi4")) {
		ret = loop_set_pin(121, 124, 4, skip);
	} else if (strstr(name, "spi5")) {
		ret = loop_set_pin(125, 128, 4, skip);
	} else if (strstr(name, "i2c0")) {
		ret = loop_set_pin(64, 65, 1, skip);
	} else if (strstr(name, "pix0")) {
		ret = loop_set_pin(141, 142, 1, skip);
	} else if (strstr(name, "pix1")) {
		ret = loop_set_pin(62, 63, 1, skip);
	} else if (strstr(name, "lpc")) {
		ret = loop_set_pin(70, 75, 5, skip);
		readl(LS1A500_LPC_CFG1_REG) |= (1 << 31);	// Access Firmware Memory
#define LPC_4M_FLASH
#ifdef LPC_4M_FLASH
		readl(LS1A500_GENERAL_CFG2) &= ~(1 << 5);	// Select 4M LPC flash
#endif
	} else if (strstr(name, "lio")) {
		ret = loop_set_pin(100, 142, 2, skip);
		/* set lio 16bit mode, and set delay to make sure that rdn is morethan 35ns */
		readl(LS1A500_GENERAL_CFG1) |= (1 << 28) | (8 << 23);
	} else {
		printf("unknow device name: %s\n", name);
	}
	if(ret == -1)
		printf("note: conflict for resetting dev: %s\n", name);
	return ret;
}

int cmd_set_dev_pins(struct cmd_tbl *cmdtp, int flag, int ac, char * const av[])
{
	if(ac != 2) {
		printf("set_dev_pins <dev_name>\n");
		printf("\teg: set_dev_pins uart0/spi1/lio ...\n");
		return 0;
	}
	cfg_func_multi(av[1], 0);
	return 0;
}

U_BOOT_CMD(
	gpio_out,    3,    1,    cmd_ls1a500_gpio_out,
	"set gpio output",
	"\n    - set gpio out put\n"
	"gpio_out <gpio num> <val>"
);

U_BOOT_CMD(
	gpio_in,    2,    1,    cmd_ls1a500_gpio_in,
	"set gpio in, and get val",
	"\n    - set_dev_pins <dev_name>\n"
	"set_dev_pins uart0/spi1/lio ...\n"
);

U_BOOT_CMD(
	set_dev_pins,    2,    1,    cmd_set_dev_pins,
	"cfg multi for devices",
	"\n    - cfg multi for devices\n"
	"gpio_in <gpio num>"
);
