/*************************
 * this file i2c address use 64 bit access
 * this code used for control voltage
*************************/

#define PRER_LO_REG			(0x0)
#define PRER_HI_REG			(0x1)
#define CTR_REG				(0x2)
#define TXR_REG				(0x3)
#define RXR_REG				(0x3)
#define CR_REG				(0x4)
#define SR_REG				(0x4)

#define CR_START			0x80
#define CR_STOP				0x40
#define CR_READ				0x20
#define CR_WRITE			0x10
#define CR_ACK				0x8
#define CR_IACK				0x1

#define SR_NOACK			0x80
#define SR_BUSY				0x40
#define SR_AL				0x20
#define SR_TIP				0x2
#define SR_IF				0x1

#define MPS_ADDR			(0x60 << 1)

/*
 * use register:
 *	v0, v1, s1
 *	input: a3
 *	a3: i2c register base address
 *	s1: (node_id << 44) do not change it
 */
LEAF(ls_v_i2cinit)
	.set    push
	.set    noreorder
	.set    mips3
	//LPB clock_a,SCL clock_s,prescale = clock_a / (4 * clock_s);
	or	a3, s1
	lb	v1, CTR_REG(a3)
	and	v1, ~(1 << 7)
	sb	v1, CTR_REG(a3)

#ifdef BONITO_100M
	li	v1, 0x53 //100M
#else
	li	v1, 0x63 //25M
#endif
	sb	v1, PRER_LO_REG(a3)

#ifdef BONITO_100M
	li	v1, 0x2 //
#else
	li	v1, 0x0 //25M
#endif
	sb	v1, 0x1(a3)

	lb	v1, CTR_REG(a3)
	or	v1, (1 << 7)
	sb	v1, CTR_REG(a3)

	jr      ra
	nop
	.set pop
END(ls_v_i2cinit)

LEAF(ls_v_i2cwrite)
/*
 * use register:
 *	v0, v1, s1
 *	a0, a1, a3
 *	input: a0,a1,a2
 *	a0: device ID
 *	a1: register offset/command, [16-31] store write count now 1 mean 1 byte default 0/2  byte 2
 *	a2: configure value
 *	a3: i2c register base address
 *	v0: return value
 *	s1: (node_id << 44) do not change it
 */
	.set    push
	.set    noreorder
	.set    mips3
/*i2c_send_addr*/
	/* load device address */
	move	v1, a0
	or	a3, s1
	sb	v1, TXR_REG(a3)

	/* send start frame */
	li	v1, CR_START | CR_WRITE
	sb	v1, CR_REG(a3)

	/* wait send finished */
//	i2c_wait_tip
1:
	lbu	v1, SR_REG(a3)
	andi	v1, v1, SR_TIP
	bnez	v1, 1b
	nop

	//check ACK
	lbu	v1, SR_REG(a3)
	andi	v1, v1, SR_NOACK
	bnez	v1, 3f
	nop

	/* load data(offset/command) to be send */
	move	v1, a1
	sb	v1, TXR_REG(a3)

	/* send data frame */
	li	v1, CR_WRITE
	sb	v1, CR_REG(a3)

	/* wait send finished */
//	i2c_wait_tip
1:
	lbu	v1, SR_REG(a3)
	andi	v1, v1, SR_TIP
	bnez	v1, 1b
	nop

	//check ACK
	lbu	v1, SR_REG(a3)
	andi	v1, v1, SR_NOACK
	bnez	v1, 3f
	nop
/*i2c_send_addr*/

/* i2c write max data is word*/
/*i2c tx byte*/
	/* load configure value */
	move	v1, a2
	sb	v1, TXR_REG(a3)

	/* send start frame */
	li	v1, CR_WRITE
	sb	v1, CR_REG(a3)

	/* wait send finished */
//	i2c_wait_tip
1:
	lbu	v1, SR_REG(a3)
	andi	v1, v1, SR_TIP
	bnez	v1, 1b
	nop

	//check ACK
	lbu	v1, SR_REG(a3)
	andi	v1, v1, SR_NOACK
	bnez	v1, 3f
	nop
	/*get count*/
	and v1, a1, (1 << 16)
	bnez v1, 3f
	nop
/*i2c tx byte*/
/*i2c tx byte*/
	/* load configure value */
	move	v1, a2
	srl	v1, v1, 8
	sb	v1, TXR_REG(a3)

	/* send start frame */
	li	v1, CR_WRITE
	sb	v1, CR_REG(a3)

	/* wait send finished */
//	i2c_wait_tip
1:
	lbu	v1, SR_REG(a3)
	andi	v1, v1, SR_TIP
	bnez	v1, 1b
	nop

	//check ACK
	lbu	v1, SR_REG(a3)
	andi	v1, v1, SR_NOACK
	bnez	v1, 3f
	nop

	b	4f
	nop
/*i2c tx byte*/
3:
/* i2c_stop */
	/* free i2c bus */
	li	v1, CR_STOP
	sb	v1, CR_REG(a3)
1:
	lbu	v1, SR_REG(a3)
	andi	v1, v1, SR_BUSY
	bnez	v1, 1b
	nop
	li	v0, 0 //return value check this function
4:
	jr	ra
	nop
	.set pop
END(ls_v_i2cwrite)

LEAF(ls_v_i2cread)
/*
 * use register:
 *	v0, v1
 *	a0, a1
 *	input: a0,a1
 *	a0: device ID
 *	a1: register offset
 *	v0: return value
 *
 */
    .set    push
    .set    noreorder
    .set    mips3
/*i2c_send_addr*/
/*send device addr*/
	/* load device address */
	move	v1, a0
	or	a3, s1
	sb	v1, TXR_REG(a3)

	/* send start frame */
	li	v1, CR_START | CR_WRITE
	sb	v1, CR_REG(a3)

	/* wait send finished */
//	i2c_wait_tip
1:
	lbu	v1, SR_REG(a3)
	andi	v1, v1, SR_TIP
	bnez	v1, 1b
	nop

	//check ACK
	lbu	v1, SR_REG(a3)
	andi	v1, v1, SR_NOACK
	bnez	v1, 3f
	nop
/*send device addr*/
	/* load data(offset/command) to be send */
	move	v1, a1
	sb	v1, TXR_REG(a3)

	/* send data frame */
	li	v1, CR_WRITE
	sb	v1, CR_REG(a3)

	/* wait send finished */
//	i2c_wait_tip
1:
	lbu	v1, SR_REG(a3)
	andi	v1, v1, SR_TIP
	bnez	v1, 1b
	nop

	//check ACK
	lbu	v1, SR_REG(a3)
	andi	v1, v1, SR_NOACK
	bnez	v1, 3f
	nop
/*i2c_send_addr*/

/* i2c_read_word*/
/*send device addr*/
	/* load device address (dev_addr | 0x1/READ) */
	ori	v1, a0, 0x1
	sb	v1, TXR_REG(a3)

	/* send start frame */
	li	v1, CR_START | CR_WRITE
	sb	v1, CR_REG(a3)

	/* wait send finished */
//	i2c_wait_tip
1:
	lbu	v1, SR_REG(a3)
	andi	v1, v1, SR_TIP
	bnez	v1, 1b
	nop

	 //check ACK
	lbu	v1, SR_REG(a3)
	andi	v1, v1, SR_NOACK
	bnez	v1, 3f
	nop
/*send device addr*/

	/* receive data from fifo */
	li	v1, CR_READ
	sb	v1, CR_REG(a3)

//	i2c_wait_tip
1:
	lbu	v1, SR_REG(a3)
	andi	v1, v1, SR_TIP
	bnez	v1, 1b
	nop

	/* read data from fifo */
	lbu	a1, RXR_REG(a3)

	/* receive data from fifo */
	li	v1, CR_READ | CR_ACK
	sb	v1, CR_REG(a3)

//	i2c_wait_tip
1:
	lbu	v1, SR_REG(a3)
	andi	v1, v1, SR_TIP
	bnez	v1, 1b
	nop

	/* read data from fifo */
	lbu	v1, RXR_REG(a3)
	sll	v1, v1,0x8
	addu a1, a1, v1
3:
/* i2c_stop */
	/* free i2c bus */
	li	v1, CR_STOP
	sb	v1, CR_REG(a3)
1:
	lbu	v1, SR_REG(a3)
	andi	v1, v1, SR_BUSY
	bnez	v1, 1b
	nop

	move	v0, a1

	jr	ra
	nop
    .set pop
END(ls_v_i2cread)

LEAF(v_n_ctrl)
/*
 * v0 give the voltage level
 * use register:
 * a0,a1,a2,k0
 * a0: device ID
 * a1: command
 * a2: command value
 * k0: save ra
 * v0: return value
 */
	.set    push
	.set    noreorder
	.set    mips3
	move	k0, ra

	dli	a3, LS1A500_I2C0_REG_BASE
	bal	ls_v_i2cinit
	nop
/* enable write */
	li	a0, MPS_ADDR
	li	a1, 0x1
	li	a2, 0xe0
	dli	a3, LS1A500_I2C0_REG_BASE
	bal	ls_v_i2cwrite
	nop
	li	v1, CR_STOP
	sb	v1, CR_REG(a3)
1:
	lbu	v1, SR_REG(a3)
	andi	v1, v1, SR_BUSY
	bnez	v1, 1b
	nop
#define INPUT_PARAM
#ifdef INPUT_PARAM  //this code used for debug

	PRINTSTR("\r\nInput parameter 0.9v:0x1e, 0.95v:0x23, 1.0v:0x28, 1.05v:0x2d, 1.1v:0x3d, 1.15v:0x42, 1.15v:0x37, 1.20v:0x3c")
	PRINTSTR("\r\nInput parameter 1.25v:0x41, 1.3v:0x46")

	bal	inputaddress
	nop
	move	a2, v0
#else
	//li	a2, 0x1e //0.9v
	//li	a2, 0x23 //0.95v
	//li	a2, 0x28 //1.0v
	//li	a2, 0x2d //1.05v
	//li	a2, 0x32 //1.1v
	//li	a2, 0x37 //1.15v
	//li	a2, 0x3c //1.2v
	li	a2, 0x41 //1.25v
	//li	a2, 0x46 //1.3v
#endif
	li	a0, MPS_ADDR
	li	a1, 0x0 //VSEL
	dli	a3, LS1A500_I2C0_REG_BASE

	bal	ls_v_i2cwrite
	nop
	beqz	v0, 2f
	nop

	li	v1, CR_STOP
	sb	v1, CR_REG(a3) //if not change a3 == a3 or s1
1:
	lbu	v1, SR_REG(a3)
	andi	v1, v1, SR_BUSY
	bnez	v1, 1b
	nop

	b 3f
	nop
2:
	TTYDBG("v ctrl err\r\n")
	move	ra,k0
	jr	ra
	nop
3:
	TTYDBG("v ctrl end\r\n")
	move	ra,k0
	jr	ra
	nop
	.set pop
END(v_n_ctrl)

LEAF(v_n_ctrl_read)
/*
 * use register:
 * a0,a1,k0
 * a0: device ID
 * a1: command
 * k0: save ra
 * v0: return value
 */
	.set    push
	.set    noreorder
	.set    mips3

	move	k0, ra

	dli	a3, LS1A500_I2C0_REG_BASE
	bal	ls_v_i2cinit
	nop
	li	a0, MPS_ADDR
	li	a1, 0x0 //READ_VOUT
	bal	ls_v_i2cread
	nop
	move	a0, v0
	bal	hexserial
	nop
	TTYDBG("\r\n")

	move	ra, k0
	jr	ra
	nop
	.set pop
END(v_n_ctrl_read)

