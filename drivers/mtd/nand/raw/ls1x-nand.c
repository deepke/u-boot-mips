#include<linux/types.h>
#include<linux/mtd/mtd.h>
#include<linux/mtd/nand.h>
#include<linux/mtd/partitions.h>
#include <linux/mtd/rawnand.h>
#include<malloc.h>
#include <linux/compiler.h>
#include <dm.h>
#include <asm/io.h>
 
extern void *
dma_alloc_coherent(void *dev, size_t size, dma_addr_t *dma_handle,
		   gfp_t flag);

#define MAX_BUFF_SIZE	4096
#define PAGE_SHIFT      12
#ifdef CONFIG_TARGET_LS1B
#define NO_SPARE_ADDRH(x)   ((x) >> (32 - (PAGE_SHIFT - 1 )))   
#define NO_SPARE_ADDRL(x)   ((x) << (PAGE_SHIFT - 1))
#define SPARE_ADDRH(x)      ((x) >> (32 - (PAGE_SHIFT )))   
#define SPARE_ADDRL(x)      ((x) << (PAGE_SHIFT ))
#elif defined(CONFIG_TARGET_LS1A) || defined(CONFIG_TARGET_LS232)
#define NO_SPARE_ADDRH(x)   (x)   
#define NO_SPARE_ADDRL(x)   0
#define SPARE_ADDRH(x)      (x)   
#define SPARE_ADDRL(x)      0  
#endif 
#define ALIGN_DMA(x)       (((x)+ 3)/4)

#ifdef CONFIG_TARGET_LS232
#define USE_POLL
#endif
#ifdef USE_POLL
#define complete(...)
#define wait_for_completion_timeout(...)
#define init_completion(...)
#define request_irq(...) (0)
#define free_irq(...) 
#endif

#define STATUS_TIME_LOOP_R  60 
#define STATUS_TIME_LOOP_WS  400  
#define STATUS_TIME_LOOP_WM  60  
#define STATUS_TIME_LOOP_E  100  

#define NAND_CMD        0x1
#define NAND_ADDRL      0x2
#define NAND_ADDRH      0x4
#define NAND_TIMING     0x8
#define NAND_IDL        0x10
#define NAND_STATUS_IDL 0x20
#define NAND_PARAM      0x40
#define NAND_OP_NUM     0X80
#define NAND_CS_RDY_MAP 0x100

#define DMA_ORDERAD     0x1
#define DMA_SADDR       0x2
#define DMA_DADDR       0x4
#define DMA_LENGTH      0x8
#define DMA_STEP_LENGTH 0x10
#define DMA_STEP_TIMES  0x20
#define DMA_CMD         0x40

#if defined(CONFIG_TARGET_LS1A) || defined(CONFIG_TARGET_LS232)
#define NAND_ECC_OFF	0
#define NAND_ECC_ON	1
#define RAM_OP_OFF	0
#define RAM_OP_ON	1
#endif

#define  _NAND_IDL      ( *((volatile unsigned int*)(info->mmio_base+0x10)))
#define  _NAND_IDH       (*((volatile unsigned int*)(info->mmio_base+0x14)))
#define  _NAND_BASE      info->mmio_base
#define  _NAND_SET_REG(x,y)   do{*((volatile unsigned int*)(_NAND_BASE+x)) = (y);}while(0)                           
#define  _NAND_READ_REG(x,y)  do{(y) =  *((volatile unsigned int*)(_NAND_BASE+x));}while(0) 

#define _NAND_TIMING_TO_READ    _NAND_SET_REG(0xc,0x205)
#define _NAND_TIMING_TO_WRITE   _NAND_SET_REG(0xc,0x205)


enum{
    ERR_NONE        = 0,
    ERR_DMABUSERR   = -1,
    ERR_SENDCMD     = -2,
    ERR_DBERR       = -3,
    ERR_BBERR       = -4,
};
enum{
    STATE_READY = 0,
    STATE_BUSY  ,
};

struct ls1g_nand_platform_data{
        int enable_arbiter;
        struct mtd_partition *parts;
        unsigned int nr_parts;
};
struct ls1g_nand_cmdset {
        uint32_t    cmd_valid:1;	//0
	uint32_t    read:1;		//1
	uint32_t    write:1;		//2
	uint32_t    erase_one:1;	//3
	uint32_t    erase_con:1;	//4
	uint32_t    read_id:1;		//5
	uint32_t    reset:1;		//6
	uint32_t    read_sr:1;		//7
	uint32_t    op_main:1;		//8
	uint32_t    op_spare:1;		//9
	uint32_t    done:1;		//10
#if defined(CONFIG_TARGET_LS1A) || defined(CONFIG_TARGET_LS232)
	uint32_t    ecc_rd:1;		//11
	uint32_t    ecc_wr:1;		//12
 	uint32_t    int_en:1;		//13
	uint32_t    resv14:1;		//14
	uint32_t    ram_op:1;		//15
#elif CONFIG_TARGET_LS1B
	uint32_t    resv1:5;//11-15 reserved
#endif
        uint32_t    nand_rdy:4;//16-19
        uint32_t    nand_ce:4;//20-23
#if defined(CONFIG_TARGET_LS1A) || defined(CONFIG_TARGET_LS232)
	uint32_t    ecc_dma_req:1;         //24
	uint32_t    nul_dma_req:1;         //25
	uint32_t resv26 :6;//26-31 reserved
#else
        uint32_t    resv26:8;//24-31 reserved
#endif
};
struct ls1g_nand_dma_desc{
        uint32_t    orderad;
        uint32_t    saddr;
        uint32_t    daddr;
        uint32_t    length;
        uint32_t    step_length;
        uint32_t    step_times;
        uint32_t    cmd;
};
struct ls1g_nand_dma_cmd{
        uint32_t    dma_int_mask:1;
        uint32_t    dma_int:1;
        uint32_t    dma_sl_tran_over:1;
        uint32_t    dma_tran_over:1;
        uint32_t    dma_r_state:4;
        uint32_t    dma_w_state:4;
        uint32_t    dma_r_w:1;
        uint32_t    dma_cmd:2;
        uint32_t    revl:17;
};
struct ls1g_nand_desc{
        uint32_t    cmd;
        uint32_t    addrl;
        uint32_t    addrh;
        uint32_t    timing;
        uint32_t    idl;//readonly
        uint32_t    status_idh;//readonly
        uint32_t    param;
        uint32_t    op_num;
        uint32_t    cs_rdy_map;
};
struct ls1g_nand_info {
	struct nand_chip	nand_chip;

        /* MTD data control*/
	unsigned int 		buf_start;
	unsigned int		buf_count;
        /* NAND registers*/
	void __iomem		*mmio_base;
        struct ls1g_nand_desc   nand_regs;
        unsigned int            nand_addrl;
        unsigned int            nand_addrh;
        unsigned int            nand_timing;
        unsigned int            nand_op_num;
        unsigned int            nand_cs_rdy_map;
        unsigned int            nand_cmd;

	/* DMA information */

        struct ls1g_nand_dma_desc  dma_regs;
        void            	*order_reg_addr;  
        unsigned int            dma_orderad;
        unsigned int            dma_saddr;
        unsigned int            dma_daddr;
        unsigned int            dma_length;
        unsigned int            dma_step_length;
        unsigned int            dma_step_times;
        unsigned int            dma_cmd;
        void			*drcmr_dat;//dma descriptor address;
	dma_addr_t 		drcmr_dat_phys;
        size_t                  drcmr_dat_size;
	unsigned char		*data_buff;//dma data buffer;
	dma_addr_t 		data_buff_phys;
	size_t			data_buff_size;
        unsigned long           cac_size;
        unsigned long           num;
        unsigned long           size;
        struct timer_list       test_timer;
        void            	*dma_ask;
        unsigned int            dma_ask_phy;

	/* relate to the command */
	unsigned int		state;
	size_t			data_size;	/* data size in FIFO */
        unsigned int            cmd;
        unsigned int            page_addr;
        unsigned int            seqin_column;
        unsigned int            seqin_page_addr;
        unsigned int            timing_flag;
        unsigned int            timing_val;
};


#define show_data_debug  0
#define show_debug(x,y)     show_debug_msk(x,y)
#define show_debug_msk(x,y)   do{ if(show_data_debug) {printk(KERN_ERR "%s:\n",__func__);show_data(x,y);} }while(0)

static void show_data(void * base,int num)
{
    int i=0;
    unsigned char *arry=( unsigned char *) base;
    for(i=0;i<num;i++){
        if(!(i % 32)){
            printk(KERN_ERR "\n");
        }
        if(!(i % 16)){
            printk("  ");
        }
        printk("%02x ",arry[i]);
    }
    printk(KERN_ERR "\n");
    
}

static int ls1g_nand_ecc_calculate(struct mtd_info *mtd,
		const uint8_t *dat, uint8_t *ecc_code)
{
	return 0;
}
static int ls1g_nand_ecc_correct(struct mtd_info *mtd,
		uint8_t *dat, uint8_t *read_ecc, uint8_t *calc_ecc)
{
	struct ls1g_nand_info *info = mtd->priv;
	/*
	 * Any error include ERR_SEND_CMD, ERR_DBERR, ERR_BUSERR, we
	 * consider it as a ecc error which will tell the caller the
	 * read fail We have distinguish all the errors, but the
	 * nand_read_ecc only check this function return value
	 */
	return 0;
}

static void ls1g_nand_ecc_hwctl(struct mtd_info *mtd, int mode)
{
	return;
}




static int ls1g_nand_waitfunc(struct mtd_info *mtd, struct nand_chip *this)
{
        struct ls1g_nand_info *info = mtd->priv;
     unsigned char status;
    udelay(50);
#ifdef CONFIG_TARGET_LS232
    status = _NAND_IDH>>16;
    printk("status=0x%x\n", status);
    return status;
#else
    return 0;
#endif
}
static void ls1g_nand_select_chip(struct mtd_info *mtd, int chip)
{
	return;
}
static int ls1g_nand_dev_ready(struct mtd_info *mtd)
{
	return 1;
}
static void ls1g_nand_read_buf(struct mtd_info *mtd, uint8_t *buf, int len)
{
        struct ls1g_nand_info *info = mtd->priv;
	struct ls1g_nand_dma_desc *dma_base = (volatile struct ls1g_nand_dma_desc *)(info->drcmr_dat);
	struct ls1g_nand_desc *nand_base = (struct ls1g_nand_desc *)(info->mmio_base);
        int i,real_len = min_t(size_t, len, info->buf_count - info->buf_start);
	memcpy(buf, info->data_buff + info->buf_start, real_len);

        show_debug(info->data_buff,0x40);

        info->buf_start += real_len;
}
static u16 ls1g_nand_read_word(struct mtd_info *mtd)
{
        struct ls1g_nand_info *info = mtd->priv;
        u16 retval = 0xFFFF;
        if(!(info->buf_start & 0x1) && info->buf_start < info->buf_count){
            retval = *(u16 *)(info->data_buff + info->buf_start);
        }
        info->buf_start += 2;
        return retval;


}
static uint8_t ls1g_nand_read_byte(struct mtd_info *mtd)
{
        struct ls1g_nand_info *info = mtd->priv;
	char retval = 0xFF;
	if (info->buf_start < info->buf_count)
		retval = info->data_buff[(info->buf_start)++];
	return retval;
}
static void ls1g_nand_write_buf(struct mtd_info *mtd,const uint8_t *buf, int len)
{
        int i;
        struct ls1g_nand_info *info = mtd->priv;
	int real_len = min_t(size_t, len, info->buf_count - info->buf_start);

	memcpy(info->data_buff + info->buf_start, buf, real_len);
            show_debug(info->data_buff,0x20);
	info->buf_start += real_len;
}
static void ls1g_nand_cmdfunc(struct mtd_info *mtd, unsigned command,int column, int page_addr);
static void ls1g_nand_init_mtd(struct mtd_info *mtd,struct ls1g_nand_info *info)
{
	struct nand_chip *this = &info->nand_chip;

	this->options = 8;//(f->flash_width == 16) ? NAND_BUSWIDTH_16: 0;

	this->waitfunc		= ls1g_nand_waitfunc;
	this->select_chip	= ls1g_nand_select_chip;
	this->dev_ready		= ls1g_nand_dev_ready;
	this->cmdfunc		= ls1g_nand_cmdfunc;
	this->read_word		= ls1g_nand_read_word;
	this->read_byte		= ls1g_nand_read_byte;
	this->read_buf		= ls1g_nand_read_buf;
	this->write_buf		= ls1g_nand_write_buf;

#if 0
        this->ecc.mode		= NAND_ECC_NONE;
#else
	this->ecc.mode		= NAND_ECC_SOFT;
#endif
	this->ecc.hwctl		= ls1g_nand_ecc_hwctl;
	this->ecc.calculate	= ls1g_nand_ecc_calculate;
	this->ecc.correct	= ls1g_nand_ecc_correct;

}
#define write_z_cmd  do{                                    \
            *((volatile unsigned int *)(info->mmio_base)) = 0;   \
            *((volatile unsigned int *)(info->mmio_base)) = 0;   \
            *((volatile unsigned int *)(info->mmio_base)) = 400; \
    }while(0)
static int nand_num=0;

static unsigned ls1g_nand_status(struct ls1g_nand_info *info)
{
    return(*((volatile unsigned int*)info->mmio_base) & (0x1<<10));
}
/*
 *  flags & 0x1   orderad
 *  flags & 0x2   saddr
 *  flags & 0x4   daddr
 *  flags & 0x8   length
 *  flags & 0x10  step_length
 *  flags & 0x20  step_times
 *  flags & 0x40  cmd
 ***/
static void dma_setup(unsigned int flags,struct ls1g_nand_info *info)
{
    struct ls1g_nand_dma_desc *dma_base = (volatile struct ls1g_nand_dma_desc *)(info->drcmr_dat);
    int status_time;
    dma_base->orderad = (flags & DMA_ORDERAD)== DMA_ORDERAD ? info->dma_regs.orderad : info->dma_orderad;
    dma_base->saddr = (flags & DMA_SADDR)== DMA_SADDR ? info->dma_regs.saddr : info->dma_saddr;
    dma_base->daddr = (flags & DMA_DADDR)== DMA_DADDR ? info->dma_regs.daddr : info->dma_daddr;
    dma_base->length = (flags & DMA_LENGTH)== DMA_LENGTH ? info->dma_regs.length: info->dma_length;
    dma_base->step_length = (flags & DMA_STEP_LENGTH)== DMA_STEP_LENGTH ? info->dma_regs.step_length: info->dma_step_length;
    dma_base->step_times = (flags & DMA_STEP_TIMES)== DMA_STEP_TIMES ? info->dma_regs.step_times: info->dma_step_times;
    dma_base->cmd = (flags & DMA_CMD)== DMA_CMD ? info->dma_regs.cmd: info->dma_cmd;

    if((dma_base->cmd)&(0x1 << 12)){    
        flush_dcache_range((unsigned long)info->data_buff, (unsigned long)info->data_buff + info->cac_size);
    }

    *(volatile unsigned int *)info->order_reg_addr = ((unsigned int )info->drcmr_dat_phys) | 0x1<<3;
    while (*(volatile unsigned int *)info->order_reg_addr & 0x8 );

    while(!(ls1g_nand_status(info)));
    info->state = STATE_READY;
}
/**
 *  flags & 0x1     cmd
 *  flags & 0x2     addrl
 *  flags & 0x4     addrh
 *  flags & 0x8     timing
 *  flags & 0x10    idl
 *  flags & 0x20    status_idh
 *  flags & 0x40    param
 *  flags & 0x80    op_num
 *  flags & 0x100   cs_rdy_map
 ****/
static void nand_setup(unsigned int flags ,struct ls1g_nand_info *info)
{
    int i,val1,val2,val3;
    struct ls1g_nand_desc *nand_base = (struct ls1g_nand_desc *)(info->mmio_base);
    nand_base->cmd = 0;
    nand_base->addrl = (flags & NAND_ADDRL)==NAND_ADDRL ? info->nand_regs.addrl: info->nand_addrl;

    nand_base->addrh = (flags & NAND_ADDRH)==NAND_ADDRH ? info->nand_regs.addrh: info->nand_addrh;

    nand_base->timing = (flags & NAND_TIMING)==NAND_TIMING ? info->nand_regs.timing: info->nand_timing;

    nand_base->op_num = (flags & NAND_OP_NUM)==NAND_OP_NUM ? info->nand_regs.op_num: info->nand_op_num;
    nand_base->cs_rdy_map = (flags & NAND_CS_RDY_MAP)==NAND_CS_RDY_MAP ? info->nand_regs.cs_rdy_map: info->nand_cs_rdy_map;
    nand_base->param = ((nand_base->param) & 0xc000ffff) | (nand_base->op_num << 16);

    if(flags & NAND_CMD){
            nand_base->cmd = (info->nand_regs.cmd) & (~0xff);
            nand_base->cmd = info->nand_regs.cmd;
    }
    else
        nand_base->cmd = info->nand_cmd;
}
static void ls1g_nand_cmdfunc(struct mtd_info *mtd, unsigned command,int column, int page_addr)
{
        struct ls1g_nand_info *info = mtd->priv;
        unsigned cmd_prev;
        int status_time,page_prev;
        cmd_prev = info->cmd;
        page_prev = info->page_addr;
        info->cmd = command;
        info->page_addr = page_addr;
        switch(command){
            case NAND_CMD_READOOB:
                if(info->state == STATE_BUSY){
                    printk("nandflash chip if busy,dma_base->orderad...\n");
                    return;
                }

                info->state = STATE_BUSY; 
                info->buf_count = mtd->oobsize;
                info->buf_start = 0;
                info->cac_size = info->buf_count;
                if(info->buf_count <=0 )
                    break;
		flush_dcache_range((unsigned long)(info->data_buff), (unsigned long)(info->data_buff)+info->cac_size);
		info->dma_regs.cmd = 0;
                /*nand regs set*/
                info->nand_regs.addrh =  SPARE_ADDRH(page_addr);
                info->nand_regs.addrl = SPARE_ADDRL(page_addr) + mtd->writesize;
                info->nand_regs.op_num = info->buf_count;
               /*nand cmd set */ 
                info->nand_regs.cmd = 0; 
#if defined(CONFIG_TARGET_LS1A) || defined(CONFIG_TARGET_LS232)
				((struct ls1g_nand_cmdset*)&(info->nand_regs.cmd))->int_en = 1;
                ((struct ls1g_nand_cmdset*)&(info->nand_regs.cmd))->ram_op = RAM_OP_OFF;
                ((struct ls1g_nand_cmdset*)&(info->nand_regs.cmd))->ecc_rd = NAND_ECC_OFF;
                ((struct ls1g_nand_cmdset*)&(info->nand_regs.cmd))->ecc_wr = NAND_ECC_OFF;
#endif
                ((struct ls1g_nand_cmdset*)&(info->nand_regs.cmd))->read = 1;
                ((struct ls1g_nand_cmdset*)&(info->nand_regs.cmd))->op_spare = 1;
                ((struct ls1g_nand_cmdset*)&(info->nand_regs.cmd))->cmd_valid = 1;
                /*dma regs config*/
                info->dma_regs.length =ALIGN_DMA(info->buf_count);
                ((struct ls1g_nand_dma_cmd *)&(info->dma_regs.cmd))->dma_int_mask = 1;
                /*dma GO set*/       
                nand_setup(NAND_ADDRL|NAND_ADDRH|NAND_OP_NUM|NAND_CMD,info);
                dma_setup(DMA_LENGTH|DMA_CMD,info);
                break;
            case NAND_CMD_READ0:
                if(info->state == STATE_BUSY){
                    printk("nandflash chip if busy...\n");
                    return;
                }
                info->state = STATE_BUSY;
                info->buf_count = mtd->oobsize + mtd->writesize ;
                info->buf_start =  0 ;
                info->cac_size = info->buf_count;
                if(info->buf_count <=0 )
                    break;
		flush_dcache_range((unsigned long)(info->data_buff), (unsigned long)(info->data_buff)+info->cac_size);
                info->nand_regs.addrh = SPARE_ADDRH(page_addr);
                info->nand_regs.addrl = SPARE_ADDRL(page_addr);
                info->nand_regs.op_num = info->buf_count;
               /*nand cmd set */ 
                info->nand_regs.cmd = 0; 
                info->dma_regs.cmd = 0;
#if defined(CONFIG_TARGET_LS1A) || defined(CONFIG_TARGET_LS232)
				((struct ls1g_nand_cmdset*)&(info->nand_regs.cmd))->int_en = 1;
                ((struct ls1g_nand_cmdset*)&(info->nand_regs.cmd))->ram_op = RAM_OP_OFF;
                ((struct ls1g_nand_cmdset*)&(info->nand_regs.cmd))->ecc_rd = NAND_ECC_OFF;
                ((struct ls1g_nand_cmdset*)&(info->nand_regs.cmd))->ecc_wr = NAND_ECC_OFF;
#endif
                ((struct ls1g_nand_cmdset*)&(info->nand_regs.cmd))->read = 1;
                ((struct ls1g_nand_cmdset*)&(info->nand_regs.cmd))->op_spare = 1;
                ((struct ls1g_nand_cmdset*)&(info->nand_regs.cmd))->op_main = 1;
                ((struct ls1g_nand_cmdset*)&(info->nand_regs.cmd))->cmd_valid = 1; 
                /*dma regs config*/
                info->dma_regs.length = ALIGN_DMA(info->buf_count);
                ((struct ls1g_nand_dma_cmd *)&(info->dma_regs.cmd))->dma_int_mask = 1;
                nand_setup(NAND_ADDRL|NAND_ADDRH|NAND_OP_NUM|NAND_CMD,info);
                dma_setup(DMA_LENGTH|DMA_CMD,info);
                break;
            case NAND_CMD_SEQIN:
                if(info->state == STATE_BUSY){
                    printk("nandflash chip if busy...\n");
                    return;
                }
                info->state = STATE_BUSY;
                info->buf_count = mtd->oobsize + mtd->writesize - column;
                info->buf_start = 0;
                info->seqin_column = column;
                info->seqin_page_addr = page_addr;
                break;
            case NAND_CMD_PAGEPROG:
                if(info->state == STATE_BUSY){
                    printk("nandflash chip if busy...\n");
                    return;
                }
                info->state = STATE_BUSY;
                if(cmd_prev != NAND_CMD_SEQIN){
                    printk("Prev cmd don't complete...\n");
                    break;
                }
                if(info->buf_count <= 0 )
                    break;
		info->cac_size = info->buf_count;
                /*nand regs set*/
                info->nand_regs.addrh =  SPARE_ADDRH(info->seqin_page_addr);
                info->nand_regs.addrl =  SPARE_ADDRL(info->seqin_page_addr) + info->seqin_column;
                info->nand_regs.op_num = info->buf_start;
                /*nand cmd set */ 
                info->nand_regs.cmd = 0; 
                info->dma_regs.cmd = 0;
#if defined(CONFIG_TARGET_LS1A) || defined(CONFIG_TARGET_LS232)
				((struct ls1g_nand_cmdset*)&(info->nand_regs.cmd))->int_en = 1;
                ((struct ls1g_nand_cmdset*)&(info->nand_regs.cmd))->ram_op = RAM_OP_OFF;
                ((struct ls1g_nand_cmdset*)&(info->nand_regs.cmd))->ecc_rd = NAND_ECC_OFF;
                ((struct ls1g_nand_cmdset*)&(info->nand_regs.cmd))->ecc_wr = NAND_ECC_OFF;
#endif
                ((struct ls1g_nand_cmdset*)&(info->nand_regs.cmd))->write = 1;
                if(info->seqin_column < mtd->writesize)
                    ((struct ls1g_nand_cmdset*)&(info->nand_regs.cmd))->op_main = 1;
                ((struct ls1g_nand_cmdset*)&(info->nand_regs.cmd))->op_spare = 1;
                ((struct ls1g_nand_cmdset*)&(info->nand_regs.cmd))->cmd_valid = 1; 
                /*dma regs config*/
                info->dma_regs.length = ALIGN_DMA(info->buf_start);
                ((struct ls1g_nand_dma_cmd *)&(info->dma_regs.cmd))->dma_int_mask = 1;
                ((struct ls1g_nand_dma_cmd *)&(info->dma_regs.cmd))->dma_r_w = 1;
                nand_setup(NAND_ADDRL|NAND_ADDRH|NAND_OP_NUM|NAND_CMD,info);
                dma_setup(DMA_LENGTH|DMA_CMD,info);
                break;
            case NAND_CMD_RESET:
                /*Do reset op anytime*/
               /*nand cmd set */ 
                info->nand_regs.cmd = 0; 
#if defined(CONFIG_TARGET_LS1A) || defined(CONFIG_TARGET_LS232)
				((struct ls1g_nand_cmdset*)&(info->nand_regs.cmd))->int_en = 0;
                ((struct ls1g_nand_cmdset*)&(info->nand_regs.cmd))->ram_op = RAM_OP_OFF;
#endif
                ((struct ls1g_nand_cmdset*)&(info->nand_regs.cmd))->reset = 1;
                ((struct ls1g_nand_cmdset*)&(info->nand_regs.cmd))->cmd_valid = 1; 
                nand_setup(NAND_CMD,info);
                status_time = STATUS_TIME_LOOP_R;
                while(!ls1g_nand_status(info)){
                    if(!(status_time--)){
                        write_z_cmd;
                        break;
                    }
                    udelay(50);
                }
                break;
            case NAND_CMD_ERASE1:
                if(info->state == STATE_BUSY){
                    printk("nandflash chip if busy...\n");
                    return;
                }
                info->state = STATE_BUSY;
                
                /*nand regs set*/
                info->nand_regs.addrh =  NO_SPARE_ADDRH(page_addr);
                info->nand_regs.addrl =  NO_SPARE_ADDRL(page_addr) ;
               /*nand cmd set */
                info->nand_regs.cmd = 0; 
#if defined(CONFIG_TARGET_LS1A) || defined(CONFIG_TARGET_LS232)
				((struct ls1g_nand_cmdset*)&(info->nand_regs.cmd))->int_en = 0;
                ((struct ls1g_nand_cmdset*)&(info->nand_regs.cmd))->ram_op = RAM_OP_OFF;
#endif
                ((struct ls1g_nand_cmdset*)&(info->nand_regs.cmd))->erase_one = 1;
                ((struct ls1g_nand_cmdset*)&(info->nand_regs.cmd))->cmd_valid = 1;
                nand_setup(NAND_ADDRL|NAND_ADDRH|NAND_OP_NUM|NAND_CMD,info);
                status_time = STATUS_TIME_LOOP_E;
                udelay(2000);    
                while(!ls1g_nand_status(info)){
                    if(!(status_time--)){
                        write_z_cmd;
                        break;
                    }
                    udelay(50);    
                }
                info->state = STATE_READY;
                break;
            case NAND_CMD_STATUS:
                if(info->state == STATE_BUSY){
                    printk("nandflash chip if busy...\n");
                    return;
                }
                info->state = STATE_BUSY;
                info->buf_count = 0x1;
                info->buf_start = 0x0;
                *(unsigned char *)info->data_buff=ls1g_nand_status(info) | 0x80;
                info->state = STATE_READY;
                break;
            case NAND_CMD_READID:
                if(info->state == STATE_BUSY){
                    printk("nandflash chip if busy...\n");
                    return;
                }
                info->state = STATE_BUSY;
                info->buf_count = 0x5;
                info->buf_start = 0;


               {

                   unsigned int id_val_l=0,id_val_h=0;
                   unsigned char *data = (unsigned char *)(info->data_buff);
#ifdef	LS1B01
                    unsigned int timing = 0;
                   _NAND_READ_REG(0xc,timing);
                   _NAND_SET_REG(0xc,0x30f0); 
                   _NAND_SET_REG(0x0,0x21); 

                   while(((id_val_l |= _NAND_IDL) & 0xff)  == 0){
                       id_val_h = _NAND_IDH;
                   }

                   printk("id_val_l=0x%08x\nid_val_h=0x%08x\n",id_val_l,id_val_h);
                   _NAND_SET_REG(0xc,timing);
#else
		   _NAND_SET_REG(0x0,0x21); 
                   udelay(1); 
                   id_val_l = _NAND_IDL;
                   id_val_h = _NAND_IDH;
#endif
                   data[0]  = (id_val_h & 0xff);
                   data[1]  = (id_val_l & 0xff000000)>>24;
                   data[2]  = (id_val_l & 0x00ff0000)>>16;
                   data[3]  = (id_val_l & 0x0000ff00)>>8;
     		   data[4]  = (id_val_l & 0x000000ff);

               }
                info->state = STATE_READY;
                break;
            case NAND_CMD_ERASE2:
            case NAND_CMD_READ1:
                break;
            case NAND_CMD_RNDOUT:
                 
                info->buf_start =  column ;
                break;
            default :
                printk(KERN_ERR "non-supported command.\n");
		break;
        }
        if(info->cmd == NAND_CMD_READ0 || info->cmd == NAND_CMD_READOOB ){
	    flush_dcache_range((unsigned long)(info->data_buff), (unsigned long)(info->data_buff)+info->cac_size);
        }
        info->state = STATE_READY;

}

static void ls1g_nand_init_info(struct ls1g_nand_info *info)
{
    *(volatile unsigned int *)(info->mmio_base+0x18) = 0x08005300;
    info->timing_flag = 1;/*0:read; 1:write;*/
    info->num=0;
    info->size=0;
    info->cac_size = 0; 
    info->state = STATE_READY;
    info->cmd = -1;
    info->page_addr = -1;
    info->nand_addrl = 0x0;
    info->nand_addrh = 0x0;
    info->nand_timing =0x412;// 0x4<<8 | 0x12;
    info->nand_op_num = 0x0;
    info->nand_cs_rdy_map = 0x88442200;
    info->nand_cmd = 0;

    info->dma_orderad = 0x0;
    info->dma_saddr = info->data_buff_phys;
    info->dma_length = 0x0;
    info->dma_step_length = 0x0;
    info->dma_step_times = 0x1;
    info->dma_cmd = 0x0;


}
static int ls1g_nand_probe(struct udevice *dev)
{
	struct ls1g_nand_info *info = dev_get_priv(dev);
	struct nand_chip *this = &info->nand_chip;
	phys_addr_t baseaddr;
	struct mtd_info *mtd;
	int ret = 0, irq;

	mtd = nand_to_mtd(this);
	baseaddr = dev_read_addr(dev);
	info->mmio_base = map_physmem(baseaddr, 0, MAP_NOCACHE);
        info->dma_daddr = baseaddr+0x40;

	mtd->priv = info;
        info->drcmr_dat = dma_alloc_coherent(NULL, MAX_BUFF_SIZE,
				&info->drcmr_dat_phys, GFP_KERNEL);
        info->dma_ask = dma_alloc_coherent(NULL, MAX_BUFF_SIZE,
				&info->dma_ask_phy, GFP_KERNEL);
        info->order_reg_addr = map_physmem(dev_read_addr_index(dev, 1), 0, MAP_NOCACHE);

    	info->data_buff = dma_alloc_noncoherent(NULL, MAX_BUFF_SIZE,
				&info->data_buff_phys, GFP_KERNEL);
       	info->data_buff_size = MAX_BUFF_SIZE;
        
        ls1g_nand_init_mtd(mtd, info);

	ls1g_nand_init_info(info);

	if (nand_scan(mtd, 1)) {
		printf("failed to scan nand\n");
		return -ENXIO;
	}

	return nand_register(0, mtd);
}

static const struct udevice_id ls1x_nand_dt_ids[] = {
	{
		.compatible = "loongson,ls1a-nand",
		//.data = (unsigned long)&mxs_nand_imx6q_data,
	},
	{ /* sentinel */ }
};


U_BOOT_DRIVER(ls1x_nand) = {
	.name = "ls1x-nand",
	.id = UCLASS_MTD,
	.of_match = ls1x_nand_dt_ids,
	.probe = ls1g_nand_probe,
	.priv_auto = sizeof(struct ls1g_nand_info),
};

void board_nand_init(void)
{
	struct udevice *dev;
	int ret;

	ret = uclass_get_device_by_driver(UCLASS_MTD,
					  DM_GET_DRIVER(ls1x_nand),
					  &dev);
	if (ret && ret != -ENODEV)
		pr_err("Failed to initialize ls1x NAND controller. (error %d)\n",
		       ret);
}

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Loongson_1a NAND controller driver");
