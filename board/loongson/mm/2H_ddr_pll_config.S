LEAF(Is_ddr_pll_hard_ctrl_mode)
/************************
 * read out the ddr pll mode
 *
 * output:
 *  v0:
 *  0: soft control mode
 *  1: hard control mode
************************/

    dli     v0, LS2H_BOOTCFG_ADDR
    ld      v0, 0x0(v0)
    dsrl    v0, v0, (LS2H_BOOTCFG_OFFSET + LS2H_DDR_PLL_MODE_OFFSET)
    and     v0, v0, 0x1

    j       ra
    nop
END(Is_ddr_pll_hard_ctrl_mode)

LEAF(set_ddr_pll_freq)
/************************
input:
      a0:ddr_pll_config
************************/

#if 0
        li     t0,0xaff00034    //DDR_CONF_CTL_03
        lw     t1,0x0(t0) 
	
	or     t1,t1,0x00000001  //set refresh
        lw     t1,0x0(t0) 

        li     t2,0x00000100  //delay
2:
        subu   t2,t2,0x1
        bnez   t2,2b
        nop


        li     t0,0xaff00034    //DDR_CONF_CTL_03
        lw     t1,0x0(t0) 
	
	and     t1,t1,0xfffffeff  //clear start
        lw     t1,0x0(t0) 

        li     t2,0x00000100  //delay
2:
        subu   t2,t2,0x1
        bnez   t2,2b
        nop

#endif
            
        li     t0,0xbfd00220    //CLOCK_CTRL0
        lw     t1,0x0(t0) 
	
	or     t1,t1,0x00010000  //set _set 
	sw     t1,0x0(t0)
        and    t1,t1,0xfffdffff //set _sel
	sw     t1,0x0(t0)
        
        li     t2,0x00000100  //delay
2:
        subu   t2,t2,0x1
        bnez   t2,2b
        nop

        or     t1,t1,0x00040000 //set _pd
        sw     t1,0x0(t0)

        and    t1,t1,0x0007ffff //clear ldf idf odf
        or     t1,t1,a0         //pll config
        sw     t1,0x0(t0)        //set ldf idf odf
     
        
        li     t2,0xbfd00224   //CLOCK_CTRL1
        lw     t3,0x0(t2)
        and    t3,t3,0xdfffffff  //set normal mode
        sw     t3,0x0(t2)

        li     t2,0x00000800  //delay
2:
        subu   t2,t2,0x1
        bnez   t2,2b
        nop
    
        and    t1,t1,0xfffbffff  //clear _pd
        sw     t1,0x0(t0)

        li     t2,0xbfd00210    //CHIP_SAMPLE0
2:
        lw     t3,0x0(t2)
        and    t3,t3,0x00000200  //wait for ready
        beqz   t3,2b
        nop

        or     t1,t1,0x00020000  //clear _sel
        sw     t1,0x0(t0)        

        and    t1,t1,0xfffeffff  //clear _set
        sw     t1,0x0(t0)        
         
	


        li     t2,0x00000100  //delay
2:
        subu   t2,t2,0x1
        bnez   t2,2b
        nop

#if 0
	
        li     t0,0xaff00034    //DDR_CONF_CTL_03
        lw     t1,0x0(t0) 
	
	or     t1,t1,0x00000100  //set start
        lw     t1,0x0(t0) 

        li     t2,0x00000100  //delay
2:
        subu   t2,t2,0x1
        bnez   t2,2b
        nop


        li     t0,0xaff00034    //DDR_CONF_CTL_03
        lw     t1,0x0(t0) 
	
	and     t1,t1,0xfffffffe  //clear refresh
        lw     t1,0x0(t0) 

        li     t2,0x00000100  //delay
2:
        subu   t2,t2,0x1
        bnez   t2,2b
        nop
	
#endif	
	
	j      ra
	nop 

END(set_ddr_pll_freq)
