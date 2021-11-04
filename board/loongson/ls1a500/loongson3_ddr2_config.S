/**********************************
    loongson3_ddr2_config.S
        used to set up ddr controllers MC0 and MC1
        and set up the memory space on L2 Xbar
    input: s1--MC1 & MC0 DIMM info and Node ID
    note: s1 is damaged by the end of this file
    original: whd
    rewrite by Chen Xinke on 11/11/2010
    1: reorder the program
    2: DIMM info and memory size is set according to s1[MC1&0_MEMSIZE]
    note: config L2 Xbar still need to be finished,currently only support limited MEMSIZE.
    v1.0    raw
    v1.2    add support for 4G memsize per MC, modify the L2-Xbar config manner of MC1
            to reduce code size.
    v1.4    Modify L2 Xbar config reg code at Interleave mode to reduce code size
            new code:
            1. according to Memsize config open space
            2. config interleave bits
    v1.6    Modify L2 Xbar address window configuration to new pmon-kernel interface.
            (default use NO_INTERLEAVE)
    v1.8    Modify supported Interleave bit. Add Interleave support when each MC uses 1GB or 2GB.
    v2.0    Add support for 8GB per MC.
************************************/

#######################################################
/**************************
0. s1 reset code
**************************/
//set use which MC: 01-MC0; 10-MC1; 00-MC0&MC1
#ifdef DEBUG_DDR_PARAM
    PRINTSTR("Please input DDR SELLECT :\r\n[ 8]:MC1_ONLY\r\n[ 4]:MC0_ONLY\r\n");
    dli     t6, 0x00
    bal     inputaddress    #input value stored in v0
    nop
    dli     t6, 0xc
    and     v0, v0, t6
    dli     t6, 0xc
    not     t6, t6
    and     s1, s1, t6
    or      s1, s1, v0
#endif

#ifdef  AUTO_ARB_LEVEL
//store memory system info into s3(dimm i2c addr)
#ifdef  CHECK_ARB_LEVEL_DIMM
    //store dimm i2c addr info to s3
    dsrl    a2, s1, 16
    dli     a1, 0xffff
    and     a2, a2, a1
    GET_NODE_ID_a1
    dsll    a1, a1, 4
    dsll    a2, a2, a1
    or      s3, s3, a2
#endif

#ifdef  DEBUG_AUTO_ARB_LEVEL
    PRINTSTR("\r\ns3 = 0x");
    dsrl    a0, s3, 32
    bal     hexserial
    nop
    PRINTSTR("__")
    move    a0, s3
    bal     hexserial
    nop
    PRINTSTR("\r\n")
#endif
#endif

#ifdef  AUTO_DDR_CONFIG
    bal     PROBE_NODE_DIMM
    nop
#endif

#ifdef  PRINT_MSG
    /* show value of s1 */
    PRINTSTR("\r\n\r\ns1 = 0x");
    dsrl    a0, s1, 32
    bal     hexserial
    nop
    PRINTSTR("__")
    move    a0, s1
    bal     hexserial
    nop
    PRINTSTR("\r\n")
#endif

#ifdef LSMCD3_2
//for DDR3 RDIMM, if it has 2 rank, use only 1. temp code for 3A MC
    //check MC1 first
    dsrl    t1, s1, 32
    dli     a1, 0xd0000000
    and     a1, t1, a1
    dli     a0, 0xd0000000
    bne     a1, a0, 1f      //not DDR3 RDIMM, do nothing
    nop
    dli     t2, 0xff00
    and     t2, t1, t2      //t2 store memsize

    dli     a1, 0x30000     //check slot 0 CS_MAP
    and     a1, t1, a1
    dli     a0, 0x30000
    bne     a1, a0, 2f      //not 2 rank
    nop
    //slot 0 has 2 rank DIMM
    dli     a1, 0xfffdffff
    and     t1, t1, a1      //clear cs 1
    dsrl    t2, t2, 1       //half memsize
2:
    dli     a1, 0xc0000     //check slot 1 CS_MAP
    and     a1, t1, a1
    dli     a0, 0xc0000
    bne     a1, a0, 2f      //not 2 rank
    nop
    //slot 1 has 2 rank DIMM
    dli     a1, 0xfff7ffff
    and     t1, t1, a1      //clear cs 3
    dsrl    t2, t2, 1       //half memsize
2:
    //use reduced(if needed) memsize
    dli     a1, 0xffff00ff
    and     t1, t1, a1
    or      t1, t1, t2
    //use modified infor for MC1
    dli     a1, 0xffffffff
    and     s1, s1, a1
    dsll    t1, t1, 32
    or      s1, s1, t1
1:
    //check MC0
    dli     a1, 0xffffffff
    and     t1, s1, a1
    dli     a1, 0xd0000000
    and     a1, t1, a1
    dli     a0, 0xd0000000
    bne     a1, a0, 1f      //not DDR3 RDIMM, do nothing
    nop
    dli     t2, 0xff00
    and     t2, t1, t2      //t2 store memsize

    dli     a1, 0x30000     //check slot 0 CS_MAP
    and     a1, t1, a1
    dli     a0, 0x30000
    bne     a1, a0, 2f      //not 2 rank
    nop
    //slot 0 has 2 rank DIMM
    dli     a1, 0xfffdffff
    and     t1, t1, a1      //clear cs 1
    dsrl    t2, t2, 1       //half memsize
2:
    dli     a1, 0xc0000     //check slot 1 CS_MAP
    and     a1, t1, a1
    dli     a0, 0xc0000
    bne     a1, a0, 2f      //not 2 rank
    nop
    //slot 1 has 2 rank DIMM
    dli     a1, 0xfff7ffff
    and     t1, t1, a1      //clear cs 3
    dsrl    t2, t2, 1       //half memsize
2:
    //use reduced(if needed) memsize
    dli     a1, 0xffff00ff
    and     t1, t1, a1
    or      t1, t1, t2
    //use modified infor for MC0
    dli     a1, 0xffffffff00000000
    and     s1, s1, a1
    or      s1, s1, t1
1:
#if 1
    /* show value of s1 */
    PRINTSTR("\r\nnew s1 = 0x");
    dsrl    a0, s1, 32
    bal     hexserial
    nop
    PRINTSTR("__")
    move    a0, s1
    bal     hexserial
    nop
    PRINTSTR("\r\n")
#endif
#endif

#ifndef LSMCD3_2
#ifdef DISABLE_DDR_A15
//hot fix for lack of address pin A15 when use LSMC. For example RS780E R1.03
    //check MC1 first
    move    t1, s1  //store s1
    dsrl    s1, s1, 32
    GET_SDRAM_TYPE
    beqz    a1, 1f
    nop
    GET_ROW_SIZE
    bnez    a1, 1f  //row size diff not 0, do nothing
    nop
    //set row_size diff to 1
    dli     a1, 1
    dsll    a1, a1, ROW_SIZE_OFFSET
    or      s1, s1, a1
    //half memsize
    dli     t2, 0xff00
    and     t2, s1, t2      //t2 store memsize
    dsrl    t2, t2, 1
    dli     a1, 0xffff00ff
    and     s1, s1, a1
    or      s1, s1, t2
1:
    //reconstruct s1
    dli     a1, 0xffffffff
    and     t1, t1, a1
    dsll    s1, s1, 32
    or      s1, s1, t1

    //check MC0
    move    t1, s1
    dsll    s1, s1, 32
    dsrl    s1, s1, 32
    GET_SDRAM_TYPE
    beqz    a1, 1f
    nop
    GET_ROW_SIZE
    bnez    a1, 1f  //row size diff not 0, do nothing
    nop
    //set row_size diff to 1
    dli     a1, 1
    dsll    a1, a1, ROW_SIZE_OFFSET
    or      s1, s1, a1
    //half memsize
    dli     t2, 0xff00
    and     t2, s1, t2      //t2 store memsize
    dsrl    t2, t2, 1
    dli     a1, 0xffff00ff
    and     s1, s1, a1
    or      s1, s1, t2
1:
    //reconstruct s1
    dli     a1, 0xffffffff00000000
    and     t1, t1, a1
    or      s1, s1, t1
#if 1
    /* show value of s1 */
    PRINTSTR("\r\nnew s1 = 0x");
    dsrl    a0, s1, 32
    bal     hexserial
    nop
    PRINTSTR("__")
    move    a0, s1
    bal     hexserial
    nop
    PRINTSTR("\r\n")
#endif
#endif
#endif

#ifdef DEBUG_DDR_PARAM
    PRINTSTR("\r\nChange s1?:\r\n");
    dli     t6, 0x00
    bal     inputaddress    #input value stored in v0
    nop
    beqz    v0, 1f
    nop
    move    s1, v0
1:
#endif

//make sure s1[3:2] is correctly set.
    GET_MC_SEL_BITS
    dli     a2, 3
    bne     a1, a2, 1f
    nop
//s1[3:2]=0b'11, clear to 0b'00
    dli     a2, 0xc
    not     a2, a2
    and     s1, s1, a2
1:
/**************************
1. 1. check NODE memory size.
*  2. set MC0/1_ONLY if the following 2 conditions are satisfied:
*     (1). s1[3:2]=0b'00
*     (2). MC0 or MC1 MEMSIZE > 0.
* when use AUTO_DDR_CONFIG, one MC may have no DIMM while the other has, in this case,
* the code should set MC0_ONLY or MC1_ONLY in s1 automatically, because the code of 
* configuring L2-Xbar will use this message.
**************************/
    GET_MC0_ONLY
    bnez    a1, 1f
    nop
    GET_MC1_ONLY
    bnez    a1, 2f
    nop
//s1[3:2]=0b'00
    //check memory size in this case
    GET_MC0_MEMSIZE
    move    t5, a1
    GET_MC1_MEMSIZE
    daddu   a1, a1, t5
    beqz    a1, 89f
    nop
    dli     t5, 0x20
    bgt     a1, t5, 89f
    nop
    GET_MC0_MEMSIZE
    bnez    a1, 3f
    nop
    //MC0_MEMSIZE=0, MC1_MEMSIZE must !=0, set MC1_ONLY
    dli     t5, 0x8
    or      s1, t5
    b       4f
    nop
3:  //MC0_MEMSIZE!=0
    GET_MC1_MEMSIZE
    bnez    a1, 4f
    nop
    //MC1_MEMSIZE=0 set use MC0_ONLY
    dli     t5, 0x4
    or      s1, t5
    b       4f
    nop
1:  //MC0_ONLY
    GET_MC0_MEMSIZE
    b       5f
    nop
2:  //MC1_ONLY
    GET_MC1_MEMSIZE
5:
    beqz    a1, 89f
    nop
    dli     t5, 0x10
    bgt     a1, t5, 89f
    nop
4:
#if 0
    PRINTSTR("\r\ns1 = 0x")
    move    a0, s1
    bal     hexserial
    nop
    PRINTSTR("\r\n")
#endif
/************************
2. set up Memory Controller.
************************/
/***********************
for single chip or multi-chip:
t0: X-bar config base
t2: chip configuration register location
t0,t2 shouldn't be changed to the end of this file.
**********************/
    GET_NODE_ID_a0
    dli     t2, 0x900000001fe10000 //no use
    dli     t0, 0x900000001fe10000
    or      t2, t2, a0
    or      t0, t0, a0

//init MC1 will damage MC0 s1 info
//config MC0 if not define MC1_ONLY
//-------------------------------------
10:
    GET_MC1_ONLY
    bnez    a1, 11f
    nop

    dli     t3, 0x0
#ifdef  AUTO_ARB_LEVEL
    bal     SET_AUTO_ARB_LEVEL_MARK
    nop
#endif

    bal     mc_init
    nop

    PRINTSTR("\r\nMC0 Config DONE\r\n")
//-------------------------------------
//config MC1 if not define MC0_ONLY
11:
    GET_MC0_ONLY
    bnez    a1, 12f
    nop

    dli     t3, 0x1

    //shift MC1 DIMM info to low 32bit of s1
    dsrl    t5, s1, 32
    dli     a1, 0xffff8000
    and     t5, t5, a1
    dli     a1, 0xffffffff8000ffff
    and     s1, s1, a1
    or      s1, s1, t5

#ifdef  AUTO_ARB_LEVEL
    bal     SET_AUTO_ARB_LEVEL_MARK
    nop
#endif

    bal     mc_init
    nop
12:
89:
    TTYDBG("\r\nMC1 Config DONE\r\n")
    sync
    nop
    nop
    nop
    nop
