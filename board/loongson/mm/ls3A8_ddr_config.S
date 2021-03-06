/**********************************
    Author: chenxinke
    Date:   20150707
    mc_init for 3A8
    v1.0
    input:
    t7(option ARB_LEVEL)--do arb level, 0--not level; 1--do level;
    t3--MC select: 0--MC0; 1--MC1
**********************************/
#include "lsmc_config_param.S"

        .global mc_init
        .ent    mc_init
        .set    noreorder
        .set    mips3
mc_init:
#ifdef  ARB_LEVEL
    sd	    ra, (sp)
    sd	    s3, 8(sp)
#else
    move t7, ra
#endif

    sync
    nop
    nop
    nop
    nop

    GET_NODE_ID_a0;
    XBAR_CONFIG_NODE_a0(0x10, \
                    0x0000000000000000, \
                    0xfffffffff0000000, \
                    0x00000000000000f0)
    beqz    t3, 1f
    nop
    GET_NODE_ID_a0;
    XBAR_CONFIG_NODE_a0(0x10, \
                    0x0000000000000000, \
                    0xfffffffff0000000, \
                    0x00000000000000f1)
1:

    sync
    nop
    nop
    nop
    nop

    TTYDBG("\r\nEnable register space of MEMORY\r\n")
    bal     enable_mc_conf_space
    nop

    GET_NODE_ID_a0;
    dli     t8, DDR_MC_CONFIG_BASE
    or      t8, t8, a0


    //set parameter start location
#ifdef  ARB_LEVEL
    bnez    t7, 1f
    nop
    //if use leveled ddr param, the param location is fixed
    dla     a2, ddr2_reg_data_mc0_leveled
    beqz    t3, 21f
    nop
    dla     a2, ddr2_reg_data_mc1_leveled
21:
#ifdef  MULTI_CHIP
    GET_NODE_ID_a1
    beqz    a1, 21f
    nop
    dla     a2, n1_ddr2_reg_data_mc0_leveled
    beqz    t3, 21f
    nop
    dla     a2, n1_ddr2_reg_data_mc1_leveled
21:
#endif
    b       4f
    nop
1:
#endif
    GET_SDRAM_TYPE
    dli     t1, 0x2
    beq     t1, a1, 2f
    nop
    dli     t1, 0x3
    beq     t1, a1, 3f
    nop
    //not DDR2 and not DDR3, errors
    PRINTSTR("\r\n!!! ERROR: NOT recognized DDR SDRAM TYPE. !!!\r\n");
    b       3f
    nop
2:  //DDR2
    GET_DIMM_TYPE
    bnez    a1, 1f
    nop
    //UDIMM
    dla     a2, ddr2_reg_data
    beqz    t3, 21f
    nop
    dla     a2, ddr2_reg_data_mc1
21:
#ifdef  MULTI_NODE_DDR_PARAM
    GET_NODE_ID_a1
    beqz    a1, 4f
    nop
    dla     a2, n1_ddr2_reg_data
    beqz    t3, 21f
    nop
    dla     a2, n1_ddr2_reg_data_mc1
21:
#endif
    b       4f
    nop
1:  //RDIMM
    dla     a2, ddr2_RDIMM_reg_data
    beqz    t3, 21f
    nop
    dla     a2, ddr2_RDIMM_reg_data_mc1
21:
#ifdef  MULTI_NODE_DDR_PARAM
    GET_NODE_ID_a1
    beqz    a1, 4f
    nop
    dla     a2, n1_ddr2_RDIMM_reg_data
    beqz    t3, 21f
    nop
    dla     a2, n1_ddr2_RDIMM_reg_data_mc1
21:
#endif
    b       4f
    nop
3:  //DDR3
    GET_DIMM_TYPE
    bnez    a1, 1f
    nop
    //UDIMM
    dla     a2, ddr3_reg_data
    beqz    t3, 21f
    nop
    dla     a2, ddr3_reg_data_mc1
21:
#ifdef  MULTI_NODE_DDR_PARAM
    GET_NODE_ID_a1
    beqz    a1, 4f
    nop
    dla     a2, n1_ddr3_reg_data
    beqz    t3, 21f
    nop
    dla     a2, n1_ddr3_reg_data_mc1
21:
#endif
    b       4f
    nop
1:  //RDIMM
    dla     a2, ddr3_RDIMM_reg_data
    beqz    t3, 21f
    nop
    dla     a2, ddr3_RDIMM_reg_data_mc1
21:
#ifdef  MULTI_NODE_DDR_PARAM
    GET_NODE_ID_a1
    beqz    a1, 4f
    nop
    dla     a2, n1_ddr3_RDIMM_reg_data
    beqz    t3, 21f
    nop
    dla     a2, n1_ddr3_RDIMM_reg_data_mc1
21:
#endif
    b       4f
    nop
4:

    bal     ddr2_config
    nop

#ifdef  PRINT_DDR_LEVELING   //print registers
    PRINTSTR("The MC param is:\r\n")
    dli     t1, DDR_PARAM_NUM
    GET_NODE_ID_a0
    dli     t5, DDR_MC_CONFIG_BASE
    or      t5, t5, a0
1:
    ld      t6, 0x0(t5)
    move    a0, t5
    and     a0, a0, 0xfff
    bal     hexserial
    nop
    PRINTSTR(":  ")
    dsrl    a0, t6, 32
    bal     hexserial
    nop
    move    a0, t6
    bal     hexserial
    nop
    PRINTSTR("\r\n")

    daddiu  t1, t1, -1
    daddiu  t5, t5, 8
    bnez    t1, 1b
    nop
#endif

    TTYDBG("\r\nDisable register space of MEMORY\r\n")
    bal     disable_mc_conf_space
    nop
    TTYDBG("\r\nDisable register space of MEMORY done.\r\n")



#ifndef DISABLE_HARD_LEVELING
    //read ARB_level
    beqz    t7, 88f
    nop
    dli     a2, 0x3
    GET_SDRAM_TYPE
    bne     a1, a2, 88f
    nop
    //DDR3 SDRAM, do hard leveling
    PRINTSTR("Start Hard Leveling...\r\n")
    TTYDBG("\r\nEnable register space of MEMORY\r\n")
    bal     enable_mc_conf_space
    nop

    move    k0, t3  //store t3, because ddr3_leveling will demage t3
    GET_NODE_ID_a0
    dli     t8, DDR_MC_CONFIG_BASE
    or      t8, t8, a0
    bal     ddr3_leveling
    nop
    move    t3, k0

    //Re-set t0&t2 because mc_init(ddr3_leveling) will change t0~t9
    GET_NODE_ID_a0
    dli     t2, 0x900000001fe00180
    dli     t0, 0x900000003ff00000
    or      t2, t2, a0
    or      t0, t0, a0

#ifdef  PRINT_DDR_LEVELING   //print registers
    PRINTSTR("The MC param after leveling is:\r\n")
    dli     t1, DDR_PARAM_NUM
    GET_NODE_ID_a0
    dli     t5, DDR_MC_CONFIG_BASE
    or      t5, t5, a0
1:
    ld      t6, 0x0(t5)
    move    a0, t5
    and     a0, a0, 0xfff
    bal     hexserial
    nop
    PRINTSTR(":  ")
    dsrl    a0, t6, 32
    bal     hexserial
    nop
    //PRINTSTR("  ")
    move    a0, t6
    bal     hexserial
    nop
    PRINTSTR("\r\n")

    daddiu  t1, t1, -1
    daddiu  t5, t5, 8
    bnez    t1, 1b
    nop
#endif
#if 0   //def  DEBUG_DDR_PARAM   //Change parameters of MC
    GET_NODE_ID_a0;
    dli     a1, DDR_MC_CONFIG_BASE
    or      t8, a0, a1

    PRINTSTR("\r\nChange some parameters of MC:");
1:
    PRINTSTR("\r\nPlease input the register number you want to change!!!(0xfff:jump out.): ");
    dli     t6, 0x00
    bal     inputaddress
    nop
    move    t5, v0
    
    dli     a1, 0x320
    bge     t5, a1, 2f    #if input address offset exceed range,jump out
    nop
    and     t5, t5, 0xff8
    daddu   t5, t5, t8

    PRINTSTR("\r\nPlease input the data-hex: ");
    dli     t6, 0x00
    bal     inputaddress
    nop
    sd      v0, 0x0(t5)    #v0 is the input value

    //print the new register value
    move    t6, t5
    PRINTSTR("\r\nRegister 0x")
    dsubu   t5, t5, t8
    move    a0, t5
    bal     hexserial
    nop
    PRINTSTR(": ")
    ld      t6, 0x0(t6)
    dsrl    a0, t6, 32
    bal     hexserial
    nop
    move    a0, t6
    bal     hexserial
    nop

    b        1b
    nop
2:    
#endif

#if 0 //temp add for 3a3000-2h 8G RDIMM, use 32bit channel
    GET_NODE_ID_a0;
    dli     a1, DDR_MC_CONFIG_BASE
    or      t8, a0, a1

    dli     a1, 0x000801e4ff050101
    sd      a1, 0x1f0(t8)
    dli     a1, 0x0008010e01010006
    sd      a1, 0x210(t8)
#endif

    //TTYDBG("Disable register space of MEMORY\r\n")
    bal     disable_mc_conf_space
    nop
88:
#endif

#ifndef  DISABLE_DIMM_ECC
    //Init ECC according to DIMM ECC info
    GET_DIMM_ECC
    beqz    a1, 4f
    nop
    TTYDBG("ECC init start(maybe take 1 minute or so)....\r\n")

    //TTYDBG("Enable register space of MEMORY\r\n")
    bal     enable_mc_conf_space
    nop

    GET_NODE_ID_a0;
    dli     t8, DDR_MC_CONFIG_BASE
    or      t8, t8, a0

    //disable ECC interrupt
    ld      a2, ECC_INT_ENABLE_ADDR(t8)
    dli     a1, 0x3
    dsll    a1, a1, ECC_INT_ENABLE_OFFSET 
    not     a1, a2
    and     a2, a2, a1
    sd      a2, ECC_INT_ENABLE_ADDR(t8)

    //enable ECC function but without reporting error
    ld      a2, ECC_ENABLE_ADDR(t8)
    dli     a1, 0x7
    dsll    a1, a1, ECC_ENABLE_OFFSET
    not     a1, a1
    and     a2, a2, a1
    dli     a1, 0x1
    dsll    a1, a1, ECC_ENABLE_OFFSET
    or      a2, a2, a1
    sd      a2, ECC_ENABLE_ADDR(t8)

    //TTYDBG("Disable register space of MEMORY\r\n")
    bal     disable_mc_conf_space
    nop

    //route 0x1000000000 ~ 0x1FFFFFFFFF(64G) to MC for ECC init
    sync
    nop
    nop
    nop
    nop
    GET_NODE_ID_a0;
    XBAR_CONFIG_NODE_a0(ARB_TEMP_L2WINDOW_OFFSET, \
                    0x0000001000000000, \
                    0xFFFFFFF000000000, \
                    0x00000000000000F0)
    beqz    t3, 1f
    nop
    GET_NODE_ID_a0;
    XBAR_CONFIG_NODE_a0(ARB_TEMP_L2WINDOW_OFFSET, \
                    0x0000001000000000, \
                    0xFFFFFFF000000000, \
                    0x00000000000000F1)

1:
    sync
    nop
    nop
    nop
    nop
//init mem to all 0
    dli     t1, 0xb800001000000000
    GET_NODE_ID_a0
    or      t1, t1, a0
    GET_MC0_MEMSIZE
    beqz    t3, 1f
    nop
    GET_MC1_MEMSIZE
1:
    dsll    a1, a1, 29   //a1*512M
    daddu   t5, t1, a1
//write memory
1:
    bgeu    t1, t5, 1f
    nop

    sd      $0, 0x0(t1)
    sd      $0, 0x8(t1)
    sd      $0, 0x10(t1)
    sd      $0, 0x18(t1)
    sd      $0, 0x20(t1)
    sd      $0, 0x28(t1)
    sd      $0, 0x30(t1)
    sd      $0, 0x38(t1)
    daddu   t1, t1, 0x40
    b       1b
    nop
1:

    sync
    nop
    nop
    nop
    nop

    L2XBAR_CLEAR_WINDOW(ARB_TEMP_L2WINDOW_OFFSET)

    sync
    nop
    nop
    nop
    nop

    //TTYDBG("Enable register space of MEMORY\r\n")
    bal     enable_mc_conf_space
    nop

    //enable ECC function with reporting error
    ld      a2, ECC_ENABLE_ADDR(t8)
    dli     a1, 0x7
    dsll    a1, a1, ECC_ENABLE_OFFSET
    not     a1, a1
    and     a2, a2, a1
    dli     a1, 0x7
    dsll    a1, a1, ECC_ENABLE_OFFSET
    or      a2, a2, a1
    sd      a2, ECC_ENABLE_ADDR(t8)

    //enable ECC interrupt
    ld      a2, ECC_INT_ENABLE_ADDR(t8)
    dli     a1, 0x3
    dsll    a1, a1, ECC_INT_ENABLE_OFFSET 
    or      a2, a2, a1
    sd      a2, ECC_INT_ENABLE_ADDR(t8)

    //PRINTSTR("\r\nDisable register space of MEMORY\r\n")
    bal     disable_mc_conf_space
    nop

    TTYDBG("MC ECC init done.\r\n")
4:
#endif

#ifdef  ARB_LEVEL
    ld	    s3, 8(sp)
    ld	    ra, (sp)
#else
    move ra, t7
#endif
    jr      ra
    nop
    .end    mc_init

#ifdef  AUTO_ARB_LEVEL
LEAF(SET_AUTO_ARB_LEVEL_MARK)
    move    t6, ra

//set t7 to mc level info address
//if define CHECK_ARB_LEVEL_DIMM, set t1 at the same time for CHECK_DIMM_CHANGE
    dla     t7, c0_mc0_level_info
#ifdef  CHECK_ARB_LEVEL_DIMM
    move    t1, s3
#endif
    beqz    t3, 1f
    nop
    dla     t7, c0_mc1_level_info
#ifdef  CHECK_ARB_LEVEL_DIMM
    dsrl    t1, s3, 8
#endif
1:
#ifdef  MULTI_CHIP
    GET_NODE_ID_a1
    beqz    a1, 4f
    nop
    dla     t7, c1_mc0_level_info
#ifdef  CHECK_ARB_LEVEL_DIMM
    dsrl    t1, s3, 16
#endif
    beqz    t3, 4f
    nop
    dla     t7, c1_mc1_level_info
#ifdef  CHECK_ARB_LEVEL_DIMM
    dsrl    t1, s3, 24
#endif
4:
#endif
    daddu   t7, t7, s0
#ifdef  DEBUG_AUTO_ARB_LEVEL
    PRINTSTR("\r\nstored level info addr is 0x")
    dsrl    a0, t7, 32
    bal     hexserial
    nop
    move    a0, t7
    bal     hexserial
    nop
    PRINTSTR("\r\nsaved dimm infor is 0x")
    ld      t8, 0x0(t7)
    dsrl    a0, t8, 32
    bal     hexserial
    nop
    move    a0, t8
    bal     hexserial
    nop
    PRINTSTR("\r\n")
    ld      t8, 0x8(t7)
    dsrl    a0, t8, 32
    bal     hexserial
    nop
    move    a0, t8
    bal     hexserial
    nop
    PRINTSTR("\r\n")
    ld      t8, 0x10(t7)
    dsrl    a0, t8, 32
    bal     hexserial
    nop
    move    a0, t8
    bal     hexserial
    nop
    PRINTSTR("\r\n")
    ld      t8, 0x18(t7)
    dsrl    a0, t8, 32
    bal     hexserial
    nop
    move    a0, t8
    bal     hexserial
    nop
    PRINTSTR("\r\n")
    ld      t8, 0x20(t7)
    dsrl    a0, t8, 32
    bal     hexserial
    nop
    move    a0, t8
    bal     hexserial
    nop
    PRINTSTR("\r\n")
#endif
    //check level mark(first boot or first populate DIMM)
    ld      v0, 0x0(t7)
    and     v0, v0, 0x1
    beqz    v0, 1f 
    nop
    PRINTSTR("\r\nThis MC has been leveled.");

#ifdef  CHECK_ARB_LEVEL_FREQ
    bal     CHECK_DDR_FREQ_CHANGE
    nop
    bnez    v0, 1f
    nop
    PRINTSTR("\r\nThis MC frequency has not been changed.");
#endif

#ifdef  CHECK_ARB_LEVEL_DIMM
    bal     CHECK_DIMM_CHANGE
    nop
    bnez    v0, 1f
    nop
    PRINTSTR("\r\nThis MC DIMMs have not been changed.");
#endif
    //set no arb level mark
    move    t7, $0
    b       2f
    nop
1:
    //set do arb level mark
    or      t7, $0, 0x1
    or      v0, $0, 0x1
    dsll    a2, v0, 32
    GET_NODE_ID_a1
    dsll    a1, a1, 1
    dsll    a2, a2, a1
    dsll    a2, a2, t3
    or      s3, s3, a2
2:
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
    move    ra, t6
    jr      ra
    nop
END(SET_AUTO_ARB_LEVEL_MARK)
#endif

//for 3A8
LEAF(enable_mc_conf_space)
/*********************
pre-condition::
    t2: chip configure register address
    t3: MC select
*********************/
    lw      a2, 0x0(t2)
    li      v1, 0x1
    beqz    t3, 1f
    nop
    sll     v1, v1, 5
1:
    sll     v1, v1, DDR_CONFIG_DISABLE_OFFSET
    not     v1, v1
    and     a2, a2, v1
    sw      a2, 0x0(t2)
    sync

    jr      ra
    nop
END(enable_mc_conf_space)

LEAF(disable_mc_conf_space)
/*********************
pre-condition::
    t2: chip configure register address
    t3: MC select
*********************/
    lw      a2, 0x0(t2)
    li      v1, 0x1
    beqz    t3, 1f
    nop
    sll     v1, v1, 5
1:
    sll     v1, v1, DDR_CONFIG_DISABLE_OFFSET
    or      a2, a2, v1
    sw      a2, 0x0(t2)
    sync

    jr      ra
    nop
END(disable_mc_conf_space)

LEAF(enable_mc_read_buffer)

    jr      ra
    nop
END(enable_mc_read_buffer)

LEAF(disable_mc_read_buffer)

    jr      ra
    nop
END(disable_mc_read_buffer)

LEAF(disable_cpu_buffer_read)

    jr      ra
    nop
END(disable_cpu_buffer_read)

LEAF(get_mem_clk)
/*********************
    v0: current ddr freq setting
*********************/
    li t0,0xbfe00190
    ld t1,0x0(t0)
    dsrl t2,t1,37
    andi t2,t2,0x0000001f
    move v0,t2

    nop
    jr ra
    nop
END(get_mem_clk)
