/** \file
 * This file defines the synopsys GMAC device dependent functions.
 * Most of the operations on the GMAC device are available in this file.
 * Functions for initiliasing and accessing MAC/DMA/PHY registers and the DMA descriptors
 * are encapsulated in this file. The functions are platform/host/OS independent.
 * These functions in turn use the low level device dependent (HAL) functions to 
 * access the register space.
 * \internal
 * ------------------------REVISION HISTORY---------------------------------
 * Synopsys                 01/Aug/2007                              Created
 */
#include "synopGMAC_Dev.h"
#include <linux/delay.h>

/**
  * Function to set the MDC clock for mdio transactiona
  *
  * @param[in] pointer to device structure.
  * @param[in] clk divider value.
  * \return Reuturns 0 on success else return the error value.
  */
s32 synopGMAC_set_mdc_clk_div(synopGMACdevice *gmacdev,u32 clk_div_val)
{
	u32 orig_data;
	orig_data = synopGMACReadReg(gmacdev->MacBase,GmacGmiiAddr); //set the mdc clock to the user defined value
	orig_data &= (~ GmiiCsrClkMask);	   
	orig_data |= clk_div_val;
	synopGMACWriteReg(gmacdev->MacBase, GmacGmiiAddr ,orig_data);
	return 0;
}

/**
  * Returns the current MDC divider value programmed in the ip.
  *
  * @param[in] pointer to device structure.
  * @param[in] clk divider value.
  * \return Returns the MDC divider value read.
  */
u32 synopGMAC_get_mdc_clk_div(synopGMACdevice *gmacdev)
{
	u32 data;
	data = synopGMACReadReg(gmacdev->MacBase,GmacGmiiAddr);
	data &= GmiiCsrClkMask;
	return data;
}


/**
  * Function to read the Phy register. The access to phy register
  * is a slow process as the data is moved accross MDI/MDO interface
  * @param[in] pointer to Register Base (It is the mac base in our case) .
  * @param[in] PhyBase register is the index of one of supported 32 PHY devices.
  * @param[in] Register offset is the index of one of the 32 phy register.
  * @param[out] u16 data read from the respective phy register (only valid iff return value is 0).
  * \return Returns 0 on success else return the error status.
  */
s32 synopGMAC_read_phy_reg(ulong RegBase,u32 PhyBase, u32 RegOffset, u16 * data )
{
u32 addr;
u32 loop_variable;
addr = ((PhyBase << GmiiDevShift) & GmiiDevMask) | ((RegOffset << GmiiRegShift) & GmiiRegMask) | GmiiCsrClk3;	//sw: add GmiiCsrClk 
addr = addr | GmiiBusy ; //Gmii busy bit

synopGMACWriteReg(RegBase,GmacGmiiAddr,addr); //write the address from where the data to be read in GmiiGmiiAddr register of synopGMAC ip

        for(loop_variable = 0; loop_variable < DEFAULT_LOOP_VARIABLE; loop_variable++){ //Wait till the busy bit gets cleared with in a certain amount of time
                if (!(synopGMACReadReg(RegBase,GmacGmiiAddr) & GmiiBusy)){
               		 break;
                }
        plat_delay(DEFAULT_DELAY_VARIABLE);
        }
        if(loop_variable < DEFAULT_LOOP_VARIABLE)
               * data = (u16)(synopGMACReadReg(RegBase,GmacGmiiData) & 0xFFFF);
        else{
        TR("Error::: PHY not responding Busy bit didnot get cleared !!!!!!\n");
	return -ESYNOPGMACPHYERR;
        }
//sw	
#if SYNOP_REG_DEBUG
	TR("read phy reg: offset = 0x%x\tdata = 0x%x\n",RegOffset,*data);
#endif

return -ESYNOPGMACNOERR;
}

/**
  * Function to write to the Phy register. The access to phy register
  * is a slow process as the data is moved accross MDI/MDO interface
  * @param[in] pointer to Register Base (It is the mac base in our case) .
  * @param[in] PhyBase register is the index of one of supported 32 PHY devices.
  * @param[in] Register offset is the index of one of the 32 phy register.
  * @param[in] data to be written to the respective phy register.
  * \return Returns 0 on success else return the error status.
  */
s32 synopGMAC_write_phy_reg(ulong RegBase, u32 PhyBase, u32 RegOffset, u16 data)
{
u32 addr;
u32 loop_variable;

synopGMACWriteReg(RegBase,GmacGmiiData,data); // write the data in to GmacGmiiData register of synopGMAC ip

addr = ((PhyBase << GmiiDevShift) & GmiiDevMask) | ((RegOffset << GmiiRegShift) & GmiiRegMask) | GmiiWrite | GmiiCsrClk3;	//sw: add GmiiCsrclk

addr = addr | GmiiBusy ; //set Gmii clk to 20-35 Mhz and Gmii busy bit
 
synopGMACWriteReg(RegBase,GmacGmiiAddr,addr);
        for(loop_variable = 0; loop_variable < DEFAULT_LOOP_VARIABLE; loop_variable++){
                if (!(synopGMACReadReg(RegBase,GmacGmiiAddr) & GmiiBusy)){
                	break;
                }
        plat_delay(DEFAULT_DELAY_VARIABLE);
        }

        if(loop_variable < DEFAULT_LOOP_VARIABLE){
	return -ESYNOPGMACNOERR;
	}
        else{
        TR("Error::: PHY not responding Busy bit didnot get cleared !!!!!!\n");
	return -ESYNOPGMACPHYERR;
        }
#if SYNOP_REG_DEBUG
	TR("write phy reg: offset = 0x%x\tdata = 0x%x",RegOffset,data);
#endif
}


/**
  * Function to read the GMAC IP Version and populates the same in device data structure.
  * @param[in] pointer to synopGMACdevice.
  * \return Always return 0.
  */

s32 synopGMAC_read_version (synopGMACdevice * gmacdev) 
{	
	u32 data = 0;
	data = synopGMACReadReg(gmacdev->MacBase, GmacVersion );
	gmacdev->Version = data;
	TR("Version = 0x%x\n",data);
	TR("The data read from %08x is %08x\n",(gmacdev->MacBase+GmacVersion),data);
	return 0;
}


/**
  * Function to reset the GMAC core. 
  * This reests the DMA and GMAC core. After reset all the registers holds their respective reset value
  * @param[in] pointer to synopGMACdevice.
  * \return 0 on success else return the error status.
  */
s32 synopGMAC_reset (synopGMACdevice * gmacdev) 
{	
	u32 data = 0;
	int limit = 1000;
	synopGMACWriteReg(gmacdev->DmaBase, DmaBusMode ,DmaResetOn);
	do {
		data = synopGMACReadReg(gmacdev->DmaBase, DmaBusMode);
		if (!(data & DmaResetOn))
			break;
		mdelay(10);
	} while(--limit);

	if (!limit)
		printf("reset failed:DATA after Reset = %08x\n", data);
	
	return limit?0:-EBUSY;	
}


/**
  * Function to program DMA bus mode register. 
  * 
  * The Bus Mode register is programmed with the value given. The bits to be set are
  * bit wise or'ed and sent as the second argument to this function.
  * @param[in] pointer to synopGMACdevice.
  * @param[in] the data to be programmed.
  * \return 0 on success else return the error status.
  */
s32 synopGMAC_dma_bus_mode_init(synopGMACdevice * gmacdev, u32 init_value )
{
	synopGMACWriteReg(gmacdev->DmaBase, DmaBusMode ,init_value );
	return 0;

}

/**
  * Function to program DMA Control register. 
  * 
  * The Dma Control register is programmed with the value given. The bits to be set are
  * bit wise or'ed and sent as the second argument to this function.
  * @param[in] pointer to synopGMACdevice.
  * @param[in] the data to be programmed.
  * \return 0 on success else return the error status.
  */
s32 synopGMAC_dma_control_init(synopGMACdevice * gmacdev, u32 init_value )
{
	synopGMACWriteReg(gmacdev->DmaBase, DmaControl, init_value);
	return 0;
}


/*Gmac configuration functions*/

/**
  * Enable the watchdog timer on the receiver. 
  * When enabled, Gmac enables Watchdog timer, and GMAC allows no more than
  * 2048 bytes of data (10,240 if Jumbo frame enabled).
  * @param[in] pointer to synopGMACdevice.
  * \return returns void.
  */
void synopGMAC_wd_enable(synopGMACdevice * gmacdev)
{
	synopGMACClearBits(gmacdev->MacBase, GmacConfig, GmacWatchdog);
	return;
}
/**
  * Disable the watchdog timer on the receiver. 
  * When disabled, Gmac disabled watchdog timer, and can receive frames up to
  * 16,384 bytes.
  * @param[in] pointer to synopGMACdevice.
  * \return returns void.
  */

void synopGMAC_wd_disable(synopGMACdevice * gmacdev)
{
	synopGMACSetBits(gmacdev->MacBase, GmacConfig, GmacWatchdog);
	return;
}

/**
  * Enables the Jabber frame support. 
  * When enabled, GMAC disabled the jabber timer, and can transfer 16,384 byte frames.
  * @param[in] pointer to synopGMACdevice.
  * \return returns void.
  */
void synopGMAC_jab_enable(synopGMACdevice * gmacdev)
{
	synopGMACSetBits(gmacdev->MacBase, GmacConfig, GmacJabber);
	return;
}

/**
  * Enables Frame bursting (Only in Half Duplex Mode). 
  * When enabled, GMAC allows frame bursting in GMII Half Duplex mode.
  * Reserved in 10/100 and Full-Duplex configurations.
  * @param[in] pointer to synopGMACdevice.
  * \return returns void.
  */
void synopGMAC_frame_burst_enable(synopGMACdevice * gmacdev)
{
	synopGMACSetBits(gmacdev->MacBase, GmacConfig, GmacFrameBurst);
	return;
}

/**
  * Disable Jumbo frame support. 
  * When Disabled GMAC does not supports jumbo frames.
  * Giant frame error is reported in receive frame status.
  * @param[in] pointer to synopGMACdevice.
  * \return returns void.
  */
void synopGMAC_jumbo_frame_disable(synopGMACdevice * gmacdev)
{
	synopGMACClearBits(gmacdev->MacBase, GmacConfig, GmacJumboFrame);
	return;
}



/**
  * Selects the GMII port. 
  * When called GMII (1000Mbps) port is selected (programmable only in 10/100/1000 Mbps configuration).
  * @param[in] pointer to synopGMACdevice.
  * \return returns void.
  */
void synopGMAC_select_gmii(synopGMACdevice * gmacdev)
{
	synopGMACClearBits(gmacdev->MacBase, GmacConfig, GmacMiiGmii);
	return;
}
/**
  * Selects the MII port. 
  * When called MII (10/100Mbps) port is selected (programmable only in 10/100/1000 Mbps configuration).
  * @param[in] pointer to synopGMACdevice.
  * \return returns void.
  */
void synopGMAC_select_mii(synopGMACdevice * gmacdev)
{
	synopGMACSetBits(gmacdev->MacBase, GmacConfig, GmacMiiGmii);
	return;
}

/**
  * Enables Receive Own bit (Only in Half Duplex Mode). 
  * When enaled GMAC receives all the packets given by phy while transmitting.
  * @param[in] pointer to synopGMACdevice.
  * \return returns void.
  */
void synopGMAC_rx_own_enable(synopGMACdevice * gmacdev)
{
	synopGMACClearBits(gmacdev->MacBase, GmacConfig, GmacRxOwn);
	return;
}

/**
  * Sets the GMAC in loopback mode. 
  * When on GMAC operates in loop-back mode at GMII/MII.
  * @param[in] pointer to synopGMACdevice.
  * \return returns void.
  * \note (G)MII Receive clock is required for loopback to work properly, as transmit clock is
  * not looped back internally.
  */
void synopGMAC_loopback_on(synopGMACdevice * gmacdev)
{
	synopGMACSetBits(gmacdev->MacBase, GmacConfig, GmacLoopback);
	return;
}
/**
  * Sets the GMAC in Normal mode. 
  * @param[in] pointer to synopGMACdevice.
  * \return returns void.
  */
void synopGMAC_loopback_off(synopGMACdevice * gmacdev)
{
	synopGMACClearBits(gmacdev->MacBase, GmacConfig, GmacLoopback);
	return;
}

/**
  * Sets the GMAC core in Full-Duplex mode. 
  * @param[in] pointer to synopGMACdevice.
  * \return returns void.
  */
void synopGMAC_set_full_duplex(synopGMACdevice * gmacdev)
{
	synopGMACSetBits(gmacdev->MacBase, GmacConfig, GmacDuplex);
	return;
}
/**
  * Sets the GMAC core in Half-Duplex mode. 
  * @param[in] pointer to synopGMACdevice.
  * \return returns void.
  */
void synopGMAC_set_half_duplex(synopGMACdevice * gmacdev)
{
	synopGMACClearBits(gmacdev->MacBase, GmacConfig, GmacDuplex);
	return;
}

/**
  * GMAC tries retransmission (Only in Half Duplex mode).
  * If collision occurs on the GMII/MII, GMAC attempt retries based on the 
  * back off limit configured. 
  * @param[in] pointer to synopGMACdevice.
  * \return returns void.
  * \note This function is tightly coupled with synopGMAC_back_off_limit(synopGMACdev *, u32).
  */
void synopGMAC_retry_enable(synopGMACdevice * gmacdev)
{
	synopGMACClearBits(gmacdev->MacBase, GmacConfig, GmacRetry);
	return;
}

/**
  * GMAC doesnot strips the Pad/FCS field of incoming frames.
  * GMAC will pass all the incoming frames to Host unmodified. 
  * @param[in] pointer to synopGMACdevice.
  * \return returns void.
  */
void synopGMAC_pad_crc_strip_disable(synopGMACdevice * gmacdev)
{
	synopGMACClearBits(gmacdev->MacBase, GmacConfig, GmacPadCrcStrip);
	return;
}
/**
  * GMAC programmed with the back off limit value.
  * @param[in] pointer to synopGMACdevice.
  * \return returns void.
  * \note This function is tightly coupled with synopGMAC_retry_enable(synopGMACdevice * gmacdev)
  */
void synopGMAC_back_off_limit(synopGMACdevice * gmacdev, u32 value)
{
	u32 data;
	data = synopGMACReadReg(gmacdev->MacBase, GmacConfig);
	data &= (~GmacBackoffLimit);
	data |= value;
	synopGMACWriteReg(gmacdev->MacBase, GmacConfig,data);
	return;
}

/**
  * Disables the Deferral check in GMAC (Only in Half Duplex mode).
  * GMAC defers until the CRS signal goes inactive.
  * @param[in] pointer to synopGMACdevice.
  * \return returns void.
  */
void synopGMAC_deferral_check_disable(synopGMACdevice * gmacdev)
{
	synopGMACClearBits(gmacdev->MacBase, GmacConfig, GmacDeferralCheck);
	return;
}
/**
  * Enable the reception of frames on GMII/MII.
  * @param[in] pointer to synopGMACdevice.
  * \return returns void.
  */
void synopGMAC_rx_enable(synopGMACdevice * gmacdev)
{
	synopGMACSetBits(gmacdev->MacBase, GmacConfig, GmacRx);
	return;
}
/**
  * Enable the transmission of frames on GMII/MII.
  * @param[in] pointer to synopGMACdevice.
  * \return returns void.
  */
void synopGMAC_tx_enable(synopGMACdevice * gmacdev)
{
	synopGMACSetBits(gmacdev->MacBase, GmacConfig, GmacTx);
	return;
}


/*Receive frame filter configuration functions*/

/**
  * Enables reception of all the frames to application.
  * GMAC passes all the frames received to application irrespective of whether they
  * pass SA/DA address filtering or not.
  * @param[in] pointer to synopGMACdevice.
  * \return returns void.
  */
void synopGMAC_frame_filter_enable(synopGMACdevice * gmacdev)
{
	synopGMACClearBits(gmacdev->MacBase, GmacFrameFilter, GmacFilter);
	return;
}
/**
  * Disables reception of all the frames to application.
  * GMAC passes only those received frames to application which 
  * pass SA/DA address filtering.
  * @param[in] pointer to synopGMACdevice.
  * \return void. 
  */
void synopGMAC_frame_filter_disable(synopGMACdevice * gmacdev)
{
	synopGMACSetBits(gmacdev->MacBase, GmacFrameFilter, GmacFilter);
	return;
}

/**
  * Disables Source address filtering.
  * When disabled GMAC forwards the received frames with updated SAMatch bit in RxStatus. 
  * @param[in] pointer to synopGMACdevice.
  * \return void.
  */
void synopGMAC_src_addr_filter_disable(synopGMACdevice * gmacdev)
{
	synopGMACClearBits(gmacdev->MacBase, GmacFrameFilter, GmacSrcAddrFilter);
	return;
}
/**
  * Enables the normal Destination address filtering.
  * @param[in] pointer to synopGMACdevice.
  * \return void.
  */
void synopGMAC_dst_addr_filter_normal(synopGMACdevice * gmacdev)
{
	synopGMACClearBits(gmacdev->MacBase, GmacFrameFilter, GmacDestAddrFilterNor);
	return;
}

/**
  * Enables forwarding of control frames.
  * When set forwards all the control frames (incl. unicast and multicast PAUSE frames).
  * @param[in] pointer to synopGMACdevice.
  * \return void.
  * \note Depends on RFE of FlowControlRegister[2]
  */
void synopGMAC_set_pass_control(synopGMACdevice * gmacdev,u32 passcontrol)
{	
	u32 data;
	data = synopGMACReadReg(gmacdev->MacBase, GmacFrameFilter);
	data &= (~GmacPassControl);
	data |= passcontrol;
	synopGMACWriteReg(gmacdev->MacBase,GmacFrameFilter,data);
	return;
}

/**
  * Enables Broadcast frames.
  * When enabled Address filtering module passes all incoming broadcast frames.
  * @param[in] pointer to synopGMACdevice.
  * \return void.
  */
void synopGMAC_broadcast_enable(synopGMACdevice * gmacdev)
{
	synopGMACClearBits(gmacdev->MacBase, GmacFrameFilter, GmacBroadcast );
	return;
}

/**
  * Disable Multicast frames.
  * When disabled multicast frame filtering depends on HMC bit.
  * @param[in] pointer to synopGMACdevice.
  * \return void.
  */
void synopGMAC_multicast_disable(synopGMACdevice * gmacdev)
{
	synopGMACClearBits(gmacdev->MacBase, GmacFrameFilter, GmacMulticastFilter);
	return;
}

/**
  * Disables multicast hash filtering.
  * When disabled GMAC performs perfect destination address filtering for multicast frames, it compares 
  * DA field with the value programmed in DA register.
  * @param[in] pointer to synopGMACdevice.
  * \return void.
  */
void synopGMAC_multicast_hash_filter_disable(synopGMACdevice * gmacdev)
{
	synopGMACClearBits(gmacdev->MacBase, GmacFrameFilter, GmacMcastHashFilter);
	return;
}

/**
  * Enables promiscous mode.
  * When enabled Address filter modules pass all incoming frames regardless of their Destination
  * and source addresses.
  * @param[in] pointer to synopGMACdevice.
  * \return void.
  */
void synopGMAC_promisc_enable(synopGMACdevice * gmacdev)
{
	synopGMACSetBits(gmacdev->MacBase, GmacFrameFilter, GmacPromiscuousMode);
	return;
}
/**
  * Clears promiscous mode.
  * When called the GMAC falls back to normal operation from promiscous mode.
  * @param[in] pointer to synopGMACdevice.
  * \return void.
  */
void synopGMAC_promisc_disable(synopGMACdevice * gmacdev)
{
	synopGMACClearBits(gmacdev->MacBase, GmacFrameFilter, GmacPromiscuousMode);
	return;
}


/**
  * Disables multicast hash filtering.
  * When disabled GMAC performs perfect destination address filtering for unicast frames, it compares 
  * DA field with the value programmed in DA register.
  * @param[in] pointer to synopGMACdevice.
  * \return void.
  */
void synopGMAC_unicast_hash_filter_disable(synopGMACdevice * gmacdev)
{
	synopGMACClearBits(gmacdev->MacBase, GmacFrameFilter, GmacUcastHashFilter);
	return;
}
	
/**
  * Disables detection of pause frames with stations unicast address.
  * When disabled GMAC only detects with the unique multicast address (802.3x).
  * @param[in] pointer to synopGMACdevice.
  * \return void.
  */
void synopGMAC_unicast_pause_frame_detect_disable(synopGMACdevice * gmacdev)
{
	synopGMACClearBits(gmacdev->MacBase, GmacFlowControl, GmacUnicastPauseFrame);
	return;
}
/**
  * Rx flow control enable.
  * When Enabled GMAC will decode the rx pause frame and disable the tx for a specified time.
  * @param[in] pointer to synopGMACdevice.
  * \return void.
  */
void synopGMAC_rx_flow_control_enable(synopGMACdevice * gmacdev)
{
	synopGMACSetBits(gmacdev->MacBase, GmacFlowControl, GmacRxFlowControl);
	return;
}
/**
  * Rx flow control disable.
  * When disabled GMAC will not decode pause frame.
  * @param[in] pointer to synopGMACdevice.
  * \return void.
  */
void synopGMAC_rx_flow_control_disable(synopGMACdevice * gmacdev)
{
	synopGMACClearBits(gmacdev->MacBase, GmacFlowControl, GmacRxFlowControl);
	return;
}
/**
  * Tx flow control enable.
  * When Enabled 
  * 	- In full duplex GMAC enables flow control operation to transmit pause frames.
  *	- In Half duplex GMAC enables the back pressure operation
  * @param[in] pointer to synopGMACdevice.
  * \return void.
  */
void synopGMAC_tx_flow_control_enable(synopGMACdevice * gmacdev)
{
	synopGMACSetBits(gmacdev->MacBase, GmacFlowControl, GmacTxFlowControl);
	return;
}

/**
  * Tx flow control disable.
  * When Disabled 
  * 	- In full duplex GMAC will not transmit any pause frames.
  *	- In Half duplex GMAC disables the back pressure feature.
  * @param[in] pointer to synopGMACdevice.
  * \return void.
  */
void synopGMAC_tx_flow_control_disable(synopGMACdevice * gmacdev)
{
	synopGMACClearBits(gmacdev->MacBase, GmacFlowControl, GmacTxFlowControl);
	return;
}


/**
  * This enables the pause frame generation after programming the appropriate registers.
  * presently activation is set at 3k and deactivation set at 4k. These may have to tweaked
  * if found any issues
  * @param[in] pointer to synopGMACdevice.
  * \return void.
  */
void synopGMAC_pause_control(synopGMACdevice *gmacdev)
{
	u32 omr_reg;
	u32 mac_flow_control_reg;
	omr_reg = synopGMACReadReg(gmacdev->DmaBase,DmaControl);
	omr_reg |= DmaRxFlowCtrlAct4K | DmaRxFlowCtrlDeact5K |DmaEnHwFlowCtrl;
	synopGMACWriteReg(gmacdev->DmaBase, DmaControl, omr_reg);

	mac_flow_control_reg = synopGMACReadReg(gmacdev->MacBase,GmacFlowControl);
	mac_flow_control_reg |= GmacRxFlowControl | GmacTxFlowControl | 0xFFFF0000;
	synopGMACWriteReg(gmacdev->MacBase,GmacFlowControl,mac_flow_control_reg);

	return;

}

/**
  * Example mac initialization sequence.
  * This function calls the initialization routines to initialize the GMAC register.
  * One can change the functions invoked here to have different configuration as per the requirement
  * @param[in] pointer to synopGMACdevice.
  * \return Returns 0 on success.
  */
s32 synopGMAC_mac_init(synopGMACdevice * gmacdev)
{
	u32 PHYreg;
	
	if(gmacdev->DuplexMode == FULLDUPLEX){
		printf("\n===phy FULLDUPLEX MODE\n");	//sw:	debug
		synopGMAC_wd_enable(gmacdev);
		synopGMAC_jab_enable(gmacdev);
		synopGMAC_frame_burst_enable(gmacdev);
		synopGMAC_jumbo_frame_disable(gmacdev);
		synopGMAC_rx_own_enable(gmacdev);
#if SYNOP_LOOPBACK_MODE
		synopGMAC_loopback_on(gmacdev);
#else
		synopGMAC_loopback_off(gmacdev);
#endif
		synopGMAC_set_full_duplex(gmacdev);  //1
		synopGMAC_retry_enable(gmacdev);
		synopGMAC_pad_crc_strip_disable(gmacdev);
		synopGMAC_back_off_limit(gmacdev,GmacBackoffLimit0);
		synopGMAC_deferral_check_disable(gmacdev);
		
		synopGMAC_tx_enable(gmacdev);	//according to Tang Dan's commitment
		synopGMAC_rx_enable(gmacdev);

		synopGMACSetBits(gmacdev->DmaBase,DmaControl, DmaStoreAndForward );//3
		synopGMACSetBits(gmacdev->DmaBase,DmaControl, DmaFwdErrorFrames );
		if(gmacdev->Speed == SPEED1000)
			synopGMAC_select_gmii(gmacdev);
		else{
			synopGMAC_select_mii(gmacdev);
			if(gmacdev->Speed == SPEED100)
				synopGMACSetBits(gmacdev->MacBase, GmacConfig, GmacFESpeed100);
			else
				synopGMACSetBits(gmacdev->MacBase, GmacConfig, GmacFESpeed10);
		}
		

		/*Frame Filter Configuration*/
	 	synopGMAC_frame_filter_disable(gmacdev); //2
		synopGMAC_set_pass_control(gmacdev,GmacPassControl0);
		synopGMAC_broadcast_enable(gmacdev);
		synopGMAC_src_addr_filter_disable(gmacdev);
		synopGMAC_multicast_disable(gmacdev);
		synopGMAC_dst_addr_filter_normal(gmacdev);
		synopGMAC_multicast_hash_filter_disable(gmacdev);
		synopGMAC_promisc_enable(gmacdev);
		synopGMAC_unicast_hash_filter_disable(gmacdev);
	
		/*Flow Control Configuration*/
		synopGMAC_unicast_pause_frame_detect_disable(gmacdev);
		synopGMAC_rx_flow_control_enable(gmacdev);
		synopGMAC_tx_flow_control_enable(gmacdev);
	}
	else{//for Half Duplex configuration
		
		printf("\n===phy HALFDUPLEX MODE\n");	//sw:	debug
		synopGMAC_wd_enable(gmacdev );
		synopGMAC_jab_enable(gmacdev);
		synopGMAC_frame_burst_enable(gmacdev);
		synopGMAC_jumbo_frame_disable(gmacdev);
		synopGMAC_rx_own_enable(gmacdev);
#if SYNOP_LOOPBACK_MODE
		synopGMAC_loopback_on(gmacdev);
#else
		synopGMAC_loopback_off(gmacdev);
#endif
		synopGMAC_set_half_duplex(gmacdev);
		synopGMAC_retry_enable(gmacdev);
		synopGMAC_pad_crc_strip_disable(gmacdev);
		synopGMAC_back_off_limit(gmacdev,GmacBackoffLimit0);
		synopGMAC_deferral_check_disable(gmacdev);

//sw: set efe & tsf
		synopGMACSetBits(gmacdev->DmaBase,DmaControl, DmaStoreAndForward );
		synopGMACSetBits(gmacdev->DmaBase,DmaControl, DmaFwdErrorFrames );
//sw: put it in the end
		synopGMAC_tx_enable(gmacdev);	
		synopGMAC_rx_enable(gmacdev);


		if(gmacdev->Speed == SPEED1000)
			synopGMAC_select_gmii(gmacdev);
		else{
			synopGMAC_select_mii(gmacdev );
			if(gmacdev->Speed == SPEED100)
				synopGMACSetBits(gmacdev->MacBase, GmacConfig, GmacFESpeed100 );
			else
				synopGMACSetBits(gmacdev->MacBase, GmacConfig, GmacFESpeed10 );
		}
		
//		synopGMACSetBits(gmacdev->MacBase, GmacConfig, GmacDisableCrs);
//		synopGMAC_select_gmii(gmacdev);

		/*Frame Filter Configuration*/
	 	synopGMAC_frame_filter_disable(gmacdev);

		synopGMAC_set_pass_control(gmacdev,GmacPassControl0);
		synopGMAC_broadcast_enable(gmacdev);
		synopGMAC_src_addr_filter_disable(gmacdev);
		synopGMAC_multicast_disable(gmacdev);
		synopGMAC_dst_addr_filter_normal(gmacdev);
		synopGMAC_multicast_hash_filter_disable(gmacdev);

		synopGMAC_promisc_enable(gmacdev);
		synopGMAC_unicast_hash_filter_disable(gmacdev);
		
//sw: loopback mode
//		synopGMAC_loopback_on(gmacdev);
		
		/*Flow Control Configuration*/		
		synopGMAC_unicast_pause_frame_detect_disable(gmacdev);
		synopGMAC_rx_flow_control_disable(gmacdev);
		synopGMAC_tx_flow_control_disable(gmacdev);

		/*To set PHY register to enable CRS on Transmit*/
	}
		gmacdev->LinkState = gmacdev->LinkState0;
	return 0;
}


/**
  * Sets the Mac address in to GMAC register.
  * This function sets the MAC address to the MAC register in question.
  * @param[in] pointer to synopGMACdevice to populate mac dma and phy addresses.
  * @param[in] Register offset for Mac address high
  * @param[in] Register offset for Mac address low
  * @param[in] buffer containing mac address to be programmed.
  * \return 0 upon success. Error code upon failure.
  */
s32 synopGMAC_set_mac_addr(synopGMACdevice *gmacdev, u32 MacHigh, u32 MacLow, u8 *MacAddr)
{
	u32 data;

		data = (MacAddr[5] << 8) | MacAddr[4];
		synopGMACWriteReg(gmacdev->MacBase,MacHigh,data);
		data = (MacAddr[3] << 24) | (MacAddr[2] << 16) | (MacAddr[1] << 8) | MacAddr[0] ;
		synopGMACWriteReg(gmacdev->MacBase,MacLow,data);

	return 0;
}


/**
  * Get the Mac address in to the address specified.
  * The mac register contents are read and written to buffer passed.
  * @param[in] pointer to synopGMACdevice to populate mac dma and phy addresses.
  * @param[in] Register offset for Mac address high
  * @param[in] Register offset for Mac address low
  * @param[out] buffer containing the device mac address.
  * \return 0 upon success. Error code upon failure.
  */
s32 synopGMAC_get_mac_addr(synopGMACdevice *gmacdev, u32 MacHigh, u32 MacLow, u8 *MacAddr)
{
	u32 data;
		
	data = synopGMACReadReg(gmacdev->MacBase,MacHigh);
	MacAddr[5] = (data >> 8) & 0xff;
	MacAddr[4] = (data)        & 0xff;

	data = synopGMACReadReg(gmacdev->MacBase,MacLow);
	MacAddr[3] = (data >> 24) & 0xff;
	MacAddr[2] = (data >> 16) & 0xff;
	MacAddr[1] = (data >> 8 ) & 0xff;
	MacAddr[0] = (data )      & 0xff;

	TR("MacAddr = 0x%x\t0x%x\t0x%x\t0x%x\t0x%x\t0x%x\n",MacAddr[0],MacAddr[1],MacAddr[2],MacAddr[3],MacAddr[4],MacAddr[5]);

	return 0;
}


/**
  * Attaches the synopGMAC device structure to the hardware.
  * Device structure is populated with MAC/DMA and PHY base addresses.
  * @param[in] pointer to synopGMACdevice to populate mac dma and phy addresses.
  * @param[in] GMAC IP mac base address.
  * @param[in] GMAC IP dma base address.
  * @param[in] GMAC IP phy base address.
  * \return 0 upon success. Error code upon failure.
  * \note This is important function. No kernel api provided by Synopsys 
  */

s32 synopGMAC_attach (synopGMACdevice * gmacdev, ulong macBase, ulong dmaBase, u32 phyBase,u8 *mac_addr) 
{
	/*Make sure the Device data strucure is cleared before we proceed further*/
	memset((void *) gmacdev,0,sizeof(synopGMACdevice));
	/*Populate the mac and dma base addresses*/
	gmacdev->MacBase = macBase;
	gmacdev->DmaBase = dmaBase;
	gmacdev->PhyBase = phyBase;
	{
		int i,j;
		u16 data;

		for (i = phyBase,j=0;j<32;i=(i+1)&0x1f,j++) 
		{
			synopGMAC_read_phy_reg(gmacdev->MacBase,i,2,&data);
			if(data != 0 && data != 0xffff) break;
			synopGMAC_read_phy_reg(gmacdev->MacBase,i,3,&data);
			if(data != 0 && data != 0xffff) break;
		}

		if(j==32) { 
			TR("phy_detect: can't find PHY!\n");
		}

		gmacdev->PhyBase = i;
	}

	/* Program/flash in the station/IP's Mac address */
	if (mac_addr)
	synopGMAC_set_mac_addr(gmacdev,GmacAddr0High,GmacAddr0Low, mac_addr); 

	return 0;	
}




/**
  * Initialize the rx descriptors for ring or chain mode operation.
  * 	- Status field is initialized to 0.
  *	- EndOfRing set for the last descriptor.
  *	- buffer1 and buffer2 set to 0 for ring mode of operation. (note)
  *	- data1 and data2 set to 0. (note)
  * @param[in] pointer to DmaDesc structure.
  * @param[in] whether end of ring
  * \return void.
  * \note Initialization of the buffer1, buffer2, data1,data2 and status are not done here. This only initializes whether one wants to use this descriptor
  * in chain mode or ring mode. For chain mode of operation the buffer2 and data2 are programmed before calling this function.
  */
void synopGMAC_rx_desc_init_ring(DmaDesc *desc, bool last_ring_desc)
{
	desc->status = 0;
	desc->length = last_ring_desc ? RxDescEndOfRing : 0;

	return;
}
/**
  * Initialize the tx descriptors for ring or chain mode operation.
  * 	- Status field is initialized to 0.
  *	- EndOfRing set for the last descriptor.
  *	- buffer1 and buffer2 set to 0 for ring mode of operation. (note)
  *	- data1 and data2 set to 0. (note)
  * @param[in] pointer to DmaDesc structure.
  * @param[in] whether end of ring
  * \return void.
  * \note Initialization of the buffer1, buffer2, data1,data2 and status are not done here. This only initializes whether one wants to use this descriptor
  * in chain mode or ring mode. For chain mode of operation the buffer2 and data2 are programmed before calling this function.
  */
void synopGMAC_tx_desc_init_ring(DmaDesc *desc, bool last_ring_desc)
{
	#ifdef ENH_DESC
	desc->status = last_ring_desc? TxDescEndOfRing : 0;
	desc->length = 0; 
	#else
	desc->status = 0;
	desc->length = last_ring_desc? TxDescEndOfRing : 0;
	#endif

//	desc->buffer1 = 0;
//	desc->buffer2 = 0;
//	desc->data1 = 0;
//	desc->data2 = 0;
	return;
}



/**
  * Initialize the rx descriptors for chain mode of operation.
  * 	- Status field is initialized to 0.
  *	- EndOfRing set for the last descriptor.
  *	- buffer1 and buffer2 set to 0.
  *	- data1 and data2 set to 0.
  * @param[in] pointer to DmaDesc structure.
  * @param[in] whether end of ring
  * \return void.
  */

void synopGMAC_rx_desc_init_chain(DmaDesc * desc)
{
	desc->status = 0;
	desc->length = RxDescChain;
	desc->buffer1 = 0;
	desc->data1 = 0;
	return;
}
/**
  * Initialize the rx descriptors for chain mode of operation.
  * 	- Status field is initialized to 0.
  *	- EndOfRing set for the last descriptor.
  *	- buffer1 and buffer2 set to 0.
  *	- data1 and data2 set to 0.
  * @param[in] pointer to DmaDesc structure.
  * @param[in] whether end of ring
  * \return void.
  */
void synopGMAC_tx_desc_init_chain(DmaDesc * desc)
{
	#ifdef ENH_DESC
	desc->status = TxDescChain;
	desc->length = 0;
	#else
	desc->length = TxDescChain;
	#endif
	desc->buffer1 = 0;
	desc->data1 = 0;
	return;
}


s32 synopGMAC_init_tx_rx_desc_queue(synopGMACdevice *gmacdev)
{
	s32 i;
	for(i =0; i < gmacdev -> TxDescCount; i++){
	synopGMAC_tx_desc_init_ring(gmacdev->TxDesc + i, i == gmacdev->TxDescCount-1);
	}
	TR("At line %d\n",__LINE__);
	for(i =0; i < gmacdev -> RxDescCount; i++){
	synopGMAC_rx_desc_init_ring(gmacdev->RxDesc + i, i == gmacdev->RxDescCount-1);
	}
	
	gmacdev->TxNext = 0;
	gmacdev->TxBusy = 0;
	gmacdev->TxNextDesc = gmacdev->TxDesc;
	gmacdev->TxBusyDesc = gmacdev->TxDesc;
	gmacdev->BusyTxDesc  = 0; 
	gmacdev->RxNext = 0;
	gmacdev->RxBusy = 0;
	gmacdev->RxNextDesc = gmacdev->RxDesc;
	gmacdev->RxBusyDesc = gmacdev->RxDesc;
	gmacdev->BusyRxDesc   = 0; 
	
	return -ESYNOPGMACNOERR;
}
/**
  * Programs the DmaRxBaseAddress with the Rx descriptor base address.
  * Rx Descriptor's base address is available in the gmacdev structure. This function progrms the 
  * Dma Rx Base address with the starting address of the descriptor ring or chain.
  * @param[in] pointer to synopGMACdevice.
  * \return returns void.
  */
void synopGMAC_init_rx_desc_base(synopGMACdevice *gmacdev)
{
	synopGMACWriteReg(gmacdev->DmaBase,DmaRxBaseAddr,(u32)gmacdev->RxDescDma );
	return;
}

/**
  * Programs the DmaTxBaseAddress with the Tx descriptor base address.
  * Tx Descriptor's base address is available in the gmacdev structure. This function progrms the 
  * Dma Tx Base address with the starting address of the descriptor ring or chain.
  * @param[in] pointer to synopGMACdevice.
  * \return returns void.
  */
void synopGMAC_init_tx_desc_base(synopGMACdevice *gmacdev)
{
	synopGMACWriteReg(gmacdev->DmaBase,DmaTxBaseAddr,(u32)gmacdev->TxDescDma);
	return;
}

/** 
  * Makes the Dma as owner for this descriptor.
  * This function sets the own bit of status field of the DMA descriptor,
  * indicating the DMA is the owner for this descriptor. 
  * @param[in] pointer to DmaDesc structure.
  * \return returns void.
  */
void synopGMAC_set_owner_dma(DmaDesc *desc)
{
desc->status |=  DescOwnByDma;
}

/** 
  * set tx descriptor to indicate SOF.
  * This Descriptor contains the start of ethernet frame.
  * @param[in] pointer to DmaDesc structure.
  * \return returns void.
  */
void synopGMAC_set_desc_sof(DmaDesc *desc)
{
#ifdef ENH_DESC
desc->status |= DescTxFirst;//ENH_DESC
#else
desc->length |= DescTxFirst;
#endif

}

/** 
  * set tx descriptor to indicate EOF.
  * This descriptor contains the End of ethernet frame.
  * @param[in] pointer to DmaDesc structure.
  * \return returns void.
  */
void synopGMAC_set_desc_eof(DmaDesc *desc)
{
#ifdef ENH_DESC
desc->status |= DescTxLast;//ENH_DESC
#else
desc->length |= DescTxLast;
#endif
}


/** 
  * checks whether this descriptor contains start of frame.
  * This function is to check whether the descriptor's data buffer 
  * contains a fresh ethernet frame?
  * @param[in] pointer to DmaDesc structure.
  * \return returns true if SOF in current descriptor, else returns fail.
  */
bool synopGMAC_is_sof_in_rx_desc(DmaDesc *desc)
{
return ((desc->status & DescRxFirst) == DescRxFirst);                      
}

/** 
  * checks whether this descriptor contains end of frame.
  * This function is to check whether the descriptor's data buffer 
  * contains end of ethernet frame?
  * @param[in] pointer to DmaDesc structure.
  * \return returns true if SOF in current descriptor, else returns fail.
  */
bool synopGMAC_is_eof_in_rx_desc(DmaDesc *desc)
{
return ((desc->status & DescRxLast) == DescRxLast);                      
}

/** 
  * checks whether destination address filter failed in the rx frame.
  * @param[in] pointer to DmaDesc structure.
  * \return returns true if Failed, false if not.
  */
bool synopGMAC_is_da_filter_failed(DmaDesc *desc)
{
return ((desc->status & DescDAFilterFail) == DescDAFilterFail);                      
}

/** 
  * checks whether source address filter failed in the rx frame.
  * @param[in] pointer to DmaDesc structure.
  * \return returns true if Failed, false if not.
  */
bool synopGMAC_is_sa_filter_failed(DmaDesc *desc)
{
return ((desc->status & DescSAFilterFail) == DescSAFilterFail);                      
}

/** 
  * Checks whether the descriptor is owned by DMA.
  * If descriptor is owned by DMA then the OWN bit is set to 1. This API is same for both ring and chain mode.
  * @param[in] pointer to DmaDesc structure.
  * \return returns true if Dma owns descriptor and false if not.
  */
bool synopGMAC_is_desc_owned_by_dma(DmaDesc *desc)
{
return ((desc->status & DescOwnByDma) == DescOwnByDma );
}

/**
  * returns the byte length of received frame including CRC.
  * This returns the no of bytes received in the received ethernet frame including CRC(FCS).
  * @param[in] pointer to DmaDesc structure.
  * \return returns the length of received frame lengths in bytes.
  */
u32 synopGMAC_get_rx_desc_frame_length(u32 status)
{
	return ((status & DescFrameLengthMask) >> DescFrameLengthShift);
}

/**
  * Checks whether the descriptor is valid
  * if no errors such as CRC/Receive Error/Watchdog Timeout/Late collision/Giant Frame/Overflow/Descriptor
  * error the descritpor is said to be a valid descriptor.
  * @param[in] pointer to DmaDesc structure.
  * \return True if desc valid. false if error.
  */
bool synopGMAC_is_desc_valid(u32 status)
{
	return ((status & DescError) == 0);
}

/**
  * Checks whether the descriptor is empty.
  * If the buffer1 and buffer2 lengths are zero in ring mode descriptor is empty.
  * In chain mode buffer2 length is 0 but buffer2 itself contains the next descriptor address.
  * @param[in] pointer to DmaDesc structure.
  * \return returns true if descriptor is empty, false if not empty.
  */
bool synopGMAC_is_desc_empty(DmaDesc *desc)
{
	//if both the buffer1 length and buffer2 length are zero desc is empty
	return(((desc->length  & DescSize1Mask) == 0) && ((desc->length  & DescSize2Mask) == 0) );
}


/**
  * Checks whether the rx descriptor is valid.
  * if rx descripor is not in error and complete frame is available in the same descriptor
  * @param[in] pointer to DmaDesc structure.
  * \return returns true if no error and first and last desc bits are set, otherwise it returns false.
  */
bool synopGMAC_is_rx_desc_valid(u32 status)
{
	return ((status & DescError) == 0) && ((status & DescRxFirst) == DescRxFirst) && ((status & DescRxLast) == DescRxLast);
}

/**
  * Checks whether the tx is aborted due to collisions.
  * @param[in] pointer to DmaDesc structure.
  * \return returns true if collisions, else returns false.
  */
bool synopGMAC_is_tx_aborted(u32 status)
{
	return (((status & DescTxLateCollision) == DescTxLateCollision) | ((status & DescTxExcCollisions) == DescTxExcCollisions));

}

/**
  * Checks whether the tx carrier error.
  * @param[in] pointer to DmaDesc structure.
  * \return returns true if carrier error occured, else returns falser.
  */
bool synopGMAC_is_tx_carrier_error(u32 status)
{
	return (((status & DescTxLostCarrier) == DescTxLostCarrier)  | ((status & DescTxNoCarrier) == DescTxNoCarrier));
}


/**
  * Gives the transmission collision count.
  * returns the transmission collision count indicating number of collisions occured before the frame was transmitted.
  * Make sure to check excessive collision didnot happen to ensure the count is valid.
  * @param[in] pointer to DmaDesc structure.
  * \return returns the count value of collision.
  */
u32 synopGMAC_get_tx_collision_count(u32 status)
{
	return ((status & DescTxCollMask) >> DescTxCollShift);
}
u32 synopGMAC_is_exc_tx_collisions(u32 status)
{
	return ((status & DescTxExcCollisions) == DescTxExcCollisions);
}


/**
  * Check for damaged frame due to overflow or collision.
  * Retruns true if rx frame was damaged due to buffer overflow in MTL or late collision in half duplex mode.
  * @param[in] pointer to DmaDesc structure.
  * \return returns true if error else returns false.
  */
bool synopGMAC_is_rx_frame_damaged(u32 status)
{
//bool synopGMAC_dma_rx_collisions(u32 status)
	return (((status & DescRxDamaged) == DescRxDamaged) | ((status & DescRxCollision) == DescRxCollision));
}

/**
  * Check for damaged frame due to collision.
  * Retruns true if rx frame was damaged due to late collision in half duplex mode.
  * @param[in] pointer to DmaDesc structure.
  * \return returns true if error else returns false.
  */
bool synopGMAC_is_rx_frame_collision(u32 status)
{
//bool synopGMAC_dma_rx_collisions(u32 status)
	return ((status & DescRxCollision) == DescRxCollision);
}

/**
  * Check for receive CRC error.
  * Retruns true if rx frame CRC error occured.
  * @param[in] pointer to DmaDesc structure.
  * \return returns true if error else returns false.
  */
bool synopGMAC_is_rx_crc(u32 status)
{
//u32 synopGMAC_dma_rx_crc(u32 status)
	return ((status & DescRxCrc) == DescRxCrc);
}

/**
  * Indicates rx frame has non integer multiple of bytes. (odd nibbles).
  * Retruns true if dribbling error in rx frame.
  * @param[in] pointer to DmaDesc structure.
  * \return returns true if error else returns false.
  */
bool synopGMAC_is_frame_dribbling_errors(u32 status)
{
//u32 synopGMAC_dma_rx_frame_errors(u32 status)
	return ((status & DescRxDribbling) == DescRxDribbling);
}

/**
  * Indicates error in rx frame length.
  * Retruns true if received frame length doesnot match with the length field
  * @param[in] pointer to DmaDesc structure.
  * \return returns true if error else returns false.
  */
bool synopGMAC_is_rx_frame_length_errors(u32 status)
{
//u32 synopGMAC_dma_rx_length_errors(u32 status)
	return((status & DescRxLengthError) == DescRxLengthError);
}

/**
  * Checks whether this rx descriptor is last rx descriptor.
  * This returns true if it is last descriptor either in ring mode or in chain mode.
  * @param[in] pointer to devic structure.
  * @param[in] pointer to DmaDesc structure.
  * \return returns true if it is last descriptor, false if not.
  * \note This function should not be called before initializing the descriptor using synopGMAC_desc_init().
  */
bool synopGMAC_is_last_rx_desc(synopGMACdevice * gmacdev,DmaDesc *desc)
{
//bool synopGMAC_is_last_desc(DmaDesc *desc)
return (((desc->length & RxDescEndOfRing) == RxDescEndOfRing) || ((u32)gmacdev->RxDesc == desc->data2));
}

/**
  * Checks whether this tx descriptor is last tx descriptor.
  * This returns true if it is last descriptor either in ring mode or in chain mode.
  * @param[in] pointer to devic structure.
  * @param[in] pointer to DmaDesc structure.
  * \return returns true if it is last descriptor, false if not.
  * \note This function should not be called before initializing the descriptor using synopGMAC_desc_init().
  */
bool synopGMAC_is_last_tx_desc(synopGMACdevice * gmacdev,DmaDesc *desc)
{
//bool synopGMAC_is_last_desc(DmaDesc *desc)
#ifdef ENH_DESC
	return (((desc->status & TxDescEndOfRing) == TxDescEndOfRing) || ((u32)gmacdev->TxDesc == desc->data2));
#else
	return (((desc->length & TxDescEndOfRing) == TxDescEndOfRing) || ((u32)gmacdev->TxDesc == desc->data2));
#endif
}

/**
  * Checks whether this rx descriptor is in chain mode.
  * This returns true if it is this descriptor is in chain mode.
  * @param[in] pointer to DmaDesc structure.
  * \return returns true if chain mode is set, false if not.
  */
bool synopGMAC_is_rx_desc_chained(DmaDesc * desc)
{
	return((desc->length & RxDescChain) == RxDescChain);             
}

/**
  * Checks whether this tx descriptor is in chain mode.
  * This returns true if it is this descriptor is in chain mode.
  * @param[in] pointer to DmaDesc structure.
  * \return returns true if chain mode is set, false if not.
  */
bool synopGMAC_is_tx_desc_chained(DmaDesc * desc)
{
#ifdef ENH_DESC
	return((desc->status & TxDescChain) == TxDescChain);             
#else
	return((desc->length & TxDescChain) == TxDescChain);             
#endif
}

/**
  * Driver Api to get the descriptor field information.
  * This returns the status, dma-able address of buffer1, the length of buffer1, virtual address of buffer1
  * dma-able address of buffer2, length of buffer2, virtural adddress of buffer2.
  * @param[in]  pointer to DmaDesc structure.
  * @param[out] pointer to status field fo descriptor.
  * @param[out] dma-able address of buffer1.
  * @param[out] length of buffer1.
  * @param[out] virtual address of buffer1.
  * @param[out] dma-able address of buffer2.
  * @param[out] length of buffer2.
  * @param[out] virtual address of buffer2.
  * \return returns void.
  */
void synopGMAC_get_desc_data(DmaDesc * desc, u32 * Status, dma_addr_t * Buffer1, u32 * Length1, ulong * Data1, dma_addr_t * Buffer2, u32 * Length2, ulong * Data2)
{

	if(Status != 0)   
		*Status = desc->status;

	if(Buffer1 != 0)
		*Buffer1 = desc->buffer1;
	if(Length1 != 0)
		*Length1 = (desc->length & DescSize1Mask) >> DescSize1Shift;
	if(Data1 != 0)
		*Data1 = desc->data1;

	if(Buffer2 != 0)
		*Buffer2 = desc->buffer2;
	if(Length2 != 0)
		*Length2 = (desc->length & DescSize2Mask) >> DescSize2Shift;
	if(Data1 != 0)
		*Data2 = desc->data2;
	
	return;

}

#ifdef ENH_DESC_8W
/**
  * This function is defined two times. Once when the code is compiled for ENHANCED DESCRIPTOR SUPPORT and Once for Normal descriptor
  * Get the index and address of Tx desc.
  * This api is same for both ring mode and chain mode.
  * This function tracks the tx descriptor the DMA just closed after the transmission of data from this descriptor is 
  * over. This returns the descriptor fields to the caller.
  * @param[in] pointer to synopGMACdevice.
  * @param[out] status field of the descriptor.
  * @param[out] Dma-able buffer1 pointer.
  * @param[out] length of buffer1 (Max is 2048).
  * @param[out] virtual pointer for buffer1.
  * @param[out] Dma-able buffer2 pointer.
  * @param[out] length of buffer2 (Max is 2048).
  * @param[out] virtual pointer for buffer2.
  * @param[out] u32 data indicating whether the descriptor is in ring mode or chain mode.
  * \return returns present tx descriptor index on success. Negative value if error.
  */
s32 synopGMAC_get_tx_qptr(synopGMACdevice * gmacdev, u32 * Status, dma_addr_t * Buffer1, u32 * Length1, ulong * Data1, dma_addr_t * Buffer2, u32 * Length2, ulong * Data2,
                          u32 * Ext_Status, u32 * Time_Stamp_High, u32 * Time_Stamp_Low)
{
	u32  txover      = gmacdev->TxBusy;
	DmaDesc * txdesc = gmacdev->TxBusyDesc;
	
	if(synopGMAC_is_desc_owned_by_dma(txdesc))
		return -1;
	if(synopGMAC_is_desc_empty(txdesc))
		return -1;

	(gmacdev->BusyTxDesc)--; //busy tx descriptor is reduced by one as it will be handed over to Processor now

	if(Status != 0)   
		*Status = txdesc->status;

	if(Ext_Status != 0)
		*Ext_Status = txdesc->extstatus;
        if(Time_Stamp_High != 0)
		*Time_Stamp_High = txdesc->timestamphigh; 
        if(Time_Stamp_Low != 0)
		*Time_Stamp_High = txdesc->timestamplow; 

	if(Buffer1 != 0)
		*Buffer1 = txdesc->buffer1;
	if(Length1 != 0)
		*Length1 = (txdesc->length & DescSize1Mask) >> DescSize1Shift;
	if(Data1 != 0)
		*Data1 = txdesc->data1;

	if(Buffer2 != 0)
		*Buffer2 = txdesc->buffer2;
	if(Length2 != 0)
		*Length2 = (txdesc->length & DescSize2Mask) >> DescSize2Shift;
	if(Data1 != 0)
		*Data2 = txdesc->data2;

	gmacdev->TxBusy     = synopGMAC_is_last_tx_desc(gmacdev,txdesc) ? 0 : txover + 1;

	if(synopGMAC_is_tx_desc_chained(txdesc)){
	   	gmacdev->TxBusyDesc = (DmaDesc *)txdesc->data2;
		synopGMAC_tx_desc_init_chain(txdesc);
	}
	else{
		gmacdev->TxBusyDesc = synopGMAC_is_last_tx_desc(gmacdev,txdesc) ? gmacdev->TxDesc : (txdesc + 1);
		synopGMAC_tx_desc_init_ring(txdesc, synopGMAC_is_last_tx_desc(gmacdev,txdesc));
	}
	TR("%02d %08x %08x %08x %08x %08x %08x %08x\n",txover,(u32)txdesc,txdesc->status,txdesc->length,txdesc->buffer1,txdesc->buffer2,txdesc->data1,txdesc->data2);

	return txover;	
}
#else

/**
  * Get the index and address of Tx desc.
  * This api is same for both ring mode and chain mode.
  * This function tracks the tx descriptor the DMA just closed after the transmission of data from this descriptor is 
  * over. This returns the descriptor fields to the caller.
  * @param[in] pointer to synopGMACdevice.
  * @param[out] status field of the descriptor.
  * @param[out] Dma-able buffer1 pointer.
  * @param[out] length of buffer1 (Max is 2048).
  * @param[out] virtual pointer for buffer1.
  * @param[out] Dma-able buffer2 pointer.
  * @param[out] length of buffer2 (Max is 2048).
  * @param[out] virtual pointer for buffer2.
  * @param[out] u32 data indicating whether the descriptor is in ring mode or chain mode.
  * \return returns present tx descriptor index on success. Negative value if error.
  */
s32 synopGMAC_get_tx_qptr(synopGMACdevice * gmacdev, u32 * Status, dma_addr_t * Buffer1, u32 * Length1, ulong * Data1, dma_addr_t * Buffer2, u32 * Length2, ulong * Data2 )
{
	u32  txover      = gmacdev->TxBusy;
	DmaDesc * txdesc = gmacdev->TxBusyDesc;
	int i;
	
//sw: dbg
	

	//pci_sync_cache(0, (vm_offset_t)txdesc, 64, SYNC_R);
	//pci_sync_cache(0, (vm_offset_t)txdesc, 64, SYNC_W);
#if SYNOP_TX_DEBUG
	TR("Cache sync before get a used tx dma desc!\n");
	TR("\n==%02d %08x %08x %08x %08x %08x %08x %08x\n",txover,(u32)txdesc,txdesc->status,txdesc->length,txdesc->buffer1,txdesc->buffer2,txdesc->data1,txdesc->data2);
#endif
	if(synopGMAC_is_desc_owned_by_dma(txdesc))
	{	
#if SYNOP_TX_DEBUG
		TR("==desc owned!\n");
#endif
		return -1;
	}
#if 0
	for(i=0;i<500000;i++)
	{
	if(synopGMAC_is_desc_empty(txdesc))
	{	
#if SYNOP_TX_DEBUG
	//	TR("==desc owned by dma\n");
#endif
	//	return -1;
		continue;
	}
	else
		break;
	}

	if(i>=500000)
	{
		TR("i=%d\n",i);
		return -1;
	}
	do
	{
		;
	}while(synopGMAC_is_desc_empty(txdesc));
#endif
//	gmacdev->TxBusy     = synopGMAC_is_last_tx_desc(gmacdev,txdesc) ? 0 : txover + 1;
//	gmacdev->TxBusyDesc = synopGMAC_is_last_tx_desc(gmacdev,txdesc) ? gmacdev->TxDesc : (txdesc + 1);
	if(synopGMAC_is_desc_empty(txdesc))
	{
#if SYNOP_TX_DEBUG
		TR("Tx Desc Empty!\n");
#endif
		return -1;
	}
	(gmacdev->BusyTxDesc)--; //busy tx descriptor is reduced by one as it will be handed over to Processor now

	if(Status != 0)   
		*Status = txdesc->status;

	if(Buffer1 != 0)
		*Buffer1 = txdesc->buffer1;
	if(Length1 != 0)
		*Length1 = (txdesc->length & DescSize1Mask) >> DescSize1Shift;
	if(Data1 != 0)
		*Data1 = txdesc->data1;

	if(Buffer2 != 0)
		*Buffer2 = txdesc->buffer2;
	if(Length2 != 0)
		*Length2 = (txdesc->length & DescSize2Mask) >> DescSize2Shift;
	if(Data1 != 0)
		*Data2 = txdesc->data2;

	gmacdev->TxBusy     = synopGMAC_is_last_tx_desc(gmacdev,txdesc) ? 0 : txover + 1;

	if(synopGMAC_is_tx_desc_chained(txdesc)){
	   	gmacdev->TxBusyDesc = (DmaDesc *)txdesc->data2;
		synopGMAC_tx_desc_init_chain(txdesc);
	}
	else{
		gmacdev->TxBusyDesc = synopGMAC_is_last_tx_desc(gmacdev,txdesc) ? gmacdev->TxDesc : (txdesc + 1);
		synopGMAC_tx_desc_init_ring(txdesc, synopGMAC_is_last_tx_desc(gmacdev,txdesc));
	}
	//TR("%02d %08x %08x %08x %08x %08x %08x %08x\n",txover,(u32)txdesc,txdesc->status,txdesc->length,txdesc->buffer1,txdesc->buffer2,txdesc->data1,txdesc->data2);
	//pci_sync_cache(0, (vm_offset_t)txdesc, 64, SYNC_W);
#if SYNOP_TX_DEBUG
	TR("Cache sync after re-init a tx dma desc!\n");
#endif

	return txover;	
}

#endif
/**
  * Populate the tx desc structure with the buffer address.
  * Once the driver has a packet ready to be transmitted, this function is called with the 
  * valid dma-able buffer addresses and their lengths. This function populates the descriptor
  * and make the DMA the owner for the descriptor. This function also controls whetther Checksum
  * offloading to be done in hardware or not. 
  * This api is same for both ring mode and chain mode.
  * @param[in] pointer to synopGMACdevice.
  * @param[in] Dma-able buffer1 pointer.
  * @param[in] length of buffer1 (Max is 2048).
  * @param[in] virtual pointer for buffer1.
  * @param[in] Dma-able buffer2 pointer.
  * @param[in] length of buffer2 (Max is 2048).
  * @param[in] virtual pointer for buffer2.
  * @param[in] u32 data indicating whether the descriptor is in ring mode or chain mode.
  * @param[in] u32 indicating whether the checksum offloading in HW/SW.
  * \return returns present tx descriptor index on success. Negative value if error.
  */
u32 len;
s32 synopGMAC_set_tx_qptr(synopGMACdevice * gmacdev, dma_addr_t Buffer1, u32 Length1, ulong Data1, dma_addr_t Buffer2, u32 Length2, ulong Data2,u32 offload_needed,u32 * index, DmaDesc * Dpr)
{
	u32  txnext      = gmacdev->TxNext;
	DmaDesc * txdesc = gmacdev->TxNextDesc;

	*index = txnext;
	Dpr = txdesc;

	if(!synopGMAC_is_desc_empty(txdesc))
	{
		TR("set tx qptr: desc empty!\n");
		return -1;
	}

	(gmacdev->BusyTxDesc)++; //busy tx descriptor is reduced by one as it will be handed over to Processor now
	
	if(synopGMAC_is_tx_desc_chained(txdesc)){
		txdesc->length |= ((Length1 <<DescSize1Shift) & DescSize1Mask);
		#ifdef ENH_DESC
		txdesc->status |=  (DescTxFirst | DescTxLast | DescTxIntEnable); //ENH_DESC
		#else
		txdesc->length |=  (DescTxFirst | DescTxLast | DescTxIntEnable); //Its always assumed that complete data will fit in to one descriptor
		#endif

	 	txdesc->buffer1 = Buffer1;
		txdesc->data1 = Data1;

	if(offload_needed){
		/*
		 Make sure that the OS you are running supports the IP and TCP checkusm offloaidng,
		 before calling any of the functions given below.		 
		 */
		synopGMAC_tx_checksum_offload_ipv4hdr(gmacdev, txdesc);
		synopGMAC_tx_checksum_offload_tcponly(gmacdev, txdesc);
//		synopGMAC_tx_checksum_offload_tcp_pseudo(gmacdev, txdesc);
	}
		#ifdef ENH_DESC
		txdesc->status |= DescOwnByDma;//ENH_DESC
		#else
		txdesc->status = DescOwnByDma;
		#endif

		gmacdev->TxNext = synopGMAC_is_last_tx_desc(gmacdev,txdesc) ? 0 : txnext + 1;
	   	gmacdev->TxNextDesc = (DmaDesc *)txdesc->data2;
	}
	else{
		txdesc->length |= (((Length1 <<DescSize1Shift) & DescSize1Mask) | ((Length2 <<DescSize2Shift) & DescSize2Mask));
		#ifdef ENH_DESC
		txdesc->status |=  (DescTxFirst | DescTxLast | DescTxIntEnable); //ENH_DESC
		#else
		txdesc->length |=  (DescTxFirst | DescTxLast | DescTxIntEnable); //Its always assumed that complete data will fit in to one descriptor
		#endif

	 	txdesc->buffer1 = Buffer1;
		txdesc->data1 = Data1;

	 	txdesc->buffer2 = Buffer2;
		txdesc->data2 = Data2;

		if(offload_needed){
		/*
		 Make sure that the OS you are running supports the IP and TCP checkusm offloaidng,
		 before calling any of the functions given below.		 
		 */
//sw: i am not sure about the checksum.so i omit it in the outside
		synopGMAC_tx_checksum_offload_ipv4hdr(gmacdev, txdesc);
		synopGMAC_tx_checksum_offload_tcponly(gmacdev, txdesc);
//		synopGMAC_tx_checksum_offload_tcp_pseudo(gmacdev, txdesc);
		}
		#ifdef ENH_DESC	
		txdesc->status |= DescOwnByDma;//ENH_DESC
		#else
		txdesc->status = DescOwnByDma;
		#endif

#if 1
		gmacdev->TxNext = synopGMAC_is_last_tx_desc(gmacdev,txdesc) ? 0 : txnext + 1;
		gmacdev->TxNextDesc = synopGMAC_is_last_tx_desc(gmacdev,txdesc) ? gmacdev->TxDesc : (txdesc + 1);
#endif
	}


#if SYNOP_TX_DEBUG
	TR("%02d %08x %08x %08x %08x %08x %08x %08x\n",txnext,(u32)txdesc,txdesc->status,txdesc->length,txdesc->buffer1,txdesc->buffer2,txdesc->data1,txdesc->data2);
#endif
#if SYNOP_TX_DEBUG
	TR("Cache sync to set a tx desc!\n");
#endif
#if SYNOP_TX_DEBUG
	//TR("Cache sync for data in the buf of the tx desc!\n");
#endif
	return txnext;	
}
#ifdef ENH_DESC_8W
/**
  * Prepares the descriptor to receive packets.
  * The descriptor is allocated with the valid buffer addresses (sk_buff address) and the length fields
  * and handed over to DMA by setting the ownership. After successful return from this function the
  * descriptor is added to the receive descriptor pool/queue.
  * This api is same for both ring mode and chain mode.
  * @param[in] pointer to synopGMACdevice.
  * @param[in] Dma-able buffer1 pointer.
  * @param[in] length of buffer1 (Max is 2048).
  * @param[in] Dma-able buffer2 pointer.
  * @param[in] length of buffer2 (Max is 2048).
  * @param[in] u32 data indicating whether the descriptor is in ring mode or chain mode.
  * \return returns present rx descriptor index on success. Negative value if error.
  */
s32 synopGMAC_set_rx_qptr(synopGMACdevice * gmacdev, dma_addr_t Buffer1, u32 Length1, ulong Data1, dma_addr_t Buffer2, u32 Length2, ulong Data2)
{
	u32  rxnext      = gmacdev->RxNext;
	DmaDesc * rxdesc = gmacdev->RxNextDesc;

	if(!synopGMAC_is_desc_empty(rxdesc))
		return -1;

	if(synopGMAC_is_rx_desc_chained(rxdesc)){
		rxdesc->length |= ((Length1 <<DescSize1Shift) & DescSize1Mask);

		rxdesc->buffer1 = Buffer1;
		rxdesc->data1 = Data1;

		rxdesc->extstatus = 0;
		rxdesc->reserved1 = 0;
		rxdesc->timestamplow = 0;
		rxdesc->timestamphigh = 0;

		if((rxnext % MODULO_INTERRUPT) !=0)
		rxdesc->length |= RxDisIntCompl;		
  
		rxdesc->status = DescOwnByDma;

		gmacdev->RxNext     = synopGMAC_is_last_rx_desc(gmacdev,rxdesc) ? 0 : rxnext + 1;
	   	gmacdev->RxNextDesc = (DmaDesc *)rxdesc->data2;
	}
	else{
		rxdesc->length |= (((Length1 <<DescSize1Shift) & DescSize1Mask) | ((Length2 << DescSize2Shift) & DescSize2Mask));

		rxdesc->buffer1 = Buffer1;
		rxdesc->data1 = Data1;

		rxdesc->extstatus = 0;
		rxdesc->reserved1 = 0;
		rxdesc->timestamplow = 0;
		rxdesc->timestamphigh = 0;

		rxdesc->buffer2 = Buffer2;
		rxdesc->data2 = Data2;
	
		if((rxnext % MODULO_INTERRUPT) !=0)
		rxdesc->length |= RxDisIntCompl;		

		rxdesc->status = DescOwnByDma;

		gmacdev->RxNext     = synopGMAC_is_last_rx_desc(gmacdev,rxdesc) ? 0 : rxnext + 1;
		gmacdev->RxNextDesc = synopGMAC_is_last_rx_desc(gmacdev,rxdesc) ? gmacdev->RxDesc : (rxdesc + 1);
	}
#if SYNOP_RX_DEBUG
	TR("%02d %08x %08x %08x %08x %08x %08x %08x %08x %08x\n",rxnext,(u32)rxdesc,rxdesc->status,rxdesc->length,rxdesc->buffer1,rxdesc->buffer2,rxdesc->data1,rxdesc->data2,rxdesc->dummy1,rxdesc->dummy2);
#endif

	(gmacdev->BusyRxDesc)++; //One descriptor will be given to Hardware. So busy count incremented by one
	//pci_sync_cache(0, (vm_offset_t)rxdesc,64, SYNC_W);
	return rxnext;
}

#else
/**
  * Prepares the descriptor to receive packets.
  * The descriptor is allocated with the valid buffer addresses (sk_buff address) and the length fields
  * and handed over to DMA by setting the ownership. After successful return from this function the
  * descriptor is added to the receive descriptor pool/queue.
  * This api is same for both ring mode and chain mode.
  * @param[in] pointer to synopGMACdevice.
  * @param[in] Dma-able buffer1 pointer.
  * @param[in] length of buffer1 (Max is 2048).
  * @param[in] Dma-able buffer2 pointer.
  * @param[in] length of buffer2 (Max is 2048).
  * @param[in] u32 data indicating whether the descriptor is in ring mode or chain mode.
  * \return returns present rx descriptor index on success. Negative value if error.
  */
s32 synopGMAC_set_rx_qptr(synopGMACdevice * gmacdev, dma_addr_t Buffer1, u32 Length1, ulong Data1, dma_addr_t Buffer2, u32 Length2, ulong Data2)
{
	u32  rxnext      = gmacdev->RxNext;
	DmaDesc * rxdesc = gmacdev->RxNextDesc;

	if(!synopGMAC_is_desc_empty(rxdesc))
		return -1;

//	printf("refill desc %d dma_addr1 0x%x\n", rxdesc-gmacdev->RxDesc, Buffer1);
	if(synopGMAC_is_rx_desc_chained(rxdesc)){
		rxdesc->length |= ((Length1 <<DescSize1Shift) & DescSize1Mask);

		rxdesc->buffer1 = Buffer1;
		rxdesc->data1 = Data1;

		if((rxnext % MODULO_INTERRUPT) !=0)
		rxdesc->length |= RxDisIntCompl;		

		rxdesc->status = DescOwnByDma;

		gmacdev->RxNext     = synopGMAC_is_last_rx_desc(gmacdev,rxdesc) ? 0 : rxnext + 1;
	   	gmacdev->RxNextDesc = (DmaDesc *)rxdesc->data2;
	}
	else{
		rxdesc->length |= (((Length1 <<DescSize1Shift) & DescSize1Mask) | ((Length2 << DescSize2Shift) & DescSize2Mask));

		rxdesc->buffer1 = Buffer1;
		rxdesc->data1 = Data1;

		rxdesc->buffer2 = Buffer2;
		rxdesc->data2 = Data2;
	
		if((rxnext % MODULO_INTERRUPT) !=0)
		rxdesc->length |= RxDisIntCompl;		

		rxdesc->status = DescOwnByDma;

		gmacdev->RxNext     = synopGMAC_is_last_rx_desc(gmacdev,rxdesc) ? 0 : rxnext + 1;
		gmacdev->RxNextDesc = synopGMAC_is_last_rx_desc(gmacdev,rxdesc) ? gmacdev->RxDesc : (rxdesc + 1);
	}
#if SYNOP_RX_DEBUG
	TR("%02d %08x %08x %08x %08x %08x %08x %08x\n",rxnext,(u32)rxdesc,rxdesc->status,rxdesc->length,rxdesc->buffer1,rxdesc->buffer2,rxdesc->data1,rxdesc->data2);
#endif
	(gmacdev->BusyRxDesc)++; //One descriptor will be given to Hardware. So busy count incremented by one
	return rxnext;
}

s32 synopGMAC_set_rx_qptr_init(synopGMACdevice * gmacdev, dma_addr_t Buffer1, u32 Length1, ulong Data1, dma_addr_t Buffer2, u32 Length2, ulong Data2)
{
	u32  rxnext      = gmacdev->RxNext;
	DmaDesc * rxdesc = gmacdev->RxNextDesc;

/* sw	
	if(synopGMAC_is_desc_owned_by_dma(rxdesc))
		return -1;
*/

	if(!synopGMAC_is_desc_empty(rxdesc))
		return -1;

	if(synopGMAC_is_rx_desc_chained(rxdesc)){
		rxdesc->length |= ((Length1 <<DescSize1Shift) & DescSize1Mask);

		rxdesc->buffer1 = Buffer1;
		rxdesc->data1 = Data1;

		if((rxnext % MODULO_INTERRUPT) !=0)
		rxdesc->length |= RxDisIntCompl;		

		rxdesc->status = DescOwnByDma;
		rxdesc->status = 0;

		gmacdev->RxNext     = synopGMAC_is_last_rx_desc(gmacdev,rxdesc) ? 0 : rxnext + 1;
	   	gmacdev->RxNextDesc = (DmaDesc *)rxdesc->data2;
	}
	else{
		rxdesc->length |= (((Length1 <<DescSize1Shift) & DescSize1Mask) | ((Length2 << DescSize2Shift) & DescSize2Mask));

		rxdesc->buffer1 = Buffer1;
		rxdesc->data1 = Data1;

		rxdesc->buffer2 = Buffer2;
		rxdesc->data2 = Data2;
	
		if((rxnext % MODULO_INTERRUPT) !=0)
		rxdesc->length |= RxDisIntCompl;		

		rxdesc->status = DescOwnByDma;
		rxdesc->status = 0;

		gmacdev->RxNext     = synopGMAC_is_last_rx_desc(gmacdev,rxdesc) ? 0 : rxnext + 1;
		gmacdev->RxNextDesc = synopGMAC_is_last_rx_desc(gmacdev,rxdesc) ? gmacdev->RxDesc : (rxdesc + 1);
	}
	TR("%02d %08x %08x %08x %08x %08x %08x %08x\n",rxnext,(u32)rxdesc,rxdesc->status,rxdesc->length,rxdesc->buffer1,rxdesc->buffer2,rxdesc->data1,rxdesc->data2);
	(gmacdev->BusyRxDesc)++; //One descriptor will be given to Hardware. So busy count incremented by one
	return rxnext;
}
#endif
#ifdef ENH_DESC_8W
/**
  * This function is defined two times. Once when the code is compiled for ENHANCED DESCRIPTOR SUPPORT and Once for Normal descriptor
  * Get back the descriptor from DMA after data has been received.
  * When the DMA indicates that the data is received (interrupt is generated), this function should be
  * called to get the descriptor and hence the data buffers received. With successful return from this
  * function caller gets the descriptor fields for processing. check the parameters to understand the 
  * fields returned.`
  * @param[in] pointer to synopGMACdevice.
  * @param[out] pointer to hold the status of DMA.
  * @param[out] Dma-able buffer1 pointer.
  * @param[out] pointer to hold length of buffer1 (Max is 2048).
  * @param[out] virtual pointer for buffer1.
  * @param[out] Dma-able buffer2 pointer.
  * @param[out] pointer to hold length of buffer2 (Max is 2048).
  * @param[out] virtual pointer for buffer2.
  * \return returns present rx descriptor index on success. Negative value if error.
  */
s32 synopGMAC_get_rx_qptr(synopGMACdevice * gmacdev, u32 * Status, dma_addr_t * Buffer1, u32 * Length1, ulong * Data1, dma_addr_t * Buffer2, u32 * Length2, ulong * Data2,
                          u32 * Ext_Status, u32 * Time_Stamp_High, u32 * Time_Stamp_Low)
{
	u32 rxnext       = gmacdev->RxBusy;	// index of descriptor the DMA just completed. May be useful when data 
						//is spread over multiple buffers/descriptors
	DmaDesc * rxdesc = gmacdev->RxBusyDesc;
	if(synopGMAC_is_desc_owned_by_dma(rxdesc))
		return -1;
	if(synopGMAC_is_desc_empty(rxdesc))
		return -1;
	

	if(Status != 0)
		*Status = rxdesc->status;// send the status of this descriptor

	if(Ext_Status != 0)
		*Ext_Status = rxdesc->extstatus;
        if(Time_Stamp_High != 0)
		*Time_Stamp_High = rxdesc->timestamphigh; 
        if(Time_Stamp_Low != 0)
		*Time_Stamp_Low = rxdesc->timestamplow; 

	if(Length1 != 0)
		*Length1 = (rxdesc->length & DescSize1Mask) >> DescSize1Shift;
	if(Buffer1 != 0)
		*Buffer1 = rxdesc->buffer1;
	if(Data1 != 0)
		*Data1 = rxdesc->data1;

	if(Length2 != 0)
		*Length2 = (rxdesc->length & DescSize2Mask) >> DescSize2Shift;
	if(Buffer2 != 0)
		*Buffer2 = rxdesc->buffer2;
	if(Data2 != 0)
		*Data2 = rxdesc->data2;

	gmacdev->RxBusy     = synopGMAC_is_last_rx_desc(gmacdev,rxdesc) ? 0 : rxnext + 1;

	if(synopGMAC_is_rx_desc_chained(rxdesc)){
	   	gmacdev->RxBusyDesc = (DmaDesc *)rxdesc->data2;
		synopGMAC_rx_desc_init_chain(rxdesc);
	}
	else{
		gmacdev->RxBusyDesc = synopGMAC_is_last_rx_desc(gmacdev,rxdesc) ? gmacdev->RxDesc : (rxdesc + 1);
		synopGMAC_rx_desc_init_ring(rxdesc, synopGMAC_is_last_rx_desc(gmacdev,rxdesc));
	}
	(gmacdev->BusyRxDesc)--; //busy tx descriptor is reduced by one as it will be handed over to Processor now
	TR("%02d %08x %08x %08x %08x %08x %08x %08x\n",rxnext,(u32)rxdesc,rxdesc->status,rxdesc->length,rxdesc->buffer1,rxdesc->buffer2,rxdesc->data1,rxdesc->data2);
	return(rxnext);

}
#else

/**
  * Get back the descriptor from DMA after data has been received.
  * When the DMA indicates that the data is received (interrupt is generated), this function should be
  * called to get the descriptor and hence the data buffers received. With successful return from this
  * function caller gets the descriptor fields for processing. check the parameters to understand the 
  * fields returned.`
  * @param[in] pointer to synopGMACdevice.
  * @param[out] pointer to hold the status of DMA.
  * @param[out] Dma-able buffer1 pointer.
  * @param[out] pointer to hold length of buffer1 (Max is 2048).
  * @param[out] virtual pointer for buffer1.
  * @param[out] Dma-able buffer2 pointer.
  * @param[out] pointer to hold length of buffer2 (Max is 2048).
  * @param[out] virtual pointer for buffer2.
  * \return returns present rx descriptor index on success. Negative value if error.
  */
s32 synopGMAC_get_rx_qptr(synopGMACdevice * gmacdev, u32 * Status, dma_addr_t * Buffer1, u32 * Length1, ulong * Data1, dma_addr_t * Buffer2, u32 * Length2, ulong * Data2)
{
	u32 rxnext       = gmacdev->RxBusy;	// index of descriptor the DMA just completed. May be useful when data 
						//is spread over multiple buffers/descriptors
	DmaDesc * rxdesc = gmacdev->RxBusyDesc;

	u32 len;

	if(synopGMAC_is_desc_owned_by_dma(rxdesc))
	{
		return -1;
	}
	
		
	if(synopGMAC_is_desc_empty(rxdesc))
	{
		return -1;
	}
	

	if(Status != 0)
		*Status = rxdesc->status;// send the status of this descriptor

	if(Length1 != 0)
		*Length1 = (rxdesc->length & DescSize1Mask) >> DescSize1Shift;
	if(Buffer1 != 0)
		*Buffer1 = rxdesc->buffer1;
	if(Data1 != 0)
		*Data1 = rxdesc->data1;
	if(Length2 != 0)
		*Length2 = (rxdesc->length & DescSize2Mask) >> DescSize2Shift;
	if(Buffer2 != 0)
		*Buffer2 = rxdesc->buffer2;
	if(Data2 != 0)
		*Data2 = rxdesc->data2;

	len =  synopGMAC_get_rx_desc_frame_length(*Status);
	gmacdev->RxBusy     = synopGMAC_is_last_rx_desc(gmacdev,rxdesc) ? 0 : rxnext + 1;

	if(synopGMAC_is_rx_desc_chained(rxdesc)){
	   	gmacdev->RxBusyDesc = (DmaDesc *)rxdesc->data2;
		synopGMAC_rx_desc_init_chain(rxdesc);
	}
	else{
		gmacdev->RxBusyDesc = synopGMAC_is_last_rx_desc(gmacdev,rxdesc) ? gmacdev->RxDesc : (rxdesc + 1);
//sw: raw data		
		synopGMAC_rx_desc_init_ring(rxdesc, synopGMAC_is_last_rx_desc(gmacdev,rxdesc));
	}

	(gmacdev->BusyRxDesc)--; //This returns one descriptor to processor. So busy count will be decremented by one
				 //sw: it's BusyRxDesc but not RxBusyDesc -_-
	return(rxnext);

}

#endif

/**
  * Clears all the pending interrupts.
  * If the Dma status register is read then all the interrupts gets cleared
  * @param[in] pointer to synopGMACdevice.
  * \return returns void.
  */
void synopGMAC_clear_interrupt(synopGMACdevice *gmacdev)
{
	u32 data;
	data = synopGMACReadReg(gmacdev->DmaBase, DmaStatus);
	TR("DMA status reg = 0x%x before cleared!\n",data);
	synopGMACWriteReg(gmacdev->DmaBase, DmaStatus ,data);
	data = synopGMACReadReg(gmacdev->DmaBase, DmaStatus);
	TR("DMA status reg = 0x%x after cleared!\n",data);
}

/**
  * Returns the all unmasked interrupt status after reading the DmaStatus register.
  * @param[in] pointer to synopGMACdevice.
  * \return 0 upon success. Error code upon failure.
  */
u32 synopGMAC_get_interrupt_type(synopGMACdevice *gmacdev)
{
	u32 data;
	u32 interrupts = 0;
	data = synopGMACReadReg(gmacdev->DmaBase, DmaStatus);
	
//	TR("DMA status reg is  %08x\n",data);
	//data = data & ~0x84;	//sw: some bits shoud not be cleaned
	synopGMACWriteReg(gmacdev->DmaBase, DmaStatus ,data); //manju: I think this is the appropriate location to clear the interrupts
	if(data & DmaIntErrorMask)	interrupts |= synopGMACDmaError;
	if(data & DmaIntRxNormMask)	interrupts |= synopGMACDmaRxNormal;
	if(data & DmaIntRxAbnMask)	interrupts |= synopGMACDmaRxAbnormal;
	if(data & DmaIntRxStoppedMask)	interrupts |= synopGMACDmaRxStopped;
	if(data & DmaIntTxNormMask)	interrupts |= synopGMACDmaTxNormal;
	if(data & DmaIntTxAbnMask)	interrupts |= synopGMACDmaTxAbnormal;
	if(data & DmaIntTxStoppedMask)	interrupts |= synopGMACDmaTxStopped;

	return interrupts;
}

/**
  * Returns the interrupt mask.
  * @param[in] pointer to synopGMACdevice.
  * \return 0 upon success. Error code upon failure.
  */
u32 synopGMAC_get_interrupt_mask(synopGMACdevice *gmacdev)
{
	return(synopGMACReadReg(gmacdev->DmaBase, DmaInterrupt));
}

/**
  * Enable all the interrupts.
  * Enables the DMA interrupt as specified by the bit mask.
  * @param[in] pointer to synopGMACdevice.
  * @param[in] bit mask of interrupts to be enabled.
  * \return returns void.
  */
void synopGMAC_enable_interrupt(synopGMACdevice *gmacdev, u32 interrupts)
{
	synopGMACWriteReg(gmacdev->DmaBase, DmaInterrupt, interrupts);
	return;
}


/**
  * Disable all the interrupts.
  * Disables all DMA interrupts.
  * @param[in] pointer to synopGMACdevice.
  * \return returns void.
  * \note This function disabled all the interrupts, if you want to disable a particular interrupt then
  *  use synopGMAC_disable_interrupt().
  */
void synopGMAC_disable_interrupt_all(synopGMACdevice *gmacdev)
{
	synopGMACWriteReg(gmacdev->DmaBase, DmaInterrupt, DmaIntDisable);
	return;
}

/**
  * Enable the DMA Reception.
  * @param[in] pointer to synopGMACdevice.
  * \return returns void.
  */
void synopGMAC_enable_dma_rx(synopGMACdevice * gmacdev)
{
//	synopGMACSetBits(gmacdev->DmaBase, DmaControl, DmaRxStart);
	u32 data;
	data = synopGMACReadReg(gmacdev->DmaBase, DmaControl);
  	data |= DmaRxStart; 
	synopGMACWriteReg(gmacdev->DmaBase, DmaControl ,data);
}

/**
  * Enable the DMA Transmission.
  * @param[in] pointer to synopGMACdevice.
  * \return returns void.
  */
void synopGMAC_enable_dma_tx(synopGMACdevice * gmacdev)
{
//	synopGMACSetBits(gmacdev->DmaBase, DmaControl, DmaTxStart);
	u32 data;
	data = synopGMACReadReg(gmacdev->DmaBase, DmaControl);
  	data |= DmaTxStart; 
	synopGMACWriteReg(gmacdev->DmaBase, DmaControl ,data);

}

/**
  * Resumes the DMA Transmission.
  * the DmaTxPollDemand is written. (the data writeen could be anything).
  * This forces the DMA to resume transmission.
  * @param[in] pointer to synopGMACdevice.
  * \return returns void.
  */
void synopGMAC_resume_dma_tx(synopGMACdevice * gmacdev)
{
	synopGMACWriteReg(gmacdev->DmaBase, DmaTxPollDemand, 1);

}
/**
  * Resumes the DMA Reception.
  * the DmaRxPollDemand is written. (the data writeen could be anything).
  * This forces the DMA to resume reception.
  * @param[in] pointer to synopGMACdevice.
  * \return returns void.
  */
void synopGMAC_resume_dma_rx(synopGMACdevice * gmacdev)
{
	synopGMACWriteReg(gmacdev->DmaBase, DmaRxPollDemand, 0);

}
/**
  * Take ownership of this Descriptor.
  * The function is same for both the ring mode and the chain mode DMA structures.
  * @param[in] pointer to synopGMACdevice.
  * \return returns void.
  */
void synopGMAC_take_desc_ownership(DmaDesc * desc)
{
	if(desc){
		desc->status &= ~DescOwnByDma;  //Clear the DMA own bit
//		desc->status |= DescError;	// Set the error to indicate this descriptor is bad
	}
}

/**
  * Take ownership of all the rx Descriptors.
  * This function is called when there is fatal error in DMA transmission.
  * When called it takes the ownership of all the rx descriptor in rx descriptor pool/queue from DMA.
  * The function is same for both the ring mode and the chain mode DMA structures.
  * @param[in] pointer to synopGMACdevice.
  * \return returns void.
  * \note Make sure to disable the transmission before calling this function, otherwise may result in racing situation.
  */
void synopGMAC_give_desc_ownership_rx(synopGMACdevice * gmacdev)
{
	s32 i;
	DmaDesc *desc;
	desc = gmacdev->RxDesc;
	for(i = 0; i < gmacdev->RxDescCount; i++){
		if(synopGMAC_is_rx_desc_chained(desc)){	//This descriptor is in chain mode
	
			synopGMAC_set_owner_dma(desc);
			desc = (DmaDesc *)desc->data2;
		}
		else{
			synopGMAC_set_owner_dma(desc + i);
		}
	}
}

/**
  * Take ownership of all the rx Descriptors.
  * This function is called when there is fatal error in DMA transmission.
  * When called it takes the ownership of all the tx descriptor in tx descriptor pool/queue from DMA.
  * The function is same for both the ring mode and the chain mode DMA structures.
  * @param[in] pointer to synopGMACdevice.
  * \return returns void.
  * \note Make sure to disable the transmission before calling this function, otherwise may result in racing situation.
  */
void synopGMAC_take_desc_ownership_tx(synopGMACdevice * gmacdev)
{
	s32 i;
	DmaDesc *desc;
	desc = gmacdev->TxDesc;
	for(i = 0; i < gmacdev->TxDescCount; i++){
		if(synopGMAC_is_tx_desc_chained(desc)){	//This descriptor is in chain mode
			synopGMAC_take_desc_ownership(desc);
			desc = (DmaDesc *)desc->data2;
		}
		else{
			synopGMAC_take_desc_ownership(desc + i);
		}
	}
	
}

/**
  * Disable the DMA for Transmission.
  * @param[in] pointer to synopGMACdevice.
  * \return returns void.
  */

void synopGMAC_disable_dma_tx(synopGMACdevice * gmacdev)
{	
//	synopGMACClearBits(gmacdev->DmaBase, DmaControl, DmaTxStart);
	u32 data;
	data = synopGMACReadReg(gmacdev->DmaBase, DmaControl);
  	data &= (~DmaTxStart); 
	synopGMACWriteReg(gmacdev->DmaBase, DmaControl ,data);
}
/**
  * Disable the DMA for Reception.
  * @param[in] pointer to synopGMACdevice.
  * \return returns void.
  */
void synopGMAC_disable_dma_rx(synopGMACdevice * gmacdev)
{	
//	synopGMACClearBits(gmacdev->DmaBase, DmaControl, DmaRxStart);
	u32 data;
	data = synopGMACReadReg(gmacdev->DmaBase, DmaControl);
  	data &= (~DmaRxStart); 
	synopGMACWriteReg(gmacdev->DmaBase, DmaControl ,data);
}

/**
  * Disables the assertion of PMT interrupt.
  * This disables the assertion of PMT interrupt due to Magic Pkt or Wakeup frame
  * reception.
  * @param[in] pointer to synopGMACdevice.
  * \return returns void.
  */
void synopGMAC_pmt_int_disable(synopGMACdevice *gmacdev)
{
	synopGMACSetBits(gmacdev->MacBase,GmacInterruptMask,GmacPmtIntMask); 
        return;
}
/**
  * Checks whether the packet received is a magic packet?.
  * @param[in] pointer to synopGMACdevice.
  * \return returns True if magic packet received else returns false.
  */
bool synopGMAC_is_magic_packet_received(synopGMACdevice *gmacdev)
{
	u32 data;
	data = 	synopGMACReadReg(gmacdev->MacBase,GmacPmtCtrlStatus);	
	return((data & GmacPmtMagicPktReceived) == GmacPmtMagicPktReceived);
}
/**
  * Checks whether the packet received is a wakeup frame?.
  * @param[in] pointer to synopGMACdevice.
  * \return returns true if wakeup frame received else returns false.
  */
bool synopGMAC_is_wakeup_frame_received(synopGMACdevice *gmacdev)
{
	u32 data;
	data = 	synopGMACReadReg(gmacdev->MacBase,GmacPmtCtrlStatus);	
	return((data & GmacPmtWakeupFrameReceived) == GmacPmtWakeupFrameReceived);
}

/**
  * Read the MMC Rx interrupt status.
  * @param[in] pointer to synopGMACdevice.
  * \return returns the Rx interrupt status.
  */
u32 synopGMAC_read_mmc_rx_int_status(synopGMACdevice *gmacdev)
{
	return(	synopGMACReadReg(gmacdev->MacBase,GmacMmcIntrRx));
}
/**
  * Read the MMC Tx interrupt status.
  * @param[in] pointer to synopGMACdevice.
  * \return returns the Tx interrupt status.
  */
u32 synopGMAC_read_mmc_tx_int_status(synopGMACdevice *gmacdev)
{
	return(	synopGMACReadReg(gmacdev->MacBase,GmacMmcIntrTx));
}
/**
  * Disable the MMC Tx interrupt.
  * The MMC tx interrupts are masked out as per the mask specified.
  * @param[in] pointer to synopGMACdevice.
  * @param[in] tx interrupt bit mask for which interrupts needs to be disabled.
  * \return returns void.
  */
void synopGMAC_disable_mmc_tx_interrupt(synopGMACdevice *gmacdev, u32 mask)
{
	synopGMACSetBits(gmacdev->MacBase,GmacMmcIntrMaskTx,mask);
	return;
}
/**
  * Disable the MMC Rx interrupt.
  * The MMC rx interrupts are masked out as per the mask specified.
  * @param[in] pointer to synopGMACdevice.
  * @param[in] rx interrupt bit mask for which interrupts needs to be disabled.
  * \return returns void.
  */
void synopGMAC_disable_mmc_rx_interrupt(synopGMACdevice *gmacdev, u32 mask)
{
	synopGMACSetBits(gmacdev->MacBase,GmacMmcIntrMaskRx,mask);
	return;
}
/**
  * Disable the MMC ipc rx checksum offload interrupt.
  * The MMC ipc rx checksum offload interrupts are masked out as per the mask specified.
  * @param[in] pointer to synopGMACdevice.
  * @param[in] rx interrupt bit mask for which interrupts needs to be disabled.
  * \return returns void.
  */
void synopGMAC_disable_mmc_ipc_rx_interrupt(synopGMACdevice *gmacdev, u32 mask)
{
	synopGMACSetBits(gmacdev->MacBase,GmacMmcRxIpcIntrMask,mask);
	return;
}

/** 
  * When the Enhanced Descriptor is enabled then the bit 0 of RDES0 indicates whether the
  * Extended Status is available (RDES4). Time Stamp feature and the Checksum Offload Engine2
  * makes use of this extended status to provide the status of the received packet.
  * @param[in] pointer to synopGMACdevice
  * \return returns TRUE or FALSE
  */
#ifdef ENH_DESC_8W

/**
  * This function indicates whether extended status is available in the RDES0.
  * Any function which accesses the fields of extended status register must ensure a check on this has been made
  * This is valid only for Enhanced Descriptor.
  * @param[in] pointer to synopGMACdevice.
  * @param[in] u32 status field of the corresponding descriptor.
  * \return returns TRUE or FALSE.
  */
bool synopGMAC_is_ext_status(synopGMACdevice *gmacdev,u32 status) 		      // extended status present indicates that the RDES4 need to be probed
{
	return((status & DescRxEXTsts ) != 0 ); // if extstatus set then it returns 1
}
/**
  * This function returns true if the IP header checksum bit is set in the extended status.
  * Valid only when enhaced status available is set in RDES0 bit 0.
  * This is valid only for Enhanced Descriptor.
  * @param[in] pointer to synopGMACdevice.
  * @param[in] u32 status field of the corresponding descriptor.
  * \return returns TRUE or FALSE.
  */
bool synopGMAC_ES_is_IP_header_error(synopGMACdevice *gmacdev,u32 ext_status)          // IP header (IPV4) checksum error
{
	return((ext_status & DescRxIpHeaderError) != 0 ); // if IPV4 header error return 1
}
/**
  * This function returns true if the Checksum is bypassed in the hardware.
  * Valid only when enhaced status available is set in RDES0 bit 0.
  * This is valid only for Enhanced Descriptor.
  * @param[in] pointer to synopGMACdevice.
  * @param[in] u32 status field of the corresponding descriptor.
  * \return returns TRUE or FALSE.
  */
bool synopGMAC_ES_is_rx_checksum_bypassed(synopGMACdevice *gmacdev,u32 ext_status)     // Hardware engine bypassed the checksum computation/checking
{
	return((ext_status & DescRxChkSumBypass ) != 0 ); // if checksum offloading bypassed return 1
}
/**
  * This function returns true if payload checksum error is set in the extended status.
  * Valid only when enhaced status available is set in RDES0 bit 0.
  * This is valid only for Enhanced Descriptor.
  * @param[in] pointer to synopGMACdevice.
  * @param[in] u32 status field of the corresponding descriptor.
  * \return returns TRUE or FALSE.
  */
bool synopGMAC_ES_is_IP_payload_error(synopGMACdevice *gmacdev,u32 ext_status)         // IP payload checksum is in error (UDP/TCP/ICMP checksum error)
{
	return((ext_status & DescRxIpPayloadError) != 0 ); // if IP payload error return 1
}
#endif



/**
  * Decodes the Rx Descriptor status to various checksum error conditions.
  * @param[in] pointer to synopGMACdevice.
  * @param[in] u32 status field of the corresponding descriptor.
  * \return returns decoded enum (u32) indicating the status.
  */
u32 synopGMAC_is_rx_checksum_error(synopGMACdevice *gmacdev, u32 status)
{
	if     (((status & DescRxChkBit5) == 0) && ((status & DescRxChkBit7) == 0) && ((status & DescRxChkBit0) == 0))
	return RxLenLT600;
	else if(((status & DescRxChkBit5) == 0) && ((status & DescRxChkBit7) == 0) && ((status & DescRxChkBit0) != 0))
	return RxIpHdrPayLoadChkBypass;
	else if(((status & DescRxChkBit5) == 0) && ((status & DescRxChkBit7) != 0) && ((status & DescRxChkBit0) != 0))
	return RxChkBypass;
	else if(((status & DescRxChkBit5) != 0) && ((status & DescRxChkBit7) == 0) && ((status & DescRxChkBit0) == 0))
	return RxNoChkError;
	else if(((status & DescRxChkBit5) != 0) && ((status & DescRxChkBit7) == 0) && ((status & DescRxChkBit0) != 0))
	return RxPayLoadChkError;
	else if(((status & DescRxChkBit5) != 0) && ((status & DescRxChkBit7) != 0) && ((status & DescRxChkBit0) == 0))
	return RxIpHdrChkError;
	else if(((status & DescRxChkBit5) != 0) && ((status & DescRxChkBit7) != 0) && ((status & DescRxChkBit0) != 0))
	return RxIpHdrPayLoadChkError;
	else
	return RxIpHdrPayLoadRes;
}
/**
  * Checks if any Ipv4 header checksum error in the frame just transmitted.
  * This serves as indication that error occureed in the IPv4 header checksum insertion.
  * The sent out frame doesnot carry any ipv4 header checksum inserted by the hardware.
  * @param[in] pointer to synopGMACdevice.
  * @param[in] u32 status field of the corresponding descriptor.
  * \return returns true if error in ipv4 header checksum, else returns false.
  */
bool synopGMAC_is_tx_ipv4header_checksum_error(synopGMACdevice *gmacdev, u32 status)
{
	return((status & DescTxIpv4ChkError) == DescTxIpv4ChkError);
}


/**
  * Checks if any payload checksum error in the frame just transmitted.
  * This serves as indication that error occureed in the payload checksum insertion.
  * The sent out frame doesnot carry any payload checksum inserted by the hardware.
  * @param[in] pointer to synopGMACdevice.
  * @param[in] u32 status field of the corresponding descriptor.
  * \return returns true if error in ipv4 header checksum, else returns false.
  */
bool synopGMAC_is_tx_payload_checksum_error(synopGMACdevice *gmacdev, u32 status)
{
	return((status & DescTxPayChkError) == DescTxPayChkError);
}
/**
  * The check summ offload engine is bypassed in the tx path.
  * Checksum is not computed in the Hardware.
  * @param[in] pointer to synopGMACdevice.
  * @param[in] Pointer to tx descriptor for which  ointer to synopGMACdevice.
  * \return returns void.
  */
void synopGMAC_tx_checksum_offload_bypass(synopGMACdevice *gmacdev, DmaDesc *desc)
{
	#ifdef ENH_DESC
	desc->status = (desc->length & (~DescTxCisMask));//ENH_DESC
	#else
	desc->length = (desc->length & (~DescTxCisMask));
	#endif

}
/**
  * The check summ offload engine is enabled to do only IPV4 header checksum.
  * IPV4 header Checksum is computed in the Hardware.
  * @param[in] pointer to synopGMACdevice.
  * @param[in] Pointer to tx descriptor for which  ointer to synopGMACdevice.
  * \return returns void.
  */
void synopGMAC_tx_checksum_offload_ipv4hdr(synopGMACdevice *gmacdev, DmaDesc *desc)
{
	#ifdef ENH_DESC
	desc->status = ((desc->status & (~DescTxCisMask)) | DescTxCisIpv4HdrCs);//ENH_DESC
	#else
	desc->length = ((desc->length & (~DescTxCisMask)) | DescTxCisIpv4HdrCs);
	#endif

}

/**
  * The check summ offload engine is enabled to do TCPIP checsum assuming Pseudo header is available.
  * Hardware computes the tcp ip checksum assuming pseudo header checksum is computed in software.
  * Ipv4 header checksum is also inserted.
  * @param[in] pointer to synopGMACdevice.
  * @param[in] Pointer to tx descriptor for which  ointer to synopGMACdevice.
  * \return returns void.
  */
void synopGMAC_tx_checksum_offload_tcponly(synopGMACdevice *gmacdev, DmaDesc *desc)
{
	#ifdef ENH_DESC
	desc->status = ((desc->status & (~DescTxCisMask)) | DescTxCisTcpOnlyCs);//ENH_DESC
	#else
	desc->length = ((desc->length & (~DescTxCisMask)) | DescTxCisTcpOnlyCs);
	#endif

}
/**
  * The check summ offload engine is enabled to do complete checksum computation.
  * Hardware computes the tcp ip checksum including the pseudo header checksum.
  * Here the tcp payload checksum field should be set to 0000.
  * Ipv4 header checksum is also inserted.
  * @param[in] pointer to synopGMACdevice.
  * @param[in] Pointer to tx descriptor for which  ointer to synopGMACdevice.
  * \return returns void.
  */
void synopGMAC_tx_checksum_offload_tcp_pseudo(synopGMACdevice *gmacdev, DmaDesc *desc)
{
	#ifdef ENH_DESC
	desc->status = ((desc->length & (~DescTxCisMask)) | DescTxCisTcpPseudoCs);
	#else
	desc->length = ((desc->length & (~DescTxCisMask)) | DescTxCisTcpPseudoCs);
	#endif

}
