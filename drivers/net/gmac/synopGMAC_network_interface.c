#include <common.h>
#include <dm.h>
#include <linux/io.h>
#include "synopGMAC_network_interface.h"
#include <pci.h>
#include <hexdump.h>


//sw:	ioctl in linux 		to be fixed
#define SIOCDEVPRIVATE	0x89f0
#define IOCTL_READ_REGISTER  SIOCDEVPRIVATE+1
#define IOCTL_WRITE_REGISTER SIOCDEVPRIVATE+2
#define IOCTL_READ_IPSTRUCT  SIOCDEVPRIVATE+3
#define IOCTL_READ_RXDESC    SIOCDEVPRIVATE+4
#define IOCTL_READ_TXDESC    SIOCDEVPRIVATE+5
#define IOCTL_POWER_DOWN     SIOCDEVPRIVATE+6


//static struct timer_list synopGMAC_cable_unplug_timer;
int set_lpmode(synopGMACdevice * gmacdev);
int set_phyled(synopGMACdevice * gmacdev);


static int rtl88e1111_config_init(synopGMACdevice *gmacdev)
{
	int retval, err;
	u16 data;

	synopGMAC_read_phy_reg(gmacdev->MacBase,gmacdev->PhyBase,0x14,&data);
	data = data | 0x82;
	err = synopGMAC_write_phy_reg(gmacdev->MacBase,gmacdev->PhyBase,0x14,data);
	synopGMAC_read_phy_reg(gmacdev->MacBase,gmacdev->PhyBase,0x00,&data);
	data = data | 0x8000;
	err = synopGMAC_write_phy_reg(gmacdev->MacBase,gmacdev->PhyBase,0x00,data);

       /*init link and act indication leds*/
           synopGMAC_read_phy_reg((u32 *) gmacdev->MacBase, gmacdev->PhyBase,
                                  PHY_LED_CONTROL, &data);
       data = (data | 0x51) &0xf4ff;
           synopGMAC_write_phy_reg((u32 *) gmacdev->MacBase, gmacdev->PhyBase,
                                   PHY_LED_CONTROL, data);
	if (err < 0)
		return err;
	return 0;
}
#include "mii.h"
#include "mii.c"


s32 synopGMAC_check_phy_init (synopGMACdevice * gmacdev) 
{	
	u32 data;
	data = synopGMACReadReg(gmacdev->MacBase,0xd8);
	gmacdev->LinkState0 = data;
	if((data & 8) == 0){
		if(gmacdev->LinkState)
			printf("No Link: %08x\n",data);
		gmacdev->LinkState = 0;
		gmacdev->DuplexMode = 0;
		gmacdev->Speed = 0;
		gmacdev->LoopBackMode = 0; 
				
	}
	else{
		if(gmacdev->LinkState!=data)
		{
			gmacdev->DuplexMode = (data & 1)  ? FULLDUPLEX: HALFDUPLEX ;
			printf("Link is up in %s mode\n",(gmacdev->DuplexMode == FULLDUPLEX) ? "FULL DUPLEX": "HALF DUPLEX");

			/*if not set to Master configuration in case of Half duplex mode set it manually as Master*/

			switch((data>>1)&3)
			{
				case 2:
					gmacdev->Speed      =   SPEED1000;
					printf("Link is with 1000M Speed \n");
					break;
				case 1:
					gmacdev->Speed      =   SPEED100;
					printf("Link is with 100M Speed \n");
					break;
				default:
					gmacdev->Speed      =   SPEED10;
					printf("Link is with 10M Speed \n");
					break;
			}
		}
	}
	return gmacdev->Speed|(gmacdev->DuplexMode<<4);
}

/**
  * This sets up the transmit Descriptor queue in ring or chain mode.
  * This function is tightly coupled to the platform and operating system
  * Device is interested only after the descriptors are setup. Therefore this function
  * is not included in the device driver API. This function should be treated as an
  * example code to design the descriptor structures for ring mode or chain mode.
  * This function depends on the pcidev structure for allocation consistent dma-able memory in case of linux.
  * This limitation is due to the fact that linux uses pci structure to allocate a dmable memory
  *	- Allocates the memory for the descriptors.
  *	- Initialize the Busy and Next descriptors indices to 0(Indicating first descriptor).
  *	- Initialize the Busy and Next descriptors to first descriptor address.
  * 	- Initialize the last descriptor with the endof ring in case of ring mode.
  *	- Initialize the descriptors in chain mode.
  * @param[in] pointer to synopGMACdevice.
  * @param[in] pointer to pci_device structure.
  * @param[in] number of descriptor expected in tx descriptor queue.
  * @param[in] whether descriptors to be created in RING mode or CHAIN mode.
  * \return 0 upon success. Error code upon failure.
  * \note This function fails if allocation fails for required number of descriptors in Ring mode, but in chain mode
  *  function returns -ESYNOPGMACNOMEM in the process of descriptor chain creation. once returned from this function
  *  user should for gmacdev->TxDescCount to see how many descriptors are there in the chain. Should continue further
  *  only if the number of descriptors in the chain meets the requirements  
  */

s32 synopGMAC_setup_tx_desc_queue(synopGMACdevice * gmacdev,u32 no_of_desc, u32 desc_mode)
{
	s32 i;
	DmaDesc * bf1;

	DmaDesc *first_desc = NULL;
	DmaDesc *second_desc = NULL;
	dma_addr_t dma_addr;
	gmacdev->TxDescCount = 0;

	TR("Total size of memory required for Tx Descriptors in Ring Mode = 0x%08x\n",((sizeof(DmaDesc) * no_of_desc)));
	if (desc_mode == RINGMODE) {
		first_desc = plat_alloc_consistent_dmaable_memory (gmacdev, sizeof(DmaDesc) * no_of_desc,&dma_addr);
		if(first_desc == NULL){
			printf("Error in Tx Descriptors memory allocation\n");
			return -ESYNOPGMACNOMEM;
		}

		gmacdev->TxDescCount = no_of_desc;
		gmacdev->TxDesc      = first_desc;
		gmacdev->TxDescDma  = dma_addr;

		TR("\n===Tx first_desc: %x\n",gmacdev->TxDesc);
		//gmacdev->FirstTxDesc = first_desc;

		for(i =0; i < gmacdev -> TxDescCount; i++){
			synopGMAC_tx_desc_init_ring(gmacdev->TxDesc + i, i == gmacdev->TxDescCount-1);
		}
	} else {
		first_desc = plat_alloc_consistent_dmaable_memory(gmacdev,
					sizeof(DmaDesc), &dma_addr);
		if (first_desc == NULL) {
			TR("Error in Tx Descriptor Memory allocation in Ring mode\n");
			return -ESYNOPGMACNOMEM;
		}
		gmacdev->TxDesc = first_desc;
		gmacdev->TxDescDma = dma_addr;

		TR("Tx===================================================================Tx\n");
		first_desc->buffer2 = gmacdev->TxDescDma;
		first_desc->data2 = (ulong) gmacdev->TxDesc;

		gmacdev->TxDescCount = 1;

		for (i = 0; i < (no_of_desc - 1); i++) {
			second_desc = plat_alloc_consistent_dmaable_memory(gmacdev,
						sizeof(DmaDesc), &dma_addr);
			if (second_desc == NULL) {
				TR("Error in Tx Descriptor Memory allocation in Chain mode\n");
				return -ESYNOPGMACNOMEM;
			}
			first_desc->buffer2 = dma_addr;
			first_desc->data2 = (ulong) second_desc;

			second_desc->buffer2 = gmacdev->TxDescDma;
			second_desc->data2 = (ulong) gmacdev->TxDesc;

			synopGMAC_tx_desc_init_chain(first_desc);
			TR("%02d %p %08x %08x %08x %08x %lx %lx \n",
			   gmacdev->TxDescCount, first_desc, first_desc->status,
			   first_desc->length, first_desc->buffer1,
			   first_desc->buffer2, first_desc->data1,
			   first_desc->data2);
			gmacdev->TxDescCount += 1;
			first_desc = second_desc;
		}

		synopGMAC_tx_desc_init_chain(first_desc);
		TR("%02d %p %08x %08x %08x %08x %lx %lx \n",
		   gmacdev->TxDescCount, first_desc, first_desc->status,
		   first_desc->length, first_desc->buffer1, first_desc->buffer2,
		   first_desc->data1, first_desc->data2);
		TR("Tx===================================================================Tx\n");
	}

	gmacdev->TxNext = 0;
	gmacdev->TxBusy = 0;
	gmacdev->TxNextDesc = gmacdev->TxDesc;
	gmacdev->TxBusyDesc = gmacdev->TxDesc;
	gmacdev->BusyTxDesc  = 0; 

	return -ESYNOPGMACNOERR;
}


/**
  * This sets up the receive Descriptor queue in ring or chain mode.
  * This function is tightly coupled to the platform and operating system
  * Device is interested only after the descriptors are setup. Therefore this function
  * is not included in the device driver API. This function should be treated as an
  * example code to design the descriptor structures in ring mode or chain mode.
  * This function depends on the pcidev structure for allocation of consistent dma-able memory in case of linux.
  * This limitation is due to the fact that linux uses pci structure to allocate a dmable memory
  *	- Allocates the memory for the descriptors.
  *	- Initialize the Busy and Next descriptors indices to 0(Indicating first descriptor).
  *	- Initialize the Busy and Next descriptors to first descriptor address.
  * 	- Initialize the last descriptor with the endof ring in case of ring mode.
  *	- Initialize the descriptors in chain mode.
  * @param[in] pointer to synopGMACdevice.
  * @param[in] pointer to pci_device structure.
  * @param[in] number of descriptor expected in rx descriptor queue.
  * @param[in] whether descriptors to be created in RING mode or CHAIN mode.
  * \return 0 upon success. Error code upon failure.
  * \note This function fails if allocation fails for required number of descriptors in Ring mode, but in chain mode
  *  function returns -ESYNOPGMACNOMEM in the process of descriptor chain creation. once returned from this function
  *  user should for gmacdev->RxDescCount to see how many descriptors are there in the chain. Should continue further
  *  only if the number of descriptors in the chain meets the requirements  
  */
s32 synopGMAC_setup_rx_desc_queue(synopGMACdevice * gmacdev,u32 no_of_desc, u32 desc_mode)
{
	s32 i;
	DmaDesc * bf1;
	DmaDesc *first_desc = NULL;
	DmaDesc *second_desc = NULL;
	dma_addr_t dma_addr;
	gmacdev->RxDescCount = 0;

	if (desc_mode == RINGMODE) {
	TR("total size of memory required for Rx Descriptors in Ring Mode = 0x%08x\n",((sizeof(DmaDesc) * no_of_desc)));
	first_desc = plat_alloc_consistent_dmaable_memory (gmacdev, sizeof(DmaDesc) * no_of_desc, &dma_addr);
	if(first_desc == NULL){
		printf("Error in Rx Descriptor Memory allocation in Ring mode\n");
		return -ESYNOPGMACNOMEM;
	}

	gmacdev->RxDescCount = no_of_desc;
	gmacdev->RxDesc      = (DmaDesc *)first_desc;
	gmacdev->RxDescDma   = dma_addr;

	for(i =0; i < gmacdev -> RxDescCount; i++){
		synopGMAC_rx_desc_init_ring(gmacdev->RxDesc + i, i == gmacdev->RxDescCount-1);
	}
	} else {
		first_desc = plat_alloc_consistent_dmaable_memory(gmacdev,
				sizeof(DmaDesc), &dma_addr);
		if (first_desc == NULL) {
			TR("Error in Rx Descriptor Memory allocation in Ring mode\n");
			return -ESYNOPGMACNOMEM;
		}
		gmacdev->RxDesc = first_desc;
		gmacdev->RxDescDma = dma_addr;

		TR("Rx===================================================================Rx\n");
		first_desc->buffer2 = gmacdev->RxDescDma;
		first_desc->data2 = (ulong) gmacdev->RxDesc;

		gmacdev->RxDescCount = 1;

		for (i = 0; i < (no_of_desc - 1); i++) {
			second_desc = plat_alloc_consistent_dmaable_memory(gmacdev,
					    sizeof(DmaDesc), &dma_addr);
			if (second_desc == NULL) {
				TR("Error in Rx Descriptor Memory allocation in Chain mode\n");
				return -ESYNOPGMACNOMEM;
			}
			first_desc->buffer2 = dma_addr;
			first_desc->data2 = (ulong) second_desc;

			second_desc->buffer2 = gmacdev->RxDescDma;
			second_desc->data2 = (ulong) gmacdev->RxDesc;

			synopGMAC_rx_desc_init_chain(first_desc);
			TR("%02d  %p %08x %08x %08x %08x %lx %lx \n",
			   gmacdev->RxDescCount, first_desc, first_desc->status,
			   first_desc->length, first_desc->buffer1,
			   first_desc->buffer2, first_desc->data1,
			   first_desc->data2);
			gmacdev->RxDescCount += 1;
			first_desc = second_desc;
		}
		synopGMAC_rx_desc_init_chain(first_desc);
		TR("%02d  %p %08x %08x %08x %08x %lx %lx \n",
		   gmacdev->RxDescCount, first_desc, first_desc->status,
		   first_desc->length, first_desc->buffer1, first_desc->buffer2,
		   first_desc->data1, first_desc->data2);
		TR("Rx===================================================================Rx\n");

	}

	gmacdev->RxNext = 0;
	gmacdev->RxBusy = 0;
	gmacdev->RxNextDesc = gmacdev->RxDesc;
	gmacdev->RxBusyDesc = gmacdev->RxDesc;

	gmacdev->BusyRxDesc   = 0; 

	return -ESYNOPGMACNOERR;
}


/**
 * Function to handle housekeeping after a packet is transmitted over the wire.
 * After the transmission of a packet DMA generates corresponding interrupt 
 * (if it is enabled). It takes care of returning the sk_buff to the linux
 * kernel, updating the networking statistics and tracking the descriptors.
 * @param[in] pointer to net_device structure. 
 * \return void.
 * \note This function runs in interrupt context
 */
void synop_handle_transmit_over(struct synopGMACNetworkAdapter * tp)
{
	struct	synopGMACNetworkAdapter *adapter;
	synopGMACdevice * gmacdev;
	s32 desc_index;
	ulong data1, data2;
	u32 status;
	u32 length1, length2;
	dma_addr_t  dma_addr1, dma_addr2;
#ifdef ENH_DESC_8W
	u32 ext_status;
	u16 time_stamp_higher;
	u32 time_stamp_high;
	u32 time_stamp_low;
#endif

	adapter = tp;
	if(adapter == NULL){
		return;
	}
	
	gmacdev = adapter->synopGMACdev;
	if(gmacdev == NULL){
		return;
	}
	
	/*Handle the transmit Descriptors*/
	do {
#ifdef ENH_DESC_8W
	desc_index = synopGMAC_get_tx_qptr(gmacdev, &status, &dma_addr1, &length1, &data1, &dma_addr2, &length2, &data2,&ext_status,&time_stamp_high,&time_stamp_low);
        synopGMAC_TS_read_timestamp_higher_val(gmacdev, &time_stamp_higher);
#else
	desc_index = synopGMAC_get_tx_qptr(gmacdev, &status, &dma_addr1, &length1, &data1, &dma_addr2, &length2, &data2);
#endif
		if(desc_index >= 0 && data1 != 0){
			#ifdef	IPC_OFFLOAD
			if(synopGMAC_is_tx_ipv4header_checksum_error(gmacdev, status)){
			}
			if(synopGMAC_is_tx_payload_checksum_error(gmacdev, status)){
			printf("Harware Failed to Insert Payload Checksum\n");
			}
			#endif
		
			plat_free_memory((void *)(data1));	//sw:	data1 = buffer1
			
			if(synopGMAC_is_desc_valid(status)){
				adapter->synopGMACNetStats.tx_bytes += length1;
				adapter->synopGMACNetStats.tx_packets++;
			}
			else {	
				adapter->synopGMACNetStats.tx_errors++;
				adapter->synopGMACNetStats.tx_aborted_errors += synopGMAC_is_tx_aborted(status);
				adapter->synopGMACNetStats.tx_carrier_errors += synopGMAC_is_tx_carrier_error(status);
			}
		}	adapter->synopGMACNetStats.collisions += synopGMAC_get_tx_collision_count(status);
	} while(desc_index >= 0);
}




/**
 * Function to Receive a packet from the interface.
 * After Receiving a packet, DMA transfers the received packet to the system memory
 * and generates corresponding interrupt (if it is enabled). This function prepares
 * the sk_buff for received packet after removing the ethernet CRC, and hands it over
 * to linux networking stack.
 * 	- Updataes the networking interface statistics
 *	- Keeps track of the rx descriptors
 * @param[in] pointer to net_device structure. 
 * \return void.
 * \note This function runs in interrupt context.
 */

int synop_handle_received_data(struct synopGMACNetworkAdapter* adapter, char **packetp)
{
	synopGMACdevice * gmacdev;
	s32 desc_index;
	int i;
	char * ptr;
	u32 bf1;
	ulong data1;
	ulong data2;
	u32 len;
	u32 status;
	dma_addr_t  dma_addr1;
	dma_addr_t  dma_addr2;

	gmacdev = adapter->synopGMACdev;

	/*Handle the Receive Descriptors*/
	do{
		desc_index = synopGMAC_get_rx_qptr(gmacdev, &status,&dma_addr1,NULL, &data1,&dma_addr2,NULL,&data2);
		TR("==handle rx desc%d\n",desc_index);
		if(desc_index >= 0 && data1 != 0){

			if(synopGMAC_is_rx_desc_valid(status)){

				len =  synopGMAC_get_rx_desc_frame_length(status) - 4; //Not interested in Ethernet CRC bytes
				adapter->synopGMACNetStats.rx_packets++;
				adapter->synopGMACNetStats.rx_bytes += len;
#define USE_OLD_MODE
#ifdef USE_OLD_MODE
				net_process_received_packet(data1, len);
#else
				*packetp =  data1;
				return len;
#endif
			}
			else{
				adapter->synopGMACNetStats.rx_errors++;
				adapter->synopGMACNetStats.collisions       += synopGMAC_is_rx_frame_collision(status);
				adapter->synopGMACNetStats.rx_crc_errors    += synopGMAC_is_rx_crc(status);
				adapter->synopGMACNetStats.rx_frame_errors  += synopGMAC_is_frame_dribbling_errors(status);
				adapter->synopGMACNetStats.rx_length_errors += synopGMAC_is_rx_frame_length_errors(status);
			}

#ifdef USE_OLD_MODE
			desc_index = synopGMAC_set_rx_qptr(gmacdev,dma_addr1, RX_BUF_SIZE, data1,0,0,0);

			if(desc_index < 0){
				plat_free_memory((void *)data1);
			}
#endif
		}

	}while(desc_index >= 0);

	return -EAGAIN;
}

int gmac_free_pkt(struct synopGMACNetworkAdapter* adapter, uchar *packet, int len)
{
	synopGMACdevice * gmacdev;
	dma_addr_t dma_addr1;
        gmacdev = adapter->synopGMACdev;
	dma_addr1 = plat_dma_map_single(gmacdev, packet, RX_BUF_SIZE);
	int  desc_index = synopGMAC_set_rx_qptr(gmacdev,dma_addr1, RX_BUF_SIZE, packet,0,0,0);

	if(desc_index < 0){
		plat_free_memory((void *)packet);
	}
	return 0;
}



/**
 * Interrupt service routing.
 * This is the function registered as ISR for device interrupts.
 * @param[in] interrupt number. 
 * @param[in] void pointer to device unique structure (Required for shared interrupts in Linux).
 * @param[in] pointer to pt_regs (not used).
 * \return Returns IRQ_NONE if not device interrupts IRQ_HANDLED for device interrupts.
 * \note This function runs in interrupt context
 *
 */

//irqreturn_t synopGMAC_intr_handler(s32 intr_num, void * dev_id, struct pt_regs *regs)
int synopGMAC_intr_handler(struct synopGMACNetworkAdapter * adapter, char **ppacket)
{
        /*Kernels passes the netdev structure in the dev_id. So grab it*/
        synopGMACdevice * gmacdev;
        u32 interrupt;
	static u32 leftrx;
        u32 dma_status_reg;
        s32 status;
        dma_addr_t  dma_addr;
	int ret = -EAGAIN;

        gmacdev = adapter->synopGMACdev;

        /*Read the Dma interrupt status to know whether the interrupt got generated by our device or not*/
        dma_status_reg = synopGMACReadReg(gmacdev->DmaBase, DmaStatus);

        if(dma_status_reg == 0 && !leftrx)
                return -EAGAIN;

        synopGMAC_disable_interrupt_all(gmacdev);

        if(dma_status_reg & GmacPmtIntr){
                TR("%s:: Interrupt due to PMT module\n",__FUNCTION__);
                //synopGMAC_linux_powerup_mac(gmacdev);
        }

        if(dma_status_reg & GmacMmcIntr){
                TR("%s:: Interrupt due to MMC module\n",__FUNCTION__);
                TR("%s:: synopGMAC_rx_int_status = %08x\n",__FUNCTION__,synopGMAC_read_mmc_rx_int_status(gmacdev));
                TR("%s:: synopGMAC_tx_int_status = %08x\n",__FUNCTION__,synopGMAC_read_mmc_tx_int_status(gmacdev));
        }

        if(dma_status_reg & GmacLineIntfIntr)
        {
		synopGMAC_check_phy_init(gmacdev);
		synopGMAC_mac_init(gmacdev);
        }
        /*Now lets handle the DMA interrupts*/
        interrupt = synopGMAC_get_interrupt_type(gmacdev);
//sw
        if(interrupt == 0 && !leftrx)
                return -EAGAIN;



	if(interrupt & (synopGMACDmaError | synopGMACDmaRxAbnormal)){
                u8 mac_addr[6] = DEFAULT_MAC_ADDRESS;//after soft reset, configure the MAC address to default value
		if(interrupt & synopGMACDmaRxAbnormal){
			DmaDesc * rxdesc = gmacdev->RxBusyDesc;
			TR0("%s::Abnormal Rx Interrupt Seen\n",__FUNCTION__);
			TR0("rxdesc=0x%lx buffer1=0x%lx\n rxbase=0x%x rxcurdec=0x%x rxcuraddr=0x%x\n",(long)rxdesc,(long)rxdesc->buffer1, synopGMACReadReg(gmacdev->DmaBase,DmaRxBaseAddr), synopGMACReadReg(gmacdev->DmaBase, DmaRxCurrDesc), synopGMACReadReg(gmacdev->DmaBase, DmaRxCurrAddr));

			adapter->synopGMACNetStats.rx_over_errors++;
			/*Now Descriptors have been created in synop_handle_received_data(). Just issue a poll demand to resume DMA operation*/
		} else {

			printf("%s::Fatal Bus Error Inetrrupt Seen\n",__FUNCTION__);
			printf("====DMA error!!!\n");
		}

		synopGMAC_disable_dma_tx(gmacdev);
		synopGMAC_disable_dma_rx(gmacdev);

		synopGMAC_reset(gmacdev);//reset the DMA engine and the GMAC ip
#if 0
		/*can not call this, will clear rx desc len info*/		
		synopGMAC_init_tx_rx_desc_queue(gmacdev);
#else
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
#endif
		synopGMAC_take_desc_ownership_tx(gmacdev);
		synopGMAC_give_desc_ownership_rx(gmacdev);

		synopGMAC_set_mac_addr(gmacdev,GmacAddr0High,GmacAddr0Low, mac_addr);
		synopGMAC_init_rx_desc_base(gmacdev);
		synopGMAC_init_tx_desc_base(gmacdev);

#ifdef ENH_DESC_8W
		synopGMAC_dma_bus_mode_init(gmacdev, DmaBurstLength32 | DmaDescriptorSkip2 | DmaDescriptor8Words ); //pbl32 incr with rxthreshold 128 and Desc is 8 Words
#else
		synopGMAC_dma_bus_mode_init(gmacdev, DmaBurstLength4 | DmaDescriptorSkip1 );                      //pbl4 incr with rxthreshold 128 
#endif
		synopGMAC_dma_control_init(gmacdev,DmaStoreAndForward |DmaTxSecondFrame|DmaRxThreshCtrl128 );	
		synopGMAC_check_phy_init(gmacdev);
		synopGMAC_mac_init(gmacdev);
		synopGMAC_enable_dma_rx(gmacdev);
		synopGMAC_enable_dma_tx(gmacdev);
		synopGMAC_resume_dma_rx(gmacdev);//To handle GBPS with 12 descriptors
        }

        if((interrupt & (synopGMACDmaRxNormal|synopGMACDmaRxAbnormal)) || leftrx)
        {
                ret = synop_handle_received_data(adapter, ppacket);
		leftrx = ret > 0;
        }

        if(interrupt & synopGMACDmaRxStopped){
                printf("%s::Receiver stopped seeing Rx interrupts\n",__FUNCTION__); //Receiver gone in to stopped state
                adapter->synopGMACNetStats.rx_over_errors++;
                do{
                        void *skb = plat_alloc_memory(RX_BUF_SIZE);          //should skb aligned here?
                        if(skb == NULL){
                                TR0("ERROR in skb buffer allocation\n");
                                break;
                        }

                        dma_addr = plat_dma_map_single(gmacdev,skb,RX_BUF_SIZE);
                        status = synopGMAC_set_rx_qptr(gmacdev,dma_addr,RX_BUF_SIZE, skb,0,0,0);
                        printf("%s::Set Rx Descriptor no %08x for skb %08x \n",__FUNCTION__,status,skb);
                        if(status < 0)
                        {
                                printf("==%s:no free\n",__FUNCTION__);
                                plat_free_memory((void *)skb);
                        }
                }while(status >= 0);

                        synopGMAC_enable_dma_rx(gmacdev);
        }

        if(interrupt & synopGMACDmaTxNormal)
        {
                //xmit function has done its job
                synop_handle_transmit_over(adapter);    //Do whatever you want after the transmission is over
        }

        if(interrupt & synopGMACDmaTxAbnormal){
                synop_handle_transmit_over(adapter);
        }



        if(interrupt & synopGMACDmaTxStopped){
                printf("%s::Transmitter stopped sending the packets\n",__FUNCTION__);
                synopGMAC_disable_dma_tx(gmacdev);
                synopGMAC_take_desc_ownership_tx(gmacdev);

                synopGMAC_enable_dma_tx(gmacdev);
                TR("%s::Transmission Resumed\n",__FUNCTION__);
        }
//      synopGMAC_clear_interrupt(gmacdev);

        /* Enable the interrrupt before returning from ISR*/
//        synopGMAC_enable_interrupt(gmacdev,DmaIntEnable);
        return ret;
}
/**
 * Function used when the interface is opened for use.
 * We register synopGMAC_linux_open function to linux open(). Basically this 
 * function prepares the the device for operation . This function is called whenever ifconfig (in Linux)
 * activates the device (for example "ifconfig eth0 up"). This function registers
 * system resources needed 
 * 	- Attaches device to device specific structure
 * 	- Programs the MDC clock for PHY configuration
 * 	- Check and initialize the PHY interface 
 *	- ISR registration
 * 	- Setup and initialize Tx and Rx descriptors
 *	- Initialize MAC and DMA
 *	- Allocate Memory for RX descriptors (The should be DMAable)
 * 	- Initialize one second timer to detect cable plug/unplug
 *	- Configure and Enable Interrupts
 *	- Enable Tx and Rx
 *	- start the Linux network queue interface
 * @param[in] pointer to net_device structure. 
 * \return Returns 0 on success and error status upon failure.
 * \callgraph
 */

s32 _synopGMAC_linux_open(struct synopGMACNetworkAdapter *adapter, unsigned char *enetaddr)
{
	s32 status = 0;
	s32 retval = 0;
	int delay = 100;
	
	dma_addr_t  dma_addr;
	void *skb;	//sw	we just use the name skb in pomn
        synopGMACdevice * gmacdev;
	TR0("%s called \n",__FUNCTION__);
	gmacdev = (synopGMACdevice *)adapter->synopGMACdev;
	
	gmacdev->LinkState = -1;
	
	/*Now platform dependent initialization.*/

	//Lets reset the IP
	if (synopGMAC_reset(gmacdev)) {
		printf("gmac rest failed, open failed");
	//	return -EBUSY;
	}
	
	synopGMAC_set_mac_addr(gmacdev,GmacAddr0High,GmacAddr0Low, enetaddr); 
	
	//Lets read the version of ip in to device structure	
	synopGMAC_read_version(gmacdev);
	
	synopGMAC_get_mac_addr(adapter->synopGMACdev,GmacAddr0High,GmacAddr0Low, enetaddr); 
	
	//Check for Phy initialization
	synopGMAC_set_mdc_clk_div(gmacdev,GmiiCsrClk3);
	gmacdev->ClockDivMdc = synopGMAC_get_mdc_clk_div(gmacdev);

	
	//Set up the tx and rx descriptor queue/ring
	synopGMAC_setup_tx_desc_queue(gmacdev,TRANSMIT_DESC_SIZE, RINGMODE);
	synopGMAC_init_tx_desc_base(gmacdev);	//Program the transmit descriptor base address in to DmaTxBase addr

	synopGMAC_setup_rx_desc_queue(gmacdev,RECEIVE_DESC_SIZE, RINGMODE);
	synopGMAC_init_rx_desc_base(gmacdev);	//Program the transmit descriptor base address in to DmaTxBase addr


#ifdef ENH_DESC_8W
	synopGMAC_dma_bus_mode_init(gmacdev, DmaBurstLength32 | DmaDescriptorSkip2 | DmaDescriptor8Words ); //pbl32 incr with rxthreshold 128 and Desc is 8 Words
#else
	synopGMAC_dma_bus_mode_init(gmacdev, DmaBurstLength4 | DmaDescriptorSkip1 );                      //pbl4 incr with rxthreshold 128 
#endif
	
	synopGMAC_dma_control_init(gmacdev,DmaStoreAndForward |DmaTxSecondFrame|DmaRxThreshCtrl128 );	

	
	//Initialize the mac interface
	synopGMAC_check_phy_init(gmacdev);
	synopGMAC_mac_init(gmacdev);
	

	synopGMAC_pause_control(gmacdev); // This enables the pause control in Full duplex mode of operation

	do{
		skb = plat_alloc_memory(RX_BUF_SIZE);		//should skb aligned here?
		if(skb == NULL){
			TR0("ERROR in skb buffer allocation\n");
			break;
		}
		
		dma_addr = plat_dma_map_single(gmacdev,skb,RX_BUF_SIZE);

		status = synopGMAC_set_rx_qptr(gmacdev,dma_addr,RX_BUF_SIZE, skb,0,0,0);
		if(status < 0)
		{
			plat_free_memory((void *)skb);
		}	
	}while(status >= 0);


	synopGMAC_clear_interrupt(gmacdev);
	/*
	Disable the interrupts generated by MMC and IPC counters.
	If these are not disabled ISR should be modified accordingly to handle these interrupts.
	*/	
	synopGMAC_disable_mmc_tx_interrupt(gmacdev, 0xFFFFFFFF);
	synopGMAC_disable_mmc_rx_interrupt(gmacdev, 0xFFFFFFFF);
	synopGMAC_disable_mmc_ipc_rx_interrupt(gmacdev, 0xFFFFFFFF);

	synopGMAC_enable_interrupt(gmacdev,DmaIntEnable);
	synopGMAC_enable_dma_rx(gmacdev);
	synopGMAC_enable_dma_tx(gmacdev);


        plat_delay(DEFAULT_LOOP_VARIABLE);
	synopGMAC_check_phy_init(gmacdev);
	synopGMAC_mac_init(gmacdev);

	return retval;

}

/**
 * Function to transmit a given packet on the wire.
 * Whenever Linux Kernel has a packet ready to be transmitted, this function is called.
 * The function prepares a packet and prepares the descriptor and 
 * enables/resumes the transmission.
 * @param[in] pointer to sk_buff structure. 
 * @param[in] pointer to net_device structure.
 * \return Returns 0 on success and Error code on failure. 
 * \note structure sk_buff is used to hold packet in Linux networking stacks.
 */

s32 _synopGMAC_linux_xmit_frames(struct synopGMACNetworkAdapter *adapter, volatile void *packet, int length)
{
	s32 status = 0;
	dma_addr_t  dma_addr;
	u32 offload_needed = 0;
	void *skb;
	u32 index;
	DmaDesc * dpr;
	int len = length;
	int i;
	char * ptr;

	//u32 flags;
	synopGMACdevice * gmacdev;

	if(adapter == NULL)
		return -1;

	gmacdev = (synopGMACdevice *) adapter->synopGMACdev;
	if(gmacdev == NULL)
		return -1;
//printf("xmit %d busy %d\n", length,synopGMAC_is_desc_owned_by_dma(gmacdev->TxNextDesc));
			if(!synopGMAC_is_desc_owned_by_dma(gmacdev->TxNextDesc))
			{

				skb = plat_alloc_memory(TX_BUF_SIZE);
				if(skb == 0)
				{
					printf("===error in alloc bf1\n");	
					return -1;
				}

				/*Now we have skb ready and OS invoked this function. Lets make our DMA know about this*/

				memcpy((void *)skb, packet, len);
				dma_addr = plat_dma_map_single(gmacdev,skb,len);

				status = synopGMAC_set_tx_qptr(gmacdev, dma_addr, len, skb,0,0,0,offload_needed,&index,dpr);

				if(status < 0){
					printf("%s No More Free Tx Descriptors\n",__FUNCTION__);
					plat_free_memory((void *)skb);
					return -EBUSY;
				}
			}
	
	synopGMAC_resume_dma_tx(gmacdev);
	return -ESYNOPGMACNOERR;
}

static int alaska88e151x_config_init(synopGMACdevice *gmacdev)
{
	int err;
	u16 data;

	// reset phy
	err = synopGMAC_read_phy_reg(gmacdev->MacBase,gmacdev->PhyBase,0x0, &data);
	data = data | 0x8000;
	err = synopGMAC_write_phy_reg(gmacdev->MacBase,gmacdev->PhyBase,0x0,data);

	/*set led link stat*/
	synopGMAC_write_phy_reg(gmacdev->MacBase,gmacdev->PhyBase, 22, 3);
#ifdef LED_88E151X
	synopGMAC_write_phy_reg(gmacdev->MacBase,gmacdev->PhyBase, 16, LED_88E151X);
#else
	synopGMAC_write_phy_reg(gmacdev->MacBase,gmacdev->PhyBase, 16, 0x1038);
#endif
	synopGMAC_write_phy_reg(gmacdev->MacBase,gmacdev->PhyBase, 22, 0);

	if (err < 0)
		return err;
	return 0;
}

int init_phy(synopGMACdevice *gmacdev)
{
	u16 data2;
	u16 data3;
	u16 data;
	synopGMAC_read_phy_reg(gmacdev->MacBase,gmacdev->PhyBase,2,&data2);
	synopGMAC_read_phy_reg(gmacdev->MacBase,gmacdev->PhyBase,3,&data3);

	if(data2 == 0x141 && ((data3 >> 10) == 0x3)){ //marvel ethernet phy
		if(((data3 >> 4) & 0x3f) == 0x0C){  //88E11
			/*set 88e1111 clock phase delay*/
			rtl88e1111_config_init(gmacdev);
		}else if(((data3 >> 4) & 0x3f) == 0x1D) { //88E15
			alaska88e151x_config_init(gmacdev);
		}
	} else if(data2 == 0x22) {
		synopGMAC_write_phy_reg(gmacdev->MacBase,gmacdev->PhyBase,0xc,0xf0f0);
		synopGMAC_write_phy_reg(gmacdev->MacBase,gmacdev->PhyBase,0xb,0x8104);
		synopGMAC_write_phy_reg(gmacdev->MacBase,gmacdev->PhyBase,0xb,0x104);
	}
	else if(data2 == 0x84b9)
	{
		synopGMAC_read_phy_reg(gmacdev->MacBase,gmacdev->PhyBase,22,&data);
		data |= 0x002c;
		synopGMAC_write_phy_reg(gmacdev->MacBase,gmacdev->PhyBase,22,data);

	}

		return 0;
}

s32 _synopGMAC_dummy_reset(struct synopGMACNetworkAdapter * adapter)
{
	synopGMACdevice	* gmacdev;
	DmaDesc * desc;
	
	gmacdev = adapter->synopGMACdev;
	return synopGMAC_reset(gmacdev);
}


int set_lpmode(synopGMACdevice * gmacdev)
{
	u16 data;
	int status;
	int delay;
	
	printf("===reset phy...\n");

	status = synopGMAC_read_phy_reg(gmacdev->MacBase,1,0,&data);
//sw: if you set bit 13,it resets!!
//	data = 0x6000;
	data = 0x4040;
	printf("===set phy loopback mode , reg0: %x\n",data);
	
	status = synopGMAC_write_phy_reg(gmacdev->MacBase,1,0,data);
	if(status != 0)
		return 0;

	delay = 200;
	while(delay > 0)
		delay--;
	
	status = synopGMAC_read_phy_reg(gmacdev->MacBase,1,0,&data);
	printf("===phy loopback mode , reg0: %x\n",data);
	
	return 1;
	
}


int set_phyled(synopGMACdevice * gmacdev)
{
	synopGMAC_write_phy_reg(gmacdev->MacBase,1,0x1c, 0xb842);
}


void reg_init(synopGMACdevice * gmacdev)
{
	u32 data;

	synopGMACWriteReg(gmacdev->DmaBase, DmaBusMode,0x0);
	
	data = synopGMACReadReg(gmacdev->DmaBase, DmaBusMode);
  	data |= 0x400; 
	synopGMACWriteReg(gmacdev->DmaBase, DmaBusMode,data);

	data = synopGMACReadReg(gmacdev->MacBase, GmacConfig );
  	data |= 0x800; 
	synopGMACWriteReg(gmacdev->MacBase, GmacConfig,data);
	
	data = synopGMACReadReg(gmacdev->MacBase, GmacFrameFilter );
  	data |= 0x80000000; 
	synopGMACWriteReg(gmacdev->MacBase, GmacFrameFilter ,data);

	
	data = synopGMACReadReg(gmacdev->DmaBase, DmaControl );
  	data |= 0x2000; 
	synopGMACWriteReg(gmacdev->DmaBase, DmaControl,data);

	data = synopGMACReadReg(gmacdev->MacBase, GmacConfig );
  	data |= 0x4; 
	synopGMACWriteReg(gmacdev->MacBase, GmacConfig,data);

	data = synopGMACReadReg(gmacdev->MacBase, GmacConfig );
  	data |= 0x8; 
	synopGMACWriteReg(gmacdev->MacBase, GmacConfig,data);

	printf("====done! OK!\n");
	
}
	
static int mdio_read(synopGMACPciNetworkAdapter *adapter, int addr, int reg)
{
	synopGMACdevice * gmacdev;
	u16 data;
	gmacdev = adapter->synopGMACdev;
	
	synopGMAC_read_phy_reg((u32 *)gmacdev->MacBase,addr,reg, &data);
	return data;
}

static void mdio_write(synopGMACPciNetworkAdapter *adapter, int addr, int reg, int data)
{
	synopGMACdevice * gmacdev;
	gmacdev = adapter->synopGMACdev;
	synopGMAC_write_phy_reg((u32 *)gmacdev->MacBase,addr,reg,data);
}




void _gmac_halt(struct synopGMACNetworkAdapter *adapter)
{
	synopGMACdevice * gmacdev;
	if(adapter == NULL)
		return;

	gmacdev = (synopGMACdevice *) adapter->synopGMACdev;
	if(gmacdev == NULL)
		return;
	//synopGMAC_dummy_reset(dev);
	synopGMAC_disable_interrupt_all(gmacdev);
	synopGMAC_disable_dma_tx(gmacdev);
	synopGMAC_disable_dma_rx(gmacdev);

	synopGMAC_take_desc_ownership_tx(gmacdev);
	synopGMAC_give_desc_ownership_rx(gmacdev);
}

struct synopGMACNetworkAdapter *gmac_adapter_init(struct synopGMACNetworkAdapter * synopGMACadapter ,ulong base_addr)
{
	
	TR("Now Going to Call register_netdev to register the network interface for GMAC core\n");

	synopGMACadapter->synopGMACdev    = NULL;
	
	/*Allocate Memory for the the GMACip structure*/
	synopGMACadapter->synopGMACdev = (synopGMACdevice *) plat_alloc_memory(sizeof (synopGMACdevice));
	memset((char *)synopGMACadapter->synopGMACdev ,0, sizeof (synopGMACdevice));
	if(!synopGMACadapter->synopGMACdev){
		printf("Error in Memory Allocataion \n");
	}
	
	synopGMAC_attach(synopGMACadapter->synopGMACdev,(ulong) base_addr + MACBASE,(ulong) base_addr + DMABASE, DEFAULT_PHY_BASE, NULL);

	init_phy(synopGMACadapter->synopGMACdev);
	synopGMAC_reset(synopGMACadapter->synopGMACdev);

	

	/* MII setup */
	synopGMACadapter->mii.phy_id_mask = 0x1F;
	synopGMACadapter->mii.reg_num_mask = 0x1F;
	synopGMACadapter->mii.dev = synopGMACadapter;
	synopGMACadapter->mii.mdio_read = mdio_read;
	synopGMACadapter->mii.mdio_write = mdio_write;
	synopGMACadapter->mii.phy_id = synopGMACadapter->synopGMACdev->PhyBase;
	synopGMACadapter->mii.supports_gmii = mii_check_gmii_support(&synopGMACadapter->mii);
	return synopGMACadapter;
}

#ifdef CONFIG_DM_ETH
static int gmac_eth_start(struct udevice *dev)
{
	struct synopGMACNetworkAdapter *adapter = dev_get_priv(dev);
	struct eth_pdata *pdata = dev_get_plat(dev);

	return _synopGMAC_linux_open(adapter, pdata->enetaddr);
}

int gmac_eth_send(struct udevice *dev, void *packet, int length)
{
	struct synopGMACNetworkAdapter *adapter = dev_get_priv(dev);
	_synopGMAC_linux_xmit_frames(adapter, packet, length);
	synop_handle_transmit_over(adapter);
	return 0;
}

int gmac_eth_recv(struct udevice *dev, int flags, uchar **packetp)
{
	int ret;
	struct synopGMACNetworkAdapter *adapter = dev_get_priv(dev);
	synopGMACdevice            *gmacdev = adapter->synopGMACdev;
	ret = synopGMAC_intr_handler(adapter, packetp);
#if defined(CONFIG_HEXDUMP) && defined(DEBUG)
	if (ret > 0) {
	  printf("uboot desc[%d] len %d\n", gmacdev->RxNext, ret);
	  print_hex_dump_bytes("", DUMP_PREFIX_OFFSET, *packetp,
			  ret);
	}
#endif
	return ret;
}

int gmac_eth_free_pkt(struct udevice *dev, uchar *packet, int length)
{
	struct synopGMACNetworkAdapter *adapter = dev_get_priv(dev);
	return gmac_free_pkt(adapter, packet, length);
}

void gmac_eth_stop(struct udevice *dev)
{
	struct synopGMACNetworkAdapter *adapter = dev_get_priv(dev);
	_gmac_halt(adapter);
}

int gmac_eth_write_hwaddr(struct udevice *dev)
{
	struct synopGMACNetworkAdapter *adapter = dev_get_priv(dev);
	struct eth_pdata *pdata = dev_get_plat(dev);
	synopGMACdevice            *gmacdev = adapter->synopGMACdev;
	synopGMAC_set_mac_addr(gmacdev,GmacAddr0High,GmacAddr0Low, pdata->enetaddr); 
	return 0;
}

const struct eth_ops gmac_eth_ops = {
	.start			= gmac_eth_start,
	.send			= gmac_eth_send,
	.recv			= gmac_eth_recv,
	.free_pkt		= gmac_eth_free_pkt,
	.stop			= gmac_eth_stop,
	.write_hwaddr		= gmac_eth_write_hwaddr,
};

int gmac_eth_of_to_plat(struct udevice *dev)
{
	struct eth_pdata *pdata = dev_get_plat(dev);
	const char *phy_mode;
	int ret = 0;

	pdata->iobase = map_physmem(dev_read_addr(dev), 0, MAP_NOCACHE);
	pdata->phy_interface = -1;
	phy_mode = dev_read_string(dev, "phy-mode");
	if (phy_mode)
		pdata->phy_interface = phy_get_interface_by_name(phy_mode);
	if (pdata->phy_interface == -1) {
		debug("%s: Invalid PHY interface '%s'\n", __func__, phy_mode);
		return -EINVAL;
	}

	pdata->max_speed = dev_read_u32_default(dev, "max-speed", 0);


	return ret;
}

static int gmac_eth_bind(struct udevice *dev)
{
	return 0;
}

int gmac_eth_probe(struct udevice *dev)
{
	struct synopGMACNetworkAdapter *synopGMACadapter = dev_get_priv(dev);
	struct eth_pdata *pdata = dev_get_plat(dev);

#ifdef CONFIG_DM_PCI
	if(device_is_on_pci_bus(dev)) {
	char mac[6] = { 0, 1, 2, 3, 4, 5 };
		pdata->iobase = dm_pci_map_bar(dev, PCI_BASE_ADDRESS_0, PCI_REGION_MEM);
		memcpy(pdata->enetaddr, mac, 6);
	}
#endif

	synopGMACadapter = gmac_adapter_init(synopGMACadapter, pdata->iobase);
	return 0;
}

static const struct udevice_id gmac_eth_ids[] = {
	{ .compatible = "loongson,ls1a-gmac" },
	{ }
};

U_BOOT_DRIVER(eth_gmac) = {
	.name	= "eth_gmac",
	.id	= UCLASS_ETH,
	.of_match = gmac_eth_ids,
	.of_to_plat = gmac_eth_of_to_plat,
	.bind	= gmac_eth_bind,
	.probe	= gmac_eth_probe,
	.ops	= &gmac_eth_ops,
	.priv_auto = sizeof(struct synopGMACNetworkAdapter),
	.plat_auto = sizeof(struct eth_pdata),
	.flags = DM_FLAG_ALLOC_PRIV_DMA,
};

static struct pci_device_id gmac_supported[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_LOONGSON, PCI_DEVICE_ID_LOONGSON_GMAC) },
};
U_BOOT_PCI_DEVICE(eth_gmac, gmac_supported);
#else
static int gmac_init(struct eth_device *dev, bd_t * bd)
{	
	return synopGMAC_linux_open(dev);
}


static int gmac_recv(struct eth_device *dev)
{
	return synopGMAC_intr_handler(dev->priv);
}

/* Send a data block via Ethernet. */
static int gmac_send(struct eth_device *dev, volatile void *packet, int length)
{
	struct synopGMACNetworkAdapter *adapter;
	adapter = (struct synopGMACNetworkAdapter *) dev->priv;
	synopGMAC_linux_xmit_frames(dev, packet, length);
	synop_handle_transmit_over(dev->priv);
	return 0;
}

s32 synopGMAC_linux_open(struct eth_device *dev)
{
	struct synopGMACNetworkAdapter *adapter = dev->priv;
	return _synopGMAC_linux_open(adapter, dev->enetaddr);
}

s32 synopGMAC_linux_xmit_frames(struct eth_device *dev, volatile void *packet, int length)
{
	struct synopGMACNetworkAdapter *adapter = dev->priv;
	return _synopGMAC_linux_xmit_frames(adapter, packet, length);
}

s32 synopGMAC_dummy_reset(struct eth_device *dev)
{
	struct synopGMACNetworkAdapter *adapter = dev->priv;
	return _synopGMAC_dummy_reset(adapter);
}

void gmac_halt(struct eth_device *dev)
{
	struct synopGMACNetworkAdapter *adapter = dev->priv;
     _gmac_halt(adapter);
}


/**
 * Function to initialize the Linux network interface.
 * 
 * Linux dependent Network interface is setup here. This provides 
 * an example to handle the network dependent functionality.
 *
 * \return Returns 0 on success and Error code on failure.
 */
s32  gmac_initialize(char* dev_name,ulong base_addr)
{

	struct synopGMACNetworkAdapter * synopGMACadapter;
	struct eth_device *dev;
	synopGMACadapter = (struct synopGMACNetworkAdapter * )plat_alloc_memory(sizeof (struct synopGMACNetworkAdapter)); 
	memset((char *)synopGMACadapter ,0, sizeof (struct synopGMACNetworkAdapter));
	gmac_adapter_init(synopGMACadapter, base_addr);

        dev = malloc(sizeof(*dev));
        if (!dev) {
                return 0;
        }
        memset(dev, 0, sizeof(*dev));

        dev->iobase = base_addr;
	dev->priv = synopGMACadapter;

        memcpy(dev->name, dev_name, ETH_NAME_LEN);

        dev->init = gmac_init;
        dev->halt = gmac_halt;
        dev->send = gmac_send;
        dev->recv = gmac_recv;

        eth_register(dev);

	return 0;
}
#endif

