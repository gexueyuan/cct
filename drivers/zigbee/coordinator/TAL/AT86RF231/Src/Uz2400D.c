
#include "uz2400d.h"
#include <stdint.h>
//#include "core_cm3.h"
#include "pal_config.h"
#include "tal.h"

//unsigned char  TXFIFO[14]={
//	0x00,         //HEADER LENGTH
//	0x0c,         //PACKET LENGTH
//	0x21,
//	0x88,         //FCF
//	0x00,         //SEQ NO.
//	0xaa,
//	0xaa,         //TARGET PAN ID
//	0x11,
//	0x11,         //TARGET ADDRESS
//	0xaa,
//	0xaa,         //SOURCE PAN ID
//	0x12,
//	0x34,         //SOURCE ADDRESS
//	0x04          //PAYLOAD
//	};

//void SetupRfSendPowerUZ2400(unsigned char dB)
//{
//	unsigned char Value    = 0x00;
//	unsigned char Operator = 0x00;
//
//	switch(dB)
//	{
//	    case 0x1F:
//		    dB = 0;
//			break;
//		case 0x1B:
//		    dB = 1;
//			break;
//		case 0x17:
//		    dB = 3;
//			break;
//		case 0x13:
//		    dB = 5;
//			break;
//		case 0x0F:
//		    dB = 7;
//			break;
//		case 0x0B:
//		    dB = 10;
//			break;
//		case 0x07:
//		    dB = 15;
//			break;
//		case 0x03:
//		    dB = 25;
//			break;
//		default:
//		    break;
//
//	}
//
//	if(dB > 40 || dB < 0)
//	   return;
//
//	if(dB == 0)
//	{ //Maximum Tx Power
//		Value = 0;
//	}
//	else
//	{
//		if((Operator = dB - 40) >=0){ //dB = 40
//			Value |= 0xc0;
//			spi_lw(RFCTRL3, Value); //Set Power
//			Value &= 0x00;
//			Value |=0x40;
//		}else if((Operator = dB - 30) >=0){//30 <= dB < 40
//			Value |= 0xc0;
//		}else if((Operator = dB - 20) >=0){//20 <= dB < 30
//			Value |= 0x80;
//		}else if((Operator = dB - 10) >=0){//10 <= dB < 20
//			Value |= 0x40;
//		}else{                                            // 0 < dB < 10
//			Operator = dB;
//		}
//
//		if(Operator != 0){
//			if(Operator == 1){
//				Value |= 0x08;
//			}else if(Operator == 2 || Operator == 3){
//				Value |= 0x10;
//			}else if(Operator == 4){
//				Value |= 0x18;
//			}else if(Operator == 5){
//				Value |= 0x20;
//			}else if(Operator == 6){
//				Value |= 0x28;
//			}else if(Operator == 7){
//				Value |= 0x30;
//			}else{//Operator == 8 or 9
//				Value |= 0x38;
//			}
//		}
//	}
//
//	spi_lw(RFCTRL3, Value);
//}
//-----------------------------------------------------------------------------
// Delay_192us
//-----------------------------------------------------------------------------
 void Delay_192us(void)
 {
// 	unsigned char temp0;
//	for( temp0 = 0x1e;temp0 != 0 ; temp0--)
//	{
//		asm("nop");
//		asm("nop");
//		asm("nop");
//		asm("nop");
//		asm("nop");
//	}
//	asm("nop");
//	asm("nop");
//	asm("nop");
	 pal_timer_delay(190);
}

/***********************************************/
void spi_sw(unsigned char Address, unsigned char Value){
    uint8_t register_value;
    ENTER_CRITICAL_REGION();
	
	Address = (Address << 1) | 0x01; //Shfit the Address
   	// Step1: Send the WRITE command
	SS_LOW();                       // Activate Slave Select
    SPI_WRITE(Address);				// Write Address to Slave.
   //	while (!SPIF);
   	//SPIF     = 0;
	//Step2: Send the data 
    SPI_WRITE(Value);	                    //Write the data
   	//while (!SPIF);
   	//SPIF     = 0;
    SS_HIGH();                       // Deactivate Slave Select
   	//Delay_us (1);
    LEAVE_CRITICAL_REGION();
    register_value = register_value;
}
/***********************************************/
unsigned char spi_sr(unsigned char Address){
    uint8_t register_value;

    ENTER_CRITICAL_REGION();
	 
	Address = (Address << 1);
	
   	// Step1: Send the READ command
	SS_LOW();                         // Activate Slave Select
   	SPI_WRITE(Address);					// Write Address to Slave.
        //        SPI_READ(register_value);
   //	while (!SPIF);
   //	SPIF     = 0;
   	// Step2: Read the value returned
   	SPI_WRITE(0x00);                       // Dummy write to output serial clock
        //SPI_READ(register_value);
   	//while (!SPIF);                      // Wait for the value to be read
   	//SPIF     = 0;
   	SS_HIGH();                      // Deactivate Slave Select
   	//Delay_us (1);


    LEAVE_CRITICAL_REGION();

    return register_value;
}
/***********************************************/
void spi_lw(unsigned short Address, unsigned char Value)
{
    uint8_t register_value;
    ENTER_CRITICAL_REGION();
	
	Address = (Address << 5) | 0x8010;	   //Shfit the Address
	SS_LOW();                       // Activate Slave Select
    SPI_WRITE(((unsigned char*)&Address)[1]);				// Write Address to Slave.

//	SPI0DAT=((unsigned char*)&Address)[1];    //Write the high byte of the Address
//   	while (!SPIF);
//   	SPIF     = 0;
    SPI_WRITE(((unsigned char*)&Address)[0]);				// Write Address to Slave.
//	SPI0DAT=((unsigned char*)&Address)[0];    //Write the low byte of the Address
//   	while (!SPIF);
//   	SPIF     = 0;
  
    SPI_WRITE(Value);                            //Write the data
  // 	while (!SPIF);
  // 	SPIF     = 0;
    SS_HIGH();                        // Deactivate Slave Select
   	//Delay_us (1);
    LEAVE_CRITICAL_REGION();
    register_value = register_value;
}
/***********************************************/
unsigned char spi_lr(unsigned short Address) {

    uint8_t register_value;
    ENTER_CRITICAL_REGION();


	Address = (Address << 5) | 0x8000;		 //Shfit the Address
	SS_LOW();
	 
    SPI_WRITE(((unsigned char*)&Address)[1]);				// Write Address to Slave.
    SPI_WRITE(((unsigned char*)&Address)[0]);				// Write Address to Slave.


    SPI_WRITE(0x00);
   // SPI_READ(register_value);
    SS_HIGH();                        // Deactivate Slave Select
   //	Delay_us (1);

    LEAVE_CRITICAL_REGION();

    return register_value;
}
/***********************************************/
void spi_fill_fifo(unsigned short Address, unsigned char *DataPtr, unsigned char Length){

    uint8_t register_value;
    ENTER_CRITICAL_REGION();

	Address = (Address << 5) | 0x8010;		//Shfit the Address 
	SS_LOW();
    SPI_WRITE(((unsigned char*)&Address)[1]);				// Write Address to Slave.
    SPI_WRITE(((unsigned char*)&Address)[0]);				// Write Address to Slave.

	while (Length--)              //FOR FIFO, JUST NEED GIVE THE FIRST DATA'S ADDRESS, FIFO CAN ADD ADDRESS ITSELF
	     {
	    	SPI_WRITE(*DataPtr++);
	     }
	SS_HIGH();                       // Deactivate Slave Select
   //	Delay_us (1);
    LEAVE_CRITICAL_REGION();
    register_value = register_value;
}
/***********************************************/
void spi_rd_rx_fifo(unsigned char *DataPtr, unsigned char Length) {
	//unsigned char Length;
    uint8_t register_value;
    ENTER_CRITICAL_REGION();
	
	SS_LOW();
	SPI_WRITE(0xE0);

	SPI_WRITE(0x00);


	//SPI_WRITE(0x00);

	//Length=register_value;
	//Length += 2;
  	while(Length--){
  		SPI_WRITE(0x00);
                //SPI_READ(register_value);
        *DataPtr = register_value;
        DataPtr++;
  	}
  	SS_HIGH();                         // Deactivate Slave Select
   	//Delay_us (1);
    LEAVE_CRITICAL_REGION();
   // register_value = register_value;
}
/***********************************************/
void UmInit(void)//芯片初始化
{
 //unsigned char i;
	PAL_RST_LOW();
	Delay_192us();
    PAL_RST_HIGH();
    PAL_SLP_TR_HIGH();
	do 
  		spi_sw(0x26, 0x20); //enable SPI sync
	while((spi_sr(0x26)&0x20)!= 0x20); //check SPI sync status

	//spi_sw(0x00, 0x04); //set coordinator
	spi_sw(PAONSETUP, 0x08); //fine-tune TX timing

	spi_sw(FIFOEN, 0x94); //fine-tune TX timing
	spi_sw(TXPEMISP, 0x95); //fine-tune TX timing
	spi_sw(BBREG2, 0xBC);
	spi_sw(BBREG3, 0x50);
	spi_sw(BBREG5, 0x07);
	spi_sw(BBREG6, 0x40); //append RSSI value to received packet

	spi_lw(RFCTRL0, 0x03); //RF optimized control
	spi_lw(RFCTRL1, 0x01); //RF optimized control
	spi_lw(RFCTRL2, 0x74); //RF optimized control
	spi_lw(RFCTRL4, 0x06); //RF optimized control
	spi_lw(RFCTRL6, 0x10); //RF optimized control
	spi_lw(RFCTRL7, 0xec); //RF optimized control
	spi_lw(RFCTRL8, 0x8c); //RF optimized control, VDD >= 2.0
	spi_lw(GPIO_DIR, 0x00); //Setting GPIO to output mode
	//spi_lw(SEC_APP, 0x20); //enable IEEE802.15.4-2006 security support
	spi_lw(RFCTL50, 0x07); //for DC-DC off. Attention! DC-DC on and off can not coexiest.
	spi_lw(RFCTL51, 0xc0); //RF optimized control
	spi_lw(RFCTL52, 0x01); //RF optimized control
	spi_lw(RFCTL59, 0x00); //RF optimized control
	spi_lw(RFCTL73, 0x80); //RF optimized control, VDD >= 2.0V
	spi_lw(RFCTL74, 0xe5); //RF optimized control
	spi_lw(RFCTL75, 0x13); //RF optimized control
	spi_lw(RFCTL76, 0x07); //RF optimized control

	//spi_sw(ACKTMOUT, 0x80); //enable pend bit in ack

	spi_sw(INTMSK, ~0x09); //clear all interrupt masks 00
	spi_sw(SOFTRST, 0x02); //reset baseband

	//spi_sw(SPIRXF, 0x01); //flush fifo(spi_sr(RXFLUSH)|0x01)

	spi_sw(RFCTL, 0x04); //reset RF
	spi_sw(RFCTL, 0x00);
	spi_sw(RFCTL, 0x02);
	Delay_192us();
	spi_sw(0x36, 0x01);
	Delay_192us();
	spi_sw(0x36, 0x00);




	//spi_sw(SLPACK, 0x7f);


//	while(spi_lr (RFSTATE) != 0x50);


	
	
	
//    spi_sw(PAONSETUP, 0x08);
//    spi_sw(FIFOEN, 0x94);
//    spi_sw(TXPEMISP, 0x95);
//    spi_sw(BBREG3, 0x50);
//    spi_sw(BBREG5, 0x07);
//    spi_sw(BBREG6, 0x40);
//    spi_lw(RFCTRL0,0x03);
//    spi_lw(RFCTRL1,0x02);
//    spi_lw(RFCTRL2, 0x66);
//    spi_lw(RFCTRL4,0x06);
//    spi_lw(RFCTRL6, 0x30);
//    spi_lw(RFCTRL7, 0xe0);
//    spi_lw(RFCTRL8, 0x8C);
//	spi_lw(GPIO_DIR,0x00);
////	spi_lw(SEC_APP,0x20);
//	spi_lw(SEC_APP,0x10);
//	spi_lw(RFCTL50,0x05);
//	spi_lw(RFCTL51,0xC0);
//	spi_lw(RFCTL52,0x01);
//	spi_lw(RFCTL59,0x00);
//	spi_lw(RFCTL73,0x40);
//	spi_lw(RFCTL74,0xc5);
//	spi_lw(RFCTL75,0x13);
//	spi_lw(RFCTL76,0x07);
//	spi_sw(INTMSK, 0x00);
//
////	spi_sw(TXMCR,0x12);
////	spi_sw(TXPEND,0x1c);
//	spi_sw(SOFTRST, 0x02);
//	spi_sw(RFCTL, 0x04);
//	spi_sw(RFCTL, 0x00);
//	spi_sw(RFCTL, 0x02);
//	//wait 192us
//	Delay_192us();
//	spi_sw(RFCTL, 0x01);
//	//wait 192us
//	Delay_192us();
//	spi_sw(RFCTL, 0x00);
//	spi_sw(SLPACK, 0x7f);
//
//
//	while(spi_lr (RFSTATE) != 0x50);









// ------ U-Power 1000D enable --------------------
 spi_lw(TESTMODE, 0x29);
 //spi_lw(0x203, 0x00);//f8
 //spi_lw(0x253, 0x00);//0f
 //spi_lw(0x274, 0xe5);//8A
//-----------------------------------------------
 

}


void SetupRfSendPowerUZ2400(unsigned char dB)
{
	switch(dB)
	{
	    case 0:		  //0
		    spi_lw(RFCTL53, 0x00);
		    spi_lw(RFCTL74, 0xC5);
		    spi_lw(RFCTRL3, 0x00);
			break;
		case 1:		  //-1
		    spi_lw(RFCTL53, 0x00);
		    spi_lw(RFCTL74, 0xC5);
		    spi_lw(RFCTRL3, 0x28);
			break;
		case 2:		  //-3
		    spi_lw(RFCTL53, 0x00);
		    spi_lw(RFCTL74, 0xC5);
		    spi_lw(RFCTRL3, 0x80);
			break;
		case 3:		  //-5
		    spi_lw(RFCTL53, 0x00);
		    spi_lw(RFCTL74, 0xC5);
		    spi_lw(RFCTRL3, 0xB8);
			break;
		case 4:		  //-7
		    spi_lw(RFCTL53, 0x00);
		    spi_lw(RFCTL74, 0xC5);
		    spi_lw(RFCTRL3, 0xE0);
			break;
		case 5:		  //-10
		    spi_lw(RFCTL53, 0x0C);
		    spi_lw(RFCTL74, 0x81);
		    spi_lw(RFCTRL3, 0xF8);
			break;
		case 6:		  //-15
		    spi_lw(RFCTL53, 0x0C);
		    spi_lw(RFCTL74, 0x09);
		    spi_lw(RFCTRL3, 0xF8);
			break;
		case 7:		  //-25
		    spi_lw(RFCTL53, 0x09);
		    spi_lw(RFCTL74, 0x01);
		    spi_lw(RFCTRL3, 0xF8);
			break;
		default:
		    spi_lw(RFCTL53, 0x00);
		    spi_lw(RFCTL74, 0xC5);
		    spi_lw(RFCTRL3, 0x00);
		    break;
	}

}
void UzSetChannel(unsigned char NewChannel)             //选择通道11-26，对应2405MHz-2480MHz
{

	unsigned char val;

    if (NewChannel > 26 || NewChannel < 11)
    return; //Check Channel Range
/*
    while(spi_lr(RFSTATE) != 0x50);
    spi_lw(RFCTRL0, ((NewChannel - 11) << 4) + 0x02);   // Shift logic channel, Program Channel
    reset_RF_state_machine();
*/
	val = spi_sr(0x26);
	spi_sw(0x26, val|0x2);

	spi_lw(0x200, ((NewChannel - 11) << 4) | 0x03); //set channel

	spi_sw(0x36, 0x02);
	Delay_192us(); //delay 192us
	spi_sw(0x36, 0x01);
	Delay_192us(); //delay 192us
	spi_sw(0x36, 0x00);

	spi_sw(0x26, val);
       
}
/***********************************************/
void UMID()       		//INT THE MODULE ADDRESS
{
    uint8_t *ptr_to_reg;

	spi_sw(PANIDL,tal_pib.PANId); // PAN ID
	spi_sw(PANIDH,tal_pib.PANId>>8);

    ptr_to_reg = (uint8_t *)&tal_pib.IeeeAddress;
    for (uint8_t i = 0; i < 8; i++)
    {
    	spi_sw((EADR0 + i), *ptr_to_reg);
        ptr_to_reg++;
    }


	spi_sw(SADRL,tal_pib.ShortAddress);  //Short Address
	spi_sw(SADRH,tal_pib.ShortAddress>>8);

    /* Configure TX_ARET; CSMA and CCA */
	//spi_sw(SR_CCA_MODE, tal_pib.CCAMode);

#ifdef SW_CONTROLLED_CSMA
    /*
     * If receiver is enabled during backoff periods,
     * CSMA and frame re-transmissions are handled by software.
     * Setup trx for immediate transmission.
     */
    pal_trx_bit_write(SR_MAX_FRAME_RETRIES, 0);
    pal_trx_bit_write(SR_MAX_CSMA_RETRIES, 7);
#else
   // pal_trx_bit_write(SR_MIN_BE, tal_pib.MinBE);
   // pal_trx_bit_write(SR_MAX_BE, tal_pib.MaxBE);
#endif

    //pal_trx_bit_write(SR_AACK_I_AM_COORD, tal_pib.PrivatePanCoordinator);

    /* set phy parameter */

#ifdef HIGH_DATA_RATE_SUPPORT
    apply_channel_page_configuration(tal_pib.CurrentPage);
#endif

    SetupRfSendPowerUZ2400(tal_pib.TransmitPower);
	UzSetChannel(tal_pib.CurrentChannel);
}

//void TX_TEST()
//{
//  	unsigned char temp=5;
//  	spi_fill_fifo(0x000,TXFIFO,14);
//	spi_sw(SOFTRST, 0x02);
//	spi_sw(TXNMTRIG, 0x01);	      //TRIGGER THE TX
//	while(temp--)
//	{
//		if(spi_sr(0x31)&0x01)  //CHECK TX FINISHED
//		{
//			//LED = 1;
//			//Delay_ms(500);
//			//LED = 0;
//			//Delay_ms(500);
//	        break;
//     	}
//	}
//}

void reset_RF_state_machine(void)//复位状态机
{
                              //Reset RF state machine
    spi_sw(RFCTL, 0x04);
    //spi_sw(RFCTL, 0x00);
                              //wait until RX state
    //while(spi_lr(RFSTATE) != 0x50);
}


