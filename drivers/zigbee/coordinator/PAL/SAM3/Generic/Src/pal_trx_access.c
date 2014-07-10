/**
 * @file pal_trx_access.c
 *
 * @brief Transceiver registers & Buffer accessing functions for
 *        AT91SAM3 32bit SPI based MCUs.
 *
 * This file implements functions for reading and writing transceiver
 * registers and transceiver buffer for AT91SAM3 32bit SPI based MCUs.
 *
 * $Id: pal_trx_access.c 29250 2011-11-10 13:41:43Z uwalter $
 *
 * @author    Atmel Corporation: http://www.atmel.com
 * @author    Support email: avr@atmel.com
 */
/*
 * Copyright (c) 2010, Atmel Corporation All rights reserved.
 *
 * Licensed under Atmel's Limited License Agreement --> EULA.txt
 */
/* === Includes ============================================================= */

#include <stdint.h>
#include <string.h>
#include "pal.h"
#include "return_val.h"
#include "pal_trx_access.h"

/* === Macros =============================================================== */

/* === Prototypes =========================================================== */

/* === Implementation ======================================================= */

/**
 * @brief Writes data into a transceiver register
 *
 * This function writes a value into transceiver register.
 *
 * @param addr Address of the trx register
 * @param data Data to be written to trx register
 *
 */

//void halCommonSpiWrite(u16 data)
//{
//
//   SPI_I2S_SendData(SPI1, data); 
//
//   /* Wait until the Rx FIFO buffer will become not empty */
//   while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE)==RESET);
//        
//   /* Read a data */
//   SPI_I2S_ReceiveData(SPI1);
//
//} /* end halCommonSpiWrite() */
//
//u16 halCommonSpiReadWrite(u16 write_data)
//{
//   u16 read_data;
//
//   SPI_I2S_SendData(SPI1, write_data); 
//
//   /* Wait until the Rx FIFO buffer will become not empty */
//   while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE)==RESET);
//        
//   /* Read a data */
//   read_data = SPI_I2S_ReceiveData(SPI1);
//
//   return read_data;
//} /* end halCommonSpiReadWrite */









void pal_trx_reg_write(uint8_t addr, uint8_t data)
{   
    uint8_t register_value;
    ENTER_CRITICAL_REGION();
    

    /* Prepare the command byte */
    addr |= WRITE_ACCESS_COMMAND;

    /* Start SPI transaction by pulling SEL low */
    SS_LOW();
 
    /* Send the Read command byte */
    SPI_WRITE(addr);
//    while ((SPI1->SR & SPI_I2S_FLAG_RXNE) == 0);          
//    register_value = SPI1->DR;  

    /* Write the byte in the transceiver data register */
    SPI_WRITE(data);
//    while ((SPI1->SR & SPI_I2S_FLAG_RXNE) == 0);       
//    register_value = SPI1->DR;  
//    
    
//    
//        halCommonSpiWrite(addr);
//
//    halCommonSpiWrite(data);
    
    /* Stop the SPI transaction by setting SEL high */
    SS_HIGH();

    LEAVE_CRITICAL_REGION();
    register_value = register_value;
}



/**
 * @brief Reads current value from a transceiver register
 *
 * This function reads the current value from a transceiver register.
 *
 * @param addr Specifies the address of the trx register from which
 *             the data shall be read
 *
 * @return Value of the register read
 */
uint8_t pal_trx_reg_read(uint8_t addr)
{
    uint8_t register_value;

    ENTER_CRITICAL_REGION();

    /* Prepare the command byte */
    addr |= READ_ACCESS_COMMAND;

    /* Start SPI transaction by pulling SEL low */
    SS_LOW();
    
    
    
    
    
    
    
    /* Send the write command byte */
    SPI_WRITE(addr);
    /*
     * Done to clear the RDRF bit in the SPI status register, which will be set
     * as a result of reception of some data from the transceiver as a result
     * of SPI write operation done above.
     */
 //   SPI_READ(register_value);

    /* Do dummy write for initiating SPI read */
    SPI_WRITE(SPI_DUMMY_VALUE);

    /* Read the byte received */
 //   SPI_READ(register_value);
    
//        register_value = halCommonSpiReadWrite(addr);
//    register_value = halCommonSpiReadWrite(SPI_DUMMY_VALUE);
    
    
    
    
    
    
    
    
    
    
    /* Stop the SPI transaction by setting SEL high */
    SS_HIGH();

    LEAVE_CRITICAL_REGION();

    return register_value;
}



/**
 * @brief Reads frame buffer of the transceiver
 *
 * This function reads the frame buffer of the transceiver.
 *
 * @param[out] data Pointer to the location to store frame
 * @param[in] length Number of bytes to be read from the frame buffer.
 */
void pal_trx_frame_read(uint8_t *data, uint8_t length)
{
  //  uint8_t dummy_rx_data;
    uint8_t register_value;
    ENTER_CRITICAL_REGION();

    /* Start SPI transaction by pulling SEL low */
    SS_LOW();

    /* Send the command byte */
    SPI_WRITE(TRX_CMD_FR);

    /*
     * Done to clear the RDRF bit in the SPI status register, which will be set
     * as a result of reception of some data from the transceiver as a result
     * of SPI write operation done above.
     */
  //  SPI_READ(dummy_rx_data);

    /*
     * Done to avoid compiler warning about variable being not used after
     * setting.
     */
  //  dummy_rx_data = dummy_rx_data;
#ifdef NODMA_SPI
    do
    {
        /* Do dummy write for initiating SPI read */
        SPI_WRITE(SPI_DUMMY_VALUE);

        /* Upload the received byte in the user provided location */
        //SPI_READ(*data);
        *data = register_value;
        data++;

    }
    while (--length > 0);
#else

    /* Disable both read and write. */
    SPI_USED->SPI_PTCR = SPI_PTCR_RXTDIS | SPI_PTCR_TXTDIS;

    memset(data, 0xFF, length);

    SPI_USED->SPI_RPR = SPI_USED->SPI_TPR = (uint32_t)data;
    SPI_USED->SPI_RCR = SPI_USED->SPI_TCR = length;

    /* Enable read and write. */
    SPI_USED->SPI_PTCR = SPI_PTCR_RXTEN | SPI_PTCR_TXTEN;

    /* Wait for end of read */
    while (SPI_USED->SPI_RCR);

#endif
    SS_HIGH();

    LEAVE_CRITICAL_REGION();
}



/**
 * @brief Writes data into frame buffer of the transceiver
 *
 * This function writes data into the frame buffer of the transceiver
 *
 * @param[in] data Pointer to data to be written into frame buffer
 * @param[in] length Number of bytes to be written into frame buffer
 */
void pal_trx_frame_write(uint8_t *data, uint8_t length)
{
    uint8_t command_data = TRX_CMD_FW ;
    uint8_t register_value;
    
    ENTER_CRITICAL_REGION();

    /* Start SPI transaction by pulling SEL low */
    SS_LOW();

#ifdef NODMA_SPI
    /* Send the command byte */
    SPI_WRITE(command_data);


    do
    {
        /* Upload the user data to transceiver data register */
        SPI_WRITE(*data);
        data++;

    }
    while (--length > 0);
#else
    /* DMA transfer for SAM3S  */
    /* Disable both read and write. */

    SPI_USED->SPI_PTCR = SPI_PTCR_TXTDIS;

    /* Setup dma transfer including trx command byte */
    SPI_USED->SPI_TPR =  (uint32_t)&command_data;
    SPI_USED->SPI_TCR = (uint16_t)1;
    SPI_USED->SPI_TNPR = (uint32_t)data;
    SPI_USED->SPI_TNCR = (uint16_t)length;
    /* start transfer */
    SPI_USED->SPI_PTCR = SPI_PTCR_TXTEN;
    /* Wait while transfer isnt finished */
    while (!(SPI_USED->SPI_SR & SPI_SR_TXEMPTY) ||
           !(SPI_USED->SPI_SR & SPI_SR_TXBUFE));

    SPI_USED->SPI_PTCR = SPI_PTCR_TXTDIS;
#endif
    /* Wait for end of write; send counter should not matter. */
    /* Stop the SPI transaction by setting SEL high. */
    SS_HIGH();

    LEAVE_CRITICAL_REGION();
    register_value = register_value;
}



/**
 * @brief Subregister read
 *
 * @param addr Offset of the register
 * @param mask Bit mask of the subregister
 * @param pos  Bit position of the subregister
 *
 * @return  value of the read bit(s)
 */
uint8_t pal_trx_bit_read(uint8_t addr, uint8_t mask, uint8_t pos)
{
    uint8_t ret;

    ret = pal_trx_reg_read(addr);
    ret &= mask;
    ret >>= pos;

    return ret;
}



/**
 * @brief Subregister write
 *
 * @param[in] reg_addr Offset of the register
 * @param[in] mask Bit mask of the subregister
 * @param[in] pos Bit position of the subregister
 * @param[out] new_value Data, which is muxed into the register
 */
void pal_trx_bit_write(uint8_t reg_addr, uint8_t mask, uint8_t pos, uint8_t new_value)
{
    uint8_t current_reg_value;

    current_reg_value = pal_trx_reg_read(reg_addr);
    current_reg_value &= (uint8_t)~(uint32_t)mask;  // Implicit casting required to avoid IAR Pa091.
    new_value <<= pos;
    new_value &= mask;
    new_value |= current_reg_value;

    pal_trx_reg_write(reg_addr, new_value);
}



#if defined(ENABLE_TRX_SRAM) || defined(DOXYGEN)
/**
 * @brief Writes data into SRAM of the transceiver
 *
 * This function writes data into the SRAM of the transceiver
 *
 * @param addr Start address in the SRAM for the write operation
 * @param data Pointer to the data to be written into SRAM
 * @param length Number of bytes to be written into SRAM
 */
void pal_trx_sram_write(uint8_t addr, uint8_t *data, uint8_t length)
{
    ENTER_CRITICAL_REGION();

    /* Start SPI transaction by pulling SEL low */
    SS_LOW();

    /* Send the command byte */
    SPI_WRITE(TRX_CMD_SW);

    /* Send the address from which the write operation should start */
    SPI_WRITE(addr);

#ifdef NODMA_SPI
    do
    {
        /* Upload the user data to transceiver data register */
        SPI_WRITE(*data);
        data++;

    }
    while (--length > 0);
#else
    /* DMA transfer for SAM3S  */
    /* Disable both read and write. */

    SPI_USED->SPI_PTCR = SPI_PTCR_TXTDIS;

    /* Setup dma transfer including trx command byte */
    SPI_USED->SPI_TPR = (uint32_t)data;
    SPI_USED->SPI_TCR = (uint16_t)length;
    /* start transfer */
    SPI_USED->SPI_PTCR = SPI_PTCR_TXTEN;
    /* Wait while transfer isnt finished */
    while (!(SPI_USED->SPI_SR & SPI_SR_TXEMPTY) ||
           !(SPI_USED->SPI_SR & SPI_SR_TXBUFE));

    SPI_USED->SPI_PTCR = SPI_PTCR_TXTDIS;
#endif

    /* Stop the SPI transaction by setting SEL high */
    SS_HIGH();

    LEAVE_CRITICAL_REGION();
}
#endif  /* #if defined(ENABLE_TRX_SRAM) || defined(DOXYGEN) */



#if defined(ENABLE_TRX_SRAM) || defined(ENABLE_TRX_SRAM_READ) || defined(DOXYGEN)
/**
 * @brief Reads data from SRAM of the transceiver
 *
 * This function reads from the SRAM of the transceiver
 *
 * @param[in] addr Start address in SRAM for read operation
 * @param[out] data Pointer to the location where data stored
 * @param[in] length Number of bytes to be read from SRAM
 */
void pal_trx_sram_read(uint8_t addr, uint8_t *data, uint8_t length)
{
    uint8_t dummy_rx_data;

    PAL_WAIT_500_NS();

    ENTER_CRITICAL_REGION();

    /* Start SPI transaction by pulling SEL low */
    SS_LOW();

    /* Send the command byte */
    SPI_WRITE(TRX_CMD_SR);

    /*
     * Done to clear the RDRF bit in the SPI status register, which will be set
     * as a result of reception of some data from the transceiver as a result
     * of SPI write operation done above.
     */
    SPI_READ(dummy_rx_data);

    /* Send the address from which the read operation should start */
    SPI_WRITE(addr);

    /*
     * Done to clear the RDRF bit in the SPI status register, which will be set
     * as a result of reception of some data from the transceiver as a result
     * of SPI write operation done above.
     */
    SPI_READ(dummy_rx_data);

    /*
     * Done to avoid compiler warning about variable being not used after
     * setting.
     */
    dummy_rx_data = dummy_rx_data;

#ifdef NODMA_SPI
    do
    {
        /* Do dummy write for initiating SPI read */
        SPI_WRITE(SPI_DUMMY_VALUE);

        /* Upload the received byte in the user provided location */
        SPI_READ(*data);
        data++;

    }
    while (--length > 0);
#else

    /* Disable both read and write. */
    SPI_USED->SPI_PTCR = SPI_PTCR_RXTDIS | SPI_PTCR_TXTDIS;

    memset(data, 0xFF, length);

    SPI_USED->SPI_RPR = SPI_USED->SPI_TPR = (uint32_t)data;
    SPI_USED->SPI_RCR = SPI_USED->SPI_TCR = length;

    /* Enable read and write. */
    SPI_USED->SPI_PTCR = SPI_PTCR_RXTEN | SPI_PTCR_TXTEN;

    /* Wait for end of read */
    while (SPI_USED->SPI_RCR);

#endif

    SS_HIGH();

    LEAVE_CRITICAL_REGION();
}
#endif  /* #if defined(ENABLE_TRX_SRAM) || defined(ENABLE_TRX_SRAM_READ) || defined(DOXYGEN) */

#if defined(ENABLE_TRX_SRAM) || defined(DOXYGEN)
/**
 * @brief Writes and reads data into/from SRAM of the transceiver
 *
 * This function writes data into the SRAM of the transceiver and
 * simultaneously reads the bytes.
 *
 * @param addr Start address in the SRAM for the write operation
 * @param idata Pointer to the data written/read into/from SRAM
 * @param length Number of bytes written/read into/from SRAM
 */
void pal_trx_aes_wrrd(uint8_t addr, uint8_t *idata, uint8_t length)
{
    uint8_t dummy_rx_data;
    uint8_t *odata;

    PAL_WAIT_500_NS();

    ENTER_CRITICAL_REGION();

    /* Start SPI transaction by pulling SEL low */
    SS_LOW();

    /* Send the command byte */
    SPI_WRITE(TRX_CMD_SW);

    /*
     * Done to clear the RDRF bit in the SPI status register, which will be set
     * as a result of reception of some data from the transceiver as a result
     * of SPI write operation done above.
     */
    SPI_READ(dummy_rx_data);

    /* Write SRAM start address, clear the RDRF bit */
    SPI_WRITE(addr);
    SPI_READ(dummy_rx_data);

    /* Now transfer data */
    odata = idata;

    /* Write data byte 0 - the obtained value in SPDR is meaningless */
    SPI_WRITE(*idata++);
    SPI_READ(dummy_rx_data);

    /*
     * Done to avoid compiler warning about variable being not used after
     * setting.
     */
    dummy_rx_data = dummy_rx_data;

    /* Process data bytes 1...length-1: write and read */
    do
    {
        SPI_WRITE(*idata++);

        /* Upload the received byte in the user provided location */
        SPI_READ(*odata++);

    }
    while (--length > 0);

    /* To get the last data byte, write some dummy byte */
    SPI_WRITE(SPI_DUMMY_VALUE);
    SPI_READ(*odata);

    /* Stop the SPI transaction by setting SEL high */
    SS_HIGH();

    LEAVE_CRITICAL_REGION();
}
#endif  /* #if defined(ENABLE_TRX_SRAM) || defined(DOXYGEN) */

/* EOF */
