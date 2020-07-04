#include "qspi.h"

void qspi_init( void ) {
  // Set Flash size; 512Mb = 64MB = 2^(25+1) bytes.
  QUADSPI->DCR |=  ( 25 << QUADSPI_DCR_FSIZE_Pos );
  // Set 1-wire data mode with 32-bit addressing.
  QUADSPI->CCR |=  ( ( 3 << QUADSPI_CCR_ADSIZE_Pos ) |
                     ( 1 << QUADSPI_CCR_IMODE_Pos ) );
  // Wait an extra half-cycle to read, and set a clock prescaler.
  QUADSPI->CR  |=  ( QUADSPI_CR_SSHIFT |
                     ( 2 << QUADSPI_CR_PRESCALER_Pos ) );
  // Enable the peripheral.
  QUADSPI->CR  |=  ( QUADSPI_CR_EN );

  // Send 'enter QSPI mode' command.
  QUADSPI->CCR |=  ( 0x35 << QUADSPI_CCR_INSTRUCTION_Pos );
  while ( QUADSPI->SR & QUADSPI_SR_BUSY ) {};
  qspi_reg_wait( 0x05, 0x41, 0x40 );
  // Use all 4 data wires once the command is acknowledged.
  QUADSPI->CCR |=  ( 3 << QUADSPI_CCR_IMODE_Pos );

  // Send 'enable 4-byte addressing' command.
  // Note: the peripheral may start a new transfer as soon as the
  // 'instruction' field is written, so it is safest to disable
  // the peripheral before clearing that field.
  while ( QUADSPI->SR & QUADSPI_SR_BUSY ) {};
  QUADSPI->CR  &= ~( QUADSPI_CR_EN );
  QUADSPI->CCR &= ~( QUADSPI_CCR_INSTRUCTION );
  QUADSPI->CR  |=  ( QUADSPI_CR_EN );
  QUADSPI->CCR |=  ( 0xB7 << QUADSPI_CCR_INSTRUCTION_Pos );
  while ( QUADSPI->SR & QUADSPI_SR_BUSY ) {};
  qspi_reg_wait( 0x15, 0x20, 0x20 );

  // Test writing some data.
  // No need to run this every time; Flash is non-volatile, but it
  // has limited "write endurance" on the order of ~10k-100k cycles.
  //qspi_erase_sector( 0 );
  //qspi_write_word( 0, 0x12345678 );

  // Enable memory-mapped mode. MX25L512 Flash chips use
  // 6 "dummy cycles" with Quad I/O "fast read" instructions by
  // default, which allows up to 84MHz communication speed.
  QUADSPI->CR  &= ~( QUADSPI_CR_EN );
  QUADSPI->CCR &= ~( QUADSPI_CCR_INSTRUCTION );
  QUADSPI->CCR |= ( 3 << QUADSPI_CCR_FMODE_Pos |
                    3 << QUADSPI_CCR_ADMODE_Pos |
                    3 << QUADSPI_CCR_DMODE_Pos |
                    3 << QUADSPI_CCR_IMODE_Pos |
                    0xEC << QUADSPI_CCR_INSTRUCTION_Pos |
                    6 << QUADSPI_CCR_DCYC_Pos );
  QUADSPI->CR  |=  ( QUADSPI_CR_EN );
}

// Use 'status-polling' mode to wait for Flash register status.
void qspi_reg_wait( uint8_t reg, uint32_t msk, uint32_t mat ) {
  // Disable the peripheral.
  QUADSPI->CR   &= ~( QUADSPI_CR_EN );
  // Set the 'mask', 'match', and 'polling interval' values.
  QUADSPI->PSMKR = msk;
  QUADSPI->PSMAR = mat;
  QUADSPI->PIR   = 0x10;
  // Set the 'auto-stop' bit to end the transaction after a match.
  QUADSPI->CR   |=  ( QUADSPI_CR_APMS );
  // Clear instruction, data, and address modes.
  QUADSPI->CCR  &= ~( QUADSPI_CCR_INSTRUCTION |
                      QUADSPI_CCR_IMODE |
                      QUADSPI_CCR_DMODE |
                      QUADSPI_CCR_ADMODE );
  // Set 4-wire instruction and data modes, and auto-polling mode.
  QUADSPI->CCR  |=  ( 3 << QUADSPI_CCR_IMODE_Pos |
                      3 << QUADSPI_CCR_DMODE_Pos |
                      2 << QUADSPI_CCR_FMODE_Pos );
  // Re-enable the peripheral.
  QUADSPI->CR   |=  ( QUADSPI_CR_EN );
  // Set the given 'read register' instruction.
  QUADSPI->CCR  |=  ( reg << QUADSPI_CCR_INSTRUCTION_Pos );
  // Wait for a match.
  while ( QUADSPI->SR & QUADSPI_SR_BUSY ) {};
  // Acknowledge the 'status match flag.'
  QUADSPI->FCR |=  ( QUADSPI_FCR_CSMF );
  // Un-set the data mode and disable auto-polling.
  QUADSPI->CCR  &= ~( QUADSPI_CCR_FMODE |
                      QUADSPI_CCR_DMODE );
}

// Enable writes on the QSPI Flash. Must be done before every
// erase / program operation.
void qspi_wen() {
  // Wait for the peripheral to finish any ongoing transactions.
  while ( QUADSPI->SR & QUADSPI_SR_BUSY ) {};
  // Disable the peripheral and clear the data / address modes.
  QUADSPI->CR  &= ~( QUADSPI_CR_EN );
  QUADSPI->CCR &= ~( QUADSPI_CCR_INSTRUCTION |
                     QUADSPI_CCR_DMODE |
                     QUADSPI_CCR_ADMODE );
  // Re-enable the peripheral and send the 'write enable' command.
  QUADSPI->CR  |=  ( QUADSPI_CR_EN );
  QUADSPI->CCR |=  ( 0x06 << QUADSPI_CCR_INSTRUCTION_Pos );
  // Wait for the transaction to finish.
  while ( QUADSPI->SR & QUADSPI_SR_BUSY ) {};
  // Wait until 'writes enabled' is set in the config register.
  qspi_reg_wait( 0x05, 0x43, 0x42 );
}

// Erase a 4KB sector. Sector address = ( snum * 0x1000 )
void qspi_erase_sector( uint32_t snum ) {
  // Send 'enable writes' command.
  qspi_wen();
  // Erase the sector, and wait for the operation to complete.
  while ( QUADSPI->SR & QUADSPI_SR_BUSY ) {};
  QUADSPI->CR  &= ~( QUADSPI_CR_EN );
  QUADSPI->CCR &= ~( QUADSPI_CCR_INSTRUCTION |
                     QUADSPI_CCR_DMODE |
                     QUADSPI_CCR_ADMODE );
  QUADSPI->CCR |=  ( 3 << QUADSPI_CCR_ADMODE_Pos );
  QUADSPI->CR  |=  ( QUADSPI_CR_EN );
  QUADSPI->CCR |=  ( 0x20 << QUADSPI_CCR_INSTRUCTION_Pos );
  // The address is equal to the sector number * 4KB.
  QUADSPI->AR   =  ( snum * 0x1000 );
  while ( QUADSPI->SR & QUADSPI_SR_BUSY ) {};
  QUADSPI->CR  &= ~( QUADSPI_CR_EN );
  QUADSPI->CCR &= ~( QUADSPI_CCR_ADMODE |
                     QUADSPI_CCR_INSTRUCTION );
  // Wait for the 'write in progress' bit to clear.
  qspi_reg_wait( 0x05, 0x43, 0x40 );
}

// Write one word of data (4 bytes) to a QSPI Flash chip.
void qspi_write_word( uint32_t addr, uint32_t data ) {
  // Send 'enable writes' command.
  qspi_wen();
  // Write the word of data.
  while ( QUADSPI->SR & QUADSPI_SR_BUSY ) {};
  QUADSPI->CR  &= ~( QUADSPI_CR_EN );
  QUADSPI->CCR &= ~( QUADSPI_CCR_INSTRUCTION );
  QUADSPI->CCR |=  ( 3 << QUADSPI_CCR_ADMODE_Pos |
                     3 << QUADSPI_CCR_DMODE_Pos );
  // Set data length (3 + 1 = 4 bytes).
  QUADSPI->DLR = 3;
  QUADSPI->CR  |=  ( QUADSPI_CR_EN );
  QUADSPI->CCR |=  ( 0x02 << QUADSPI_CCR_INSTRUCTION_Pos );
  QUADSPI->DR   =  ( data );
  QUADSPI->AR   =  ( addr );
  while ( QUADSPI->SR & QUADSPI_SR_BUSY ) {};
  QUADSPI->CR  &= ~( QUADSPI_CR_EN );
  QUADSPI->DLR = 0;
  QUADSPI->CCR &= ~( QUADSPI_CCR_ADMODE |
                     QUADSPI_CCR_DMODE |
                     QUADSPI_CCR_INSTRUCTION );
  // Wait for the 'write in progress' bit to clear.
  qspi_reg_wait( 0x05, 0x41, 0x40 );
}
