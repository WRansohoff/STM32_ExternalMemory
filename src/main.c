#include "main.h"

// Reset handler: set the stack pointer and branch to main().
__attribute__( ( naked ) ) void reset_handler( void ) {
  // Set the stack pointer to the 'end of stack' value.
  __asm__( "LDR r0, =_estack\n\t"
           "MOV sp, r0" );
  // Branch to main().
  __asm__( "B main" );
}

/**
 * Main program.
 */
int main( void ) {
  // Copy initialized data from .sidata (Flash) to .data (RAM)
  memcpy( &_sdata, &_sidata, ( ( void* )&_edata - ( void* )&_sdata ) );
  // Clear the .bss section in RAM.
  memset( &_sbss, 0x00, ( ( void* )&_ebss - ( void* )&_sbss ) );

  // Enable floating-point unit.
  SCB->CPACR    |=  ( 0xF << 20 );

  // Set clock speed to 216MHz (each tick is a bit less than 5ns)
  // PLL out = ( 16MHz * ( N / M ) / P ). P = 2, N = 54, M = 2.
  FLASH->ACR   |=  ( 7 << FLASH_ACR_LATENCY_Pos );
  RCC->PLLCFGR &= ~( RCC_PLLCFGR_PLLN |
                     RCC_PLLCFGR_PLLM );
  RCC->PLLCFGR |=  ( ( 54 << RCC_PLLCFGR_PLLN_Pos ) |
                     ( 2 << RCC_PLLCFGR_PLLM_Pos ) );
  RCC->CR      |=  ( RCC_CR_PLLON );
  while ( !( RCC->CR & RCC_CR_PLLRDY ) ) {};
  RCC->CFGR    |=  ( 2 << RCC_CFGR_SW_Pos );
  while ( ( RCC->CFGR & RCC_CFGR_SWS ) != ( 2 << RCC_CFGR_SWS_Pos ) ) {};
  SystemCoreClock = 216000000;

  // Setup the SysTick peripheral to generate an interrupt every 1ms.
  SysTick_Config( SystemCoreClock / 1000 );

  // Enable peripheral clocks: GPIOA-H, USART6, FMC, QSPI, DMA2,
  // SYSCFG, TIM5.
  RCC->AHB1ENR |=  ( RCC_AHB1ENR_GPIOAEN |
                     RCC_AHB1ENR_GPIOBEN |
                     RCC_AHB1ENR_GPIOCEN |
                     RCC_AHB1ENR_GPIODEN |
                     RCC_AHB1ENR_GPIOEEN |
                     RCC_AHB1ENR_GPIOFEN |
                     RCC_AHB1ENR_GPIOGEN |
                     RCC_AHB1ENR_GPIOHEN |
                     RCC_AHB1ENR_DMA2EN );
  RCC->AHB3ENR |=  ( RCC_AHB3ENR_FMCEN |
                     RCC_AHB3ENR_QSPIEN );
  RCC->APB1ENR |=  ( RCC_APB1ENR_TIM5EN );
  RCC->APB2ENR |=  ( RCC_APB2ENR_SYSCFGEN |
                     RCC_APB2ENR_USART6EN );

  // Initialize GPIO pins for the appropriate peripherals.
  initialize_pins();

  // Enable interrupts: EXTI8, DMA2_Stream0.
  NVIC_SetPriorityGrouping( 0 );
  uint32_t low_pri_encoding = NVIC_EncodePriority( 0, 11, 0 );
  NVIC_SetPriority( EXTI9_5_IRQn, low_pri_encoding );
  NVIC_EnableIRQ( EXTI9_5_IRQn );
  uint32_t mid_pri_encoding = NVIC_EncodePriority( 0, 7, 0 );
  NVIC_SetPriority( DMA2_Stream0_IRQn, mid_pri_encoding );
  NVIC_EnableIRQ( DMA2_Stream0_IRQn );

  // Setup USART6 for 115200-baud TX.
  USART6->BRR  =  ( SystemCoreClock / 115200 );
  USART6->CR1 |=  ( USART_CR1_UE | USART_CR1_TE );

  // Start TFT backlight PWM at a 10% duty cycle, 30KHz (TIM5_CH2).
  uint32_t psc = 0;
  uint32_t arr = SystemCoreClock / 30000;
  if ( arr > 0xFFFF ) {
    psc = ( arr / 0xFFFF ) + 1;
    arr = arr / psc;
  }
  uint16_t ccr = ( uint16_t )( arr * 0.1 );
  TIM5->PSC    = psc;
  TIM5->ARR    = arr;
  TIM5->CCR2   = ccr;
  TIM5->CCER  |=  ( TIM_CCER_CC2E );
  TIM5->CCMR1 &= ~( TIM_CCMR1_OC2M );
  TIM5->CCMR1 |=  ( 0x6 << TIM_CCMR1_OC2M_Pos );
  TIM5->EGR   |=  ( TIM_EGR_UG );
  TIM5->CR1   |=  ( TIM_CR1_CEN );

  // Initialize the memory controller for the connected PSRAM.
  // The 1MB of addressable RAM easily fits in the 64MB bank 1.
  // Bank 1 is enabled (but configured for NOR Flash) at reset.
  // So clear the 'flash enable' bit and set it to PSRAM mode.
  FMC_Bank1->BTCR[ 0 ] &= ~( FMC_BCR1_FACCEN |
                             FMC_BCR1_MTYP );
  FMC_Bank1->BTCR[ 0 ] |=  ( 1 << FMC_BCR1_MTYP_Pos );
  // Set access timings.
  FMC_Bank1->BTCR[ 1 ] &= ~( FMC_BTR1_BUSTURN |
                             FMC_BTR1_DATAST |
                             FMC_BTR1_ADDHLD |
                             FMC_BTR1_ADDSET );
  // Bus turnaround: ~5ns, 1 cycle.
  // Data phase duration: ~60ns, 12 cycles.
  // Address setup: ~5ns, 1 cycles.
  // Address hold: ~5ns, 1 cycle.
  FMC_Bank1->BTCR[ 1 ] |=  ( ( 1 << FMC_BTR1_BUSTURN_Pos ) |
                             ( 12 << FMC_BTR1_DATAST_Pos ) |
                             ( 1 << FMC_BTR1_ADDSET_Pos ) |
                             ( 1 << FMC_BTR1_ADDHLD_Pos ) );
  // Test external RAM reads and writes.
  // Note: Uninitialized RAM will hold semi-random values.
  psram  = ( uint32_t* )0x60000000;
  psramh = ( uint16_t* )0x60000000;
  psramb = ( uint8_t*  )0x60000000;
  psram[ 0 ] = 0x01234567;
  psram[ 1 ] = 0x89ABCDEF;
  printf( "RAM[0]: 0x%02X\r\n", psramb[ 0 ] );
  printf( "RAM[0]: 0x%04X\r\n", psramh[ 0 ] );
  printf( "RAM[4]: 0x%08lX\r\n", psram[ 1 ] );
  printf( "RAM[8]: 0x%08lX (uninitialized)\r\n", psram[ 2 ] );

  // Initialize the memory controller for the connected TFT display.
  // It is connected to the 'NE2' chip select signal, so use bank 2.
  FMC_Bank1->BTCR[ 2 ] &= ~( FMC_BCR2_FACCEN |
                             FMC_BCR2_MTYP );
  FMC_Bank1->BTCR[ 3 ] &= ~( FMC_BTR2_BUSTURN |
                             FMC_BTR2_DATAST |
                             FMC_BTR2_ADDHLD |
                             FMC_BTR2_ADDSET );
  // Bus turnaround: ~5ns, 1 cycle.
  // Data phase duration: ~60ns, 12 cycles.
  // Address setup: ~5ns, 1 cycles.
  // Address hold: ~5ns, 1 cycle.
  FMC_Bank1->BTCR[ 3 ] |=  ( ( 1 << FMC_BTR2_BUSTURN_Pos ) |
                             ( 12 << FMC_BTR2_DATAST_Pos ) |
                             ( 1 << FMC_BTR2_ADDSET_Pos ) |
                             ( 1 << FMC_BTR2_ADDHLD_Pos ) );
  FMC_Bank1->BTCR[ 2 ] |=  ( 1 << FMC_BCR2_MTYP_Pos |
                             FMC_BCR2_MBKEN );

  // Initialize the TFT display.
  tft_init();

  // Use the PSRAM as a display framebuffer; clear out an area of
  // 240*240*2 bytes (16 bits per pixel). Start with a purple color.
  // Color format is [MSb] R-G-B [LSb]. 5 bits for R and B, 6 for G.
  for ( int i = 0; i < ( 240 * 240 ); ++i ) { psramh[ i ] = 0x781F; }

  // Configure the DMA peripheral to ferry display data from
  // PSRAM to the display in 'memory-to-memory' mode on stream 0.
  // (Note: DMA1 peripheral does not support memory-to-memory mode.)
  DMA2_Stream0->CR  |=  ( ( 2 << DMA_SxCR_PL_Pos ) |
                          ( 1 << DMA_SxCR_MSIZE_Pos ) |
                          ( 1 << DMA_SxCR_PSIZE_Pos ) |
                          DMA_SxCR_PINC |
                          DMA_SxCR_TCIE |
                          ( 2 << DMA_SxCR_DIR_Pos ) );
  DMA2_Stream0->PAR  =  ( uint32_t )psramh;
  DMA2_Stream0->M0AR =  ( uint32_t )tft_dat;
  DMA2_Stream0->NDTR =  ( 240 * 240 );
  DMA2_Stream0->CR  |=  ( DMA_SxCR_EN );
  // Wait for the transfer to complete.
  while ( DMA2_Stream0->CR & DMA_SxCR_EN ) {};

  // Initialize the QSPI Flash connection.
  qspi_init();
  // Test reading values from memory-mapped Flash.
  int val = *( ( uint32_t* ) 0x90000000 );
  printf( "QSPI[0]: 0x%08X\r\n", val );
  val = *( ( uint32_t* ) 0x90000002 );
  printf( "QSPI[2]: 0x%08X\r\n", val );

  // Set initial LED values.
  // Blue (A5) off, red (A7) off, green (B1) on.
  GPIOA->ODR   &= ~( ( 1 << 5 ) | ( 1 << 7 ) );
  GPIOB->ODR   &= ~( 1 << 1 );

  // Configure pin C8 for EXTI input interrupts on both edges.
  SYSCFG->EXTICR[ ( 8 / 4 ) ] &= ~( SYSCFG_EXTICR3_EXTI8 );
  SYSCFG->EXTICR[ ( 8 / 4 ) ] |=  ( SYSCFG_EXTICR3_EXTI8_PC );
  EXTI->IMR  |=  ( 1 << 8 );
  EXTI->RTSR |=  ( 1 << 8 );

  // Done with the startup logic.
  printf( "Done initializing! Starting display loop...\r\n" );

  // Main loop.
  uint16_t col = 0;
  while ( 1 ) {
    // Set the 'write lock' bit and wait for ongoing DMA to finish.
    wrl = 1;
    while ( bus_busy ) {};

    // Set the new framebuffer color in external memory.
    for ( int i = 0; i < ( 240 * 240 ); ++i ) {
      psramh[ i ] = col;
    }
    col += 1;

    // Reset the display drawing area.
    tft_draw_fullscreen();

    // Release the write lock, and wait for DMA to start again
    // next time the vblank interrupt triggers.
    wrl = 0;
    while ( !bus_busy ) {};
  }
  return 0; // lol
}

// SysTick interrupt handler: increment the global 'systick' value.
void SysTick_IRQn_handler( void ) {
  ++systick;
}

// EXTI interrupt handler for lines 5-9. EXTI8 is connected to the
// display's "tearing effect / vblank" output.
void EXTI9_5_IRQn_handler( void ) {
  if ( EXTI->PR & EXTI_PR_PR8 ) {
    // Start a DMA transfer if the 'write lock' variable is not set.
    if ( !wrl ) {
      DMA2_Stream0->CR  |=  ( DMA_SxCR_EN );
      bus_busy = 1;
    }
    // Acknowledge the interrupt.
    EXTI->PR |=  ( EXTI_PR_PR8 );
  }
}

// DMA2, Stream0 interrupt handler.
void DMA2_Stream0_IRQn_handler( void ) {
  // Clear the 'transfer complete' flag when it is set.
  if ( DMA2->LISR & DMA_LISR_TCIF0 ) {
    DMA2->LIFCR |=  ( DMA_LIFCR_CTCIF0 );
    bus_busy = 0;
  }
}
