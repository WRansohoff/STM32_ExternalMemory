#include "setup.h"

void initialize_pins( void ) {
  // Configure pins A5, A7, B1 as LED outputs.
  GPIOA->MODER    |=  ( ( 1 << ( 5 * 2 ) ) |
                        ( 1 << ( 7 * 2 ) ) );
  GPIOB->MODER    |=  ( 1 << ( 1 * 2 ) );

  // Configure "virtual COM port" pins. (USART6: C6, C7)
  GPIOC->MODER    |=  ( ( 2 << ( 6 * 2 ) ) |
                        ( 2 << ( 7 * 2 ) ) );
  GPIOC->OSPEEDR  |=  ( ( 2 << ( 6 * 2 ) ) |
                        ( 2 << ( 7 * 2 ) ) );
  GPIOC->AFR[ 0 ] |=  ( ( 8 << ( 6 * 4 ) ) |
                        ( 8 << ( 7 * 4 ) ) );

  // Configure B2, B6, C9, C10, D13, E2 for QSPI.
  GPIOB->MODER    |=  ( ( 2 << ( 2 * 2 ) ) |
                        ( 2 << ( 6 * 2 ) ) );
  GPIOB->OSPEEDR  |=  ( ( 3 << ( 2 * 2 ) ) |
                        ( 3 << ( 6 * 2 ) ) );
  GPIOB->PUPDR    |=  ( 1 << ( 6 * 2 ) );
  GPIOB->AFR[ 0 ] |=  ( ( 9 << ( 2 * 4 ) ) |
                        ( 10 << ( 6 * 4 ) ) );
  GPIOC->MODER    |=  ( ( 2 << ( 9 * 2 ) ) |
                        ( 2 << ( 10 * 2 ) ) );
  GPIOC->OSPEEDR  |=  ( ( 3 << ( 9 * 2 ) ) |
                        ( 3 << ( 10 * 2 ) ) );
  GPIOC->AFR[ 1 ] |=  ( ( 9 << ( ( 9 - 8 ) * 4 ) ) |
                        ( 9 << ( ( 10 - 8 ) * 4 ) ) );
  GPIOD->MODER    |=  ( 2 << ( 13 * 2 ) );
  GPIOD->OSPEEDR  |=  ( 3 << ( 13 * 2 ) );
  GPIOD->AFR[ 1 ] |=  ( 9 << ( ( 13 - 8 ) * 4 ) );
  GPIOE->MODER    |=  ( 2 << ( 2 * 2 ) );
  GPIOE->OSPEEDR  |=  ( 3 << ( 2 * 2 ) );
  GPIOE->AFR[ 0 ] |=  ( 9 << ( 2 * 4 ) );

  // FMC Pins: B7, D0, D1, D4, D5, D7, D8, D9, D10, D11, D12, D14,
  // D15, E0, E1, E7, E8, E9, E10, E11, E12, E13, E14, E15, F0, F1,
  // F2, F3, F4, F5, F12, F13, F14, F15, G0, G1, G2, G3, G4, G5, G9
  GPIOB->MODER    |=  ( 2 << ( 7 * 2 ) );
  GPIOB->OSPEEDR  |=  ( 3 << ( 7 * 2 ) );
  GPIOB->AFR[ 0 ] |=  ( 12 << ( 7 * 4 ) );
  GPIOD->MODER    |=  ( ( 2 << ( 0 * 2 ) ) |
                        ( 2 << ( 1 * 2 ) ) |
                        ( 2 << ( 4 * 2 ) ) |
                        ( 2 << ( 5 * 2 ) ) |
                        ( 2 << ( 7 * 2 ) ) |
                        ( 2 << ( 8 * 2 ) ) |
                        ( 2 << ( 9 * 2 ) ) |
                        ( 2 << ( 10 * 2 ) ) |
                        ( 2 << ( 11 * 2 ) ) |
                        ( 2 << ( 12 * 2 ) ) |
                        ( 2 << ( 14 * 2 ) ) |
                        ( 2 << ( 15 * 2 ) ) );
  GPIOD->OSPEEDR  |=  ( ( 3 << ( 0 * 2 ) ) |
                        ( 3 << ( 1 * 2 ) ) |
                        ( 3 << ( 4 * 2 ) ) |
                        ( 3 << ( 5 * 2 ) ) |
                        ( 3 << ( 7 * 2 ) ) |
                        ( 3 << ( 8 * 2 ) ) |
                        ( 3 << ( 9 * 2 ) ) |
                        ( 3 << ( 10 * 2 ) ) |
                        ( 3 << ( 11 * 2 ) ) |
                        ( 3 << ( 12 * 2 ) ) |
                        ( 3 << ( 14 * 2 ) ) |
                        ( 3 << ( 15 * 2 ) ) );
  GPIOD->AFR[ 0 ] |=  ( ( 12 << ( 0 * 4 ) ) |
                        ( 12 << ( 1 * 4 ) ) |
                        ( 12 << ( 4 * 4 ) ) |
                        ( 12 << ( 5 * 4 ) ) |
                        ( 12 << ( 7 * 4 ) ) );
  GPIOD->AFR[ 1 ] |=  ( ( 12 << ( ( 8 - 8 ) * 4 ) ) |
                        ( 12 << ( ( 9 - 8 ) * 4 ) ) |
                        ( 12 << ( ( 10 - 8 ) * 4 ) ) |
                        ( 12 << ( ( 11 - 8 ) * 4 ) ) |
                        ( 12 << ( ( 12 - 8 ) * 4 ) ) |
                        ( 12 << ( ( 14 - 8 ) * 4 ) ) |
                        ( 12 << ( ( 15 - 8 ) * 4 ) ) );
  GPIOE->MODER    |=  ( ( 2 << ( 0 * 2 ) ) |
                        ( 2 << ( 1 * 2 ) ) |
                        ( 2 << ( 7 * 2 ) ) |
                        ( 2 << ( 8 * 2 ) ) |
                        ( 2 << ( 9 * 2 ) ) |
                        ( 2 << ( 10 * 2 ) ) |
                        ( 2 << ( 11 * 2 ) ) |
                        ( 2 << ( 12 * 2 ) ) |
                        ( 2 << ( 13 * 2 ) ) |
                        ( 2 << ( 14 * 2 ) ) |
                        ( 2 << ( 15 * 2 ) ) );
  GPIOE->OSPEEDR  |=  ( ( 3 << ( 0 * 2 ) ) |
                        ( 3 << ( 1 * 2 ) ) |
                        ( 3 << ( 7 * 2 ) ) |
                        ( 3 << ( 8 * 2 ) ) |
                        ( 3 << ( 9 * 2 ) ) |
                        ( 3 << ( 10 * 2 ) ) |
                        ( 3 << ( 11 * 2 ) ) |
                        ( 3 << ( 12 * 2 ) ) |
                        ( 3 << ( 13 * 2 ) ) |
                        ( 3 << ( 14 * 2 ) ) |
                        ( 3 << ( 15 * 2 ) ) );
  GPIOE->AFR[ 0 ] |=  ( ( 12 << ( 0 * 4 ) ) |
                        ( 12 << ( 1 * 4 ) ) |
                        ( 12 << ( 7 * 4 ) ) );
  GPIOE->AFR[ 1 ] |=  ( ( 12 << ( ( 8 - 8 ) * 4 ) ) |
                        ( 12 << ( ( 9 - 8 ) * 4 ) ) |
                        ( 12 << ( ( 10 - 8 ) * 4 ) ) |
                        ( 12 << ( ( 11 - 8 ) * 4 ) ) |
                        ( 12 << ( ( 12 - 8 ) * 4 ) ) |
                        ( 12 << ( ( 13 - 8 ) * 4 ) ) |
                        ( 12 << ( ( 14 - 8 ) * 4 ) ) |
                        ( 12 << ( ( 15 - 8 ) * 4 ) ) );
  GPIOF->MODER    |=  ( ( 2 << ( 0 * 2 ) ) |
                        ( 2 << ( 1 * 2 ) ) |
                        ( 2 << ( 2 * 2 ) ) |
                        ( 2 << ( 3 * 2 ) ) |
                        ( 2 << ( 4 * 2 ) ) |
                        ( 2 << ( 5 * 2 ) ) |
                        ( 2 << ( 12 * 2 ) ) |
                        ( 2 << ( 13 * 2 ) ) |
                        ( 2 << ( 14 * 2 ) ) |
                        ( 2 << ( 15 * 2 ) ) );
  GPIOF->OSPEEDR  |=  ( ( 3 << ( 0 * 2 ) ) |
                        ( 3 << ( 1 * 2 ) ) |
                        ( 3 << ( 2 * 2 ) ) |
                        ( 3 << ( 3 * 2 ) ) |
                        ( 3 << ( 4 * 2 ) ) |
                        ( 3 << ( 5 * 2 ) ) |
                        ( 3 << ( 12 * 2 ) ) |
                        ( 3 << ( 13 * 2 ) ) |
                        ( 3 << ( 14 * 2 ) ) |
                        ( 3 << ( 15 * 2 ) ) );
  GPIOF->AFR[ 0 ] |=  ( ( 12 << ( 0 * 4 ) ) |
                        ( 12 << ( 1 * 4 ) ) |
                        ( 12 << ( 2 * 4 ) ) |
                        ( 12 << ( 3 * 4 ) ) |
                        ( 12 << ( 4 * 4 ) ) |
                        ( 12 << ( 5 * 4 ) ) );
  GPIOF->AFR[ 1 ] |=  ( ( 12 << ( ( 12 - 8 ) * 4 ) ) |
                        ( 12 << ( ( 13 - 8 ) * 4 ) ) |
                        ( 12 << ( ( 14 - 8 ) * 4 ) ) |
                        ( 12 << ( ( 15 - 8 ) * 4 ) ) );
  GPIOG->MODER    |=  ( ( 2 << ( 0 * 2 ) ) |
                        ( 2 << ( 1 * 2 ) ) |
                        ( 2 << ( 2 * 2 ) ) |
                        ( 2 << ( 3 * 2 ) ) |
                        ( 2 << ( 4 * 2 ) ) |
                        ( 2 << ( 5 * 2 ) ) |
                        ( 2 << ( 9 * 2 ) ) );
  GPIOG->OSPEEDR  |=  ( ( 3 << ( 0 * 2 ) ) |
                        ( 3 << ( 1 * 2 ) ) |
                        ( 3 << ( 2 * 2 ) ) |
                        ( 3 << ( 3 * 2 ) ) |
                        ( 3 << ( 4 * 2 ) ) |
                        ( 3 << ( 5 * 2 ) ) |
                        ( 3 << ( 9 * 2 ) ) );
  GPIOG->AFR[ 0 ] |=  ( ( 12 << ( 0 * 4 ) ) |
                        ( 12 << ( 1 * 4 ) ) |
                        ( 12 << ( 2 * 4 ) ) |
                        ( 12 << ( 3 * 4 ) ) |
                        ( 12 << ( 4 * 4 ) ) |
                        ( 12 << ( 5 * 4 ) ) );
  GPIOG->AFR[ 1 ] |=  ( 12 << ( ( 9 - 8 ) * 4 ) );

  // Configure TFT pins: H7 is reset, H11 is backlight control.
  // H11 connects to TIM5_CH2 for PWM output.
  GPIOH->MODER    |=  ( ( 1 << ( 7 * 2 ) ) | ( 2 << ( 11 * 2 ) ) );
  GPIOH->AFR[ 1 ] |=  ( 2 << ( 11 - 8 ) * 4 );
  // C8 is a 'tearing effect / vblank' input (floating input).
  GPIOC->MODER    &= ~( 3 << ( 8 * 2 ) );
}
