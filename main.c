/***************************************************************************//**
 * @file
 * @brief Simple LED Blink Demo
 *******************************************************************************
 * # License
 * <b>Copyright 2018 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "bsp.h"
#include "em_msc.h"
#include "em_ramfunc.h"
volatile uint32_t msTicks; /* counts 1ms timeTicks */

void Delay(uint32_t dlyTicks);

/***************************************************************************//**
 * @brief SysTick_Handler
 * Interrupt Service Routine for system tick counter
 ******************************************************************************/
void SysTick_Handler(void)
{
  msTicks++;       /* increment counter necessary in Delay()*/
}

/***************************************************************************//**
 * @brief Delays number of msTick Systicks (typically 1 ms)
 * @param dlyTicks Number of ticks to delay
 ******************************************************************************/
void Delay(uint32_t dlyTicks)
{
  uint32_t curTicks;

  curTicks = msTicks;
  while ((msTicks - curTicks) < dlyTicks) ;
}
//__attribute__((section(".ram")))
//SL_RAMFUNC_DEFINITION_BEGIN
void erasepage(void)
{
  /* Unlock the MSC module. */
  MSC->LOCK = MSC_UNLOCK_CODE;
  /* Disable writing to the Flash. */
  MSC->WRITECTRL &= ~MSC_WRITECTRL_WREN;
  // Disable MSC prefetch
  MSC->READCTRL  &= ~_MSC_READCTRL_PREFETCH_MASK;

  // Disable the MSC cache
  MSC->READCTRL |= MSC_READCTRL_IFCDIS;
  // Enable writes
  MSC->WRITECTRL |= MSC_WRITECTRL_WREN;

  // Address to erase
  MSC->ADDRB = 0x0FE00000;

  // Load internal write address register from MSC_ADDRB
  MSC->WRITECMD = MSC_WRITECMD_LADDRIM;

  // Turn on PC2
  GPIO->P[2].DOUT = 0x4;

  // Enable high voltage to start erase operation
  MSC->WRITECMD = MSC_WRITECMD_ERASEPAGE;

  // Flush pipeline after starting erase
  __NOP();
  __ISB();

  // Turn off PC2
  GPIO->P[2].DOUT = 0x0;

  // Make sure erase is complete
  while (MSC->STATUS & MSC_STATUS_BUSY);

  // Halt here
  __BKPT(0);


}
//SL_RAMFUNC_DEFINITION_END
/***************************************************************************//**
 * @brief  Main function
 ******************************************************************************/
SYSTEM_ChipRevision_TypeDef chipRev;
uint32_t *addr = (uint32_t *)0x0FE00000;
uint32_t data[] = { 0x00EF3200 };
int main(void)
{
  EMU_DCDCInit_TypeDef dcdcInit = EMU_DCDCINIT_STK_DEFAULT;

  /* Chip errata */
  CHIP_Init();

  EMU_DCDCInit(&dcdcInit);
  SYSTEM_ChipRevisionGet(&chipRev);
  /* Setup SysTick Timer for 1 msec interrupts  */
  if (SysTick_Config(CMU_ClockFreqGet(cmuClock_CORE) / 1000)) {
    while (1) ;
  }

  CMU_ClockEnable(cmuClock_HFPER, true);
  CMU_ClockEnable(cmuClock_GPIO, true);
  GPIO_PinModeSet(gpioPortC,2,gpioModePushPull,0);
  // Turn off PC2
  GPIO->P[2].DOUT = 0x0;
#if 0
  MSC_Init();
  MSC_ErasePage(addr);
  MSC_WriteWord(addr, data, sizeof(data));
  MSC_Deinit();
  erasepage();
#endif
  /* Initialize LED driver */
  BSP_LedsInit();
  // Turn on LED1
  BSP_LedToggle(1);

  /* Infinite blink loop */
  while (true) {
    Delay(1000);
    // Toggle LEDs
    BSP_LedToggle(0);
    BSP_LedToggle(1);
  }
}
