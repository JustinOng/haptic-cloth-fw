#include <pdk/device.h>

#include "auto_sysclock.h"
#include "delay.h"
#include "helpers.h"

// Motor connected to PB7 (TM3PWM)

#define LDR_BIT 4

void main() {
  // Configure LDR as analog input, disable digital input
  cbi(PADIER, LDR_BIT);

  ADCM = ADCM_CLK_SYSCLK_DIV2;  // SYSCLK=1MHz, ADC Clock = 500kHz
  ADCC = (ADCC_ADC_ENABLE | ADCC_CH_AD9_PA4);

  // Initialise Timer 3
  TM3C = (TM3C_CLK_IHRC | TM3C_OUT_PB7 | TM3C_MODE_PWM);
  TM3CT = 0;                                                          // clear counter
  TM3S = (TM3S_PWM_RES_8BIT | TM3S_PRESCALE_NONE | TM3S_SCALE_DIV3);  // clock = 16M / 3
  // Overall PWM frequency of 20kHz

  while (1) {
    sbi(ADCC, ADCC_PROCESS_CONTROL_BIT);
    while (!(ADCC & ADCC_IS_ADC_CONV_READY))
      ;

    uint8_t val = 255 - ADCR;
    TM3B = val;
    _delay_ms(10);
  }
}

// Startup code - Setup/calibrate system clock
unsigned char _sdcc_external_startup(void) {
  // Initialize the system clock (CLKMD register) with the IHRC, ILRC, or EOSC clock source and correct divider.
  // The AUTO_INIT_SYSCLOCK() macro uses F_CPU (defined in the Makefile) to choose the IHRC or ILRC clock source and divider.
  // Alternatively, replace this with the more specific PDK_SET_SYSCLOCK(...) macro from pdk/sysclock.h
  AUTO_INIT_SYSCLOCK();

  // Insert placeholder code to tell EasyPdkProg to calibrate the IHRC or ILRC internal oscillator.
  // The AUTO_CALIBRATE_SYSCLOCK(...) macro uses F_CPU (defined in the Makefile) to choose the IHRC or ILRC oscillator.
  // Alternatively, replace this with the more specific EASY_PDK_CALIBRATE_IHRC(...) or EASY_PDK_CALIBRATE_ILRC(...) macro from easy-pdk/calibrate.h
  AUTO_CALIBRATE_SYSCLOCK(TARGET_VDD_MV);

  // ROP |= ROP_TMX_16MHZ | ROP_TMX_6BIT;

  return 0;  // Return 0 to inform SDCC to continue with normal initialization.
}
