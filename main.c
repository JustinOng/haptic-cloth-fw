#include <pdk/device.h>

#include "auto_sysclock.h"
#include "delay.h"
#include "helpers.h"

// Motor connected to PB7 (TM3PWM)
#define LDR_BIT 4
// Pin receiving 8N1 1200baud UART RX
#define UART_RX_BIT 1

// 1 if a byte has been received on UART
// set in ISR, cleared by main after processing
volatile uint8_t flag_uart_rx = 0;
// contains byte last received on UART
volatile uint8_t uart_rx_data = 0;

enum UART_rx_state {
  STATE_WAIT_START_BIT,
  STATE_READ_DATA
};

void interrupt(void) __interrupt(0) {
  // we only have one interrupt vector enabled, so don't bother checking INTRQ
  static enum UART_rx_state state = STATE_WAIT_START_BIT;
  // Store number of interrupts since start bit was first detected
  // Since 1200baud and 9.6kHz int, we expect 8 interrupts per bit period
  // First data bit is read at count=8 + 4 (offset 4 so at center of bit period),
  //   then at 8 bit multiples subsequently
  static uint8_t rx_read_count = 0;

  switch (state) {
    case STATE_WAIT_START_BIT:
      if ((PB & (1 << UART_RX_BIT)) == 0) {
        // Start bit received
        state = STATE_READ_DATA;
        uart_rx_data = 0;
        rx_read_count = 0;
      }
      break;
    case STATE_READ_DATA:
      rx_read_count++;

      if (
          rx_read_count == 4 + (1 * 8) ||
          rx_read_count == 4 + (2 * 8) ||
          rx_read_count == 4 + (3 * 8) ||
          rx_read_count == 4 + (4 * 8) ||
          rx_read_count == 4 + (5 * 8) ||
          rx_read_count == 4 + (6 * 8) ||
          rx_read_count == 4 + (7 * 8) ||
          rx_read_count == 4 + (8 * 8)) {
        // read in data bit, LSB first
        if (PB & (1 << UART_RX_BIT)) {
          uart_rx_data = 0x80 | (uart_rx_data >> 1);
        } else {
          uart_rx_data = uart_rx_data >> 1;
        }
      } else if (rx_read_count == 4 + (9 * 9)) {
        // stop bit, has to be HIGH
        if (PB & (1 << UART_RX_BIT)) {
          flag_uart_rx = 1;
        }

        state = STATE_WAIT_START_BIT;
      }
      break;
    default:
      state = STATE_WAIT_START_BIT;
      break;
  }

  // Clear interrupt flag
  cbi(INTRQ, INTRQ_TM2_BIT);
}

void main() {
  //tmp: PA0 as output for LED
  sbi(PAC, 0);

  // Configure LDR as analog input, disable digital input
  cbi(PADIER, LDR_BIT);

  ADCM = ADCM_CLK_SYSCLK_DIV2;  // SYSCLK=1MHz, ADC Clock = 500kHz
  ADCC = (ADCC_ADC_ENABLE | ADCC_CH_AD9_PA4);

  // Configure pin for UART_RX_BIT as input with pull-up
  cbi(PBC, UART_RX_BIT);
  sbi(PBDIER, UART_RX_BIT);
  sbi(PBPH, UART_RX_BIT);
  cbi(PBPL, UART_RX_BIT);

  // Initialise Timer 2 for software UART RX
  // Interrupt at 1200 baud * 8 = 9600Hz
  TM2C = (TM2C_CLK_IHRC | TM2C_MODE_PERIOD);
  TM2CT = 0;  // clear counter
  // Int freq = 16MHz / 1 / 7 / 238 = 9603.84Hz
  TM2S = (TM2S_PWM_RES_8BIT | TM2S_SCALE_NONE | TM2S_SCALE_DIV7);
  TM2B = 238;

  // Enable interrupts on Timer 2
  sbi(INTEN, INTEN_TM2_ENABLE_BIT);

  // Initialise Timer 3 for output of motor PWM
  TM3C = (TM3C_CLK_IHRC | TM3C_OUT_PB7 | TM3C_MODE_PWM);
  TM3CT = 0;                                                          // clear counter
  TM3S = (TM3S_PWM_RES_8BIT | TM3S_PRESCALE_NONE | TM3S_SCALE_DIV3);  // clock = 16M / 3
  // Overall PWM frequency of 20kHz

  __engint();

  while (1) {
    sbi(ADCC, ADCC_PROCESS_CONTROL_BIT);
    while (!(ADCC & ADCC_IS_ADC_CONV_READY))
      ;

    uint8_t val = 255 - ADCR;
    TM3B = val;
    if (flag_uart_rx) {
      if (uart_rx_data == 'a') {
        PA ^= (1 << 0);
      }

      flag_uart_rx = 0;
    }
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
