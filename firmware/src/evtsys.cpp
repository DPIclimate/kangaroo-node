#include <Arduino.h>

void setup_GCLK(void) {
  GCLK->GENCTRL.reg = GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_OSCULP32K | GCLK_GENCTRL_ID(5);
}

void setup_EIC(void) {
  // Enable the EIC in the power manager.
  PM->APBAMASK.reg |= PM_APBAMASK_EIC;

  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |
                      GCLK_CLKCTRL_GEN_GCLK5 |
                      GCLK_CLKCTRL_ID_EIC;

  // Connect PA19 to the EIC. The EIC is function A on every pin.
  PORT->Group[PORTA].PINCFG[19].bit.PMUXEN = 1;
  PORT->Group[PORTA].PMUX[19 >> 1].reg |= PORT_PMUX_PMUXO_A;

  // PA19 uses EXTINT3. Check for a falling edge and use filtering.
  EIC->CONFIG[0].reg |= EIC_CONFIG_SENSE3_FALL | EIC_CONFIG_FILTEN3;

  // Tell the EIC to generate an event when the sense condition of the pin is detected.
  EIC->EVCTRL.bit.EXTINTEO3 = 1;

  // Enable the EIC.
  EIC->CTRL.reg |= EIC_CTRL_ENABLE;
  while (EIC->STATUS.bit.SYNCBUSY);

}

void setup_TC(void) {
  PM->APBCMASK.bit.TC3_ = 1;

  TC3->COUNT16.CTRLA.reg = TC_CTRLA_MODE_COUNT16;

  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |
                      GCLK_CLKCTRL_GEN_GCLK5 |
                      GCLK_CLKCTRL_ID_TCC2_TC3;

  TC3->COUNT16.READREQ.reg = TC_READREQ_RCONT |
                             TC_READREQ_ADDR(TC_COUNT16_COUNT_OFFSET);

  TC3->COUNT16.EVCTRL.reg = TC_EVCTRL_TCEI |
                            TC_EVCTRL_EVACT_COUNT;

  TC3->COUNT16.CTRLA.reg = TC_CTRLA_RUNSTDBY |
                           TC_CTRLA_ENABLE;

  while (TC3->COUNT16.STATUS.bit.SYNCBUSY);
}

void setup_EVSYS(void) {
  PM->APBCMASK.reg |= PM_APBCMASK_EVSYS;

  EVSYS->USER.reg = EVSYS_USER_CHANNEL(1) |                                // Attach the event user (receiver) to channel 0 (n + 1)
                    EVSYS_USER_USER(EVSYS_ID_USER_TC3_EVU);                // Set the event user (receiver) as timer TC4

  EVSYS->CHANNEL.reg = EVSYS_CHANNEL_EDGSEL_NO_EVT_OUTPUT |                // No event edge detection
                       EVSYS_CHANNEL_PATH_ASYNCHRONOUS |                   // Set event path as asynchronous
                       EVSYS_CHANNEL_EVGEN(EVSYS_ID_GEN_EIC_EXTINT_3) |    // Set event generator (sender) as external interrupt 3
                       EVSYS_CHANNEL_CHANNEL(0);                           // Attach the generator (sender) to channel 0

}
