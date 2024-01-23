#include <Arduino.h>
#include "davis.h"
#include <lmic.h>
#include "node_config.h"
#include "evtsys.h"

#include <RTCZero.h>

#define MAJOR 1
#define MINOR 0
#define REV   2

typedef union {
    float value;
    uint8_t bytes[4];
} FLOAT;

static constexpr int MAX_MSG = 512;

Davis davis(WIND_DIR_PIN, WIND_SPEED_PIN);
RTCZero rtc;

static bool joined = false;

static bool first_run = true;

static int32_t delta_seconds = 900;

uint8_t payloadBuffer[18];

constexpr float AD_VOLTS = 3.3f;
constexpr int AD_BIT_SIZE = 10;
static float ad_step = 0;

void do_send(void);
void sensors(void);

// A buffer for printing log messages.
static char msg[MAX_MSG];

// A printf-like function to print log messages prefixed by the current
// LMIC tick value. Don't call it before os_init();
void log_msg(const char *fmt, ...) {
#ifdef USE_SERIAL
    snprintf(msg, MAX_MSG, "%04d-%02d-%02d %02d:%02d:%02d ", rtc.getYear()+2000, rtc.getMonth(), rtc.getDay(), rtc.getHours(), rtc.getMinutes(), rtc.getSeconds());
    serial.write(msg, strlen(msg));
    va_list args;
    va_start(args, fmt);
    vsnprintf(msg, MAX_MSG, fmt, args);
    va_end(args);
    serial.write(msg, strlen(msg));
    serial.println();
#endif
}

void blink(void) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(250);
    digitalWrite(LED_BUILTIN, LOW);
    delay(250);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(250);
    digitalWrite(LED_BUILTIN, LOW);
    delay(250);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(250);
    digitalWrite(LED_BUILTIN, LOW);
}

// This variable is used to keep track of LMIC state changes. When this differs from LMIC.opmode
// the LMIC state has changed, and this can be used as a flag to log the new state.
uint16_t lmicOpmode = LMIC.opmode;

#define SD_CARD_MAX 384
void log_opmode(void) {
    static char sdCardMsg[SD_CARD_MAX];

    if (lmicOpmode == 0) {
        log_msg("LMIC.opmode: OP_NONE");
    } else {
        memset(sdCardMsg, 0, SD_CARD_MAX);
        if (lmicOpmode & OP_SHUTDOWN) strncat(sdCardMsg, "OP_SHUTDOWN ", SD_CARD_MAX - strnlen(sdCardMsg, SD_CARD_MAX));
        if (lmicOpmode & OP_JOINING) strncat(sdCardMsg, "OP_JOINING ", SD_CARD_MAX - strnlen(sdCardMsg, SD_CARD_MAX));
        if (lmicOpmode & OP_TXDATA) strncat(sdCardMsg, "OP_TXDATA ", SD_CARD_MAX - strnlen(sdCardMsg, SD_CARD_MAX));
        if (lmicOpmode & OP_TXRXPEND) strncat(sdCardMsg, "OP_TXRXPEND ", SD_CARD_MAX - strnlen(sdCardMsg, SD_CARD_MAX));
        if (lmicOpmode & OP_RNDTX) strncat(sdCardMsg, "OP_RNDTX ", SD_CARD_MAX - strnlen(sdCardMsg, SD_CARD_MAX));
        if (lmicOpmode & OP_POLL) strncat(sdCardMsg, "OP_POLL ", SD_CARD_MAX - strnlen(sdCardMsg, SD_CARD_MAX));
        if (lmicOpmode & OP_NEXTCHNL) strncat(sdCardMsg, "OP_NEXTCHNL ", SD_CARD_MAX - strnlen(sdCardMsg, SD_CARD_MAX));
        log_msg("LMIC.opmode: %s (raw: %u)", sdCardMsg, lmicOpmode);
        memset(sdCardMsg, 0, SD_CARD_MAX);
    }
}

void onEvent (ev_t ev) {
    switch(ev) {
        case EV_JOINING:
            digitalWrite(LED_BUILTIN, HIGH);
            log_msg("EV_JOINING");
            joined = false;
            break;

        case EV_JOINED:
            digitalWrite(LED_BUILTIN, LOW);
            log_msg("EV_JOINED");
            LMIC_setLinkCheckMode(0);
            joined = true;
            break;

        case EV_JOIN_FAILED:
            log_msg("EV_JOIN_FAILED");
            // Signal loop() to reset LMIC so the current join attempts will be abandoned
            // and to try again later.
            joined = false;
            break;

        case EV_TXCOMPLETE:
            digitalWrite(LED_BUILTIN, LOW);
            log_msg("EV_TXCOMPLETE");
            break;

        case EV_TXSTART:
            digitalWrite(LED_BUILTIN, HIGH);
            log_msg("EV_TXSTART");
            break;

        case EV_JOIN_TXCOMPLETE:
            log_msg("EV_JOIN_TXCOMPLETE: no JoinAccept");
            break;

      case EV_SCAN_TIMEOUT:
      case EV_BEACON_FOUND:
      case EV_BEACON_MISSED:
      case EV_BEACON_TRACKED:
      case EV_RFU1:
      case EV_REJOIN_FAILED:
      case EV_LOST_TSYNC:
      case EV_RESET:
      case EV_RXCOMPLETE:
      case EV_LINK_DEAD:
      case EV_LINK_ALIVE:
      case EV_SCAN_FOUND:
      case EV_TXCANCELED:
      case EV_RXSTART:
        break;
    }
}

void do_send(void) {
    log_msg("============================================================");
    sensors();
    LMIC_setTxData2(1, payloadBuffer, 13, 0);
}

void sensors(void) {
    static constexpr uint32_t dirReadDelayInMs = 500;
    static constexpr size_t dirReadCount = 15000 / dirReadDelayInMs;

    log_msg("Measuring wind speed & direction");
    //davis.startSpeedMeasurement();

    /*
     * https://math.stackexchange.com/questions/44621/calculate-average-wind-direction
     *
     * Unit vector calc - does not take wind speed into account:
     *
        u_east = mean(sin(WD * pi/180))
        u_north = mean(cos(WD * pi/180))
        unit_WD = arctan2(u_east, u_north) * 180/pi
        unit_WD = (360 + unit_WD) % 360
     */
    float sumVx = 0.0f;
    float sumVy = 0.0f;
    float windRadians;

    // These are static to reduce the stack frame size.
    static float values[dirReadCount];

    uint32_t rawDir;
    float windDir;

    for (size_t i = 0; i < dirReadCount; i++) {
        rawDir = davis.getDirectionRaw();
        windDir = davis.getDirectionDegrees(rawDir);

        windRadians = (windDir * PI) / 180.0f;

        float sx = sinf(windRadians);
        sumVx = sumVx + sx;
        float sy = cosf(windRadians);
        sumVy = sumVy + sy;

        values[i] = abs(atan2(sx, sy));

        //log_msg("Direction: %.2f, sx = %.2f, sy = %.2f, values[%d] = %.2f", windDir, sx, sy, i, values[i]);

        delay(dirReadDelayInMs);
    }

    //uint32_t windCount = davis.stopSpeedMeasurement();

    //float meanVx = sumVx / dirReadCount;
    //float meanVy = sumVy / dirReadCount;

    float avgDirRadians = atan2f(sumVx, sumVy);
    float avgDirDeg = (avgDirRadians * 180.0f) / PI;

    //log_msg("avgDirDeg before fmodf: %.2f, avg dir radians: %.2f", avgDirDeg, avgDirRadians);

    avgDirDeg = fmodf((360 + avgDirDeg), 360);
    int16_t avgDeg = (int16_t)avgDirDeg;

    float stdSum = 0.0f;
    float absAvgDirRadians = abs(avgDirRadians);
    for (size_t i = 0; i < dirReadCount; i++) {
        stdSum += powf((values[i] - absAvgDirRadians), 2);
    }

    FLOAT stdDev;
    stdDev.value = sqrtf(stdSum / (dirReadCount - 1));  // std dev in radians
    stdDev.value = (stdDev.value * 180.0f) / PI;  // convert std dev to degrees

    log_msg("Avg wind direction over %lu s = %f (%d), std dev: %.2f", dirReadCount * dirReadDelayInMs / 1000, avgDirDeg, avgDeg, stdDev.value);

    uint32_t P = 0xFFFFFFFF;
    if ( ! first_run) {
        P = TC3->COUNT32.COUNT.reg;
    }

    TC3->COUNT32.COUNT.reg = 0;

    log_msg("Wind pulse count(P): %lu", P);

    FLOAT windKph;
    windKph.value = davis.getSpeedKph(P, delta_seconds);

    log_msg("Wind speed: count = %lu, kph = %.2f", P, windKph.value);

    uint32_t batteryReading = analogRead(VBATT);
    log_msg("measuredvbat as read: %lu", batteryReading);
    // The battery voltage goes through a voltage divider before connecting
    // to the analogue pin, so double it to get the original value.
    // Then scale it based upon the ADC step value.
    float measuredvbat = ((float)(batteryReading * 200)) * ad_step;
    log_msg("measuredvbat adjusted: %f", measuredvbat);

    // Prepare uplink payload.
    memset(payloadBuffer, 0, sizeof(payloadBuffer));
    int i = 0;

    payloadBuffer[i++] = (avgDeg >>  8) & 0xff;
    payloadBuffer[i++] =  avgDeg        & 0xff;

    payloadBuffer[i++] = stdDev.bytes[3];
    payloadBuffer[i++] = stdDev.bytes[2];
    payloadBuffer[i++] = stdDev.bytes[1];
    payloadBuffer[i++] = stdDev.bytes[0];

    // Send -1 for wind count/speed in first message after reboot.
    // The raw count in the firmware is a uint32_t from the count register but the TTN decoder is looking for a
    // int16_t, so we can send a -1.

    // TODO: Need to add 2 more bytes to handle uint_32 value.
    payloadBuffer[i++] = (P >> 8) & 0xff;
    payloadBuffer[i++] = P & 0xff;

    if ( ! first_run) {
      payloadBuffer[i++] = windKph.bytes[3];
      payloadBuffer[i++] = windKph.bytes[2];
      payloadBuffer[i++] = windKph.bytes[1];
      payloadBuffer[i++] = windKph.bytes[0];
    } else {
      payloadBuffer[i++] = 0xbf;
      payloadBuffer[i++] = 0x80;
      payloadBuffer[i++] = 0x00;
      payloadBuffer[i++] = 0x00;
    }

    // The decoder is expecting the battery voltage * 100 to be encoded
    // in a uint8_t. This encoding only handles voltages up to about 3.6
    // before the uint8_t overflows.
    payloadBuffer[i++] = (uint8_t)(measuredvbat) - 127;

    first_run = false;
}

/*
 * This function is used to set the alarm to a relative time in the future, such as when
 * sleeping between LMIC tasks. It is not used for scheduling the sensor readings, which
 * are meant to happen at fixed times such as on the hour.
 */
void set_delta_alarm() {
    int32_t ss = rtc.getSeconds();
    int32_t mm = rtc.getMinutes();
    int32_t hh = rtc.getHours();

    // Adjust the sleep time to account for the 15 second direction measurement and the 6 second uplink/downlink cycle,
    // then -1 because the feather seems to wake up a second late.
    int32_t adjusted_delta = delta_seconds - 22;

    // Sanity check.
    if (adjusted_delta < 1) {
      adjusted_delta = 1;
    }

    int32_t delta = adjusted_delta;
    int32_t hh_delta = delta / 3600; delta -= (hh_delta * 3600);
    // Will always be less than 1 hour.
    int32_t mm_delta = delta / 60; delta -= (mm_delta * 60);
    // Will always be less than 1 minute.
    int32_t ss_delta = delta;

    ss += ss_delta;
    if (ss > 59) {
        ss = ss % 60;
        mm_delta++;
    }

    mm += mm_delta;
    if (mm > 59) {
        mm = mm % 60;
        hh_delta++;
    }

    hh = (hh + hh_delta) % 24;

    log_msg("Delta(s) = %d, wake at %02d:%02d:%02d", delta_seconds, hh, mm, ss);

    rtc.setAlarmTime((uint8_t)(hh & 0xff), (uint8_t)(mm & 0xff), (uint8_t)(ss & 0xff));
    rtc.enableAlarm(RTCZero::MATCH_HHMMSS);
}

void setup() {

#ifdef USE_SERIAL
    serial.begin(115200);
    log_msg("=== BOOT ===");

    uint8_t reset_cause = PM->RCAUSE.reg;
    if (reset_cause & PM_RCAUSE_POR) {
        log_msg("Power on reset");
    } else if (reset_cause & PM_RCAUSE_WDT) {
        log_msg("WDT reset");
    } else if (reset_cause & (PM_RCAUSE_BOD12 | PM_RCAUSE_BOD33)) {
        log_msg("BOD reset");
    } else if (reset_cause & PM_RCAUSE_SYST) {
        log_msg("System reset request");
    } else if (reset_cause & PM_RCAUSE_EXT) {
        log_msg("External reset");
    } else {
        log_msg("reset_cause = %ud", reset_cause);
    }

    log_msg("Kangaroo v%u.%u.%u", MAJOR, MINOR, REV);
    log_msg("DEVEUI: %02X %02X %02X %02X %02X %02X %02X %02X", DEVEUI[7], DEVEUI[6], DEVEUI[5], DEVEUI[4], DEVEUI[3], DEVEUI[2], DEVEUI[1], DEVEUI[0]);
#endif

    blink();

    // Now set up XOSC32K to run during standby, and feed the EIC from clock gen 1 (which is configured to use XOSC32K).
    // This allows the pin interrupts to be generated to wake from standby mode.
    SYSCTRL->XOSC32K.reg |=  (SYSCTRL_XOSC32K_RUNSTDBY | SYSCTRL_XOSC32K_ONDEMAND); // set external 32k oscillator to run when idle or sleep mode is chosen
    REG_GCLK_CLKCTRL  |= GCLK_CLKCTRL_ID(GCM_EIC) |  // generic clock multiplexer id for the external interrupt controller
                         GCLK_CLKCTRL_GEN_GCLK1 |    // generic clock 1 which is xosc32k
                         GCLK_CLKCTRL_CLKEN;         // enable it
    while (GCLK->STATUS.bit.SYNCBUSY);               // write protected, wait for sync

    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;   // Enable Standby or "deep sleep" mode

    pinMode(PUSH_BUTTON, INPUT_PULLDOWN);

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    // Deactivate LoRa radio CS to avoid SPI bus contention.
    pinMode(LORA_CS_PIN, OUTPUT);
    digitalWrite(LORA_CS_PIN, HIGH);

    rtc.begin(false);

    // Disable NVM power reduction during standby, errata 1.5.8.
    NVMCTRL->CTRLB.bit.SLEEPPRM = 3;

    // Fix for errata 12291 and also in case we're using too much current
    // in standby mode.
    SYSCTRL->VREG.bit.RUNSTDBY = 1;

    ad_step = AD_VOLTS / pow(2.0f, AD_BIT_SIZE);

    randomSeed(davis.getDirectionRaw());

    os_init();
    LMIC_reset();
    LMIC_startJoining();

    while ( ! joined) {
      if (LMIC.opmode != lmicOpmode) {
        lmicOpmode = LMIC.opmode;
        log_opmode();
      }

      os_runloop_once();
    }

    setup_GCLK();
    setup_EIC();
    setup_TC();
    setup_EVSYS();
}

void main_standby_sleep(void) {
#ifdef USE_SERIAL
    log_msg("Sleeping");
    serial.flush();
#endif

    // Lots of forum posts explaining why disabling IRQs before standby is a good idea,
    // and it somehow still allows the MCU to be woken with an interrupt.
    __disable_irq();

    // Yet another reason a SAMD21 might not come out of standby. Something about
    // the systick interrupt happening at an inconvenient time.
    // See https://www.avrfreaks.net/forum/samd21-samd21e16b-sporadically-locks-and-does-not-wake-standby-sleep-mode
    SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;

    rtc.standbyMode();

    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;

    __enable_irq();
}

void loop() {
    do_send();
    while (! LMIC_queryTxReady()) {
      os_runloop_once();
      if (LMIC.opmode != lmicOpmode) {
        lmicOpmode = LMIC.opmode;
        log_opmode();
      }
    }

    set_delta_alarm();
    main_standby_sleep();
    rtc.disableAlarm();
    log_msg("Awake");
}
