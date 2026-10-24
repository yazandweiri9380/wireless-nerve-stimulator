// ============================================================================
// Neuro Stimulator — BLE Peripheral
// SparkFun Artemis ATP + Intan RHS2116
// ============================================================================
#include <SPI.h>
#include <ArduinoBLE.h>
#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"
#include "am_hal_iom.h"
#include "am_hal_gpio.h"
#define NOP __asm__ volatile ("nop")

// ============================================================================
// Timing
// ============================================================================
#define ROBUST_TIMER_NUM 0
// CTIMER = 3 MHz -> 1 tick = 0.333333 us
// 100000 ticks = 33.333333 ms = exactly 30 Hz
const uint32_t CYCLE_PERIOD_ticks = 100000;

const uint32_t TICKS_PER_US = 3;

// One-time startup phase alignment.
// Target: normal BLE command arrivals occur nominally 12 ms after each
// stimulation-cycle boundary.
// 12 ms @ 3 MHz = 36000 ticks.

const uint32_t COMMAND_PHASE_ticks = 36000;

// The first runtime packet defines the phase reference. The first stimulation
// deadline is 33.333333 ms - 12 ms = 21.333333 ms = 64000 ticks later.

const uint32_t FIRST_PACKET_TO_CYCLE_ticks =

    CYCLE_PERIOD_ticks - COMMAND_PHASE_ticks;

// Width of EACH stimulation phase.
uint32_t STIM_PHASE_WIDTH_US = 25;
const uint32_t STIM_MIN_PHASE_US = 4;

// Electrical interval from CH10 end to CH15 start.
// 2250 ticks / 3 MHz = 750 us. With a 25 us + 25 us biphasic pulse,
// this gives an 800 us onset-to-onset interval.
const uint32_t INTERCHANNEL_DELAY_TICKS = 2250;

// BLE processing stops 5.2 ms before stimulation.
// 15600 ticks / 3 MHz = 5200 us.
const uint32_t BLE_GUARD_TICKS = 15600;

// If the cycle is already late, move the recovered trigger 100 us ahead.
const uint32_t OVERRUN_RECOVERY_TICKS = 300;

// ============================================================================
// Latency / output verification
// ============================================================================
const uint32_t Max_Rep = 60000;
uint32_t sample = 0;
uint8_t Received_Data[Max_Rep][2];

uint32_t overrunn = 0;
uint32_t t_end = 0;
uint32_t t_tsart = 0;

// ============================================================================
// RHS2116 SPI
//
// Artemis ATP:
// GPIO 8  = IOM1 SCK
// GPIO 9  = IOM1 MISO
// GPIO 10 = IOM1 MOSI
// GPIO 3  = manually controlled CS
// ============================================================================
#define CS_PIN   3
#define SDI_PIN  9
#define SDO_PIN  10
#define CLK_PIN  8
#define STIM_IOM 1

void *stimIOMHandle = nullptr;
IOM0_Type *stimIOM = nullptr;

// ===========================================================================
// FRAM SPI — remains on MbedSPI
// ============================================================================
#define chipSelectPinMem 25
#define mySDIMem         6
#define mySDOMem         7
#define myCLKMem         5
MbedSPI mySPIMem(mySDIMem, mySDOMem, myCLKMem);

// ============================================================================
// Unused pins
// ============================================================================
const uint8_t UnusedPins[24] = {
    0, 1, 2, 4, 14, 15, 17, 18, 19, 22, 23, 24,
    26, 28, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45};

// ============================================================================
// Main stimulation deadline
// ============================================================================
volatile uint32_t NextT = 0;

// DWT is the master clock for the actual 30 Hz stimulation onset.
// CTIMER NextT is retained for BLE guard timing and long-term diagnostics.
volatile uint32_t NextCycleDWT = 0;

uint32_t cyclePeriodCpuCycles = 0;
uint32_t firstPacketToCycleCpuCycles = 0;

// Preserve original intent: first stimulation after validation is zero.
bool firstStimCycle = true;

// ============================================================================
// BLE service and characteristics
// ============================================================================
BLEService NeuroService("19C10000-E8F2-537E-4F6C-D104768A1214");
BLECharacteristic StimRXChar(
    "19C10031-E8F2-537E-4F6C-D104768A1214",
    BLEWriteWithoutResponse | BLEWrite,
    3
);

BLECharacteristic MirrorChar(
    "19C10033-E8F2-537E-4F6C-D104768A1214",
    BLERead,
    2
);

// ============================================================================
// Startup-validation staging
//
// These variables are now used only before runtime FIFO operation begins.
// ============================================================================
volatile uint8_t stagedAmpCH10 = 0;
volatile uint8_t stagedAmpCH15 = 0;
volatile bool stagedValid = false;
uint8_t activeAmpCH10 = 0;
uint8_t activeAmpCH15 = 0;

// ============================================================================
// Runtime two-entry FIFO
// ============================================================================
struct StimSample {
    uint8_t ch10;
    uint8_t ch15;
};

const uint8_t STIM_FIFO_SIZE = 2;
StimSample stimFIFO[STIM_FIFO_SIZE];
uint8_t fifoReadIndex = 0;
uint8_t fifoWriteIndex = 0;
uint8_t fifoCount = 0;

// Diagnostics
uint32_t fifoOverflows = 0;
uint32_t fifoUnderruns = 0;

// False during startup validation.
// True once validation is complete and the peripheral is armed for runtime.
bool runtimeFIFOEnabled = false;

// ============================================================================
// Fault / reliability state
// ============================================================================
volatile uint8_t badChecksumConsecutive = 0;
volatile bool faultLatched = false;

// ============================================================================
// Startup validation
// ============================================================================
volatile uint8_t validPacketCount = 0;
const uint8_t VALIDATION_PACKET_TARGET = 5;

// READY marker returned through the existing 2-byte mirror characteristic.
// Both values are outside the valid amplitude range (0...100).
const uint8_t READY_MARKER_0 = 0xA5;
const uint8_t READY_MARKER_1 = 0x5A;

// One-time phase anchor captured when the first valid runtime packet arrives.
volatile bool firstRuntimePacketSeen = false;
volatile uint32_t firstRuntimePacketTick = 0;
volatile uint32_t firstRuntimePacketCycle = 0;

// ============================================================================
// State machine
// ============================================================================
enum RunState : uint8_t {
    WAIT_FOR_CENTRAL = 0,
    GRACE_AFTER_CONNECT,
    INIT_SPI_AND_CONFIGURE_DEVICE,
    VALIDATION_PHASE,
    WAIT_FOR_FIRST_RUNTIME_PACKET,
    RUN_33MS_CYCLE,
    FAULT_DEEP_SLEEP
};

RunState runState = WAIT_FOR_CENTRAL;
BLEDevice activeCentral;

uint32_t graceStartMs = 0;
const uint32_t CONNECT_GRACE_MS = 1500;
bool spiInitialized = false;
bool deviceConfigured = false;

// ============================================================================
// Forward declarations
// ============================================================================
void initBLE();
void resetSessionState();
void handleDisconnect();
void initStimSPI();
void configureStimDeviceOnce();
void stopSPIBus();
void enterFaultDeepSleep();
void updateMirror(uint8_t ch10, uint8_t ch15);
void onStimWrite(BLEDevice central, BLECharacteristic characteristic);
void SendWriteCommand(uint8_t regnum, uint16_t data);
void SendWriteCommandU(uint8_t regnum, uint16_t data);
void SendReadCommand(uint8_t regnum);
void SendReadCommandM(uint8_t regnum);
void SendConvertCommand(uint8_t channelnum);
void SendClearCommand();
void StopCurrent();
void initRobustMicros();

uint32_t stimulateAtDWT(uint8_t channel, uint8_t strength,
                        uint32_t phaseWidth_us, uint32_t bTargetCycle);

// ============================================================================
// General helpers
// ============================================================================
uint8_t xorChecksum(uint8_t a, uint8_t b) {
    return a ^ b;
}

uint8_t clampAmplitude(uint8_t raw) {
    return (raw > 100) ? 100 : raw;
}

// ============================================================================
// Two-entry FIFO helpers
// ============================================================================
void clearStimFIFO() {
    fifoReadIndex = 0;
    fifoWriteIndex = 0;
    fifoCount = 0;
}

// Add one valid BLE amplitude pair.
//
// The FIFO always keeps the two FRESHEST pending commands.
//
// If the FIFO is already full:
// - discard the OLDEST queued command
// - keep the newer queued command
// - insert the newly arrived command
//

// fifoOverflows is still incremented so we can measure how often this occurs.
void enqueueStimSample(uint8_t ch10, uint8_t ch15) {
    if (fifoCount >= STIM_FIFO_SIZE) {
        fifoOverflows++;
        // Discard the oldest queued sample.
        fifoReadIndex++;
        if (fifoReadIndex >= STIM_FIFO_SIZE) {
            fifoReadIndex = 0;
        }
        fifoCount--;
    }
    // Store the newest received sample.
    stimFIFO[fifoWriteIndex].ch10 = ch10;
    stimFIFO[fifoWriteIndex].ch15 = ch15;
    fifoWriteIndex++;
    if (fifoWriteIndex >= STIM_FIFO_SIZE) {
        fifoWriteIndex = 0;
    }
    fifoCount++;
}

// Consume exactly one FIFO sample.
//
// If empty:
// - count an underrun
// - leave activeAmpCH10 / activeAmpCH15 unchanged
//
// Therefore the previous stimulation amplitude is repeated for that cycle.
void loadNextStimSample() {
    if (fifoCount == 0) {
        fifoUnderruns++;
        return;
    }
    activeAmpCH10 = stimFIFO[fifoReadIndex].ch10;
    activeAmpCH15 = stimFIFO[fifoReadIndex].ch15;
    fifoReadIndex++;
    if (fifoReadIndex >= STIM_FIFO_SIZE) {
        fifoReadIndex = 0;
    }
    fifoCount--;
}

// ============================================================================
// 3 MHz hardware timing
// ============================================================================
uint32_t robustTicks()
{
    uint32_t high1, high2, low;
    do {
        high1 = am_hal_ctimer_read(
            ROBUST_TIMER_NUM,
            AM_HAL_CTIMER_TIMERB
        );
        low = am_hal_ctimer_read(
            ROBUST_TIMER_NUM,
            AM_HAL_CTIMER_TIMERA
        );
        high2 = am_hal_ctimer_read(
            ROBUST_TIMER_NUM,
            AM_HAL_CTIMER_TIMERB
        );
    } while ((high1 & 0xFFFFu) !=
             (high2 & 0xFFFFu));
    return ((high2 & 0xFFFFu) << 16) |
            (low & 0xFFFFu);
}

// ============================================================================
// Native Apollo3 RHS2116 SPI
// ============================================================================
// Apollo FIFOPUSH transmits the least-significant BYTE first.
static inline uint32_t reverseBytes32(uint32_t x) {
    return ((x & 0x000000FFu) << 24) |
           ((x & 0x0000FF00u) << 8)  |
           ((x & 0x00FF0000u) >> 8)  |
           ((x & 0xFF000000u) >> 24);
}

// Construct an RHS2116 WRITE packet.
//
// update = false -> command 0x80
// update = true  -> command 0xA0
static inline uint32_t rhsWriteWord(bool update, uint8_t regnum, uint16_t data) {
    uint32_t commandByte = update ? 0xA0u : 0x80u;
    return (commandByte << 24) |
           ((uint32_t)regnum << 16) |
           (uint32_t)data;
}

// Load one complete RHS word into the Apollo TX FIFO.
static inline void rhsPrepare(uint32_t command) {
    stimIOM->INTCLR = 1u;
    stimIOM->FIFOPUSH = reverseBytes32(command);
}

// Launch one separate 32-bit RHS2116 SPI frame:
//
// CS LOW -> 32 clocks -> CS HIGH
static inline uint32_t rhsLaunch() {
    am_hal_gpio_output_clear(CS_PIN);

    // TSIZE = 4 bytes, CMD = WRITE
    stimIOM->CMD = (4u << 8) | 1u;
    while ((stimIOM->INTSTAT & 1u) == 0) {
        NOP;
    }
    am_hal_gpio_output_set(CS_PIN);
    uint32_t rxWord = stimIOM->FIFOPOP;
    stimIOM->INTCLR = 1u;
    return reverseBytes32(rxWord);
}

// Immediate native RHS transaction.
static inline uint32_t rhsSend(uint32_t command) {
    rhsPrepare(command);
    return rhsLaunch();
}

// ============================================================================
// Cortex-M4 DWT cycle counter for short, phase-relative stimulation timing
//
// DWT->CYCCNT is now the master clock for the actual 30 Hz stimulation onset,
// the pulse-1 -> pulse-2 spacing, and the B->C / C->D phase timing.
//
// The 3 MHz CTIMER is retained for the BLE guard window and long-term
// diagnostics, but it no longer determines the actual pulse onset.
// ============================================================================
void initDWTCycleCounter() {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static inline uint32_t microsecondsToCpuCycles(uint32_t us) {
    return us * (SystemCoreClock / 1000000u);
}

// Convert 3 MHz CTIMER ticks to DWT CPU cycles.
static inline uint32_t timerTicksToCpuCycles(uint32_t ticks) {
    return ticks * (SystemCoreClock / 3000000u);
}

static inline void waitUntilCpuCycle(uint32_t targetCycle) {
    while ((int32_t)(DWT->CYCCNT - targetCycle) < 0) {
        NOP;
    }
}

// Launch one RHS2116 SPI frame and capture the DWT timestamp immediately
// after asserting CS low.
static inline uint32_t rhsLaunchStamped(uint32_t *csLowCycle) {
    am_hal_gpio_output_clear(CS_PIN);
    uint32_t stamp = DWT->CYCCNT;
    if (csLowCycle != nullptr) {
        *csLowCycle = stamp;
    }
    stimIOM->CMD = (4u << 8) | 1u;
    while ((stimIOM->INTSTAT & 1u) == 0) {
        NOP;
    }
    am_hal_gpio_output_set(CS_PIN);
    uint32_t rxWord = stimIOM->FIFOPOP;
    stimIOM->INTCLR = 1u;
    return reverseBytes32(rxWord);
}

// ============================================================================
// DWT-scheduled biphasic stimulation
//
// bTargetCycle:
//     absolute DWT cycle at which the pulse begins
//
// Returns:
//     actual DWT cycle captured at the B transaction launch
// ============================================================================
uint32_t stimulateAtDWT(uint8_t channel, uint8_t strength,
                        uint32_t phaseWidth_us, uint32_t bTargetCycle) {
    if (channel > 15) {
        return 0;
    }
    if (phaseWidth_us < STIM_MIN_PHASE_US) {
        phaseWidth_us = STIM_MIN_PHASE_US;
    }
    const uint32_t phaseCycles = microsecondsToCpuCycles(phaseWidth_us);
    const uint32_t irqGuardCycles = microsecondsToCpuCycles(100u);
    uint8_t negativeMagnitudeRegister = 64 + channel;
    uint8_t positiveMagnitudeRegister = 96 + channel;
    uint16_t magnitude = 0x8000 + strength;

    // Do all non-time-critical SPI work early while output is off.
    SendWriteCommand(negativeMagnitudeRegister, magnitude);
    SendWriteCommand(positiveMagnitudeRegister, magnitude);
    uint32_t turnOn = rhsWriteWord(true, 42, (uint16_t)(1u << channel));
    uint32_t positivePolarity = rhsWriteWord(true, 44, 0xFFFF);
    uint32_t turnOff = rhsWriteWord(true, 42, 0x0000);
    uint32_t restoreNegative = rhsWriteWord(false, 44, 0x0000);

    // A can be sent early because no RHS transaction occurs between A and B.
    rhsSend(turnOn);

    // Leave IRQs enabled for most of the inter-pulse interval.
    uint32_t guardStartCycle = bTargetCycle - irqGuardCycles;
    waitUntilCpuCycle(guardStartCycle);

    // Mask IRQs only for the last 100 us before pulse 2 and the pulse itself.
    uint32_t previousPrimask = __get_PRIMASK();
    __disable_irq();
    rhsPrepare(positivePolarity);
    waitUntilCpuCycle(bTargetCycle);
    uint32_t bStartCycle = 0;
    rhsLaunchStamped(&bStartCycle);
    rhsPrepare(turnOff);
    uint32_t cTargetCycle = bStartCycle + phaseCycles;
    waitUntilCpuCycle(cTargetCycle);
    uint32_t cStartCycle = 0;
    rhsLaunchStamped(&cStartCycle);
    rhsPrepare(restoreNegative);
    uint32_t dTargetCycle = cStartCycle + phaseCycles;
    waitUntilCpuCycle(dTargetCycle);
    rhsLaunchStamped(nullptr);
    if (previousPrimask == 0u) {
        __enable_irq();
    }
    return bStartCycle;
}

// ============================================================================
// setup()
// ============================================================================
void setup() {
    for (int x = 0; x < 24; x++) {
        am_hal_gpio_pinconfig(UnusedPins[x], g_AM_HAL_GPIO_DISABLE);
    }
    for (uint16_t i = 0; i < Max_Rep; i++) {
        Received_Data[i][0] = 121;
        Received_Data[i][1] = 131;
    }
    initBLE();
    initRobustMicros();

    // DWT is the master clock for all stimulation timing.
    initDWTCycleCounter();

    // At 48 MHz this is exactly 1,600,000 cycles per 30 Hz period.
    cyclePeriodCpuCycles = SystemCoreClock / 30u;

    // Preserve the existing 12 ms startup phase relationship.
    firstPacketToCycleCpuCycles =
        cyclePeriodCpuCycles -
        timerTicksToCpuCycles(COMMAND_PHASE_ticks);

    // Serial.begin(115200);
}

// ============================================================================
// loop()
// ============================================================================
void loop() {
    while (sample < Max_Rep) {
        switch (runState) {

        // --------------------------------------------------------------------
        // Wait for BLE central.
        // --------------------------------------------------------------------
        case WAIT_FOR_CENTRAL: {
            BLEDevice central = BLE.central();
            if (central) {
                activeCentral = central;
                graceStartMs = millis();
                runState = GRACE_AFTER_CONNECT;
            }
            break;
        }

        // --------------------------------------------------------------------
        // BLE service-discovery grace period.
        // --------------------------------------------------------------------
        case GRACE_AFTER_CONNECT: {
            if (!activeCentral.connected()) {
                handleDisconnect();
                break;
            }
            if ((millis() - graceStartMs) >= CONNECT_GRACE_MS) {
                runState = INIT_SPI_AND_CONFIGURE_DEVICE;
            }
            break;
        }

        // --------------------------------------------------------------------
        // Initialize FRAM / RHS2116.
        // --------------------------------------------------------------------
        case INIT_SPI_AND_CONFIGURE_DEVICE: {
            if (!activeCentral.connected()) {
                handleDisconnect();
                break;
            }
            if (!spiInitialized) {
                initStimSPI();
            }
            if (!deviceConfigured) {
                configureStimDeviceOnce();
            }
            runState = VALIDATION_PHASE;
            validPacketCount = 0;
            break;
        }

        // --------------------------------------------------------------------
        // Startup BLE validation.
        //
        // This section retains the existing staging mechanism.
        // Runtime FIFO is not yet active.
        // --------------------------------------------------------------------
        case VALIDATION_PHASE: {
            if (!activeCentral.connected()) {
                handleDisconnect();
                break;
            }
            __disable_irq();
            bool packetReady = stagedValid;
            uint8_t packetCH10 = stagedAmpCH10;
            uint8_t packetCH15 = stagedAmpCH15;

            if (packetReady) {
            stagedValid = false;
            }

            __enable_irq();

            if (packetReady) {
                activeAmpCH10 = packetCH10;
                activeAmpCH15 = packetCH15;
                updateMirror(activeAmpCH10, activeAmpCH15);
                validPacketCount++;
                if (validPacketCount >= VALIDATION_PACKET_TARGET) {

                    // The fifth validation mirror has already been updated above.
                    // Keep servicing BLE during the existing 500 ms startup delay so
                    // the host can complete its fifth mirror read before READY replaces
                    // the mirror value.
                    uint32_t initWaitEnd = robustTicks() + 1500000;  // 500 ms
                    while ((int32_t)(initWaitEnd - robustTicks()) > 0) {
                        BLE.poll();
                    }

                    // Validation staging is no longer needed.
                    stagedValid = false;

                    // Start normal operation with a completely empty FIFO.
                    clearStimFIFO();
                    runtimeFIFOEnabled = true;
                    fifoOverflows = 0;
                    fifoUnderruns = 0;

                    // Preserve the existing intentionally-zero first stimulation cycle.
                    activeAmpCH10 = 0;
                    activeAmpCH15 = 0;
                    firstStimCycle = true;

                    // Arm the one-time phase capture. Do NOT establish NextT yet.
                    firstRuntimePacketSeen = false;
                    firstRuntimePacketTick = 0;
                    firstRuntimePacketCycle = 0;
                    NextCycleDWT = 0;

                    // Tell the host that the peripheral is fully initialized and
                    // waiting for the first normal runtime packet.
                    updateMirror(READY_MARKER_0, READY_MARKER_1);
                    runState = WAIT_FOR_FIRST_RUNTIME_PACKET;
                }
            }
            break;
        }

        // --------------------------------------------------------------------
        // Armed state for one-time startup phase alignment.
        //
        // The first valid runtime packet is timestamped in onStimWrite(), queued
        // normally, and used only to establish the stimulation-cycle phase.
        // --------------------------------------------------------------------
        case WAIT_FOR_FIRST_RUNTIME_PACKET: {
            if (!activeCentral.connected()) {
                handleDisconnect();
                break;
            }

            // Service BLE until the first normal runtime packet arrives.
            BLE.poll();
            if (firstRuntimePacketSeen) {

                // The first packet arrival is defined to be 12 ms after the
                // preceding conceptual cycle boundary. Therefore the first actual
                // stimulation deadline occurs 21.333333 ms (64000 ticks) later.
                // CTIMER schedule retained for BLE guard timing.
                NextT = firstRuntimePacketTick + FIRST_PACKET_TO_CYCLE_ticks;

                // DWT schedule is the master for actual pulse-1 onset.
                NextCycleDWT =
                    firstRuntimePacketCycle + firstPacketToCycleCpuCycles;

                // Keep the duration diagnostic comparable to the original run.
                // The conceptual preceding cycle boundary is 12 ms before the
                // first runtime packet, so back-date the millisecond diagnostic
                // by COMMAND_PHASE_ticks / 3000 = 12 ms.
                t_tsart = millis() - (COMMAND_PHASE_ticks / (TICKS_PER_US * 1000u));
                runState = RUN_33MS_CYCLE;
            }
            break;
        }

        // --------------------------------------------------------------------
        // Main 33.333333 ms cycle.
        // --------------------------------------------------------------------
        case RUN_33MS_CYCLE: {

            // ================================================================
            // BLE-active interval
            //
            // Allow BLE processing until 5.2 ms before stimulation.
            // Valid runtime packets are now placed into the 2-entry FIFO.
            // ================================================================
            uint32_t bleStopTime = NextT - BLE_GUARD_TICKS;
            while ((int32_t)(bleStopTime - robustTicks()) > 0) {
                BLE.poll();
            }

            // ----------------------------------------------------------------
            // Safety checks before entering the timing-critical region.
            // ----------------------------------------------------------------
            if (!activeCentral.connected()) {
                handleDisconnect();
                break;
            }
            if (badChecksumConsecutive >= 5) {
                runState = FAULT_DEEP_SLEEP;
                break;
            }

            // ================================================================
            // Consume exactly ONE FIFO entry per stimulation cycle.
            //
            // The initial intentionally-zero cycle does not consume an entry.
            //
            // If FIFO is empty on later cycles:
            //     activeAmp remains unchanged
            //     previous stimulation value is repeated
            //     fifoUnderruns increments
            // ================================================================
            if (!firstStimCycle) {
                loadNextStimSample();
            }

            // Magnitudes that will actually be applied in this cycle.
            uint8_t stimCH10 = firstStimCycle ? 0 : activeAmpCH10;
            uint8_t stimCH15 = firstStimCycle ? 0 : activeAmpCH15;

            // ================================================================
            // Timing-critical section begins.
            //
            // Scheduler remains locked through pulse 1, the inter-pulse interval,
            // and pulse 2. Hardware IRQs are masked only during the final
            // 100 us before each B launch and the short pulse.
            // ================================================================
            osKernelLock();

            // ----------------------------------------------------------------
            // Detect a missed stimulation deadline.
            // ----------------------------------------------------------------
            // DWT owns the actual stimulation deadline. If this code arrives
            // after the target, recover both timing domains together.
            uint32_t nowCycle = DWT->CYCCNT;
            if ((int32_t)(nowCycle - NextCycleDWT) > 0) {
                overrunn++;

                // Put the recovered pulse 100 us into the future.
                NextCycleDWT =
                    nowCycle + microsecondsToCpuCycles(100u);

                // Keep the CTIMER BLE-guard schedule aligned with the recovery.
                NextT = robustTicks() + OVERRUN_RECOVERY_TICKS;
            }

(void)stimulateAtDWT(10, stimCH10, 5,  NextCycleDWT);
(void)stimulateAtDWT(10, stimCH10, 10, NextCycleDWT);
(void)stimulateAtDWT(10, stimCH10, 20, NextCycleDWT);
(void)stimulateAtDWT(10, stimCH10, 30, NextCycleDWT);

(void)stimulateAtDWT(10, stimCH10, 5,  NextCycleDWT);
(void)stimulateAtDWT(10, stimCH10, 10, NextCycleDWT);
(void)stimulateAtDWT(10, stimCH10, 20, NextCycleDWT);
(void)stimulateAtDWT(10, stimCH10, 30, NextCycleDWT);

(void)stimulateAtDWT(10, stimCH10, 5,  NextCycleDWT);
(void)stimulateAtDWT(10, stimCH10, 10, NextCycleDWT);
(void)stimulateAtDWT(10, stimCH10, 20, NextCycleDWT);
(void)stimulateAtDWT(10, stimCH10, 30, NextCycleDWT);

(void)stimulateAtDWT(10, stimCH10, 5,  NextCycleDWT);
(void)stimulateAtDWT(10, stimCH10, 10, NextCycleDWT);
(void)stimulateAtDWT(10, stimCH10, 20, NextCycleDWT);
(void)stimulateAtDWT(10, stimCH10, 30, NextCycleDWT);

(void)stimulateAtDWT(10, stimCH10, 5,  NextCycleDWT);
(void)stimulateAtDWT(10, stimCH10, 10, NextCycleDWT);
(void)stimulateAtDWT(10, stimCH10, 20, NextCycleDWT);
(void)stimulateAtDWT(10, stimCH10, 30, NextCycleDWT);

(void)stimulateAtDWT(10, stimCH10, 5,  NextCycleDWT);
(void)stimulateAtDWT(10, stimCH10, 10, NextCycleDWT);
(void)stimulateAtDWT(10, stimCH10, 20, NextCycleDWT);
(void)stimulateAtDWT(10, stimCH10, 30, NextCycleDWT);

(void)stimulateAtDWT(10, stimCH10, 5,  NextCycleDWT);
(void)stimulateAtDWT(10, stimCH10, 10, NextCycleDWT);
(void)stimulateAtDWT(10, stimCH10, 20, NextCycleDWT);
(void)stimulateAtDWT(10, stimCH10, 30, NextCycleDWT);

(void)stimulateAtDWT(10, stimCH10, 5,  NextCycleDWT);
(void)stimulateAtDWT(10, stimCH10, 10, NextCycleDWT);
(void)stimulateAtDWT(10, stimCH10, 20, NextCycleDWT);
(void)stimulateAtDWT(10, stimCH10, 30, NextCycleDWT);

(void)stimulateAtDWT(10, stimCH10, 5,  NextCycleDWT);
(void)stimulateAtDWT(10, stimCH10, 10, NextCycleDWT);
(void)stimulateAtDWT(10, stimCH10, 20, NextCycleDWT);
(void)stimulateAtDWT(10, stimCH10, 30, NextCycleDWT);

(void)stimulateAtDWT(10, stimCH10, 5,  NextCycleDWT);
(void)stimulateAtDWT(10, stimCH10, 10, NextCycleDWT);
(void)stimulateAtDWT(10, stimCH10, 20, NextCycleDWT);
(void)stimulateAtDWT(10, stimCH10, 30, NextCycleDWT);

(void)stimulateAtDWT(10, stimCH10, 5,  NextCycleDWT);
(void)stimulateAtDWT(10, stimCH10, 10, NextCycleDWT);
(void)stimulateAtDWT(10, stimCH10, 20, NextCycleDWT);
(void)stimulateAtDWT(10, stimCH10, 30, NextCycleDWT);

(void)stimulateAtDWT(10, stimCH10, 5,  NextCycleDWT);
(void)stimulateAtDWT(10, stimCH10, 10, NextCycleDWT);
(void)stimulateAtDWT(10, stimCH10, 20, NextCycleDWT);
(void)stimulateAtDWT(10, stimCH10, 30, NextCycleDWT);

(void)stimulateAtDWT(10, stimCH10, 5,  NextCycleDWT);
(void)stimulateAtDWT(10, stimCH10, 10, NextCycleDWT);
(void)stimulateAtDWT(10, stimCH10, 20, NextCycleDWT);
(void)stimulateAtDWT(10, stimCH10, 30, NextCycleDWT);

(void)stimulateAtDWT(10, stimCH10, 5,  NextCycleDWT);
(void)stimulateAtDWT(10, stimCH10, 10, NextCycleDWT);
(void)stimulateAtDWT(10, stimCH10, 20, NextCycleDWT);
(void)stimulateAtDWT(10, stimCH10, 30, NextCycleDWT);

(void)stimulateAtDWT(10, stimCH10, 5,  NextCycleDWT);
(void)stimulateAtDWT(10, stimCH10, 10, NextCycleDWT);
(void)stimulateAtDWT(10, stimCH10, 20, NextCycleDWT);
(void)stimulateAtDWT(10, stimCH10, 30, NextCycleDWT);

            // Both stimulation events are complete.
            osKernelUnlock();
            firstStimCycle = false;

            // ----------------------------------------------------------------
            // Record the actual values used for this stimulation cycle.
            // ----------------------------------------------------------------
            Received_Data[sample][0] = stimCH10;
            Received_Data[sample][1] = stimCH15;
            sample++;

            // ----------------------------------------------------------------
            // Schedule the next 30-Hz stimulation event.
            // ----------------------------------------------------------------
            // Absolute phase accumulators: do not derive the next cycle from
            // the actual launch time. This prevents accumulated drift.
            NextCycleDWT += cyclePeriodCpuCycles;
            NextT += CYCLE_PERIOD_ticks;
            break;
        }

        // --------------------------------------------------------------------
        case FAULT_DEEP_SLEEP: {
            enterFaultDeepSleep();
            break;
        }
        }
    }

    // ========================================================================
    // Measurement run complete
    // ========================================================================
    t_end = millis();
    Serial.begin(115200);
    for (int p = 0; p < Max_Rep; p++) {
        Serial.print(Received_Data[p][0]);
        Serial.print(" , ");
        Serial.println(Received_Data[p][1]);
    }
    Serial.print("Time Duration = ");
    Serial.println(uint32_t(t_end - t_tsart));
    Serial.print("OverRuns = ");
    Serial.println(overrunn);
    Serial.print("FIFO Overflows = ");
    Serial.println(fifoOverflows);
    Serial.print("FIFO Underruns = ");
    Serial.println(fifoUnderruns);
    Serial.end();
    enterFaultDeepSleep();
    while (1) {
    }
}

// ============================================================================
// RHS2116 helper functions
// ============================================================================
// WRITE, U = 0
void SendWriteCommand(uint8_t regnum, uint16_t data) {
    rhsSend(
        0x80000000u |
        ((uint32_t)regnum << 16) |
        data
    );
}

// WRITE, U = 1
void SendWriteCommandU(uint8_t regnum, uint16_t data) {
    rhsSend(
        0xA0000000u |
        ((uint32_t)regnum << 16) |
        data
    );
}

void StopCurrent() {
    rhsSend(0xA02A0000u);
}

void SendReadCommand(uint8_t regnum) {
    rhsSend(
        0xC0000000u |
        ((uint32_t)regnum << 16)
    );
}

void SendReadCommandM(uint8_t regnum) {
    rhsSend(
        0xD0000000u |
        ((uint32_t)regnum << 16)
    );
}

void SendConvertCommand(uint8_t channelnum) {
    rhsSend((uint32_t)channelnum << 16);
}

void SendClearCommand() {
    rhsSend(0x6A000000u);
}

// ============================================================================
// FRAM helpers
// ============================================================================
void SaveMem(uint16_t RegAdrs, uint8_t dataTosave) {
    am_hal_gpio_output_clear(chipSelectPinMem);
    mySPIMem.transfer(0b00000010);
    mySPIMem.transfer16(RegAdrs);
    mySPIMem.transfer(dataTosave);
    am_hal_gpio_output_set(chipSelectPinMem);
}

uint8_t ReadMem(uint16_t RegAdrs) {
    uint8_t out = 0;
    am_hal_gpio_output_clear(chipSelectPinMem);
    mySPIMem.transfer(0b00000011);
    mySPIMem.transfer16(RegAdrs);
    out = mySPIMem.transfer(0xFF);
    am_hal_gpio_output_set(chipSelectPinMem);
    return out;
}

// ============================================================================
// BLE initialization
// ============================================================================
void initBLE() {
    BLE.begin();
    BLE.setLocalName("Neuro");
    BLE.setAdvertisedService(NeuroService);
    NeuroService.addCharacteristic(StimRXChar);
    NeuroService.addCharacteristic(MirrorChar);
    BLE.addService(NeuroService);
    StimRXChar.setEventHandler(BLEWritten, onStimWrite);
    updateMirror(0, 0);
    BLE.advertise();
}

// ============================================================================
// Reset session
// ============================================================================
void resetSessionState() {

    // Startup staging
    stagedAmpCH10 = 0;
    stagedAmpCH15 = 0;
    stagedValid = false;
    activeAmpCH10 = 0;
    activeAmpCH15 = 0;

    // Runtime FIFO
    clearStimFIFO();
    runtimeFIFOEnabled = false;
    fifoOverflows = 0;
    fifoUnderruns = 0;

    // Reliability
    badChecksumConsecutive = 0;
    faultLatched = false;

    // Validation
    validPacketCount = 0;
    firstStimCycle = true;
    firstRuntimePacketSeen = false;
    firstRuntimePacketTick = 0;
    firstRuntimePacketCycle = 0;
    NextCycleDWT = 0;
    updateMirror(0, 0);
    stopSPIBus();
    deviceConfigured = false;
}

// ============================================================================
// Disconnect handler
// ============================================================================
void handleDisconnect() {
    resetSessionState();
    runState = WAIT_FOR_CENTRAL;
    BLE.advertise();
}

// ============================================================================
// SPI initialization
// ============================================================================
void initStimSPI() {

    // ========================================================================
    // FRAM initialization
    // ========================================================================
    pinMode(chipSelectPinMem, OUTPUT);
    digitalWrite(chipSelectPinMem, HIGH);
    mySPIMem.begin();
    mySPIMem.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
    delay(250);

    // Write enable
    digitalWrite(chipSelectPinMem, LOW);
    mySPIMem.transfer(0b00000110);
    digitalWrite(chipSelectPinMem, HIGH);
    delayMicroseconds(10);
    digitalWrite(chipSelectPinMem, LOW);
    mySPIMem.transfer(0b10011111);
    digitalWrite(chipSelectPinMem, HIGH);
    delayMicroseconds(10);
    digitalWrite(chipSelectPinMem, LOW);
    digitalWrite(chipSelectPinMem, HIGH);

    // ========================================================================
    // Loading the inverse model parameters here.
    // ========================================================================

    
    // Put memory into sleep.
    digitalWrite(chipSelectPinMem, LOW);
    mySPIMem.transfer(0b10111001);
    digitalWrite(chipSelectPinMem, HIGH);
    delayMicroseconds(10);
    mySPIMem.end();
    delay(5);

    // ========================================================================
    // RHS2116 native Apollo3 IOM1
    // ========================================================================
    // Manual CS
    am_hal_gpio_pinconfig(CS_PIN, g_AM_HAL_GPIO_OUTPUT);
    am_hal_gpio_output_set(CS_PIN);

    // GPIO8 = IOM1 SCK
    am_hal_gpio_pincfg_t sckCfg = g_AM_HAL_GPIO_OUTPUT;
    sckCfg.uFuncSel = AM_HAL_PIN_8_M1SCK;
    am_hal_gpio_pinconfig(CLK_PIN, sckCfg);

    // GPIO10 = IOM1 MOSI
    am_hal_gpio_pincfg_t mosiCfg = g_AM_HAL_GPIO_OUTPUT;
    mosiCfg.uFuncSel = AM_HAL_PIN_10_M1MOSI;
    am_hal_gpio_pinconfig(SDO_PIN, mosiCfg);

    // GPIO9 = IOM1 MISO
    am_hal_gpio_pincfg_t misoCfg = g_AM_HAL_GPIO_INPUT;
    misoCfg.uFuncSel = AM_HAL_PIN_9_M1MISO;
    am_hal_gpio_pinconfig(SDI_PIN, misoCfg);

    // IOM1 configuration
    am_hal_iom_config_t iomConfig = {};
    iomConfig.eInterfaceMode = AM_HAL_IOM_SPI_MODE;
    iomConfig.ui32ClockFreq = AM_HAL_IOM_12MHZ;
    iomConfig.eSpiMode = AM_HAL_IOM_SPI_MODE_0;
    iomConfig.pNBTxnBuf = nullptr;
    iomConfig.ui32NBTxnBufLength = 0;
    am_hal_iom_initialize(STIM_IOM, &stimIOMHandle);
    am_hal_iom_power_ctrl(stimIOMHandle, AM_HAL_SYSCTRL_WAKE, false);
    am_hal_iom_configure(stimIOMHandle, &iomConfig);
    am_hal_iom_enable(stimIOMHandle);
    stimIOM = IOMn(STIM_IOM);

    // Full duplex
    stimIOM->MSPICFG |= (1u << 2);

    // Reset FIFOs
    stimIOM->FIFOCTRL = 0;
    stimIOM->FIFOCTRL = (1u << 1);

    // Clear stale interrupt flags
    stimIOM->INTCLR = 0xFFFFFFFFu;
    spiInitialized = true;
}

// ============================================================================
// One-time RHS2116 configuration
// ============================================================================
void configureStimDeviceOnce() {
    if (!spiInitialized) {
        return;
    }
    delay(50);
    SendReadCommand(62);
    SendReadCommand(62);
    delay(50);
    SendReadCommand(255);
    SendWriteCommand(32, 0x0000);
    SendWriteCommand(33, 0x0000);
    SendWriteCommand(38, 0xFFFF);
    SendClearCommand();

    // Core configuration registers
    SendWriteCommand(0, 0x00C7);
    SendWriteCommand(1, 0x051A);
    SendWriteCommand(2, 0x0040);
    SendWriteCommand(3, 0x0080);
    SendWriteCommand(4, 0x0016);
    SendWriteCommand(5, 0x0017);
    SendWriteCommand(6, 0x00A8);
    SendWriteCommand(7, 0x000A);
    SendWriteCommandU(10, 0x0000);
    SendWriteCommandU(12, 0xFFFF);
    SendWriteCommand(8, 0x0000);
    SendWriteCommand(34, 0x000F);
    SendWriteCommand(35, 0x00FF);
    SendWriteCommand(36, 0x0080);
    SendWriteCommand(37, 0x00B8);
    SendWriteCommandU(42, 0x0000);
    SendWriteCommandU(44, 0xFFFF);
    SendWriteCommandU(46, 0x0000);
    SendWriteCommandU(48, 0x0000);

    // Channel amplitude registers
    for (uint8_t reg = 64; reg <= 79; reg++) {
        SendWriteCommandU(reg, 0x8000);
    }
    for (uint8_t reg = 96; reg <= 111; reg++) {
        SendWriteCommandU(reg, 0x8000);
    }
    SendWriteCommand(32, 0xAAAA);
    SendWriteCommand(33, 0x00FF);
    SendReadCommandM(255);
    deviceConfigured = true;
}

// ============================================================================
// Stop RHS2116 SPI
// ============================================================================
void stopSPIBus() {
    if (!spiInitialized) {
        return;
    }
    am_hal_gpio_output_set(CS_PIN);
    if (stimIOMHandle != nullptr) {
        am_hal_iom_disable(stimIOMHandle);
        am_hal_iom_uninitialize(stimIOMHandle);
        stimIOMHandle = nullptr;
        stimIOM = nullptr;
    }
    spiInitialized = false;
}

// ============================================================================
// Fault deep sleep
// ============================================================================
void enterFaultDeepSleep() {
    faultLatched = true;
    StopCurrent();
    // Do not allow queued amplitudes to survive a fault.
    clearStimFIFO();
    runtimeFIFOEnabled = false;
    stopSPIBus();
    if (activeCentral.connected()) {
        activeCentral.disconnect();
    }
    BLE.stopAdvertise();
    BLE.end();
    Serial.end();
    powerControlADC(false);
    am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_IOM0);
    am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_IOM1);
    am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_IOM2);
    am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_IOM3);
    am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_IOM4);
    am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_IOM5);
    am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_ADC);
    am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_UART0);
    am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_UART1);
    for (int x = 0; x < 50; x++) {
        am_hal_gpio_pinconfig(x, g_AM_HAL_GPIO_DISABLE);
    }
    am_hal_pwrctrl_memory_deepsleep_powerdown(AM_HAL_PWRCTRL_MEM_ALL);
    am_hal_pwrctrl_memory_deepsleep_retain(AM_HAL_PWRCTRL_MEM_SRAM_384K);
    am_hal_stimer_config(AM_HAL_STIMER_CFG_CLEAR | AM_HAL_STIMER_CFG_FREEZE);
    am_hal_stimer_config(AM_HAL_STIMER_XTAL_32KHZ);
    while (1) {
        am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
    }
}

// ============================================================================
// BLE receive callback
// ============================================================================
void onStimWrite(BLEDevice central, BLECharacteristic characteristic) {
    if (faultLatched) {
        return;
    }

    // Reject malformed packets.
    if (characteristic.valueLength() != 3) {
        if (badChecksumConsecutive < 255) {
            badChecksumConsecutive++;
        }
        return;
    }
    uint8_t rx[3];
    characteristic.readValue(rx, 3);
    uint8_t rawCh10 = rx[0];
    uint8_t rawCh15 = rx[1];
    uint8_t chk = rx[2];

    // ========================================================================
    // Valid checksum
    // ========================================================================
    if (xorChecksum(rawCh10, rawCh15) == chk) {
        uint8_t newCH10 = clampAmplitude(rawCh10);
        uint8_t newCH15 = clampAmplitude(rawCh15);
        badChecksumConsecutive = 0;

        // --------------------------------------------------------------------
        // Startup validation:
        // retain existing single-value staging behavior.
        // --------------------------------------------------------------------
        if (!runtimeFIFOEnabled) {
            stagedAmpCH10 = newCH10;
            stagedAmpCH15 = newCH15;
            stagedValid = true;
        }

        // --------------------------------------------------------------------
        // Runtime FIFO path.
        //
        // The FIRST valid runtime packet is both:
        //   1) a normal FIFO sample, and
        //   2) the one-time phase reference.
        //
        // All later packets are queued exactly as before.
        // --------------------------------------------------------------------
        else {
            if (runState == WAIT_FOR_FIRST_RUNTIME_PACKET &&
                !firstRuntimePacketSeen) {

                // Timestamp the same packet on both clocks.
                // DWT is the actual stimulation phase reference.
                firstRuntimePacketCycle = DWT->CYCCNT;

                // CTIMER is retained for BLE guard timing.
                firstRuntimePacketTick = robustTicks();

                // Do not discard the phase-reference packet; queue it normally.
                enqueueStimSample(newCH10, newCH15);

                // Publish this flag LAST so the state machine cannot act before
                // both the timestamp and FIFO entry are valid.
                firstRuntimePacketSeen = true;
            }
            else {
                enqueueStimSample(newCH10, newCH15);
            }
        }
    }

    // ========================================================================
    // Bad checksum
    // ========================================================================
    else {
        if (badChecksumConsecutive < 255) {
            badChecksumConsecutive++;
        }
    }
}

// ============================================================================
// BLE mirror
// ============================================================================
void updateMirror(uint8_t ch10, uint8_t ch15) {
    uint8_t buf[2] = { ch10, ch15 };
    MirrorChar.writeValue(buf, 2);
}

// ============================================================================
// Initialize 3 MHz CTIMER
// ============================================================================
void initRobustMicros()
{
    am_hal_ctimer_stop(
        ROBUST_TIMER_NUM,
        AM_HAL_CTIMER_BOTH
    );
    am_hal_ctimer_clear(
        ROBUST_TIMER_NUM,
        AM_HAL_CTIMER_BOTH
    );
    am_hal_ctimer_config_t timerCfg = {};
    timerCfg.ui32Link = 1;
    timerCfg.ui32TimerAConfig =
        AM_HAL_CTIMER_FN_CONTINUOUS |
        AM_HAL_CTIMER_HFRC_3MHZ;
    timerCfg.ui32TimerBConfig = 0;
    am_hal_ctimer_config(
        ROBUST_TIMER_NUM,
        &timerCfg
    );
    am_hal_ctimer_start(
        ROBUST_TIMER_NUM,
        AM_HAL_CTIMER_TIMERA
    );
}