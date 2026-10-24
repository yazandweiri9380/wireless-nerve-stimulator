#include <ArduinoBLE.h>

#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"

// ============================================================================
// MASTER ARTEMIS BLE HOST
//
// Initialization-aligned implementation:
//   - Artemis acts as BLE Central
//   - Finds peripheral named "Neuro"
//   - Performs the same 5-packet startup validation
//   - Waits for a one-time READY marker from the peripheral
//   - Sends [CH10, CH15, XOR] using Write Without Response
//   - Uses Apollo3 3 MHz CTIMER
//   - Runtime packet period = 99996 ticks = 33.332 ms
//   - The first runtime packet becomes the peripheral's one-time phase reference
//   - No continuous synchronization after startup
//   - No sequence number
//   - No BLE connection-interval tuning
//   - No timing guard around BLE.poll()
// ============================================================================


// ============================================================================
// BLE configuration
// ============================================================================

const char* TARGET_NAME = "Neuro";

const char* RX_UUID =
    "19c10031-e8f2-537e-4f6c-d104768a1214";

const char* MIRROR_UUID =
    "19c10033-e8f2-537e-4f6c-d104768a1214";

BLEDevice neuroPeripheral;
BLECharacteristic stimRXChar;
BLECharacteristic mirrorChar;


// ============================================================================
// Timing
// ============================================================================

#define ROBUST_TIMER_NUM 0

// Same value as stimulation receiver
const uint32_t CYCLE_PERIOD_ticks = 100000;

// 50 ms validation wait
// 50,000 us × 3 ticks/us = 150,000 ticks
const uint32_t VALIDATION_WAIT_ticks = 150000;

// Match current long-duration receiver test
const uint32_t MAX_REP = 60000;

const uint32_t RECEIVER_READY_WAIT_ticks = 6000000;  // 2.0 s before validation

// One-time READY handshake after validation.
// Values are outside the valid 0...100 amplitude range.
const uint8_t READY_MARKER_0 = 0xA5;
const uint8_t READY_MARKER_1 = 0x5A;

// Poll every 50 ms while waiting for READY; timeout after 3 s.
const uint32_t READY_POLL_ticks = 150000;
const uint32_t READY_TIMEOUT_ticks = 9000000;

// ============================================================================
// Packet generation
// ============================================================================

const uint8_t STEP_MIN = 50;
const uint8_t STEP_MAX = 100;

const uint8_t VALIDATION_COUNT = 5;


// ============================================================================
// Host timing diagnostics
//
// Nothing is printed during the runtime loop.
// ============================================================================

uint32_t hostScheduleOverruns = 0;
uint32_t writeFailures = 0;

uint32_t maxSendLatenessTicks = 0;
uint32_t maxWriteDurationTicks = 0;

uint32_t lateOver100us = 0;
uint32_t lateOver1ms = 0;


// ============================================================================
// Helpers
// ============================================================================

uint8_t xorChecksum(uint8_t a, uint8_t b) {
    return a ^ b;
}


// Race-condition immune 32-bit CTIMER read.
// Timer A = lower 16 bits
// Timer B = upper 16 bits
uint32_t robustTicks() {

    uint32_t high1;
    uint32_t high2;
    uint32_t low;

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

    } while (high1 != high2);

    return (high2 << 16) | (low & 0xFFFF);
}


// Configure the same 3 MHz CTIMER used by the receiver
void initRobustMicros() {

    am_hal_ctimer_clear(
        ROBUST_TIMER_NUM,
        AM_HAL_CTIMER_BOTH
    );

    am_hal_ctimer_config_single(
        ROBUST_TIMER_NUM,
        AM_HAL_CTIMER_BOTH,
        AM_HAL_CTIMER_HFRC_3MHZ |
        AM_HAL_CTIMER_FN_CONTINUOUS
    );

    am_hal_ctimer_period_set(
        ROBUST_TIMER_NUM,
        AM_HAL_CTIMER_BOTH,
        0xFFFFFFFF,
        3
    );

    am_hal_ctimer_start(
        ROBUST_TIMER_NUM,
        AM_HAL_CTIMER_BOTH
    );
}


// Wrap-safe wait using the hardware timer.
//
// BLE.poll() is deliberately allowed to run right up to the deadline.
// We are NOT adding a BLE guard yet because this is the baseline test.
void waitUntilTick(uint32_t deadline) {

    while ((int32_t)(deadline - robustTicks()) > 0) {
        BLE.poll();
    }
}


// Poll BLE for a fixed number of 3 MHz timer ticks
void pollForTicks(uint32_t ticks) {

    uint32_t deadline = robustTicks() + ticks;

    while ((int32_t)(deadline - robustTicks()) > 0) {
        BLE.poll();
    }
}


// ============================================================================
// BLE scanning
// ============================================================================

bool findNeuroPeripheral() {

    Serial.println("Scanning for Neuro...");

    BLE.scanForName(TARGET_NAME);

    while (true) {

        BLEDevice peripheral = BLE.available();

        if (peripheral) {

            if (peripheral.hasLocalName() &&
                peripheral.localName() == TARGET_NAME) {

                BLE.stopScan();

                neuroPeripheral = peripheral;

                Serial.print("Found Neuro: ");
                Serial.println(neuroPeripheral.address());

                return true;
            }
        }

        BLE.poll();
    }
}


// ============================================================================
// Connect and discover characteristics
// ============================================================================

bool connectToNeuro() {

    Serial.print("Connecting to ");
    Serial.print(neuroPeripheral.address());
    Serial.println(" ...");

    if (!neuroPeripheral.connect()) {
        Serial.println("Connection failed.");
        return false;
    }

    Serial.println("Connected.");

    Serial.println("Discovering BLE attributes...");

    if (!neuroPeripheral.discoverAttributes()) {
        Serial.println("Attribute discovery failed.");
        neuroPeripheral.disconnect();
        return false;
    }

    stimRXChar =
        neuroPeripheral.characteristic(RX_UUID);

    mirrorChar =
        neuroPeripheral.characteristic(MIRROR_UUID);

    if (!stimRXChar) {
        Serial.println("RX characteristic not found.");
        neuroPeripheral.disconnect();
        return false;
    }

    if (!mirrorChar) {
        Serial.println("Mirror characteristic not found.");
        neuroPeripheral.disconnect();
        return false;
    }

    if (!stimRXChar.canWrite()) {
        Serial.println("RX characteristic is not writable.");
        neuroPeripheral.disconnect();
        return false;
    }

    // Make sure the peripheral supports the same WWR method
    // used by the Python host.
    if (!(stimRXChar.properties() &
          BLEWriteWithoutResponse)) {

        Serial.println(
            "RX characteristic does not support "
            "Write Without Response."
        );

        neuroPeripheral.disconnect();
        return false;
    }

    if (!mirrorChar.canRead()) {
        Serial.println("Mirror characteristic is not readable.");
        neuroPeripheral.disconnect();
        return false;
    }

    Serial.println("Characteristics discovered.");

    return true;
}


// ============================================================================
// Write one stimulation packet
// ============================================================================

bool writeStimPacket(
    uint8_t ch10,
    uint8_t ch15,
    bool corruptChecksum = false
) {

    uint8_t chk = xorChecksum(ch10, ch15);

    if (corruptChecksum) {
        chk ^= 0xFF;
    }

    uint8_t packet[3] = {
        ch10,
        ch15,
        chk
    };

    // false = Write Without Response
    return stimRXChar.writeValue(
        packet,
        sizeof(packet),
        false
    );
}


// ============================================================================
// Startup validation
// ============================================================================

bool validateLink() {

    Serial.println(
        "Starting startup validation..."
    );

    for (uint8_t i = 0;
         i < VALIDATION_COUNT;
         i++) {

        uint8_t ch10 = i;
        uint8_t ch15 = 100 - i;

        if (!writeStimPacket(ch10, ch15)) {

            Serial.println(
                "Validation write failed."
            );

            return false;
        }


        // Same 50 ms wait used in the Python host.
        pollForTicks(VALIDATION_WAIT_ticks);


        // Explicitly request the remote characteristic value.
        if (!mirrorChar.read()) {

            Serial.println(
                "Mirror read failed."
            );

            return false;
        }

        if (mirrorChar.valueLength() != 2) {

            Serial.println(
                "Incorrect mirror length."
            );

            return false;
        }

        uint8_t got10 = mirrorChar[0];
        uint8_t got15 = mirrorChar[1];


        Serial.print("validate ");
        Serial.print(i + 1);
        Serial.print("/");
        Serial.print(VALIDATION_COUNT);

        Serial.print(": sent=[");
        Serial.print(ch10);
        Serial.print(",");
        Serial.print(ch15);

        Serial.print("] mirror=[");
        Serial.print(got10);
        Serial.print(",");
        Serial.print(got15);
        Serial.println("]");


        if ((got10 != ch10) ||
            (got15 != ch15)) {

            Serial.println(
                "Validation failed."
            );

            return false;
        }
    }

    Serial.println("Validation passed.");

    return true;
}


// ============================================================================
// Wait for one-time peripheral READY marker
// ============================================================================

bool waitForPeripheralReady() {

    Serial.println("Waiting for peripheral READY...");

    uint32_t startTick = robustTicks();

    while ((robustTicks() - startTick) < READY_TIMEOUT_ticks) {

        BLE.poll();

        if (mirrorChar.read()) {

            if (mirrorChar.valueLength() == 2) {

                uint8_t b0 = mirrorChar[0];
                uint8_t b1 = mirrorChar[1];

                if ((b0 == READY_MARKER_0) &&
                    (b1 == READY_MARKER_1)) {

                    Serial.println("Peripheral READY.");
                    return true;
                }
            }
        }

        pollForTicks(READY_POLL_ticks);
    }

    Serial.println("Peripheral READY timeout.");
    return false;
}


// ============================================================================
// Runtime baseline with one-time initialization alignment
// ============================================================================

void runBaseline() {

    Serial.println();
    Serial.println(
        "Starting Artemis-to-Artemis baseline..."
    );

    Serial.print("CYCLE_PERIOD_ticks = ");
    Serial.println(CYCLE_PERIOD_ticks);

    Serial.print("Runtime packets = ");
    Serial.println(MAX_REP);

    Serial.println();


    // Reset diagnostics
    hostScheduleOverruns = 0;
    writeFailures = 0;

    maxSendLatenessTicks = 0;
    maxWriteDurationTicks = 0;

    lateOver100us = 0;
    lateOver1ms = 0;


    uint8_t stepValue = 0;

    uint32_t packetCount = 0;

    uint32_t startMillis = millis();


    // Send the first runtime packet immediately,
    // as the Python program does.
    uint32_t NextT = robustTicks();


    while (
        packetCount < MAX_REP &&
        neuroPeripheral.connected()
    ) {

        // --------------------------------------------------------------
        // Wait until the next absolute 3 MHz timer deadline.
        //
        // BLE.poll() is allowed during this waiting period.
        // --------------------------------------------------------------

        waitUntilTick(NextT);


        // Capture actual application-level send-call time
        // BEFORE doing anything else.
        uint32_t actualSendTick = robustTicks();

        int32_t lateness =
            (int32_t)(actualSendTick - NextT);


        // --------------------------------------------------------------
        // Generate same packet sequence as Python
        // --------------------------------------------------------------

        //uint8_t ch10 = stepValue;
        uint8_t ch10 = 100;

        uint8_t ch15 =
            STEP_MAX - stepValue;

        uint8_t chk =
            xorChecksum(ch10, ch15);

        uint8_t packet[3] = {
            ch10,
            ch15,
            chk
        };


        // --------------------------------------------------------------
        // Write Without Response
        // --------------------------------------------------------------

        uint32_t writeStart =
            robustTicks();

        bool writeOK =
            stimRXChar.writeValue(
                packet,
                sizeof(packet),
                false
            );

        uint32_t writeDuration =
            robustTicks() - writeStart;


        // --------------------------------------------------------------
        // Diagnostics
        //
        // No Serial printing here.
        // --------------------------------------------------------------

        if (!writeOK) {
            writeFailures++;
        }

        if (lateness > 0) {

            uint32_t lateTicks =
                (uint32_t)lateness;

            if (lateTicks >
                maxSendLatenessTicks) {

                maxSendLatenessTicks =
                    lateTicks;
            }

            // 100 us = 300 ticks
            if (lateTicks > 300) {
                lateOver100us++;
            }

            // 1 ms = 3000 ticks
            if (lateTicks > 3000) {
                lateOver1ms++;
            }
        }


        if (writeDuration >
            maxWriteDurationTicks) {

            maxWriteDurationTicks =
                writeDuration;
        }


        packetCount++;


        // --------------------------------------------------------------
        // Advance amplitude
        // --------------------------------------------------------------

        stepValue++;

        if (stepValue > STEP_MAX) {
            stepValue = STEP_MIN;
        }


        // --------------------------------------------------------------
        // Next ideal transmission deadline
        // --------------------------------------------------------------

        NextT += CYCLE_PERIOD_ticks;


        // If execution has slipped by an entire cycle,
        // count it and re-anchor rather than producing a catch-up burst.
        uint32_t now = robustTicks();

        if ((int32_t)(now - NextT) >= 0) {

            hostScheduleOverruns++;

            NextT =
                now + CYCLE_PERIOD_ticks;
        }
    }


    // Give BLE a short opportunity to flush the final command.
    pollForTicks(300000);   // 100 ms


    uint32_t durationMs =
        millis() - startMillis;


    // ========================================================================
    // Results
    // ========================================================================

    Serial.println();
    Serial.println(
        "========== MASTER RESULTS =========="
    );

    Serial.print("Packets sent = ");
    Serial.println(packetCount);

    Serial.print("Time Duration = ");
    Serial.println(durationMs);

    Serial.print(
        "Host Schedule Overruns = "
    );
    Serial.println(hostScheduleOverruns);

    Serial.print(
        "BLE Write Failures = "
    );
    Serial.println(writeFailures);

    Serial.print(
        "Max Send Lateness ticks = "
    );
    Serial.println(maxSendLatenessTicks);

    Serial.print(
        "Max Send Lateness us = "
    );
    Serial.println(
        maxSendLatenessTicks / 3.0,
        3
    );

    Serial.print(
        "Send lateness >100 us = "
    );
    Serial.println(lateOver100us);

    Serial.print(
        "Send lateness >1 ms = "
    );
    Serial.println(lateOver1ms);

    Serial.print(
        "Max writeValue duration ticks = "
    );
    Serial.println(maxWriteDurationTicks);

    Serial.print(
        "Max writeValue duration us = "
    );
    Serial.println(
        maxWriteDurationTicks / 3.0,
        3
    );

    Serial.println(
        "===================================="
    );
}


// ============================================================================
// setup()
// ============================================================================

void setup() {

    Serial.begin(9600);

    // Do not wait indefinitely for the USB serial port.
    delay(1000);

    Serial.println();
    Serial.println(
        "Artemis BLE Master Host"
    );


    // Same 3 MHz timing source as receiver
    initRobustMicros();


    if (!BLE.begin()) {

        Serial.println(
            "BLE initialization failed."
        );

        while (1);
    }


    if (!findNeuroPeripheral()) {
        while (1);
    }


    if (!connectToNeuro()) {
        while (1);
    }

// Receiver intentionally waits 1.5 s after connection before
// entering VALIDATION_PHASE. Allow that grace period plus margin.
Serial.println("Waiting for receiver initialization...");
pollForTicks(RECEIVER_READY_WAIT_ticks);


    if (!validateLink()) {

        Serial.println(
            "Startup validation failed."
        );

        neuroPeripheral.disconnect();

        while (1);
    }


    // The peripheral has completed validation and its existing startup delay,
    // and is now armed waiting for the first runtime packet.
    if (!waitForPeripheralReady()) {

        Serial.println(
            "Peripheral did not become ready."
        );

        neuroPeripheral.disconnect();
        while (1);
    }


    // The first runtime packet sent here becomes the one-time phase reference
    // at the peripheral. Runtime timing after that remains fully independent.
    runBaseline();


    Serial.println(
        "Baseline test complete."
    );

    if (neuroPeripheral.connected()) {
        neuroPeripheral.disconnect();
    }
}


// ============================================================================
// loop()
// ============================================================================

void loop() {

    // Test executes once from setup().
    // Stay idle afterward.

    BLE.poll();
}