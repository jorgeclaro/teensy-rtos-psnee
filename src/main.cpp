#include <Arduino.h>
#include "arduino_freertos.h"
#include "avr/pgmspace.h"
#include "FreeRTOS.h"
#include <queue.h>
#include <semphr.h>
#include <task.h>
#include <stream_buffer.h>

// Pin definitions
#define BIOS_A18 4      // connect to PSOne BIOS A18 (pin 31 on that chip)
#define BIOS_D2  5      // connect to PSOne BIOS D2 (pin 15 on that chip)
#define SQCK 6          // connect to PSX HC-05 SQCK pin 26 (PU-7 and early PU-8 Mechacons: pin 41)
#define SUBQ 7          // connect to PSX HC-05 SUBQ pin 24 (PU-7 and early PU-8 Mechacons: pin 39)  
#define DATA 8          // connect to point 6 in old modchip diagrams
#define GATE_WFCK 9     // connect to point 5 in old modchip diagrams

// RTOS config
#define DEBUG true                                      // Enables logger task
#define LOGGER_BUFFER_SIZE 128                          // Logger buffer size
#define LOGGER_MSG_MAX_SIZE 100                         // Maximum message size
#define MAX_TASK_NUM 20                                 // Maximum number of tasks
#define SUBQ_CAPTURE_TIMEOUT 2000                       // Timeout for SUBQ capture in microseconds (2000)
#define SUBQ_PACKET_LENGTH 12                           // Length of SUBQ packets
#define SUBQ_QUEUE_SIZE 1                               // Size of the SUBQ queue
#define SCEX_INJECTION_TIMEOUT 5000                     // Timeout for SCEX injection in milliseconds (5000)
#define SCEX_INJECTION_BITS_DELAY 4000                  // Bit delay for SCEX injection (3950 ~ 4100 microseconds)
#define SCEX_INJECTION_LOOP_DELAY_PU22 90               // Loop delay for PU-22+ in milliseconds (80 ~ 100)
#define SCEX_INJECTION_LOOP_DELAY_OLDCROW 72            // Loop delay for Oldcrow in milliseconds (72)
#define SCEX_INJECTION_LOOPS 2                          // Number of SCEX injection loops (2 to cover all boards)
#define SCEX_INJECTION_ATTEMPTS 3                       // Number of SCEX injection attempts (3 to cover all boards)
#define BOARD_DETECTION_SAMPLE_PERIOD 1000              // Sample period for board detection in milliseconds (1000)
#define BOARD_DETECTION_SAMPLE_INTERVAL 1               // Sample interval for board detection in milliseconds (1)
#define BOARD_DETECTION_GATE_WFCK_LOWS_THRESHOLD 20     // Threshold for WFCK low readings (20)
#define BOARD_DETECTION_SQCK_HIGHS_THRESHOLD 100        // Threshold for SQCK high readings (100)
#define BIOS_PATCH true                                 // Enables Bios Patch task
#define BIOS_PATCH_STAGE1_DELAY 1250                    // Delay for BIOS patch stage 1 in milliseconds (1250)
#define BIOS_PATCH_STAGE1_ATTEMPTS 1                    // Number of attempts for BIOS patch stage 1 (1)
#define BIOS_PATCH_STAGE2_DELAY 17                      // Delay for BIOS patch stage 2 in microseconds (17)
#define BIOS_PATCH_STAGE3_DELAY 4                       // Delay for BIOS patch stage 3 in microseconds (4)
#define BIOS_PATCH_TIMEOUT 2000                         // Timeout for BIOS patch in milliseconds (3000)

// SCEX data
static const unsigned char SCEEData[] = {0b01011001, 0b11001001, 0b01001011, 0b01011101, 0b11101010, 0b00000010};
static const unsigned char SCEAData[] = {0b01011001, 0b11001001, 0b01001011, 0b01011101, 0b11111010, 0b00000010};
static const unsigned char SCEIData[] = {0b01011001, 0b11001001, 0b01001011, 0b01011101, 0b11011010, 0b00000010};

// Specified target region
static const char region = 'e';

QueueHandle_t loggerQueue;
QueueHandle_t scqkwfckDriveDataQueue;
QueueHandle_t subqDriveDataQueue;
QueueHandle_t subqRawDataQueue;
QueueHandle_t pu22modeQueue;
QueueHandle_t powerQueue;
QueueHandle_t scexQueue;

SemaphoreHandle_t powerSem;
SemaphoreHandle_t pu22modeSem;
SemaphoreHandle_t injectScexSem;
SemaphoreHandle_t patchBiosSem;

TaskHandle_t tLoggerHandler;
TaskHandle_t tCaptureSQCKandQFCKDataHandler;
TaskHandle_t tPowerHandler;
TaskHandle_t tPu22modeHandler;
TaskHandle_t tPalBiosPatchHandler;
TaskHandle_t tCaptureSUBQPacketsHandler;
TaskHandle_t tPrintHexSUBQPacketsHandler;
TaskHandle_t tCheckSUBQWobleAreaHandler;
TaskHandle_t tInjectSCEXHandler;
TaskHandle_t tStatsHandler;
TaskHandle_t tBlinkerHandler;
TaskHandle_t tDebugHandler;

typedef struct {
    unsigned int gate_wfck_highs;
    unsigned int gate_wfck_lows;
    unsigned int sqck_highs;
    unsigned int sqck_lows;
} SCQKGateWFCKDriveDataPoint;

typedef struct {
    boolean isDataSector;
    boolean hasWobbleInCDDASpace;
    boolean option1;
    boolean option2;
    boolean garbageCollectionCheck;
    boolean isGameDisk;
    boolean checkingWobble;
} SUBQDriveDataPoint;

/**
 * Uses the ARM Cortex-M7 DWT cycle counter for precise microsecond delays.
 */
void delayMicrosecondsDWT(uint32_t us) {
    uint32_t start = ARM_DWT_CYCCNT; // Get current cycle count
    uint32_t target = (F_CPU / 1000000) * us; // Convert microseconds to cycles

    while ((ARM_DWT_CYCCNT - start) < target) {
        // Busy-wait until the required cycle count is reached
    }
}

/**
 * Enables the DWT (Data Watchpoint and Trace) cycle counter.
 * Must be called once at startup.
 */
void enableDWT() {
    ARM_DEMCR |= ARM_DEMCR_TRCENA; // Enable trace
    ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA; // Enable cycle counter
}

void sendRTOSMsg(const char* message) {
    if (DEBUG) {
        if (uxQueueSpacesAvailable(loggerQueue) > 0) {
            size_t logLength = configMAX_TASK_NAME_LEN + strlen(message) + 3;
            char log[logLength];
            snprintf(log, logLength, "%s: %s", pcTaskGetName(NULL), message);
            xQueueSend(loggerQueue, log, portMAX_DELAY);
                
            // With malloc
            //char* log = formatStringMalloc("%s: %s", pcTaskGetName(NULL), message);
            //xQueueSend(loggerQueue, log, portMAX_DELAY);
            //vPortFree(log);
        }
    }
}

bool readBit(int index, const unsigned char *ByteSet) {
    int byte_index = index >> 3;
    byte bits = ByteSet[byte_index];
    // same as (index - byte_index<<3) or (index%8)
    int bit_index = index & 0x7;
    byte mask = 1 << bit_index;
    return (0 != (bits & mask));
}

void patchBios() {
    sendRTOSMsg("Starting PAL BIOS patch");

    digitalWriteFast(arduino::LED_BUILTIN, arduino::HIGH);
    pinMode(BIOS_A18, arduino::INPUT);
    pinMode(BIOS_D2, arduino::INPUT);

    bool pulseFound = false;

    for (int i = 0; i < BIOS_PATCH_STAGE1_ATTEMPTS; i++) {
        sendRTOSMsg("Waiting for stage 1 A18 intro pulse");

        TickType_t now = xTaskGetTickCount();

        while (!digitalReadFast(BIOS_A18)) {
            if ((xTaskGetTickCount() - now) > pdMS_TO_TICKS(BIOS_PATCH_TIMEOUT)) {
                pulseFound = false;
                break;
            }
        }

        pulseFound = true;
        
        sendRTOSMsg("Stage 1 A18 intro pulse found");

        vTaskDelay(BIOS_PATCH_STAGE1_DELAY);
    }

    sendRTOSMsg("A18 intro pulse exausted all attempts");
    
    if (pulseFound) {
        // max 17us (maximize this when tuning!)
        delayMicrosecondsDWT(BIOS_PATCH_STAGE2_DELAY);
        digitalWriteFast(BIOS_A18, arduino::LOW);
        digitalWriteFast(BIOS_D2, arduino::HIGH);

        // min 2us (minimize this when tuning, after maximizing first us delay!)
        delayMicrosecondsDWT(BIOS_PATCH_STAGE3_DELAY);
        digitalWriteFast(BIOS_D2, arduino::LOW);
    } else {
        sendRTOSMsg("Failed to patch PAL BIOS");
    }

    pinMode(BIOS_A18, arduino::INPUT);
    pinMode(BIOS_D2, arduino::INPUT);
    digitalWriteFast(arduino::LED_BUILTIN, arduino::LOW);
    sendRTOSMsg("Completed PAL BIOS patch");
}

void checkPower(unsigned int sqck_highs) {
    boolean power = false;

    // Attempt to peek the queue, only set power to false if the peek fails
    if (xQueuePeek(powerQueue, &power, pdMS_TO_TICKS(0)) != pdPASS) {
        sendRTOSMsg("Assuming power off");
    }

    // Evaluate the current power state
    boolean power_tmp = (sqck_highs > BOARD_DETECTION_SQCK_HIGHS_THRESHOLD);

    // Only proceed if the power state has changed
    if (power != power_tmp) {
        if (power_tmp) {
            sendRTOSMsg("Power on");
            sendRTOSMsg("Allowing path BIOS");
            if (BIOS_PATCH) {
                xSemaphoreGive(patchBiosSem);
            }
        } else {
            sendRTOSMsg("Power off");
        }

        // Update the power state and overwrite the queue
        power = power_tmp;
        xQueueOverwrite(powerQueue, &power);

        sendRTOSMsg("Resuming pu22mode task");
        vTaskResume(tPu22modeHandler);
    }
}

void checkPu22mode(unsigned int gate_wfck_lows) {

    // Board detection
    //
    // GATE: __-----------------------  // this is a PU-7 .. PU-20 board!
    //
    // WFCK: __-_-_-_-_-_-_-_-_-_-_-_-  // this is a PU-22 or newer board!
  
    boolean pu22mode = false;

    // Attempt to peek the queue, only set pu22mode to false if the peek fails
    if (xQueuePeek(pu22modeQueue, &pu22mode, pdMS_TO_TICKS(0)) != pdPASS) {
        sendRTOSMsg("Assuming Oldcrow mode");
    }

    // Evaluate the current pu22mode state
    boolean pu22mode_tmp = (gate_wfck_lows > BOARD_DETECTION_GATE_WFCK_LOWS_THRESHOLD);

    // Only proceed if the pu22mode state has changed
    if (pu22mode != pu22mode_tmp) {
        if (pu22mode_tmp) {
            sendRTOSMsg("Pu22 mode");
        } else {
            sendRTOSMsg("Oldcrow mode");
        }

        // Update the pu22mode state and overwrite the queue
        pu22mode = pu22mode_tmp;
        xQueueOverwrite(pu22modeQueue, &pu22mode);
    }
}

SCQKGateWFCKDriveDataPoint captureSQCKandGateWFCK() {
    unsigned int gate_wfck_highs = 0;
    unsigned int gate_wfck_lows = 0;
    unsigned int sqck_highs = 0;
    unsigned int sqck_lows = 0;

    TickType_t now = xTaskGetTickCount();

    while ((xTaskGetTickCount() - now) < pdMS_TO_TICKS(BOARD_DETECTION_SAMPLE_PERIOD)) {
        if (digitalReadFast(SQCK) == 1) sqck_highs++;
        if (digitalReadFast(SQCK) == 0) sqck_lows++;
        if (digitalReadFast(GATE_WFCK) == 1) gate_wfck_highs++;
        if (digitalReadFast(GATE_WFCK) == 0) gate_wfck_lows++;

        // 1ms interval -> 1000 reads
        vTaskDelay(pdMS_TO_TICKS(BOARD_DETECTION_SAMPLE_INTERVAL));
    }

    SCQKGateWFCKDriveDataPoint p = {
        gate_wfck_highs,
        gate_wfck_lows,
        sqck_highs,
        sqck_lows
    };

    return p;
}

void captureSUBQPackets(byte *scbuf_ptr) {
    TickType_t now = xTaskGetTickCount();
    byte bitbuf = 0;
    bool sample = 0;

    for (byte scpos = 0; scpos < SUBQ_PACKET_LENGTH; scpos++) {
        for (byte bitpos = 0; bitpos < 8; bitpos++) {
            while (digitalReadFast(SQCK) == 1) {
                // Convert timeout from microseconds to ticks
                TickType_t timeoutTicks = SUBQ_CAPTURE_TIMEOUT / (1000000 / configTICK_RATE_HZ);
                
                if ((xTaskGetTickCount() - now) > timeoutTicks) {
                    // Reset SUBQ packet stream
                    scpos = 0;
                    bitbuf = 0;
                    now = xTaskGetTickCount();
                    continue;
                }
            }

            // Wait for clock to go low
            while (digitalReadFast(SQCK) == 0);

            sample = digitalReadFast(SUBQ);
            bitbuf |= sample << bitpos;

            // Update now to the latest tick count
            now = xTaskGetTickCount();
        }

        scbuf_ptr[scpos] = bitbuf;
        bitbuf = 0;
    }
}

void printHexSUBQPackets(byte *scbuf_ptr) {
    size_t lenght = SUBQ_PACKET_LENGTH * 3 + 1;
    size_t remaining = lenght;
    char buffer[lenght] = {0};
    char *ptr = buffer;

    for (size_t i = 0; i < SUBQ_PACKET_LENGTH; i++) {
        int written = snprintf(ptr, remaining, "%02X ", scbuf_ptr[i]);
        if (written < 0 || (size_t)written >= remaining) {
            break;
        }
        ptr += written;
        remaining -= written;
    }
    sendRTOSMsg(buffer);
}

SUBQDriveDataPoint parseSUBQPacket(byte *scbuf_ptr) {
    // we only want to unlock game discs (0x41) and only if the read head is in the outer TOC area
    // we want to see a TOC sector repeatedly before injecting (helps with timing and marginal lasers)
    // all this logic is because we don't know if the HC-05 is actually processing a getSCEX() command
    // while the laser lens moves to correct for the error, they can pick up a few TOC sectors
  
    boolean isDataSector = (((scbuf_ptr[0] & 0x40) == 0x40) && (((scbuf_ptr[0] & 0x10) == 0) && ((scbuf_ptr[0] & 0x80) == 0)));
    boolean hasWobbleInCDDASpace = (isDataSector || scbuf_ptr[0] == 0x01);				        // [0] = 0x41 (psx game disk) then goto 0x01
    boolean option1 = scbuf_ptr[2] == 0xA0 || scbuf_ptr[2] == 0xA1 || scbuf_ptr[2] == 0xA2;		// if [2] = A0, A1, A2 ..
    boolean option2 = scbuf_ptr[2] == 0x01 && (scbuf_ptr[3] >= 0x98 || scbuf_ptr[3] <= 0x02);	// .. or = 01 but then [3] is either > 98 or < 02
    boolean garbageCollectionCheck = scbuf_ptr[1] == 0x00 && scbuf_ptr[6] == 0x00;
    boolean isGameDisk = isDataSector && (option1 || option2) && garbageCollectionCheck;
    boolean checkingWobble = hasWobbleInCDDASpace && garbageCollectionCheck;

    SUBQDriveDataPoint p = {
        isDataSector,
        hasWobbleInCDDASpace,
        option1,
        option2,
        garbageCollectionCheck,
        isGameDisk,
        checkingWobble,
    };

    return p;
}

void injectSCEXLoop() {
    boolean pu22mode = false;

    if (xQueuePeek(pu22modeQueue, &pu22mode, pdMS_TO_TICKS(0)) != pdPASS) {
        sendRTOSMsg("Assuming Oldcrow");
    }

    sendRTOSMsg("SCEX inj begin");
    digitalWriteFast(arduino::LED_BUILTIN, arduino::HIGH);

    for (size_t loop_counter = 0; loop_counter < SCEX_INJECTION_LOOPS; loop_counter++) {
        for (size_t attempts_counter = 0; attempts_counter < SCEX_INJECTION_ATTEMPTS; attempts_counter++) {
            for (byte bit_counter = 0; bit_counter < 44; bit_counter++) {
                if (!readBit(bit_counter, region == 'e' ? SCEEData : region == 'a' ? SCEAData : SCEIData)) {
                    pinMode(DATA, arduino::OUTPUT);
                    digitalWriteFast(DATA, arduino::LOW);
                    delayMicrosecondsDWT(SCEX_INJECTION_BITS_DELAY);
                } else {
                    if (pu22mode) {
                        pinMode(DATA, arduino::OUTPUT);
                        uint32_t startCycles = ARM_DWT_CYCCNT;  // Capture start cycle count
                        do {
                            digitalWriteFast(DATA, digitalReadFast(GATE_WFCK));
                        } while ((ARM_DWT_CYCCNT - startCycles) < (F_CPU / 1000000) * SCEX_INJECTION_BITS_DELAY);
                    } else {
                        pinMode(DATA, arduino::INPUT);
                        delayMicrosecondsDWT(SCEX_INJECTION_BITS_DELAY);
                    }
                }
            }

            pinMode(DATA, arduino::OUTPUT);
            digitalWriteFast(DATA, arduino::LOW);
        }

        // HC-05 waits for a bit of silence (pin low) before it begins decoding
        vTaskDelay(pdMS_TO_TICKS(pu22mode ? SCEX_INJECTION_LOOP_DELAY_PU22 : SCEX_INJECTION_LOOP_DELAY_OLDCROW));
    }

    if (!pu22mode) {
        pinMode(GATE_WFCK, arduino::INPUT);
    }

    pinMode(DATA, arduino::INPUT);
    digitalWriteFast(arduino::LED_BUILTIN, arduino::LOW);
    sendRTOSMsg("SCEX inj end");
}

void printStats() {
    TaskStatus_t taskStatusArray[MAX_TASK_NUM];
    UBaseType_t taskCount;
    uint32_t totalRunTime;

    taskCount = uxTaskGetSystemState(taskStatusArray, MAX_TASK_NUM, &totalRunTime);

    for (UBaseType_t i = 0; i < taskCount; i++) {
        char log[LOGGER_MSG_MAX_SIZE];

        sniprintf(log, LOGGER_MSG_MAX_SIZE, "Task: %s\t State: %d\t Priority: %lu\t Stack: %lu\t  Runtime: %lu",
            taskStatusArray[i].pcTaskName,
            taskStatusArray[i].eCurrentState,
            (unsigned long)taskStatusArray[i].uxCurrentPriority,
            (unsigned long)taskStatusArray[i].usStackHighWaterMark,
            (unsigned long)taskStatusArray[i].ulRunTimeCounter
        );
        sendRTOSMsg(log);
    }
}

void checkTaskCreation(BaseType_t result) {
    if (result != pdPASS){
        Serial.print("Task creation failed!\n");
        for (;;);
    }
}

static void ThreadLogger(void*) {
    char receivedMessage[LOGGER_MSG_MAX_SIZE];

   for (;;) {
        if (xQueueReceive(loggerQueue, &receivedMessage, portMAX_DELAY)) {
            Serial.println(receivedMessage);
            vTaskDelay(100);
        }
    }
}

static void ThreadCaptureSQCKandQFCKData(void*) {
    SCQKGateWFCKDriveDataPoint p;
   for (;;) {
        p = captureSQCKandGateWFCK();
        xQueueOverwrite(scqkwfckDriveDataQueue, &p);
        xSemaphoreGive(powerSem);
        xSemaphoreGive(pu22modeSem);
        vTaskDelay(250);
    }
}

static void ThreadPower(void*) {
    SCQKGateWFCKDriveDataPoint p;
    for (;;) {
        xSemaphoreTake(powerSem, portMAX_DELAY);
        if (xQueuePeek(scqkwfckDriveDataQueue, &p, pdMS_TO_TICKS(0)) == pdPASS) {
            checkPower(p.sqck_highs);
        }
    }
}

static void ThreadPu22mode(void*) {
    boolean power;
    SCQKGateWFCKDriveDataPoint p;

    for (;;) {
        xSemaphoreTake(pu22modeSem, portMAX_DELAY);
        if (xQueuePeek(powerQueue, &power, pdMS_TO_TICKS(0)) == pdPASS) {
            if (power) {
                if (xQueuePeek(scqkwfckDriveDataQueue, &p, pdMS_TO_TICKS(0)) == pdPASS) {
                    checkPu22mode(p.gate_wfck_lows);
                }
                sendRTOSMsg("Suspending");
                vTaskSuspend(tPu22modeHandler);
            }
        }
    }
}

static void ThreadPalBiosPatch(void*) {
   for (;;) {
        xSemaphoreTake(patchBiosSem, portMAX_DELAY);
        patchBios();
    }
}

static void ThreadCaptureSUBQPackets(void*) {
    byte scbuf[SUBQ_PACKET_LENGTH] = { 0 };
    byte *scbuf_ptr = scbuf;

   for (;;) {
        captureSUBQPackets(scbuf_ptr);
        xQueueSend(subqRawDataQueue, scbuf, portMAX_DELAY);
    }
}

static void ThreadPrintHexSUBQPackets(void*) {
    byte scbuf[SUBQ_PACKET_LENGTH] = { 0 };
    byte *scbuf_ptr = scbuf;

   for (;;) {
        if (xQueuePeek(subqRawDataQueue, &scbuf, pdMS_TO_TICKS(0)) == pdPASS) {
            printHexSUBQPackets(scbuf_ptr);
        }
    }
}

static void ThreadCheckSUBQWobleArea(void*) {
    byte scbuf[SUBQ_PACKET_LENGTH] = { 0 };
    byte *scbuf_ptr = scbuf;
    SUBQDriveDataPoint p;
    boolean scexAllow = false;
    TickType_t lastAllowTime = 0;

    for (;;) {
        xQueueReceive(subqRawDataQueue, &scbuf, portMAX_DELAY);
        p = parseSUBQPacket(scbuf_ptr);

        if (p.isGameDisk || p.checkingWobble) {
            scexAllow = true;
            lastAllowTime = xTaskGetTickCount();  // Reset timeout timer
            sendRTOSMsg("SCEX inj allow");
        } else if (scexAllow && (xTaskGetTickCount() - lastAllowTime) > pdMS_TO_TICKS(SCEX_INJECTION_TIMEOUT)) {
            scexAllow = false;
            sendRTOSMsg("SCEX inj timeout");
        }

        xQueueOverwrite(scexQueue, &scexAllow);
        xQueueOverwrite(subqDriveDataQueue, &p);
    }
}

static void ThreadInjectSCEX(void*) {
    boolean scexAllow = false;

    for (;;) {
        if (xQueuePeek(scexQueue, &scexAllow, pdMS_TO_TICKS(0)) == pdPASS) {
            if (scexAllow) {
                injectSCEXLoop();
            }
        }
    }
}

static void ThreadStats(void*) {
   for (;;) {
        printStats();
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

static void ThreadDebug(void*) {
    boolean pu22mode = false;
    boolean power = false;
    SUBQDriveDataPoint p1;

    for (;;) {
        if (xQueuePeek(powerQueue, &power, pdMS_TO_TICKS(0)) == pdPASS) {
            size_t powerStrLength = 10;
            char powerStr[powerStrLength];
            snprintf(powerStr, powerStrLength, "power: %d", power);
            sendRTOSMsg(powerStr);
        }
        
        if (xQueuePeek(pu22modeQueue, &pu22mode, pdMS_TO_TICKS(0)) == pdPASS) {
            size_t pu22modeStrLength = 14;
            char pu22modeStr[pu22modeStrLength];
            snprintf(pu22modeStr, pu22modeStrLength, "pu22mode: %d", pu22mode);
            sendRTOSMsg(pu22modeStr);
        }

        if (xQueuePeek(subqDriveDataQueue, &p1, pdMS_TO_TICKS(0)) == pdPASS) {
            size_t subqDriveDataStrLenght = 20;
            char subqDriveDataStr[subqDriveDataStrLenght];
            snprintf(subqDriveDataStr, subqDriveDataStrLenght, "game: %d wobble: %d", p1.isGameDisk, p1.checkingWobble);
            sendRTOSMsg(subqDriveDataStr);
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static void ThreadBlinker(void*) {
    for (;;) {    
        vTaskDelay(pdMS_TO_TICKS(1000));
        digitalWriteFast(arduino::LED_BUILTIN, arduino::HIGH);
        vTaskDelay(pdMS_TO_TICKS(1000));
        digitalWriteFast(arduino::LED_BUILTIN, arduino::LOW);
    }
}

FLASHMEM __attribute__((noinline)) void setup() {
    pinMode(SQCK, arduino::INPUT);
    pinMode(SUBQ, arduino::INPUT);
    pinMode(DATA, arduino::INPUT);
    pinMode(GATE_WFCK, arduino::INPUT);
    pinMode(arduino::LED_BUILTIN, arduino::OUTPUT);

    enableDWT();

    if (DEBUG) {
        Serial.begin(115200);
        Serial.println(PSTR("Booting FreeRTOS kernel " tskKERNEL_VERSION_NUMBER ""));
        Serial.println(PSTR("Built by gcc " __VERSION__ " (newlib " _NEWLIB_VERSION ") on " __DATE__ ""));
        if (CrashReport) {
            Serial.print(CrashReport);
            Serial.println();
            Serial.flush();
        }
    }

    loggerQueue = xQueueCreate(LOGGER_BUFFER_SIZE, LOGGER_MSG_MAX_SIZE);
    scqkwfckDriveDataQueue = xQueueCreate(1, sizeof(SCQKGateWFCKDriveDataPoint));
    subqDriveDataQueue = xQueueCreate(1, sizeof(SUBQDriveDataPoint));
    subqRawDataQueue = xQueueCreate(SUBQ_QUEUE_SIZE, sizeof(byte) * SUBQ_PACKET_LENGTH);
    pu22modeQueue = xQueueCreate(1, sizeof(boolean));
    powerQueue = xQueueCreate(1, sizeof(boolean));
    scexQueue = xQueueCreate(1, sizeof(boolean));
    powerSem = xSemaphoreCreateBinary();
    pu22modeSem = xSemaphoreCreateBinary();
    patchBiosSem = xSemaphoreCreateBinary();

    checkTaskCreation(xTaskCreate(ThreadCaptureSQCKandQFCKData, "sqckqfck", 256, NULL, 1, &tCaptureSQCKandQFCKDataHandler));
    checkTaskCreation(xTaskCreate(ThreadPower, "power", 256, NULL, 1, &tPowerHandler));
    checkTaskCreation(xTaskCreate(ThreadPu22mode, "pu22mode", 256, NULL, 1, &tPu22modeHandler));
    checkTaskCreation(xTaskCreate(ThreadCaptureSUBQPackets, "subq", 256, NULL, 1, &tCaptureSUBQPacketsHandler));
    checkTaskCreation(xTaskCreate(ThreadCheckSUBQWobleArea, "wobble", 256, NULL, 1, &tCheckSUBQWobleAreaHandler));
    checkTaskCreation(xTaskCreate(ThreadInjectSCEX, "scex", 256, NULL, 1, &tInjectSCEXHandler));

    if (BIOS_PATCH) {
        checkTaskCreation(xTaskCreate(ThreadPalBiosPatch, "bios", 256, NULL, 1, &tPalBiosPatchHandler));
    }

    if (DEBUG) {
        checkTaskCreation(xTaskCreate(ThreadLogger, "logger", 512, NULL, 1, &tLoggerHandler));
        //checkTaskCreation(xTaskCreate(ThreadPrintHexSUBQPackets, "hex", 256, NULL, 1, &tPrintHexSUBQPacketsHandler));
        //checkTaskCreation(xTaskCreate(ThreadStats, "stats", 512, NULL, 1, &tStatsHandler));
        //checkTaskCreation(xTaskCreate(ThreadDebug, "debug", 256, NULL, 1, &tDebugHandler));
        //checkTaskCreation(xTaskCreate(ThreadBlinker, "blinker", 128, NULL, 1, &tBlinkerHandler));

        Serial.println("Starting RTOS scheduler");
        Serial.flush();
    }

    vTaskStartScheduler();
}

int main() {
    setup();

    return 1;
}
