#include <Arduino.h>
#include "arduino_freertos.h"
#include "avr/pgmspace.h"
#include "FreeRTOS.h"
#include <queue.h>
#include <semphr.h>
#include <task.h>
#include <stream_buffer.h>

#define DEBUG true

// RTOS config
#define LOGGER_BUFFER_SIZE 256
#define MAX_MSG_SIZE 100
#define MAX_TASK_NUM 20

// SUBQ Packet config
#define SUBQ_PACKET_LENGTH 12
#define SUBQ_QUEUE_SIZE 5

// Pin definitions
#define BIOS_A18 4      // connect to PSOne BIOS A18 (pin 31 on that chip)
#define BIOS_D2  5      // connect to PSOne BIOS D2 (pin 15 on that chip)
#define SQCK 6          // connect to PSX HC-05 SQCK pin 26 (PU-7 and early PU-8 Mechacons: pin 41)
#define SUBQ 7          // connect to PSX HC-05 SUBQ pin 24 (PU-7 and early PU-8 Mechacons: pin 39)  
#define DATA 8          // connect to point 6 in old modchip diagrams
#define GATE_WFCK 9     // connect to point 5 in old modchip diagrams

// SCEX data
static const unsigned char SCEEData[] = {0b01011001, 0b11001001, 0b01001011, 0b01011101, 0b11101010, 0b00000010};
static const unsigned char SCEAData[] = {0b01011001, 0b11001001, 0b01001011, 0b01011101, 0b11111010, 0b00000010};
static const unsigned char SCEIData[] = {0b01011001, 0b11001001, 0b01001011, 0b01011101, 0b11011010, 0b00000010};

// microseconds 250 bits/s (ATtiny 8Mhz works from 3950 to 4100)
static const int scex_injection_bits_delay = 4000;

// milliseconds 72 in oldcrow. PU-22+ work best with 80 to 100
static const int scex_injection_loop_delay = 90;

// nr injection loops (2 to cover all boards)
static const int scex_injection_loops = 3;

// nr attempts for scex (3 to cover all boards)
static const int scex_injection_attempts = 3;

// microseconds 2000
static const int subq_capture_timeout = 2000;

// milliseconds 1000
static const int board_detection_sample_period = 1000;

// milliseconds 1
static const int board_detection_sample_interval = 1;

// readings in board_detection_sample_period 20
static const int board_detection_gate_wfck_lows_threshold = 20;

// readings in board_detection_sample_period 100
static const int board_detection_sqck_highs_threshold = 100;

// milliseconds 1350
static const int bios_patch_stage1_delay = 1250;

// nr attempts stage1 2
static const int bios_patch_stage1_attempts = 2;

// microseconds 17
static const int bios_patch_stage2_delay = 17;

// microseconds 4
static const int bios_patch_stage3_delay = 4;

// milliseconds 3000
static const int bios_patch_timeout = 2000;

QueueHandle_t loggerQueue;
QueueHandle_t scqkwfckDriveDataQueue;
QueueHandle_t subqDriveDataQueue;
QueueHandle_t subqRawDataQueue;
QueueHandle_t pu22modeQueue;
QueueHandle_t powerQueue;

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


SCQKGateWFCKDriveDataPoint createSCQKGateWFCKDataPoint(
    unsigned int gate_wfck_highs,
    unsigned int gate_wfck_lows,
    unsigned int sqck_highs,
    unsigned int sqck_lows
) {
    SCQKGateWFCKDriveDataPoint p = {
        gate_wfck_highs,
        gate_wfck_lows,
        sqck_highs,
        sqck_lows
    };
    return p;
}

SUBQDriveDataPoint createSUBQDataPoint(
    boolean isDataSector,
    boolean hasWobbleInCDDASpace,
    boolean option1,
    boolean option2,
    boolean garbageCollectionCheck,
    boolean isGameDisk,
    boolean checkingWobble
) {
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

/*
char* formatStringMalloc(const char* format, ...) {
    char* buffer = (char*)pvPortMalloc(MAX_MSG_SIZE);
    if (buffer == NULL) {
        return NULL;
    }
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, MAX_MSG_SIZE, format, args);
    va_end(args);
    return buffer;
}
*/

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

    for (int i = 0; i < bios_patch_stage1_attempts; i++) {
        sendRTOSMsg("Waiting for stage 1 A18 intro pulse");

        TickType_t now = xTaskGetTickCount();

        while (!digitalReadFast(BIOS_A18)) {
            if ((xTaskGetTickCount() - now) > pdMS_TO_TICKS(bios_patch_timeout)) {
                pulseFound = false;
                break;
            }
        }

        pulseFound = true;
        
        sendRTOSMsg("Stage 1 A18 intro pulse found");

        vTaskDelay(bios_patch_stage1_delay);
    }

    sendRTOSMsg("A18 intro pulse exausted all attempts");
    
    if (pulseFound) {
        // max 17us for 16Mhz ATmega (maximize this when tuning!)
        delayMicroseconds(bios_patch_stage2_delay);
        digitalWriteFast(BIOS_A18, arduino::LOW);
        digitalWriteFast(BIOS_D2, arduino::HIGH);

        // min 2us for 16Mhz ATmega, 8Mhz requires 3us (minimize this when tuning, after maximizing first us delay!)
        delayMicroseconds(bios_patch_stage3_delay);
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
    boolean power_tmp = (sqck_highs > board_detection_sqck_highs_threshold);

    // Only proceed if the power state has changed
    if (power != power_tmp) {
        if (power_tmp) {
            sendRTOSMsg("Power on");
            sendRTOSMsg("Allowing path BIOS");
            xSemaphoreGive(patchBiosSem);        
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
    boolean pu22mode = false;

    // Attempt to peek the queue, only set pu22mode to false if the peek fails
    if (xQueuePeek(pu22modeQueue, &pu22mode, pdMS_TO_TICKS(0)) != pdPASS) {
        sendRTOSMsg("Assuming Oldcrow mode");
    }

    // Evaluate the current pu22mode state
    boolean pu22mode_tmp = (gate_wfck_lows > board_detection_gate_wfck_lows_threshold);

    // Only proceed if the pu22mode state has changed
    if (pu22mode != pu22mode_tmp) {
        if (pu22mode_tmp) {
            sendRTOSMsg("PU22 mode");
        } else {
            sendRTOSMsg("Oldcrow mode");
        }

        // Update the pu22mode state and overwrite the queue
        pu22mode = pu22mode_tmp;
        xQueueOverwrite(pu22modeQueue, &pu22mode);
    }
}

SCQKGateWFCKDriveDataPoint captureSQCKandGateWFCK() {
  
    // Board detection
    //
    // GATE: __-----------------------  // this is a PU-7 .. PU-20 board!
    //
    // WFCK: __-_-_-_-_-_-_-_-_-_-_-_-  // this is a PU-22 or newer board!
  
    unsigned int gate_wfck_highs = 0;
    unsigned int gate_wfck_lows = 0;
    unsigned int sqck_highs = 0;
    unsigned int sqck_lows = 0;

    TickType_t now = xTaskGetTickCount();

    while ((xTaskGetTickCount() - now) < pdMS_TO_TICKS(board_detection_sample_period)) {
        if(digitalReadFast(SQCK)==1) sqck_highs++;
        if(digitalReadFast(SQCK)==0) sqck_lows++;
        if(digitalReadFast(GATE_WFCK)==1) gate_wfck_highs++;
        if(digitalReadFast(GATE_WFCK)==0) gate_wfck_lows++;

        // 1ms interval -> 1000 reads
        vTaskDelay(pdMS_TO_TICKS(board_detection_sample_interval));
    }

    SCQKGateWFCKDriveDataPoint p = createSCQKGateWFCKDataPoint(gate_wfck_highs, gate_wfck_lows, sqck_highs, sqck_lows);

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
                TickType_t timeoutTicks = subq_capture_timeout / (1000000 / configTICK_RATE_HZ);
                
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

    SUBQDriveDataPoint p = createSUBQDataPoint(isDataSector, hasWobbleInCDDASpace, option1, option2, garbageCollectionCheck, isGameDisk, checkingWobble);

    return p;
}

void injectSCEX() {
    boolean pu22mode = false;

    if (xQueuePeek(pu22modeQueue, &pu22mode, pdMS_TO_TICKS(0)) != pdPASS) {}

    digitalWriteFast(arduino::LED_BUILTIN, arduino::HIGH);

    for (unsigned int loop_counter = 0; loop_counter < scex_injection_loops; loop_counter++) {
        // HC-05 waits for a bit of silence (pin low) before it begins decoding
        vTaskDelay(scex_injection_loop_delay);
        
        for (unsigned int attempts_counter = 0; attempts_counter < scex_injection_attempts; attempts_counter++) {
            for (byte bit_counter = 0; bit_counter < 44; bit_counter++) {
                const char region = 'e';
                if (readBit(bit_counter, region == 'e' ? SCEEData : region == 'a' ? SCEAData : SCEIData ) == false) {
                    pinMode(DATA, arduino::OUTPUT);
                    digitalWriteFast(DATA, arduino::LOW);
                    delayMicroseconds(scex_injection_bits_delay);
                } else {
                    if (pu22mode) {
                        pinMode(DATA, arduino::OUTPUT);
                        unsigned long now = micros();
                        do {
                            digitalWriteFast(DATA, digitalReadFast(GATE_WFCK));
                        } while ((micros() - now) < scex_injection_bits_delay);
                    } else {
                        pinMode(DATA, arduino::INPUT);
                        delayMicroseconds(scex_injection_bits_delay);
                    }
                }
            }

            pinMode(DATA, arduino::OUTPUT);
            digitalWriteFast(DATA, arduino::LOW);
        }
    }

    if (!pu22mode) {
        pinMode(GATE_WFCK, arduino::INPUT);
    }

    pinMode(DATA, arduino::INPUT);
    digitalWriteFast(arduino::LED_BUILTIN, arduino::LOW);
    sendRTOSMsg("SCEX injection completed");
}

void printStats() {
    TaskStatus_t taskStatusArray[MAX_TASK_NUM];
    UBaseType_t taskCount;
    uint32_t totalRunTime;

    taskCount = uxTaskGetSystemState(taskStatusArray, MAX_TASK_NUM, &totalRunTime);

    for (UBaseType_t i = 0; i < taskCount; i++) {
        char log[MAX_MSG_SIZE];

        sniprintf(log, MAX_MSG_SIZE, "Task: %s\t State: %d\t Priority: %lu\t Stack: %lu\t  Runtime: %lu",
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
    if (result == pdPASS){
        return;
    }

    Serial.print("Task creation failed!\n");
    for (;;);
}

static void ThreadLogger(void*) {
    char receivedMessage[MAX_MSG_SIZE];

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
        vTaskDelay(250);
    }
}

static void ThreadPower(void*) {
    SCQKGateWFCKDriveDataPoint p;
    for (;;) {
        if (xQueuePeek(scqkwfckDriveDataQueue, &p, pdMS_TO_TICKS(0)) == pdPASS) {
            checkPower(p.sqck_highs);
            vTaskDelay(250);
        }
    }
}

static void ThreadPu22mode(void*) {
    boolean power;
    SCQKGateWFCKDriveDataPoint p;

    for (;;) {
        if(xQueuePeek(powerQueue, &power, pdMS_TO_TICKS(0)) == pdPASS) {
            if (power) {
                if (xQueuePeek(scqkwfckDriveDataQueue, &p, pdMS_TO_TICKS(0)) == pdPASS) {
                    checkPu22mode(p.gate_wfck_lows);
                    vTaskDelay(250);
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

   for (;;) {
        xQueueReceive(subqRawDataQueue, &scbuf, portMAX_DELAY);
        p = parseSUBQPacket(scbuf_ptr);

        if (p.isGameDisk && p.checkingWobble) {
            xSemaphoreGive(injectScexSem);
            sendRTOSMsg("Allowing SCEX inject");
        }
    
        xQueueOverwrite(subqDriveDataQueue, &p);
    }
}

static void ThreadInjectSCEX(void*) {
   for (;;) {
        xSemaphoreTake(injectScexSem, portMAX_DELAY);
        injectSCEX();
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
            snprintf(subqDriveDataStr, subqDriveDataStrLenght, "game: %d, wobble: %d", p1.isGameDisk, p1.checkingWobble);
            sendRTOSMsg(subqDriveDataStr);
        }

        /*
        // With malloc
        taskENTER_CRITICAL();
        char *msg = formatStringMalloc("with malloc: %d", 10);
        sendRTOSMsg(msg);
        vPortFree(msg);
        taskEXIT_CRITICAL();
        */

        vTaskDelay(pdMS_TO_TICKS(5000));
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

    if (DEBUG) {
        Serial.begin(0);
        Serial.println(PSTR("Booting FreeRTOS kernel " tskKERNEL_VERSION_NUMBER ""));
        Serial.println(PSTR("Built by gcc " __VERSION__ " (newlib " _NEWLIB_VERSION ") on " __DATE__ ""));
        if (CrashReport) {
            Serial.print(CrashReport);
            Serial.println();
            Serial.flush();
        }
    }

    loggerQueue = xQueueCreate(LOGGER_BUFFER_SIZE, MAX_MSG_SIZE);
    scqkwfckDriveDataQueue = xQueueCreate(1, sizeof(SCQKGateWFCKDriveDataPoint));
    subqDriveDataQueue = xQueueCreate(1, sizeof(SUBQDriveDataPoint));
    subqRawDataQueue = xQueueCreate(SUBQ_QUEUE_SIZE, sizeof(byte) * SUBQ_PACKET_LENGTH);
    pu22modeQueue = xQueueCreate(1, sizeof(boolean));
    powerQueue = xQueueCreate(1, sizeof(boolean));
    patchBiosSem = xSemaphoreCreateCounting(1, 0);
    injectScexSem = xSemaphoreCreateCounting(1, 0);

    checkTaskCreation(xTaskCreate(ThreadCaptureSQCKandQFCKData, "sqckqfck", 256, NULL, 1, &tCaptureSQCKandQFCKDataHandler));
    checkTaskCreation(xTaskCreate(ThreadPower, "power", 256, NULL, 1, &tPowerHandler));
    checkTaskCreation(xTaskCreate(ThreadPu22mode, "pu22mode", 256, NULL, 1, &tPu22modeHandler));
    checkTaskCreation(xTaskCreate(ThreadPalBiosPatch, "bios", 256, NULL, 1, &tPalBiosPatchHandler));
    checkTaskCreation(xTaskCreate(ThreadCaptureSUBQPackets, "subq", 256, NULL, 1, &tCaptureSUBQPacketsHandler));
    checkTaskCreation(xTaskCreate(ThreadCheckSUBQWobleArea, "wobble", 256, NULL, 1, &tCheckSUBQWobleAreaHandler));
    checkTaskCreation(xTaskCreate(ThreadInjectSCEX, "scex", 256, NULL, 1, &tInjectSCEXHandler));

    if (DEBUG) {
        checkTaskCreation(xTaskCreate(ThreadLogger, "logger", 512, NULL, 1, &tLoggerHandler));
        //checkTaskCreation(xTaskCreate(ThreadPrintHexSUBQPackets, "hex", 256, NULL, 1, &tPrintHexSUBQPacketsHandler));
        //checkTaskCreation(xTaskCreate(ThreadStats, "stats", 512, NULL, 1, &tStatsHandler));
        //checkTaskCreation(xTaskCreate(ThreadDebug, "debug", 128, NULL, 1, &tDebugHandler));
        //checkTaskCreation(xTaskCreate(ThreadBlinker, "blinker", 128, NULL, 1, &tBlinkerHandler));
    }

    if (DEBUG) {
        Serial.println("Starting RTOS scheduler");
        Serial.flush();
    }

    vTaskStartScheduler();
}

int main() {
    setup();

    return 1;
}
