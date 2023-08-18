#include <Arduino.h>
#include <FreeRTOS_TEENSY4.h>

#define debug

#if defined debug
    #define debug_begin(x)      Serial.begin(x)
    #define debug_print(...)    if (Serial.availableForWrite()) { Serial.print(__VA_ARGS__); }
    #define debug_printf(...)   if (Serial.availableForWrite()) { Serial.printf(__VA_ARGS__); }
    #define debug_println(...)  if (Serial.availableForWrite()) { Serial.println(__VA_ARGS__); }
#else
    #define debug_begin(x)
    #define debug(x)
    #define debugln(x)
#endif

#define BIOS_A18 4      // connect to PSOne BIOS A18 (pin 31 on that chip)
#define BIOS_D2  5      // connect to PSOne BIOS D2 (pin 15 on that chip)
#define SQCK 6          // connect to PSX HC-05 SQCK pin 26 (PU-7 and early PU-8 Mechacons: pin 41)
#define SUBQ 7          // connect to PSX HC-05 SUBQ pin 24 (PU-7 and early PU-8 Mechacons: pin 39)  
#define DATA 8          // connect to point 6 in old modchip diagrams
#define GATE_WFCK 9     // connect to point 5 in old modchip diagrams

static const unsigned char SCEEData[] = {0b01011001, 0b11001001, 0b01001011, 0b01011101, 0b11101010, 0b00000010};
static const unsigned char SCEAData[] = {0b01011001, 0b11001001, 0b01001011, 0b01011101, 0b11111010, 0b00000010};
static const unsigned char SCEIData[] = {0b01011001, 0b11001001, 0b01001011, 0b01011101, 0b11011010, 0b00000010};

// microseconds 250 bits/s (ATtiny 8Mhz works from 3950 to 4100)
static const int scex_injection_bits_delay = 4000;

// milliseconds 72 in oldcrow. PU-22+ work best with 80 to 100
static const int scex_injection_loop_delay = 90;

// 2 cycles seems optimal to cover all boards
static const int scex_injection_loops = 2;

// 3 attempts seems optimal to cover all boards
static const int scex_injection_attempts = 3;

// microseconds
static const int subq_capture_timeout = 2000;

// syntetic value
static const int subq_woble_hysteresis = 14;

// milliseconds
static const int board_detection_sample_period = 1000;

// milliseconds
static const int board_detection_sample_interval = 1;

// readings in board_detection_sample_period
static const int board_detection_gate_wfck_lows_threshold = 20;

// readings in board_detection_sample_period
static const int board_detection_sqck_highs_threshold = 100;

// milliseconds
static const int bios_patch_stage1_delay = 1350;

// microseconds
static const int bios_patch_stage2_delay = 17;

// microseconds
static const int bios_patch_stage3_delay = 4;

// milliseconds
static const int bios_patch_timeout = 5000;

static const int SUBQ_PACKET_LENGTH = 12;
static const int SUBQ_QUEUE_SIZE = 10;

boolean power = false;
boolean pu22mode = false;
boolean isPSXGameDisk = false;
boolean isInCDDASpace = false;
boolean allowSCEXInject = false;
byte hysteresis = 0;
QueueHandle_t subqReadsQueue;
SemaphoreHandle_t injectScexSem;
SemaphoreHandle_t patchBiosSem;
TaskHandle_t t0Handler;
TaskHandle_t t1Handler;
TaskHandle_t t2Handler;
TaskHandle_t t3Handler;
TaskHandle_t t4Handler;
TaskHandle_t t5Handler;

bool readBit(int index, const unsigned char *ByteSet) {
    int byte_index = index >> 3;
    byte bits = ByteSet[byte_index];
    // same as (index - byte_index<<3) or (index%8)
    int bit_index = index & 0x7;
    byte mask = 1 << bit_index;
    return (0 != (bits & mask));
}

void setupPatchBios() {
    digitalWriteFast(LED_BUILTIN, HIGH);
    DDRD &= B11110111;
    DDRD &= B11101111;
    //pinMode(BIOS_A18, INPUT);
    //pinMode(BIOS_D2, INPUT);
}

bool waitA18IntroPulse() {
    debug_println("Wait for stage 1 A18 intro pulse");
    
    uint32_t now = millis();

    while (!digitalReadFast(BIOS_A18)) {
        if((millis() - now) > bios_patch_timeout) {
            debug_println("A18 intro pulse not found");
            return false;
        }
    };

    return true;
}

bool stage1PatchBios(bool thread) {	
    if (!waitA18IntroPulse()) {
        return false;
    }

    debug_println("Wait through stage 1 of A18 activity");
    thread ? vTaskDelay(bios_patch_stage1_delay) : delay(bios_patch_stage1_delay);

    if (!waitA18IntroPulse()) {
        return false;
    }

    return true;
}

void stage2PatchBios() {
    // max 17us for 16Mhz ATmega (maximize this when tuning!)
    delayMicroseconds(bios_patch_stage2_delay);
    digitalWriteFast(BIOS_A18, LOW);
    digitalWriteFast(BIOS_D2, HIGH);
}

void stage3PatchBios() {
    // min 2us for 16Mhz ATmega, 8Mhz requires 3us (minimize this when tuning, after maximizing first us delay!)
    delayMicroseconds(bios_patch_stage3_delay);
    digitalWriteFast(BIOS_D2, LOW);
}

void clearPatchBios() {
    DDRD &= B11110111;
    DDRD &= B11101111;
    //pinMode(BIOS_A18, INPUT);
    //pinMode(BIOS_D2, INPUT);
    digitalWriteFast(LED_BUILTIN, LOW);
}

void FASTRUN patchBios(bool thread) {
    debug_println("Init PAL BIOS patch");

    setupPatchBios();
    
    if (!stage1PatchBios(thread)) {
        debug_println("Quitting BIOS patch");
        clearPatchBios();
        return;
    }
    
    stage2PatchBios();
    stage3PatchBios();

    clearPatchBios();
    
    debug_println("Completed PAL BIOS patch");
}

void boardDetection() {
    unsigned long now = millis();
    unsigned int gate_wfck_highs = 0;
    unsigned int gate_wfck_lows = 0;
    unsigned int sqck_highs = 0;
    unsigned int sqck_lows = 0;
    boolean pu22mode_tmp;
    boolean power_tmp;

    // Board detection
    //
    // GATE: __-----------------------  // this is a PU-7 .. PU-20 board!
    //
    // WFCK: __-_-_-_-_-_-_-_-_-_-_-_-  // this is a PU-22 or newer board!
  
    while ((millis() - now) < board_detection_sample_period) {
        if(digitalReadFast(SQCK)==1) sqck_highs++;
        if(digitalReadFast(SQCK)==0) sqck_lows++;
        if(digitalReadFast(GATE_WFCK)==1) gate_wfck_highs++;
        if(digitalReadFast(GATE_WFCK)==0) gate_wfck_lows++;
        // 1ms interval -> 1000 reads
        vTaskDelay(board_detection_sample_interval);
    }

    power_tmp = sqck_highs > board_detection_sqck_highs_threshold ? true : false;
    
    if(power == false && power_tmp == true) {
        xSemaphoreGive(patchBiosSem);
    }

    power = power_tmp;
    
    pu22mode_tmp = gate_wfck_lows > board_detection_gate_wfck_lows_threshold ? true : false;
    pu22mode = pu22mode_tmp;
}

void captureSUBQPackets(byte *scbuf_ptr) {
    uint32_t now = micros();
    byte bitbuf = 0;
    bool sample = 0;

    for (byte scpos = 0; scpos < SUBQ_PACKET_LENGTH; scpos++) {
        for (byte bitpos = 0; bitpos < 8; bitpos++) {
            while (digitalReadFast(SQCK) == 1) {
                // wait for clock to go high
                // timeout resets the 12 byte stream in case the PSX sends malformatted clock pulses, as happens on bootup
                if((micros() - now) > subq_capture_timeout) {
                    // reset SUBQ packet stream
                    scpos = 0;
                    bitbuf = 0;
                    now = micros();
                    continue;
                }
            }

            // wait for clock to go low
            while ((digitalReadFast(SQCK)) == 0);
            
            sample = digitalReadFast(SUBQ);
            bitbuf |= sample << bitpos;

            // no problem with this bit
            now = micros();
        }

        scbuf_ptr[scpos] = bitbuf;
        bitbuf = 0;
    }
}

void printSUBQPackets(byte *scbuf_ptr) {
    for (byte scpos = 0; scpos < SUBQ_PACKET_LENGTH; scpos++) {
        if (scbuf_ptr[scpos] < 0x10) {
            debug_print("0");
        }

        debug_print(scbuf_ptr[scpos], HEX);
        debug_print(" ");
    }

    debug_println("");
}

void checkWobbleArea(byte *scbuf_ptr) {
    // check if read head is in wobble area
    // We only want to unlock game discs (0x41) and only if the read head is in the outer TOC area.
    // We want to see a TOC sector repeatedly before injecting (helps with timing and marginal lasers).
    // All this logic is because we don't know if the HC-05 is actually processing a getSCEX() command.
    // Hysteresis is used because older drives exhibit more variation in read head positioning.
    // While the laser lens moves to correct for the error, they can pick up a few TOC sectors.

    boolean isDataSector = (((scbuf_ptr[0] & 0x40) == 0x40) && (((scbuf_ptr[0] & 0x10) == 0) && ((scbuf_ptr[0] & 0x80) == 0)));
    boolean hasWobbleInCDDASpace = (isDataSector || scbuf_ptr[0] == 0x01);				// [0] = 0x41 (psx game disk) then goto 0x01
    boolean option1 = scbuf_ptr[2] == 0xA0 || scbuf_ptr[2] == 0xA1 || scbuf_ptr[2] == 0xA2;		// if [2] = A0, A1, A2 ..
    boolean option2 = scbuf_ptr[2] == 0x01 && (scbuf_ptr[3] >= 0x98 || scbuf_ptr[3] <= 0x02);	// .. or = 01 but then [3] is either > 98 or < 02
    boolean garbageCollectionCheck = scbuf_ptr[1] == 0x00 && scbuf_ptr[6] == 0x00;			// Garbage collection checks

    if (isDataSector && (option1 || option2) && garbageCollectionCheck) {
        isPSXGameDisk = true;
        hysteresis++;
    } else if (hysteresis > 0 && hasWobbleInCDDASpace && garbageCollectionCheck) {
        isInCDDASpace = true;
        hysteresis++;
    } else {
        isPSXGameDisk = false;
        isInCDDASpace = false;
        if (hysteresis > 0) {
            hysteresis--;
        }
    }
    
    if (hysteresis >= subq_woble_hysteresis) {
        allowSCEXInject = true;
        hysteresis = 11;
        xSemaphoreGive(injectScexSem);
    } else {
        allowSCEXInject = false;
    }
}

void injectSCEX() {
    // HC-05 waits for a bit of silence (pin low) before it begins decoding
    vTaskDelay(scex_injection_loop_delay);
    
    digitalWriteFast(LED_BUILTIN, HIGH);

    for (unsigned int loop_counter = 0; loop_counter < scex_injection_loops; loop_counter++) {
        for (unsigned int attempts_counter = 0; attempts_counter < scex_injection_attempts; attempts_counter++) {
            for (byte bit_counter = 0; bit_counter < 44; bit_counter++) {
                const char region = 'e';
                if (readBit(bit_counter, region == 'e' ? SCEEData : region == 'a' ? SCEAData : SCEIData ) == false) {
                    DDRB |= B00000001;
                    //pinModeFast(DATA, OUTPUT);
                    digitalWriteFast(DATA, LOW);
                    delayMicroseconds(scex_injection_bits_delay);
                } else {
                    if (pu22mode) {
                        DDRB |= B00000001;
                        //pinModeFast(DATA, OUTPUT);
                        unsigned long now = micros();
                        do {
                            // output wfck signal on data pin
                            digitalWriteFast(DATA, digitalReadFast(GATE_WFCK));
                        } while ((micros() - now) < scex_injection_bits_delay);
                    } else {
                        DDRB &= B11111110;
                        //pinModeFast(DATA, INPUT);
                        delayMicroseconds(scex_injection_bits_delay);
                    }
                }
            }

            DDRB |= B00000001;
            //pinModeFast(DATA, OUTPUT);
            digitalWriteFast(DATA, LOW);
        }

        vTaskDelay(scex_injection_loop_delay);
    }

    if (!pu22mode) {
        DDRD &= B01111111;
        //pinModeFast(GATE_WFCK, INPUT);
    }

    DDRB &= B11111110;
    //pinModeFast(DATA, INPUT);

    digitalWriteFast(LED_BUILTIN, LOW);
}

static void Thread0(void* arg) {
     debug_println("Thread 0 : Board Detection");

    while (1) {
        boardDetection();
    }
}

static void Thread1(void* arg) {
    debug_println("Thread 1 : Patch Bios");

    while(1) {
        xSemaphoreTake(patchBiosSem, portMAX_DELAY);
        patchBios(true);
    }
}

static void Thread2(void* arg) {
    byte scbuf [SUBQ_PACKET_LENGTH] = { 0 };
    byte *scbuf_ptr = scbuf;

    debug_println("Thread 2 : Capture SUBQ Packets");
    
    while (1) {
        captureSUBQPackets(scbuf_ptr);
        printSUBQPackets(scbuf_ptr);
        xQueueSend(subqReadsQueue, scbuf, portMAX_DELAY);
    }
}

static void Thread3(void* arg) {
    byte scbuf [SUBQ_PACKET_LENGTH] = { 0 };
    byte *scbuf_ptr = scbuf;

    debug_println("Thread 3 : Check Woble Area");

    while (1) {
        xQueueReceive(subqReadsQueue, scbuf, portMAX_DELAY);
        checkWobbleArea(scbuf_ptr);
    }
}

static void Thread4(void* arg) {
    debug_println("Thread 4 : Inject SCEX");
        
    while (1) {
        xSemaphoreTake(injectScexSem, portMAX_DELAY);
        injectSCEX();
    }
}

static void Thread5(void* arg) {
    debug_println("Thread 5 : Print to Serial");
    
    while (1) {    
        debug_printf("power: %b\n", power);
        debug_printf("pu22mode: %b\n", pu22mode);
        debug_printf("isPSXGameDisk: %b\n", isPSXGameDisk);
        debug_printf("isInCDDASpace: %b\n", isInCDDASpace);
        debug_printf("hysteresis: &d\n", hysteresis);
        debug_printf("allowSCEXInject: %b\n", allowSCEXInject);
        vTaskDelay(1000);
    }
}

void setup() {
    pinMode(SQCK, INPUT);
    pinMode(SUBQ, INPUT);
    pinMode(DATA, INPUT);
    pinMode(GATE_WFCK, INPUT);
    pinMode(LED_BUILTIN, OUTPUT);

    debug_begin(115200);
    debug_println("Initializing RTOS tasks");

    subqReadsQueue = xQueueCreate(SUBQ_QUEUE_SIZE, sizeof(byte[SUBQ_PACKET_LENGTH]));
    patchBiosSem = xSemaphoreCreateCounting(1, 0);
    injectScexSem = xSemaphoreCreateCounting(1, 0);

    portBASE_TYPE t0 = xTaskCreate(Thread0, "detectBoardType", configMINIMAL_STACK_SIZE, NULL, 3, &t0Handler);
    portBASE_TYPE t1 = xTaskCreate(Thread1, "palBiosPatch", configMINIMAL_STACK_SIZE, NULL, 3, &t1Handler);
    portBASE_TYPE t2 = xTaskCreate(Thread2, "captureSUBQPackets", configMINIMAL_STACK_SIZE, NULL, 1, &t2Handler);
    portBASE_TYPE t3 = xTaskCreate(Thread3, "checkSUBQWobleArea", configMINIMAL_STACK_SIZE, NULL, 2, &t3Handler);
    portBASE_TYPE t4 = xTaskCreate(Thread4, "injectSCEXloop", configMINIMAL_STACK_SIZE, NULL, 2, &t4Handler);
    portBASE_TYPE t5 = xTaskCreate(Thread5, "printToSerial", configMINIMAL_STACK_SIZE, NULL, 3, &t5Handler);

    if (subqReadsQueue == NULL ||
        injectScexSem == NULL ||
        patchBiosSem == NULL ||
        t0 != pdPASS ||
        t1 != pdPASS ||
        t2 != pdPASS ||
        t3 != pdPASS ||
        t4 != pdPASS ||
        t5 != pdPASS
        ) {
        debug_println("Task creation problem");
        return;
    }

    debug_println("Starting RTOS scheduler");
    vTaskStartScheduler();
}

int main() {
    setup();

    debug_println("Something went wrong");

    return 1;
}
