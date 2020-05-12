#include <Arduino.h>
#include <FreeRTOS_TEENSY4.h>

#define PRINT(x) Serial.availableForWrite() ? Serial.print(x) : 0
#define PRINT_HEX(x) Serial.availableForWrite() ? Serial.print(x, HEX) : 0
#define PRINTLN(x) Serial.availableForWrite() ? Serial.println(x) : 0
#define PRINTLN_HEX(x) Serial.availableForWrite() ? Serial.println(x, HEX) : 0

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


boolean power = false;
boolean pu22mode = false;
QueueHandle_t subqReadsQueue;
SemaphoreHandle_t injectScexSem;
SemaphoreHandle_t patchBiosSem;
TaskHandle_t t0Handler;
TaskHandle_t t1Handler;
TaskHandle_t t2Handler;
TaskHandle_t t3Handler;
TaskHandle_t t4Handler;

bool readBit(int index, const unsigned char *ByteSet) {
	int byte_index = index >> 3;
	byte bits = ByteSet[byte_index];
	// same as (index - byte_index<<3) or (index%8)
	int bit_index = index & 0x7;
	byte mask = 1 << bit_index;
	return (0 != (bits & mask));
}

void FASTRUN patchBios(bool thread) {
	PRINTLN("Init Pal BIOS patch");
	digitalWriteFast(LED_BUILTIN, HIGH);
	
	DDRD &= B11110111;
	DDRD &= B11101111;
	//pinMode(BIOS_A18, INPUT);
	//pinMode(BIOS_D2, INPUT);

	uint32_t now = millis();
	
	PRINTLN("Wait for stage 1 A18 intro pulse");
	while (!digitalReadFast(BIOS_A18)) {
		if((millis() - now) > bios_patch_timeout) {
			PRINTLN("A18 intro pulse not found - Quitting Bios patch");
            DDRD &= B11110111;
            DDRD &= B11101111;
            //pinMode(BIOS_A18, INPUT);
            //pinMode(BIOS_D2, INPUT);
			digitalWriteFast(LED_BUILTIN, LOW);
			return;
		}
	};

	PRINTLN("Wait through stage 1 of A18 activity");
	thread ? vTaskDelay(bios_patch_stage1_delay) : delay(bios_patch_stage1_delay);

	now = millis();
	
	PRINTLN("Wait for priming A18 pulse");
	while (!digitalReadFast(BIOS_A18)) {
		if((millis() - now) > bios_patch_timeout) {
			PRINTLN("A18 priming pulse not found - Quitting Bios patch");
            DDRD &= B11110111;
            DDRD &= B11101111;
            //pinMode(BIOS_A18, INPUT);
            //pinMode(BIOS_D2, INPUT);
			digitalWriteFast(LED_BUILTIN, LOW);
			return;
		}
	}

	delayMicroseconds(bios_patch_stage2_delay); // max 17us for 16Mhz ATmega (maximize this when tuning!)
	digitalWriteFast(BIOS_A18, LOW);
	digitalWriteFast(BIOS_D2, HIGH);
	delayMicroseconds(bios_patch_stage3_delay); // min 2us for 16Mhz ATmega, 8Mhz requires 3us (minimize this when tuning, after maximizing first us delay!)
	digitalWriteFast(BIOS_D2, LOW);

	DDRD &= B11110111;
	DDRD &= B11101111;
	//pinMode(BIOS_A18, INPUT);
	//pinMode(BIOS_D2, INPUT);

	PRINTLN("Done Pal BIOS patch");
	digitalWriteFast(LED_BUILTIN, LOW);
}

static void Thread0(void* arg) {
	unsigned long now = millis();
	unsigned int gate_wfck_highs = 0;
	unsigned int gate_wfck_lows = 0;
	unsigned int sqck_highs = 0;
	unsigned int sqck_lows = 0;
	boolean mode_tmp;
	boolean power_tmp;

	//pinMode(SQCK, INPUT);
	//pinMode(GATE_WFCK, INPUT);
	
 	PRINTLN("Thread 0 : Board Detection");

	while (1) {
		for(;;) {

			// Board detection
			//
			// GATE: __-----------------------  // this is a PU-7 .. PU-20 board!
			//
			// WFCK: __-_-_-_-_-_-_-_-_-_-_-_-  // this is a PU-22 or newer board!
			
			now = millis();
			sqck_highs = 0;
			sqck_lows = 0;
			gate_wfck_lows = 0;
			gate_wfck_highs = 0;
			gate_wfck_lows = 0;

			while ((millis() - now) < board_detection_sample_period) {
				if(digitalReadFast(SQCK)==1) sqck_highs++;
				if(digitalReadFast(SQCK)==0) sqck_lows++;
				if(digitalReadFast(GATE_WFCK)==1) gate_wfck_highs++;
				if(digitalReadFast(GATE_WFCK)==0) gate_wfck_lows++;
				//1ms interval -> 1000 reads
				vTaskDelay(board_detection_sample_interval);
			}

			power_tmp = sqck_highs > board_detection_sqck_highs_threshold ? true : false;
			
			if(power == false && power_tmp == true) {
				xSemaphoreGive(patchBiosSem);
			}

			power = power_tmp;

			PRINT("sqck_highs: ");
			PRINTLN(sqck_highs);
			PRINT("sqck_lows: ");
			PRINTLN(sqck_lows);
			PRINT("power: ");
			PRINTLN(power_tmp);
			
			mode_tmp = gate_wfck_lows > board_detection_gate_wfck_lows_threshold ? true : false;
			pu22mode = mode_tmp;
			
			PRINT("gate_wfck_lows: ");
			PRINTLN(gate_wfck_lows);
			PRINT("mode: ");
			PRINTLN(mode_tmp);
		}
	}
}

static void Thread1(void* arg) {
	PRINTLN("Thread 1 : Patch Bios");
	while(1) {
		for(;;) {
			xSemaphoreTake(patchBiosSem, portMAX_DELAY);
			patchBios(true);
		}
	}
}

static void Thread2(void* arg) {
	byte scbuf [12] = { 0 };
	byte bitbuf = 0;
	bool sample = 0;

	uint32_t now;

	//pinMode(SUBQ, INPUT);

	PRINTLN("Thread 2 : Capture SUBQ Packets");
	
	while (1) {
		for(;;) {
			now = micros();
			bitbuf = 0;
			sample = 0;

			for (byte scpos = 0; scpos < 12; scpos++) {
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
	
				scbuf[scpos] = bitbuf;
				bitbuf = 0;
			}

			for (byte scpos = 0; scpos < 12; scpos++) {
				if (scbuf[scpos] < 0x10) {
					PRINT("0");
				}
				//printhex(scbuf[scpos], HEX);
				PRINT_HEX(scbuf[scpos]);
				PRINT(" ");
			}
			PRINTLN("");
			
			xQueueSend(subqReadsQueue, scbuf, portMAX_DELAY);
		}
	}
}

static void Thread3(void* arg) {
	byte scbuf [12] = { 0 };
	byte hysteresis  = 0;

	PRINTLN("Thread 3 : Check Woble Area");

	while (1) {
		for(;;) {
			// check if read head is in wobble area
			// We only want to unlock game discs (0x41) and only if the read head is in the outer TOC area.
			// We want to see a TOC sector repeatedly before injecting (helps with timing and marginal lasers).
			// All this logic is because we don't know if the HC-05 is actually processing a getSCEX() command.
			// Hysteresis is used because older drives exhibit more variation in read head positioning.
			// While the laser lens moves to correct for the error, they can pick up a few TOC sectors.

			xQueueReceive(subqReadsQueue, scbuf, portMAX_DELAY);

			boolean isDataSector = (((scbuf[0] & 0x40) == 0x40) && (((scbuf[0] & 0x10) == 0) && ((scbuf[0] & 0x80) == 0)));

			if (
				(isDataSector &&  scbuf[1] == 0x00 &&  scbuf[6] == 0x00) &&   // [0] = 41 means psx game disk. the other 2 checks are garbage protection
				(scbuf[2] == 0xA0 || scbuf[2] == 0xA1 || scbuf[2] == 0xA2 ||  // if [2] = A0, A1, A2 ..
				(scbuf[2] == 0x01 && (scbuf[3] >= 0x98 || scbuf[3] <= 0x02))) // .. or = 01 but then [3] is either > 98 or < 02
			) {
				hysteresis++;
			} else if (hysteresis > 0 &&
				((scbuf[0] == 0x01 || isDataSector) && (scbuf[1] == 0x00) &&  scbuf[6] == 0x00)
			) {
				// This CD has the wobble into CD-DA space
				// started at 0x41, then went into 0x01
				hysteresis++;
			} else if (hysteresis > 0) {
				// None of the above
				// Initial detection was noise
				// Decrease the counter
				hysteresis--; 
			}

			PRINT("hysteresis: ");
			PRINTLN(hysteresis);

			if (hysteresis >= subq_woble_hysteresis) {
				PRINTLN("Is in woble area");
				hysteresis = 11;

				xSemaphoreGive(injectScexSem);
				continue;
			}

			PRINTLN("Not in woble area");
		}
	}
}

static void Thread4(void* arg) {
	//pinMode(LED_BUILTIN, OUTPUT);
	//pinMode(DATA, INPUT);

	PRINTLN("Thread 4 : Inject SCEX");
		
	while (1) {
		for(;;) {
			xSemaphoreTake(injectScexSem, portMAX_DELAY);
			
			// HC-05 waits for a bit of silence (pin low) before it begins decoding
			vTaskDelay(scex_injection_loop_delay);
			
			digitalWriteFast(LED_BUILTIN, HIGH);

			for (unsigned int loop_counter = 0; loop_counter < scex_injection_loops; loop_counter++) {
				for (unsigned int attempts_counter = 0; attempts_counter < scex_injection_attempts; attempts_counter++) {
					for (byte bit_counter = 0; bit_counter < 44; bit_counter++) {
						const char region = 'e';
						if (readBit(bit_counter, region == 'e' ? SCEEData : region == 'a' ? SCEAData : SCEIData ) == false) {
							DDRB |= B00000001;
							//DDRB |= 0;
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
	}
}

void setup() {
	Serial.begin(115200);

	pinMode(SQCK, INPUT);
	pinMode(SUBQ, INPUT);
	pinMode(DATA, INPUT);
	pinMode(GATE_WFCK, INPUT);
	pinMode(LED_BUILTIN, OUTPUT);

	subqReadsQueue = xQueueCreate(10, sizeof(byte[12]));
	patchBiosSem = xSemaphoreCreateCounting(1, 0);
	injectScexSem = xSemaphoreCreateCounting(1, 0);

	portBASE_TYPE t0 = xTaskCreate(Thread0, "detectBoardType", configMINIMAL_STACK_SIZE, NULL, 3, &t0Handler);
	portBASE_TYPE t1 = xTaskCreate(Thread1, "palBiosPatch", configMINIMAL_STACK_SIZE, NULL, 3, &t1Handler);
	portBASE_TYPE t2 = xTaskCreate(Thread2, "captureSUBQPackets", configMINIMAL_STACK_SIZE, NULL, 1, &t2Handler);
	portBASE_TYPE t3 = xTaskCreate(Thread3, "checkSUBQWobleArea", configMINIMAL_STACK_SIZE, NULL, 2, &t3Handler);
	portBASE_TYPE t4 = xTaskCreate(Thread4, "injectSCEXloop", configMINIMAL_STACK_SIZE, NULL, 2, &t4Handler);

	if (subqReadsQueue == NULL ||
		injectScexSem == NULL ||
		patchBiosSem == NULL ||
		t0 != pdPASS ||
		t1 != pdPASS ||
		t2 != pdPASS ||
		t3 != pdPASS ||
		t4 != pdPASS
		) {
		PRINTLN("Task creation problem");
		while(1);
	}

	// Cold start
	power = true; 
	patchBios(false); 
	
	PRINTLN("Starting RTOS");
	vTaskStartScheduler();
}

void loop() {
	PRINTLN("FAIL");
	while(1);
}

int main() {
	setup();
	loop();

	return 1;
}
