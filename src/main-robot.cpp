// include
#include <Arduino.h>
#include <EEPROM.h>
#include <SMSBL.h>
#include <stdint.h>
// #include <SMSCL.h>
#include <MFRC522.h>  //for RFID-Module
#include <SD.h>       //for SD-card reading
#include <SPI.h>      //for communication with SD-Module and RFID-Reader

#include "HARDWARE_VERSION.h"  // define (uncomment) the hardware version / configuration here to adapt the correct pins:
#include "debounce.h"          // button debouncing and press detection. Define the pins for the buttons in this file!

/* 16.02.2021 Changes in this version:
 */
// defines
//  version string that will be printed to PC via UART at startup.
#define FIRMWARE_VERSION_STRING "1.8.15"

// hardware dependent definitions (change in HARDWARE_VERSION.h):
#ifdef HARDWARE_V1
#define SERVO_POWER_PIN 20  // This pin controls the MOSFET that switches on the power supply of the servos.
// LEDs
#define LED_TEACH_PIN 13
#define LED_PLAY_PIN 12
#define LED_POWER_PIN 11
// Beeper Alarmsignal
#define ALARMSIGNAL_PIN 5
#define ALARMSIGNAL_GND_PIN 6
#endif

#ifdef HARDWARE_V1_PCB_ERGOSURG
#define SERVO_POWER_PIN 12  // This pin controls the MOSFET that switches on the power supply of the servos.
// LEDs
#define LED_TEACH_PIN 11
#define LED_PLAY_PIN 9
#define LED_POWER_PIN 7  // supposed to be on 7
// Beeper Alarmsignal
#define ALARMSIGNAL_PIN 46
#define ALARMSIGNAL_GND_PIN 4
// Pins for Pot / SD-Module / RFID-Module
#define POTI_PIN A7
#define SD_SS_PIN 47
#define RFID_SS_PIN 43
#endif

#ifdef HARDWARE_V2
#define SERVO_POWER_PIN 7  // This pin controls the MOSFET that switches on the power supply of the servos.
// LEDs
#define LED_TEACH_PIN A4
#define LED_PLAY_PIN A3
#define LED_POWER_PIN A5
#define LED_GND_PIN A6  // in this version, one pin is configured as GND (output and LOW) for LEDs
// Button GND
#define BUTTON_GND_PIN A7  // in this version, one pin is configured as GND (output and LOW) for buttons
// Beeper Alarmsignal
#define ALARMSIGNAL_PIN 5
#define ALARMSIGNAL_GND_PIN 6
#endif

#ifdef HARDWARE_V2_MAKITA
#define SERVO_POWER_PIN 7  // This pin controls the MOSFET that switches on the power supply of the servos.
// LEDs
#define LED_TEACH_PIN A5
#define LED_PLAY_PIN A4
#define LED_POWER_PIN A3
#define LED_GND_PIN A1  // in this version, one pin is configured as GND (output and LOW) for LEDs
// Button GND
#define BUTTON_GND_PIN A0  // in this version, one pin is configured as GND (output and LOW) for buttons
// Beeper Alarmsignal
#define ALARMSIGNAL_PIN 5
#define ALARMSIGNAL_GND_PIN 6
#endif

#ifdef HARDWARE_V3_PCB_MIMED

// pin controls the MOSFET that switches on the power supply of the servos
#define SERVO_POWER_PIN 7
// LEDs, in this version, one pin is configured as GND (output and LOW) for LEDs
#define LED_PLAY_PIN ADIGI5   // blau ADIGI4
#define LED_TEACH_PIN ADIGI4  // blau ADIGI5
#define LED_POWER_PIN ADIGI3  // blau ADIGI3
#define LED_GND_PIN ADIGI6    // blau ADIGI1
// Button GND, in this version, one pin is configured as GND (output and LOW) for buttons
#define BUTTON_GND_PIN ADIGI7  // blau ADIGI0
// Beeper Alarmsignal
#define ALARMSIGNAL_PIN 5
#define ALARMSIGNAL_GND_PIN 6
#endif

#define BUFFER_SIZE 64             // Size of the Input Buffer for command strings from PC
#define REC_SIZE 500               // maximum number of points for recording: REC_SIZE*4*2byte has to be smaller than memory size
#define REC_INTERVAL 50            // Recording interval in milliseconds for teaching: 50ms->20Hz
#define PRINT_POS_INTERVAL_MS 100  // Interval in milliseconds for read-Mode

#define MAX_SERVO_NUMBER 7    // number of servos for teaching. Adapt this when there are more servos to avoid illegal memory access (array size)!
#define MAX_SERVO_SCAN_ID 20  // max. servo ID that is checked for a responding servo. Make sure each connected servo has an ID that is smaller than this value.

#define PC_BAUD_RATE 500000      // Baudrate for communication with the PC (max. 500000)
#define SERVO_BAUD_RATE 1000000  // Baudrate for communication with the servos. To change this, also all servos have to be configured to work on the new baudrate. Default is 1 MBaud.

#define DEBUG_INFOS  // print debug information if defined. Uncomment to disable
// #define PRINT_LOOP_DURATION_INFO  // if defined, information about the loop() duration will be printed via UART. Uncomment to disable this
// #define DEBUG_ALARMOFF            // no alarm signal if defined. Uncomment to disable

// #define SM_TORQUE_ENABLE 40
// definitions
//  Statemachine:
typedef enum {
	INIT = 0,  // after power on until everything is ready
	// SERVOS_OFF,   // servo power supply is off
	GOTO_IDLE,     // prepare everything for IDLE (reset positions)
	IDLE,          // servos are at zero torque, ready for commands
	GOTO_HOLD,     // prepare everything for HOLD_POS (send position command, after that go to HOLD_POS)
	HOLD_POS,      // hold current position actively, ready for commands
	START_RECORD,  // prepare teach-in mode (init timer and index variables etc.)
	RECORD,        // teach-in mode to learn a new movement
	CHECK_RECORD,  // after finishing recording, check the positions if they are OK
	START_PLAY,    // before starting recorded movement, wait for further button presses
	PLAY,          // play a recorded movement
	MOVE,          // perform a move command from PC
	// READ,         // read the servo positions and send them to the PC
	SET_LIMITS,    // move robot to set position limits
	RESET_LIMITS,  // reset position limits
	BATT_LOW,      // battery is low, warn the user with the buzzer and hold current position. start countdown until shutdown.
	BATT_EMPTY,    // battery almost empty, robot will shut down.
	SHUTDOWN,      // maybe move slowly to save position before shutdown, then turn servo power off and after that turn everything off
	WAIT,          // wait for the specified amount of time before entering next state. For an example see below.
	ERROR,         // default error state if anything is not correct
} LCL_STATE_T;

static LCL_STATE_T lcl_state = INIT;  // the current state of the robot state machine

// FUNCTIONS
/*static float mapfloat(float x, float in_min, float in_max, float out_min, float out_max);
//static void flushSerial();
static void printInfo();
//tatic void setupServo(int servoNum);
//static void record();
//static void play();
//static void setLimits();
//static void setLimit(int servoID);
static void printPos();             // prints the position of all servos over the serial interface to the PC
static void printCommandInfos();    // prints information about the available commands to the PC
static void servos_reboot();        // turns off and on the servo power supply to bring the servos to zero torque mode (e.g. for teaching)
static void servos_torque_enable(); // hold current position
//static void servo_power_on();          // turn servos on
static void alarmsignal_init();
static void alarmsignal_set(uint8_t k, uint32_t t1, uint32_t t2);
static void alarmsignal_run();
static void button_timer_init();
static void chars_from_serial();
static uint8_t parse_serial_move();
static void leds_init();
//static void validate_recording();
static int16_t servo_readPos_w_dummy(uint8_t k);
static void servo_writePos_multiturn(uint8_t k, int32_t position, uint16_t speed);
static void checkTurns();*/

SMSBL servo;  // servo 1,2,4,5
// SMSCL servo3; // servo 3, apparently not needed any more

File TextFile;  // Object of class File to open the Text-File on the SD-card, where the positions are saved

static uint8_t num_servos = 0;               // number of connected servos
static uint8_t servo_ids[MAX_SERVO_NUMBER];  // array for storing the IDs of each servo. (IDs are 8 bit values)

// multiturn mode variables
int servo_turns[MAX_SERVO_NUMBER];               // array for storing the current turn of each servo compared to the last powerup
int16_t last_servo_positions[MAX_SERVO_NUMBER];  // array for storing the last positions of each loop;

// serial read variables
static char inputBuffer[BUFFER_SIZE];  // Input Buffer for command strings from PC
static boolean newData = false;        // true when there is a new string ready to be parsed
static char *ptr;                      //
char delimiter[] = " ;,=";             // define delimiters for string input

// read variables
static uint8_t readstream_flag = 0;  // true when the read stream mode is enabled
static uint32_t readstream_timer = 0;

// read load variables
static uint8_t loadstream_flag = 0;  // true when the loadstream mode is enabled
static uint32_t loadstream_timer = 0;

// Play_flag
static uint8_t Play_finished_flag = 0;

// move variables
static uint32_t move_start = 0;  // time when move command is recognized
static int16_t move_setpos[MAX_SERVO_NUMBER];
static int16_t move_currpos[MAX_SERVO_NUMBER];
static int16_t move_speeds[MAX_SERVO_NUMBER];
static uint32_t move_dura = 1000;
static uint32_t move_delayed = 1000;

static uint8_t pos_reached_count = 0;
static uint8_t pos_reached_flag = 0;

// record variables
static uint8_t record_valid_flag = 0;                    // flag to show if the recorded data is valid
static int16_t record_data[REC_SIZE][MAX_SERVO_NUMBER];  // array for recorded positions in teaching mode
static int16_t first_record_data[MAX_SERVO_NUMBER];      // array for first recorded positions in teaching mode
static uint16_t record_index = 0;                        // array index for recording
static uint16_t record_length = 0;                       // length of recorded data
static uint32_t last_record_time_ms = 0;                 // needed to generate the desired recording frequency
static uint32_t last_checkrecord_time_ms = 0;            // needed to generate the desired recording frequency

static uint16_t eeprom_index = 0;
static uint16_t eeprom_start_index = 3;
static uint16_t eeprom_adress = 0;
static uint16_t eeprom_adress_length = 0;
static uint8_t eeprom_save_flag = 0;
static uint8_t sd_save_flag = 0;

// play variables
static uint16_t play_index = 0;
static uint32_t last_play_time_ms = 0;  // needed to play movement with record frequency
static int8_t play_button_count = 0;    // Parhofer

// set limit variables
static uint16_t setlimit_index = 0;
static uint32_t last_setlimit_time_ms = 0;
static int16_t limit_min[MAX_SERVO_NUMBER];
static int16_t limit_max[MAX_SERVO_NUMBER];

// alarm signal variables
static uint8_t beep_start = 0;
static uint8_t beep_count = 0;
static uint32_t beep_timer_ms = 0;
static uint32_t beep_length_ms = 0;
static uint32_t beep_period_ms = 0;

// current time
static uint32_t current_ms = 0;  // holds the current system time and is updated in every loop iteration

// for waiting before entering a new state, set the desired time value using set_wait_timer() and set lcl_state to WAIT.
// The state that shall be entered after the waiting time has to be written to "lcl_next_state_after_wait";
/* Example:
  set_wait_timer(500);                                // wait 500 ms before entering next state
  lcl_state = WAIT;                                   // go to wait state
  lcl_next_state_after_wait = GOTO_HOLD;              // after waiting, go to GOTO_HOLD
*/
static uint32_t statemachine_waiting_start_ms = 0;    // variable to save the starting time for waiting. Has to be set to the current system time at the beginning of the waiting (done by set_wait_timer())
static uint32_t statemachine_waitingtime_ms = 0;      // desired waiting time in ms
static LCL_STATE_T lcl_next_state_after_wait = INIT;  // for the "wait" state, a waiting time and the next state after this waiting time is needed.

// init RFID
// Pin-numbers for the RFID-Module
constexpr uint8_t RST_PIN_RFID = 39;
constexpr uint8_t SS_PIN_RFID = 43;
// Instance of the class
MFRC522 rfid(SS_PIN_RFID, RST_PIN_RFID);
MFRC522::MIFARE_Key key;

// SETUP
void setup() {
	// init the buttons
	debounce_init();
	button_timer_init();
	// init LEDs
	leds_init();
	// UART for communication with PC
	Serial.begin(PC_BAUD_RATE);
	// UART for communication with servos
	Serial1.begin(SERVO_BAUD_RATE);
	// attach UART to Servo bus
	servo.pSerial = &Serial1;
	// servo3.pSerial = &Serial1;
	alarmsignal_init();

	// print Firmware information
	Serial.println(F("LCL Robot control Firmware. S. Schiele, C. Rehekampff at TU Munich \n- Institute of Microtechnology and Medical Device Technology (Prof. Tim C. Lueth) - www.mimed.de"));
	Serial.println(F(FIRMWARE_VERSION_STRING));
	Serial.println(F("Compiled: " __DATE__ ", " __TIME__ ""));

	// servo pin init and init delay
	pinMode(SERVO_POWER_PIN, OUTPUT);     // config the pin as output that controls the servo power supply (power will be turned on in the INIT-state)
	digitalWrite(SERVO_POWER_PIN, HIGH);  // turn on power supply of the servos
	Serial.print(F("Enabling servos..."));

	// go to WAIT state for 3s to make sure the servos finished their start-up process
	/*   lcl_state = WAIT;
	statemachine_waiting_start_ms = millis();
	statemachine_waitingtime_ms = 3000;
	//then go to INIT state
	lcl_next_state_after_wait = INIT;*/

	// init the POTI INPUT
	pinMode(POTI_PIN, INPUT);
	// Init SPI bus and rfid
	SPI.begin();
	rfid.PCD_Init();  // Init MFRC522

	delay(3000);
	lcl_state = INIT;
}

// MAIN LOOP
void loop() {
#ifdef PRINT_LOOP_DURATION_INFO
	// for testing / debugging: how long did one loop take?
	static uint32_t max_loop_duration_ms = 0;
	static uint32_t min_loop_duration_ms = 1000000;
	static uint32_t loop_duration_print_timer = 0;  // timer variable to print the loop time info only each second

	uint32_t loop_duration_ms = current_ms;  // save last timestamp
#endif
	// read current system time once at the beginning of the loop (in milliseconds)
	current_ms = millis();
#ifdef PRINT_LOOP_DURATION_INFO
	loop_duration_ms = current_ms - loop_duration_ms;  // calculate duration of this loop iteration

	if (loop_duration_ms > max_loop_duration_ms)  // new max. time?
	{
		max_loop_duration_ms = loop_duration_ms;
	}
	if (loop_duration_ms < min_loop_duration_ms)  // new min time?
	{
		min_loop_duration_ms = loop_duration_ms;
	}

	if ((current_ms - loop_duration_print_timer) >= 1000)  // print loop time info once a second
	{
		loop_duration_print_timer = current_ms;
		Serial.print(F("max loop time: "));
		Serial.print(max_loop_duration_ms);
		Serial.println(F(" ms"));

		Serial.print(F("min loop time: "));
		Serial.print(min_loop_duration_ms);
		Serial.println(F(" ms"));

		Serial.print(F("current loop time: "));
		Serial.print(loop_duration_ms);
		Serial.println(F(" ms"));
	}
#endif

	// INPUT COMMANDS 1) Button commands
	// INPUT COMMANDS 1.1) reset all button flags
	uint8_t record_button_flag = 0;
	uint8_t play_button_flag = 0;
	uint8_t play_button_long_flag = 0;
	uint8_t power_button_short_flag = 0;
	// uint8_t power_button_long_flag = 0;

	// INPUT COMMANDS 1.2) before each state machine iteration, all buttons are checked.
	// Button presses are safed into variables.
	// The reaction to the button presses depends on the active state and is done there.
	// if the active state does not react to a pressed button, the button press will be ignored
	if (get_key_press(1 << KEY_START_REC)) {
		record_button_flag = 1;
#ifdef DEBUG_INFOS
		Serial.println(F("record button"));
#endif
		// record();
	}

	if (get_key_short(1 << KEY_PLAY)) {
		if (play_button_count == -1)  // stop infinite repeat
		{
			play_button_count = 0;
		}

		play_button_flag = 1;
		play_button_count++;  // Parhofer

#ifdef DEBUG_INFOS
		Serial.println(F("play button"));
#endif
		// play();
	}

	if (get_key_long(1 << KEY_PLAY)) {
		if (play_button_count == -1)  // stop infinite repeat
		{
			play_button_count = 0;
		} else {
			play_button_flag = 1;
			// play_button_count = 5; //Parhofer
			play_button_count = -1;  // infinite (Parhofer)
#ifdef DEBUG_INFOS
			Serial.println(F("play button long"));
#endif
		}
	}

	if (get_key_short(1 << KEY_POWER)) {
		power_button_short_flag = 1;
#ifdef DEBUG_INFOS
		Serial.println(F("power button short"));
#endif
	}
	/*   if (get_key_long(1 << KEY_POWER))
	  {
	  power_button_long_flag = 1;
	  #ifdef DEBUG_INFOS
	  Serial.println(F("power button long"));
	  #endif
	  } */

	// INPUT COMMANDS: 2) Commands from serial port
	// INPUT COMMANDS: 2.1) reset all serial command flags
	uint8_t move_serial_flag = 0;     //
	uint8_t record_serial_flag = 0;   //
	uint8_t play_serial_flag = 0;     //
	uint8_t enable_serial_flag = 0;   //
	uint8_t disable_serial_flag = 0;  //
	uint8_t setlimits_serial_flag = 0;
	uint8_t resetlimits_serial_flag = 0;
	uint8_t read_serial_flag = 0;

	// INPUT COMMANDS 2.2) before each state machine iteration,
	// serial port is checked for commands and safed into variables.
	// The reaction to the serial commands depends on the active state and is done there.
	chars_from_serial();  // reading chars from serial port

	// INPUT COMMANDS 2.3) parse new command string if new data is available
	if (newData) {
		// parse inputBuffer, get string until first delimiter
		ptr = strtok(inputBuffer, delimiter);

		// disable/enable command
		if (strcmp(ptr, "disable") == 0) {
			disable_serial_flag = 1;
		} else if (strcmp(ptr, "enable") == 0) {
			enable_serial_flag = 1;
		}
		// move command
		else if (strcmp(ptr, "move") == 0) {
			move_serial_flag = 1;
			move_start = current_ms;
			parse_serial_move();  // move_dura and move_delayed are set in this function
		}
		// teach command
		else if (strcmp(ptr, "teach") == 0) {
			record_serial_flag = 1;
		}

		// set limits command
		else if (strcmp(ptr, "setlimits") == 0) {
			setlimits_serial_flag = 1;
		}
		// reset limits command
		else if (strcmp(ptr, "resetlimits") == 0) {
			resetlimits_serial_flag = 1;
		}
		// play command
		else if (strcmp(ptr, "play") == 0) {
			play_serial_flag = 1;
		}
		// read stream command: does not lead to a state! just prints positions to serial port
		else if (strcmp(ptr, "read") == 0) {
			read_serial_flag = 1;
		}
		// read command: does not lead to a state! just prints positions to serial port
		else if (strcmp(ptr, "readstream") == 0) {
			readstream_flag = !readstream_flag;
			readstream_timer = millis();  // safe current timestamp
		}
		// info command: does not lead to a state! just prints info to serial port
		else if (strcmp(ptr, "info") == 0) {
			printInfo();
		}
		// reset all flags if enter is pressed
		else if (ptr == NULL)  // ----------------------------TODO: Enter Zeichen? alle Flags?
		{
			// reset read, other flags are resetted automatically in next iteration
			readstream_flag = 0;
			loadstream_flag = 0;
			// go to hold state
			lcl_state = GOTO_HOLD;
		}
		// read command: does not lead to a state! just prints loads to serial port
		else if (strcmp(ptr, "loadstream") == 0) {
			loadstream_flag = !loadstream_flag;
			loadstream_timer = millis();  // safe current timestamp
		} else {
			Serial.print(ptr);
			Serial.println(F(" is not a valid command!"));
		}
		newData = false;
	}

	// print servo positions to serial port
	if (read_serial_flag) {
		printPos();
	}
	// stream servo positions to serial port
	if (readstream_flag) {
		if ((current_ms - readstream_timer) >= PRINT_POS_INTERVAL_MS) {
			readstream_timer = current_ms;
			printPos();
		}
	}
	// stream servo loads to serial port
	if (loadstream_flag) {
		if ((current_ms - loadstream_timer) >= PRINT_POS_INTERVAL_MS)  // use same intervall as for pos read stram
		{
			loadstream_timer = current_ms;
			printLoad();
		}
	}

	// beeps beep_count times, one beep lasts t1 ms, with a period of t2 ms
	alarmsignal_run();

	// calls the function to check current turn of each servo
	checkTurns();

	// MAIN STATE MACHINE FOR ROBOT
	switch (lcl_state) {
		case INIT: {
			// STATE: scan for servos, if success, robot is ready. Else go to error state
#ifdef DEBUG_INFOS
			Serial.println("INIT");
#endif
			Serial.println(F("OK"));

			current_ms = millis();

			// servos enabled after power-on
			servos_torque_enable();

			// scan for connected servos
			int16_t servo_status;
			servo_status = scan_servos();

			if (servo_status < 0)  // error when scanning for servos
			{
				lcl_state = ERROR;
				Serial.println("Error scanning for servos!");
			} else  // scanning successful, check number of detected servos
			{
				num_servos = (uint8_t)servo_status;
				if (num_servos == MAX_SERVO_NUMBER)  // all servos detected
				{
					Serial.print(F("Success, "));
					Serial.print(num_servos);
					Serial.println(F(" detected! Going to state HOLD_POS"));

					servos_torque_enable();

					for (uint8_t i = 0; i < num_servos; i++) {
						last_servo_positions[i] = servo_readPos_w_dummy(servo_ids[i]);
					}
					lcl_state = GOTO_HOLD;
					// print the info about the available commands
					printInfo();
					// show user that setup is done: green LED and beep 2 times
					power_led(1);
					alarmsignal_set(2, 100, 200);
				} else  // not all servos detected
				{
					Serial.print(F("Not all servos detected, only "));
					Serial.print(num_servos);
					Serial.print(F(" instead of "));
					Serial.println(MAX_SERVO_NUMBER);

					lcl_state = ERROR;
				}
				// number of bytes per record
				eeprom_adress_length = 2 * num_servos;
			}
			delay(1000);
			// Init SD-Card-Reader
			Serial.print("Initializing SD card ...");
			if (!SD.begin(SD_SS_PIN)) {
				Serial.print("Initialization failed!");
				lcl_state = ERROR;
			} else {
				Serial.println("Initialization done.");
			}

			break;
		}
		case GOTO_HOLD: {
#ifdef DEBUG_INFOS
			Serial.println("GOTO_HOLD");
#endif
			// reset flags
			power_button_short_flag = 0;
			enable_serial_flag = 0;
			// turn off LEDs
			teach_led(1);
			play_led(1);
			// enable torque
			servos_torque_enable();

			// next state:
			lcl_state = HOLD_POS;
			break;
		}
		case HOLD_POS: {
			// STATE: hold servos in current position
#ifdef DEBUG_INFOS
			// Serial.println("HOLD_POS");
#endif
			// next state:
			if (power_button_short_flag || disable_serial_flag) {
				lcl_state = GOTO_IDLE;
			} else if (enable_serial_flag) {
			} else if (play_button_flag || play_serial_flag) {
				lcl_state = START_PLAY;
			} else if (record_button_flag || record_serial_flag) {
				lcl_state = START_RECORD;
			} else if (move_serial_flag) {
				lcl_state = MOVE;
			} else if (setlimits_serial_flag) {
				lcl_state = SET_LIMITS;
			} else if (resetlimits_serial_flag) {
				lcl_state = RESET_LIMITS;
			} else {
			}
			break;
		}
		case GOTO_IDLE: {
#ifdef DEBUG_INFOS
			Serial.println("GOTO_IDLE");
#endif
			// reset flags
			power_button_short_flag = 0;
			disable_serial_flag = 0;
			// setting torque to zero
			// servos_reboot();
			servos_torque_disable();
			// next state:
			lcl_state = IDLE;
			break;
		}
		case IDLE: {
#ifdef DEBUG_INFOS
			// Serial.println("IDLE");
#endif
			// STATE: servos at zero torque
			// next state:
			if (disable_serial_flag) {
			} else if (power_button_short_flag || enable_serial_flag) {
				lcl_state = GOTO_HOLD;
			} else if (play_button_flag || play_serial_flag) {
				lcl_state = START_PLAY;
			} else if (record_button_flag || record_serial_flag) {
				lcl_state = START_RECORD;
			} else if (move_serial_flag) {
				lcl_state = MOVE;
			} else if (setlimits_serial_flag) {
				lcl_state = SET_LIMITS;
			} else if (resetlimits_serial_flag) {
				lcl_state = RESET_LIMITS;
			} else {
			}
			break;
		}
		case START_RECORD: {
#ifdef DEBUG_INFOS
			Serial.println("START_RECORD");
#endif
			// reset flags
			record_button_flag = 0;
			record_serial_flag = 0;
			// set record timer to current time before entering RECORD state
			last_record_time_ms = current_ms;
			// reset record count index
			record_index = 0;
			// turn on LED
			teach_led(0);
			// set toque to zero
			// servos_reboot();
			servos_torque_disable();
			// next state
			lcl_state = RECORD;
			Serial.println(F("Recording started!"));

			// open the TextFile to save the positions
			SD.remove("temp.txt");  // Remove the old temp file
			TextFile = SD.open("temp.txt", FILE_WRITE);
			break;
		}
		case RECORD: {
			// STATE: record positions of all servos with the specified frequency
			if (record_button_flag || record_serial_flag) {
				// finish recording (and check recording if valid) and check recorded values
				// record_button_flag = 0;
				// record_serial_flag = 0;
				// power_button_short_flag = 0;
				eeprom_index = 0;
				eeprom_save_flag = 0;
				record_length = record_index;
				sd_save_flag = 0;
				TextFile.close();
				lcl_state = CHECK_RECORD;
				Serial.println(F("Recording stopped!"));
			} else if (power_button_short_flag) {
				lcl_state = GOTO_HOLD;
				Serial.println(F("Recording aborted!"));
			} else {
				if ((current_ms - last_record_time_ms) >= REC_INTERVAL)  // time to record the next position?
				{
					last_record_time_ms = current_ms;  // save timer value for next interval
					                                   // record current position:
//        if (record_index < REC_SIZE) // avoid buffer overflow
//        {
#ifdef DEBUG_INFOS
					Serial.print(F("REC"));
					Serial.print(record_index);
					Serial.print(F(" "));
#endif
					for (uint8_t k = 0; k < num_servos; k++)  // read all servos
					{
						int16_t pos_k = servo_readPos_w_dummy(servo_ids[k]);
						if (pos_k < 0)  // reading invalid? then go to ERROR state
						{
							lcl_state = ERROR;
							Serial.println(F("Error when reading servo positions! Going to error state!"));
						} else  // reading valid? then save it to array
						{
							// record_data[record_index][k] = pos_k + (servo_turns[k] * 4096);
							// write positions in file
							TextFile.print(pos_k + (servo_turns[k] * 4096));
							TextFile.print("\t");
							if (record_index == 0)  // save first Position
							{
								first_record_data[k] = pos_k + (servo_turns[k] * 4096);
							}
						}
#ifdef DEBUG_INFOS
						Serial.print(pos_k);
						Serial.print(F(" "));
#endif
					}
					if (record_index % 50 == 0)  // Die Textdatei muss in kleiner Textblöcke unterteilt werden, um diese Später einzeln in die neue Datei schreiben zu können
					{
						TextFile.print(";");  // start new line
					}
					TextFile.println("");  // start new line
					record_index++;
#ifdef DEBUG_INFOS
					Serial.println();
#endif
					//        }
					// else
					// {
					//   // buffer full --> stop recording. Since not the complete movement could be recorded, maybe go to error and discard recording? Or keep the incomplete recording?
					//   alarmsignal_set(3, 100, 200);
					//   eeprom_index = 0;
					//   eeprom_save_flag = 0;
					//   record_length = record_index;
					//   lcl_state = CHECK_RECORD;
					//   Serial.println(F("Buffer full, recording stopped!"));
					// }
				}
			}
			break;
		}
		case CHECK_RECORD: {
			// STATE: check if recording is valid and if so, set a flag.
			// after that, go to idle or hold_pos etc.

			// set timer for
			last_checkrecord_time_ms = millis();  // safe current timestamp

			// read servo limits
			// Serial.print(F("Servo limits: "));
			// Serial.println();
			/*     for (uint8_t j = 0; j < num_servos; j++)
			{
			  limit_min[j] = servo.readminAngle(servo_ids[j]);
			  limit_max[j] = servo.readmaxAngle(servo_ids[j]);
			         Serial.print(j);
			      Serial.print(F("  "));
			      Serial.print(limit_min[j]);
			      Serial.print(F("  "));
			      Serial.print(limit_max[j]);
			      Serial.println();
			} */

			// check for invalid teaching values
			record_valid_flag = 1;

			// pointer to start position
			// int16_t *rd_ptr = record_data[0];
			int16_t *rd_ptr = first_record_data;
			// set servo values
			uint8_t pos_reached_flag = servo_setConfig(rd_ptr, 100, 15);

			//     // write record data to EEPROM
			//     uint32_t checkrecord_time_ms = last_checkrecord_time_ms;
			//     while ((!eeprom_save_flag) && ((checkrecord_time_ms - last_checkrecord_time_ms) < (REC_INTERVAL - 2 - eeprom_adress_length * 4)))
			//     {
			//       // update timer
			//       checkrecord_time_ms = millis();
			//       // calc adress
			//       eeprom_adress = eeprom_start_index + eeprom_index * eeprom_adress_length;
			//       if ((eeprom_adress + 2 * (eeprom_adress_length + 4)) >= EEPROM.length())
			//       {
			//         // memory full
			//         eeprom_save_flag = 1;
			//         // set flag at first EEPROM entry to 1 -> valid record
			//         EEPROM.update(0, 1);
			//         // save record length
			//         EEPROM.put(1, 4096 - 3 - 4); // 3 bytes data information, 4 bytes crc
			//         // CRC
			//         EEPROM.put(4095 - 4, eeprom_crc());
			//         Serial.println(F("EEPROM full, could not store whole movement in non-volatile memory!"));
			//       }
			//       else if (eeprom_index < record_length)
			//       {
			//         if (eeprom_is_ready())
			//         {
			//           // write
			//           EEPROM.put(eeprom_adress, record_data[eeprom_index]);
			// #ifdef DEBUG_INFOS
			//           Serial.print(F("EEPROM index: "));
			//           Serial.print(eeprom_index);
			//           Serial.print(F(" EEPROM adress: "));
			//           Serial.println(eeprom_adress);
			// #endif
			//           // update index
			//           eeprom_index++;
			//         }
			//       }
			//       else
			//       {
			//         // finished writing
			//         eeprom_save_flag = 1;
			//         // set flag at first EEPROM entry to 1 -> valid record
			//         EEPROM.update(0, 1);
			//         // save record length
			//         EEPROM.put(1, record_length);
			//         // save CRC
			//         EEPROM.put(eeprom_adress, eeprom_crc());
			//         Serial.println(F("Movement saved in EEPROM"));
			//       }
			//     }

			// write data to file
			// TODO mehrere Speicherpunkte
			uint8_t save_finished_flag = 0;
			unsigned long TextFilePosition = 0;
			String selectedFile = Fileselect();
			Serial.println(selectedFile + " selected");
			SD.remove(selectedFile);  // delete the old file
			while (save_finished_flag == 0) {
				TextFile = SD.open("temp.txt");
				TextFile.seek(TextFilePosition);
				String TextData = TextFile.readStringUntil(';');
				TextFilePosition = TextFile.position();
				if (!TextFile.available()) {
					save_finished_flag = 1;
					sd_save_flag = 1;
				}
				TextFile.close();

				TextFile = SD.open(selectedFile, FILE_WRITE);
				TextFile.print(TextData);
				TextFile.close();
			}
			Serial.println("file saved on sd-card");

			pos_reached_flag = servo_setConfig(rd_ptr, 100, 15);

			// check if start position is reached
			// if (pos_reached_flag && eeprom_save_flag)
			if (pos_reached_flag && sd_save_flag) {
				// reset flag
				pos_reached_flag = 0;

				// next state
				lcl_state = GOTO_HOLD;

				// Serial.print("pos_reached_flag: ");
				// Serial.println(pos_reached_flag);
			} else if (power_button_short_flag || enable_serial_flag) {
				// stop at current position
				int16_t pos = 0;
				for (uint8_t i = 0; i < num_servos; i++) {
					pos = servo_readPos_w_dummy(i);
					servo_writePos_multiturn(i, pos, 100);
				}
				lcl_state = GOTO_HOLD;
			}
			break;
		}
		case START_PLAY: {
#ifdef DEBUG_INFOS
			Serial.println("START_PLAY");
#endif

			//     // get record data from EEPROM, if no valid data on RAM and valid data on EEPROM is available
			//     if (EEPROM.read(0) && !record_valid_flag)
			//     {
			//       //record_length = (EEPROM.read(1) << 8 | EEPROM.read(2));
			//       EEPROM.get(1, record_length);
			// #ifdef DEBUG_INFOS
			//       Serial.print("Read from EEPROM, record length: ");
			//       Serial.print(record_length);
			// #endif
			//       // read data: start from 3, byte 0 is valid flag, byte 1-2 is length
			//       for (uint16_t i = 0; i < record_length; i++)
			//       {
			// #ifdef DEBUG_INFOS
			//         Serial.println();
			//         Serial.print(eeprom_start_index + i * eeprom_adress_length);
			// #endif
			//         for (uint8_t k = 0; k < MAX_SERVO_NUMBER; k++)
			//         {
			//           //record_data[i][k] = (EEPROM.read(eeprom_start_index + i * eeprom_adress_length + k * 2) << 8 | EEPROM.read(eeprom_start_index + i * eeprom_adress_length + 1 + k * 2));
			//           EEPROM.get(eeprom_start_index + i * eeprom_adress_length + k * 2, record_data[i][k]);
			// #ifdef DEBUG_INFOS
			//           Serial.print(" ");
			//           Serial.print(record_data[i][k]);
			// #endif
			//         }
			//       }
			//       // EEPROM.get() //----------------------------------------------------------TODO: CRC, check if stored data is valid
			//       // set valid flag
			//       record_valid_flag = 1;
			//     }

			// check which file must be opened
			String selectedFile = Fileselect();
			Serial.println(selectedFile + " selected");
			if (SD.exists(selectedFile)) {
				TextFile = SD.open(selectedFile);
				for (uint8_t k = 0; k < num_servos; k++) {
					first_record_data[k] = TextFile.parseInt();
					if (first_record_data[k] == 0)  // 0-Werte verbieten um mögliche Fehlfunktionen zu vermeiden
					{
						record_valid_flag = 0;
					} else {
						record_valid_flag = 1;
					}
				}
				TextFile.close();
			} else {
				record_valid_flag = 0;
			}

			// check if valid recording is available
			if (record_valid_flag) {
				// turn LED on
				play_led(0);
				// power_led(0); //Parhofer

				// go to start position of teached movement
				// pointer to start position
				int16_t *rd_ptr = first_record_data;
				pos_reached_flag = servo_setConfig(rd_ptr, 100, 15);
				Serial.print("Pos reached?: ");
				Serial.println(pos_reached_flag);
				if (pos_reached_flag) {
					// reset flags
					pos_reached_flag = 0;
					play_button_flag = 0;
					play_serial_flag = 0;
					// init time variables
					last_play_time_ms = current_ms;  // set play timer to current time before entering PLAY state
					play_index = 0;                  // reset index
					Play_finished_flag = 0;

					TextFile = SD.open(selectedFile);  // open the selected File
					lcl_state = PLAY;
					Serial.println(F("Play recorded movement!"));
				}
			} else {
				// no valid recording
				lcl_state = GOTO_HOLD;
			}
			if (power_button_short_flag || enable_serial_flag) {
				// stop at current position
				int16_t pos = 0;
				for (uint8_t i = 0; i < num_servos; i++) {
					pos = servo_readPos_w_dummy(i);
					servo_writePos_multiturn(i, pos, 100);
				}
				lcl_state = GOTO_HOLD;
			}

			break;
		}
		case PLAY: {
			// STATE: plays recorded movement
			// time to play the next position?
			if ((current_ms - last_play_time_ms) >= REC_INTERVAL) {
				// save timer value for next interval
				last_play_time_ms = current_ms;
				int16_t d_pos;
				int16_t v = 500;  // default für ersten Punkt
				static int16_t correction[MAX_SERVO_NUMBER] = {0, 0, 0, 0, 0};
				int16_t newPos;
				static int16_t oldPos[MAX_SERVO_NUMBER] = {first_record_data[0], first_record_data[1], first_record_data[2], first_record_data[3], first_record_data[4]};  // save old Position with correction
				int16_t v_max_loadcontrol[MAX_SERVO_NUMBER] = {0, 500, 600, 0, 0};
				static int8_t count = 15;
				Serial.print(play_index);
				Serial.print(" ");
				if (count == 15)  // abfragen der nötigen Korrekturwerte (alle 15 Koordinaten, um "wackeln" zu vermeiden)
				{
					for (uint8_t j = 0; j < num_servos; j++) {
						correction[j] = getCorrectoin(j);
					}
					count = 0;
				}
				count++;

				// read all servos
				for (uint8_t k = 0; k < num_servos; k++) {
					{
						newPos = TextFile.parseInt();
						if (newPos == 0)  // bei Einlesefehler soll der letzte Wert angefahren werden und nicht 0
						{
							newPos = oldPos[k];
						}
						// newPos = record_data[play_index][k];
						// Serial.print(record_data[play_index][k]);
						// Serial.print(" ");
						if (play_index > 0) {  // nur erlaubt ab dem zweiten Durchgang, um Zugriff auf recording[0-1] zu vermeiden
							// d_pos = abs(record_data[play_index][k] - record_data[play_index - 1][k]); // zurückzulegende Strecke bis zur nächsten Position (ohne Korrektur)
							d_pos = abs(newPos - oldPos[k]);  // zurückzulegende Strecke bis zur nächsten Position (ohne Korrektur)
							// d_pos = abs(record_data[play_index][k] - oldPos[k]);                        // zurückzulegende Strecke bis zur nächsten Position (ohne Korrektur)
							v = 1000 * d_pos / (REC_INTERVAL);           // Skalierung der Geschwindigkeit, so dass exakt nach REC_INTERVAL die Zielposition erreicht wird. Die 1000 kommen durch Die Rechnung in Millisekunden.
							if (servo_ids[k] == 4 || servo_ids[k] == 5)  // ----- todo: Fallunterscheidung für 4. Servo, das andere Geschwindigkeitsskalierung hat.
							{
								v = v / 50;
							}
							Serial.print(newPos);
							Serial.print(" \t");
							// Korrekturwert soll nur bei kleinen Geschwindigkeiten miteinbezogen werden
							if (v < v_max_loadcontrol[k]) {
								// newPos = record_data[play_index][k] + correction[k];
								newPos = newPos + correction[k];
								d_pos = abs(newPos - oldPos[k]);  // zurückzulegende Strecke bis zur nächsten Position (mit Korrektur)
								v = 1000 * d_pos / (REC_INTERVAL);
								if (servo_ids[k] == 4 || servo_ids[k] == 5)  // ----- todo: Fallunterscheidung für 4. Servo, das andere Geschwindigkeitsskalierung hat.
								{
									v = v / 50;
								}
							} else if (v < 2 * v_max_loadcontrol[k]) {
								// newPos = record_data[play_index][k] + (1-((v/v_max_loadcontrol[k])-1))*correction[k];
								newPos = newPos + (1 - ((v / v_max_loadcontrol[k]) - 1)) * correction[k];
								d_pos = abs(newPos - oldPos[k]);  // zurückzulegende Strecke bis zur nächsten Position (mit Korrektur)
								v = 1000 * d_pos / (REC_INTERVAL);
								if (servo_ids[k] == 4 || servo_ids[k] == 5)  // ----- todo: Fallunterscheidung für 4. Servo, das andere Geschwindigkeitsskalierung hat.
								{
									v = v / 50;
								}
							} else {
								// newPos = record_data[play_index][k];
							}
						}
						// servo_writePos_multiturn(servo_ids[k], correction[k], v); // letzter Wert ist Geschwindigkeit
						// servo_writePos_multiturn(servo_ids[k], record_data[play_index][k], v); // letzter Wert ist Geschwindigkeit
						servo_writePos_multiturn(servo_ids[k], newPos, v);  // letzter Wert ist Geschwindigkeit
						// hold_flag = 1; // muss auf 1 gesetzt werden, sobald Servos einen Fahrbefehl bekommen haben.
						oldPos[k] = newPos;
					}
				}
				play_index++;
				Serial.println();
				if (TextFile.available() <= 3) {
					Play_finished_flag = 1;
					Serial.println("finish");
				}
			}

			// next state:
			// if (play_index >= record_length)
			if (Play_finished_flag == 1) {  // Parhofer
				if (play_button_count > 0)  // avoid overflow
				{
					play_button_count--;  // Parhofer
					if (play_button_count == 0) {
						lcl_state = GOTO_HOLD;
					} else {
						Serial.print(play_button_count);          // Parhofer
						Serial.println(F(" Repeats remaining"));  // Parhofer*/
						lcl_state = START_PLAY;                   // Parhofer
					}
				} else if (play_button_count == -1)  // -1 means infinite repeat
				{
					Serial.println(F("INFINITE Repeats remaining"));  // Parhofer*/
					lcl_state = START_PLAY;                           // Parhofer
				}

			} else if (disable_serial_flag) {
				lcl_state = GOTO_IDLE;
			} else if (power_button_short_flag || enable_serial_flag) {
				lcl_state = GOTO_HOLD;
			}
			/*else if (play_button_flag || play_serial_flag) // uncomented by Parhofer
			{
			  lcl_state = GOTO_HOLD;
			}*/
			else if (record_button_flag || record_serial_flag) {
				lcl_state = START_RECORD;
			} else if (move_serial_flag) {
				lcl_state = MOVE;
			} else if (setlimits_serial_flag) {
				lcl_state = SET_LIMITS;
			} else if (resetlimits_serial_flag) {
				lcl_state = RESET_LIMITS;
			} else {
			}
			break;
		}
		case MOVE: {
			// STATE: if move command from serial port is received
#ifdef DEBUG_INFOS
			// Serial.println("MOVE");
#endif
			pos_reached_count = 0;
			for (int i = 0; i < num_servos; i++) {
				move_currpos[i] = servo_readPos_w_dummy(servo_ids[i]);
				int16_t distance = abs((move_currpos[i] + (servo_turns[i] * 4096)) - move_setpos[i]);  // calc distance from current position and position from command
				float distanceRad = distance * (6.28 / 4096);                                          // convert to radians
				uint16_t m_nr = servo.readModelNumber(servo_ids[i]);                                   // read servo model number for calculating speed values
				// int temp = servo.readVersion(servo_ids[i]);
				float spe = distanceRad * 1000 / move_dura;  // calc speed value with duration T
				switch (m_nr)                                // switch case for different servo models
				{
					case 10248:  // SM...
						move_speeds[i] = mapfloat(spe, 0, 6.81 * 2.5, 0, 255);
						break;
					case 6150:                                             // || 11272                              // SM85CL
						move_speeds[i] = mapfloat(spe, 0, 3.87, 0, 4096);  // NEEDS TO BE ADJUSTED
						break;
					case 30728:                                            // SM...
						move_speeds[i] = mapfloat(spe, 0, 5.24, 0, 4096);  // NEEDS TO BE ADJUSTED
						break;
					default:
						Serial.print(F("Couldnt find modelnumber. Speed set to 5"));
						move_speeds[i] = 10;
						break;
				}
				// check if servo has reached goal
				if (abs(move_setpos[i] - (move_currpos[i] + (servo_turns[i] * 4096))) < 5) {
					pos_reached_count++;
				}
			}
			if ((current_ms - move_delayed) >= move_start)  // time to start moving?
			{
				for (int i = 0; i < num_servos; i++) {
					servo_writePos_multiturn(servo_ids[i], move_setpos[i], move_speeds[i]);
				}
#ifdef DEBUG_INFOS
				Serial.print("Move to: ");
				for (int i = 0; i < num_servos; i++) {
					Serial.print(move_setpos[i]);
					Serial.print(" ");
				}
				Serial.println();
				Serial.print("Current: ");
				for (int i = 0; i < num_servos; i++) {
					Serial.print(move_currpos[i]);
					Serial.print(" ");
				}
				Serial.println();
#endif
			}

			// next state:
			if ((pos_reached_count == num_servos) || power_button_short_flag || enable_serial_flag) {
				lcl_state = GOTO_HOLD;
			} else if (disable_serial_flag) {
				lcl_state = GOTO_IDLE;
			} else if (play_button_flag || play_serial_flag) {
				lcl_state = START_PLAY;
			} else if (record_button_flag || record_serial_flag) {
				lcl_state = START_RECORD;
			} else if (move_serial_flag) {
				lcl_state = MOVE;
			} else if (setlimits_serial_flag) {
				lcl_state = SET_LIMITS;
			} else if (resetlimits_serial_flag) {
				lcl_state = RESET_LIMITS;
			} else {
			}
			break;
		}
		case SET_LIMITS: {  // STATE: move servos to define limits
			if (!setlimit_index) {
				// new SET_LIMITS state: limits are set to current position
				for (int i = 0; i < num_servos; i++) {
					int16_t pos_k = servo_readPos_w_dummy(servo_ids[i]);
					servo.writeMinAngle(servo_ids[i], pos_k);
					servo.writeMaxAngle(servo_ids[i], pos_k);
					servo.ResetAngle(servo_ids[i]);  //----------------------------------TODO: was bedeutet das??
				}
				setlimit_index++;
				// disable servos
				servos_torque_disable();
			}
			// limits are expanded to current positions
			else {
				if ((current_ms - last_setlimit_time_ms) > REC_INTERVAL) {
					for (int k = 0; k < num_servos; k++) {
						int16_t pos_k = servo_readPos_w_dummy(servo_ids[k]);
						if (pos_k < servo.readminAngle(servo_ids[k])) {
							servo.writeMinAngle(servo_ids[k], pos_k);
						}
						if (pos_k > servo.readmaxAngle(servo_ids[k])) {
							servo.writeMaxAngle(servo_ids[k], pos_k);
						}
					}
					setlimit_index++;
				}
			}
			// next state:
			if (power_button_short_flag || enable_serial_flag) {
				lcl_state = GOTO_HOLD;
			} else if (play_button_flag || play_serial_flag) {
				lcl_state = START_PLAY;
			} else if (record_button_flag || record_serial_flag) {
				lcl_state = START_RECORD;
			} else if (move_serial_flag) {
				lcl_state = MOVE;
			} else if (resetlimits_serial_flag) {
				lcl_state = RESET_LIMITS;
			} else {
			}
			break;
		}
		case RESET_LIMITS: {
			// reset limits to 0
			for (int i = 0; i < num_servos; i++) {
				servo.writeMinAngle(servo_ids[i], 0);
				servo.writeMaxAngle(servo_ids[i], 0);
				// servo.ResetAngle(servo_ids[i]);
			}
			// next state:
			lcl_state = GOTO_HOLD;
			break;
		}
		case BATT_LOW: {
			break;
		}
		case BATT_EMPTY: {
			break;
		}
		case SHUTDOWN: {
			break;
		}
		case WAIT: {
			if ((current_ms - statemachine_waiting_start_ms) >= statemachine_waitingtime_ms)  // time is over
			{
				lcl_state = lcl_next_state_after_wait;  // go to next state
			}
			Serial.print(current_ms);
			Serial.print(" ");
			Serial.print(current_ms - statemachine_waiting_start_ms);
			Serial.print(" ");
			Serial.print(lcl_next_state_after_wait);
			Serial.println();
			break;
		}
		case ERROR: {
#ifdef DEBUG_INFOS
			// Serial.println("ERROR");
#endif
			teach_led(1);
			play_led(1);
			power_led(1);
			delay(100);
			teach_led(0);
			play_led(0);
			power_led(0);
			delay(100);
			break;
		}
		default: {
			Serial.println("unknown Error!");
			lcl_state = ERROR;
			break;
		}
	}
	// Serial.println(lcl_state);
}

// reading chars from serial port
static void chars_from_serial() {
	boolean overFlown = false;                            // true when the input string received was too long.
	while (Serial.available() > 0 && (newData == false))  // check incoming serial data from PC as long as there are new characters available in the buffer or until command is complete
	{
		// index for buffer
		static uint8_t ndx = 0;
		// define end markers
		static const char endMarker = '\r';
		static const char endMarker2 = '\n';
		// received character
		char rc;
		// get one character from the buffer
		rc = Serial.read();

		// put chars into buffer until end of line is reached
		if (rc != endMarker && rc != endMarker2) {
			inputBuffer[ndx] = rc;
			ndx++;
			if (ndx >= BUFFER_SIZE)  // avoid buffer overflow
			{
				overFlown = true;
				ndx = BUFFER_SIZE - 1;
			}
		} else  // end of command string
		{
			inputBuffer[ndx] = '\0';  // terminate string
			if (Serial.available()) {
				char rc_temp = Serial.peek();
				// remove '\r\n' or '\n\r' but keep '\r\r' or '\n\n'
				if ((rc == '\r' && rc_temp == '\n') || (rc == '\n' && rc_temp == '\r')) {
					// dummy read to remove next character from buffer
					Serial.read();
				}
			}
			ndx = 0;         // reset index
			newData = true;  // set flag for string parsing
		}
	}
	if (overFlown) {
		newData = false;
		overFlown = false;
		Serial.println("Input String too long");
	}
}

// parse serial move command: returns 1 if command fits number of servos, otherwise 0
static uint8_t parse_serial_move() {
	uint8_t posSet = 0;
	while (ptr != NULL)  // ptr!=(char)0x0D &&
	{
		ptr = strtok(NULL, delimiter);
		if (!posSet) {
			uint8_t i = 0;
			while (i < num_servos && ptr != NULL) {
				if (i > 0) {
					ptr = strtok(NULL, delimiter);
				}
				// move_setpos[i] = atof(ptr) * (4096 / 6.28);
				move_setpos[i] = atof(ptr);
				// move_setpos[i] = constrain(move_setpos[i], servo.readminAngle(servo_ids[i]), servo.readmaxAngle(servo_ids[i]));
				posSet++;
				i++;
			}
		} else {
			if (strcmp(ptr, "T") == 0) {
				ptr = strtok(NULL, delimiter);
				move_dura = atoi(ptr);
			} else if (strcmp(ptr, "D") == 0) {
				ptr = strtok(NULL, delimiter);
				move_delayed = atoi(ptr);
			} else {
				// Serial.println(F("Input ignored.")); // auskommentiert, da trotz korrektem Kommandostring am Ende immer dieser Case zutrifft. Todo: klären warum
			}
		}
#ifdef DEBUG_INFOS
		Serial.println(posSet);
#endif
		if (posSet != num_servos) {
			Serial.println(F("Wrong move command: Number of Servos does not fit number of positions."));
			// move_serial_flag = 0;
			return 0;
		} else {
			return 1;
		}
	}
}

// init timer 5 for a 10 ms timer interrupt for button debouncing in background.
// make sure not to use functions that need timer 5 (e.g. servo libs, analogWrite on the pins of timer 5)
static void button_timer_init() {
	// set everything to default
	TCCR5A = 0;
	TCCR5B = 0;
	TCCR5C = 0;
	TCNT5 = 0;

	OCR5A = 625 - 1;          // desired interrupt frequency: 100 Hz. 62500 Hz timer clock / 625 = 100 Hz. --> timer range 0...624 = 625 steps
	TCCR5B |= (1 << WGM52);   // timer mode CTC, Top value = OCR5A
	TIMSK5 |= (1 << OCIE5A);  // enable timer compare match interrupt --> make sure the corresponding interrupt service routine is defined, otherwise a reset will be triggered

	TCCR5B |= (1 << CS52);  // start timer with prescaler 256 --> 62500 Hz timer clock
}

// timer interrupt routine that will be called at 100 Hz for button checking and debouncing
ISR(TIMER5_COMPA_vect) {
	debounce_isr();  // check buttons and debounce them; this function takes 8 microseconds
}

// scan for connected servos. Returns either -1 for error or the number of servos if successful
static int16_t scan_servos() {
	// scan for connected servos
	int16_t num = 0;
	// print infos
	Serial.println(F("Scanning for connected servos..."));
	for (uint8_t i = 0; i <= MAX_SERVO_SCAN_ID; i++) {
		int16_t temp = servo.Ping(i);
		if (temp != -1)  // servo responding?
		{
			if (num < MAX_SERVO_NUMBER)  // avoid array overflow
			{
				servo_ids[num] = temp;
				num++;
			} else {
				// Serial.println(F("Error, too many servos found!"));
				num = -1;  // error
				break;
			}
		}
	}
	/*   // print infos
	  if (num == MAX_SERVO_NUMBER)
	  {
	  Serial.println(F("Servos are ready."));
	  }
	  else
	  {
	  Serial.println(F("Error, number of detected servos is wrong!"));
	  } */
	// return number of found servos
	return num;
}

static void set_wait_timer(uint32_t wait_ms) {
	statemachine_waiting_start_ms = current_ms;  // save current timestamp as starting point for waiting
	statemachine_waitingtime_ms = wait_ms;       // save desired waiting time
}

// sets pins for LEDs
static void leds_init() {
#ifdef HARDWARE_V2
	pinMode(LED_GND_PIN, OUTPUT);
	digitalWrite(LED_GND_PIN, LOW);

	pinMode(BUTTON_GND_PIN, OUTPUT);
	digitalWrite(BUTTON_GND_PIN, LOW);
#endif

#ifdef HARDWARE_V2_MAKITA
	pinMode(LED_GND_PIN, OUTPUT);
	digitalWrite(LED_GND_PIN, LOW);

	pinMode(BUTTON_GND_PIN, OUTPUT);
	digitalWrite(BUTTON_GND_PIN, LOW);
#endif

#ifdef HARDWARE_V3_PCB_MIMED
	pinMode(LED_GND_PIN, OUTPUT);
	digitalWrite(LED_GND_PIN, LOW);

	pinMode(BUTTON_GND_PIN, OUTPUT);
	digitalWrite(BUTTON_GND_PIN, LOW);
#endif

	pinMode(LED_TEACH_PIN, OUTPUT);
	pinMode(LED_PLAY_PIN, OUTPUT);
	pinMode(LED_POWER_PIN, OUTPUT);
}

static void teach_led(uint8_t on_off) {
	if (on_off) {
		digitalWrite(LED_TEACH_PIN, HIGH);
		// digitalWrite(LED_POWER_PIN, LOW); //Parhofer
		// digitalWrite(LED_PLAY_PIN, LOW); //Parhofer
	} else {
		digitalWrite(LED_TEACH_PIN, LOW);
		// digitalWrite(LED_POWER_PIN, HIGH); //Parhofer
		// digitalWrite(LED_PLAY_PIN, LOW); //Parhofer
	}
}

static void play_led(uint8_t on_off) {
	if (on_off) {
		digitalWrite(LED_PLAY_PIN, HIGH);
		// digitalWrite(LED_POWER_PIN, LOW); //Parhofer
		// digitalWrite(LED_TEACH_PIN, LOW); //Parhofer
	} else {
		digitalWrite(LED_PLAY_PIN, LOW);
		// digitalWrite(LED_POWER_PIN, HIGH); //Parhofer
		// digitalWrite(LED_TEACH_PIN, LOW); //Parhofer
	}
}

static void power_led(uint8_t on_off) {
	if (on_off) {
		digitalWrite(LED_POWER_PIN, LOW);
	} else {
		digitalWrite(LED_POWER_PIN, HIGH);
	}
}

// For this function, only ONE SERVO is allowed to be connected to the bus!
// set a servo's ID to a new value. Also changes baudrate to 1 MBaud (works only for 115200 and 1 MBaud).
// finally moves the servo to its middle position.
/* static void setupServo(int servoNum)
  {
  Serial.println();
  Serial.print(F("Trying to set new ID to: "));
  Serial.println(servoNum);
  Serial.print(F("Current ID:\t"));
  Serial.print(servo.Ping(0xfe)); // 0xfe = Broadcast to all servos
  Serial.println();
  int tempID = -1;
  long baudRate = 115200;   // try servo communication first at 115200 Baud
  Serial1.begin(baudRate);  // init Serial 1
  servo.pSerial = &Serial1; // Serial 1 is the servo bus
  //servo3.pSerial = &Serial1;  // TODO: warum doppelt? gehört eigtl. weg
  tempID = servo.Ping(0xfe); // broadcast ping again
  if (tempID == -1)
  {                     // -1 = invalid = no response
    baudRate = 1000000; // then try at 1 MBaud
    Serial1.end();      // re-init Serial 1 at 1 MBaud
    Serial1.begin(baudRate);
    servo.pSerial = &Serial1;
    tempID = servo.Ping(0xfe); // ping again...
  }
  if (tempID != -1)
  { // received valid response?
           if (servoNum != 3)
      {
    // servo nr. 3 is a SMSCL type, all the others are SMSBL type
    servo.unLockEprom(tempID);
    servo.writeByte(tempID, SMSBL_ID, servoNum); // write new ID
    if (baudRate == 115200)
    {                                                // if old baudrate was 115200, set baudrate to 1 MBaud.
      servo.writeByte(servoNum, SMSBL_BAUD_RATE, 0); // after this, the servo has switched to 1 MBaud.
      Serial1.end();                                 // so a re-init of Serial 1 is needed
      Serial1.begin(1000000);                        // 1 MBaud
    }
    servo.LockEprom(servoNum);
           }
      else
      { // for servo nr. 3, register addresses are different, the rest is as above
        servo3.unLockEprom(tempID);
        servo3.writeByte(tempID, SMSCL_ID, servoNum);
        if (baudRate == 115200)
        {
          servo3.writeByte(servoNum, SMSCL_BAUD_RATE, 0);
          Serial1.end();
          Serial1.begin(1000000);
        }
        servo3.LockEprom(servoNum);
      }
    servo.WritePos(servoNum, 2048, 4000); // move servo to middle position
  }
  else
  {
    Serial.println(F("No servo connected"));
  }
  Serial.print(F("New ID:\t\t"));
  Serial.print(servo.Ping(0xfe)); // read ID back from the servo
  Serial.println();
} */

// servo read where position is read multiple times until valid position value is available
// (maximum 3 tries)
static int16_t servo_readPos_w_dummy(uint8_t k) {
	int16_t temp = -1;
	uint8_t retry = 0;
	while ((temp < 0) && (retry < 10))  // actPos < 0 means reading error. Avoid endless loop by limiting to 3 retrys in case of reading errors.
	{
		retry++;
		temp = servo.ReadPos(k, NULL);
	}
	return temp;
}

// EDIT 2021-02-16 Not needed, negative values possible
// write servo position in multiturn mode, convert to feetech value range:
static void servo_writePos_multiturn(uint8_t k, int16_t position, uint16_t speed) {
	/*if (position < 0)
	{
	  // positive values: 0 to  32766 ->     0 to 32766
	  // negative values: 0 to -32766 -> 32768 to 65534
	  position = abs(position) + 32768;
	}*/
	servo.WritePos(k, position, speed);

	// Serial.print(position);
	// Serial.print(" ");
}

// go to position p with speed s and valid error e
static uint8_t servo_setConfig(int16_t *p, uint16_t s, uint16_t e) {
	uint8_t p_reached_count = 0;
#ifdef DEBUG_INFOS
	// Serial.print("Go to Position: ");
#endif
	for (int k = 0; k < num_servos; k++) {
		if (k <= 2) {
			servo_writePos_multiturn(servo_ids[k], p[k], s);  // Geschwindigkeit Speed Velocity
		} else {
			int teilfaktor = 10;  // Verhältnis Endeffektor-Speed zu anderen Servos
			if (s < teilfaktor)   // abrunden auf 0 verhindern.
			{
				s = teilfaktor;
			}
			servo_writePos_multiturn(servo_ids[k], p[k], s / teilfaktor);  // Endeffektor-Servo langsamer machen. (Parhofer, 03.05.2023)
		}
#ifdef DEBUG_INFOS
		// Serial.print(p[k]);
		// Serial.print(" ");
#endif
		// check if servo has reached goal
		int16_t p_curr = servo_readPos_w_dummy(servo_ids[k]);
		if (abs(p[k] - p_curr + (servo_turns[k] * 4096)) < e) {
			p_reached_count++;
		}
	}
#ifdef DEBUG_INFOS
	// Serial.println();
#endif

	if (p_reached_count == num_servos) {
		return 1;
	} else {
		return 0;
	}
}

/* void validate_recording() {

  } */

/* void play()
  {

  play_led(0); // turn LED off
  // after play function is done, clear buffered button presses (= ignore button presses during playing):
  get_key_press(1 << KEY_START_REC);
  get_key_press(1 << KEY_PLAY);
  get_key_press(1 << KEY_POWER);
  } */

/* void setLimit(int servoID)
  {
  teach_led(1);
  short temp = -1;
  while (temp < 0)
  {
    temp = servo.ReadPos(servoID, NULL);
  }
  //     if (servoID != 3)
  //  {
  servo.unLockEprom(servoID);
  servo.writeWord(servoID, SMSBL_MAX_ANGLE_LIMIT_L, temp);
  servo.writeWord(servoID, SMSBL_MIN_ANGLE_LIMIT_L, temp);
  //     }
  //  else
  //  {
  //    servo3.unLockEprom(servoID);
  //    servo3.writeWord(servoID, SMSBL_MAX_ANGLE_LIMIT_L, temp);
  //    servo3.writeWord(servoID, SMSBL_MIN_ANGLE_LIMIT_L, temp);
  //  }
  servo.EnableTorque(servoID, 0);
  short minAngle = temp;
  short maxAngle = temp;
  while (get_key_press(1 << KEY_START_REC) == 0)
  {
    short temp = -1;
    while (temp < 0)
    {
      temp = servo.ReadPos(servoID, NULL);
    }
    Serial.print("Current Pos:\t");
    Serial.println(temp);
    if (temp < minAngle)
    {
      minAngle = temp;
    }
    if (temp > maxAngle)
    {
      maxAngle = temp;
    }
    delay(REC_INTERVAL);
  }

  teach_led(0);
  //    if (servoID != 3)
  //     {
  servo.writeWord(servoID, SMSBL_MAX_ANGLE_LIMIT_L, maxAngle);
  servo.writeWord(servoID, SMSBL_MIN_ANGLE_LIMIT_L, minAngle);
  servo.LockEprom(servoID);
  //     }
  //    else
  //    {
  //      servo3.writeWord(servoID, SMSBL_MAX_ANGLE_LIMIT_L, maxAngle);
  //      servo3.writeWord(servoID, SMSBL_MIN_ANGLE_LIMIT_L, minAngle);
  //      servo3.LockEprom(servoID);
  //    }
  servo.EnableTorque(servoID, 1);
  get_key_press(1 << KEY_START_REC);
  get_key_press(1 << KEY_PLAY);
  get_key_press(1 << KEY_POWER);
  } */

/* void setLimits()
  {
  for (int i = 0; i < num_servos; i++)
  {
    //for (int i = 0; i < 3; i++) {
    int16_t pos_k = servo_readPos_w_dummy(servo_ids[i]);
    servo.writeMinAngle(servo_ids[i], pos_k);
    servo.writeMaxAngle(servo_ids[i], pos_k);
    servo.ResetAngle(servo_ids[i]);
  }
  for (int k = 0; k < num_servos; k++)
  {
    int16_t pos_k = servo_readPos_w_dummy(servo_ids[k]);
    if (pos_k < servo.readminAngle(servo_ids[k]))
    {
      servo.writeMinAngle(servo_ids[k], pos_k);
    }
    if (pos_k > servo.readmaxAngle(servo_ids[k]))
    {
      servo.writeMaxAngle(servo_ids[k], pos_k);
    }
    delay(REC_INTERVAL);
  }
  } */

static void printInfo() {
	Serial.println();
	Serial.print(F("ServoCount:\t\t"));
	Serial.print(num_servos);
	Serial.println();
	Serial.print(F("ServoIDs:\t\t"));
	for (int i = 0; i < num_servos; i++) {
		Serial.print(servo_ids[i]);
		if (i != num_servos - 1) {
			Serial.print(",");
		}
	}
	Serial.println();
	Serial.print("ServoModels:\t\t");
	for (int i = 0; i < num_servos; i++) {
		Serial.print("{");
		Serial.print(servo_ids[i]);
		Serial.print("}");

		Serial.print("[");
		int temp = servo.readModelNumber(servo_ids[i]);
		switch (temp) {
			case 10248: Serial.print(F("SM40BL")); break;
			case 6150: Serial.print(F("SM85CL")); break;
			case 30728: Serial.print(F("SM120BL")); break;
			default:
				Serial.print(F("Invalid Modelnumber "));
				Serial.print(temp);
				break;
		}
		Serial.print("]");
		if (i != num_servos - 1) {
			Serial.print(",");
		}
	}
	Serial.println();
	Serial.print(F("ServoLimits:\t\t"));
	for (int i = 0; i < num_servos; i++) {
		Serial.print("{");
		Serial.print(servo_ids[i]);
		Serial.print("}");

		Serial.print("[");
		Serial.print(servo.readminAngle(servo_ids[i]));
		Serial.print("-");
		Serial.print(servo.readmaxAngle(servo_ids[i]));
		Serial.print("]");
		if (i != num_servos - 1) {
			Serial.print(",");
		}
	}
	printCommandInfos();
	Serial.println();
}

static void printCommandInfos() {
	Serial.println();
	Serial.println(F("Available Commands:"));
	Serial.println(F("\tinfo:       information about servo motors"));
	Serial.println(F("\tread:       reading servo positions by typing 'read'"));
	Serial.println(F("\treadstream: start or stop streaming servo positions by typing 'readstream'"));
	Serial.println(F("\tmove:       move to configuration by specifying servo positions, time T \n\t            to get there in ms and start time D of movement in ms"));
	Serial.println(F("\t            Mapping: [0,4095] maps to [0deg,360deg] or [0,2*pi]"));
	Serial.println(F("\t            Min/Max position values: -32766 (-8*2*pi) to 32766 (8*2*pi))"));
	Serial.println(F("\t            EXAMPLE: move 2000 2118 1816 500 T=1000 D=1000"));
	Serial.println(F("\tenable:     enable Motor torque. Takes 1 second to complete."));
	Serial.println(F("\tdisable:    disable Motor torque. Takes 1 second to complete."));
	Serial.println(F("\tteach:      move robot by hand and record movement. Push any button to stop."));
	Serial.println(F("\tplay:       robot does previously recorded movement"));
	Serial.println(F("\tsetlimits:  sets new position limits for servo motors. Move all motors to the \n\t            desired limits. Do not turn around more than 360 degrees!"));
	Serial.println(F("\tresetlimits:resets all position limits for servo motors (->no limits)"));
	Serial.println(F("\tservosetup: Works only with one single servo connected! Sets the ID of a connected \n\t            servo to the given value, sets the servo baudrate to 1 MBaud and drives the servo to its middle position."));
	Serial.println(F("\t            EXAMPLE: servosetup 1"));
}

// shutting servos off, then rebooting
static void servos_reboot() {
	Serial.print(F("rebooting servos..."));
	digitalWrite(SERVO_POWER_PIN, LOW);  // power off
	delay(100);
	digitalWrite(SERVO_POWER_PIN, HIGH);  // power on
	alarmsignal_set(3, 100, 200);
	Serial.println(F(" READY!"));
}

// disables torque: servos can be moved freely
static void servos_torque_disable() {
	for (uint8_t i = 0; i < num_servos; i++) {
		servo.EnableTorque(servo_ids[i], 0);
	}
	alarmsignal_set(3, 50, 100);
}

// disables torque: servos can be moved freely
static void servos_torque_enable() {
	int16_t pos = 0;
	for (uint8_t i = 0; i < num_servos; i++) {
		servo.EnableTorque(servo_ids[i], 1);
		// pos = servo_readPos_w_dummy(i);
		// servo_writePos_multiturn(i,pos,100);
	}
}

static void printPos() {
	Serial.print("Positions: ");
	for (int k = 0; k < num_servos; k++) {
		int16_t pos = servo_readPos_w_dummy(servo_ids[k]) + (servo_turns[k] * 4096);
		Serial.print(pos);
		Serial.print("  ");
		Serial.print(servo_turns[k]);
		Serial.print("  ");
	}
	Serial.print("\r\n");
}

/* static void servo_power_on() // turn on power supply of the servos
  {
  digitalWrite(SERVO_POWER_PIN, HIGH);
  } */

static float mapfloat(float x, float in_min, float in_max, float out_min, float out_max) { return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min; }

/* static void flushSerial()
  {
  while (Serial.available() > 9)
  {
    Serial.read(); // dummy read
  }
  } */

// init alarm signal pins
static void alarmsignal_init() {
	pinMode(ALARMSIGNAL_PIN, OUTPUT);  // alarmsignal pins init
	digitalWrite(ALARMSIGNAL_PIN, HIGH);
	pinMode(ALARMSIGNAL_GND_PIN, OUTPUT);
	digitalWrite(ALARMSIGNAL_GND_PIN, LOW);
}

// sets alarmsignal to beep k times, one beep lasts t1 ms, with a period of t2
static void alarmsignal_set(uint8_t k, uint32_t t1, uint32_t t2) {
	beep_start = 1;
	beep_count = k;
	beep_length_ms = t1;
	beep_period_ms = t2;
}

// beeps beep_count times, one beep lasts beep_length_ms ms, with a period of beep_period_ms
static void alarmsignal_run() {
#ifndef DEBUG_ALARMOFF
	if (beep_count > 0) {
		if (beep_start) {
			beep_timer_ms = current_ms;
			digitalWrite(ALARMSIGNAL_PIN, HIGH);
			beep_count--;
			beep_start = 0;
			// Serial.print("Beep starts ");
			// Serial.println(beep_count);
		}
		if (current_ms - beep_timer_ms > beep_length_ms) {
			digitalWrite(ALARMSIGNAL_PIN, LOW);
		}
		if (current_ms - beep_timer_ms > beep_period_ms) {
			beep_start = 1;
		}
	} else {
		digitalWrite(ALARMSIGNAL_PIN, LOW);
	}
#endif
}

// Function that keeps count on which turn each servo is since the last startup. This is needed to make use of Multiturnmode.
void checkTurns() {
	for (uint8_t i = 0; i < num_servos; i++) {
		uint16_t temp_pos = servo_readPos_w_dummy(servo_ids[i]);
		int16_t diff = last_servo_positions[i] - temp_pos;
		if (diff > 4000) {
			servo_turns[i]++;
		} else if (diff < -4000) {
			servo_turns[i]--;
		}
		last_servo_positions[i] = temp_pos;
	}
}

// cyclic redundancy check (EEPROM)
unsigned long eeprom_crc(void) {
	const unsigned long crc_table[16] = {0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac, 0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c, 0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c, 0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c};
	unsigned long crc = ~0L;
	for (int index = 0; index < EEPROM.length(); ++index) {
		crc = crc_table[(crc ^ EEPROM[index]) & 0x0f] ^ (crc >> 4);
		crc = crc_table[(crc ^ (EEPROM[index] >> 4)) & 0x0f] ^ (crc >> 4);
		crc = ~crc;
	}
	return crc;
}

// print load to serial function
static void printLoad() {
	Serial.print("Loads: ");
	for (int k = 0; k < num_servos; k++) {
		int16_t load = servo_readLoad_w_dummy(servo_ids[k]);
		Serial.print(load);
		Serial.print("\t\t");
	}
	Serial.print("\r\n");
}

// servo read load where load is read multiple times until valid load value is available
// (maximum 5 tries)
static int16_t servo_readLoad_w_dummy(uint8_t k) {
	int16_t temp = -1;
	uint8_t retry = 0;
	while ((temp < 0) && (retry < 5))  // actPos < 0 means reading error. Avoid endless loop by limiting to 5 retrys in case of reading errors.
	{
		retry++;
		temp = servo.ReadLoad(k);
	}
	// fix Load value to make it more readable
	if (temp == -1)  // set load to 0 if no valid value was reat
	{
		temp = 0;
	} else if (temp > 1000)  // transform negative loads to negative values
	{
		temp = -(temp - 1024);
	}
	return temp;
}

// calculate necessary correction from load
static int16_t getCorrectoin(uint8_t k) {
	int16_t LoadLimits[num_servos] = {30, 60, 25, 20, 20};
	int16_t CorLimits[num_servos] = {10, 13, 10, 5, 5};
	int16_t cor = 0;
	int16_t load = servo_readLoad_w_dummy(servo_ids[k]);

	if (load > 5) {
		if (load > LoadLimits[k]) {
			cor = -CorLimits[k];
		} else {
			cor = float(-CorLimits[k]) / float(LoadLimits[k]) * (load - 5);
		}
	} else if (load < -5) {
		if (load < -LoadLimits[k]) {
			cor = CorLimits[k];
		} else {
			cor = float(CorLimits[k]) / float(LoadLimits[k]) * (load + 5);
		}
	}
	// Serial.println(load);
	//  Serial.println(cor);
	return cor;
}

String Fileselect() {
	int valuePOTI = analogRead(POTI_PIN);
	if (valuePOTI <= 204) {
		return "File_4.txt";
	} else if (valuePOTI <= 409) {
		return "File_3.txt";
	} else if (valuePOTI <= 613) {
		return "File_2.txt";
	} else if (valuePOTI <= 818) {
		return "File_1.txt";
	} else if (valuePOTI <= 1023)  // RFID
	{
		String RFID_Nr = "";
		// Read RFID
		// Look for new cards and Verify if the NUID has been readed
		for (int i = 0; i < 5; i++) {
			if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
				for (byte i = 0; i < 2; i++) {
					RFID_Nr = RFID_Nr + (rfid.uid.uidByte[i]);
				}
				Serial.print("RFID-Card detected: ");
				Serial.println(RFID_Nr);
				// Stop encryption on PCD
				rfid.PCD_StopCrypto1();
				// return String
				return (RFID_Nr + ".txt");
			}
		}
	}
	return "";
}

#include <math.h>
// These values are constants that result from the construction of the robot and are needed in the code:
const double gamma = 1.0471975512;
const double b = 96.4;
const double l1 = 225;
const double l2 = 290;
const double l3 = 200;
const double l4 = 435;
const double r1 = 12.5;
const double r2 = 7.5;
const double r3 = 10.5;
const double s = 2;
const double o1 = 4254;
const double o2 = 1653;
const double o3 = 2341;
const double o4 = 2223;
const double o5 = 6211;
const double o6 = 5275;
const double o7 = 5000;
const double phi1min = 165;
const double phi2min = 1467;
const double phi3min = 474;
const double phi4min = 0;
const double phi5min = 0;
const double phi6min = 0;
const double phi7min = 0;
const double phi1max = 4178;
const double phi2max = 3931;
const double phi3max = 4113;
const double phi4max = 5122;
const double phi5max = 32766;
const double phi6max = 32766;
const double phi7max = 32767;
const double phi56diffmin = -468;
const double phi56diffmax = 970;
const double phi7diffmin = 0;
const double phi7diffmax = 200000;
const double alpha_min = 0.38;
const double rad_to_enc1 = -2048.0 / PI;
const double rad_to_enc2 = 2048.0 / PI;
const double rad_to_enc3 = 2048.0 / PI;
const double rad_to_enc4 = 2048.0 / PI;
const double rad_to_enc5 = 2048.0 / PI;
const double rad_to_enc6 = 2048.0 / PI;
const double rad_to_enc7 = 2048.0 / PI;

struct Angles {
	double phi1;
	double phi2;
	double phi3;
	double phi4;
	double phi5;
	double phi6;
	double phi7;
	double aoa;
};

struct AnglesRot {
	double phi5;
	double phi6;
};

struct Vector3 {
	double x;
	double y;
	double z;
};

double dot(Vector3 a, Vector3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

Vector3 cross(Vector3 a, Vector3 b) {
	Vector3 c;
	c.x = a.y * b.z - a.z * b.y;
	c.y = a.z * b.x - a.x * b.z;
	c.z = a.x * b.y - a.y * b.x;
	return c;
}

Angles getAnglesPos(double x, double y, double z, double eta, double theta, double odaoa) {
	Angles aPos;
	// calculates new coordinate d in the rotated coordinate system:
	double temp = sq(x) + sq(y) - sq(b);
	double d = NAN;
	if (temp > 0) {
		d = sqrt(temp);
	}
	// calculates the angle phi1, which is also the angle by which the co-sy is rotated:
	aPos.phi1 = atan2(y, x) + atan2(b, d);
	// tests if original desired angle of attack allows for desired rotation of the tool and adjust it if not:
	double daoa = rangeOfMotion(eta, theta, aPos.phi1, odaoa);
	double comp1 = sqrt(sq(d - l4 * sin(daoa)) + sq(z + l4 * cos(daoa) - l1));
	double comp2 = sqrt(sq(d) + sq(z - l1));
	// tests if position is in range of motion for desired angle of attack and if yes calculates phi2 to phi4:
	if (comp1 <= (l2 + l3)) {
		temp = (sq(d - l4 * sin(daoa)) + sq(z + l4 * cos(daoa) - l1) - sq(l2) - sq(l3)) / (2 * l2 * l3);
		temp = constrain(temp, -1.0, 1.0);
		aPos.phi3 = acos(temp);
		double ny = atan2((z + l4 * cos(daoa) - l1), (d - l4 * sin(daoa)));
		double delta = atan2((l3 * sin(aPos.phi3)), (l2 + l3 * cos(aPos.phi3)));
		aPos.phi2 = ny + delta;
		aPos.phi4 = aPos.phi2 - aPos.phi3 - daoa + PI / 2;
		aPos.aoa = daoa;
	}
	// tests if position is in range of motion and if yes calculates phi2 to phi4 and new angle of attack:
	else if (comp2 <= (l2 + l3 + l4)) {
		aPos.phi3 = 0;
		temp = (sq(d) + sq(z - l1) - sq(l2 + l3) - sq(l4)) / (2 * (l2 + l3) * l4);
		temp = constrain(temp, -1.0, 1.0);
		aPos.phi4 = acos(temp);
		double ny = atan2((z - l1), d);
		double delta = atan2(l4 * sin(aPos.phi4), (l2 + l3 + l4 * cos(aPos.phi4)));
		aPos.phi2 = ny + delta;
		aPos.aoa = aPos.phi2 - aPos.phi3 - aPos.phi4 + PI / 2;
	}
	// sets angles to unrealistic values to make clear, that the positon isn’t possible:
	else {
		aPos.phi1 = aPos.phi2 = aPos.phi3 = aPos.phi4 = aPos.aoa = NAN;
	}
	return aPos;
}

AnglesRot getAnglesRot(double alpha, double beta) {
	AnglesRot aRot;
	aRot.phi5 = beta - acos(tan(alpha / 2) / tan(gamma));
	aRot.phi6 = beta + acos(tan(alpha / 2) / tan(gamma));
	return aRot;
}

double getAnglesStab(double h, double alpha, double beta) {
	double phi7 = beta + acos(tan(alpha / 2) / tan(gamma)) - (r2 / r3) * acos(cos(alpha) / sin(2 * gamma) - 1 / tan(2 * gamma)) - 2 * PI * r1 * h / (r3 * s);
	return phi7;
}

double testAngles(double phi1, double phi2, double phi3, double phi4, double phi5, double phi6, double phi7) {
	// if one rotations of  motors isn’t possible or not a number false is returned
	if (phi1 < phi1min || phi1 > phi1max || isnan(phi1)) return NAN;
	if (phi2 < phi2min || phi2 > phi2max || isnan(phi2)) return NAN;
	if (phi3 < phi3min || phi3 > phi3max || isnan(phi3)) return NAN;
	if (phi4 < phi4min || phi4 > phi4max || isnan(phi4)) return NAN;
	if (phi5 < phi5min || phi5 > phi5max || isnan(phi5)) return NAN;
	if (phi6 < phi6min || phi6 > phi6max || isnan(phi6)) return NAN;
	if (phi7 < phi7min || phi7 > phi7max || isnan(phi7)) return NAN;
	if ((phi5 - phi6) < phi56diffmin || (phi5 - phi6) > phi56diffmax) return NAN;
	if ((phi7 - (0.5 * (phi5 + phi6))) < phi7diffmin || (phi7 - (0.5 * (phi5 + phi6))) > phi7diffmax) return NAN;
	if (isnan(phi5) || isnan(phi6) || isnan(phi7)) return NAN;
	return 1;
}

double rangeOfMotion(double eta, double theta, double phi1, double odaoa) {
	double daoa = odaoa;
	double alpha = getAlpha(eta, theta, phi1, daoa);
	if (alpha < alpha_min) {
		// calculates new daoa such that alpha is possible
		alpha = alpha_min;
		double daoa1 = sqrt(sq(cos(phi1 - eta) * sin(theta)) + sq(cos(theta)));
		double daoa2 = -cos(phi1 - eta) * sin(theta);
		daoa = acos(cos(alpha) / daoa1) + acos(cos(theta) / daoa2);
	}
	return daoa;
}

double getAlpha(double eta, double theta, double phi1, double aoa) {
	double alpha = acos(cos(aoa) * sin(theta) - sin(aoa) * sin(theta) * cos(phi1 - eta));
	return alpha;
}

double getBeta(double eta, double theta, double phi1, double aoa, double alpha) {
	Vector3 rA = {-cos(phi1) * sin(aoa), -sin(phi1) * sin(aoa), cos(aoa)};
	Vector3 u = {cos(phi1) * cos(aoa), sin(phi1) * cos(aoa), sin(aoa)};
	Vector3 v = {cos(eta) * sin(theta) - cos(phi1) * sin(aoa) * cos(alpha), sin(eta) * sin(theta) - sin(phi1) * sin(aoa) * cos(alpha), cos(theta) - cos(aoa) * cos(alpha)};
	double beta = atan2(dot(rA, cross(u, v)), dot(u, v));
	return beta;
}

// This function calculates the needed angles through the inverse kinematics in rad and takes the desired angle of attack and possibly changes it, if the desired one isn’t possible.
Angles getAnglesAll(double x, double y, double z, double eta, double theta, double h, double odaoa) {
	// calculate angles phi1 to phi4, which are needed to reach desired position, and angle of attack:
	Angles a = getAnglesPos(x, y, z, eta, theta, odaoa);
	// calculate angles alpha and beta, which are needed to calculate phi5 to phi7:
	double alpha = getAlpha(eta, theta, a.phi1, a.aoa);
	double beta = getBeta(eta, theta, a.phi1, a.aoa, alpha);
	// calculate angles phi5 and phi6, which are needed to reach desired rotation:
	AnglesRot aRot = getAnglesRot(alpha, beta);
	a.phi5 = aRot.phi5;
	a.phi6 = aRot.phi6;
	// calculate angle phi7, which is needed to reach desired stabbing position:
	a.phi7 = getAnglesStab(h, alpha, beta);
	return a;
}

// This function calculates the needed angles for the actuation of the motors in encoder-bits from the angles in rad and tests if the desired angles are inside the range of motion.
Angles transformAnglesAll(double phi1, double phi2, double phi3, double phi4, double phi5, double phi6, double phi7) {
	// angles in rad are transformed into encoder-bits and offset is added to account for construction
	Angles e;
	e.phi1 = phi1 * rad_to_enc1 + o1;
	e.phi2 = phi2 * rad_to_enc2 + o2;
	e.phi3 = phi3 * rad_to_enc3 + o3;
	e.phi4 = phi4 * rad_to_enc4 + o4;
	e.phi5 = phi5 * rad_to_enc5 + o5;
	e.phi6 = phi6 * rad_to_enc6 + o6;
	e.phi7 = phi7 * rad_to_enc7 + o7;
	// tests if angles are possible and in range of motion
	e.aoa = testAngles(e.phi1, e.phi2, e.phi3, e.phi4, e.phi5, e.phi6, e.phi7);
	return e;
}

boolean moveTo(double x, double y, double z, double eta, double theta, double h, double odaoa) {
	// -------------------------------------------------
	// 1. Inverse Kinematik berechnen
	// -------------------------------------------------
	Angles a = getAnglesAll(x, y, z, eta, theta, h, odaoa);

	// -------------------------------------------------
	// 2. Winkel in Encoderwerte transformieren
	// -------------------------------------------------
	Angles e = transformAnglesAll(a.phi1, a.phi2, a.phi3, a.phi4, a.phi5, a.phi6, a.phi7);

	// -------------------------------------------------
	// 3. Alle Rückgabewerte ausgeben
	// -------------------------------------------------
	Serial.println("----- transformAnglesAll -----");

	Serial.print("phi1: ");
	Serial.println(e.phi1);

	Serial.print("phi2: ");
	Serial.println(e.phi2);

	Serial.print("phi3: ");
	Serial.println(e.phi3);

	Serial.print("phi4: ");
	Serial.println(e.phi4);

	Serial.print("phi5: ");
	Serial.println(e.phi5);

	Serial.print("phi6: ");
	Serial.println(e.phi6);

	Serial.print("phi7: ");
	Serial.println(e.phi7);

	Serial.print("valid: ");
	Serial.println(e.aoa);

	Serial.println("------------------------------");

	// -------------------------------------------------
	// 4. Prüfen ob Bewegung gültig ist
	// -------------------------------------------------
	if (e.aoa != 1 || isnan(e.aoa)) {
		Serial.println("Move not possible!");
		return false;
	}

	// -------------------------------------------------
	// 5. Zielpositionen vorbereiten
	// -------------------------------------------------
	int16_t targetPos[MAX_SERVO_NUMBER];

	targetPos[0] = (int16_t)e.phi1;
	targetPos[1] = (int16_t)e.phi2;
	targetPos[2] = (int16_t)e.phi3;
	targetPos[3] = (int16_t)e.phi4;
	targetPos[4] = (int16_t)e.phi5;
	targetPos[5] = (int16_t)e.phi6;
	targetPos[6] = (int16_t)e.phi7;

	// -------------------------------------------------
	// 6. Servos anfahren
	// Nutzt exakt dieselbe Funktion wie dein restlicher Code
	// -------------------------------------------------

	uint16_t speed = 200;
	uint16_t allowedError = 20;

	servo_setConfig(targetPos, speed, allowedError);

	Serial.println("Move started.");

	return true;
}

void giveEncAngles(double x, double y, double z, double eta, double theta, double h, double odaoa) {
	// 1. Inverse Kinematik
	Angles a = getAnglesAll(x, y, z, eta, theta, h, odaoa);

	// 2. In Encoderwerte umrechnen
	Angles e = transformAnglesAll(a.phi1, a.phi2, a.phi3, a.phi4, a.phi5, a.phi6, a.phi7);

	Serial.println();
	Serial.println("===== Encoder Angles =====");

	Serial.print("phi1 = ");
	Serial.println(e.phi1);

	Serial.print("phi2 = ");
	Serial.println(e.phi2);

	Serial.print("phi3 = ");
	Serial.println(e.phi3);

	Serial.print("phi4 = ");
	Serial.println(e.phi4);

	Serial.print("phi5 = ");
	Serial.println(e.phi5);

	Serial.print("phi6 = ");
	Serial.println(e.phi6);

	Serial.print("phi7 = ");
	Serial.println(e.phi7);

	if (isnan(e.aoa)) {
		Serial.println("Result: INVALID");
	} else {
		Serial.println("Result: VALID");
	}

	Serial.println("==========================");
}