#include <stdio.h>
#include <stdbool.h> 
#include <NFC_THMS_to_Serial.h>
//#include <SoftwareReset.h>

// Version: V1.4

/*----------- ToDo -------------*/
//  - Handling serial conmmands to
//    - Instruction to reset and reboot.
//  - Check if measurement is complete in ndef-text (!= Do: 2)
//  - Check error messages from tag (== Do:FF ???)
//  - Reset of TAG (Power-cycle NFC-Field)
//  - Check if Do-Instruction is valid???


/*----------- CONFIGURATION -------------*/
#define FSM_SLOWDOWN                        500    // Slowdown of Finite-State-Machine in ms
#define DEFAULT_FOR_CONTINUOUS_MEASUREMENT  true   // Default setup to do continuous measurement.
#define DEFAULT_MEASUREMENT_INTERVAL_IN_S   120    // Default interval for continuouse measurementv
// DEBUG CONFIGURATION: To print debug infos beginning with ">>> "
#define PRINT_DEBUG_INFO_ERROR              true   // To print errors via uart.
#define PRINT_DEBUG_INFO_STANDAR            true   // To print standard info via uart.
#define PRINT_DEBUG_INFO_FSM                false   // To print FSM-State info via uart.
#define PRINT_NEXT_MEASUREMENT_INFO         false   // To print time to next measurement in seconds via uart.
#define PRINT_EXTENDED_INFO                 false   // To print some extended standard info via uart.

#define MAXIMAL_INFO_MESSAGE_LENGT    81
#define MAXIMAL_NDEF_MESSAGE_LENGT    81

/*---------------------------------------*/

/*---------- Typedefinitions ------------*/
typedef enum {
  FSM_IDLE                      = 0x00,
  FSM_SEARCH_SENSOR             = 0x01,
  //FSM_DO_MEASUREMENT            = 0x02,
  FSM_WRITE_INSTRUCTION         = 0x03,
  FSM_READ_TAG_DATA             = 0x04,
  FSM_WRITE_DATA                = 0x05,
  FSM_CHANGE_CONFIG             = 0x06,
  FSM_ERROR                     = 0xFF
}finite_state_machine_state_t;

//Debug infos via uart. Each info has own bit.
typedef enum {
  INFO_ERROR_INFO               = (0x1 << 0), // Info about errors
  INFO_STANDARD_INFO            = (0x1 << 1), // Standard infos
  INFO_FSM_STATE                = (0x1 << 2), // Info about actual FMS-State
  INFO_NEXT_MEASUREMENT_INFO    = (0x1 << 3), // Time to next Measurement in s
  INFO_EXTENDED_INFO            = (0x1 << 4), // Time to next Measurement in s
  INFO_ALWAYS                   = 0xFF
}uart_debug_info_t;

//Error types. Should be at least uint16_t. Each error has own bit. 
typedef enum {
  ERROR_NO_ERROR                = 0x0,
  ERROR_SET_INSTRUCTION         = (0x1 << 1), // = 0x0002
  ERROR_READ_NDEF_TEXT          = (0x1 << 2), // = 0x0004
  ERROR_UNKNOWN_FSM_STATE       = (0x1 << 3), // = 0x0008
  ERROR_NO_SENSOR_AVAILABLE     = (0x1 << 4), // = 0x0010
  ERROR_SENSOR_CONNECTION_LOST  = (0x1 << 5), // = 0x0020
  ERROR_GET_DATA                = (0x1 << 6), // = 0x0040
  ERROR_SERIAL_INPUT            = (0x1 << 7), // = 0x0080
  ERROR_UNKNOWN                 = (0x1 << 15) // = 0x8000
}error_indicator_t;

typedef enum {
  SI_SEARCH_SENSOR              = 'S', // Search for sensor.
  SI_DO_SINGLE_MEASUREMENT      = 'M', // Trigger single measurement and read it after 6 seconds.
  SI_SET_INSTRUCTION            = 'I', // Write Do-instruction to Tag (E.g. "I:0x04" for an reset or "I:0x06" to get config) //See also typedef "nt2s_do_instructions_t"
  SI_READ                       = 'R', // Read NFC-Tag data.
  SI_WRITE                      = 'W', // Write special data to NFC-Tag which is not handled via case 'I' (E.g. "W:Do:05;..." to set config.).
  SI_CONTINUOUS_MEASUREMENT     = 'C', // To enable or disable continuous measurement.
  SI_CHANGE_TIMING_4_CM         = 'T', // Change timing for continuous measurement in seconds (E.g. T:120).
  SI_RESET                      = 'X'  // Reset and reboot.
}serial_instruction_t;

/*------------ Global Variables ---------------*/
static finite_state_machine_state_t fsm_state;
static uint16_t error_no = ERROR_NO_ERROR;
static bool  continuous_measurement_m = DEFAULT_FOR_CONTINUOUS_MEASUREMENT;
static uint16_t cont_meas_interval_in_s_m = DEFAULT_MEASUREMENT_INTERVAL_IN_S;
static bool  sensor_available_m = false;
static unsigned long next_measurement_time_s_m = 0;
static uint8_t debug_level = (PRINT_DEBUG_INFO_ERROR*INFO_ERROR_INFO) 
                           | (PRINT_DEBUG_INFO_STANDAR*INFO_STANDARD_INFO) 
                           | (PRINT_DEBUG_INFO_FSM*INFO_FSM_STATE) 
                           | (PRINT_NEXT_MEASUREMENT_INFO*INFO_NEXT_MEASUREMENT_INFO)
                           | (PRINT_EXTENDED_INFO*INFO_EXTENDED_INFO);
static char info_array_m[MAXIMAL_INFO_MESSAGE_LENGT];
static uint8_t nfc_message_m[MAXIMAL_NDEF_MESSAGE_LENGT]; //Array for text message (NDEF)
static uint8_t do_insturction_to_set_m;
bool get_response_m; // To get response after do-instruction


/*------------ Function Declaration ---------------*/
void check_sensor_availability_5s(void); /* Search sensor for 5s (5 times) certain time*/
void check_for_serial_instructions(void);  // Maximal length for instruction is 50. Each instruction has to end with '\n'.
void parse_serial_4_instruction(char buf[], int rlen); 
bool set_instruction(uint8_t do_instruction);
bool set_instruction_to_do_measurement();
void print_debug_info(uart_debug_info_t info_level);  // To print infos via USB-UART (Serial)
void print_debug_info_f(const __FlashStringHelper * string_to_print, uart_debug_info_t info_level); //Print flash string (um RAM zu sparen)
bool get_tag_data(uint8_t text_data_array[], uint8_t max_length);


void setup() {
  /* Initialisierung serielle Kommunikation*/
  Serial.begin(115200);   
  pinMode(LED_BUILTIN , OUTPUT);
  get_response_m = false;

  while(!Serial) {
    digitalWrite(LED_BUILTIN , HIGH);
    delay (100);
    digitalWrite(LED_BUILTIN , LOW);
    delay (100);
  }
  next_measurement_time_s_m = millis()/1000;
  print_debug_info_f(F("NFC-THMS to Serial"),INFO_STANDARD_INFO);
  memset(info_array_m,0,sizeof(info_array_m));
  sprintf(info_array_m,"Debug level: 0x%x",debug_level);
  print_debug_info(INFO_STANDARD_INFO);

  /* Initialisierung NFC-Gerät via I2C */        
  while (!init_NT2S()) {      
    print_debug_info_f(F("Init failure"),INFO_ERROR_INFO);
    delay (1000);
  }
  sensor_available_m = false;
  fsm_state = FSM_IDLE;
}

void loop() {
  digitalWrite(LED_BUILTIN , HIGH); // To indicate some operation.
  memset(info_array_m,0,sizeof(info_array_m));
  sprintf(info_array_m,"FSM State: 0x%x",fsm_state);
  print_debug_info(INFO_FSM_STATE);

  /*>>> FINITE STATE MACHINE <<<*/
  switch(fsm_state){
    case FSM_IDLE: {
      if(continuous_measurement_m) {
        if ((millis()/1000) >= next_measurement_time_s_m) {
          print_debug_info_f(F("Time to do auto measurement."),INFO_STANDARD_INFO);
          do_insturction_to_set_m = NT2S_DO_SINGLE_MEASUREMENT;
          fsm_state = FSM_WRITE_INSTRUCTION;
          get_response_m = true;
          next_measurement_time_s_m = (millis()/1000) + cont_meas_interval_in_s_m;          
        } else {
          memset(info_array_m,0,sizeof(info_array_m));
          static unsigned int last_time_to_next_measurement = 0;
          unsigned int time_to_next_measurement = (next_measurement_time_s_m - (millis()/1000));
          if(last_time_to_next_measurement != time_to_next_measurement){
            last_time_to_next_measurement = next_measurement_time_s_m - (millis()/1000);
            sprintf(info_array_m,"Next meas. in [s]:%u",(unsigned int)(next_measurement_time_s_m - (millis()/1000)));
            print_debug_info(INFO_NEXT_MEASUREMENT_INFO); 
          }
        }
      }
      break;
    }
    //End case FSM_IDLE

    case FSM_SEARCH_SENSOR: {
      check_sensor_availability_5s();
      if(sensor_available_m) {
        print_debug_info_f(F("Sensor found!"),INFO_STANDARD_INFO);
        fsm_state = FSM_IDLE;
      } else {
        print_debug_info_f(F("No sensor found !!!"),INFO_STANDARD_INFO);
      }
      break;
    }
    //End case FSM_SEARCH_SENSOR
      
    case FSM_WRITE_INSTRUCTION: {
      sprintf(info_array_m,"Write inst.: 0x%x",do_insturction_to_set_m);
      print_debug_info(INFO_STANDARD_INFO);
      bool instruction_is_set = false;  
      if(!sensor_available_m) {check_sensor_availability_5s();}
      if(sensor_available_m) instruction_is_set = NT2S_set_instruction(do_insturction_to_set_m);
      if(instruction_is_set) {
        print_debug_info_f(F("Instruction is sent to tag"),INFO_STANDARD_INFO);
        fsm_state = FSM_IDLE;
      } else {
        error_no |= ERROR_SET_INSTRUCTION;
        fsm_state = FSM_ERROR;
      }
      if(get_response_m) {
        if(instruction_is_set){
          print_debug_info_f(F("Wait for response..."),INFO_STANDARD_INFO);
          delay(2000);
          if(do_insturction_to_set_m == NT2S_DO_SINGLE_MEASUREMENT) delay(3000);
          fsm_state = FSM_READ_TAG_DATA;
        }
        get_response_m = false;
      }
      break;
    }
    // End case FSM_WRITE_INSTRUCTION (0x03)

    case FSM_READ_TAG_DATA :{
      bool data_reading_ok = false;
      if(!sensor_available_m) {check_sensor_availability_5s();}
      if(sensor_available_m) data_reading_ok = NT2S_read_ndef_text(nfc_message_m, MAXIMAL_NDEF_MESSAGE_LENGT);
      if(data_reading_ok) {
        print_debug_info_f(F("Read data:"),INFO_STANDARD_INFO);
        sprintf(info_array_m,"%s",nfc_message_m);
        Serial.println(info_array_m);
        fsm_state = FSM_IDLE;
      } else {error_no |= ERROR_GET_DATA; fsm_state = FSM_ERROR;}
      break;
    }
    //End case FSM_READ_TAG_DATA

    case FSM_ERROR: {
      memset(info_array_m,0,sizeof(info_array_m));
      sprintf(info_array_m,"ERROR No: 0x%x",error_no);
      print_debug_info(INFO_ERROR_INFO);
      fsm_state = FSM_IDLE;
      error_no = ERROR_NO_ERROR;
      break;
    }
    //End case FSM_ERROR

    default: {
      fsm_state = FSM_ERROR;
      error_no |= ERROR_UNKNOWN_FSM_STATE;
      break;
    }
    //End case default
  }
  digitalWrite(LED_BUILTIN , LOW); // For operation indication
  if(!get_response_m)
    delay(FSM_SLOWDOWN); 
  check_for_serial_instructions();
  Serial.flush(); // Wait for serial communication to be finished.
}

void check_for_serial_instructions(void) {
  const int buffer_size = 50;
  char buf[buffer_size];
  if (Serial.available() > 0) {
    Serial.setTimeout(10000); //Give it 10s to complete Input
    int rlen = Serial.readBytesUntil('\n', buf, buffer_size);
    parse_serial_4_instruction(buf,rlen);
  }
}

/* Search sensor for 5s (5 times) certain time*/
// ToDo: Exclude "sensor_available" -> only "sensor_available_m"
void check_sensor_availability_5s(void) {
  print_debug_info_f(F("Search sensor..."),INFO_EXTENDED_INFO);
  uint8_t try_counter = 5;
  while(!NT2S_search_sensor()) {
    try_counter--;
    delay(1000);
    if(try_counter == 0) {
      sensor_available_m = false;
      error_no |= ERROR_SENSOR_CONNECTION_LOST;
      print_debug_info_f(F("No sensor found!"),INFO_STANDARD_INFO);
      return;
    } 
  } 
  sensor_available_m = true;
  return;
}

void print_debug_info(uart_debug_info_t info_level) {
  if(info_level & debug_level) {
    Serial.print(">>> ");
    Serial.println(info_array_m);
  }
}

void print_debug_info_f(const __FlashStringHelper * string_to_print,uart_debug_info_t info_level) {
  if(info_level & debug_level) {
    Serial.print(F(">>> "));
    Serial.println(string_to_print);
  }
}

void parse_serial_4_instruction(char buf[], int rlen) {
  //ToDo: Parse for instruction or change config etc
  print_debug_info_f(F("New serial instruction"),INFO_STANDARD_INFO);
  switch(buf[0]) {
    case SI_SEARCH_SENSOR: 
    case (SI_SEARCH_SENSOR|0x20):{ //Lower case
      if(rlen <= 2) {
        fsm_state = (fsm_state == FSM_SEARCH_SENSOR)?FSM_IDLE:FSM_SEARCH_SENSOR; // Toggle state
      } else if((rlen >= 3) && (buf[1] == ':')) {
        fsm_state = ((buf[2]=='T')||(buf[2]=='t'))?FSM_SEARCH_SENSOR:FSM_IDLE;
      } else {
        fsm_state = FSM_ERROR;
        error_no |= ERROR_SERIAL_INPUT;
        break;
      }
      print_debug_info_f(
        (fsm_state == FSM_SEARCH_SENSOR)?F("Inst.: START to search sensor."):F("Inst.: STOP to search sensor.")
        ,INFO_STANDARD_INFO);
      break;}
    case SI_DO_SINGLE_MEASUREMENT: 
    case (SI_DO_SINGLE_MEASUREMENT|0x20):{ //Lower case
      print_debug_info_f(F("Instruction to do single measurement."),INFO_STANDARD_INFO);
      do_insturction_to_set_m = NT2S_DO_SINGLE_MEASUREMENT;
      fsm_state = FSM_WRITE_INSTRUCTION;
      get_response_m = true;
      break;}
    case SI_SET_INSTRUCTION: 
    case (SI_SET_INSTRUCTION|0x20): {//Lower case
      print_debug_info_f(F("Inst.: Send Do-Inst. to Tag."),INFO_STANDARD_INFO); 
      //E.g. buf = "I:0x06" -> get config
      unsigned int new_instruction;
      sscanf(buf,"I:%x", &new_instruction);
      do_insturction_to_set_m = (uint8_t) new_instruction;
      memset(info_array_m,0,sizeof(info_array_m));
      sprintf(info_array_m,"New inst.: %x",do_insturction_to_set_m);
      print_debug_info(INFO_STANDARD_INFO);  
      fsm_state = (new_instruction != NT2S_ERROR)?FSM_WRITE_INSTRUCTION:FSM_ERROR;
      // ToDo: Parse instruction
      break;}
    case SI_READ: 
    case (SI_READ|0x20):{ //Lower case
      print_debug_info_f(F("Inst.: Read tag data."),INFO_STANDARD_INFO); 
      fsm_state = FSM_READ_TAG_DATA;
      break;}
    case SI_WRITE: 
    case (SI_WRITE|0x20):{ //Lower case
      print_debug_info_f(F("Inst.: Do write data to tag."),INFO_STANDARD_INFO); 
      fsm_state = FSM_WRITE_DATA;
      // ToDo: Parse write data
      break;}
    case SI_CONTINUOUS_MEASUREMENT:
    case (SI_CONTINUOUS_MEASUREMENT|0x20): {//Lower case 
      if(rlen <= 2) {
        continuous_measurement_m = !continuous_measurement_m;//toggle
      } else if((rlen >= 3) && (buf[1] == ':')) {
        continuous_measurement_m = ((buf[2]=='T')||(buf[2]=='t'))?true:false;
      } else {
        fsm_state = FSM_ERROR;
        error_no |= ERROR_SERIAL_INPUT;
        break;
      }
      if(continuous_measurement_m) next_measurement_time_s_m = 0;
      print_debug_info_f(
        (continuous_measurement_m)?F("Inst.: START continuous measurement."):F("Inst.: STOP continuous measurement.")
        ,INFO_STANDARD_INFO);
      break;
    }
    case SI_CHANGE_TIMING_4_CM:
    case (SI_CHANGE_TIMING_4_CM|0x20): {//Lower case 
      print_debug_info_f(F("Inst.: Change timing for continuous measurement"),INFO_STANDARD_INFO); 
      if(rlen >= 3) {
        uint16_t parsed_interval;
        int t = sscanf(&buf[1],":%u",&parsed_interval);
        if(t == 1) {
          cont_meas_interval_in_s_m = parsed_interval;
          memset(info_array_m,0,sizeof(info_array_m));
          sprintf(info_array_m,"New interval for continuous measurement: %u",parsed_interval);
          print_debug_info(INFO_STANDARD_INFO);
          break;
        } 
      } 
      fsm_state = FSM_ERROR;
      error_no |= ERROR_SERIAL_INPUT;
      break;
    }
    case SI_RESET:
    case (SI_RESET|0x20): { //Lower case
      //softwareReset::standard();
      break;
    }
    default:{
      print_debug_info_f(F("Unknown serial instruction!!"),INFO_ERROR_INFO); 
      fsm_state = FSM_ERROR;
      error_no |= ERROR_SERIAL_INPUT;
      break;}
  }
}