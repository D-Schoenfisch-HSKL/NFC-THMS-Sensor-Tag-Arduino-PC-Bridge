#include <DFRobot_PN532.h>
#include <stdio.h>
#include <stdbool.h> 
#include <NFC_THMS_to_Serial.h>

// Version: V1.3

/*----------- ToDo -------------*/
//  - Handling serial conmmands to
//    - Value parsing to config continuous-measurement time
//    - Handling direct reading or writng to tag
//    - Set of special instruction to tag.
//    - Instruction to reset and reboot.
//  - Check if measurement is complete in ndef-text (!= Do: 2)
//  - Check error messages from tag (== Do:FF ???)

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
/*---------------------------------------*/

/*---------- Typedefinitions ------------*/
typedef enum {
  FSM_IDLE                      = 0x00,
  FSM_SEARCH_SENSOR             = 0x01,
  FSM_DO_MEASUREMENT            = 0x02,
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
  ERROR_SENSOR_COMMUNICATION    = (0x1 << 6), // = 0x0040
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

typedef char info_array_t[100];

/*------------ Global Variables ---------------*/
static finite_state_machine_state_t fsm_state;
static uint16_t error_no = ERROR_NO_ERROR;
static bool  continuous_measurement = DEFAULT_FOR_CONTINUOUS_MEASUREMENT;
static uint16_t cont_meas_interval_in_s = DEFAULT_MEASUREMENT_INTERVAL_IN_S;
static bool  sensor_available_m = false;
static unsigned long next_measurement_time_s = 0;
static uint8_t debug_level = (PRINT_DEBUG_INFO_ERROR*INFO_ERROR_INFO) 
                           | (PRINT_DEBUG_INFO_STANDAR*INFO_STANDARD_INFO) 
                           | (PRINT_DEBUG_INFO_FSM*INFO_FSM_STATE) 
                           | (PRINT_NEXT_MEASUREMENT_INFO*INFO_NEXT_MEASUREMENT_INFO)
                           | (PRINT_EXTENDED_INFO*INFO_EXTENDED_INFO);
static info_array_t info_array_m;
static message_array_t text_message; //Array for text message
static nt2s_do_instructions_t do_insturction_to_set_m;


/*------------ Function Declaration ---------------*/
void check_sensor_availability_5s(void); /* Search sensor for 5s (5 times) certain time*/
void check_for_serial_instructions(void);  // Maximal length for instruction is 50. Each instruction has to end with '\n'.
void parse_serial_4_instruction(char buf[], int rlen); 
bool set_instruction(nt2s_do_instructions_t do_instruction);
void print_debug_info(info_array_t info_text, uart_debug_info_t info_level);  // To print infos via USB-UART (Serial)
bool get_tag_data(message_array_t text_data_array);


void setup() {
  /* Initialisierung serielle Kommunikation*/
  Serial.begin(115200);   
  pinMode(LED_BUILTIN , OUTPUT);

  while(!Serial) {
    digitalWrite(LED_BUILTIN , HIGH);
    delay (100);
    digitalWrite(LED_BUILTIN , LOW);
    delay (100);
  }
  next_measurement_time_s = millis()/1000;
  memcpy(info_array_m, "NFC-THMS to Serial",19);
  print_debug_info(info_array_m,INFO_STANDARD_INFO);
  memset(info_array_m,0,sizeof(info_array_m));
  sprintf(info_array_m,"Debug level: 0x%x",debug_level);
  print_debug_info(info_array_m,INFO_STANDARD_INFO);

  /* Initialisierung NFC-Gerät via I2C */        
  while (!init_NT2S()) {      
  memcpy(info_array_m, "Init failure",13);
  print_debug_info(info_array_m,INFO_ERROR_INFO);
    delay (1000);
  }
  sensor_available_m = false;
  fsm_state = FSM_IDLE;
}

void loop() {
  digitalWrite(LED_BUILTIN , HIGH); // To indicate some operation.
  memset(info_array_m,0,sizeof(info_array_m));
  sprintf(info_array_m,"FSM State: 0x%x",fsm_state);
  print_debug_info(info_array_m,INFO_FSM_STATE);

  /*>>> FINITE STATE MACHINE <<<*/
  switch(fsm_state){
    case FSM_IDLE: {
      if(continuous_measurement) {
        if ((millis()/1000) >= next_measurement_time_s) {
          fsm_state = FSM_DO_MEASUREMENT;
          next_measurement_time_s = (millis()/1000) + cont_meas_interval_in_s;          
        } else {
          memset(info_array_m,0,sizeof(info_array_m));
          static unsigned int last_time_to_next_measurement = 0;
          unsigned int time_to_next_measurement = (next_measurement_time_s - (millis()/1000));
          if(last_time_to_next_measurement != time_to_next_measurement){
            last_time_to_next_measurement = next_measurement_time_s - (millis()/1000);
            sprintf(info_array_m,"Next measurement in [s]:%u",(unsigned int)(next_measurement_time_s - (millis()/1000)));
            print_debug_info(info_array_m,INFO_NEXT_MEASUREMENT_INFO); 
          }
        }
      }
      break;
    }//End case FSM_IDLE

    case FSM_SEARCH_SENSOR: {
      check_sensor_availability_5s();
      if(sensor_available_m) {
        memcpy(info_array_m, "Sensor found!",14);print_debug_info(info_array_m,INFO_STANDARD_INFO);
        fsm_state = FSM_IDLE;
      } else {
        memcpy(info_array_m, "No sensor found !!!",20);print_debug_info(info_array_m,INFO_STANDARD_INFO);
      }
      break;
    }//End case FSM_SEARCH_SENSOR

    case FSM_DO_MEASUREMENT: {
      memcpy(info_array_m, "Initiate measurement",21);print_debug_info(info_array_m,INFO_STANDARD_INFO);
      bool do_measurement_instruction_set = false;
      bool measurement_complete = false;
      if(!sensor_available_m) {check_sensor_availability_5s();}
      if(sensor_available_m) {
        do_measurement_instruction_set = set_instruction(NT2S_DO_SINGLE_MEASUREMENT);
        if(do_measurement_instruction_set) {  
          memcpy(info_array_m, "Wait for Tag to complete measurement...",40);print_debug_info(info_array_m,INFO_STANDARD_INFO);
          delay(6000); // Wait for complete Measurement
          if(!sensor_available_m) {check_sensor_availability_5s();}
          measurement_complete = get_tag_data(text_message);
          if(measurement_complete) {
            memcpy(info_array_m, "New measurement data:",22);print_debug_info(info_array_m,INFO_STANDARD_INFO);
            Serial.println(text_message);
            fsm_state = FSM_IDLE;
          } else {fsm_state = FSM_ERROR;}
        } else {fsm_state = FSM_ERROR;}
      } else {fsm_state = FSM_ERROR;}
      if(!do_measurement_instruction_set) {error_no |= ERROR_SET_INSTRUCTION;}
      if(!measurement_complete) {error_no |= ERROR_READ_NDEF_TEXT;}
      
      break;
    }//End case FSM_DO_MEASUREMENT (0x02)

    /* ToDo: Überprüfen.
    // Wenn enabled, dann gibt es Probleme beim Auslesen des Tags (NFC_THMS_to_Serial Library) !?!?!?!
    case FSM_WRITE_INSTRUCTION: {
      if(set_instruction(do_insturction_to_set_m)) {
        memcpy(info_array_m, "Instruction sent to tag", 24); 
        print_debug_info(info_array_m,INFO_STANDARD_INFO);
        fsm_state = FSM_IDLE;
      } else {
        error_no |= ERROR_SET_INSTRUCTION;
        fsm_state = FSM_ERROR;
      }
      break;
    } // End case FSM_WRITE_INSTRUCTION (0x03)
    */

    case FSM_ERROR: {
      memset(info_array_m,0,sizeof(info_array_m));
      sprintf(info_array_m,"ERROR No: 0x%x",error_no);
      print_debug_info(info_array_m,INFO_ERROR_INFO);
      fsm_state = FSM_IDLE;
      error_no = ERROR_NO_ERROR;
      break;
    }//End case FSM_ERROR

    default: {
      fsm_state = FSM_ERROR;
      error_no |= ERROR_UNKNOWN_FSM_STATE;
      break;
    }//End case default
  }
  digitalWrite(LED_BUILTIN , LOW); // For operation indication
  delay(FSM_SLOWDOWN); 
  check_for_serial_instructions();
}

bool set_instruction(nt2s_do_instructions_t do_instruction) {
  bool do_measurement_instruction_is_set = false;
  memcpy(info_array_m, "New do instruction to set",26);print_debug_info(info_array_m,INFO_EXTENDED_INFO);
  int try_counter = 5;
  do {
    if(!sensor_available_m) {check_sensor_availability_5s();} // Check for sensor   
    if(sensor_available_m) {
      memcpy(info_array_m, "Write instruction...",21);print_debug_info(info_array_m,INFO_EXTENDED_INFO);
      if(NT2S_set_instruction(do_instruction)) {
        do_measurement_instruction_is_set = true;
        memcpy(info_array_m, "Instruction writing successful.",32);print_debug_info(info_array_m,INFO_EXTENDED_INFO);
      }
    }
    try_counter--;
  } while((!do_measurement_instruction_is_set) && (try_counter != 0));
  
  return do_measurement_instruction_is_set;
}

bool get_tag_data(message_array_t text_data_array) {
  bool measurement_complete = false;
  int try_counter = 5;
  do {
    if(!sensor_available_m) {check_sensor_availability_5s();} // Check for sensor 
    if(sensor_available_m) {
      if(NT2S_read_ndef_text(text_data_array)) {
        if((memcmp(text_data_array, "Do:02", 5) != 0) && (memcmp(text_data_array, "Do: 2", 5) != 0)) { 
          measurement_complete = true;
        } else {              
          delay(1000);
        }
      } else {  
        delay(150);
        sensor_available_m = false;
      }
    }
    try_counter--;
  } while ((!measurement_complete) && (try_counter != 0));
  return measurement_complete;
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
  memcpy(info_array_m, "Search sensor...",17);print_debug_info(info_array_m,INFO_EXTENDED_INFO);
  uint8_t try_counter = 5;
  while(!NT2S_search_sensor()) {
    try_counter--;
    delay(1000);
    if(try_counter == 0) {
      sensor_available_m = false;
      error_no |= ERROR_SENSOR_CONNECTION_LOST;
      memcpy(info_array_m, "No sensor found!",17);print_debug_info(info_array_m,INFO_STANDARD_INFO);
      return;
    } 
  } 
  sensor_available_m = true;
  return;
}

void print_debug_info(info_array_t info_text_in,uart_debug_info_t info_level) {
  if(info_level & debug_level) {
    info_array_t info_text_to_print;
    sprintf(info_text_to_print,">>> %s",info_text_in);
    Serial.println(info_text_to_print);
  }
}

void parse_serial_4_instruction(char buf[], int rlen) {
  //ToDo: Parse for instruction or change config etc
  memcpy(info_array_m, "New serial instruction",23);print_debug_info(info_array_m,INFO_STANDARD_INFO);
  switch(buf[0]) {
    case SI_SEARCH_SENSOR: 
    case (SI_SEARCH_SENSOR|0x20):{ //Lower case
      if(fsm_state != FSM_SEARCH_SENSOR) {
        memcpy(info_array_m, "Inst.: START to search sensor.",31);
        fsm_state = FSM_SEARCH_SENSOR;
      } else {
        memcpy(info_array_m, "Inst.: STOP to search sensor.",30);
        fsm_state = FSM_IDLE;
      }
      print_debug_info(info_array_m,INFO_STANDARD_INFO);
      break;}
    case SI_DO_SINGLE_MEASUREMENT: 
    case (SI_DO_SINGLE_MEASUREMENT|0x20):{ //Lower case
      memcpy(info_array_m, "Instruction to do single measurement.",38);print_debug_info(info_array_m,INFO_STANDARD_INFO);
      fsm_state = FSM_DO_MEASUREMENT;
      break;}
    case SI_SET_INSTRUCTION: 
    case (SI_SET_INSTRUCTION|0x20): {//Lower case
      memcpy(info_array_m, "Inst.: Send Do-Inst. to Tag.",29);print_debug_info(info_array_m,INFO_STANDARD_INFO); 
      //E.g. buf = "I:0x06" -> get config
      char instruction_ascii[3] = "FF"; 
      sscanf(buf,"I:0x%s", instruction_ascii);
      do_insturction_to_set_m = instruction_ascii_2_enum(instruction_ascii);
      memset(info_array_m,0,sizeof(info_array_m));
      sprintf(info_array_m,"New instruction: 0x%x",do_insturction_to_set_m);
      print_debug_info(info_array_m,INFO_STANDARD_INFO);  
      fsm_state = (do_insturction_to_set_m != NT2S_ERROR)?FSM_WRITE_INSTRUCTION:FSM_ERROR;
      // ToDo: Parse instruction
      break;}
    case SI_READ: 
    case (SI_READ|0x20):{ //Lower case
      memcpy(info_array_m, "Inst.: Do read tag data.",25);print_debug_info(info_array_m,INFO_STANDARD_INFO); 
      fsm_state = FSM_READ_TAG_DATA;
      break;}
    case SI_WRITE: 
    case (SI_WRITE|0x20):{ //Lower case
      memcpy(info_array_m, "Inst.: Do write data to tag.",29);print_debug_info(info_array_m,INFO_STANDARD_INFO); 
      fsm_state = FSM_WRITE_DATA;
      // ToDo: Parse write data
      break;}
    case SI_CONTINUOUS_MEASUREMENT:
    case (SI_CONTINUOUS_MEASUREMENT|0x20): {//Lower case 
      continuous_measurement = !continuous_measurement;
      if(continuous_measurement) {
        memcpy(info_array_m, "Inst.: START continuous measurement",36); 
        next_measurement_time_s = millis()/1000;
      } else {
        memcpy(info_array_m, "Inst.: STOP continuous measurement",35);
      }
      print_debug_info(info_array_m,INFO_STANDARD_INFO); 
      break;
      }
    case SI_CHANGE_TIMING_4_CM:
    case (SI_CHANGE_TIMING_4_CM|0x20): {//Lower case 
        memcpy(info_array_m, "Inst.: Change timing for continuous measurement",48);print_debug_info(info_array_m,INFO_STANDARD_INFO); 
      // ToDo: Parse timing value
      break;}
    default:{
      memcpy(info_array_m, "Unknown serial instruction!!",29);print_debug_info(info_array_m,INFO_ERROR_INFO); 
      fsm_state = FSM_ERROR;
      error_no |= ERROR_SERIAL_INPUT;
      break;}
  }
}