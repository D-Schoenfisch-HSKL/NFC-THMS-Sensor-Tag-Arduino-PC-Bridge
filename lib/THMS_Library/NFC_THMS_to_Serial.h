/**************************************************************************/
/*!
 *   @file: NFC_THMS_to_Serial.h
 *   @autor: Pascal Flint, David Schönfisch (Hochschule Kaiserslautern)
 * 
 *   @details: Bibliothek zum Ansteuern eines NFC-Sensor-Tags mit einem PN532 modul.
 *   @date: 08.12.2021 
 * 
 *   @version: V1.1  - Stable
 *   Getestet mit NFC-THMS-Sensor-Tag. 
 *    Hardware Version V1
 *    Software Version V1.3
 *   DFRobot_PN532.h version V1.0
*/
/**************************************************************************/

#ifndef _BIBLIOTHEK_TEMPLATE_H_
#define _BIBLIOTHEK_TEMPLATE_H_

#include <stdint.h>
#include <DFRobot_PN532.h>

/*>>>------------------------------------------------------------*/
/* >> START: Symbols, Enums, Macros & Typedefs*/
#define MAX_BYTE_SIZE_TO_READ_FROM_TAG  60 //84

typedef enum {
	NT2S_INIT						   	= 0x00U,
	NT2S_IDLE 						   	= 0x01U,
	NT2S_DO_SINGLE_MEASUREMENT			= 0x02U, // Do single measurement and go to NFC_I2C_RST (power cycle)
	NT2S_NFCS_I2C_RST			        = 0x04U, // Silence NFC (for 1 s) and Reset i2c
	NT2S_CHANGE_CONFIG					= 0x05U, // Changes Config of Tag due to instruction followed on "Do:"-Instruction
	NT2S_GET_CONFIG						= 0x06U, // Get config and info for: Pulse-Length, Sensor-Signal type, Measurement-Signal type, firmware version...
	NT2S_ERROR							= 0xFFU  // Unknown instruction.
}nt2s_do_instructions_t;		// If changed update also "instruction_ascii_2_enum()" function!

typedef struct data_array_t {
	uint8_t length;
	char x[MAX_BYTE_SIZE_TO_READ_FROM_TAG];
}data_array_t;
/* >> END: Symbols, Enums, Macros & Typedefs */



/*>>>------------------------------------------------------------*/
/* >> START: External Functions (Deklarationen/Prototypen)*/

/************************************************************************************
 * @brief Do initialization
 * 
 * @return Boolean type, the result of operation.
 * @retval TRUE Initialization success
 * @retval FALSE Initialization failed
 ************************************************************************************/
bool init_NT2S(void); 

/************************************************************************************
 * @brief: Search for THMS-NFC Sensor-Tag and get its informations.
 * 
 * @return true: Sensor found.
 * @return false: No sensor found
 ************************************************************************************/
bool NT2S_search_sensor(void);

/************************************************************************************
 * @brief Reads NFC-THMS-Sensor-Tag text message to "message_array" (char array). 
 * 
 * @param message_array: Pointer for read NDEF-Text as uint8_t array
 * @param max_length: Length of message_array
 * @return true: Successful
 * @return false: Unsuccessful
 ************************************************************************************/
bool NT2S_read_ndef_text(uint8_t message_array[], uint8_t max_length);

/************************************************************************************
 * @brief Reads NFC-THMS-Sensor-Tag meamory data and copies result to "memory_data_array" (char array). 
 * 
 * @param memory_data_array: Pointer for read raw data as uint8_t array
 * @param length: Length of message_array
 * @return true: Successful
 * @return false: Unsuccesful
 ************************************************************************************/
bool NT2S_read_raw(uint8_t memory_data_array[], uint8_t length); 

/************************************************************************************
 * @brief Writes "Do-instruction" as NDEF-Message to sensor-tag
 * 
 * @param do_instruction: Do-Instruction number as uint8_t.
 * @return true: Successful
 * @return false: Unsuccessful
 ************************************************************************************/
bool NT2S_set_instruction(uint8_t do_instruction);

/************************************************************************************
 * ToDo
 * @brief Überprüfen ob do_instruction eine bekannte Instroction ist (in nt2s_do_instructions_t definiert)
 * @param do_instruction Do-instruction als byte
 * @return true: Valid instrucition
 * @return false: Invalid/Unknown insturction
 ************************************************************************************/
bool check_do_instruction(uint8_t do_instruction);

/* >> END: External Functions */

#endif /* BIBLIOTHEK_TEMPLATE_H_ */
/** @}*/
