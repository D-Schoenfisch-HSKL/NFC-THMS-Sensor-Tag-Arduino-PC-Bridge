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
#define MAX_BYTE_SIZE_TO_READ_FROM_TAG  84

typedef enum {
	NT2S_INIT						   	= 0x00U,
	NT2S_IDLE 						   	= 0x01U,
	NT2S_DO_SINGLE_MEASUREMENT			= 0x02U, // Do single measurement and go to NFC_I2C_RST (power cycle)
	NT2S_NFCS_I2C_RST			        = 0x04U, // Silence NFC (for 1 s) and Reset i2c
	NT2S_CHANGE_CONFIG					= 0x05U, // Changes Config of Tag due to instruction followed on "Do:"-Instruction
	NT2S_GET_CONFIG						= 0x06U, // Get config and info for: Pulse-Length, Sensor-Signal type, Measurement-Signal type, firmware version...
	NT2S_ERROR							= 0xFFU  // Unknown instruction.
}nt2s_do_instructions_t;		// If changed update also "instruction_ascii_2_enum()" function!

typedef byte rawdata_array_t[MAX_BYTE_SIZE_TO_READ_FROM_TAG];
typedef char message_array_t[MAX_BYTE_SIZE_TO_READ_FROM_TAG];
/* >> END: Symbols, Enums, Macros & Typedefs */



/*>>>------------------------------------------------------------*/
/* >> START: External Functions (Deklarationen/Prototypen)*/

/************************************************************************************
 * Do initialization
 * @fn init_nt2s
 * @brief Initialize the THMS-NFC Sensor-Bridge.
 * @return Boolean type, the result of operation.
 * @retval TRUE Initialization success
 * @retval FALSE Initialization failed
 ************************************************************************************/
bool init_NT2S(void); 

/************************************************************************************
 *  Search for THMS-NFC Sensor-Tag and get its informations.
 ************************************************************************************/
bool NT2S_search_sensor(void);

/************************************************************************************
 *  Reads NFC-THMS-Sensor-Tag text message to "message_array" (char array). 
 ************************************************************************************/
bool NT2S_read_ndef_text(message_array_t message_array);

/************************************************************************************
 *  Reads NFC-THMS-Sensor-Tag meamory data and copies result to "memory_data_array" (char array). 
 ************************************************************************************/
bool NT2S_read_raw(rawdata_array_t memory_data_array); 

/************************************************************************************
 *  Zurücksetzen der Variablen index_m 
 *	Zurücksetzen des Arrays info_pointer_array_m
 *	Kein Rückgabewert
 ************************************************************************************/
void reset(void);

/************************************************************************************
 *  Anweisung in Memory des Sensor-Tags schreiben
 *  Rückgabe: true, Schreibvorgang abgeschlossen
 ************************************************************************************/
bool NT2S_set_instruction(nt2s_do_instructions_t do_instruction);

/************************************************************************************
 * Umwandeln von 2 chars (ascii) zu "nt2s_do_instructions_t" enumeration
 * @fn instruction_ascii_2_enum
 * @brief Umwandeln von 2 chars (ascii) zu "nt2s_do_instructions_t" enumeration
 * @param array_of_2chars Do-instruction as two chars (ascii)
 * @retval Enumeration-typedef of do instruction
 ************************************************************************************/
nt2s_do_instructions_t instruction_ascii_2_enum(char array_of_2chars[]);

/* >> END: External Functions */

#endif /* BIBLIOTHEK_TEMPLATE_H_ */
/** @}*/
