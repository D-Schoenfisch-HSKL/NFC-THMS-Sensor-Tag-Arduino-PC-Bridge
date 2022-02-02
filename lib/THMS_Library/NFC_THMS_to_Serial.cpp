/**************************************************************************/
/*!
 *   @file: NFC_THMS_to_Serial.cpp
 *   @autor: Pascal Flint, David Schönfisch (Hochschule Kaiserslautern)
 * 
 *   @details: Bibliothek zum Ansteuern eines NFC-Sensor-Tags mit einem PN532 modul.
 *   @date: 08.12.2021 
 * 
 *   @version: V1.1  - Stable
 *   Getestet mit NFC-THMS-Sensor-Tag. 
 *    Hardware Version V1
 *    Software Version V1.4
 *   DFRobot_PN532.h version V1.0
*/
/**************************************************************************/

#include <stdint.h>
#include <DFRobot_PN532.h>
#include <NFC_THMS_to_Serial.h>

/*>>>------------------------------------------------------------*/
/* >> START: Symbols, Enums, Macros & Typedefs*/
DFRobot_PN532::sCard_t NFCcard;        /* Strukturvariable in welche Angaben zum Tag-Typ abgelegt werden */

#define  PN532_IRQ      (2)           // Interrupt pin. Irrelevant im polling mode
#define  POLLING        (1)           // Polling mode


#define MEM_BYTES       (84)  /* Anzahl Bytes, die aus dem Speicher ausgelesen werden; 4 pro Page */
#define DATA_BLOCKS     (7)   /* Anzahl Infos, in die die Messdaten unterteilt sind, im siebten Block steht nur "FE" */
#define START_BLOCK     (4)   // First Block to be read from NTAG via NFC

#define NDEF_START_SIGN 0x03  // Start sign for NDEF message in NFC tag raw data
#define NDEF_END_SIGN   0xFE  // End sign for NDEF message in NFC tag raw data

#define INITIAL_WRITE_DATA_ARRAY {NDEF_START_SIGN, 0x0D, 0xD1, 0x01,\
                                  0x09, 0x54, 0x02,  'd',\
                                   'e',  'D',  'o',  ':',\
                                   '0',  '2',  ';', NDEF_END_SIGN,\
                                  0x00, 0x00, 0x00, 0x00,\
                                  0x00, 0x00, 0x00, 0x00 }  /* Initialwerte für NFC-Daten Array */
/* >> END: Symbols, Enums, Macros & Typedefs */


/*>>>------------------------------------------------------------*/
/* >> START: Local Variables */
DFRobot_PN532_IIC  nfc(PN532_IRQ, POLLING);   /* Instanz zum Ansteuern des PN532 via I2C */
/* >> END: Local Variables */

/*>>>------------------------------------------------------------*/
/* >> START: Prototypes (Internal Functions) */
/************************************************************************************
 * Convert one uint8_t value to a char array of two
 * @return: True if succesful
 * @param[in] byte_value:	uint8 value to be converted
 * @param[in] int_as_char_array_of_2:	pointer to char array for converted hex-chars. Length needs to be 2.
 ************************************************************************************/
static bool byte2hexChar(byte byte_value, char *int_as_char_array_of_2);

/************************************************************************************
 * Reads raw data from thms-sensor.
 * @return: True if succesful.
 * @param[in] read_tag_data:	Pointer to array for raw data. (!Length must be > data_array_length).
 * @param[in] data_array_length:	Length of data to be read. 
 ************************************************************************************/
bool read_data(uint8_t read_tag_data[], size_t data_array_length);

/************************************************************************************
 * Search ndef text data entry in raw data
 * @return: True if succesful.
 * @param[in] raw_data_array:	Pointer to array for raw data. 
 * @param[out] text_start_index: Pointer to variable for start-point of text in raw data. 
 * @param[out] text_length:	Pointer to variable for length of text.
 ************************************************************************************/
bool search_text_ndef(uint8_t raw_data_array[], uint8_t max_length, uint8_t * text_start_index_p, uint8_t * text_length_p);

/* >> END: Prototypes */

/*>>>------------------------------------------------------------*/
/* >> START: External Functions */
bool init_NT2S() {
  bool init_OK_indicator = nfc.begin();
  return init_OK_indicator;
}

bool NT2S_search_sensor(void) {
  if (nfc.scan()) {     /* Prüfen Anwesenheit NFC-Tag */
    NFCcard = nfc.getInformation();     // Tag-Infoprmations (UID, AQTA, Type, ...) 
    delay(50); // Hilft möglicherweise bei Fehlermeldung ">>> Wrong tag-type: Ultralight"
    if (memcmp(NFCcard.cardType, "Ultralight", 10) != 0) {  // ENTFERNEN ?!?!
      if(NFCcard.AQTA[1] == 0x44) {
        return true;
      } else {        
        //Serial.print(">>> Error ATQA: 0x"); Serial.println(NFCcard.AQTA[1]); 
        // AQTA vermutlich Schreibfehler und sollte ATQA sein!!!!  => Answert to Request Type A
      }
    } else {
      // Serial.print(">>> Wrong tag-type: "); Serial.println(NFCcard.cardType);
      // ToDo: Gelegentlich wird (memcmp(NFCcard.cardType, "Ultralight", 10) == 0) erkannt und zugleich ist NFCcard.cardType==Ultralight
    }
  }
  return false;
}

bool NT2S_read_ndef_text(uint8_t message_array[], uint8_t max_length) {
  //Serial.print(F("Size of message_array: ")); Serial.println((int)(sizeof(message_array)/sizeof(message_array[0])),DEC); // Antwort = 255 -> ?!? O.o Kann nicht sein   
  uint8_t text_start_index = 0;
  uint8_t text_length = 0;
  bool read_success_indicator = read_data(message_array,max_length);

  if (read_success_indicator) {
    search_text_ndef(message_array,max_length,&text_start_index,&text_length);
    if (((text_length+text_start_index) < max_length) && (text_length>3)) {
      text_start_index = text_start_index+2; // Exclude language code -> +2 -2
      text_length = text_length-2;      // Exclude language code -> +2 -2
      memmove(message_array,&message_array[text_start_index],text_length);
      memset(&message_array[text_length],'\0',(max_length-(text_length)));
      return true;
    } else {
      Serial.println(F(">>> NDEF message size error!!!"));
      Serial.print(F(">>> >Start: "));Serial.println(text_start_index);
      Serial.print(F(">>> >Length: "));Serial.println(text_length);
    }
  }
  return false;
}

// Checken ob "sizeof(memory_data_array)" so OK
bool NT2S_read_raw(uint8_t memory_data_array[], uint8_t length) {
  return read_data(memory_data_array,length);
}

/*
NDEF-Textnachricht mit Do-Instruction in Memory des Sensor-Tags schreiben
*/
bool NT2S_set_instruction(uint8_t do_instruction){
  char instruction[2];
  uint8_t data[24] = INITIAL_WRITE_DATA_ARRAY;
  if(!byte2hexChar(do_instruction, instruction)) return false;
  data[12] = instruction[0];
  data[13] = instruction[1];
  for (unsigned int i = 0 ; i < 6; i++ ) {  // Write 6 Seiten (=24 Bytes)
    unsigned int try_counter = 5;
    bool write_success = false;
    do {
      write_success = nfc.writeNTAG(i+START_BLOCK, &data[i*4]);
      if(!write_success) delay(200);
      try_counter--;
    } while((try_counter != 0) && (write_success != true));
    if (!write_success) return false;
  }
  return true;
}

bool NT2S_set_do2_instruction(void){
  uint8_t data[24] = INITIAL_WRITE_DATA_ARRAY; 
  for (unsigned int i = 0 ; i < 6; i++ ) {
    unsigned int try_counter = 5;
    bool write_success = false;
    do {
      write_success = nfc.writeNTAG(i+START_BLOCK, &data[i*4]);
      if(!write_success) delay(200);
      try_counter--;
    } while((try_counter != 0) && (write_success != true));
    if (!write_success) return false;
  }
  return true;
}


/*
Überprüfen ob do_instruction bekannt ist.
ToDo!!
*/
bool check_do_instruction(uint8_t do_instruction) {
  /*ToDo .........!!!
  nt2s_do_instructions_t return_value = NT2S_ERROR;
  switch(array_of_2chars[0]){
    case ('0'): {
      switch(array_of_2chars[1]) {
        case ('0'): return_value = NT2S_INIT; break;
        case ('1'): return_value = NT2S_IDLE; break;
        case ('2'): return_value = NT2S_DO_SINGLE_MEASUREMENT; break;
        case ('4'): return_value = NT2S_NFCS_I2C_RST; break;
        case ('5'): return_value = NT2S_CHANGE_CONFIG; break;
        case ('6'): return_value = NT2S_GET_CONFIG; break;
        default: return_value = NT2S_ERROR;  break;
      }
      break;
    }
  }
  return return_value; 
  */
 return false;
}

/* >> END: External Functions */



/*>>>------------------------------------------------------------*/
/* >> START: Internal (Static) Functions */
static bool byte2hexChar(byte byte_value, char * int_as_char_array_of_2) {
	char hex_char_array[3] = "00"; // Null termination
  memset(int_as_char_array_of_2,'0',2); // Set char array to "00"
	int n = sprintf(hex_char_array,"%2X",byte_value); //print integer value to char array as hex-vale
  //Serial.print(">>> Got HEX instruction: 0x");Serial.println(byte_value,HEX);
	if((n == 1) || (n == 2)) { //Check if number of written chars is ok
    memcpy(int_as_char_array_of_2,hex_char_array,2);
    //Serial.print(">>> HEX-Ascii instruction: ");Serial.print(int_as_char_array_of_2[0]);Serial.println(int_as_char_array_of_2[1]);
		return true; 
	}
	return false;
}

/* Auslesen memory und Ablegen in Array. Auslesen nur Blockweise (4Bytes) möglich. Wenn z.B. data_array_length = 10, dann werden nur 2 Blöcke gelesen!*/ 
bool read_data(uint8_t read_tag_data[], size_t data_array_length){       
  //Serial.print(F("Size of read_tag_data: ")); Serial.println((int)(sizeof(read_tag_data)/sizeof(read_tag_data[0])),DEC);
  //Serial.print(F("Pointer to read_tag_data: ")); Serial.println((uint8_t) &read_tag_data[0],HEX);
  //int read_byte_lengt = 0;
  unsigned int number_of_blocks_to_read = (data_array_length/4);
  for (unsigned int block_no = 0; block_no < number_of_blocks_to_read; block_no++) {
    uint8_t data_array_write_pointer = 4*block_no;
    //Serial.print(F("Array write pointer: ")); Serial.println(data_array_write_pointer,DEC); // Antwort = 255 -> ?!? O.o Kann nicht sein
    uint8_t DFRobot_success_indicator = 0;
    int try_counter = 10;
    while ((DFRobot_success_indicator != 1) && (try_counter > 0)) {
      uint8_t read_data_array[5]; // Eins mehr zur Sicherheit 
      DFRobot_success_indicator = nfc.readNTAG(read_data_array, (block_no+START_BLOCK));

      //Serial.print(F("Success indicator: ")); Serial.println(DFRobot_success_indicator,DEC); 
      //Serial.print(F("Try counter: ")); Serial.println(try_counter,DEC); 
      if(data_array_write_pointer>data_array_length) Serial.println(F(">>> FATAL ARRAY WRITE ERROR "));
      memcpy(&read_tag_data[data_array_write_pointer], read_data_array, 4);
      try_counter--;
      if(DFRobot_success_indicator != 1) delay(200);
      Serial.flush();
    }
    if(try_counter <= 0) {
      return false;
      Serial.println(F(">>> Read was not successful!")); Serial.flush(); 
    }
    //read_byte_lengt = block_no*4;
  }
  //if(read_length_p) *read_length_p = read_byte_lengt;
  //Serial.println(F("Read successful!")); Serial.flush(); 
  return true;   
}

/* Suchen von NDEF Text Message in gelesenen Rohdaten */
bool search_text_ndef(uint8_t raw_data_array[], uint8_t max_length, uint8_t * text_start_index_p,uint8_t * text_length_p){
  enum{NDEF_Start, Text_Start, NDEF_End};
  int search_state = NDEF_Start;
  if(max_length<5) return false;

  for (int i = 0; i < max_length; i++){
    switch(search_state) {
      case NDEF_Start:
        if(raw_data_array[i] == NDEF_START_SIGN) {
          search_state = Text_Start;
        }
        break;
      case Text_Start:
        if((raw_data_array[i] == 0x64) && (raw_data_array[i+1] == 0x65)) {  // ="de"
          search_state = NDEF_End;
          *text_start_index_p = i;
          i++;
        }
        break;
      case NDEF_End:
        if(raw_data_array[i] == NDEF_END_SIGN) {
          *text_length_p = i - (*text_start_index_p);
          return true;
        }
        break;
    }
  }
  return false;
}

/* >> END: Internal (Static) Functions */