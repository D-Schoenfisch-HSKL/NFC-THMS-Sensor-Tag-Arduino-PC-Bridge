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
 *    Software Version V1.3
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
#define  POLLING        (0)           // Polling mode


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
uint8_t memory_m[MEM_BYTES];                  /* Array zum Zwischenspeichern des kompletten Speicherinhalts */
uint8_t split_array_m[DATA_BLOCKS];           /* Array zum Speichern der Infolängen */  
uint8_t info_pointer_array_m[DATA_BLOCKS];  /* Array zum Speichern der Startpositionen der einzelnen Infos */
int index_m;                              /* Variable zum Durchlaufen des Index des Arrays split_array_m */
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
bool read_data(byte read_tag_data[], size_t data_array_length);

/************************************************************************************
 * Search ndef text data entry in raw data
 * @return: True if succesful.
 * @param[in] raw_data_array:	Pointer to array for raw data. 
 * @param[out] text_start_index: Pointer to variable for start-point of text in raw data. 
 * @param[out] text_length:	Pointer to variable for length of text.
 ************************************************************************************/
bool search_text_ndef(rawdata_array_t raw_data_array, uint8_t * text_start_index_p, uint8_t * text_length_p);

void memory_pointer(void);
bool read_procedure(void);
/* >> END: Prototypes */

/*>>>------------------------------------------------------------*/
/* >> START: External Functions */
bool init_NT2S() {
  return nfc.begin();
}

void reset() {
    /* Zurücksetzen der Variablen index und des Arrays info_pointer_array_m */
  index_m = 0;                  
  for (int i = 0; i < DATA_BLOCKS; i++){ 
    info_pointer_array_m[i] = 0;
  }
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

bool NT2S_read_ndef_text(message_array_t message_array) {
  rawdata_array_t raw_data_array;
  uint8_t text_start_index = 0;
  uint8_t text_length = 0;
  bool read_success_indicator = read_data(raw_data_array,sizeof(rawdata_array_t));
  if ((read_success_indicator == true) & ((text_start_index+text_length) < sizeof(rawdata_array_t))){
    memset(message_array,0,sizeof(message_array_t));
    search_text_ndef(raw_data_array,&text_start_index,&text_length);
    // Exclude language code -> +2 -2
    memcpy(message_array,&raw_data_array[text_start_index+2],text_length-2);
    return true;
  }
  return false;
}

bool NT2S_read_raw(rawdata_array_t memory_data_array) {
  return read_data(memory_data_array,sizeof(rawdata_array_t));
}

/*
Anweisung in Memory des Sensor-Tags schreiben
*/
bool NT2S_set_instruction(nt2s_do_instructions_t do_instruction){
  char instruction[2];
  uint8_t data[24] = INITIAL_WRITE_DATA_ARRAY; 
  if(!byte2hexChar((uint8_t)do_instruction, instruction)) return false;
  data[12] = instruction[0]; // Checken ob richtige Stelle!
  data[13] = instruction[1]; // Checken ob richtige Stelle!
  //uint8_t tempdata[5] = {0, 0, 0, 0};       /* Zwischenspeicher für zu schreibende Daten */
  for (unsigned int i = 0 ; i < ((sizeof(data))/4); i++ ) {
    for (unsigned int try_counter = 0; try_counter <= 5; try_counter ++){
      if (try_counter == 5) return false;
      bool write_success = nfc.writeNTAG(i+START_BLOCK, &data[i*4]);
      if (write_success == true) break;
      delay(250);
    }
  }
  return true;
}


/*
Umwandeln von 2 ascii-zeichen zu "nt2s_do_instructions_t"
*/
nt2s_do_instructions_t instruction_ascii_2_enum(char array_of_2chars[]) {
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
}

/* >> END: External Functions */



/*>>>------------------------------------------------------------*/
/* >> START: Internal (Static) Functions */
static bool byte2hexChar(byte byte_value, char * int_as_char_array_of_2) {
	memset(int_as_char_array_of_2,0,2); // Set char array to "00"
	int n = sprintf(int_as_char_array_of_2,"%2x",byte_value); //print integer value to char array as hex-vale
	if((n == 1) || (n == 2)) { //Check if number of written chars is ok
		return true; 
	}
	return false;
}

/* Auslesen memory und Ablegen in Array */ 
bool read_data(byte read_tag_data[], size_t data_array_length){          
  //int read_byte_lengt = 0;
  for (int block_no = START_BLOCK ; block_no<((int)data_array_length/4); block_no++) {
    uint8_t DFRobot_success_indicator = 0; // ??!?!?!
    int try_counter = 12;
    while ((DFRobot_success_indicator != 1) && (try_counter > 0)) {
      DFRobot_success_indicator = nfc.readNTAG(&read_tag_data[4*(block_no-START_BLOCK)], block_no);
      try_counter--;
      if(DFRobot_success_indicator == -1) delay(250);
    }
    /*do {
      DFRobot_success_indicator = nfc.readNTAG(&read_tag_data[4*(block_no-START_BLOCK)], block_no);
      try_counter--; 
      if(DFRobot_success_indicator != 1) delay(500);
    } while (!((DFRobot_success_indicator == 1) || (try_counter <= 0)));*/
    if(try_counter <= 0) {
      Serial.print(">>> Read try-counter>10 for block: ");
      Serial.println(block_no,DEC);
      return false;
      break;
    }
    //read_byte_lengt = block_no*4;
  }
  //if(read_length_p) *read_length_p = read_byte_lengt;
  return true;     
}

/* Suchen von NDEF Text Message in gelesenen Rohdaten */
bool search_text_ndef(rawdata_array_t raw_data_array, uint8_t * text_start_index_p,uint8_t * text_length_p){
  enum{NDEF_Start, Text_Start, NDEF_End};
  int search_state = NDEF_Start;

  for (int i = 0; i < (int)sizeof(rawdata_array_t); i++){
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

/*
Herausfiltern Startpositionen der Infos in der Memory
*/
void memory_pointer(){
  int position = 0;               /* Variable, die den Wert der Positionen beim Durchlaufen der Memory annimmt */
  for (int i = 0; i < MEM_BYTES; i++){
    position++;
    if (memory_m[i] == 0x03 && memory_m[i + 8] == 0x65 && index_m == 0){
      info_pointer_array_m[index_m] = position - 1;
      index_m++;
    }
    else if (memory_m[i - 1] == 0x64 && memory_m[i] == 0x65 && index_m != 0){
      info_pointer_array_m[index_m] = position;
      index_m++;
    }
    else if (memory_m[i] == 0x3B && index_m != 0){
      info_pointer_array_m[index_m] = position;
      index_m++;
    }
    else if (memory_m[i] == 0xFE && index_m != 0){
      info_pointer_array_m[index_m] = position;
      index_m++;
    }
  }
}

/* Mehrfachdurchlauf read_data */
bool read_procedure(){
  int n = 0;
  while (read_data(memory_m,MEM_BYTES) == 0){
    n++;
    if (n == 10){
      Serial.println("Read failure. Auslesen abgebrochen.");
      return false;
    }
  }
  return true;
}

/* >> END: Internal (Static) Functions */