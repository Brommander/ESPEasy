
#include "_Plugin_Helper.h"

#ifdef USES_P157

//#######################################################################################################
//######################## Plugin 157: Varta Storage ########################
//#######################################################################################################

#define PLUGIN_157
#define PLUGIN_ID_157 157
#define PLUGIN_NAME_157 "Varta - Storage Modbus [Testing]"

#define CUSTOMTASK_STR_SIZE_P157 20
#define P157_MODEL PCONFIG(0)
#define P157_MODEL_LABEL PCONFIG_LABEL(0)

// IP-Adresse in Variable

#define P157_QUERY1 PCONFIG(1)
#define P157_QUERY2 PCONFIG(2)
#define P157_QUERY3 PCONFIG(3)
#define P157_QUERY4 PCONFIG(4)

#define P157_MODEL_DFLT 0   // Element
#define P157_QUERY1_DFLT 1 //
#define P157_QUERY2_DFLT 2  //
#define P157_QUERY3_DFLT 4 //
#define P157_QUERY4_DFLT 5 //

// IP Defines.
#define IP_ADDR_SIZE_P157 15 // IPv4 Addr String size (max length). e.g. 192.168.001.255
#define IP_BUFF_SIZE_P157 16 // IPv4 Addr Buffer size, including NULL terminator.
#define IP_MIN_SIZE_P157 7   // IPv4 Addr Minimum size, allows IP strings as short as 0.0.0.0
#define IP_SEP_CHAR_P157 '.' // IPv4 Addr segments are separated by a dot.
#define IP_SEP_CNT_P157 3    // IPv4 Addr dot separator count for valid address.
#define IP_STR_DEF_P157 "255.255.255.255"

#define P157_NR_OUTPUT_VALUES 4
#define P157_NR_OUTPUT_OPTIONS_MODEL0 6
#define P157_QUERY1_CONFIG_POS 1

// IDs fuer die Zuordnung der Werte in verschiedenen Querys

#define ELM_STATE 1;//1065
#define ELM_CHARGE_POWER 2;//1066
#define ELM_TOTAL_CHARGE_LEVEL 3;//1068
#define ELM_TOTAL_CHARGE_ENERGY 4;//1069-1070
#define ELM_TOTAL_POWER_GRID 5;//1078


WiFiClient p157_client;

// Funktionen
// Forward declaration helper functions
const __FlashStringHelper *p157_getQueryString(uint8_t query);
const __FlashStringHelper *p157_getQueryValueString(uint8_t query);
unsigned int p157_getRegister(uint8_t query, uint8_t model);
float p157_readVal(uint8_t query, unsigned int model);
bool p157_validateIp(const String &ipStr);
bool p157_sendRequest(uint8_t query);
unsigned int p157_parseValues(uint8_t query);
void p157_deleteValues(unsigned int model);
// Const Variables

struct p157_dataStructELM
{
  int lenValue;
  int datatyp; // 0 = float; 1 = int16
  float factor;
  byte dataRequest[12];
  float value;
  p157_dataStructELM(int xLenValue, int xDatatyp, float xfactor, byte xDataRequest[12], float xValue)
  {
    lenValue = xLenValue;
    datatyp = xDatatyp;
    factor = xfactor;
    for (int i = 0; i < 12; i++)
    {
      dataRequest[i] = xDataRequest[i];
    }
    value = xValue;
  }
};
byte p157_reqfree[12] =     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
byte p157_state[12] =       {0, 0x01, 0, 0, 0, 0x06, 0xFF, 0x03, 0x04, 0x29, 0, 0x01}; // 1065
byte p157_chargePower[12] = {0, 0x01, 0, 0, 0, 0x06, 0xFF, 0x03, 0x04, 0x2A, 0, 0x01}; // 1066
byte p157_chargeLevel[12] = {0, 0x01, 0, 0, 0, 0x06, 0xFF, 0x03, 0x04, 0x2C, 0, 0x01};  // 1068

byte p157_chargeEnergy[12] = {0, 0x01, 0, 0, 0, 0x06, 0xFF, 0x03, 0x04, 0x2D, 0, 0x02}; // 1069-1070
byte p157_powerGrid[12] =    {0, 0x01, 0, 0, 0, 0x06, 0xFF, 0x03, 0x04, 0x36, 0, 0x01};    // 1078

p157_dataStructELM p157_myData[P157_NR_OUTPUT_OPTIONS_MODEL0] =
    {
        p157_dataStructELM(0, 1, 1.0, p157_reqfree, 0),
        p157_dataStructELM(2, 1, 1.0, p157_state, 0),
        p157_dataStructELM(2, 1, 1.0, p157_chargePower, 0),
        p157_dataStructELM(2, 1, 1.0, p157_chargeLevel, 0),
        p157_dataStructELM(4, 1, 1.0, p157_chargeEnergy, 0),
        p157_dataStructELM(2, 1, -1.0, p157_powerGrid, 0)};

/*
  get Varta STPx000TL Modbus ("sunspec") datasheet here: https://www.photovoltaikforum.com/core/attachment/81082-ba-Varta-interface-modbus-tcp-sunspec-pdf/
  Request 12 bytes:
  00 01 (Transaction id)
  00 00 (Protocol ID)
  00 06 (bytes following / length: 6)
  47 (unit ID 71 / is always 126 no matter what you configured)
  03 (Function code: read register)
  00 63 (register 99 (Total DC power)) note: according to datasheet: use register number-1 (100-1)
  00 01  (words to read: 0001)
*/
// Variables
boolean p157_MyInit = false;
uint8_t p157_step = 0;
uint16_t p157_send_count = 0;
uint16_t p157_send_errorcount = 0;
uint16_t p157_reconnectcount = 0;
String p157_IP = "";

int p157_outputOptionsAct;

boolean Plugin_157(uint8_t function, struct EventStruct *event, String &string)
{
  boolean success = false;

  switch (function)
  {

  case PLUGIN_DEVICE_ADD:
  {
    Device[++deviceCount].Number = PLUGIN_ID_157;
    Device[deviceCount].Type = DEVICE_TYPE_DUMMY;
    Device[deviceCount].VType = Sensor_VType::SENSOR_TYPE_QUAD;
    Device[deviceCount].Ports = 0;
    Device[deviceCount].PullUpOption = false;
    Device[deviceCount].InverseLogicOption = false;
    Device[deviceCount].FormulaOption = true;
    Device[deviceCount].ValueCount = P157_NR_OUTPUT_VALUES;
    Device[deviceCount].SendDataOption = true;
    Device[deviceCount].TimerOption = true;
    Device[deviceCount].GlobalSyncOption = true;
    break;
  }

  case PLUGIN_GET_DEVICENAME:
  {
    string = F(PLUGIN_NAME_157);
    break;
  }

  case PLUGIN_GET_DEVICEVALUENAMES:
  {
    for (uint8_t i = 0; i < VARS_PER_TASK; ++i)
    {
      if (i < P157_NR_OUTPUT_VALUES)
      {
        uint8_t choice = PCONFIG(i + P157_QUERY1_CONFIG_POS);
        safe_strncpy(
            ExtraTaskSettings.TaskDeviceValueNames[i],
            p157_getQueryValueString(choice),
            sizeof(ExtraTaskSettings.TaskDeviceValueNames[i]));
      }
      else
      {
        ZERO_FILL(ExtraTaskSettings.TaskDeviceValueNames[i]);
      }
    }
    break;
  }

  case PLUGIN_WEBFORM_SHOW_CONFIG:
  {
    string += serialHelper_getSerialTypeLabel(event);
    success = true;
    break;
  }

  case PLUGIN_SET_DEFAULTS:
  {
    P157_MODEL = P157_MODEL_DFLT;
    // TODO IP
    P157_QUERY1 = P157_QUERY1_DFLT;
    P157_QUERY2 = P157_QUERY2_DFLT;
    P157_QUERY3 = P157_QUERY3_DFLT;
    P157_QUERY4 = P157_QUERY4_DFLT;

    success = true;
    break;
  }

  case PLUGIN_WEBFORM_LOAD:
  {
    //< IP anzeigen
    char ipString[IP_BUFF_SIZE_P157] = "";
    String msgStr;

    addFormSubHeader(""); // Blank line, vertical space.
    addFormHeader(F("Default Settings"));

    String strings[1];
    LoadCustomTaskSettings(event->TaskIndex, strings, 1, CUSTOMTASK_STR_SIZE_P157);

    safe_strncpy(ipString, strings[0], IP_BUFF_SIZE_P157);

    // LOG
    String log1 = F("Varta: WebLoad=");
    log1 += event->TaskIndex;
    log1 += F(" IPAdresse=");
    log1 += strings[0];
    addLogMove(LOG_LEVEL_INFO, log1);

    addFormTextBox(F("IPv4 Address"), getPluginCustomArgName(0), ipString, IP_ADDR_SIZE_P157);
    msgStr = F("Typical Installations use IP Address ");
    msgStr += F(IP_STR_DEF_P157);
    addFormNote(msgStr);

    // LOG
    String log2 = F("Varta: ipString=");
    log2 += event->TaskIndex;
    log2 += F(" IPAdresse=");
    log2 += ipString;
    addLogMove(LOG_LEVEL_INFO, log2);
    //> IP anzeigen

    //< Model und verschiedene Optionen der Werte anzeigen
    {
      const __FlashStringHelper *options_model[1] = {F("ELM")};
      addFormSelector(F("Model Type"), P157_MODEL_LABEL, 1, options_model, nullptr, P157_MODEL);
    }
    {
      const uint8_t model = PCONFIG(0);                      // TODO
      uint8_t outputOptions = P157_NR_OUTPUT_OPTIONS_MODEL0; // Default immer Model0
      if (model == 1)
        outputOptions = P157_NR_OUTPUT_OPTIONS_MODEL0;
      // In a separate scope to free memory of String array as soon as possible
      //sensorTypeHelper_webformLoad_simple();//sensorTypeHelper_webformLoad_header();
      const __FlashStringHelper *options[outputOptions];
      for (int i = 0; i < outputOptions; ++i) // Test int i = 0; ; i < P157_NR_OUTPUT_OPTIONS
      {
        options[i] = p157_getQueryString(i);
      }
      for (uint8_t i = 0; i < P157_NR_OUTPUT_VALUES; ++i)
      {
        const uint8_t pconfigIndex = i + P157_QUERY1_CONFIG_POS;
        sensorTypeHelper_loadOutputSelector(event, pconfigIndex, i, outputOptions, options);
      }
    }
    //> Model und verschiedene Optionen der Werte anzeigen

    success = true;
    break;
  }

  case PLUGIN_WEBFORM_SAVE:
  {
    //< IP Adresse einlesen
    char ipString[IP_BUFF_SIZE_P157] = {0};
    char deviceTemplate[1][CUSTOMTASK_STR_SIZE_P157];
    String errorStr;
    String msgStr;

    LoadTaskSettings(event->TaskIndex);

    // Check IP Address.  Hier wird die IP-Adresse aus dem Eingabefenster ausgelesen und in ipString geschrieben
    if (!safe_strncpy(ipString, webArg(getPluginCustomArgName(0)), IP_BUFF_SIZE_P157))
    {
      // msgStr = getCustomTaskSettingsError(0); // Report string too long.
      // errorStr += msgStr;
      // msgStr    = wolStr + msgStr;
      // addLog(LOG_LEVEL_INFO, msgStr);
    }

    if (strlen(ipString) == 0)
    { // IP Address missing, use default value (without webform warning).
      strcpy_P(ipString, String(F(IP_STR_DEF_P157)).c_str());
      msgStr += F("Loaded Default IP = ");
      msgStr += F(IP_STR_DEF_P157);
      addLogMove(LOG_LEVEL_INFO, msgStr);
    }
    else if (strlen(ipString) < IP_MIN_SIZE_P157)
    { // IP Address too short, load default value. Warn User.
      strcpy_P(ipString, String(F(IP_STR_DEF_P157)).c_str());

      msgStr = F("Provided IP Invalid (Using Default). ");
      errorStr += msgStr;
      msgStr += F("[");
      msgStr += F(IP_STR_DEF_P157);
      msgStr += F("]");
      addLogMove(LOG_LEVEL_INFO, msgStr);
    }
    else if (!p157_validateIp(ipString))
    { // Unexpected IP Address value. Leave as-is, but Warn User.
      msgStr = F("WARNING, Please Review IP Address. ");
      errorStr += msgStr;
      msgStr += F("[");
      msgStr += ipString;
      msgStr += F("]");
      addLogMove(LOG_LEVEL_INFO, msgStr);
    }
    else // Am Ende in meine Variable ueberfuehren
    {
      p157_IP = ipString;
    }
    // Save the user's IP Address parameters into Custom Settings.
    safe_strncpy(deviceTemplate[0], ipString, IP_BUFF_SIZE_P157);
    // Save IP String parameters.
    SaveCustomTaskSettings(event->TaskIndex, reinterpret_cast<const uint8_t *>(&deviceTemplate), sizeof(deviceTemplate));
    //> IP Adresse einlesen

    //< Model und ausgewaehlte Daten einlesen
    P157_MODEL = getFormItemInt(P157_MODEL_LABEL);
    // Save output selector parameters.
    for (uint8_t i = 0; i < P157_NR_OUTPUT_VALUES; ++i)
    {
      const uint8_t pconfigIndex = i + P157_QUERY1_CONFIG_POS;
      const uint8_t choice = PCONFIG(pconfigIndex);
      sensorTypeHelper_saveOutputSelector(event, pconfigIndex, i, p157_getQueryValueString(choice));
    }
    P157_MODEL = getFormItemInt(P157_MODEL_LABEL);
    //> Model und ausgewaehlte Daten einlesen

    p157_MyInit = false; // Force device setup next time
    success = true;
    break;
  }

  case PLUGIN_INIT:
  {
    p157_client.stop();
    p157_deleteValues(P157_MODEL);
    if (P157_MODEL == 1)
      p157_outputOptionsAct = P157_NR_OUTPUT_OPTIONS_MODEL0;
    else
      p157_outputOptionsAct = P157_NR_OUTPUT_OPTIONS_MODEL0;
    // IP Adresse auslesen
    char ipString[IP_BUFF_SIZE_P157] = "";
    String strings[1];
    LoadCustomTaskSettings(event->TaskIndex, strings, 1, CUSTOMTASK_STR_SIZE_P157);
    safe_strncpy(ipString, strings[0], IP_BUFF_SIZE_P157);
    p157_IP = ipString;

    p157_step = 0;
    p157_send_count = 0;
    p157_send_errorcount = 0;
    p157_reconnectcount = 0;
    p157_MyInit = true;
    success = true;
    if (p157_MyInit)
    {
      String log5 = F("Modbus: Init=");
      log5 += event->TaskIndex;
      log5 += F(" Model=");
      log5 += P157_MODEL;
      log5 += F(" IP=");
      log5 += p157_IP;
      addLogMove(LOG_LEVEL_INFO, log5);
    }
    break;
  }

  case PLUGIN_EXIT:
  {
    p157_MyInit = false;
    p157_client.stop();    
    p157_deleteValues(P157_MODEL);
    break;
  }

  case PLUGIN_READ:
  {
    if (p157_MyInit)
    {
      int model = P157_MODEL;
      UserVar[event->BaseVarIndex + 0] = p157_readVal(P157_QUERY1, model);
      UserVar[event->BaseVarIndex + 1] = p157_readVal(P157_QUERY2, model);
      UserVar[event->BaseVarIndex + 2] = p157_readVal(P157_QUERY3, model);
      UserVar[event->BaseVarIndex + 3] = p157_readVal(P157_QUERY4, model);
      success = true;
      break;
    }
    break;
  }

  case PLUGIN_TEN_PER_SECOND: // PLUGIN_TEN_PER_SECOND:
  {
    int lquery = 0;
    switch (p157_step)
    {
    case 10: // Daten von Query 1 bei Wechselrichter anfragen
      if (P157_QUERY1 != 0)
      {
        lquery = P157_QUERY1;
      }
      p157_step = 11;
      break;
    case 11: // Nach Query 1 einen Zyklus Pause
      p157_step = 12;
      break;
    case 12: // Nach Query 1 einen Zyklus Pause
      p157_step = 20;
      break;
    case 20: // Daten von Query 2 bei Wechselrichter anfragen
      if (P157_QUERY2 != 0)
      {
        lquery = P157_QUERY2;
      }
      p157_step = 21;
      break;
    case 21: // Nach Query 2 einen Zyklus Pause
      p157_step = 22;
      break;
    case 22: // Nach Query 2 einen Zyklus Pause
      p157_step = 30;
      break;
    case 30: // Daten von Query 3 bei Wechselrichter anfragen
      if (P157_QUERY3 != 0)
      {
        lquery = P157_QUERY3;
      }
      p157_step = 31;
      break;
    case 31: // Nach Query 3 einen Zyklus Pause
      p157_step = 32;
      break;
    case 32: // Nach Query 3 einen Zyklus Pause
      p157_step = 40;
      break;
    case 40: // Daten von Query 4 bei Wechselrichter anfragen
      if (P157_QUERY4 != 0)
      {
        lquery = P157_QUERY4;
      }
      p157_step = 41;
      break;
    case 41: // Nach Query 4 einen Zyklus Pause
      p157_step = 42;
      break;
    case 42: // Nach Query 4 einen Zyklus Pause
      p157_step = 10;
      break;
    default:
      p157_step = 10;
      break;
    }
    if (lquery != 0)
    {       
      boolean error = p157_sendRequest(lquery);
      if (error)
      {
        error = p157_parseValues(lquery);
      }
      if (!error)
      {
        p157_send_errorcount ++;        
        if (p157_send_errorcount > 10)
        {
          p157_send_errorcount = 0;
          p157_client.flush();
          p157_client.stop(); 
          p157_deleteValues(P157_MODEL);
          p157_reconnectcount ++;
        }
      }      
    }
    String logSend =  F("Varta : ");
    if (loglevelActiveFor(LOG_LEVEL_DEBUG))
    {
      logSend +=  F("SendError : ");
      logSend += (String)p157_send_errorcount;
    }
    logSend +=  F("ReConnect : ");
    logSend += (String)p157_reconnectcount;
    if (loglevelActiveFor(LOG_LEVEL_DEBUG) || p157_step == 10)
    {
      addLogMove(LOG_LEVEL_INFO, logSend);
    }
    success = true;
    break;
  }
  }
  return success;
}

// Funktionen
float p157_readVal(uint8_t query, unsigned int model)
{
  if (model == 0)
  { // ELM
    switch (query)
    {
    default:
      return p157_myData[query].value;
    }
  }
  return 0;
}

unsigned int p157_getRegister(uint8_t query, uint8_t model)
{
  if (model == 0)
  { // ELM
    switch (query)
    {
    case 1:
      return ELM_STATE;
    case 2:
      return ELM_CHARGE_POWER;
    case 3:
      return ELM_TOTAL_CHARGE_LEVEL;
    case 4:
      return ELM_TOTAL_CHARGE_ENERGY;
    case 5:
      return ELM_TOTAL_POWER_GRID;    
    default:
      return query;
    }
  }
  return 0;
}

const __FlashStringHelper *p157_getQueryString(uint8_t query)
{
  switch (query)
  {
  case 1:
    return F("STATE");
  case 2:
    return F("CHARGE_POWER");
  case 3:
    return F("CHARGE_LEVEL");
  case 4:
    return F("CHARGE_ENERGY");
  case 5:
    return F("POWER_GRID");  
  }
  return F("");
}

const __FlashStringHelper *p157_getQueryValueString(uint8_t query)
{
  switch (query)
  {
  case 1:
    return F("State");
  case 2:
    return F("Charge_Power");
  case 3:
    return F("Charge_Level");
  case 4:
    return F("Charge_Energy");
  case 5:
    return F("Power_Grid");
  }
  return F("");
}

bool p157_validateIp(const String &ipStr)
// ************************************************************************************************
// p157_validateIp(): IPv4 Address validity checker.
// Arg: IP Address String in dot separated format (192.168.1.255).
// Return true if IP string appears legit.
{
  IPAddress ip;
  unsigned int length = ipStr.length();

  if ((length < IP_MIN_SIZE_P157) || (length > IP_ADDR_SIZE_P157))
  {
    return false;
  }
  else if (ip.fromString(ipStr) == false)
  { // ThomasTech's Trick to Check IP for valid formatting.
    return false;
  }

  return true;
}

bool p157_sendRequest(uint8_t query)
{
  char *lIP = &p157_IP[0]; // assign the address of 1st character of the string to a pointer to char
  String log = F("Varta : Sendrequest ");
  log += p157_IP;
  log += '|';
  log += query;
  addLogMove(LOG_LEVEL_DEBUG, log);
  if (!p157_client.connected())
  {
    if (!p157_client.connect(lIP, 502, 1000))
    { // MODBUS port for Varta is always 502
      addLog(LOG_LEVEL_INFO, F("Varta   : SendRequest; connection failed"));
      return 0;
    }
  }
  int lLen = sizeof(p157_myData[query].dataRequest);
  if (lLen > 0)
  {
    addLog(LOG_LEVEL_DEBUG, F("Varta   : SendRequest; Sending"));
    if (p157_send_count <= 0 || p157_send_count > 65535) //Min Max Anzahl erreicht
    {
      p157_send_count = 1;
    }
    else
    {
      p157_send_count += 1; //Request hochzaehlen
    }
    log += '(';
    log += p157_send_count;
    log += ')';
    uint8_t HBy =  (uint8_t) (p157_send_count >> 8);
    uint8_t LBy = (uint8_t) (p157_send_count);
    p157_myData[query].dataRequest[0] = HBy;
    p157_myData[query].dataRequest[1] = LBy;

    byte *lPointer = &p157_myData[query].dataRequest[0];
    p157_client.write(lPointer, lLen); // p157_client.write(&reqTotalDCpower[0], sizeof(reqTotalDCpower)); //-->geht
    // Log
    if (loglevelActiveFor(LOG_LEVEL_DEBUG))
    {
      for (int i = 0; i < lLen; i++)
      {
        log += '|';
        log += p157_myData[query].dataRequest[i];
      }
    }
    addLogMove(LOG_LEVEL_DEBUG, log);
    return 1;
  }
  else
  {
    log += F("Daten zu kurz");
    addLogMove(LOG_LEVEL_INFO, log);
    return 0;
  }
}

unsigned int p157_parseValues(uint8_t query)
{
  String log = F("Varta : parseValues ");
  uint8_t high1 = 0, low1 = 0, high2 = 0, low2 = 0;
  unsigned long timeout = millis();
  while (p157_client.available() < 11)
  {
    delay(1); // important to service the tcp stack
    if (millis() - timeout > 2000)
    {
      p157_client.flush();//Wenn ich zulange brauche besser Puffer loeschen und neu versuchen
      return 0;
    }
  }
  int bytesToReceive = p157_client.available();
  log += '(';  
  log += query;
  log += ',';
  log += bytesToReceive;
  log += ')';

  uint8_t llen = p157_myData[query].lenValue;
  byte b = 0;
  for (int a = 0; a < bytesToReceive - llen ; a++) //llen -> Anzahl Bytes
  { 
    b = p157_client.read();
    if (a == bytesToReceive - llen - 9)
    {
      high1 = b;
    }
    if (a== bytesToReceive - llen - 8)
    {
      low1 = b;
    }
    log += '|';
    log += b;   
  }
  uint16_t lreceivedId = ((high1 << 8) + (low1 << 0));  

  log += '(';
  log += p157_send_count;  
  log += '=';
  log += lreceivedId;  
  log += ')';
  if (p157_send_count != lreceivedId)
  { 
    p157_client.flush();//Wenn ich alte Daten bekomme oder Puffer alte Daten hat Puffer loeschen und neu versuchen
    addLogMove(LOG_LEVEL_INFO, log);
    return 0;
  }
  high1 = 0, low1 = 0, high2 = 0, low2 = 0;
  if (llen > 0)
  {
    high1 = p157_client.read();
  }
  if (llen > 1)
  {
    low1 = p157_client.read();
  }
  if (llen > 2)
  {
    high2 = p157_client.read();
  }
  if (llen > 3)
  {
    low2 = p157_client.read();
  }

  float lValue = 0.0;
  int16_t lValueInt = 0;
  if (p157_myData[query].datatyp == 1)
  {
    lValueInt = ((high1 << 8) + (low1 << 0));
    lValue = lValueInt;
  }
  else
  {
    unsigned char pBuffer[] = {low2, high2, low1, high1};
    memcpy(&lValue, pBuffer, sizeof(float)); // Kopieren des Arrays auf eine float-Variable
    // lValue = ((high2 << 24) + (low2 << 16) + (high1 << 8) + (low1 << 0)); --> geht nicht
    // lValue = ((high1 << 24) + (low1 << 16) + (high2 << 8) + (low2 << 0)); --> geht nicht
    // lValue = ((low2 << 24) + (high2 << 16) + (low1 << 8) + (high1 << 0)); --> geht nicht
    // lValue = ((low1 << 24) + (high1 << 16) + (low2 << 8) + (high2 << 0)); --> geht nicht
  }

  p157_myData[query].value = lValue * p157_myData[query].factor;

  log += ';';
  log += high1;
  log += '|';
  log += low1;
  log += '|';
  log += high2;
  log += '|';
  log += low2;
  addLogMove(LOG_LEVEL_DEBUG, log);
  return 1;
}

void p157_deleteValues(unsigned int model)
{
  if (model == 0)
  {
    for (int i = 0; i < P157_NR_OUTPUT_OPTIONS_MODEL0; i++)
    {
      p157_myData[i].value = 0;
    }
  }
}

#endif // USES_P157
