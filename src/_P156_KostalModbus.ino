
#include "_Plugin_Helper.h"

#ifdef USES_P156

//#######################################################################################################
//######################## Plugin 156: Kostal Inverter ########################
//#######################################################################################################

#define PLUGIN_156
#define PLUGIN_ID_156 156
#define PLUGIN_NAME_156 "Kostal - Inverter Modbus [Testing]"

#define CUSTOMTASK_STR_SIZE_P156 20
#define P156_MODEL PCONFIG(0)
#define P156_MODEL_LABEL PCONFIG_LABEL(0)

// IP-Adresse in Variable

#define P156_QUERY1 PCONFIG(1)
#define P156_QUERY2 PCONFIG(2)
#define P156_QUERY3 PCONFIG(3)
#define P156_QUERY4 PCONFIG(4)

#define P156_MODEL_DFLT 0   // plenticore
#define P156_QUERY1_DFLT 10 //
#define P156_QUERY2_DFLT 1  //
#define P156_QUERY3_DFLT 15 //
#define P156_QUERY4_DFLT 18 //

// IP Defines.
#define IP_ADDR_SIZE_P156 15 // IPv4 Addr String size (max length). e.g. 192.168.001.255
#define IP_BUFF_SIZE_P156 16 // IPv4 Addr Buffer size, including NULL terminator.
#define IP_MIN_SIZE_P156 7   // IPv4 Addr Minimum size, allows IP strings as short as 0.0.0.0
#define IP_SEP_CHAR_P156 '.' // IPv4 Addr segments are separated by a dot.
#define IP_SEP_CNT_P156 3    // IPv4 Addr dot separator count for valid address.
#define IP_STR_DEF_P156 "255.255.255.255"

#define P156_NR_OUTPUT_VALUES 4
#define P156_NR_OUTPUT_OPTIONS_MODEL0 19
#define P156_QUERY1_CONFIG_POS 1

// IDs fuer die Zuordnung der Werte in verschiedenen Querys

#define KPL_INVERTERSTATE 1;
#define KPL_TOTAL_DC_POWER 2;
#define KPL_HOME_CONS_BATT 3;
#define KPL_HOME_CONS_GRID 4;
#define KPL_HOME_CONS_PV 5;
#define KPL_TOTAL_HOME_CONS_BATT 6;
#define KPL_TOTAL_HOME_CONS_GRID 7;
#define KPL_TOTAL_HOME_CONS_PV 8;
#define KPL_TOTAL_HOME_CONSUMPTION 9;
#define KPL_TOTAL_AC_POWER 10;
#define KPL_BATT_CHARGE_CURRENT 11;
#define KPL_BATT_STATE_CHARGE 12;
#define KPL_BATT_TEMPERATUR 13;
#define KPL_BATT_VOLTAGE 14;
#define KPL_TOTAL_YIELD 15;
#define KPL_DAILY_YIELD 16;
#define KPL_YEARLY_YIELD 17;
#define KPL_MONTHLY_YIELD 18;

WiFiClient p156_client;

// Funktionen
// Forward declaration helper functions
const __FlashStringHelper *p156_getQueryString(uint8_t query);
const __FlashStringHelper *p156_getQueryValueString(uint8_t query);
unsigned int p156_getRegister(uint8_t query, uint8_t model);
float p156_readVal(uint8_t query, unsigned int model);
bool p156_validateIp(const String &ipStr);
bool p156_sendRequest(uint8_t query);
unsigned int p156_parseValues(uint8_t query);
void p156_deleteValues(unsigned int model);
// Const Variables

struct p156_dataStructKPL
{
  int lenValue;
  int datatyp; // 0 = float; 1 = int16
  byte dataRequest[12];
  float value;
  p156_dataStructKPL(int xLenValue, int xDatatyp, byte xDataRequest[12], float xValue)
  {
    lenValue = xLenValue;
    datatyp = xDatatyp;
    for (int i = 0; i < 12; i++)
    {
      dataRequest[i] = xDataRequest[i];
    }
    value = xValue;
  }
};
byte p156_reqfree[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};                            // 0x38 - 56
byte p156_reqInverterState[12] = {0, 0x01, 0, 0, 0, 0x06, 0x47, 0x03, 0, 0x38, 0, 0x02}; // 0x38 - 56
byte p156_reqTotalDCpower[12] = {0, 0x01, 0, 0, 0, 0x06, 0x47, 0x03, 0, 0x64, 0, 0x02};  // DC Power{0, 0x01, 0, 0, 0, 0x06, 0x47, 0x03, 0, 0x63, 0, 0x01};  // 0x64 - 100

byte p156_reqHomeConsumptionBattery[12] = {0, 0x01, 0, 0, 0, 0x06, 0x47, 0x03, 0, 0x6A, 0, 0x02}; // 0x6A - 106 [Watt]
byte p156_reqHomeConsumptionGrid[12] = {0, 0x01, 0, 0, 0, 0x06, 0x47, 0x03, 0, 0x6C, 0, 0x02};    // 0x6E - 108 [Watt]
byte p156_reqHomeConsumptionPV[12] = {0, 0x01, 0, 0, 0, 0x06, 0x47, 0x03, 0, 0x74, 0, 0x02};      // 0x74 - 116 [Watt]

byte p156_reqTotalHomeConsumptionBattery[12] = {0, 0x01, 0, 0, 0, 0x06, 0x47, 0x03, 0, 0x6E, 0, 0x02}; // 0x6E - 110 [WattStunden]
byte p156_reqTotalHomeConsumptionGrid[12] = {0, 0x01, 0, 0, 0, 0x06, 0x47, 0x03, 0, 0x70, 0, 0x02};    // 0x70 - 112 [WattStunden]
byte p156_reqTotalHomeConsumptionPV[12] = {0, 0x01, 0, 0, 0, 0x06, 0x47, 0x03, 0, 0x72, 0, 0x02};      // 0x72 - 114 [WattStunden]

byte p156_reqTotalHomeConsumption[12] = {0, 0x01, 0, 0, 0, 0x06, 0x47, 0x03, 0, 0x76, 0, 0x02}; // 0x76 - 118

byte p156_reqTotalACpower[12] = {0, 0x01, 0, 0, 0, 0x06, 0x47, 0x03, 0, 0xAC, 0, 0x02}; // 0xAC - 172

byte p156_reqBatteryChargeCurrent[12] = {0, 0x01, 0, 0, 0, 0x06, 0x47, 0x03, 0, 0xBE, 0, 0x02}; // 0xBE - 190
byte p156_reqBatteryStateOfCharge[12] = {0, 0x01, 0, 0, 0, 0x06, 0x47, 0x03, 0, 0xD2, 0, 0x02}; // 0xD2 - 210 %
byte p156_reqBatteryTemperature[12] = {0, 0x01, 0, 0, 0, 0x06, 0x47, 0x03, 0, 0xD6, 0, 0x02};   // 0xD6 - 214 °C
byte p156_reqBatteryVoltage[12] = {0, 0x01, 0, 0, 0, 0x06, 0x47, 0x03, 0, 0xD8, 0, 0x02};       // 0xD8 - 216 V

byte p156_reqTotalYield[12] = {0, 0x01, 0, 0, 0, 0x06, 0x47, 0x03, 0x01, 0x40, 0, 0x02};   // 0x140 - 320 Wh
byte p156_reqDailyYield[12] = {0, 0x01, 0, 0, 0, 0x06, 0x47, 0x03, 0x01, 0x42, 0, 0x02};   // 0x142 - 322 Wh
byte p156_reqYearlyYield[12] = {0, 0x01, 0, 0, 0, 0x06, 0x47, 0x03, 0x01, 0x44, 0, 0x02};  // 0x144 - 324 Wh
byte p156_reqMonthlyYield[12] = {0, 0x01, 0, 0, 0, 0x06, 0x47, 0x03, 0x01, 0x46, 0, 0x02}; // 0x146 - 326 Wh

p156_dataStructKPL p156_myData[P156_NR_OUTPUT_OPTIONS_MODEL0] =
    {
        p156_dataStructKPL(1, 0, p156_reqfree, 0),
        p156_dataStructKPL(2, 1, p156_reqInverterState, 0),
        p156_dataStructKPL(2, 0, p156_reqTotalDCpower, 0),
        p156_dataStructKPL(2, 0, p156_reqHomeConsumptionBattery, 0),
        p156_dataStructKPL(2, 0, p156_reqHomeConsumptionGrid, 0),
        p156_dataStructKPL(2, 0, p156_reqHomeConsumptionPV, 0),
        p156_dataStructKPL(2, 0, p156_reqTotalHomeConsumptionBattery, 0),
        p156_dataStructKPL(2, 0, p156_reqTotalHomeConsumptionGrid, 0),
        p156_dataStructKPL(2, 0, p156_reqTotalHomeConsumptionPV, 0),
        p156_dataStructKPL(2, 0, p156_reqTotalHomeConsumption, 0),
        p156_dataStructKPL(2, 0, p156_reqTotalACpower, 0),
        p156_dataStructKPL(2, 0, p156_reqBatteryChargeCurrent, 0),
        p156_dataStructKPL(2, 0, p156_reqBatteryStateOfCharge, 0),
        p156_dataStructKPL(2, 0, p156_reqBatteryTemperature, 0),
        p156_dataStructKPL(2, 0, p156_reqBatteryVoltage, 0),
        p156_dataStructKPL(2, 0, p156_reqTotalYield, 0),
        p156_dataStructKPL(2, 0, p156_reqDailyYield, 0),
        p156_dataStructKPL(2, 0, p156_reqYearlyYield, 0),
        p156_dataStructKPL(2, 0, p156_reqMonthlyYield, 0)};

/*
  get Kostal STPx000TL Modbus ("sunspec") datasheet here: https://www.photovoltaikforum.com/core/attachment/81082-ba-kostal-interface-modbus-tcp-sunspec-pdf/
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
boolean p156_MyInit = false;
uint8_t p156_step = 0;
uint16_t p156_send_count = 0;
uint16_t p156_send_errorcount = 0;
uint16_t p156_reconnectcount = 0;
String p156_IP = "";

int p156_outputOptionsAct;

boolean Plugin_156(uint8_t function, struct EventStruct *event, String &string)
{
  boolean success = false;

  switch (function)
  {

  case PLUGIN_DEVICE_ADD:
  {
    Device[++deviceCount].Number = PLUGIN_ID_156;
    Device[deviceCount].Type = DEVICE_TYPE_DUMMY;
    Device[deviceCount].VType = Sensor_VType::SENSOR_TYPE_QUAD;
    Device[deviceCount].Ports = 0;
    Device[deviceCount].PullUpOption = false;
    Device[deviceCount].InverseLogicOption = false;
    Device[deviceCount].FormulaOption = true;
    Device[deviceCount].ValueCount = P156_NR_OUTPUT_VALUES;
    Device[deviceCount].SendDataOption = true;
    Device[deviceCount].TimerOption = true;
    Device[deviceCount].GlobalSyncOption = true;
    break;
  }

  case PLUGIN_GET_DEVICENAME:
  {
    string = F(PLUGIN_NAME_156);
    break;
  }

  case PLUGIN_GET_DEVICEVALUENAMES:
  {
    for (uint8_t i = 0; i < VARS_PER_TASK; ++i)
    {
      if (i < P156_NR_OUTPUT_VALUES)
      {
        uint8_t choice = PCONFIG(i + P156_QUERY1_CONFIG_POS);
        safe_strncpy(
            ExtraTaskSettings.TaskDeviceValueNames[i],
            p156_getQueryValueString(choice),
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
    P156_MODEL = P156_MODEL_DFLT;
    // TODO IP
    P156_QUERY1 = P156_QUERY1_DFLT;
    P156_QUERY2 = P156_QUERY2_DFLT;
    P156_QUERY3 = P156_QUERY3_DFLT;
    P156_QUERY4 = P156_QUERY4_DFLT;

    success = true;
    break;
  }

  case PLUGIN_WEBFORM_LOAD:
  {
    //< IP anzeigen
    char ipString[IP_BUFF_SIZE_P156] = "";
    String msgStr;

    addFormSubHeader(""); // Blank line, vertical space.
    addFormHeader(F("Default Settings"));

    String strings[1];
    LoadCustomTaskSettings(event->TaskIndex, strings, 1, CUSTOMTASK_STR_SIZE_P156);

    safe_strncpy(ipString, strings[0], IP_BUFF_SIZE_P156);

    // LOG
    String log1 = F("Kostal: WebLoad=");
    log1 += event->TaskIndex;
    log1 += F(" IPAdresse=");
    log1 += strings[0];
    addLogMove(LOG_LEVEL_INFO, log1);

    addFormTextBox(F("IPv4 Address"), getPluginCustomArgName(0), ipString, IP_ADDR_SIZE_P156);
    msgStr = F("Typical Installations use IP Address ");
    msgStr += F(IP_STR_DEF_P156);
    addFormNote(msgStr);

    // LOG
    String log2 = F("Kostal: ipString=");
    log2 += event->TaskIndex;
    log2 += F(" IPAdresse=");
    log2 += ipString;
    addLogMove(LOG_LEVEL_INFO, log2);
    //> IP anzeigen

    //< Model und verschiedene Optionen der Werte anzeigen
    {
      const __FlashStringHelper *options_model[1] = {F("KPL")};
      addFormSelector(F("Model Type"), P156_MODEL_LABEL, 1, options_model, nullptr, P156_MODEL);
    }
    {
      const uint8_t model = PCONFIG(0);                      // TODO
      uint8_t outputOptions = P156_NR_OUTPUT_OPTIONS_MODEL0; // Default immer Model0
      if (model == 1)
        outputOptions = P156_NR_OUTPUT_OPTIONS_MODEL0;
      // In a separate scope to free memory of String array as soon as possible
      //sensorTypeHelper_webformLoad_simple();//sensorTypeHelper_webformLoad_header();
      const __FlashStringHelper *options[outputOptions];
      for (int i = 0; i < outputOptions; ++i) // Test int i = 0; ; i < P156_NR_OUTPUT_OPTIONS
      {
        options[i] = p156_getQueryString(i);
      }
      for (uint8_t i = 0; i < P156_NR_OUTPUT_VALUES; ++i)
      {
        const uint8_t pconfigIndex = i + P156_QUERY1_CONFIG_POS;
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
    char ipString[IP_BUFF_SIZE_P156] = {0};
    char deviceTemplate[1][CUSTOMTASK_STR_SIZE_P156];
    String errorStr;
    String msgStr;

    LoadTaskSettings(event->TaskIndex);

    // Check IP Address.  Hier wird die IP-Adresse aus dem Eingabefenster ausgelesen und in ipString geschrieben
    if (!safe_strncpy(ipString, webArg(getPluginCustomArgName(0)), IP_BUFF_SIZE_P156))
    {
      // msgStr = getCustomTaskSettingsError(0); // Report string too long.
      // errorStr += msgStr;
      // msgStr    = wolStr + msgStr;
      // addLog(LOG_LEVEL_INFO, msgStr);
    }

    if (strlen(ipString) == 0)
    { // IP Address missing, use default value (without webform warning).
      strcpy_P(ipString, String(F(IP_STR_DEF_P156)).c_str());
      msgStr += F("Loaded Default IP = ");
      msgStr += F(IP_STR_DEF_P156);
      addLogMove(LOG_LEVEL_INFO, msgStr);
    }
    else if (strlen(ipString) < IP_MIN_SIZE_P156)
    { // IP Address too short, load default value. Warn User.
      strcpy_P(ipString, String(F(IP_STR_DEF_P156)).c_str());

      msgStr = F("Provided IP Invalid (Using Default). ");
      errorStr += msgStr;
      msgStr += F("[");
      msgStr += F(IP_STR_DEF_P156);
      msgStr += F("]");
      addLogMove(LOG_LEVEL_INFO, msgStr);
    }
    else if (!p156_validateIp(ipString))
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
      p156_IP = ipString;
    }
    // Save the user's IP Address parameters into Custom Settings.
    safe_strncpy(deviceTemplate[0], ipString, IP_BUFF_SIZE_P156);
    // Save IP String parameters.
    SaveCustomTaskSettings(event->TaskIndex, reinterpret_cast<const uint8_t *>(&deviceTemplate), sizeof(deviceTemplate));
    //> IP Adresse einlesen

    //< Model und ausgewaehlte Daten einlesen
    P156_MODEL = getFormItemInt(P156_MODEL_LABEL);
    // Save output selector parameters.
    for (uint8_t i = 0; i < P156_NR_OUTPUT_VALUES; ++i)
    {
      const uint8_t pconfigIndex = i + P156_QUERY1_CONFIG_POS;
      const uint8_t choice = PCONFIG(pconfigIndex);
      sensorTypeHelper_saveOutputSelector(event, pconfigIndex, i, p156_getQueryValueString(choice));
    }
    P156_MODEL = getFormItemInt(P156_MODEL_LABEL);
    //> Model und ausgewaehlte Daten einlesen

    p156_MyInit = false; // Force device setup next time
    success = true;
    break;
  }

  case PLUGIN_INIT:
  {
    p156_client.stop();
    p156_deleteValues(P156_MODEL);
    if (P156_MODEL == 1)
      p156_outputOptionsAct = P156_NR_OUTPUT_OPTIONS_MODEL0;
    else
      p156_outputOptionsAct = P156_NR_OUTPUT_OPTIONS_MODEL0;
    // IP Adresse auslesen
    char ipString[IP_BUFF_SIZE_P156] = "";
    String strings[1];
    LoadCustomTaskSettings(event->TaskIndex, strings, 1, CUSTOMTASK_STR_SIZE_P156);
    safe_strncpy(ipString, strings[0], IP_BUFF_SIZE_P156);
    p156_IP = ipString;

    p156_step = 0;
    p156_send_count = 0;
    p156_send_errorcount = 0;
    p156_reconnectcount = 0;
    p156_MyInit = true;
    success = true;
    if (p156_MyInit)
    {
      String log5 = F("Modbus: Init=");
      log5 += event->TaskIndex;
      log5 += F(" Model=");
      log5 += P156_MODEL;
      log5 += F(" IP=");
      log5 += p156_IP;
      addLogMove(LOG_LEVEL_INFO, log5);
    }
    break;
  }

  case PLUGIN_EXIT:
  {
    p156_MyInit = false;
    p156_client.stop();    
    p156_deleteValues(P156_MODEL);
    break;
  }

  case PLUGIN_READ:
  {
    if (p156_MyInit)
    {
      int model = P156_MODEL;
      UserVar[event->BaseVarIndex + 0] = p156_readVal(P156_QUERY1, model);
      UserVar[event->BaseVarIndex + 1] = p156_readVal(P156_QUERY2, model);
      UserVar[event->BaseVarIndex + 2] = p156_readVal(P156_QUERY3, model);
      UserVar[event->BaseVarIndex + 3] = p156_readVal(P156_QUERY4, model);
      success = true;
      break;
    }
    break;
  }

  case PLUGIN_TEN_PER_SECOND: // PLUGIN_TEN_PER_SECOND:
  {
    int lquery = 0;        
    switch (p156_step)
    {
    case 10: // Daten von Query 1 bei Wechselrichter anfragen      
      if (P156_QUERY1 != 0)
      {
        lquery = P156_QUERY1;
      }
      p156_step = 11;
      break;
    case 11: // Nach Query 1 einen Zyklus Pause
      p156_step = 20;
      break;
    case 20: // Daten von Query 2 bei Wechselrichter anfragen
      if (P156_QUERY2 != 0)
      {
        lquery = P156_QUERY2;
      }
      p156_step = 21;
      break;
    case 21: // Nach Query 2 einen Zyklus Pause
      p156_step = 30;
      break;
    case 30: // Daten von Query 3 bei Wechselrichter anfragen
      if (P156_QUERY3 != 0)
      {
        lquery = P156_QUERY3;
      }
      p156_step = 31;
      break;
    case 31: // Nach Query 3 einen Zyklus Pause
      p156_step = 40;
      break;
    case 40: // Daten von Query 4 bei Wechselrichter anfragen
      if (P156_QUERY4 != 0)
      {
        lquery = P156_QUERY4;
      }
      p156_step = 41;
      break;
    case 41: // Nach Query 4 einen Zyklus Pause
      p156_step = 10;
      break;
    default:
      p156_step = 10;
      break;
    }
    if (lquery != 0)
    {
      boolean error = p156_sendRequest(lquery);
      if (error)
      {
        error = p156_parseValues(lquery);
      }
      if (!error)
      {
        p156_send_errorcount ++;        
        if (p156_send_errorcount > 10)
        {
          p156_send_errorcount = 0;
          p156_client.flush();
          p156_client.stop(); 
          p156_deleteValues(P156_MODEL);
          p156_reconnectcount ++;
        }
      }
    }
    String logSend =  F("Kostal : ");
    if (loglevelActiveFor(LOG_LEVEL_DEBUG))
    {
      logSend +=  F("SendError : ");
      logSend += (String)p156_send_errorcount;
    }
    logSend +=  F("ReConnect : ");
    logSend += (String)p156_reconnectcount;
    if (loglevelActiveFor(LOG_LEVEL_DEBUG) || p156_step == 10)
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
float p156_readVal(uint8_t query, unsigned int model)
{
  if (model == 0)
  { // KPL
    switch (query)
    {
    default:
      return p156_myData[query].value;
    }
  }
  return 0;
}

unsigned int p156_getRegister(uint8_t query, uint8_t model)
{
  if (model == 0)
  { // KPL
    switch (query)
    {
    case 1:
      return KPL_INVERTERSTATE;
    case 2:
      return KPL_TOTAL_DC_POWER;
    case 3:
      return KPL_HOME_CONS_BATT;
    case 4:
      return KPL_HOME_CONS_GRID;
    case 5:
      return KPL_HOME_CONS_PV;
    case 6:
      return KPL_TOTAL_HOME_CONS_BATT;
    case 7:
      return KPL_TOTAL_HOME_CONS_GRID;
    case 8:
      return KPL_TOTAL_HOME_CONS_PV;
    case 9:
      return KPL_TOTAL_HOME_CONSUMPTION;
    case 10:
      return KPL_TOTAL_AC_POWER;
    case 11:
      return KPL_BATT_CHARGE_CURRENT;
    case 12:
      return KPL_BATT_STATE_CHARGE;
    case 13:
      return KPL_BATT_TEMPERATUR;
    case 14:
      return KPL_BATT_VOLTAGE;
    case 15:
      return KPL_TOTAL_YIELD;
    default:
      return query;
    }
  }
  return 0;
}

const __FlashStringHelper *p156_getQueryString(uint8_t query)
{
  switch (query)
  {
  case 1:
    return F("INVERTERSTATE");
  case 2:
    return F("TOTAL_DC_POWER");
  case 3:
    return F("HOME_CONS_BATT");
  case 4:
    return F("HOME_CONS_GRID");
  case 5:
    return F("HOME_CONS_PV");
  case 6:
    return F("TOTAL_HOME_CONS_BATT");
  case 7:
    return F("TOTAL_HOME_CONS_GRID");
  case 8:
    return F("TOTAL_HOME_CONS_PV");
  case 9:
    return F("TOTAL_HOME_CONSUMPTION");
  case 10:
    return F("TOTAL_AC_POWER");
  case 11:
    return F("BATT_CHARGE_CURRENT");
  case 12:
    return F("BATT_STATE_CHARGE");
  case 13:
    return F("BATT_TEMPERATUR");
  case 14:
    return F("BATT_VOLTAGE");
  case 15:
    return F("TOTAL_YIELD");
  case 16:
    return F("DAILY_YIELD");
  case 17:
    return F("YEARLY_YIELD");
  case 18:
    return F("MONTHLY_YIELD");
  }
  return F("");
}

const __FlashStringHelper *p156_getQueryValueString(uint8_t query)
{
  switch (query)
  {
  case 1:
    return F("Inverterstate");
  case 2:
    return F("Total_DC_Power");
  case 3:
    return F("Home_Cons_Batt");
  case 4:
    return F("Home_Cons_Grid");
  case 5:
    return F("Home_Cons_PV");
  case 6:
    return F("Total_Home_Cons_Batt");
  case 7:
    return F("Total_Home_Cons_Grid");
  case 8:
    return F("Total_Home_Cons_PV");
  case 9:
    return F("Total_Home_Consumption");
  case 10:
    return F("Total_AC_Power");
  case 11:
    return F("Batt_Charge_Current");
  case 12:
    return F("Batt_State_Charge");
  case 13:
    return F("Batt_Temperatur");
  case 14:
    return F("Batt_Voltage");
  case 15:
    return F("Total_Yield");
  case 16:
    return F("Daily_Yield");
  case 17:
    return F("Yearly_Yield");
  case 18:
    return F("Monthly_Yield");
  }
  return F("");
}

bool p156_validateIp(const String &ipStr)
// ************************************************************************************************
// p156_validateIp(): IPv4 Address validity checker.
// Arg: IP Address String in dot separated format (192.168.1.255).
// Return true if IP string appears legit.
{
  IPAddress ip;
  unsigned int length = ipStr.length();

  if ((length < IP_MIN_SIZE_P156) || (length > IP_ADDR_SIZE_P156))
  {
    return false;
  }
  else if (ip.fromString(ipStr) == false)
  { // ThomasTech's Trick to Check IP for valid formatting.
    return false;
  }

  return true;
}

bool p156_sendRequest(uint8_t query)
{
  char *lIP = &p156_IP[0]; // assign the address of 1st character of the string to a pointer to char
  String log = F("Kostal : Sendrequest ");
  log += p156_IP;
  log += '|';
  log += query;
  addLogMove(LOG_LEVEL_DEBUG, log);
  if (!p156_client.connected())
  {
    if (!p156_client.connect(lIP, 1502, 1000))
    { // MODBUS port for Kostal is always 1502
      addLog(LOG_LEVEL_INFO, F("Kostal   : SendRequest; connection failed"));    
      return 0;
    }
  }
  int lLen = sizeof(p156_myData[query].dataRequest);
  if (lLen > 0)
  {
    addLog(LOG_LEVEL_DEBUG, F("Kostal   : SendRequest; Sending"));
    if (p156_send_count <= 0 || p156_send_count > 65535) //Min Max Anzahl erreicht
    {
      p156_send_count = 1;
    }
    else
    {
      p156_send_count += 1; //Request hochzaehlen
    }
    log += '(';
    log += p156_send_count;
    log += ')';
    uint8_t HBy =  (uint8_t) (p156_send_count >> 8);
    uint8_t LBy = (uint8_t) (p156_send_count);
    p156_myData[query].dataRequest[0] = HBy;
    p156_myData[query].dataRequest[1] = LBy;

    byte *lPointer = &p156_myData[query].dataRequest[0];
    p156_client.write(lPointer, lLen); // p156_client.write(&reqTotalDCpower[0], sizeof(reqTotalDCpower)); //-->geht
    // Log
    if (loglevelActiveFor(LOG_LEVEL_DEBUG))
    {
      for (int i = 0; i < lLen; i++)
      {
        log += '|';
        log += p156_myData[query].dataRequest[i];
      }
      addLogMove(LOG_LEVEL_DEBUG, log);
    }
    return 1;
  }
  else
  {
    log += F("Daten zu kurz");
    addLogMove(LOG_LEVEL_INFO, log);
    return 0;
  }
}

unsigned int p156_parseValues(uint8_t query)
{
  String log = F("Kostal : parseValues ");
  uint8_t high1 = 0, low1 = 0, high2 = 0, low2 = 0;
  unsigned long timeout = millis();
  while (p156_client.available() < 11)
  {
    delay(1); // important to service the tcp stack
    if (millis() - timeout > 2000)
    {
      p156_client.flush();//Wenn ich zulange brauche besser Puffer loeschen und neu versuchen
      return 0;
    }
  }
  int bytesToReceive = p156_client.available();
  log += '(';  
  log += query;
  log += ',';
  log += bytesToReceive;
  log += ')';

  uint8_t llen = p156_myData[query].lenValue;
  byte b = 0;
  for (int a = 0; a < bytesToReceive - llen * 2; a++)
  {
    b = p156_client.read();
    if (a== bytesToReceive - llen * 2 - 9)
    {
      high1 = b;
    }
    if (a== bytesToReceive - llen * 2 - 8)
    {
      low1 = b;
    }
    log += '|';
    log += b;  
  }
  uint16_t lreceivedId = ((high1 << 8) + (low1 << 0));  

  log += '(';
  log += p156_send_count;  
  log += '=';
  log += lreceivedId;  
  log += ')';
  if (p156_send_count != lreceivedId)
  { 
    p156_client.flush();//Wenn ich alte Daten bekomme oder Puffer alte Daten hat Puffer loeschen und neu versuchen
    addLogMove(LOG_LEVEL_INFO, log);
    return 0;
  }
  high1 = 0, low1 = 0, high2 = 0, low2 = 0;
  if (llen > 0)
  {
    high1 = p156_client.read();
  }
  if (llen > 1)
  {
    low1 = p156_client.read();
  }
  if (llen > 2)
  {
    high2 = p156_client.read();
  }
  if (llen > 3)
  {
    low2 = p156_client.read();
  }

  float lValue = 0.0;
  if (p156_myData[query].datatyp == 1)
  {
    lValue = ((high2 << 24) + (low2 << 16) + (high1 << 8) + (low1 << 0));
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

  p156_myData[query].value = lValue;

  log += '|';
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

void p156_deleteValues(unsigned int model)
{
  if (model == 0)
  {
    for (int i = 0; i < P156_NR_OUTPUT_OPTIONS_MODEL0; i++)
    {
      p156_myData[i].value = 0;
    }
  }
}

#endif // USES_P156
