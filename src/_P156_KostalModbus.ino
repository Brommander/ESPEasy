#include "_Plugin_Helper.h"

#ifdef USES_P156

// #######################################################################################################
// ######################## Plugin 156: Inverter Modbus ########################
// #######################################################################################################
//  Kostal KPL  : Modbus TCP, Port 1502, Unit-ID 0x47, Func 0x03, IEEE754 floats
//  Sungrow SH  : Modbus TCP, Port 502,  Unit-ID 0x01, Func 0x04, U16/S16/U32/S32
//                Comm. address = Protocol address - 1
//                U32/S32: little-endian word order, big-endian byte order within word
//                Skalierungsfaktoren (z.B. /10 fuer 0.1-Schritte) per ESPEasy-Formel einstellen
// #######################################################################################################

#define PLUGIN_156
#define PLUGIN_ID_156 156
#define PLUGIN_NAME_156 "Inverter Modbus [Testing]"

#define CUSTOMTASK_STR_SIZE_P156 20
#define P156_MODEL PCONFIG(0)
#define P156_MODEL_LABEL PCONFIG_LABEL(0)

#define P156_QUERY1 PCONFIG(1)
#define P156_QUERY2 PCONFIG(2)
#define P156_QUERY3 PCONFIG(3)
#define P156_QUERY4 PCONFIG(4)

#define P156_PAUSE_MS PCONFIG(5)
#define P156_PAUSE_MS_LABEL PCONFIG_LABEL(5)
#define P156_PAUSE_MS_DFLT 100
#define P156_PAUSE_MS_MIN 10
#define P156_PAUSE_MS_MAX 5000

#define P156_MODEL_DFLT 0
#define P156_QUERY1_DFLT 10
#define P156_QUERY2_DFLT 1
#define P156_QUERY3_DFLT 15
#define P156_QUERY4_DFLT 18

// IP Defines
#define IP_ADDR_SIZE_P156 15
#define IP_BUFF_SIZE_P156 16
#define IP_MIN_SIZE_P156 7
#define IP_SEP_CHAR_P156 '.'
#define IP_SEP_CNT_P156 3
#define IP_STR_DEF_P156 "255.255.255.255"

#define P156_NR_OUTPUT_VALUES 4
#define P156_NR_OUTPUT_OPTIONS_MODEL0 19 // Kostal KPL
#define P156_NR_OUTPUT_OPTIONS_MODEL1 19 // Sungrow SH
#define P156_QUERY1_CONFIG_POS 1

#define KPL_INVERTERSTATE 1
#define KPL_TOTAL_DC_POWER 2
#define KPL_HOME_CONS_BATT 3
#define KPL_HOME_CONS_GRID 4
#define KPL_HOME_CONS_PV 5
#define KPL_TOTAL_HOME_CONS_BATT 6
#define KPL_TOTAL_HOME_CONS_GRID 7
#define KPL_TOTAL_HOME_CONS_PV 8
#define KPL_TOTAL_HOME_CONSUMPTION 9
#define KPL_TOTAL_AC_POWER 10
#define KPL_BATT_CHARGE_CURRENT 11
#define KPL_BATT_STATE_CHARGE 12
#define KPL_BATT_TEMPERATUR 13
#define KPL_BATT_VOLTAGE 14
#define KPL_TOTAL_YIELD 15
#define KPL_DAILY_YIELD 16
#define KPL_YEARLY_YIELD 17
#define KPL_MONTHLY_YIELD 18

WiFiClient p156_client;

// Forward declarations
const __FlashStringHelper *p156_getQueryString(uint8_t query, uint8_t model);
const __FlashStringHelper *p156_getQueryValueString(uint8_t query, uint8_t model);
unsigned int p156_getRegister(uint8_t query, uint8_t model);
float p156_readVal(uint8_t query, unsigned int model);
bool p156_validateIp(const String &ipStr);
bool p156_sendRequest(uint8_t query);
unsigned int p156_parseValues(uint8_t query);
void p156_deleteValues(unsigned int model);

// ============================================================
// Datenstruktur
// datatyp: 0 = float IEEE754 (Kostal memcpy)
//          1 = U32 / U16  unsigned int
//          2 = S32        signed 32-bit
//          3 = S16        signed 16-bit (z.B. Temperatur)
// lenValue: Anzahl Datenbytes in der Antwort (2=1 Register, 4=2 Register)
// ============================================================
struct p156_dataStructKPL
{
  int lenValue;
  int datatyp;
  byte dataRequest[12];
  float value;

  p156_dataStructKPL(int xLenValue, int xDatatyp, byte xDataRequest[12], float xValue)
  {
    lenValue = xLenValue;
    datatyp = xDatatyp;
    for (int i = 0; i < 12; i++)
      dataRequest[i] = xDataRequest[i];
    value = xValue;
  }
};

// ============================================================
// KOSTAL KPL – Modbus TCP, Port 1502, Unit 0x47, Func 0x03
// ============================================================
byte p156_reqfree[12] = {0, 0x01, 0, 0, 0, 0x06, 0x47, 0x03, 0x00, 0x00, 0, 0};
byte p156_reqInverterState[12] = {0, 0x01, 0, 0, 0, 0x06, 0x47, 0x03, 0x00, 0x38, 0, 0x02};               // 56
byte p156_reqTotalDCpower[12] = {0, 0x01, 0, 0, 0, 0x06, 0x47, 0x03, 0x00, 0x64, 0, 0x02};                // 100
byte p156_reqHomeConsumptionBattery[12] = {0, 0x01, 0, 0, 0, 0x06, 0x47, 0x03, 0x00, 0x6A, 0, 0x02};      // 106
byte p156_reqHomeConsumptionGrid[12] = {0, 0x01, 0, 0, 0, 0x06, 0x47, 0x03, 0x00, 0x6C, 0, 0x02};         // 108
byte p156_reqHomeConsumptionPV[12] = {0, 0x01, 0, 0, 0, 0x06, 0x47, 0x03, 0x00, 0x74, 0, 0x02};           // 116
byte p156_reqTotalHomeConsumptionBattery[12] = {0, 0x01, 0, 0, 0, 0x06, 0x47, 0x03, 0x00, 0x6E, 0, 0x02}; // 110
byte p156_reqTotalHomeConsumptionGrid[12] = {0, 0x01, 0, 0, 0, 0x06, 0x47, 0x03, 0x00, 0x70, 0, 0x02};    // 112
byte p156_reqTotalHomeConsumptionPV[12] = {0, 0x01, 0, 0, 0, 0x06, 0x47, 0x03, 0x00, 0x72, 0, 0x02};      // 114
byte p156_reqTotalHomeConsumption[12] = {0, 0x01, 0, 0, 0, 0x06, 0x47, 0x03, 0x00, 0x76, 0, 0x02};        // 118
byte p156_reqTotalACpower[12] = {0, 0x01, 0, 0, 0, 0x06, 0x47, 0x03, 0x00, 0xAC, 0, 0x02};                // 172
byte p156_reqBatteryChargeCurrent[12] = {0, 0x01, 0, 0, 0, 0x06, 0x47, 0x03, 0x00, 0xBE, 0, 0x02};        // 190
byte p156_reqBatteryStateOfCharge[12] = {0, 0x01, 0, 0, 0, 0x06, 0x47, 0x03, 0x00, 0xD2, 0, 0x02};        // 210
byte p156_reqBatteryTemperature[12] = {0, 0x01, 0, 0, 0, 0x06, 0x47, 0x03, 0x00, 0xD6, 0, 0x02};          // 214
byte p156_reqBatteryVoltage[12] = {0, 0x01, 0, 0, 0, 0x06, 0x47, 0x03, 0x00, 0xD8, 0, 0x02};              // 216
byte p156_reqTotalYield[12] = {0, 0x01, 0, 0, 0, 0x06, 0x47, 0x03, 0x01, 0x40, 0, 0x02};                  // 320
byte p156_reqDailyYield[12] = {0, 0x01, 0, 0, 0, 0x06, 0x47, 0x03, 0x01, 0x42, 0, 0x02};                  // 322
byte p156_reqYearlyYield[12] = {0, 0x01, 0, 0, 0, 0x06, 0x47, 0x03, 0x01, 0x44, 0, 0x02};                 // 324
byte p156_reqMonthlyYield[12] = {0, 0x01, 0, 0, 0, 0x06, 0x47, 0x03, 0x01, 0x46, 0, 0x02};                // 326

p156_dataStructKPL p156_myData[P156_NR_OUTPUT_OPTIONS_MODEL0] = {
    /* 0  */ p156_dataStructKPL(1, 0, p156_reqfree, 0),
    /* 1  */ p156_dataStructKPL(4, 1, p156_reqInverterState, 0),
    /* 2  */ p156_dataStructKPL(4, 0, p156_reqTotalDCpower, 0),
    /* 3  */ p156_dataStructKPL(4, 0, p156_reqHomeConsumptionBattery, 0),
    /* 4  */ p156_dataStructKPL(4, 0, p156_reqHomeConsumptionGrid, 0),
    /* 5  */ p156_dataStructKPL(4, 0, p156_reqHomeConsumptionPV, 0),
    /* 6  */ p156_dataStructKPL(4, 0, p156_reqTotalHomeConsumptionBattery, 0),
    /* 7  */ p156_dataStructKPL(4, 0, p156_reqTotalHomeConsumptionGrid, 0),
    /* 8  */ p156_dataStructKPL(4, 0, p156_reqTotalHomeConsumptionPV, 0),
    /* 9  */ p156_dataStructKPL(4, 0, p156_reqTotalHomeConsumption, 0),
    /* 10 */ p156_dataStructKPL(4, 0, p156_reqTotalACpower, 0),
    /* 11 */ p156_dataStructKPL(4, 0, p156_reqBatteryChargeCurrent, 0),
    /* 12 */ p156_dataStructKPL(4, 0, p156_reqBatteryStateOfCharge, 0),
    /* 13 */ p156_dataStructKPL(4, 0, p156_reqBatteryTemperature, 0),
    /* 14 */ p156_dataStructKPL(4, 0, p156_reqBatteryVoltage, 0),
    /* 15 */ p156_dataStructKPL(4, 0, p156_reqTotalYield, 0),
    /* 16 */ p156_dataStructKPL(4, 0, p156_reqDailyYield, 0),
    /* 17 */ p156_dataStructKPL(4, 0, p156_reqYearlyYield, 0),
    /* 18 */ p156_dataStructKPL(4, 0, p156_reqMonthlyYield, 0),
};

// ============================================================
// SUNGROW SH – Modbus TCP, Port 502, Unit 0x01, Func 0x04
// Comm. address = Protokoll-Adresse - 1
// U32/S32: little-endian word order, big-endian byte order within word
// ============================================================
byte p156_sg_reqfree[12] = {0, 0, 0, 0, 0, 0, 0x01, 0x04, 0x00, 0x00, 0, 0};
byte p156_sg_reqRunningState[12] = {0, 0x01, 0, 0, 0, 6, 0x01, 0x04, 0x32, 0xC7, 0, 1};       // 13000 U16
byte p156_sg_reqTotalDCPower[12] = {0, 0x01, 0, 0, 0, 6, 0x01, 0x04, 0x13, 0x98, 0, 2};       // 5017-5018 U32 W
byte p156_sg_reqBattPower[12] = {0, 0x01, 0, 0, 0, 6, 0x01, 0x04, 0x32, 0xDD, 0, 1};          // 13022 U16 W
byte p156_sg_reqLoadPower[12] = {0, 0x01, 0, 0, 0, 6, 0x01, 0x04, 0x32, 0xCF, 0, 2};          // 13008-13009 S32 W
byte p156_sg_reqExportPower[12] = {0, 0x01, 0, 0, 0, 6, 0x01, 0x04, 0x32, 0xD1, 0, 2};        // 13010-13011 S32 W (neg=Import)
byte p156_sg_reqDailyPVGen[12] = {0, 0x01, 0, 0, 0, 6, 0x01, 0x04, 0x32, 0xC9, 0, 1};         // 13002 U16 0.1kWh
byte p156_sg_reqTotalPVGen[12] = {0, 0x01, 0, 0, 0, 6, 0x01, 0x04, 0x32, 0xCA, 0, 2};         // 13003-13004 U32 0.1kWh
byte p156_sg_reqDailyImport[12] = {0, 0x01, 0, 0, 0, 6, 0x01, 0x04, 0x32, 0xEB, 0, 1};        // 13036 U16 0.1kWh
byte p156_sg_reqTotalImport[12] = {0, 0x01, 0, 0, 0, 6, 0x01, 0x04, 0x32, 0xEC, 0, 2};        // 13037-13038 U32 0.1kWh
byte p156_sg_reqTotalACPower[12] = {0, 0x01, 0, 0, 0, 6, 0x01, 0x04, 0x32, 0xE9, 0, 2};       // 13034-13035 S32 W
byte p156_sg_reqBattCurrent[12] = {0, 0x01, 0, 0, 0, 6, 0x01, 0x04, 0x32, 0xDC, 0, 1};        // 13021 U16 0.1A
byte p156_sg_reqBattSOC[12] = {0, 0x01, 0, 0, 0, 6, 0x01, 0x04, 0x32, 0xDE, 0, 1};            // 13023 U16 0.1%
byte p156_sg_reqBattTemp[12] = {0, 0x01, 0, 0, 0, 6, 0x01, 0x04, 0x32, 0xE0, 0, 1};           // 13025 S16 0.1degC
byte p156_sg_reqBattVoltage[12] = {0, 0x01, 0, 0, 0, 6, 0x01, 0x04, 0x32, 0xDB, 0, 1};        // 13020 U16 0.1V
byte p156_sg_reqTotalOutput[12] = {0, 0x01, 0, 0, 0, 6, 0x01, 0x04, 0x13, 0x8B, 0, 2};        // 5004-5005 U32 0.1kWh
byte p156_sg_reqDailyOutput[12] = {0, 0x01, 0, 0, 0, 6, 0x01, 0x04, 0x13, 0x8A, 0, 1};        // 5003 U16 0.1kWh
byte p156_sg_reqDailyBattDischarge[12] = {0, 0x01, 0, 0, 0, 6, 0x01, 0x04, 0x32, 0xE1, 0, 1}; // 13026 U16 0.1kWh
byte p156_sg_reqTotalBattDischarge[12] = {0, 0x01, 0, 0, 0, 6, 0x01, 0x04, 0x32, 0xE2, 0, 2}; // 13027-13028 U32 0.1kWh

p156_dataStructKPL p156_myDataSG[P156_NR_OUTPUT_OPTIONS_MODEL1] = {
    /* 0  */ p156_dataStructKPL(0, 0, p156_sg_reqfree, 0),               // unused
    /* 1  */ p156_dataStructKPL(2, 1, p156_sg_reqRunningState, 0),       // U16
    /* 2  */ p156_dataStructKPL(4, 1, p156_sg_reqTotalDCPower, 0),       // U32 W
    /* 3  */ p156_dataStructKPL(2, 1, p156_sg_reqBattPower, 0),          // U16 W
    /* 4  */ p156_dataStructKPL(4, 2, p156_sg_reqLoadPower, 0),          // S32 W
    /* 5  */ p156_dataStructKPL(4, 2, p156_sg_reqExportPower, 0),        // S32 W
    /* 6  */ p156_dataStructKPL(2, 1, p156_sg_reqDailyPVGen, 0),         // U16 0.1kWh
    /* 7  */ p156_dataStructKPL(4, 1, p156_sg_reqTotalPVGen, 0),         // U32 0.1kWh
    /* 8  */ p156_dataStructKPL(2, 1, p156_sg_reqDailyImport, 0),        // U16 0.1kWh
    /* 9  */ p156_dataStructKPL(4, 1, p156_sg_reqTotalImport, 0),        // U32 0.1kWh
    /* 10 */ p156_dataStructKPL(4, 2, p156_sg_reqTotalACPower, 0),       // S32 W
    /* 11 */ p156_dataStructKPL(2, 1, p156_sg_reqBattCurrent, 0),        // U16 0.1A
    /* 12 */ p156_dataStructKPL(2, 1, p156_sg_reqBattSOC, 0),            // U16 0.1%
    /* 13 */ p156_dataStructKPL(2, 3, p156_sg_reqBattTemp, 0),           // S16 0.1degC
    /* 14 */ p156_dataStructKPL(2, 1, p156_sg_reqBattVoltage, 0),        // U16 0.1V
    /* 15 */ p156_dataStructKPL(4, 1, p156_sg_reqTotalOutput, 0),        // U32 0.1kWh
    /* 16 */ p156_dataStructKPL(2, 1, p156_sg_reqDailyOutput, 0),        // U16 0.1kWh
    /* 17 */ p156_dataStructKPL(2, 1, p156_sg_reqDailyBattDischarge, 0), // U16 0.1kWh
    /* 18 */ p156_dataStructKPL(4, 1, p156_sg_reqTotalBattDischarge, 0), // U32 0.1kWh
};

// ============================================================
// Aktiv-Zeiger – werden in PLUGIN_INIT gesetzt
// ============================================================
p156_dataStructKPL *p156_activeData = p156_myData;
int p156_activePort = 1502;

// Zustandsvariablen
boolean p156_MyInit = false;
uint8_t p156_step = 10; // direkt bei Query 1 starten
uint16_t p156_send_count = 0;
uint16_t p156_send_errorcount = 0;
uint16_t p156_reconnectcount = 0;
String p156_IP = "";
int p156_outputOptionsAct;
uint32_t p156_last_send = 0; // millis()-Startzeitpunkt der letzten Pause

// ============================================================
// Plugin-Hauptfunktion
// ============================================================
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
    const uint8_t model = P156_MODEL;
    for (uint8_t i = 0; i < VARS_PER_TASK; ++i)
    {
      if (i < P156_NR_OUTPUT_VALUES)
      {
        uint8_t choice = PCONFIG(i + P156_QUERY1_CONFIG_POS);
        safe_strncpy(
            ExtraTaskSettings.TaskDeviceValueNames[i],
            p156_getQueryValueString(choice, model),
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
    P156_QUERY1 = P156_QUERY1_DFLT;
    P156_QUERY2 = P156_QUERY2_DFLT;
    P156_QUERY3 = P156_QUERY3_DFLT;
    P156_QUERY4 = P156_QUERY4_DFLT;
    P156_PAUSE_MS = P156_PAUSE_MS_DFLT;
    success = true;
    break;
  }

  case PLUGIN_WEBFORM_LOAD:
  {
    // IP-Adresse anzeigen
    char ipString[IP_BUFF_SIZE_P156] = "";
    String msgStr;
    addFormSubHeader("");
    addFormHeader(F("Default Settings"));
    String strings[1];
    LoadCustomTaskSettings(event->TaskIndex, strings, 1, CUSTOMTASK_STR_SIZE_P156);
    safe_strncpy(ipString, strings[0], IP_BUFF_SIZE_P156);

    String log1 = F("Inverter: WebLoad=");
    log1 += event->TaskIndex;
    log1 += F(" IP=");
    log1 += strings[0];
    addLogMove(LOG_LEVEL_INFO, log1);

    addFormTextBox(F("IPv4 Address"), getPluginCustomArgName(0), ipString, IP_ADDR_SIZE_P156);
    msgStr = F("Typical Installations use IP Address ");
    msgStr += F(IP_STR_DEF_P156);
    addFormNote(msgStr);

    // Modell-Auswahl
    {
      const __FlashStringHelper *options_model[] = {
          F("KPL (Kostal Plenticore)"),
          F("SH (Sungrow Hybrid)"),
      };
      constexpr size_t nrOptions = NR_ELEMENTS(options_model);
      FormSelectorOptions selector(nrOptions, options_model);
      selector.reloadonchange = true;
      selector.addFormSelector(F("Model Type"), P156_MODEL_LABEL, P156_MODEL);
    }

    // Wert-Auswahl je nach Modell
    {
      const uint8_t model = PCONFIG(0);
      uint8_t outputOptions = (model == 1)
                                  ? P156_NR_OUTPUT_OPTIONS_MODEL1
                                  : P156_NR_OUTPUT_OPTIONS_MODEL0;

      const __FlashStringHelper *options[outputOptions];
      for (int i = 0; i < outputOptions; ++i)
        options[i] = p156_getQueryString(i, model);

      for (uint8_t i = 0; i < P156_NR_OUTPUT_VALUES; ++i)
      {
        const uint8_t pconfigIndex = i + P156_QUERY1_CONFIG_POS;
        sensorTypeHelper_loadOutputSelector(event, pconfigIndex, i, outputOptions, options);
      }
    }

    // Pause-Zeit
    addFormNumericBox(F("Pause between queries (ms)"),
                      P156_PAUSE_MS_LABEL,
                      P156_PAUSE_MS,
                      P156_PAUSE_MS_MIN,
                      P156_PAUSE_MS_MAX);
    addFormNote(F("Wartezeit zwischen zwei Modbus-Anfragen (10 - 5000 ms)"));

    success = true;
    break;
  }

  case PLUGIN_WEBFORM_SAVE:
  {
    char ipString[IP_BUFF_SIZE_P156] = {0};
    char deviceTemplate[1][CUSTOMTASK_STR_SIZE_P156];
    String errorStr, msgStr;
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
      addLogMove(LOG_LEVEL_INFO, msgStr);
    }
    else if (!p156_validateIp(ipString))
    {
      msgStr = F("WARNING, Please Review IP Address. ");
      errorStr += msgStr;
      addLogMove(LOG_LEVEL_INFO, msgStr);
    }
    else
    {
      p156_IP = ipString;
    }

    safe_strncpy(deviceTemplate[0], ipString, IP_BUFF_SIZE_P156);
    SaveCustomTaskSettings(event->TaskIndex,
                           reinterpret_cast<const uint8_t *>(&deviceTemplate),
                           sizeof(deviceTemplate));

    P156_MODEL = getFormItemInt(P156_MODEL_LABEL);
    const uint8_t model = P156_MODEL;
    for (uint8_t i = 0; i < P156_NR_OUTPUT_VALUES; ++i)
    {
      const uint8_t pconfigIndex = i + P156_QUERY1_CONFIG_POS;
      const uint8_t choice = PCONFIG(pconfigIndex);
      sensorTypeHelper_saveOutputSelector(event, pconfigIndex, i,
                                          p156_getQueryValueString(choice, model));
    }
    P156_PAUSE_MS = getFormItemInt(P156_PAUSE_MS_LABEL);

    p156_MyInit = false; // Force device setup next time
    success = true;
    break;
  }

  case PLUGIN_INIT:
  {
    p156_client.stop();
    p156_deleteValues(P156_MODEL);

    if (P156_MODEL == 1)
    {
      p156_activeData = p156_myDataSG;
      p156_activePort = 502;
      p156_outputOptionsAct = P156_NR_OUTPUT_OPTIONS_MODEL1;
    }
    else
    {
      p156_activeData = p156_myData;
      p156_activePort = 1502;
      p156_outputOptionsAct = P156_NR_OUTPUT_OPTIONS_MODEL0;
    }

    char ipString[IP_BUFF_SIZE_P156] = "";
    String strings[1];
    LoadCustomTaskSettings(event->TaskIndex, strings, 1, CUSTOMTASK_STR_SIZE_P156);
    safe_strncpy(ipString, strings[0], IP_BUFF_SIZE_P156);
    p156_IP = ipString;

    p156_step = 10; // direkt bei Query 1 starten
    p156_send_count = 0;
    p156_send_errorcount = 0;
    p156_reconnectcount = 0;
    p156_last_send = millis(); // Startzeitpunkt setzen
    p156_MyInit = true;
    success = true;

    String log5 = F("Inverter: Init=");
    log5 += event->TaskIndex;
    log5 += F(" Model=");
    log5 += P156_MODEL;
    log5 += F(" Port=");
    log5 += p156_activePort;
    log5 += F(" Pause=");
    log5 += P156_PAUSE_MS;
    log5 += F("ms IP=");
    log5 += p156_IP;
    addLogMove(LOG_LEVEL_INFO, log5);
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
      UserVar.setFloat(event->TaskIndex, 0, p156_readVal(P156_QUERY1, model));
      UserVar.setFloat(event->TaskIndex, 1, p156_readVal(P156_QUERY2, model));
      UserVar.setFloat(event->TaskIndex, 2, p156_readVal(P156_QUERY3, model));
      UserVar.setFloat(event->TaskIndex, 3, p156_readVal(P156_QUERY4, model));
      success = true;
    }
    break;
  }

  case PLUGIN_TEN_PER_SECOND:
  {
    if (!p156_MyInit)
    {
      success = true;
      break;
    }

    if ((millis() - p156_last_send) < (uint32_t)P156_PAUSE_MS)
    {
      success = true;
      break;
    }

    int lquery = 0;
    switch (p156_step)
    {
    case 10:
      if (P156_QUERY1 != 0)
        lquery = P156_QUERY1;
      p156_step = 20;
      break;
    case 20:
      if (P156_QUERY2 != 0)
        lquery = P156_QUERY2;
      p156_step = 30;
      break;
    case 30:
      if (P156_QUERY3 != 0)
        lquery = P156_QUERY3;
      p156_step = 40;
      break;
    case 40:
      if (P156_QUERY4 != 0)
        lquery = P156_QUERY4;
      p156_step = 10;
      break;
    default:
      p156_step = 10;
      break;
    }

    if (lquery != 0)
    {
      boolean ok = p156_sendRequest(lquery);
      if (ok)
        ok = p156_parseValues(lquery);

      if (!ok)
      {
        p156_send_errorcount++;
        if (p156_send_errorcount > 10)
        {
          p156_send_errorcount = 0;
          p156_client.clear();
          p156_client.stop();
          p156_deleteValues(P156_MODEL);
          p156_reconnectcount++;
        }
      }
      else
      {
        p156_send_errorcount = 0; // Fehler bei Erfolg zuruecksetzen
      }

      p156_last_send = millis(); // Pausenstart merken
    }

    if (loglevelActiveFor(LOG_LEVEL_DEBUG) || p156_step == 10)
    {
      String logSend = F("Inverter: ");
      if (loglevelActiveFor(LOG_LEVEL_DEBUG))
      {
        logSend += F("SendErr=");
        logSend += p156_send_errorcount;
        logSend += F(" Pause=");
        logSend += P156_PAUSE_MS;
        logSend += F("ms ");
      }
      logSend += F("ReConn=");
      logSend += p156_reconnectcount;
      addLogMove(LOG_LEVEL_INFO, logSend);
    }
    success = true;
    break;
  }
  } // switch
  return success;
}

// ============================================================
// Hilfsfunktionen
// ============================================================

float p156_readVal(uint8_t query, unsigned int model)
{
  if (model == 1)
    return p156_myDataSG[query].value;
  return p156_myData[query].value; // model == 0
}

unsigned int p156_getRegister(uint8_t query, uint8_t model)
{
  if (model == 0)
  {
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

const __FlashStringHelper *p156_getQueryString(uint8_t query, uint8_t model)
{
  if (model == 1) // Sungrow
  {
    switch (query)
    {
    case 1:
      return F("SG_RUNNING_STATE");
    case 2:
      return F("SG_TOTAL_DC_POWER");
    case 3:
      return F("SG_BATTERY_POWER");
    case 4:
      return F("SG_LOAD_POWER");
    case 5:
      return F("SG_EXPORT_POWER");
    case 6:
      return F("SG_DAILY_PV_GEN");
    case 7:
      return F("SG_TOTAL_PV_GEN");
    case 8:
      return F("SG_DAILY_IMPORT");
    case 9:
      return F("SG_TOTAL_IMPORT");
    case 10:
      return F("SG_TOTAL_AC_POWER");
    case 11:
      return F("SG_BATT_CURRENT");
    case 12:
      return F("SG_BATT_SOC");
    case 13:
      return F("SG_BATT_TEMP");
    case 14:
      return F("SG_BATT_VOLTAGE");
    case 15:
      return F("SG_TOTAL_OUTPUT");
    case 16:
      return F("SG_DAILY_OUTPUT");
    case 17:
      return F("SG_DAILY_BATT_DISCH");
    case 18:
      return F("SG_TOTAL_BATT_DISCH");
    }
    return F("");
  }
  // model == 0: Kostal KPL
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

const __FlashStringHelper *p156_getQueryValueString(uint8_t query, uint8_t model)
{
  if (model == 1) // Sungrow
  {
    switch (query)
    {
    case 1:
      return F("SG_RunningState");
    case 2:
      return F("SG_TotalDCPower");
    case 3:
      return F("SG_BattPower");
    case 4:
      return F("SG_LoadPower");
    case 5:
      return F("SG_ExportPower");
    case 6:
      return F("SG_DailyPVGen");
    case 7:
      return F("SG_TotalPVGen");
    case 8:
      return F("SG_DailyImport");
    case 9:
      return F("SG_TotalImport");
    case 10:
      return F("SG_TotalACPower");
    case 11:
      return F("SG_BattCurrent");
    case 12:
      return F("SG_BattSOC");
    case 13:
      return F("SG_BattTemp");
    case 14:
      return F("SG_BattVoltage");
    case 15:
      return F("SG_TotalOutput");
    case 16:
      return F("SG_DailyOutput");
    case 17:
      return F("SG_DailyBattDisch");
    case 18:
      return F("SG_TotalBattDisch");
    }
    return F("");
  }
  // model == 0: Kostal KPL
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
{
  IPAddress ip;
  unsigned int length = ipStr.length();
  if ((length < IP_MIN_SIZE_P156) || (length > IP_ADDR_SIZE_P156))
    return false;
  if (ip.fromString(ipStr) == false)
    return false;
  return true;
}

bool p156_sendRequest(uint8_t query)
{
  if (!NetworkConnected(0))
    return false;
  char *lIP = &p156_IP[0];
  String log = F("Inverter: Sendrequest ");
  log += p156_IP;
  log += '|';
  log += query;
  addLogMove(LOG_LEVEL_DEBUG, log);

  if (!p156_client.connected())
  {
    if (!p156_client.connect(lIP, p156_activePort, 1000))
    {
      addLog(LOG_LEVEL_INFO, F("Inverter: SendRequest; connection failed"));
      return 0;
    }
  }

  int lLen = sizeof(p156_activeData[query].dataRequest);
  if (lLen > 0)
  {
    if (p156_send_count <= 0 || p156_send_count > 65535)
      p156_send_count = 1;
    else
      p156_send_count++;

    uint8_t HBy = (uint8_t)(p156_send_count >> 8);
    uint8_t LBy = (uint8_t)(p156_send_count);
    p156_activeData[query].dataRequest[0] = HBy;
    p156_activeData[query].dataRequest[1] = LBy;

    byte *lPointer = &p156_activeData[query].dataRequest[0];
    p156_client.write(lPointer, lLen);

    if (loglevelActiveFor(LOG_LEVEL_DEBUG))
    {
      log += '(';
      log += p156_send_count;
      log += ')';
      for (int i = 0; i < lLen; i++)
      {
        log += '|';
        log += p156_activeData[query].dataRequest[i];
      }
      addLogMove(LOG_LEVEL_DEBUG, log);
    }
    return 1;
  }
  log += F(" Daten zu kurz");
  addLogMove(LOG_LEVEL_INFO, log);
  return 0;
}

unsigned int p156_parseValues(uint8_t query)
{
  String log = F("Inverter: parseValues ");
  uint8_t high1 = 0, low1 = 0, high2 = 0, low2 = 0;

  unsigned long timeout = millis();
  while (p156_client.available() < 11)
  {
    delay(1);
    if (millis() - timeout > 2000)
    {
      p156_client.clear();
      return 0;
    }
  }

  int bytesToReceive = p156_client.available();
  log += '(';
  log += query;
  log += ',';
  log += bytesToReceive;
  log += ')';

  uint8_t llen = p156_activeData[query].lenValue;
  byte b = 0;

  // Header-Bytes lesen; letzten zwei vor Nutzdaten = Transaction-ID
  for (int a = 0; a < bytesToReceive - llen; a++)
  {
    b = p156_client.read();
    if (a == bytesToReceive - llen - 9)
      high1 = b;
    if (a == bytesToReceive - llen - 8)
      low1 = b;
    log += '|';
    log += b;
  }

  uint16_t lreceivedId = ((high1 << 8) + low1);
  log += '(';
  log += p156_send_count;
  log += '=';
  log += lreceivedId;
  log += ')';

  if (p156_send_count != lreceivedId)
  {
    p156_client.clear();
    addLogMove(LOG_LEVEL_INFO, log);
    return 0;
  }

  high1 = 0;
  low1 = 0;
  high2 = 0;
  low2 = 0;
  if (llen > 0)
    high1 = p156_client.read();
  if (llen > 1)
    low1 = p156_client.read();
  if (llen > 2)
    high2 = p156_client.read();
  if (llen > 3)
    low2 = p156_client.read();

  float lValue = 0.0f;
  switch (p156_activeData[query].datatyp)
  {
  case 1: // U32 / U16 unsigned
    lValue = (float)(uint32_t)((high2 << 24) | (low2 << 16) | (high1 << 8) | low1);
    break;
  case 2: // S32 signed
    lValue = (float)(int32_t)((high2 << 24) | (low2 << 16) | (high1 << 8) | low1);
    break;
  case 3: // S16 signed
    lValue = (float)(int16_t)((high1 << 8) | low1);
    break;
  default: // 0: IEEE754 float (Kostal)
  {
    unsigned char pBuffer[] = {low2, high2, low1, high1};
    memcpy(&lValue, pBuffer, sizeof(float));
    break;
  }
  }

  p156_activeData[query].value = lValue;
  log += '[';
  log += lValue;
  log += ']';
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
  if (model == 1)
  {
    for (int i = 0; i < P156_NR_OUTPUT_OPTIONS_MODEL1; i++)
      p156_myDataSG[i].value = 0;
  }
  else
  {
    for (int i = 0; i < P156_NR_OUTPUT_OPTIONS_MODEL0; i++)
      p156_myData[i].value = 0;
  }
}

#endif // USES_P156
