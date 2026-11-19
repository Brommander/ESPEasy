#include "_Plugin_Helper.h"

#ifdef USES_P155

// #######################################################################################################
// ######################## Plugin 155: Energy - Smartmeter ########################
// #######################################################################################################
//
// Models:
//   P155_MODEL_D0  = D0  (IEC 62056-21, 7E1, 9600 Baud)
//   P155_MODEL_SML = SML (dynamic TL-parsing, works with all SML-compliant meters)
//
// SML operation:
//   After the OBIS code, each SML-ListEntry contains these fields,
//   each preceded by a TL-byte (Type-Length):
//     Status | Time | Unit | Scaler (int8) | Value
//   TL-byte: Bits 7-4 = Type (5=int signed, 6=uint, 7=list, 0=optional)
//            Bits 3-0 = Total length incl. TL-byte (0 = field absent)
//            For lists (Type 7): Bits 3-0 = number of child elements (direct)
//   The parser reads these fields dynamically - no hardcoded byte offsets.
//
// Data integrity:
//   - Every task keeps its own state (several meters on one ESP are possible).
//   - SML: values of a telegram are only taken over when the CRC16 (X.25) at the end of the
//     telegram is valid. Incomplete telegrams (lost bytes) are discarded.
//     Meters that never send a valid CRC: after 10 telegrams without any valid CRC the check
//     is switched off for this meter (logged), the plausibility check stays active.
//   - Energy counters (SML 1.8.0 / 2.8.0, D0 1.8.0): a new value must not be lower than the last
//     one and must not rise faster than 100 kW. Other values are rejected. A new level is only
//     taken over when it is confirmed by 10 consecutive plausible readings (e.g. meter exchange).
//   - After start, values are only sent to the controllers after every selected value was received.
//
// #######################################################################################################

# define PLUGIN_155
# define PLUGIN_ID_155 155
# define PLUGIN_NAME_155 "SML - Smartmeter"

# define P155_MODEL PCONFIG(0)
# define P155_MODEL_LABEL PCONFIG_LABEL(0)
# define P155_QUERY1 PCONFIG(1)
# define P155_QUERY2 PCONFIG(2)
# define P155_QUERY3 PCONFIG(3)
# define P155_QUERY4 PCONFIG(4)

// Model index defines
# define P155_MODEL_D0 0
# define P155_MODEL_SML 1

# define P155_MODEL_DFLT P155_MODEL_SML
# define P155_BAUDRATE 9600
# define P155_QUERY1_DFLT 1
# define P155_QUERY2_DFLT 3
# define P155_QUERY3_DFLT 2
# define P155_QUERY4_DFLT 0

# define P155_NR_OUTPUT_VALUES 4
# define P155_NR_OUTPUT_OPTIONS_D0 6
# define P155_NR_OUTPUT_OPTIONS_SML 13
# define P155_NR_OPTIONS_MAX P155_NR_OUTPUT_OPTIONS_SML
# define P155_QUERY1_CONFIG_POS 1
# define P155_RX_BUFFER 64 // value bytes of one SML entry / one D0 line

// Receive buffer of the serial port. An SML telegram has several hundred bytes; with a small
// buffer bytes get lost when the ESP is busy for a moment.
# ifdef ESP32
#  define P155_SERIAL_BUFFER 1024
# else // ifdef ESP32
#  define P155_SERIAL_BUFFER 256
# endif // ifdef ESP32

# define P155_START_TIMEOUT      120000 // send anyway after 2 min if a selected value never arrives
# define P155_TELEGRAM_MAX       4096   // longer "telegram" = no end found, discard
# define P155_CRC_LEARN          10     // telegrams without any valid CRC before the check is switched off
# define P155_COUNTER_MAX_W      100000.0f // fastest plausible rise of an energy counter
# define P155_COUNTER_CONFIRM    10     // readings that confirm a new counter level

// SML register indices (1-based, 0 = placeholder)
# define DD3_TOTAL_ACTIVE_ENERGY_PLUS 1
# define DD3_TOTAL_ACTIVE_ENERGY_MINUS 6
// D0 register index
# define Q3D_TOTAL_ACTIVE_ENERGY 1

# include <ESPeasySerial.h>

const __FlashStringHelper* p155_getQueryString(uint8_t query,
                                               uint8_t model);
const __FlashStringHelper* p155_getQueryValueString(uint8_t query,
                                                    uint8_t model);

// OBIS codes SML
static const uint8_t p155_obisSML[P155_NR_OUTPUT_OPTIONS_SML][6] PROGMEM = {
  { 0, 0, 0,  0, 0, 0   }, // [0] placeholder
  { 1, 0, 1,  8, 0, 255 }, // [1] 1-0:1.8.0   Total import
  { 1, 0, 21, 7, 0, 255 }, // [2] 1-0:21.7.0  Power L1
  { 1, 0, 41, 7, 0, 255 }, // [3] 1-0:41.7.0  Power L2
  { 1, 0, 61, 7, 0, 255 }, // [4] 1-0:61.7.0  Power L3
  { 1, 0, 16, 7, 0, 255 }, // [5] 1-0:16.7.0  Total power
  { 1, 0, 2,  8, 0, 255 }, // [6] 1-0:2.8.0   Total export
  { 1, 0, 32, 7, 0, 255 }, // [7] 1-0:32.7.0  Voltage L1
  { 1, 0, 52, 7, 0, 255 }, // [8] 1-0:52.7.0  Voltage L2
  { 1, 0, 72, 7, 0, 255 }, // [9] 1-0:72.7.0  Voltage L3
  { 1, 0, 31, 7, 0, 255 }, // [10] 1-0:31.7.0 Current L1
  { 1, 0, 51, 7, 0, 255 }, // [11] 1-0:51.7.0 Current L2
  { 1, 0, 71, 7, 0, 255 }, // [12] 1-0:71.7.0 Current L3
};

// OBIS IDs D0
static const char *const p155_idD0[P155_NR_OUTPUT_OPTIONS_D0] = {
  "x-x:x.x.x*x",    // [0] placeholder
  "1-0:1.8.0*255",  // [1] Total active energy import
  "1-0:21.7.0*255", // [2] Power L1
  "1-0:41.7.0*255", // [3] Power L2
  "1-0:61.7.0*255", // [4] Power L3
  "1-0:1.7.0*255",  // [5] Power L123
};

// SML start sequence (ring buffer order: newest byte first)
// Stream: 1B 1B 1B 1B 01 01 01 01  →  ring buffer reversed: 01 01 01 01 1B 1B 1B 1B
static constexpr uint8_t P155_SML_startseq[] = { 0x01, 0x01, 0x01, 0x01, 0x1B, 0x1B, 0x1B, 0x1B };

// ============================================================
// Per-task data
// ============================================================
struct P155_data_struct : public PluginTaskData_base
{
  explicit P155_data_struct(uint8_t model) : model(model)
  {
    nrOptions = (model == P155_MODEL_SML) ? P155_NR_OUTPUT_OPTIONS_SML : P155_NR_OUTPUT_OPTIONS_D0;
    initTime  = millis();
  }

  ~P155_data_struct()
  {
    if (serial != nullptr) {
      delete serial;
      serial = nullptr;
    }
  }

  bool begin(ESPEasySerialPort port, int16_t rx, int16_t tx)
  {
    serial = new (std::nothrow) ESPeasySerial(port, rx, tx, false, P155_SERIAL_BUFFER);

    if (serial == nullptr) { return false; }
    serial->begin(P155_BAUDRATE, (model == P155_MODEL_D0) ? SERIAL_7E1 : SERIAL_8N1);
    return true;
  }

  void  loop();
  bool  hasValue(uint8_t query) const;
  bool  hasAnyValue(uint8_t q1, uint8_t q2, uint8_t q3, uint8_t q4) const;
  float readVal(uint8_t query) const;
  void  showStatus() const;

  // ---- data ----
  ESPeasySerial *serial = nullptr;
  uint8_t        model;
  uint8_t        nrOptions;
  uint32_t       initTime;

  float value[P155_NR_OPTIONS_MAX]{}; // last accepted values
  bool  got[P155_NR_OPTIONS_MAX]{};   // value received since start

  // energy counter plausibility
  struct Counter {
    bool     hasBase   = false;
    float    base      = 0.0f;
    uint32_t baseTime  = 0;
    uint8_t  candCount = 0;
    float    cand      = 0.0f;
    uint32_t candTime  = 0;
  };
  Counter counter[P155_NR_OPTIONS_MAX];

  // statistics (shown in the task settings)
  uint32_t telegramsOk     = 0;
  uint32_t telegramsCrcErr = 0;
  uint32_t telegramsLost   = 0; // started, but no end found
  uint32_t rejected        = 0; // implausible counter values

private:

  void handleSML(uint8_t b);
  void handleD0(char c);
  void parseValueSML();
  void parseValueD0();
  void telegramStart();
  void telegramEnd(uint8_t crcHi, uint8_t crcLo);
  void crcFeed(uint8_t b);
  void accept(uint8_t reg, float v);
  bool isCounter(uint8_t reg) const;
  bool plausible(float from, uint32_t fromTime, float to, uint32_t now) const;
  bool checkCounter(uint8_t reg, float v);
  bool readUint32(int startByte, int numBytes, uint32_t& result) const;
  bool readInt32(int startByte, int numBytes, int32_t& result) const;

  // receive buffer for one value / one D0 line
  char    rxBuffer[P155_RX_BUFFER]{};
  uint8_t charsRead = 0;

  // D0
  String rxID;

  // SML parser state
  uint8_t ringBuffer[8]{};
  uint8_t rxObis[6]{};
  uint8_t step         = 0;
  int     anzBytes     = 0;
  int     registerAct  = 0;
  float   scaleFactor  = 1.0f; // 10^scaler
  uint8_t autoSubState = 0;    // 0=Status,1=Time,2=Unit,3=Scaler,4=Value
  uint8_t skipBytes    = 0;
  uint8_t autoDataTyp  = 0;    // 5=signed int, 6=uint
  uint8_t listElems    = 0;

  // SML telegram / CRC
  bool     inTelegram = false;
  uint16_t crc        = 0xFFFF;
  uint16_t telLen     = 0;
  uint8_t  escCount   = 0;
  uint8_t  cmdIdx     = 0;
  uint8_t  cmd0       = 0;
  uint8_t  crcHi      = 0;
  uint32_t crcOkTotal = 0;
  uint8_t  crcErrInRow = 0;
  bool     crcOff      = false;
  float    pending[P155_NR_OPTIONS_MAX]{};
  bool     pendingSet[P155_NR_OPTIONS_MAX]{};
};

// ============================================================
// Plugin main function
// ============================================================
boolean Plugin_155(uint8_t function, struct EventStruct *event, String& string)
{
  boolean success = false;

  switch (function)
  {
    case PLUGIN_DEVICE_ADD:
    {
      auto& dev = Device[++deviceCount];
      dev.Number             = PLUGIN_ID_155;
      dev.Type               = DEVICE_TYPE_SERIAL; // enables serial port selector in UI
      dev.VType              = Sensor_VType::SENSOR_TYPE_QUAD;
      dev.Ports              = 0;
      dev.PullUpOption       = false;
      dev.InverseLogicOption = false;
      dev.FormulaOption      = true;
      dev.ValueCount         = P155_NR_OUTPUT_VALUES;
      dev.SendDataOption     = true;
      dev.TimerOption        = true;
      dev.GlobalSyncOption   = true;
      break;
    }

    case PLUGIN_GET_DEVICENAME:
    {
      string = F(PLUGIN_NAME_155);
      break;
    }

    case PLUGIN_GET_DEVICEVALUENAMES:
    {
      const uint8_t model = P155_MODEL;

      for (uint8_t i = 0; i < VARS_PER_TASK; ++i)
      {
        if (i < P155_NR_OUTPUT_VALUES)
        {
          uint8_t choice = PCONFIG(i + P155_QUERY1_CONFIG_POS);
          safe_strncpy(ExtraTaskSettings.TaskDeviceValueNames[i],
                       p155_getQueryValueString(choice, model),
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
      P155_MODEL  = P155_MODEL_DFLT;
      P155_QUERY1 = P155_QUERY1_DFLT;
      P155_QUERY2 = P155_QUERY2_DFLT;
      P155_QUERY3 = P155_QUERY3_DFLT;
      P155_QUERY4 = P155_QUERY4_DFLT;
      success     = true;
      break;
    }

    case PLUGIN_WEBFORM_LOAD:
    {
      {
        const __FlashStringHelper *options_model[] = {
          F("D0"),
          F("SML"),
        };
        FormSelectorOptions selector(NR_ELEMENTS(options_model), options_model);
        selector.reloadonchange = true;
        selector.addFormSelector(F("Model Type"), P155_MODEL_LABEL, P155_MODEL);
      }
      {
        const uint8_t model         = PCONFIG(0);
        const uint8_t outputOptions = (model == P155_MODEL_SML)
                                        ? P155_NR_OUTPUT_OPTIONS_SML
                                        : P155_NR_OUTPUT_OPTIONS_D0;

        const __FlashStringHelper *options[outputOptions];

        for (int i = 0; i < outputOptions; ++i) {
          options[i] = p155_getQueryString(i, model);
        }

        for (uint8_t i = 0; i < P155_NR_OUTPUT_VALUES; ++i) {
          sensorTypeHelper_loadOutputSelector(event, i + P155_QUERY1_CONFIG_POS,
                                              i, outputOptions, options);
        }
      }
      {
        P155_data_struct *P155_data = static_cast<P155_data_struct *>(getPluginTaskData(event->TaskIndex));

        if (P155_data != nullptr) {
          P155_data->showStatus();
        }
      }
      success = true;
      break;
    }

    case PLUGIN_WEBFORM_SAVE:
    {
      P155_MODEL = getFormItemInt(P155_MODEL_LABEL);
      const uint8_t model = P155_MODEL;

      for (uint8_t i = 0; i < P155_NR_OUTPUT_VALUES; ++i)
      {
        const uint8_t pconfigIndex = i + P155_QUERY1_CONFIG_POS;
        const uint8_t choice       = PCONFIG(pconfigIndex);
        sensorTypeHelper_saveOutputSelector(event, pconfigIndex, i,
                                            p155_getQueryValueString(choice, model));
      }
      success = true;
      break;
    }

    case PLUGIN_INIT:
    {
      const int16_t serial_rx      = CONFIG_PIN1;
      const int16_t serial_tx      = CONFIG_PIN2;
      const ESPEasySerialPort port = static_cast<ESPEasySerialPort>(CONFIG_PORT);

      initPluginTaskData(event->TaskIndex, new (std::nothrow) P155_data_struct(P155_MODEL));
      P155_data_struct *P155_data = static_cast<P155_data_struct *>(getPluginTaskData(event->TaskIndex));

      if ((P155_data == nullptr) || !P155_data->begin(port, serial_rx, serial_tx)) {
        clearPluginTaskData(event->TaskIndex);
        break;
      }

      success = true;

      if (loglevelActiveFor(LOG_LEVEL_INFO))
      {
        addLogMove(LOG_LEVEL_INFO, strformat(F("Smartmeter: Init=%d Model=%d Port=%d RX=%d TX=%d"),
                                             event->TaskIndex, P155_MODEL, static_cast<int>(port), serial_rx, serial_tx));
      }
      break;
    }

    case PLUGIN_EXIT:
    {
      // task data (and serial port) are freed by the core
      success = true;
      break;
    }

    case PLUGIN_READ:
    {
      P155_data_struct *P155_data = static_cast<P155_data_struct *>(getPluginTaskData(event->TaskIndex));

      if (P155_data == nullptr) {
        break;
      }

      const bool all = P155_data->hasValue(P155_QUERY1) && P155_data->hasValue(P155_QUERY2) &&
                       P155_data->hasValue(P155_QUERY3) && P155_data->hasValue(P155_QUERY4);

      if (!all) {
        // Start-up: only send when every selected value was received once, so counters never go out as 0.
        // After 2 min send anyway (a selected value the meter does not deliver), but only if the meter
        // delivered anything at all.
        if ((timePassedSince(P155_data->initTime) < P155_START_TIMEOUT) ||
            !P155_data->hasAnyValue(P155_QUERY1, P155_QUERY2, P155_QUERY3, P155_QUERY4)) {
          break;
        }
      }
      UserVar.setFloat(event->TaskIndex, 0, P155_data->readVal(P155_QUERY1));
      UserVar.setFloat(event->TaskIndex, 1, P155_data->readVal(P155_QUERY2));
      UserVar.setFloat(event->TaskIndex, 2, P155_data->readVal(P155_QUERY3));
      UserVar.setFloat(event->TaskIndex, 3, P155_data->readVal(P155_QUERY4));
      success = true;
      break;
    }

    case PLUGIN_TEN_PER_SECOND:
    {
      P155_data_struct *P155_data = static_cast<P155_data_struct *>(getPluginTaskData(event->TaskIndex));

      if (P155_data != nullptr) {
        P155_data->loop();
      }
      success = true;
      break;
    }
  } // switch
  return success;
}

// ============================================================
// Selector texts and value names
// ============================================================
const __FlashStringHelper* p155_getQueryString(uint8_t query, uint8_t model)
{
  if (model == P155_MODEL_D0)
  {
    switch (query)
    {
      case 1:
        return F("Total Active Energy (kWh)");
      case 2:
        return F("Power L1 (W)");
      case 3:
        return F("Power L2 (W)");
      case 4:
        return F("Power L3 (W)");
      case 5:
        return F("Power Total (W)");
    }
  }
  else if (model == P155_MODEL_SML)
  {
    switch (query)
    {
      case 1:
        return F("Total Active Energy Plus (Wh)");
      case 2:
        return F("Power L1 (W)");
      case 3:
        return F("Power L2 (W)");
      case 4:
        return F("Power L3 (W)");
      case 5:
        return F("Power Total (W)");
      case 6:
        return F("Total Active Energy Minus (Wh)");
      case 7:
        return F("Voltage L1 (V)");
      case 8:
        return F("Voltage L2 (V)");
      case 9:
        return F("Voltage L3 (V)");
      case 10:
        return F("Current L1 (A)");
      case 11:
        return F("Current L2 (A)");
      case 12:
        return F("Current L3 (A)");
    }
  }
  return F("");
}
const __FlashStringHelper* p155_getQueryValueString(uint8_t query, uint8_t model)
{
  if (model == P155_MODEL_D0)
  {
    switch (query)
    {
      case 1:
        return F("Consumption_kWh");
      case 2:
        return F("L1_W");
      case 3:
        return F("L2_W");
      case 4:
        return F("L3_W");
      case 5:
        return F("L123_W");
    }
  }
  else if (model == P155_MODEL_SML)
  {
    switch (query)
    {
      case 1:
        return F("Energy_Consumption_kWh");
      case 2:
        return F("L1_W");
      case 3:
        return F("L2_W");
      case 4:
        return F("L3_W");
      case 5:
        return F("L123_W");
      case 6:
        return F("Energy_FeedIn_kWh");
      case 7:
        return F("L1_V");
      case 8:
        return F("L2_V");
      case 9:
        return F("L3_V");
      case 10:
        return F("L1_A");
      case 11:
        return F("L2_A");
      case 12:
        return F("L3_A");
    }
  }
  return F("");
}

// ============================================================
// Values
// ============================================================
bool P155_data_struct::hasValue(uint8_t query) const
{
  if (query == 0) { return true; } // value not selected
  if (query >= nrOptions) { return true; }
  return got[query];
}

bool P155_data_struct::hasAnyValue(uint8_t q1, uint8_t q2, uint8_t q3, uint8_t q4) const
{
  const uint8_t q[] = { q1, q2, q3, q4 };

  for (uint8_t i = 0; i < 4; ++i) {
    if ((q[i] != 0) && (q[i] < nrOptions) && got[q[i]]) { return true; }
  }
  return false;
}

float P155_data_struct::readVal(uint8_t query) const
{
  if ((query == 0) || (query >= nrOptions)) { return 0.0f; }
  return value[query];
}

void P155_data_struct::showStatus() const
{
  addFormSubHeader(F("Status"));

  if (model == P155_MODEL_SML) {
    addRowLabel(F("Telegrams OK"));
    addHtmlInt(telegramsOk);
    addRowLabel(F("Telegrams with CRC error"));
    addHtmlInt(telegramsCrcErr);
    addRowLabel(F("Incomplete telegrams"));
    addHtmlInt(telegramsLost);
    addRowLabel(F("CRC check"));
    addHtml(crcOff ? F("off (meter sends no valid CRC)") : F("on"));
  }
  addRowLabel(F("Rejected counter values"));
  addHtmlInt(rejected);
}

// ============================================================
// Energy counter plausibility
// ============================================================
bool P155_data_struct::isCounter(uint8_t reg) const
{
  if (model == P155_MODEL_SML) {
    return (reg == DD3_TOTAL_ACTIVE_ENERGY_PLUS) || (reg == DD3_TOTAL_ACTIVE_ENERGY_MINUS);
  }
  return reg == Q3D_TOTAL_ACTIVE_ENERGY;
}

// A counter never decreases and does not rise faster than P155_COUNTER_MAX_W.
// SML counters are in Wh, D0 counters in kWh.
bool P155_data_struct::plausible(float from, uint32_t fromTime, float to, uint32_t now) const
{
  const float perHour = (model == P155_MODEL_SML) ? P155_COUNTER_MAX_W : (P155_COUNTER_MAX_W / 1000.0f);
  const float tol     = (model == P155_MODEL_SML) ? 10.0f : 0.01f; // float resolution / meter rounding
  const float hours   = (now - fromTime) / 3600000.0f;
  const float delta   = to - from;

  return (delta >= -tol) && (delta <= (perHour * hours) + tol);
}

// Returns true when the value may be taken over.
bool P155_data_struct::checkCounter(uint8_t reg, float v)
{
  Counter& c         = counter[reg];
  const uint32_t now = millis();

  if (c.hasBase && plausible(c.base, c.baseTime, v, now)) {
    c.base      = v;
    c.baseTime  = now;
    c.candCount = 0;
    return true;
  }

  // Not plausible (or no level yet): collect readings for a new level.
  if ((c.candCount > 0) && plausible(c.cand, c.candTime, v, now)) {
    ++c.candCount;
  } else {
    c.candCount = 1;
  }
  c.cand     = v;
  c.candTime = now;

  // After start one confirming reading is enough, a jump needs P155_COUNTER_CONFIRM readings.
  const uint8_t needed = c.hasBase ? P155_COUNTER_CONFIRM : 2;

  if (c.candCount >= needed) {
    if (c.hasBase && loglevelActiveFor(LOG_LEVEL_ERROR)) {
      addLogMove(LOG_LEVEL_ERROR, strformat(F("Smartmeter: counter %d jumps from %.3f to %.3f (confirmed, e.g. meter exchange)"),
                                            reg, c.base, v));
    }
    c.hasBase   = true;
    c.base      = v;
    c.baseTime  = now;
    c.candCount = 0;
    return true;
  }

  if (c.hasBase) {
    ++rejected;

    if (loglevelActiveFor(LOG_LEVEL_ERROR)) {
      addLogMove(LOG_LEVEL_ERROR, strformat(F("Smartmeter: implausible counter %d value %.3f rejected (last %.3f)"),
                                            reg, v, c.base));
    }
  }
  return false;
}

void P155_data_struct::accept(uint8_t reg, float v)
{
  if ((reg == 0) || (reg >= nrOptions)) { return; }

  if (isCounter(reg) && !checkCounter(reg, v)) { return; }
  value[reg] = v;
  got[reg]   = true;
}

// ============================================================
// Serial input
// ============================================================
void P155_data_struct::loop()
{
  if (serial == nullptr) { return; }

  const uint32_t start = millis();
  size_t available     = serial->available();

  while (available && timePassedSince(start) < 10)
  {
    const int r = serial->read();
    --available;

    if (available == 0) {
      available = serial->available();
    }

    if (r < 0) { continue; }

    if (model == P155_MODEL_SML) {
      handleSML(static_cast<uint8_t>(r));
    } else {
      handleD0(static_cast<char>(r));
    }
  }
}

// ============================================================
// SML: telegram frame and CRC16 (X.25)
//   Telegram: 1B1B1B1B 01010101 <data> 1B1B1B1B 1A <fill> <crc> <crc>
//   CRC over all bytes from the start sequence up to and including the fill byte.
//   Escape sequences come in 4-byte groups: 1B1B1B1B followed by a 4-byte command
//   (1B1B1B1B = escaped data, 1A xx yy zz = end, 01010101 = start).
// ============================================================
void P155_data_struct::crcFeed(uint8_t b)
{
  crc ^= b;

  for (uint8_t i = 0; i < 8; ++i) {
    crc = (crc & 1) ? ((crc >> 1) ^ 0x8408) : (crc >> 1);
  }
}

void P155_data_struct::telegramStart()
{
  if (inTelegram) { ++telegramsLost; } // previous telegram had no end (bytes lost)

  inTelegram = true;
  crc        = 0xFFFF;
  telLen     = 8;
  escCount   = 0;
  cmdIdx     = 0;

  for (int i = 7; i >= 0; --i) { crcFeed(ringBuffer[i]); }

  for (uint8_t i = 0; i < P155_NR_OPTIONS_MAX; ++i) { pendingSet[i] = false; }
}

void P155_data_struct::telegramEnd(uint8_t hi, uint8_t lo)
{
  inTelegram = false;
  const uint16_t calc     = crc ^ 0xFFFF;
  const uint16_t received = (static_cast<uint16_t>(hi) << 8) | lo;
  const uint16_t swapped  = (static_cast<uint16_t>(lo) << 8) | hi;
  const bool     ok       = (received == calc) || (swapped == calc);

  if (ok) {
    ++telegramsOk;
    ++crcOkTotal;
    crcErrInRow = 0;

    if (crcOff) {
      crcOff = false;
      addLog(LOG_LEVEL_INFO, F("Smartmeter: valid CRC received, CRC check on"));
    }
  } else {
    ++telegramsCrcErr;

    if (crcErrInRow < 255) { ++crcErrInRow; }

    if (!crcOff && (crcOkTotal == 0) && (crcErrInRow >= P155_CRC_LEARN)) {
      crcOff = true;
      addLog(LOG_LEVEL_ERROR, F("Smartmeter: meter sends no valid CRC, CRC check off (plausibility check stays on)"));
    } else if (!crcOff && loglevelActiveFor(LOG_LEVEL_INFO)) {
      addLogMove(LOG_LEVEL_INFO, strformat(F("Smartmeter: CRC error, telegram discarded (%04X / %04X)"), received, calc));
    }
  }

  if (ok || crcOff) {
    for (uint8_t i = 1; i < nrOptions; ++i) {
      if (pendingSet[i]) { accept(i, pending[i]); }
    }
  }

  for (uint8_t i = 0; i < P155_NR_OPTIONS_MAX; ++i) { pendingSet[i] = false; }
}

void P155_data_struct::handleSML(uint8_t b)
{
  // Ring buffer for SML start-sequence detection: 1B 1B 1B 1B 01 01 01 01
  for (int i = 7; i > 0; i--) {
    ringBuffer[i] = ringBuffer[i - 1];
  }
  ringBuffer[0] = b;

  // ---- frame / CRC ----
  if (inTelegram) {
    if (++telLen > P155_TELEGRAM_MAX) {
      inTelegram = false;
      ++telegramsLost;
    } else if (cmdIdx > 0) {
      // byte cmdIdx (1..4) of an escape command
      if (cmdIdx == 1) {
        cmd0 = b;
        crcFeed(b);
      } else if (cmd0 == 0x1A) {
        if (cmdIdx == 2) { crcFeed(b); }      // fill byte count
        else if (cmdIdx == 3) { crcHi = b; } // CRC byte 1
        else { telegramEnd(crcHi, b); }       // CRC byte 2
      } else {
        crcFeed(b);
      }

      if (++cmdIdx > 4) { cmdIdx = 0; }
    } else {
      crcFeed(b);

      if (b == 0x1B) {
        if (++escCount == 4) {
          escCount = 0;
          cmdIdx   = 1;
        }
      } else {
        escCount = 0;
      }
    }
  }

  if (memcmp(ringBuffer, P155_SML_startseq, NR_ELEMENTS(P155_SML_startseq)) == 0)
  {
    telegramStart();
    step         = 11;
    charsRead    = 0;
    registerAct  = 0;
    anzBytes     = 0;
    scaleFactor  = 1.0f;
    autoSubState = 0;
    skipBytes    = 0;
    listElems    = 0;
    return;
  }

  // ---- value parser ----
  switch (step)
  {
    // -------------------------------------------------------
    // States 11-13: find OBIS code
    // -------------------------------------------------------
    case 11: // find ListEntry tag 0x77

      if (b == 0x77)
      {
        step      = 12;
        charsRead = 0;
      }
      break;

    case 12: // length byte must be 0x07 (6 OBIS bytes + TL)
      step = (b == 0x07) ? 13 : 11;
      break;

    case 13: // collect 6 OBIS bytes

      if (charsRead < 6) {
        rxObis[charsRead++] = b;
      }

      if (charsRead >= 6)
      {
        charsRead   = 0;
        registerAct = 0;

        // search OBIS number in table
        for (int i = 1; i < nrOptions; i++)
        {
          if (memcmp_P(rxObis, p155_obisSML[i], 6) == 0)
          {
            registerAct = i;
            break;
          }
        }

        if (registerAct != 0)
        {
          scaleFactor  = 1.0f;
          autoSubState = 0;
          skipBytes    = 0;
          listElems    = 0;
          step         = 30;
          charsRead    = 0;
        }
        else
        {
          step = 11; // not found, keep searching
        }
      }
      break;

    // -------------------------------------------------------
    // States 30-33: dynamic TL parsing
    //
    // TL-byte format:
    //   Bits 7-4: Type (0=optional/absent, 5=signed int, 6=uint, 7=list)
    //   Bits 3-0: Total length incl. TL-byte
    //     Primitive: dataBytes = bits3-0 - 1
    //     List (Type 7): child count = bits3-0  (direct, NOT length-1)
    //
    // autoSubState: 0=Status, 1=Time, 2=Unit, 3=Scaler, 4=Value
    // -------------------------------------------------------
    case 30: // read TL-byte of current field
    {
      const uint8_t tlLen     = (b & 0x0F);
      const uint8_t tlTyp     = (b >> 4) & 0x07;
      const uint8_t dataBytes = (tlLen > 0) ? (tlLen - 1) : 0;

      if (autoSubState < 3)
      {
        // Status (0), Time (1), Unit (2)
        if (tlTyp == 7)
        {
          // List type: bits3-0 = direct child count (NOT byte length)
          if (tlLen == 0)
          {
            autoSubState++;
          }
          else
          {
            listElems = tlLen;
            step      = 36;
          }
        }
        else if (dataBytes == 0)
        {
          autoSubState++; // field not present → continue
        }
        else
        {
          skipBytes = dataBytes;
          step      = 31; // skip bytes
        }
      }
      else if (autoSubState == 3)
      {
        // Scaler field
        if (dataBytes == 0)
        {
          scaleFactor = 1.0f; // no scaler present
          autoSubState++;
        }
        else
        {
          step = 32; // read 1 scaler byte
        }
      }
      else if (autoSubState == 4)
      {
        // Value field
        if (dataBytes == 0)
        {
          step = 11; // no value - abort
        }
        else
        {
          autoDataTyp = tlTyp;
          anzBytes    = dataBytes;
          charsRead   = 0;
          step        = 33;
        }
      }
      break;
    }

    case 31: // skip bytes (primitive Status/Time/Unit)
      skipBytes--;

      if (skipBytes == 0)
      {
        autoSubState++;
        step = 30; // read next TL-Byte
      }
      break;

    case 32: // read scaler byte (int8, signed)
    {
      const int8_t scalerRaw = static_cast<int8_t>(b);
      scaleFactor = (scalerRaw != 0) ? powf(10.0f, static_cast<float>(scalerRaw)) : 1.0f;
      autoSubState++;
      step = 30; // continue with Value-TL
      break;
    }

    case 33: // collect value bytes

      if (charsRead < P155_RX_BUFFER - 1) {
        rxBuffer[charsRead++] = b;
      }

      if (charsRead >= anzBytes)
      {
        parseValueSML();
        step = 11;
      }
      break;

    // -------------------------------------------------------
    // States 36-37: skip list type (type=7) child elements
    // Required for SML_Time with timestamp, e.g.:
    //   72 62 01 65 xx xx xx xx  (list with 2 child elements)
    // -------------------------------------------------------
    case 36: // read TL-byte of a list child element
    {
      const uint8_t cLen  = (b & 0x0F);
      const uint8_t cTyp  = (b >> 4) & 0x07;
      const uint8_t cData = (cLen > 0) ? (cLen - 1) : 0;

      if (cTyp == 7)
      {
        // Nested list: replace current element with its children
        listElems = listElems - 1 + cLen;
      }
      else
      {
        listElems--;

        if (cData > 0)
        {
          skipBytes = cData;
          step      = 37; // skip data, then back to 36 or 30
          break;
        }
      }

      if (listElems == 0)
      {
        autoSubState++;
        step = 30;
      }
      break; // otherwise 36
    }

    case 37: // skip data bytes of a list child element
      skipBytes--;

      if (skipBytes == 0)
      {
        if (listElems == 0)
        {
          autoSubState++;
          step = 30;
        }
        else
        {
          step = 36; // next child element
        }
      }
      break;

    default:
      step         = 0;
      charsRead    = 0;
      anzBytes     = 0;
      registerAct  = 0;
      autoSubState = 0;
      listElems    = 0;
      break;
  }
}

// ============================================================
// SML: value of one list entry (kept until the CRC of the telegram is checked)
// ============================================================
void P155_data_struct::parseValueSML()
{
  if ((registerAct <= 0) || (registerAct >= nrOptions)) {
    return;
  }

  const int startByte = (anzBytes > 4) ? (anzBytes - 4) : 0;
  const int readBytes = (anzBytes > 4) ? 4 : anzBytes;

  float lvalue = 0.0f;

  if (autoDataTyp == 5) // signed int
  {
    int32_t raw = 0;

    if (!readInt32(startByte, readBytes, raw)) {
      return;
    }
    lvalue = raw;
  }
  else // unsigned (type 6) or unknown
  {
    uint32_t raw = 0;

    if (!readUint32(startByte, readBytes, raw)) {
      return;
    }
    lvalue = raw;
  }

  pending[registerAct]    = lvalue * scaleFactor;
  pendingSet[registerAct] = true;

# ifndef BUILD_NO_DEBUG

  if (loglevelActiveFor(LOG_LEVEL_DEBUG))
  {
    addLogMove(LOG_LEVEL_DEBUG, strformat(F("SML: reg=%d typ=%d bytes=%d factor=%f val=%f"),
                                          registerAct, autoDataTyp, anzBytes, scaleFactor, pending[registerAct]));
  }
# endif // ifndef BUILD_NO_DEBUG
}

// ============================================================
// D0
// ============================================================
void P155_data_struct::handleD0(char c)
{
  // check bounds
  if (charsRead >= P155_RX_BUFFER - 1)
  {
    charsRead = 0;
  }

  if (c == '(')
  {
    rxBuffer[charsRead] = '\0';
    rxID                = String(rxBuffer);
    charsRead           = 0;
  }
  else if (c == ')')
  {
    rxBuffer[charsRead] = '\0';

    if (charsRead > 1) {
      parseValueD0();
    }
    charsRead = 0;
  }
  else if ((c == 0x0D) || (c == 0x0A))
  {
    charsRead = 0;
  }
  else
  {
    rxBuffer[charsRead++] = c;
  }
}

void P155_data_struct::parseValueD0()
{
  int i = 1;

  for (; i < nrOptions; i++)
  {
    if (rxID.equals(p155_idD0[i]))
    {
      accept(i, String(rxBuffer).toFloat());
      break;
    }
  }

# ifndef BUILD_NO_DEBUG

  if (loglevelActiveFor(LOG_LEVEL_DEBUG))
  {
    if (i < nrOptions) {
      addLogMove(LOG_LEVEL_DEBUG, strformat(F("D0 Parse: ID=%s reg=%d val=%f"), rxID.c_str(), i, value[i]));
    } else {
      addLogMove(LOG_LEVEL_DEBUG, strformat(F("D0 Parse: ID=%s (not found)"), rxID.c_str()));
    }
  }
# endif // ifndef BUILD_NO_DEBUG
}

// ============================================================
// Buffer read helpers (bounds-checked)
// ============================================================

// Read numBytes (1-4) big-endian from rxBuffer at startByte → uint32_t
bool P155_data_struct::readUint32(int startByte, int numBytes, uint32_t& result) const
{
  if ((numBytes < 1) || (numBytes > 4)) {
    return false;
  }

  if ((startByte + numBytes) > charsRead) {
    return false;
  }

  if ((startByte + numBytes) > P155_RX_BUFFER) {
    return false;
  }
  result = 0;

  for (int i = 0; i < numBytes; i++) {
    result = (result << 8) | static_cast<uint8_t>(rxBuffer[startByte + i]);
  }
  return true;
}

// Sign extension: shift the data to the top, then arithmetic shift back
bool P155_data_struct::readInt32(int startByte, int numBytes, int32_t& result) const
{
  uint32_t raw = 0;

  if (!readUint32(startByte, numBytes, raw)) {
    return false;
  }
  const int shift = (4 - numBytes) * 8;
  result = static_cast<int32_t>(raw << shift) >> shift;
  return true;
}

#endif // USES_P155
