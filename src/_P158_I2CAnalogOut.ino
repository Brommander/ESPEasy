#include "_Plugin_Helper.h"
#ifdef USES_P158

// #######################################################################################################
// ######################## Plugin 158: I²C Analog Out ########################
// #######################################################################################################
//
// 4-Kanal-Analogausgang (10 Bit, z. B. 0-10 V) am I2C-Bus.
// Protokoll je Schreibvorgang: <Kanal 0-3> <Low-Byte> <High-Byte>, Wert 0-1023.
//
// Befehl: I2CAnalogOutCMD<Tasknummer>,<Kanal 0-3>,<Wert 0-1023>
//   Taskwert i (1-4) zeigt den zuletzt erfolgreich geschriebenen Wert von Kanal i-1,
//   -1 nach dem Start bzw. nach einem Übertragungsfehler.
//   Start/Ende des Tasks: alle 4 Kanäle werden auf 0 gesetzt.
//
// PCONFIG:
//   [0] I2C-Adresse (7 Bit)
//   [1] Ausgabetyp (Anzahl Taskwerte)
// #######################################################################################################

#include "Wire.h"

#define PLUGIN_158
#define PLUGIN_ID_158 158
#define PLUGIN_NAME_158 "I²C Analog Out [Testing]"

#define P158_DEF_I2C_ADDR       0x58 // 7-Bit-Adresse (= 0xB0 in 8-Bit-Schreibweise)
#define P158_I2C_ADDR           PCONFIG(0)
#define P158_SENSOR_TYPE_INDEX  1
#define P158_NR_OUTPUT_VALUES   getValueCountFromSensorType(static_cast<Sensor_VType>(PCONFIG(P158_SENSOR_TYPE_INDEX)))
#define P158_NR_CHANNELS        4
#define P158_MAX_VALUE          1023

#define P158_CMD_NAME "I2CAnalogOutCMD"

static uint8_t p158_writeChannel(uint8_t address, uint8_t channel, uint16_t value);
static void    p158_resetOutputs(struct EventStruct *event);

boolean Plugin_158(uint8_t function, struct EventStruct *event, String& string)
{
  boolean success = false;

  switch (function)
  {
    case PLUGIN_DEVICE_ADD:
    {
      Device[++deviceCount].Number           = PLUGIN_ID_158;
      Device[deviceCount].Type               = DEVICE_TYPE_I2C;
      Device[deviceCount].VType              = Sensor_VType::SENSOR_TYPE_SINGLE;
      Device[deviceCount].Ports              = 0;
      Device[deviceCount].PullUpOption       = false;
      Device[deviceCount].InverseLogicOption = false;
      Device[deviceCount].FormulaOption      = true;
      Device[deviceCount].ValueCount         = 1;
      Device[deviceCount].SendDataOption     = true;
      Device[deviceCount].TimerOption        = true;
      Device[deviceCount].GlobalSyncOption   = true;
      Device[deviceCount].OutputDataType     = Output_Data_type_t::Simple;
      break;
    }

    case PLUGIN_GET_DEVICENAME:
    {
      string = F(PLUGIN_NAME_158);
      break;
    }

    case PLUGIN_GET_DEVICEVALUENAMES:
    {
      for (uint8_t i = 0; i < VARS_PER_TASK; ++i) {
        if (i < P158_NR_OUTPUT_VALUES) {
          const String name = concat(F("AnalogOut"), i + 1);
          safe_strncpy(ExtraTaskSettings.TaskDeviceValueNames[i], name, sizeof(ExtraTaskSettings.TaskDeviceValueNames[i]));
          ExtraTaskSettings.TaskDeviceValueDecimals[i] = 0;
        } else {
          ZERO_FILL(ExtraTaskSettings.TaskDeviceValueNames[i]);
        }
      }
      break;
    }

    case PLUGIN_GET_DEVICEVALUECOUNT:
    {
      event->Par1 = P158_NR_OUTPUT_VALUES;
      success     = true;
      break;
    }

    case PLUGIN_GET_DEVICEVTYPE:
    {
      event->sensorType = static_cast<Sensor_VType>(PCONFIG(P158_SENSOR_TYPE_INDEX));
      event->idx        = P158_SENSOR_TYPE_INDEX;
      success           = true;
      break;
    }

    case PLUGIN_SET_DEFAULTS:
    {
      PCONFIG(P158_SENSOR_TYPE_INDEX) = static_cast<uint8_t>(Sensor_VType::SENSOR_TYPE_SINGLE);
      P158_I2C_ADDR                   = P158_DEF_I2C_ADDR;
      break;
    }

    case PLUGIN_WEBFORM_SHOW_I2C_PARAMS:
    {
      addFormTextBox(F("I2C Address (Hex)"), F("i2c_addr"), formatToHex(P158_I2C_ADDR), 4);
      addFormNote(F("7-bit address 0x08..0x77, e.g. 0x58 (= 0xB0 in 8-bit notation)"));
      break;
    }

    case PLUGIN_WEBFORM_LOAD:
    {
      success = true;
      break;
    }

    case PLUGIN_WEBFORM_SAVE:
    {
      long addr = strtol(webArg(F("i2c_addr")).c_str(), nullptr, 16);

      // 8-Bit-Schreibweise (z. B. 0xB0) in die 7-Bit-Adresse umrechnen
      if ((addr > 0x77) && (addr <= 0xFF)) {
        addr >>= 1;
      }

      if ((addr < 0x08) || (addr > 0x77)) {
        addr = P158_DEF_I2C_ADDR;
      }
      P158_I2C_ADDR = addr;
      success       = true;
      break;
    }

    case PLUGIN_INIT:
    {
      p158_resetOutputs(event);
      success = true;
      break;
    }

    case PLUGIN_EXIT:
    {
      p158_resetOutputs(event);
      break;
    }

    case PLUGIN_READ:
    {
      success = true;
      break;
    }

    case PLUGIN_WRITE:
    {
      const String cmd = parseString(string, 1);

      if (!cmd.equalsIgnoreCase(concat(F(P158_CMD_NAME), event->TaskIndex + 1))) {
        break;
      }
      success = true; // Befehl gehört zu diesem Task

      // Werte aus Rules kommen mit Nachkommastellen (z. B. "500.00"): als Zahl lesen, Nachkommastellen abschneiden
      float   fChannel = -1.0f;
      float   fValue   = -1.0f;
      const bool parsed = validFloatFromString(parseString(string, 2), fChannel) &&
                          validFloatFromString(parseString(string, 3), fValue);
      const int32_t channel = static_cast<int32_t>(fChannel);
      const int32_t value   = static_cast<int32_t>(fValue);

      if (!parsed || (fChannel < 0) || (channel >= P158_NR_CHANNELS) ||
          (fValue < 0) || (value > P158_MAX_VALUE)) {
        if (loglevelActiveFor(LOG_LEVEL_ERROR)) {
          addLogMove(LOG_LEVEL_ERROR, concat(F("I2CAnalogOut: invalid command: "), string));
        }
        break;
      }

      const uint8_t status = p158_writeChannel(P158_I2C_ADDR, channel, value);

      if (status == 0) {
        if (channel < P158_NR_OUTPUT_VALUES) {
          UserVar.setFloat(event->TaskIndex, channel, value);
        }
      } else {
        // Modul hat nicht quittiert: Werte ungültig
        for (uint8_t i = 0; i < P158_NR_CHANNELS; ++i) {
          UserVar.setFloat(event->TaskIndex, i, -1);
        }

        if (loglevelActiveFor(LOG_LEVEL_ERROR)) {
          addLogMove(LOG_LEVEL_ERROR, strformat(F("I2CAnalogOut: write to %s failed (I2C error %d)"),
                                                formatToHex(P158_I2C_ADDR).c_str(), status));
        }
      }
      break;
    }
  }
  return success;
}

// Rückgabe: 0 = OK, sonst Fehlercode von Wire.endTransmission()
static uint8_t p158_writeChannel(uint8_t address, uint8_t channel, uint16_t value)
{
  Wire.beginTransmission(address);
  Wire.write(channel);                       // Kanal
  Wire.write(static_cast<uint8_t>(value));      // Low-Byte
  Wire.write(static_cast<uint8_t>(value >> 8)); // High-Byte
  return Wire.endTransmission();
}

// Alle Kanäle auf 0, Taskwerte auf -1 (noch kein Wert geschrieben)
static void p158_resetOutputs(struct EventStruct *event)
{
  for (uint8_t i = 0; i < P158_NR_CHANNELS; ++i) {
    UserVar.setFloat(event->TaskIndex, i, -1);

    if (i > 0) { delay(10); } // dem Modul Zeit zwischen zwei Übertragungen lassen
    p158_writeChannel(P158_I2C_ADDR, i, 0);
  }
}

#endif // USES_P158
