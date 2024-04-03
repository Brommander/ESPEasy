#include "_Plugin_Helper.h"
#ifdef USES_P158

//#######################################################################################################
//######################## Plugin 158: I²C Analog Out ########################
//#######################################################################################################

#define DEF_I2C_ADDRESS_158 0xB0

#include "Wire.h"

#define PLUGIN_158
#define PLUGIN_ID_158 158
#define PLUGIN_NAME_158 "I²C Analog Out [Testing]"

#define P158_I2C_ADDR PCONFIG(0)
#define P158_SENSOR_TYPE_INDEX  1
#define P158_NR_OUTPUT_VALUES  getValueCountFromSensorType(static_cast<Sensor_VType>(PCONFIG(P158_SENSOR_TYPE_INDEX)))

#define CMD_NAME "I2CAnalogOutCMD"

boolean P158_SetValue(uint8_t address, uint8_t channel, uint16_t value);
void P158_deleteValues(struct EventStruct *event);

// Variables
boolean P158_MyInit = false;
int16_t P158_ValueCh0 = -1;
int16_t P158_ValueCh1 = -1;
int16_t P158_ValueCh2 = -1;
int16_t P158_ValueCh3 = -1;
// ************************************************************************************************

boolean Plugin_158(uint8_t function, struct EventStruct *event, String &string)
{
  boolean success = false;

  switch (function)
  {
  case PLUGIN_DEVICE_ADD:
  {
    Device[++deviceCount].Number = PLUGIN_ID_158;
    Device[deviceCount].Type = DEVICE_TYPE_I2C;
    Device[deviceCount].VType = Sensor_VType::SENSOR_TYPE_SINGLE;
    Device[deviceCount].Ports = 0;
    Device[deviceCount].PullUpOption = false;
    Device[deviceCount].InverseLogicOption = false;
    Device[deviceCount].FormulaOption = true;
    Device[deviceCount].ValueCount = 1;
    Device[deviceCount].SendDataOption = true;
    Device[deviceCount].TimerOption = true;
    Device[deviceCount].GlobalSyncOption   = true;
    Device[deviceCount].OutputDataType = Output_Data_type_t::Simple;
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
        String name = F("AnalogOut");
        if (i < P158_NR_OUTPUT_VALUES) {
          name += String(i+1),
          safe_strncpy(ExtraTaskSettings.TaskDeviceValueNames[i],name,sizeof(ExtraTaskSettings.TaskDeviceValueNames[i]));
          ExtraTaskSettings.TaskDeviceValueDecimals[i] = 2;
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
    for (uint8_t i = 0; i < VARS_PER_TASK; ++i) {
      ExtraTaskSettings.TaskDeviceValueDecimals[i] = 2;
    }
    P158_I2C_ADDR = DEF_I2C_ADDRESS_158;
    break;
  }

  case PLUGIN_WEBFORM_SHOW_I2C_PARAMS:
  {
    if ((P158_I2C_ADDR != 0xB0) || (P158_I2C_ADDR != 0xB2) || (P158_I2C_ADDR != 0xB4) || (P158_I2C_ADDR != 0xB6) || (P158_I2C_ADDR != 0xB8) || (P158_I2C_ADDR != 0xBA) || (P158_I2C_ADDR != 0xBC) || (P158_I2C_ADDR != 0xBE) ||
        (P158_I2C_ADDR != 0xD0) || (P158_I2C_ADDR != 0xD2) || (P158_I2C_ADDR != 0xD4) || (P158_I2C_ADDR != 0xD6) || (P158_I2C_ADDR != 0xD8) || (P158_I2C_ADDR != 0xDA) || (P158_I2C_ADDR != 0xDC) || (P158_I2C_ADDR != 0xDE))
    { // Validate I2C Addr.
      ;//P158_I2C_ADDR = DEF_I2C_ADDRESS_158;
    }
    String i2c_addres_string = formatToHex(P158_I2C_ADDR);
    addFormTextBox(F("I2C Address (Hex)"), F("i2c_addr"), i2c_addres_string, 4);
    break;
  }

  case PLUGIN_WEBFORM_LOAD:
  {
    success = true;
    break;
  }

  case PLUGIN_WEBFORM_SAVE:
  {
    String i2c_address = webArg(F("i2c_addr"));
    P158_I2C_ADDR = (int)strtol(i2c_address.c_str(), 0, 16);

    success = true;
    break;
  }

  case PLUGIN_INIT:
  {
    // Enable I2C
    // Wire.begin(); // This is already done by ESP Easy Core

    P158_deleteValues(event);
    P158_MyInit = true;
    success = true;
    break;
  }

  case PLUGIN_EXIT:
  {    
    P158_MyInit = false;
    P158_deleteValues(event);    
    break;
  }

  case PLUGIN_READ:
  {
    if (P158_MyInit)
    {      
      success = true;
      break;
    }
    break;
  }

  case PLUGIN_WRITE:
  {
    String tmpString = string;
    boolean result = false;
    String log;

    String cmd = parseString(tmpString, 1);
    String mycmd = F(CMD_NAME);
    mycmd += String(event->TaskIndex + 1);
    
    if (cmd.equalsIgnoreCase(mycmd) || cmd.equalsIgnoreCase(F("TaskValueSet")))
    //if (cmd.equalsIgnoreCase(F(CMD_NAME)) || cmd.equalsIgnoreCase(F("TaskValueSet")))
    {
      log += F("I2CAnalogOut");
      String paramChannel = parseString(tmpString, 2); // Output 1,2,3,4
      String paramValue = parseString(tmpString, 3);   // 0 - 1023

      uint8_t channel = paramChannel.toInt();
      uint16_t value = paramValue.toInt();

      if (value < 0 || value > 1023)
      {
        log += F("ErrorValue");
      }
      else if (channel < 0 || channel > 3)
      {
        log += F("ErrorChannel");
      }
      else // Wenn CMD Ok
      {
        result = P158_SetValue(P158_I2C_ADDR, channel, value);
      }
      if (result)
      { // Nur wenn Uebertragung keinen Fehler hat Wert eintragen
        // Wert in interne Variable schreiben
        switch (channel)
        {
        case 0:
          P158_ValueCh0 = value;
          UserVar.setFloat(event->BaseVarIndex,0,P158_ValueCh0);
          break;
        case 1:
          P158_ValueCh0 = value;
          UserVar.setFloat(event->BaseVarIndex,1,P158_ValueCh1);
          break;
        case 2:
          P158_ValueCh0 = value;
          UserVar.setFloat(event->BaseVarIndex,2,P158_ValueCh2);
          break;
        case 3:
          P158_ValueCh0 = value;
          UserVar.setFloat(event->BaseVarIndex,3,P158_ValueCh3);
          break;
        }
      }
      else
      {
        log += F(" ResetValues ");
        // Alle Werte auf -1 weil Gerät Fehler
        P158_deleteValues(event);
      }
      if (loglevelActiveFor(LOG_LEVEL_INFO))
      {
        log += F(": Addr=");
        log += formatToHex(P158_I2C_ADDR);
        log += F(": Kanal=");
        log += paramChannel;
        log += F(", Wert=");
        log += paramValue;
        addLogMove(LOG_LEVEL_INFO, log);
      }
      success = true; 
    }
  }
  break;
  }
  return success;
}

boolean P158_SetValue(uint8_t address, uint8_t channel, uint16_t value)
{
  boolean result = false;
  String log = F("I2CAnalogOut Error: ");
  int16_t status = 0;
  //uint8_t HBy = value / 256;       // HIGH-Byte berechnen
  //uint8_t LBy = value - HBy * 256; // LOW-Byte berechnen
  uint8_t HBy =  (uint8_t) (value >> 8);
  uint8_t LBy = (uint8_t) (value);
  
  Wire.beginTransmission(address); // Start Übertragung zur ANALOG-OUT Karte
  Wire.write(channel);             // Kanal schreiben
  Wire.write(LBy);                 // LOW-Byte schreiben
  Wire.write(HBy);                 // HIGH-Byte schreiben
  status = Wire.endTransmission(); // Ende
  log += F(" LowByte ");
  log += LBy;
  log += F(" HighByte ");
  log += HBy;
  switch (status)
  {
  case 0: // Transfer OK
    log += F("found Device No ");
    result = true;
    break;
  case 1:
    log += F("Too many Datas");
    break;
  case 2:
    log += F("No Device found!");
    break;
  case 3:
    log += F("Unknow Format!");
    break;
  default:
    log += F("Unknow Error!");
    break;
  }
 // if (!result)
 // {
    addLogMove(LOG_LEVEL_INFO, log);
 // }
  return result;
}
void P158_deleteValues(struct EventStruct *event)
{
  UserVar.setFloat(event->BaseVarIndex,0,-1);
  UserVar.setFloat(event->BaseVarIndex,1,-1);
  UserVar.setFloat(event->BaseVarIndex,2,-1);
  UserVar.setFloat(event->BaseVarIndex,3,-1);

  for (size_t i = 0; i < P158_NR_OUTPUT_VALUES; i++)
  {
    P158_SetValue(P158_I2C_ADDR, 0, 0);
    delay(20);
  } 

}
#endif // USES_P158

/*
if (result != "true") {
  String ErrorStr = F("ErrorSend: ");
  ErrorStr += result;
  SendStatus(event, ErrorStr + F(" <br>")); // Reply (echo) to sender. This will print message on browser.
  log += ErrorStr;)
  }
            */
