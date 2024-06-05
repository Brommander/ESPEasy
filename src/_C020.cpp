#include "src/Helpers/_CPlugin_Helper.h"
#ifdef USES_C020

#include "src/Commands/InternalCommands.h"
#include "src/Globals/EventQueue.h"
#include "src/Globals/ExtraTaskSettings.h"
#include "src/Helpers/PeriodicalActions.h"
#include "src/Helpers/StringParser.h"
#include "_Plugin_Helper.h"

#include <Platform.h>
#include "Settimino.h"

// #######################################################################################################
// ################### Controller Plugin 020: S7 Communication ##############################
// #######################################################################################################

#define CPLUGIN_020
#define CPLUGIN_ID_020 20
#define CPLUGIN_NAME_020 "S7 Communication"

struct C020_ConfigStruct
{
  void zero_last()
  {
    DBNummerStr[4] = 0;
  }
  char DBNummerStr[5] = {0};
};

byte BufferRead[160];  // 4 x 4 x 20
byte BufferWrite[160]; // 4 x 20

uint8_t C020_step = 0;
uint16_t C020_errorcount = 0;
boolean C020_MyInit = false;
IPAddress C020_IP;
int16_t C020_DBNum;
S7Client C020_client;

String CPlugin_020_pubname;
bool CPlugin_020_mqtt_retainFlag = false;

bool C020_parse_command(struct EventStruct *event);
bool C020_connect();
void C020_floatToByte(byte *bytes, float f);
bool CPlugin_020(CPlugin::Function function, struct EventStruct *event, String &string);
bool C020_load_ConfigStruct(controllerIndex_t ControllerIndex, String &DBNummerStr);

bool CPlugin_020(CPlugin::Function function, struct EventStruct *event, String &string)
{
  bool success = false;

  switch (function)
  {
  case CPlugin::Function::CPLUGIN_PROTOCOL_ADD:
  {
    ProtocolStruct& proto = getProtocolStruct(event->idx); //        = CPLUGIN_ID_020;
    proto.usesMQTT = false;
    proto.usesTemplate = false;
    proto.usesAccount = false;
    proto.usesPassword = false;
    proto.usesExtCreds = false;
    proto.defaultPort = 102;
    proto.usesID = false;
    break;
  }

  case CPlugin::Function::CPLUGIN_GET_DEVICENAME:
  {
    string = F(CPLUGIN_NAME_020);
    break;
  }

  case CPlugin::Function::CPLUGIN_INIT:
  {

    MakeControllerSettings(ControllerSettings); //-V522
    if (!AllocatedControllerSettings())
    {
      return false;
    }

    LoadControllerSettings(event->ControllerIndex, *ControllerSettings);
    C020_IP = ControllerSettings->IP;
    String DBNummerStr;
    if (!C020_load_ConfigStruct(event->ControllerIndex, DBNummerStr))
    {
      return false;
    }
    C020_DBNum = DBNummerStr.toInt();

    C020_step = 0;
    C020_errorcount = 0;
    // IP lesen und erst am Ende Init = true
    C020_MyInit = true;
    break;
  }

  case CPlugin::Function::CPLUGIN_EXIT:
  {
    C020_MyInit = false;
    C020_client.Disconnect();
    break;
  }

  case CPlugin::Function::CPLUGIN_PROTOCOL_TEMPLATE:
  {
    event->String1 = F("%sysname%/#");
    event->String2 = F("%sysname%/%tskname%/%valname%");
    break;
  }

  case CPlugin::Function::CPLUGIN_WEBFORM_LOAD:
  {
    String DBNummerStr;
    if (!C020_load_ConfigStruct(event->ControllerIndex, DBNummerStr))
    {
      return false;
    }
    addTableSeparator(F("S7 Einstellungen"), 2, 3);
    addFormTextBox(F("DBNummer"), F("C020DBNummer"), DBNummerStr, 4); // 4->Anzahl Zeichen
    break;
  }

  case CPlugin::Function::CPLUGIN_WEBFORM_SAVE:
  {
    std::shared_ptr<C020_ConfigStruct> customConfig;
    {
      std::shared_ptr<C020_ConfigStruct> tmp_shared(new (std::nothrow) C020_ConfigStruct);
      customConfig = std::move(tmp_shared);
    }

    String DBNummerStr = webArg(F("C020DBNummer"));

    strlcpy(customConfig->DBNummerStr, DBNummerStr.c_str(), sizeof(customConfig->DBNummerStr));
    customConfig->zero_last();
    SaveCustomControllerSettings(event->ControllerIndex, reinterpret_cast<const uint8_t *>(customConfig.get()), sizeof(C020_ConfigStruct));

    break;
  }

  case CPlugin::Function::CPLUGIN_TEN_PER_SECOND:
  {
    String logDB = F("S7: DB:");
    uint16_t Size;
    int Result;
    int DBNum = C020_DBNum;
    // Connection
    if (!C020_client.Connected)
    {
      if (!C020_connect())
      {
        return false;
      }
    }
    else
    {
      switch (C020_step)
      {
      case 10:
        logDB += DBNum;
        logDB += F(" Len:");
        Size = sizeof(BufferWrite);
        logDB += Size;
        logDB += F(" Uploadingresult: ");
        Serial.println(DBNum);                        // Test einen Wert
        Result = C020_client.WriteArea(S7AreaDB,      // We are requesting DB access
                                       DBNum,         // DB Number = 1
                                       0,             // Start from byte N.0
                                       Size,          // We need "Size" bytes
                                       &BufferWrite); // Put them into our target (BufferRead or PDU)
        logDB += Result;
        logDB += F(";");

        if (Result != 0) // Bei Fehler disconnect
        {
          C020_errorcount++;
          C020_client.Disconnect();
        }
        C020_step = 11;
        break;
      case 11:
        C020_step = 12;
        break;
      case 12:
        C020_step = 20;
        break;
      case 20:
        logDB += DBNum;
        logDB += F("; Len:");
        Size = sizeof(BufferRead) / 2; // Lesen in Word
        logDB += Size;
        logDB += F(" Downloadingresult: ");
        Result = C020_client.ReadArea(S7AreaDB,     // We are requesting DB access
                                      DBNum,        // DB Number = 1
                                      320,          // Start from byte N.0
                                      Size,         // We need "Size" Words
                                      &BufferRead); // Put them into our target (BufferRead or PDU)

        logDB += Result;
        logDB += F(";");

        if (Result != 0) // Bei Fehler disconnect
        {
          C020_errorcount++;
          C020_client.Disconnect();
        }
        C020_step = 21;
        break;
      case 21:
        C020_step = 22;
        break;
      case 22:
        C020_step = 10;
        break;
      default:
        C020_step = 10;
        break;
      }
      logDB += F(" Error:");
      logDB += C020_errorcount;
    }
    addLogMove(LOG_LEVEL_DEBUG, logDB);
    break;
  }
  
  case CPlugin::Function::CPLUGIN_PROTOCOL_SEND:
  {
    String pubname = CPlugin_020_pubname;
    bool mqtt_retainFlag = CPlugin_020_mqtt_retainFlag;
    String logSEND = F("S7: Task=");
    logSEND += event->TaskIndex;
    logSEND += F(", Value: ");
    LoadTaskSettings(event->TaskIndex);
    parseControllerVariables(pubname, event, false);

    uint8_t valueCount = getValueCountForTask(event->TaskIndex);

    for (uint8_t x = 0; x < valueCount; x++)
    {
      // MFD: skip publishing for values with empty labels (removes unnecessary publishing of unwanted values)
      if (ExtraTaskSettings.TaskDeviceValueNames[x][0] == 0)
      {
        continue; // we skip values with empty labels
      }
      String tmppubname = pubname;
      parseSingleControllerVariable(tmppubname, event, x, false); // TODO ???
      String value;
      float valuefloat;
      byte pBuffer[4];
      int16_t index;
      if (event->sensorType == Sensor_VType::SENSOR_TYPE_STRING)
      {
        value = event->String2.substring(0, 20); // For the log
      }
      else
      {
        valuefloat = UserVar[event->TaskIndex + x]; // value = formatUserVarNoCheck(event, x);-->String
      }

      logSEND += valuefloat;

      if (event->TaskIndex >= 0 && event->TaskIndex <= 18) // 19) // Tasks 0-19 werden gelesen und an DB gesendet
      {                                                    // event->TaskIndex = ExtraTaskSettings.TaskIndex  ??? TODO
        index = ((event->TaskIndex) * 16) + (x * 4);
        memcpy(&pBuffer[0], &valuefloat, sizeof(float)); // Kopieren des FLoats in das Array

        if (index >= 0 && index < sizeof(BufferWrite) - 3)
        {
          BufferWrite[index + 0] = pBuffer[3]; //(int32_t)valuefloat / 100;
          BufferWrite[index + 1] = pBuffer[2];
          BufferWrite[index + 2] = pBuffer[1];
          BufferWrite[index + 3] = pBuffer[0];
        }
      }
      if (event->TaskIndex >= 19 && event->TaskIndex <= 39) // Tasks 20-39 werden aus DB gelesen und an Task gesendet
      {
        index = ((event->TaskIndex - 19) * 16) + (x * 4);
        if (index >= 0 && index < sizeof(BufferRead) - 3)
        {
          pBuffer[3] = BufferRead[index + 0];
          pBuffer[2] = BufferRead[index + 1];
          pBuffer[1] = BufferRead[index + 2];
          pBuffer[0] = BufferRead[index + 3];
        }
        memcpy(&valuefloat, pBuffer, sizeof(float)); // Kopieren des FLoats in das Array

        UserVar.setFloat(event->TaskIndex,x,valuefloat);

      }
    }
    addLogMove(LOG_LEVEL_DEBUG, logSEND);
    break;
  }

  default:
    break;
  }
  return success;
}
//----------------------------------------------------------------------
// Connects to the PLC
//----------------------------------------------------------------------
bool C020_connect()
{  
  IPAddress PLC(192, 168, 5, 133); // PLC Address
  PLC = C020_IP;
  int Result = C020_client.ConnectTo(PLC,
                                     0,  // Rack (see the doc.)
                                     1); // Slot (see the doc.)
  String logConnect = F("S7: Connect to ");
  logConnect += PLC;
  if (Result == 0)
  {
    logConnect += F("Connected ! PDU Length = ");
    logConnect += C020_client.GetPDULength();
  }
  else
    logConnect += F("Connection error");
  addLogMove(LOG_LEVEL_DEBUG, logConnect);
  return Result == 0;
}

void C020_floatToByte(byte *bytes, float f)
{

  int length = sizeof(float);

  for (int i = 0; i < length; i++)
  {
    bytes[i] = ((byte *)&f)[i];
  }
}
bool C020_load_ConfigStruct(controllerIndex_t ControllerIndex, String &DBNummerStr)
{
  // Just copy the needed strings and destruct the C020_ConfigStruct as soon as possible
  std::shared_ptr<C020_ConfigStruct> customConfig;
  {
    std::shared_ptr<C020_ConfigStruct> tmp_shared(new (std::nothrow) C020_ConfigStruct);
    customConfig = std::move(tmp_shared);
  }

  if (!customConfig)
  {
    return false;
  }
  LoadCustomControllerSettings(ControllerIndex, reinterpret_cast<uint8_t *>(customConfig.get()), sizeof(C020_ConfigStruct));
  customConfig->zero_last();
  DBNummerStr = customConfig->DBNummerStr;
  return true;
}
#endif // ifdef USES_C020