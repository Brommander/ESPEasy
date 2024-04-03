#include "_Plugin_Helper.h"
#ifdef USES_P999

//#######################################################################################################
//######################## Plugin 999: WaterLevel ########################
//#######################################################################################################

#define DEF_I2C_ADDRESS_999 0x18

#include "Wire.h"

// Bibliothek für den Sensor (Im Bibliotheksverwalter unter "MicroPressure" suchen
// oder aus dem GitHub-Repository https://github.com/sparkfun/SparkFun_MicroPressure_Arduino_Library )
#include <SparkFun_MicroPressure.h>

#define PLUGIN_999
#define PLUGIN_ID_999 999
#define PLUGIN_NAME_999 "WaterLevel [Mine]"
#define PLUGIN_VALUENAME1_999  "Start"
#define PLUGIN_VALUENAME2_999  "Height_cm"
#define PLUGIN_VALUENAME3_999  "Volume_l"
#define PLUGIN_VALUENAME4_999  "Level_proz"

#define P999_I2C_ADDR PCONFIG(0)
#define P999_WATER_AREA PCONFIG(1)
#define P999_MAX_WATER_LEVEL PCONFIG(2)
#define P999_PRESS_EMPTY PCONFIG(3)

#define P999_PUMP_PIN1 PCONFIG(4)
#define P999_PUMP_PIN2 PCONFIG(5)
#define P999_VALVE_PIN1 PCONFIG(6)
#define P999_VALVE_PIN2 PCONFIG(7)

#define P999_SENSOR_TYPE_INDEX  1
#define P999_NR_OUTPUT_VALUES 4

#define P999_CMD_NAME "WaterLevelCMD"

// Zuordnung der Ein- Ausgänge

#define P999_AUF  LOW               // Ventil öffnen
#define P999_AUS  LOW               // Pumpe ausschalten
#define P999_ZU   HIGH              // Ventil schliessen
#define P999_EIN  HIGH              // Pumpe einschalten

SparkFun_MicroPressure mpr; 

boolean P999_readPressure(uint8_t address);
void P999_deleteValues(struct EventStruct *event);
void P999_step(struct EventStruct *event);

// Variables
boolean P999_MyInit = false;
int16_t P999_ValueStart = 0;
int16_t P999_ValueHeight = 0;
int16_t P999_ValueVolume = 0;
int16_t P999_ValueLevel = 0;

boolean P999_messungLeer = false;
int P999_atmDruck, P999_atmPressEmpty, P999_messDruckRoh ,P999_messDruck, P999_vergleichswert = 0;
int P999_messSchritt, P999_wassersaeule = 0;
unsigned long P999_messung = 0;
// ************************************************************************************************

boolean Plugin_999(uint8_t function, struct EventStruct *event, String &string)
{
  boolean success = false;

  switch (function)
  {
  case PLUGIN_DEVICE_ADD:
  {
    Device[++deviceCount].Number = PLUGIN_ID_999;
    Device[deviceCount].Type = DEVICE_TYPE_I2C;
    Device[deviceCount].VType = Sensor_VType::SENSOR_TYPE_QUAD;
    Device[deviceCount].Ports = 0;
    Device[deviceCount].PullUpOption = false;
    Device[deviceCount].InverseLogicOption = false;
    Device[deviceCount].FormulaOption = true;
    Device[deviceCount].ValueCount = 4;
    Device[deviceCount].SendDataOption = true;
    Device[deviceCount].TimerOption = true;
    Device[deviceCount].GlobalSyncOption   = true;
    break;
  }
  case PLUGIN_GET_DEVICENAME:
  {
    string = F(PLUGIN_NAME_999);
    break;
  }

  case PLUGIN_GET_DEVICEVALUENAMES:
  {       
    strcpy_P(ExtraTaskSettings.TaskDeviceValueNames[0], PSTR(PLUGIN_VALUENAME1_999));
    strcpy_P(ExtraTaskSettings.TaskDeviceValueNames[1], PSTR(PLUGIN_VALUENAME2_999));
    strcpy_P(ExtraTaskSettings.TaskDeviceValueNames[2], PSTR(PLUGIN_VALUENAME3_999));
    strcpy_P(ExtraTaskSettings.TaskDeviceValueNames[3], PSTR(PLUGIN_VALUENAME4_999));
    break;
  }

  case PLUGIN_SET_DEFAULTS:
  {
    for (uint8_t i = 0; i < VARS_PER_TASK; ++i) {
      ExtraTaskSettings.TaskDeviceValueDecimals[i] = 2;
    }

    //TODO
    P999_I2C_ADDR = DEF_I2C_ADDRESS_999;
    break;
  }

  case PLUGIN_WEBFORM_SHOW_I2C_PARAMS:
  {
    if ((P999_I2C_ADDR != 0x18))
    { // Validate I2C Addr.
      ;//P999_I2C_ADDR = DEF_I2C_ADDRESS_999;
    }
    String i2c_addres_string = formatToHex(P999_I2C_ADDR);
    addFormTextBox(F("I2C Address (Hex)"), F("i2c_addr"), i2c_addres_string, 4);
    break;
  }

  case PLUGIN_WEBFORM_LOAD:
  {
    //Parameter Wassertank
    addFormNumericBox(F("Water Area in cm^2 (d * d * 3,14 / 4)"), F("p999_water_area"), P999_WATER_AREA, 0, 1000000);
    addFormNumericBox(F("Max Water Level in mm"), F("p999_max_waterlevel"), P999_MAX_WATER_LEVEL, 0, 2000);
    addFormNumericBox(F("AtmPressEmpty in Pa"), F("p999_atmPressEmpty"), P999_PRESS_EMPTY, 0, 20000);
    //Wiring Pumpe und Ventil
    addFormPinSelect(PinSelectPurpose::Generic, F("Pump In1"), F("p999_pumppin1"), P999_PUMP_PIN1);
    addFormPinSelect(PinSelectPurpose::Generic, F("Pump In2"), F("p999_pumppin2"), P999_PUMP_PIN2);
    addFormPinSelect(PinSelectPurpose::Generic, F("Valve In3"), F("p999_valvepin1"), P999_VALVE_PIN1);
    addFormPinSelect(PinSelectPurpose::Generic, F("Valve In4"), F("p999_valvepin2"), P999_VALVE_PIN2);
    
    success = true;
    break;
  }

  case PLUGIN_WEBFORM_SAVE:
  {
    //i2C
    String i2c_address = webArg(F("i2c_addr"));
    P999_I2C_ADDR = (int)strtol(i2c_address.c_str(), 0, 16);
    //Parameter Wassertank
    P999_WATER_AREA  = getFormItemInt(F("p999_water_area"));
    P999_MAX_WATER_LEVEL  = getFormItemInt(F("p999_max_waterlevel"));
    P999_PRESS_EMPTY  = getFormItemInt(F("p999_atmPressEmpty"));    
    //Wiring Pumpe und Ventil
    P999_PUMP_PIN1 = getFormItemInt(F("p999_pumppin1"));
    P999_PUMP_PIN2 = getFormItemInt(F("p999_pumppin2"));
    P999_VALVE_PIN1 = getFormItemInt(F("p999_valvepin1"));
    P999_VALVE_PIN2 = getFormItemInt(F("p999_valvepin2"));
    
    success = true;
    break;
  }

  case PLUGIN_INIT:
  {
    // Enable I2C
    // Wire.begin(); // This is already done by ESP Easy Core

    // Motortreiber-Signale
    // Richtung Motor A
    pinMode(P999_VALVE_PIN2, OUTPUT);
    digitalWrite(P999_VALVE_PIN2, LOW);
    // PWM Motor A
    pinMode(P999_VALVE_PIN1, OUTPUT);
    digitalWrite(P999_VALVE_PIN1, P999_AUF);
    
    // Richtung Motor B
    pinMode(P999_PUMP_PIN2, OUTPUT);
    digitalWrite(P999_PUMP_PIN2, LOW);
    // PWM Motor B
    pinMode(P999_PUMP_PIN1, OUTPUT);
    digitalWrite(P999_PUMP_PIN1, P999_AUS); 

    P999_atmPressEmpty = P999_PRESS_EMPTY; //Pa

    P999_deleteValues(event);
    P999_MyInit = true;
    success = true;
    break;
  }

  case PLUGIN_EXIT:
  {    
    P999_MyInit = false;
    P999_deleteValues(event);    

    

    break;
  }

  case PLUGIN_READ:
  {
    if (P999_MyInit)
    {
      UserVar.setFloat(event->BaseVarIndex,0,P999_ValueStart); //--> Soll nur von den Rules beschrieben werden
      UserVar.setFloat(event->BaseVarIndex,1,P999_ValueHeight);
      UserVar.setFloat(event->BaseVarIndex,2,P999_ValueVolume);
      UserVar.setFloat(event->BaseVarIndex,3,P999_ValueLevel);
      success = true;
      break;
    }
    break;
  }

  case PLUGIN_WRITE:
  {
    String tmpString = string;
    String log;

    String cmd = parseString(tmpString, 1);
    String mycmd = F(P999_CMD_NAME);
    mycmd += String(event->TaskIndex + 1);
    
    if (cmd.equalsIgnoreCase(mycmd) || cmd.equalsIgnoreCase(F("TaskValueSet")))
    {
      log += F("WaterLevel: ");
      String paramCmd = parseString(tmpString, 2); // Output 1,2,3,4
      //String paramValue = parseString(tmpString, 3);   // 0 - 1023
     
      // Hier Befehle auswerten wenn benoetigt     
      if (paramCmd == "p1") {
        log += F("Pumpe P999_EIN");
        digitalWrite(P999_PUMP_PIN1, P999_EIN);
      }
      else if (paramCmd == "p0") {
        log += F("Pumpe P999_AUS");
        digitalWrite(P999_PUMP_PIN1, P999_AUS);
      }
      else if (paramCmd == "v1") {
        log += F("Ventil P999_ZU");
        digitalWrite(P999_VALVE_PIN1, P999_ZU);
      }
      else if (paramCmd == "v0") {
        log += F("Ventil P999_AUF");
        digitalWrite(P999_VALVE_PIN1, P999_AUF);
      }
      else if (paramCmd == "leer") {
        if (P999_messSchritt == 0) {
            log += F("Messung leer");
            P999_messungLeer = true;
            P999_ValueStart = 1;
        }
      }
      else if (paramCmd == "start") {
        if (P999_messSchritt == 0) {
            log += F("Messung gestartet");
            P999_ValueStart = 1;
        }
      }
      if (loglevelActiveFor(LOG_LEVEL_INFO))
      {
        addLogMove(LOG_LEVEL_INFO, log);
      }
      success = true; 
    }
  }
  case PLUGIN_TEN_PER_SECOND:
  {
    P999_readPressure(P999_I2C_ADDR);
    if(P999_messSchritt == 0)
    {
      if(P999_ValueStart != 0)
      {
        P999_messSchritt = 1;
      }
    }
    P999_step(event); //Schrittkette immer abarbeiten
    break;
  }
  break;
  }
  return success;
}

boolean P999_readPressure(uint8_t address)
{
  String logPress = F("WaterLevel Pressure: ");  
  // Drucksensor initialisieren
  // Die Default-Adresse des Sensors ist 0x18
  // Für andere Adresse oder I2C-Bus: mpr.begin(ADRESS, Wire1)
  if(!mpr.begin(address)) {
    addLog(LOG_LEVEL_INFO, F("WaterLevel : ReadPressure; connection failed"));
    return 0;
  }
  // Messwert in Pascal auslesen und filtern
  P999_messDruckRoh = int(mpr.readPressure(PA));
  P999_messDruck = ((P999_messDruck * 50) + P999_messDruckRoh) / 51;
  // Umrechnung Pa in mmH2O   
  P999_wassersaeule = (P999_messDruck - P999_atmDruck - P999_atmPressEmpty) * 10197 / 100000;
  if (P999_wassersaeule < 0) P999_wassersaeule = 0;

  logPress += P999_messDruckRoh;
  addLogMove(LOG_LEVEL_DEBUG_DEV, logPress);
  return 1;
}
void P999_step(struct EventStruct *event) {  
   String logStep = F("WaterLevel: ");  
   switch (P999_messSchritt) {
      case 0:  // Regelmäßig aktuellen atmosphärischen Druck erfassen
         if (!digitalRead(P999_VALVE_PIN1) && !digitalRead(P999_PUMP_PIN1)) {
            P999_atmDruck = P999_messDruck;      
         }
         break;

      case 1:  // Messung gestartet
         P999_vergleichswert = P999_messDruck;
         digitalWrite(P999_VALVE_PIN1, P999_ZU);
         digitalWrite(P999_PUMP_PIN1, P999_EIN);
         P999_messung = millis() + 2000;
         P999_messSchritt = 2;
         logStep += F("Startwert");
         logStep += (P999_vergleichswert);
         addLogMove(LOG_LEVEL_INFO, logStep);
         break;
      
      case 2:  // warten solange Druck steigt      
         if (P999_messDruck > P999_vergleichswert + 10) {
            P999_vergleichswert = P999_messDruck;
            P999_messung = millis() + 1000;
         }
         if (P999_wassersaeule > (P999_MAX_WATER_LEVEL + 200)) {
            Serial.println("Fehler: Messleitung verstopft!");
            P999_messSchritt = 4;
         }
         else if (P999_messung < millis()) {
            digitalWrite(P999_PUMP_PIN1, P999_AUS);
            P999_messung = millis() + 100;
            if (P999_messungLeer)
            {
              P999_messSchritt = 4;
            }
            else
            {
              P999_messSchritt = 3;
              logStep += F("Endwert: ");
              logStep += (P999_messDruck);
              addLogMove(LOG_LEVEL_INFO, logStep);
            }            
         }         
         break;
      case 3:  // Beruhigungszeit abgelaufen, Messwert ermitteln
         if (P999_messung < millis()) {
            P999_ValueHeight = (P999_wassersaeule / 10);// + "cm";
            P999_ValueVolume = (P999_wassersaeule / 10) * P999_WATER_AREA / 1000;// + "L";
            // Umrechnung Wassersäule in 0 - 100%
            P999_ValueLevel = map(P999_wassersaeule, 0, P999_MAX_WATER_LEVEL, 0, 100);//;+ "%";
            P999_messSchritt = 10;
         }
         break;
      case 4:  // Beruhigungszeit abgelaufen, Messwert ermitteln
         if (P999_messung < millis()) {
            P999_atmPressEmpty = P999_messDruck - P999_atmDruck; 
            P999_PRESS_EMPTY = P999_atmPressEmpty;
            SaveSettings();//Speichern in Nicht Fluechtigen Speicher
            P999_messungLeer = false;
            P999_messSchritt = 10;

            logStep += F("Leerdruck: ");
            logStep += (P999_atmPressEmpty);
            addLogMove(LOG_LEVEL_INFO, logStep);
         }
         break;
      case 10:  // Ablauf beenden
         digitalWrite(P999_VALVE_PIN1, P999_AUF);
         digitalWrite(P999_PUMP_PIN1, P999_AUS); 
         P999_ValueStart = 0; //Start Extern zuruecksetzen
         P999_messSchritt = 0;
         break;
         
      default:
         P999_messSchritt = 0;
         break;
   }
   
}
void P999_deleteValues(struct EventStruct *event)
{
  UserVar.setFloat(event->BaseVarIndex,0,-1); /
  UserVar.setFloat(event->BaseVarIndex,1,-1);
  UserVar.setFloat(event->BaseVarIndex,2,-1);
  UserVar.setFloat(event->BaseVarIndex,3,-1);
}
#endif // USES_P999
