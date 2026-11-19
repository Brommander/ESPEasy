#include "_Plugin_Helper.h"

#ifdef USES_P156

#ifdef ESP32
# include <lwip/sockets.h>
# include <errno.h>
#endif

// #######################################################################################################
// ######################## Plugin 156: General Modbus TCP ########################
// #######################################################################################################
//  PCONFIG:
//    [0] Port
//    [1] Unit ID
//    [2] alter gemeinsamer Lese-Funktionscode (0=Func03, 1=Func04). Nur noch für alte Konfigurationen:
//        "Lesen 03" (Funktion 0) liest mit 04, wenn hier 1 steht. Beim Speichern wird er auf 0 gesetzt.
//    [3-6] Datentyp Query 0-3 (0=U16,1=S16,2=U32,3=S32,4=Float)
//    [7] Pause-Gruppen gepackt (bits 0-1=Q0, 2-3=Q1, 4-5=Q2, 6-7=Q3)
//        Funktion je Wert (bits 8-9=Q0, 10-11=Q1, 12-13=Q2, 14-15=Q3):
//          0=03 lesen, 1=06 schreiben, 2=16 schreiben, 3=04 lesen
//  PCONFIG_LONG:
//    [0] IP als uint32
//    [1] reg0(bits0-15) | reg1(bits16-31)
//    [2] reg2(bits0-15) | reg3(bits16-31)
//    [3] pause1(8b,x100ms)|pause2(8b,x100ms)|pause3(8b,x1000ms)|pause4(8b,x1000ms)
//  getPluginCustomArgName (max Index 12):
//    [0]    = IP
//    [1..4] = Registeradresse Query 0..3
//    [5..8] = Pausezeit Gruppe 1..4 (ms)
//    [9..12]= Pause-Gruppe Query 0..3
//    [13..16]= Funktion Query 0..3
//
//  Funktionscode je Wert: 03/04 = lesen, 06/16 = zyklisch schreiben.
//  Zyklisch schreiben (Funktionscode 06 oder 16 bei einem Wert):
//    Der Taskwert wird im Takt seiner Pause-Gruppe in das Register geschrieben (Datentyp wie eingestellt).
//    Wert setzen: [<TaskName>].MBSet,<Wertnr 1-4>,<Wert>  (Rules, HTTP /control?cmd=..., MQTT <sysname>/cmd)
//    oder TaskValueSet (nur wenn "Allow TaskValueSet on all plugins" in den Advanced Settings an ist).
//    Nach dem Start wird nichts geschrieben, bis ein Wert gesetzt wurde.
//    Rules-Event WriteOK nur bei geändertem Wert oder nach einem Fehler, WriteError bei jedem Fehler.
//
//  Schreiben (Befehl, z. B. aus Rules, HTTP /control?cmd=... oder MQTT <sysname>/cmd):
//    [<TaskName>].MBWrite,<Register>,<Wert>[,<Datentyp>[,<FC>]]
//      Datentyp: u16, s16, u32, s32, float, u32ws, s32ws (oder 0..6).
//                Ohne Angabe: Datentyp der Query mit diesem Register, sonst u16.
//      FC:       6 (Write Single Register) oder 16 (Write Multiple Registers).
//                Ohne Angabe: 6 bei 16-Bit-Typen, 16 bei 32-Bit-Typen.
//    Ohne [<TaskName>]. wirkt der Befehl auf den ersten aktiven P156-Task.
//    Schreibaufträge haben Vorrang vor dem Lesen (Warteschlange mit 4 Plätzen).
//    Ergebnis als Rules-Event:
//      <TaskName>#WriteOK=<Register>,<Wert>
//      <TaskName>#WriteError=<Register>,<Code>
//        Code 1..11 = Modbus-Exception, 253 = Warteschlange voll,
//        254 = keine Verbindung, 255 = keine/ungültige Antwort (Timeout 500 ms)
//
//  Verbindungsaufbau (ESP32): nicht blockierend. connect() kehrt sofort zurück, der Ausgang wird in
//  den folgenden 100-ms-Takten ohne Warten geprüft (Timeout 1 s). Schlägt der Aufbau fehl oder liefert
//  das Gerät danach keine gültige Antwort, wartet der nächste Versuch 1, 2, 4, 8, 16, dann 30 s.
//  So bremst ein nicht erreichbares oder belegtes Gerät (z. B. Varta: nur ein Modbus-Client) das ESP nicht.
//  ESP8266: weiterhin blockierender Aufbau (max. 500 ms), aber mit derselben Wartezeit.
// #######################################################################################################

#define PLUGIN_156
#define PLUGIN_ID_156 156
#define PLUGIN_NAME_156 "Modbus TCP Generic"

#define P156_ARG_IP 0
#define P156_ARG_REG0 1
#define P156_ARG_PAU1 5
#define P156_ARG_GRP0 9
#define P156_ARG_MODE0 13

#define P156_MODE_READ 0
#define P156_MODE_FC6 1
#define P156_MODE_FC16 2
#define P156_MODE_RAW_READ04 3 // nur gespeichert; intern P156_MODE_READ mit readFc = 4

#define P156_PORT PCONFIG(0)
#define P156_PORT_LBL PCONFIG_LABEL(0)
#define P156_UID PCONFIG(1)
#define P156_UID_LBL PCONFIG_LABEL(1)
#define P156_FUNC PCONFIG(2)
#define P156_FUNC_LBL PCONFIG_LABEL(2)
#define P156_DT0 PCONFIG(3)
#define P156_DT0_LBL PCONFIG_LABEL(3)
#define P156_DT1 PCONFIG(4)
#define P156_DT1_LBL PCONFIG_LABEL(4)
#define P156_DT2 PCONFIG(5)
#define P156_DT2_LBL PCONFIG_LABEL(5)
#define P156_DT3 PCONFIG(6)
#define P156_DT3_LBL PCONFIG_LABEL(6)
#define P156_PGROUPS PCONFIG(7)

#define P156_IP_LONG PCONFIG_LONG(0)
#define P156_REGS01 PCONFIG_LONG(1)
#define P156_REGS23 PCONFIG_LONG(2)
#define P156_PAUSES PCONFIG_LONG(3)

#define P156_TYPE_U16 0
#define P156_TYPE_S16 1
#define P156_TYPE_U32 2
#define P156_TYPE_S32 3
#define P156_TYPE_FLOAT 4
#define P156_TYPE_U32WS 5
#define P156_TYPE_S32WS 6

#define P156_NR_OUTPUT_VALUES 4
#define P156_NR_PAUSE_GROUPS 4

#define P156_WRITE_QUEUE 4
#define P156_PENDING_NONE 255
#define P156_PENDING_WRITE 254
#define P156_WERR_QUEUE_FULL 253
#define P156_WERR_CONNECT 254
#define P156_WERR_TIMEOUT 255

// ============================================================
// Helper functions
// ============================================================
static uint8_t p156_getGroup(uint8_t qIdx, int16_t pg)
{
  return (uint8_t)((pg >> (qIdx * 2)) & 0x03);
}

static int16_t p156_packGroup(int16_t packed, uint8_t qIdx, uint8_t group)
{
  packed &= ~(0x03 << (qIdx * 2));
  packed |= ((group & 0x03) << (qIdx * 2));
  return packed;
}

static uint8_t p156_getMode(uint8_t qIdx, int16_t pg)
{
  return (uint8_t)((((uint16_t)pg) >> (8 + qIdx * 2)) & 0x03); // gespeicherter Rohwert 0..3
}

// Auswahl im Formular: 0=03 lesen, 1=04 lesen, 2=06 schreiben, 3=16 schreiben
static uint8_t p156_rawToUi(uint8_t raw, int16_t oldFunc)
{
  switch (raw)
  {
  case P156_MODE_FC6:
    return 2;
  case P156_MODE_FC16:
    return 3;
  case P156_MODE_RAW_READ04:
    return 1;
  }
  return (oldFunc == 1) ? 1 : 0; // alte Konfiguration mit gemeinsamem FC 04
}

static uint8_t p156_uiToRaw(uint8_t ui)
{
  switch (ui)
  {
  case 1:
    return P156_MODE_RAW_READ04;
  case 2:
    return P156_MODE_FC6;
  case 3:
    return P156_MODE_FC16;
  }
  return P156_MODE_READ;
}

static int16_t p156_packMode(int16_t packed, uint8_t qIdx, uint8_t mode)
{
  uint16_t p = (uint16_t)packed;
  p &= ~(0x03 << (8 + qIdx * 2));
  p |= ((mode & 0x03) << (8 + qIdx * 2));
  return (int16_t)p;
}

static uint16_t p156_getReg(uint8_t qIdx, int32_t r01, int32_t r23)
{
  if (qIdx == 0)
    return (uint16_t)(r01 & 0xFFFF);
  if (qIdx == 1)
    return (uint16_t)((r01 >> 16) & 0xFFFF);
  if (qIdx == 2)
    return (uint16_t)(r23 & 0xFFFF);
  return (uint16_t)((r23 >> 16) & 0xFFFF);
}

static int32_t p156_setReg(int32_t packed, uint8_t slot, uint16_t reg)
{
  if (slot == 0)
    return (packed & 0xFFFF0000L) | (int32_t)reg;
  return (packed & 0x0000FFFFL) | ((int32_t)reg << 16);
}

static uint32_t p156_getPauseMs(uint8_t group, int32_t pauses)
{
  switch (group)
  {
  case 0:
    return (uint32_t)((pauses >> 0) & 0xFF) * 100;
  case 1:
    return (uint32_t)((pauses >> 8) & 0xFF) * 100;
  case 2:
    return (uint32_t)((pauses >> 16) & 0xFF) * 1000;
  case 3:
    return (uint32_t)((pauses >> 24) & 0xFF) * 1000;
  }
  return 200;
}

static int32_t p156_setPause(int32_t packed, uint8_t group, uint32_t ms)
{
  uint8_t val;
  int shift;
  switch (group)
  {
  case 0:
    val = (uint8_t)(ms / 100);
    shift = 0;
    break;
  case 1:
    val = (uint8_t)(ms / 100);
    shift = 8;
    break;
  case 2:
    val = (uint8_t)(ms / 1000);
    shift = 16;
    break;
  default:
    val = (uint8_t)(ms / 1000);
    shift = 24;
    break;
  }
  packed &= ~(0xFF << shift);
  packed |= ((int32_t)val << shift);
  return packed;
}

// ============================================================
// Instance data
// ============================================================
struct P156_WriteJob
{
  uint16_t reg = 0;
  uint16_t words[2] = {0, 0};
  uint8_t nRegs = 1;
  uint8_t fc = 6;
  double value = 0.0; // nur für Event/Log
  uint8_t slot = 255; // Wert-Index bei zyklischem Schreiben, 255 = Befehl MBWrite
};

struct P156_Instance : public PluginTaskData_base
{
  taskIndex_t taskIndex = INVALID_TASK_INDEX;
  P156_WriteJob writeQueue[P156_WRITE_QUEUE];
  uint8_t writeHead = 0;
  uint8_t writeCount = 0;
  P156_WriteJob activeWrite;
  uint8_t rxBuf[12] = {};
  uint8_t rxLen = 0;
  WiFiClient client;
  char ip[16] = {};
  uint16_t port = 502;
  uint8_t unitId = 1;
  uint8_t readFc[P156_NR_OUTPUT_VALUES] = {3, 3, 3, 3}; // Funktionscode zum Lesen je Wert (3 oder 4)
  uint32_t regAddr[P156_NR_OUTPUT_VALUES] = {0, 0, 0, 0};
  uint8_t dataType[P156_NR_OUTPUT_VALUES] = {0, 0, 0, 0};
  uint8_t pauseGroup[P156_NR_OUTPUT_VALUES] = {0, 0, 0, 0};
  uint32_t pauseMs[P156_NR_PAUSE_GROUPS] = {200, 1000, 10000, 60000};
  boolean myInit = false;
  uint8_t queryIndex = 0;
  uint16_t sendCount = 0;
  uint16_t errorCount = 0;
  uint16_t reconnectCount = 0;
  uint32_t lastSend[P156_NR_OUTPUT_VALUES] = {0, 0, 0, 0}; // per query, not per group
  uint32_t requestTime = 0;                                // time when pending request was sent (0 = none)
  uint8_t pendingQuery = 255;                              // query index of pending request (255 = none)
  float values[P156_NR_OUTPUT_VALUES] = {0, 0, 0, 0};
  uint8_t mode[P156_NR_OUTPUT_VALUES] = {0, 0, 0, 0};          // P156_MODE_*
  float lastOk[P156_NR_OUTPUT_VALUES] = {NAN, NAN, NAN, NAN}; // zuletzt erfolgreich zyklisch geschriebener Wert
  uint8_t lastErr[P156_NR_OUTPUT_VALUES] = {0, 0, 0, 0};      // letzter gemeldeter Fehlercode (0 = keiner)
  bool armed[P156_NR_OUTPUT_VALUES] = {false, false, false, false}; // Schreibwert seit dem Start gesetzt
  float initVal[P156_NR_OUTPUT_VALUES] = {0, 0, 0, 0};              // Taskwert beim Start
  int connFd = -1;                // ESP32: Socket im Aufbau (-1 = kein Aufbau)
  uint32_t connStart = 0;          // Start des laufenden Aufbaus
  uint32_t nextConnectAllowed = 0; // frühester nächster Versuch (nur gültig wenn connAttempts > 0)
  uint8_t connAttempts = 0;        // Verbindungsversuche seit der letzten gültigen Antwort

  ~P156_Instance()
  {
#ifdef ESP32
    if (connFd >= 0)
      lwip_close(connFd);
#endif
  }
};

// Ergebnis von p156_connect()
#define P156_CONN_OK 1      // verbunden, Anfrage kann gesendet werden
#define P156_CONN_BUSY 0    // Aufbau läuft – im nächsten Takt erneut prüfen
#define P156_CONN_FAIL -1   // Aufbau eben fehlgeschlagen
#define P156_CONN_WAIT -2   // Wartezeit nach Fehlversuch oder kein Netzwerk
#define P156_CONNECT_TIMEOUT 1000
#define P156_BACKOFF_MAX 30000

bool p156_sendRequest(P156_Instance *inst, uint8_t qIdx);
bool p156_parseValues(P156_Instance *inst, uint8_t qIdx);
int8_t p156_connect(P156_Instance *inst);
void p156_connectFailed(P156_Instance *inst, const __FlashStringHelper *reason, int err);
bool p156_queueWrite(P156_Instance *inst, const String &cmd, String &err);
bool p156_encodeValue(double d, int type, P156_WriteJob &job, String &err);
bool p156_sendWrite(P156_Instance *inst);
uint8_t p156_parseWriteResponse(P156_Instance *inst, uint8_t &exCode);
void p156_writeResult(P156_Instance *inst, const P156_WriteJob &job, uint8_t code);

// ============================================================
// Plugin main function
// ============================================================
boolean Plugin_156(uint8_t function, struct EventStruct *event, String &string)
{
  boolean success = false;

  switch (function)
  {
  case PLUGIN_DEVICE_ADD:
  {
    auto &dev = Device[++deviceCount];
    dev.Number = PLUGIN_ID_156;
    dev.Type = DEVICE_TYPE_DUMMY;
    dev.VType = Sensor_VType::SENSOR_TYPE_QUAD;
    dev.Ports = 0;
    dev.PullUpOption = false;
    dev.InverseLogicOption = false;
    dev.FormulaOption = true;
    dev.ValueCount = P156_NR_OUTPUT_VALUES;
    dev.SendDataOption = true;
    dev.TimerOption = true;
    dev.GlobalSyncOption = true;
    break;
  }

  case PLUGIN_GET_DEVICENAME:
  {
    string = F(PLUGIN_NAME_156);
    break;
  }

  case PLUGIN_GET_DEVICEVALUENAMES:
  {
    for (uint8_t i = P156_NR_OUTPUT_VALUES; i < VARS_PER_TASK; ++i)
      ZERO_FILL(ExtraTaskSettings.TaskDeviceValueNames[i]);
    break;
  }

  case PLUGIN_SET_DEFAULTS:
  {
    P156_PORT = 502;
    P156_UID = 1;
    P156_FUNC = 0;
    P156_DT0 = P156_DT1 = P156_DT2 = P156_DT3 = P156_TYPE_U16;
    P156_PGROUPS = 0;
    IPAddress def(192, 168, 1, 1);
    P156_IP_LONG = (int32_t)(uint32_t)def;
    P156_REGS01 = P156_REGS23 = 0;
    int32_t p = 0;
    p = p156_setPause(p, 0, 200);
    p = p156_setPause(p, 1, 1000);
    p = p156_setPause(p, 2, 10000);
    p = p156_setPause(p, 3, 60000);
    P156_PAUSES = p;
    success = true;
    break;
  }

  case PLUGIN_WEBFORM_LOAD:
  {
    IPAddress ipAddr((uint32_t)P156_IP_LONG);
    String ipStr = ipAddr.toString();
    if (ipStr == F("0.0.0.0"))
      ipStr = F("192.168.1.1");

    uint32_t pau[P156_NR_PAUSE_GROUPS];
    for (uint8_t g = 0; g < P156_NR_PAUSE_GROUPS; ++g)
    {
      pau[g] = p156_getPauseMs(g, P156_PAUSES);
      if (pau[g] < 50)
        pau[g] = (g == 0) ? 200 : (g == 1) ? 1000
                              : (g == 2)   ? 10000
                                           : 60000;
    }

    // ---- Verbindung ----
    addFormHeader(F("Verbindung"));
    addFormTextBox(F("IPv4 Adresse"), getPluginCustomArgName(P156_ARG_IP), ipStr, 15);
    addFormNote(F("IP des Modbus-Geraets"));
    addFormNumericBox(F("Port"), P156_PORT_LBL, P156_PORT, 1, 65535);
    addFormNote(F("Typisch: 502 oder 1502"));
    addFormNumericBox(F("Unit ID"), P156_UID_LBL, P156_UID, 1, 255);
    addFormNote(F("Typisch: 1-255"));

    // ---- Pause-Gruppen ----
    addFormHeader(F("Pause-Gruppen"));
    addFormNote(F("Gruppe 1/2: Schritte 100ms  Gruppe 3/4: Schritte 1000ms"));
    for (uint8_t g = 0; g < P156_NR_PAUSE_GROUPS; ++g)
    {
      char buf[10];
      snprintf(buf, sizeof(buf), "%lu", (unsigned long)pau[g]);
      addFormTextBox(String(F("Gruppe ")) + (g + 1) + F(" (ms)"),
                     getPluginCustomArgName(P156_ARG_PAU1 + g), buf, 8);
    }

    // ---- Abfragen ----
    addFormHeader(F("Abfragen (4 Werte)"));

    const __FlashStringHelper *dtOpts[] = {
        F("U16 - unsigned 16-bit"),
        F("S16 - signed 16-bit"),
        F("U32 - unsigned 32-bit"),
        F("S32 - signed 32-bit"),
        F("Float - IEEE754"),
        F("U32 WS - unsigned 32-bit word-swapped"),
        F("S32 WS - signed 32-bit word-swapped"),
    };
    const __FlashStringHelper *grpOpts[] = {
        F("Gruppe 1"),
        F("Gruppe 2"),
        F("Gruppe 3"),
        F("Gruppe 4"),
    };
    const __FlashStringHelper *modeOpts[] = {
        F("03 - Holding Register lesen"),
        F("04 - Input Register lesen"),
        F("06 - Register schreiben (16 Bit)"),
        F("16 - Register schreiben (16/32 Bit, Float)"),
    };

    for (uint8_t i = 0; i < P156_NR_OUTPUT_VALUES; ++i)
    {
      addFormSubHeader(String(F("---- Wert ")) + (i + 1) + F(" ----"));

      char buf[8];
      snprintf(buf, sizeof(buf), "%u", p156_getReg(i, P156_REGS01, P156_REGS23));
      addFormTextBox(F("Registeradresse"), getPluginCustomArgName(P156_ARG_REG0 + i), buf, 6);
      addFormNote(F("PDU-Adresse 1-65535 (0 = Wert aus). 4xxxxx-Notation: Adresse minus 400001"));

      {
        FormSelectorOptions msel(NR_ELEMENTS(modeOpts), modeOpts);
        msel.addFormSelector(F("Funktionscode"),
                             getPluginCustomArgName(P156_ARG_MODE0 + i),
                             p156_rawToUi(p156_getMode(i, P156_PGROUPS), P156_FUNC));
      }

      {
        FormSelectorOptions tsel(NR_ELEMENTS(dtOpts), dtOpts);
        switch (i)
        {
        case 0:
          tsel.addFormSelector(F("Datentyp"), P156_DT0_LBL, P156_DT0);
          break;
        case 1:
          tsel.addFormSelector(F("Datentyp"), P156_DT1_LBL, P156_DT1);
          break;
        case 2:
          tsel.addFormSelector(F("Datentyp"), P156_DT2_LBL, P156_DT2);
          break;
        case 3:
          tsel.addFormSelector(F("Datentyp"), P156_DT3_LBL, P156_DT3);
          break;
        }
      }

      {
        FormSelectorOptions gsel(NR_ELEMENTS(grpOpts), grpOpts);
        gsel.addFormSelector(F("Pause-Gruppe"),
                             getPluginCustomArgName(P156_ARG_GRP0 + i),
                             p156_getGroup(i, P156_PGROUPS));
      }
    }
    addFormNote(F("Funktionscode 06/16: Der Taskwert wird im Takt der Pause-Gruppe ins Register geschrieben. Wert setzen mit "
                  "[Taskname].MBSet,Wertnr,Wert (z.B. aus Rules oder /control?cmd=...). Nach dem Start wird erst "
                  "geschrieben, wenn ein Wert gesetzt wurde. 06 geht nur mit 16-Bit-Typen, sonst wird 16 verwendet."));

    success = true;
    break;
  }

  case PLUGIN_WEBFORM_SAVE:
  {
    LoadTaskSettings(event->TaskIndex);
    String ipStr = webArg(getPluginCustomArgName(P156_ARG_IP));
    if (ipStr.length() < 7)
    {
      success = true;
      break;
    }

    IPAddress addr;
    if (addr.fromString(ipStr))
      P156_IP_LONG = (int32_t)(uint32_t)addr;

    P156_PORT = getFormItemInt(P156_PORT_LBL);
    P156_UID = getFormItemInt(P156_UID_LBL);
    P156_FUNC = 0; // Lese-Funktionscode steht jetzt je Wert (siehe unten)
    P156_DT0 = getFormItemInt(P156_DT0_LBL);
    P156_DT1 = getFormItemInt(P156_DT1_LBL);
    P156_DT2 = getFormItemInt(P156_DT2_LBL);
    P156_DT3 = getFormItemInt(P156_DT3_LBL);

    {
      int32_t pauses = 0;
      const uint32_t pdef[] = {200, 1000, 10000, 60000};
      for (uint8_t g = 0; g < P156_NR_PAUSE_GROUPS; ++g)
      {
        uint32_t ms = (uint32_t)webArg(getPluginCustomArgName(P156_ARG_PAU1 + g)).toInt();
        if (ms < 50)
          ms = pdef[g];
        pauses = p156_setPause(pauses, g, ms);
      }
      P156_PAUSES = pauses;
    }

    int32_t regs01 = 0, regs23 = 0;
    int16_t pack7 = 0;
    for (uint8_t i = 0; i < P156_NR_OUTPUT_VALUES; ++i)
    {
      uint16_t reg = (uint16_t)webArg(getPluginCustomArgName(P156_ARG_REG0 + i)).toInt();
      if (i <= 1)
        regs01 = p156_setReg(regs01, i, reg);
      else
        regs23 = p156_setReg(regs23, i - 2, reg);

      uint8_t grp = (uint8_t)webArg(getPluginCustomArgName(P156_ARG_GRP0 + i)).toInt();
      if (grp > 3)
        grp = 0;
      pack7 = p156_packGroup(pack7, i, grp);

      const uint8_t ui = (uint8_t)webArg(getPluginCustomArgName(P156_ARG_MODE0 + i)).toInt();
      pack7 = p156_packMode(pack7, i, p156_uiToRaw(ui));
    }
    P156_REGS01 = regs01;
    P156_REGS23 = regs23;
    P156_PGROUPS = pack7;

    if (loglevelActiveFor(LOG_LEVEL_INFO))
    {
      IPAddress dbgIP((uint32_t)P156_IP_LONG);
      String dbg = F("P156 SAVE[");
      dbg += event->TaskIndex;
      dbg += F("]: ip=[");
      dbg += dbgIP.toString();
      dbg += F("] reg0=");
      dbg += p156_getReg(0, P156_REGS01, P156_REGS23);
      dbg += F(" pau1=");
      dbg += p156_getPauseMs(0, P156_PAUSES);
      addLogMove(LOG_LEVEL_INFO, dbg);
    }

    P156_Instance *inst = static_cast<P156_Instance *>(getPluginTaskData(event->TaskIndex));
    if (inst)
      inst->myInit = false;
    success = true;
    break;
  }

  case PLUGIN_INIT:
  {
    initPluginTaskData(event->TaskIndex, new P156_Instance());
    P156_Instance *inst = static_cast<P156_Instance *>(getPluginTaskData(event->TaskIndex));
    if (!inst)
      break;
    if (inst->client.connected())
      inst->client.stop();

    IPAddress ipAddr((uint32_t)P156_IP_LONG);
    String ipStr = ipAddr.toString();
    if (ipStr == F("0.0.0.0"))
      ipStr = F("192.168.1.1");
    safe_strncpy(inst->ip, ipStr.c_str(), sizeof(inst->ip));

    inst->port = (P156_PORT > 0) ? (uint16_t)P156_PORT : 502;
    inst->unitId = (P156_UID > 0) ? (uint8_t)P156_UID : 1;

    int dtArr[] = {P156_DT0, P156_DT1, P156_DT2, P156_DT3};
    for (uint8_t i = 0; i < P156_NR_OUTPUT_VALUES; ++i)
    {
      inst->regAddr[i] = p156_getReg(i, P156_REGS01, P156_REGS23);
      inst->dataType[i] = (uint8_t)dtArr[i];
      inst->pauseGroup[i] = p156_getGroup(i, P156_PGROUPS);
      inst->values[i] = 0.0f;
      const uint8_t ui = p156_rawToUi(p156_getMode(i, P156_PGROUPS), P156_FUNC);
      inst->mode[i] = (ui == 2) ? P156_MODE_FC6 : ((ui == 3) ? P156_MODE_FC16 : P156_MODE_READ);
      inst->readFc[i] = (ui == 1) ? 4 : 3;
      if ((inst->mode[i] == P156_MODE_FC6) && (inst->dataType[i] >= P156_TYPE_U32))
        inst->mode[i] = P156_MODE_FC16; // 32 Bit / Float passen nicht in FC 06
      inst->lastOk[i] = NAN;
      inst->lastErr[i] = 0;
      // Schreibwert: erst schreiben, wenn nach dem Start ein Wert gesetzt wurde (MBSet oder geänderter Taskwert)
      inst->armed[i] = false;
      inst->initVal[i] = UserVar.getFloat(event->TaskIndex, i);
    }
    const uint32_t def[] = {200, 1000, 10000, 60000};
    for (uint8_t g = 0; g < P156_NR_PAUSE_GROUPS; ++g)
    {
      inst->pauseMs[g] = p156_getPauseMs(g, P156_PAUSES);
      if (inst->pauseMs[g] < 50)
        inst->pauseMs[g] = def[g];
    }
    for (uint8_t i = 0; i < P156_NR_OUTPUT_VALUES; ++i)
      inst->lastSend[i] = millis();
    inst->queryIndex = 0;
    inst->sendCount = 0;
    inst->errorCount = 0;
    inst->reconnectCount = 0;
    inst->requestTime = 0;
    inst->pendingQuery = P156_PENDING_NONE;
    inst->taskIndex = event->TaskIndex;
    inst->writeHead = 0;
    inst->writeCount = 0;
    inst->rxLen = 0;
    inst->myInit = true;
    success = true;

    if (loglevelActiveFor(LOG_LEVEL_INFO))
    {
      String log = F("P156 INIT[");
      log += event->TaskIndex;
      log += F("]: IP=");
      log += inst->ip;
      log += F(" Port=");
      log += inst->port;
      log += F(" Unit=0x");
      log += String(inst->unitId, HEX);
      log += F(" FC=");
      for (uint8_t i = 0; i < P156_NR_OUTPUT_VALUES; ++i)
      {
        if (i > 0)
          log += '/';
        log += (inst->mode[i] == P156_MODE_FC6) ? 6 : ((inst->mode[i] == P156_MODE_FC16) ? 16 : inst->readFc[i]);
      }
      log += F(" reg0=");
      log += inst->regAddr[0];
      addLogMove(LOG_LEVEL_INFO, log);
    }
    break;
  }

  case PLUGIN_EXIT:
  {
    P156_Instance *inst = static_cast<P156_Instance *>(getPluginTaskData(event->TaskIndex));
    if (inst)
    {
      inst->myInit = false;
      if (inst->client.connected())
        inst->client.stop();
    }
    break;
  }

  case PLUGIN_WRITE:
  {
    const String command = parseString(string, 1);
    if (equals(command, F("mbset")))
    {
      // MBSet,<Wertnr 1-4>,<Wert>: Schreibwert setzen (wird im Takt der Pause-Gruppe geschrieben)
      success = true;
      P156_Instance *inst = static_cast<P156_Instance *>(getPluginTaskData(event->TaskIndex));
      int32_t nr = 0;
      ESPEASY_RULES_FLOAT_TYPE d = 0;
      String err;
      if (!inst || !inst->myInit)
        err = F("Task nicht initialisiert");
      else if (!validIntFromString(parseString(string, 2), nr) || (nr < 1) || (nr > P156_NR_OUTPUT_VALUES))
        err = F("Wertnummer muss 1-4 sein");
      else if (inst->mode[nr - 1] == P156_MODE_READ)
        err = F("Wert ist auf Lesen eingestellt");
      else if (!validDoubleFromString(parseStringKeepCase(string, 3), d))
        err = F("ungültiger Wert");
      else
      {
        P156_WriteJob test;
        if (p156_encodeValue(d, inst->dataType[nr - 1], test, err))
        {
          UserVar.setFloat(event->TaskIndex, nr - 1, (float)d);
          inst->armed[nr - 1] = true;
        }
      }
      if (!err.isEmpty() && loglevelActiveFor(LOG_LEVEL_ERROR))
      {
        String l = F("P156 MBSet: ");
        l += err;
        l += F(" (");
        l += string;
        l += ')';
        addLogMove(LOG_LEVEL_ERROR, l);
      }
      break;
    }
    if (!equals(command, F("mbwrite")))
      break;
    success = true; // Befehl gehört zu P156 – Fehler werden geloggt, nicht an andere Plugins weitergereicht
    P156_Instance *inst = static_cast<P156_Instance *>(getPluginTaskData(event->TaskIndex));
    String err;
    if (!inst || !inst->myInit)
      err = F("Task nicht initialisiert");
    else if (!p156_queueWrite(inst, string, err) && err.isEmpty())
      err = F("unbekannter Fehler");
    if (!err.isEmpty())
    {
      if (loglevelActiveFor(LOG_LEVEL_ERROR))
      {
        String l = F("P156 MBWrite: ");
        l += err;
        l += F(" (");
        l += string;
        l += ')';
        addLogMove(LOG_LEVEL_ERROR, l);
      }
    }
    break;
  }

  case PLUGIN_READ:
  {
    P156_Instance *inst = static_cast<P156_Instance *>(getPluginTaskData(event->TaskIndex));
    if (inst && inst->myInit)
    {
      for (uint8_t i = 0; i < P156_NR_OUTPUT_VALUES; ++i)
      {
        if (inst->mode[i] == P156_MODE_READ) // Schreibwerte behalten den gesetzten Taskwert
          UserVar.setFloat(event->TaskIndex, i, inst->values[i]);
      }
      success = true;
    }
    break;
  }

  case PLUGIN_TEN_PER_SECOND:
  {
    P156_Instance *inst = static_cast<P156_Instance *>(getPluginTaskData(event->TaskIndex));
    if (!inst || !inst->myInit)
    {
      success = true;
      break;
    }

    // Two-phase Round-Robin:
    // Phase 1: if a request is pending, try to parse the response
    // Phase 2: only send a new request if no response is pending
    //          (queued write jobs are sent before the next read)
    if (inst->pendingQuery == P156_PENDING_WRITE)
    {
      // Phase 1 (write): pending write — try to read the response
      uint8_t exCode = 0;
      const uint8_t res = p156_parseWriteResponse(inst, exCode);
      if (res != 0)
      {
        inst->pendingQuery = P156_PENDING_NONE;
        inst->errorCount = 0; // device answered (also with exception)
        if (res != 3)
          inst->connAttempts = 0; // gültige Antwort – Verbindung ist in Ordnung
        p156_writeResult(inst, inst->activeWrite, (res == 1) ? 0 : ((res == 2) ? exCode : P156_WERR_TIMEOUT));
      }
      else if (timePassedSince(inst->requestTime) > 500)
      {
        inst->pendingQuery = P156_PENDING_NONE;
        inst->client.clear();
        inst->rxLen = 0;
        inst->errorCount++;
        p156_writeResult(inst, inst->activeWrite, P156_WERR_TIMEOUT);
      }
    }
    else if ((inst->pendingQuery == P156_PENDING_NONE) && (inst->writeCount > 0))
    {
      // Phase 2 (write): send the oldest queued write job
      // (während des Verbindungsaufbaus bleibt der Auftrag in der Warteschlange;
      //  in der Wartezeit nach Fehlversuchen wird er sofort mit 254 beendet)
      const int8_t conn = p156_connect(inst);
      if (conn != P156_CONN_BUSY)
      {
        inst->activeWrite = inst->writeQueue[inst->writeHead];
        inst->writeHead = (inst->writeHead + 1) % P156_WRITE_QUEUE;
        inst->writeCount--;
        if ((conn == P156_CONN_OK) && p156_sendWrite(inst))
          inst->pendingQuery = P156_PENDING_WRITE;
        else
        {
          inst->errorCount++;
          p156_writeResult(inst, inst->activeWrite, P156_WERR_CONNECT);
        }
      }
    }
    else if (inst->pendingQuery < P156_NR_OUTPUT_VALUES)
    {
      // Phase 1: pending request — try to read response
      uint8_t qIdx = inst->pendingQuery;
      bool ok = p156_parseValues(inst, qIdx);
      if (ok)
      {
        inst->pendingQuery = 255;
        inst->errorCount = 0;
        inst->connAttempts = 0; // gültige Antwort – Verbindung ist in Ordnung
        inst->lastSend[qIdx] = millis();
        inst->queryIndex = (qIdx + 1) % P156_NR_OUTPUT_VALUES;
      }
      else if (timePassedSince(inst->requestTime) > 500)
      {
        // Response timeout — give up, advance to next query
        inst->pendingQuery = 255;
        inst->client.clear();
        inst->lastSend[qIdx] = millis();
        inst->queryIndex = (qIdx + 1) % P156_NR_OUTPUT_VALUES;
        inst->errorCount++;
        if (inst->errorCount > 10)
        {
          inst->errorCount = 0;
          inst->client.stop();
          for (uint8_t i = 0; i < P156_NR_OUTPUT_VALUES; ++i)
            inst->values[i] = 0.0f;
          inst->reconnectCount++;
        }
      }
      // else: still within timeout window — wait, do nothing this cycle
    }
    else
    {
      // Phase 2: no pending request — send next request if timer has expired
      for (uint8_t attempt = 0; attempt < P156_NR_OUTPUT_VALUES; ++attempt)
      {
        uint8_t qIdx = (inst->queryIndex + attempt) % P156_NR_OUTPUT_VALUES;
        uint8_t group = inst->pauseGroup[qIdx];
        if (inst->regAddr[qIdx] == 0)
        {
          if (attempt == 0)
            inst->queryIndex = (qIdx + 1) % P156_NR_OUTPUT_VALUES;
          continue;
        }
        if (timePassedSince(inst->lastSend[qIdx]) >= inst->pauseMs[group])
        {
          P156_WriteJob cyc;
          if (inst->mode[qIdx] != P156_MODE_READ)
          {
            // Zyklisch schreiben: Taskwert holen und kodieren (vor dem Verbindungsaufbau)
            const float v = UserVar.getFloat(event->TaskIndex, qIdx);
            if (!inst->armed[qIdx] && (v != inst->initVal[qIdx]) && !isnan(v))
              inst->armed[qIdx] = true; // Taskwert wurde geändert (z. B. TaskValueSet)
            String err;
            if (!inst->armed[qIdx] || isnan(v) || !p156_encodeValue(v, inst->dataType[qIdx], cyc, err))
            {
              if (!err.isEmpty() && loglevelActiveFor(LOG_LEVEL_ERROR))
              {
                String l = F("P156 Schreibwert ");
                l += qIdx + 1;
                l += F(": ");
                l += err;
                addLogMove(LOG_LEVEL_ERROR, l);
              }
              inst->lastSend[qIdx] = millis(); // noch kein (gültiger) Wert – nächste Runde
              inst->queryIndex = (qIdx + 1) % P156_NR_OUTPUT_VALUES;
              break;
            }
            cyc.reg = (uint16_t)inst->regAddr[qIdx];
            cyc.fc = (inst->mode[qIdx] == P156_MODE_FC6) ? 6 : 16;
            cyc.slot = qIdx;
          }

          const int8_t conn = p156_connect(inst);
          if ((conn == P156_CONN_BUSY) || (conn == P156_CONN_WAIT))
            break; // Aufbau läuft oder Wartezeit – nichts blockieren, später erneut
          if (inst->mode[qIdx] != P156_MODE_READ)
          {
            inst->lastSend[qIdx] = millis();
            inst->queryIndex = (qIdx + 1) % P156_NR_OUTPUT_VALUES;
            inst->activeWrite = cyc;
            if ((conn == P156_CONN_OK) && p156_sendWrite(inst))
              inst->pendingQuery = P156_PENDING_WRITE;
            else
            {
              inst->errorCount++;
              p156_writeResult(inst, inst->activeWrite, P156_WERR_CONNECT);
            }
            break;
          }
          bool ok = (conn == P156_CONN_OK) && p156_sendRequest(inst, qIdx);
          if (ok)
          {
            inst->pendingQuery = qIdx; // response will be read in next cycle(s)
          }
          else
          {
            inst->errorCount++;
            inst->lastSend[qIdx] = millis();
            inst->queryIndex = (qIdx + 1) % P156_NR_OUTPUT_VALUES;
            if (inst->errorCount > 10)
            {
              inst->errorCount = 0;
              inst->client.clear();
              inst->client.stop();
              for (uint8_t i = 0; i < P156_NR_OUTPUT_VALUES; ++i)
                inst->values[i] = 0.0f;
              inst->reconnectCount++;
            }
          }
          break;
        }
      }
    }
    success = true;
    break;
  }
  }
  return success;
}

// Wartezeit nach dem n-ten Versuch ohne gültige Antwort: 1, 2, 4, 8, 16, 30, 30 … s
static uint32_t p156_backoffMs(uint8_t attempts)
{
  if (attempts == 0)
    return 0;
  const uint8_t shift = (attempts > 6) ? 5 : (attempts - 1);
  const uint32_t ms = 1000UL << shift;
  return (ms > P156_BACKOFF_MAX) ? P156_BACKOFF_MAX : ms;
}

void p156_connectFailed(P156_Instance *inst, const __FlashStringHelper *reason, int err)
{
#ifdef ESP32
  if (inst->connFd >= 0)
  {
    lwip_close(inst->connFd);
    inst->connFd = -1;
  }
#endif
  if (inst->connAttempts >= 3)
  {
    // Gerät seit mehreren Versuchen nicht erreichbar: keine alten Werte weitermelden
    for (uint8_t i = 0; i < P156_NR_OUTPUT_VALUES; ++i)
      inst->values[i] = 0.0f;
  }
  inst->reconnectCount++;
  if (loglevelActiveFor(LOG_LEVEL_INFO))
  {
    String l = F("P156: connect ");
    l += inst->ip;
    l += ':';
    l += inst->port;
    l += F(" failed (");
    l += reason;
    if (err != 0)
    {
      l += ' ';
      l += err;
    }
    l += F("), next try in ");
    l += p156_backoffMs(inst->connAttempts) / 1000;
    l += 's';
    addLogMove(LOG_LEVEL_INFO, l);
  }
}

// Nicht-blockierender Verbindungsaufbau, wird jeden Takt aufgerufen solange eine Anfrage ansteht.
// Rückgabe: P156_CONN_OK / _BUSY / _FAIL / _WAIT (siehe oben)
int8_t p156_connect(P156_Instance *inst)
{
#ifdef ESP32
  if (inst->connFd >= 0)
  {
    // Aufbau läuft: ohne zu warten prüfen, ob der Socket fertig ist
    fd_set wset;
    FD_ZERO(&wset);
    FD_SET(inst->connFd, &wset);
    struct timeval tv = {0, 0};
    const int res = lwip_select(inst->connFd + 1, nullptr, &wset, nullptr, &tv);
    if (res == 0)
    {
      if (timePassedSince(inst->connStart) < P156_CONNECT_TIMEOUT)
        return P156_CONN_BUSY;
      p156_connectFailed(inst, F("timeout"), 0);
      return P156_CONN_FAIL;
    }
    int sockErr = 0;
    socklen_t len = sizeof(sockErr);
    if (res < 0)
      sockErr = errno;
    else if (lwip_getsockopt(inst->connFd, SOL_SOCKET, SO_ERROR, &sockErr, &len) < 0)
      sockErr = errno;
    if (sockErr != 0)
    {
      p156_connectFailed(inst, F("error"), sockErr);
      return P156_CONN_FAIL;
    }
    // Verbunden: Socket wie bei NetworkClient::connect() wieder blockierend schalten und übergeben
    const int fd = inst->connFd;
    inst->connFd = -1;
    lwip_fcntl(fd, F_SETFL, lwip_fcntl(fd, F_GETFL, 0) & ~O_NONBLOCK);
    inst->client = WiFiClient(fd);
    inst->client.setConnectionTimeout(500); // Lese-/Schreib-Timeout wie bisher
    inst->rxLen = 0;
    return P156_CONN_OK;
  }
#endif

  if (inst->client.connected())
    return P156_CONN_OK;
  // Kein Versuch ohne Netzwerk (verhindert Absturz beim Booten)
  if (!NetworkConnected(0))
    return P156_CONN_WAIT;
  if ((inst->connAttempts > 0) && !timeOutReached(inst->nextConnectAllowed))
    return P156_CONN_WAIT;

  inst->client.stop(); // Reste einer abgebrochenen Verbindung freigeben
  if (inst->connAttempts < 255)
    inst->connAttempts++;
  inst->nextConnectAllowed = millis() + p156_backoffMs(inst->connAttempts);

#ifdef ESP32
  const uint32_t addr = ipaddr_addr(inst->ip);
  if (addr == IPADDR_NONE)
  {
    p156_connectFailed(inst, F("bad IP"), 0);
    return P156_CONN_FAIL;
  }
  struct sockaddr_in sa = {};
  sa.sin_family = AF_INET;
  sa.sin_port = htons(inst->port);
  sa.sin_addr.s_addr = addr;
  const int fd = lwip_socket(AF_INET, SOCK_STREAM, 0);
  if (fd < 0)
  {
    p156_connectFailed(inst, F("socket"), errno);
    return P156_CONN_FAIL;
  }
  lwip_fcntl(fd, F_SETFL, lwip_fcntl(fd, F_GETFL, 0) | O_NONBLOCK);
  if ((lwip_connect(fd, (struct sockaddr *)&sa, sizeof(sa)) < 0) && (errno != EINPROGRESS))
  {
    const int e = errno;
    lwip_close(fd);
    p156_connectFailed(inst, F("connect"), e);
    return P156_CONN_FAIL;
  }
  inst->connFd = fd;
  inst->connStart = millis();
  return P156_CONN_BUSY; // Ergebnis im nächsten Takt per select() prüfen
#else
  if (!inst->client.connect(inst->ip, inst->port, 500)) // ESP8266: blockierend, max. 500 ms
  {
    p156_connectFailed(inst, F("timeout"), 0);
    return P156_CONN_FAIL;
  }
  return P156_CONN_OK;
#endif
}

bool p156_sendRequest(P156_Instance *inst, uint8_t qIdx)
{
  if (!inst)
    return false;
  if (!inst->client.connected())
    return false;
  if (inst->sendCount >= 65535)
    inst->sendCount = 1;
  else
    inst->sendCount++;
  uint8_t regCount = (inst->dataType[qIdx] >= P156_TYPE_U32) ? 2 : 1;
  byte req[12];
  req[0] = (uint8_t)(inst->sendCount >> 8);
  req[1] = (uint8_t)inst->sendCount;
  req[2] = 0;
  req[3] = 0;
  req[4] = 0;
  req[5] = 6;
  req[6] = inst->unitId;
  req[7] = inst->readFc[qIdx];
  req[8] = (uint8_t)(inst->regAddr[qIdx] >> 8);
  req[9] = (uint8_t)inst->regAddr[qIdx];
  req[10] = 0;
  req[11] = regCount;
  inst->requestTime = millis(); // record send time for response timeout
  inst->client.write(req, sizeof(req));
  return true;
}

bool p156_parseValues(P156_Instance *inst, uint8_t qIdx)
{
  if (!inst)
    return false;

  // Non-blocking: return false immediately if response not yet arrived.
  // Timeout is handled by the caller (TEN_PER_SECOND Phase 1).
  // minBytes: 9 header bytes + 2 (U16/S16) or 4 (U32+) data bytes
  size_t available = inst->client.available();
  const uint8_t minBytes = 9 + ((inst->dataType[qIdx] >= P156_TYPE_U32) ? 4 : 2);
  if (available < minBytes)
    return false;

  int avail = available; // use cached value
  uint8_t dB = (inst->dataType[qIdx] >= P156_TYPE_U32) ? 4 : 2;
  uint8_t tH = 0, tL = 0;

  // Read header bytes (avail - dB), capture transaction ID from bytes 0-1
  for (int a = 0; a < avail - dB; a++)
  {
    byte b = inst->client.read();
    if (a == 0)
      tH = b;
    if (a == 1)
      tL = b;
  }
  if (inst->sendCount != (uint16_t)((tH << 8) | tL))
  {
    inst->client.clear();
    return false;
  }
  uint8_t h1 = 0, l1 = 0, h2 = 0, l2 = 0;
  if (dB > 0)
    h1 = inst->client.read();
  if (dB > 1)
    l1 = inst->client.read();
  if (dB > 2)
    h2 = inst->client.read();
  if (dB > 3)
    l2 = inst->client.read();
  float v = 0.0f;
  switch (inst->dataType[qIdx])
  {
  case P156_TYPE_U16:
    v = (uint16_t)((h1 << 8) | l1);
    break;
  case P156_TYPE_S16:
    v = (int16_t)((h1 << 8) | l1);
    break;
  case P156_TYPE_U32:
    v = (uint32_t)(((uint32_t)h1 << 24) | ((uint32_t)l1 << 16) | ((uint32_t)h2 << 8) | (uint32_t)l2);
    break;
  case P156_TYPE_S32:
    v = (int32_t)(((uint32_t)h1 << 24) | ((uint32_t)l1 << 16) | ((uint32_t)h2 << 8) | (uint32_t)l2);
    break;
  case P156_TYPE_FLOAT:
  {
    unsigned char b[] = {l2, h2, l1, h1};
    memcpy(&v, b, sizeof(float));
    break;
  }
  case P156_TYPE_U32WS: // Word-Swapped: h2/l2=HighWord, h1/l1=LowWord (Varta, Sungrow)
    v = (uint32_t)(((uint32_t)h2 << 24) | ((uint32_t)l2 << 16) | ((uint32_t)h1 << 8) | (uint32_t)l1);
    break;
  case P156_TYPE_S32WS:
    v = (int32_t)(((uint32_t)h2 << 24) | ((uint32_t)l2 << 16) | ((uint32_t)h1 << 8) | (uint32_t)l1);
    break;
  }
  inst->values[qIdx] = v;
  return true;
}

// ============================================================
// Write support (MBWrite command)
// ============================================================

// Datentyp aus Text: u16, s16, u32, s32, float, u32ws, s32ws oder Ziffer 0..6; -1 = ungültig
static int p156_parseType(const String &s)
{
  if (s.isEmpty())
    return -2; // nicht angegeben
  const char *names[] = {"u16", "s16", "u32", "s32", "float", "u32ws", "s32ws"};
  for (int i = 0; i <= P156_TYPE_S32WS; ++i)
  {
    if (s.equalsIgnoreCase(names[i]))
      return i;
  }
  int32_t n = 0;
  if (validIntFromString(s, n) && (n >= 0) && (n <= P156_TYPE_S32WS))
    return n;
  return -1;
}

bool p156_queueWrite(P156_Instance *inst, const String &cmd, String &err)
{
  // MBWrite,<Register>,<Wert>[,<Datentyp>[,<FC>]]
  int32_t reg = 0;
  if (!validIntFromString(parseString(cmd, 2), reg) || (reg < 0) || (reg > 65535))
  {
    err = F("ungültiges Register");
    return false;
  }
  ESPEASY_RULES_FLOAT_TYPE d = 0;
  if (!validDoubleFromString(parseStringKeepCase(cmd, 3), d))
  {
    err = F("ungültiger Wert");
    return false;
  }
  int type = p156_parseType(parseString(cmd, 4));
  if (type == -1)
  {
    err = F("ungültiger Datentyp");
    return false;
  }
  if (type == -2)
  {
    // Datentyp der Query mit diesem Register übernehmen, sonst U16
    type = P156_TYPE_U16;
    for (uint8_t i = 0; i < P156_NR_OUTPUT_VALUES; ++i)
    {
      if ((inst->regAddr[i] != 0) && (inst->regAddr[i] == (uint32_t)reg))
      {
        type = inst->dataType[i];
        break;
      }
    }
  }

  P156_WriteJob job;
  if (!p156_encodeValue(d, type, job, err))
    return false;
  job.reg = (uint16_t)reg;

  const String fcStr = parseString(cmd, 5);
  if (fcStr.isEmpty())
    job.fc = (job.nRegs == 1) ? 6 : 16;
  else
  {
    int32_t fc = 0;
    if (!validIntFromString(fcStr, fc) || ((fc != 6) && (fc != 16)))
    {
      err = F("FC muss 6 oder 16 sein");
      return false;
    }
    if ((fc == 6) && (job.nRegs != 1))
    {
      err = F("FC 6 nur für 16-Bit-Typen");
      return false;
    }
    job.fc = (uint8_t)fc;
  }

  if (inst->writeCount >= P156_WRITE_QUEUE)
  {
    err = F("Warteschlange voll");
    p156_writeResult(inst, job, P156_WERR_QUEUE_FULL);
    return false;
  }
  const uint8_t slot = (inst->writeHead + inst->writeCount) % P156_WRITE_QUEUE;
  inst->writeQueue[slot] = job;
  inst->writeCount++;
  return true;
}

// Wert in 1 oder 2 Registerworte umsetzen (Bereich wird geprüft). Setzt value, nRegs, words.
bool p156_encodeValue(double d, int type, P156_WriteJob &job, String &err)
{
  job.value = d;
  uint32_t u = 0;
  switch (type)
  {
  case P156_TYPE_U16:
    if ((d < 0) || (d > 65535))
    {
      err = F("Wert außerhalb 0..65535");
      return false;
    }
    job.nRegs = 1;
    job.words[0] = (uint16_t)lround(d);
    break;
  case P156_TYPE_S16:
    if ((d < -32768) || (d > 32767))
    {
      err = F("Wert außerhalb -32768..32767");
      return false;
    }
    job.nRegs = 1;
    job.words[0] = (uint16_t)(int16_t)lround(d);
    break;
  case P156_TYPE_U32:
  case P156_TYPE_U32WS:
    if ((d < 0) || (d > 4294967295.0))
    {
      err = F("Wert außerhalb 0..4294967295");
      return false;
    }
    u = (uint32_t)llround(d);
    break;
  case P156_TYPE_S32:
  case P156_TYPE_S32WS:
    if ((d < -2147483648.0) || (d > 2147483647.0))
    {
      err = F("Wert außerhalb des S32-Bereichs");
      return false;
    }
    u = (uint32_t)(int32_t)llround(d);
    break;
  case P156_TYPE_FLOAT:
  {
    const float f = (float)d;
    memcpy(&u, &f, sizeof(u));
    break;
  }
  }
  if (type >= P156_TYPE_U32)
  {
    job.nRegs = 2;
    if ((type == P156_TYPE_U32WS) || (type == P156_TYPE_S32WS))
    {
      // Word-Swapped: erstes Register = Low-Word (wie beim Lesen)
      job.words[0] = (uint16_t)(u & 0xFFFF);
      job.words[1] = (uint16_t)(u >> 16);
    }
    else
    {
      // Standard (auch Float): erstes Register = High-Word
      job.words[0] = (uint16_t)(u >> 16);
      job.words[1] = (uint16_t)(u & 0xFFFF);
    }
  }
  return true;
}

bool p156_sendWrite(P156_Instance *inst)
{
  if (!inst)
    return false;
  if (!inst->client.connected())
    return false;
  if (inst->sendCount >= 65535)
    inst->sendCount = 1;
  else
    inst->sendCount++;

  const P156_WriteJob &job = inst->activeWrite;
  byte req[17];
  uint8_t len = 0;
  req[0] = (uint8_t)(inst->sendCount >> 8);
  req[1] = (uint8_t)inst->sendCount;
  req[2] = 0;
  req[3] = 0;
  req[6] = inst->unitId;
  req[7] = job.fc;
  req[8] = (uint8_t)(job.reg >> 8);
  req[9] = (uint8_t)job.reg;
  if (job.fc == 6)
  {
    req[10] = (uint8_t)(job.words[0] >> 8);
    req[11] = (uint8_t)job.words[0];
    len = 12;
  }
  else
  {
    req[10] = 0;
    req[11] = job.nRegs;
    req[12] = (uint8_t)(job.nRegs * 2);
    len = 13;
    for (uint8_t i = 0; i < job.nRegs; ++i)
    {
      req[len++] = (uint8_t)(job.words[i] >> 8);
      req[len++] = (uint8_t)job.words[i];
    }
  }
  const uint16_t pduLen = len - 6; // Unit-ID + PDU
  req[4] = (uint8_t)(pduLen >> 8);
  req[5] = (uint8_t)pduLen;

  inst->client.clear(); // verspätete Bytes einer abgebrochenen Anfrage verwerfen
  inst->rxLen = 0;
  inst->requestTime = millis();
  return inst->client.write(req, len) == len;
}

// 0 = noch nicht vollständig, 1 = OK, 2 = Modbus-Exception (exCode), 3 = ungültige Antwort
uint8_t p156_parseWriteResponse(P156_Instance *inst, uint8_t &exCode)
{
  for (;;)
  {
    // Exception-Antwort: 9 Bytes, normale Antwort auf FC 6/16: 12 Bytes
    const uint8_t need = ((inst->rxLen >= 8) && (inst->rxBuf[7] & 0x80)) ? 9 : 12;
    if (inst->rxLen >= need)
      break;
    if (inst->client.available() <= 0)
      return 0;
    const int n = inst->client.read(inst->rxBuf + inst->rxLen, need - inst->rxLen);
    if (n <= 0)
      return 0;
    inst->rxLen += (uint8_t)n;
  }
  const uint8_t *b = inst->rxBuf;
  const P156_WriteJob &job = inst->activeWrite;
  inst->rxLen = 0;

  if ((((uint16_t)b[0] << 8) | b[1]) != inst->sendCount)
    return 3;
  if ((b[2] != 0) || (b[3] != 0))
    return 3;
  if ((b[7] & 0x7F) != job.fc)
    return 3;
  if (b[7] & 0x80)
  {
    exCode = b[8];
    return 2;
  }
  if ((((uint16_t)b[8] << 8) | b[9]) != job.reg)
    return 3;
  const uint16_t echo = ((uint16_t)b[10] << 8) | b[11];
  if (job.fc == 6)
    return (echo == job.words[0]) ? 1 : 3;
  return (echo == job.nRegs) ? 1 : 3;
}

void p156_writeResult(P156_Instance *inst, const P156_WriteJob &job, uint8_t code)
{
  if (job.slot < P156_NR_OUTPUT_VALUES)
  {
    // Zyklisches Schreiben: OK nur melden, wenn sich der Wert geändert hat oder vorher ein Fehler war
    const float v = (float)job.value;
    if (code == 0)
    {
      inst->lastErr[job.slot] = 0;
      if (v == inst->lastOk[job.slot])
        return;
      inst->lastOk[job.slot] = v;
    }
    else
    {
      inst->lastOk[job.slot] = NAN;
      if (code == inst->lastErr[job.slot])
        return; // derselbe Fehler wie beim letzten Mal – nicht in jedem Takt melden
      inst->lastErr[job.slot] = code;
    }
  }

  const String taskName = getTaskDeviceName(inst->taskIndex);
  const String valStr = doubleToString(job.value, 3, true);

  if (code == 0)
  {
    // Register wird gelesen? Dann sofort neu lesen, damit der Taskwert stimmt
    for (uint8_t i = 0; i < P156_NR_OUTPUT_VALUES; ++i)
    {
      if ((inst->mode[i] == P156_MODE_READ) && (inst->regAddr[i] != 0) && (inst->regAddr[i] == job.reg))
        inst->lastSend[i] = millis() - inst->pauseMs[inst->pauseGroup[i]] - 1;
    }
  }

  if (loglevelActiveFor(code == 0 ? LOG_LEVEL_INFO : LOG_LEVEL_ERROR))
  {
    String l = F("P156 MBWrite ");
    l += taskName;
    l += F(" reg=");
    l += job.reg;
    l += F(" FC=");
    l += job.fc;
    l += F(" val=");
    l += valStr;
    if (code == 0)
      l += F(" OK");
    else
    {
      l += F(" Fehler ");
      l += code;
    }
    addLogMove(code == 0 ? LOG_LEVEL_INFO : LOG_LEVEL_ERROR, l);
  }

  if (Settings.UseRules)
  {
    String ev = taskName;
    if (code == 0)
    {
      ev += F("#WriteOK=");
      ev += job.reg;
      ev += ',';
      ev += valStr;
    }
    else
    {
      ev += F("#WriteError=");
      ev += job.reg;
      ev += ',';
      ev += code;
    }
    eventQueue.addMove(std::move(ev));
  }
}

#endif // USES_P156