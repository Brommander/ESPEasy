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
//  Gemeinsame Verbindung: Alle P156-Tasks mit derselben IP und demselben Port nutzen eine TCP-Verbindung
//  (viele Geräte, z. B. Varta, erlauben nur sehr wenige Modbus-Clients). Je Verbindung ist immer nur eine Anfrage
//  unterwegs; wartet ein anderer Task, kommt er als Nächster dran. Transaktions-IDs werden je Verbindung vergeben.
//  Blockweises Lesen: Ist ein Lesewert fällig, werden alle anderen Lesewerte mit demselben Funktionscode, die
//  zusammen in höchstens 32 Registern liegen, in derselben Anfrage mitgelesen (zählen als frisch gelesen).
//  Antwortet das Gerät auf einen Block mit einer Exception, wird für diesen Task wieder einzeln gelesen.
//  Nach 3 Fehlversuchen in Folge stehen die Lesewerte lokal auf 0 (für Rules), an die Controller
//  wird bis zur nächsten gültigen Antwort nichts gesendet.
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
  bool gotValue[P156_NR_OUTPUT_VALUES] = {false, false, false, false}; // Lesewert seit dem Start mindestens einmal gelesen
  uint32_t initTime = 0;                                               // Zeitpunkt von PLUGIN_INIT
  float initVal[P156_NR_OUTPUT_VALUES] = {0, 0, 0, 0};              // Taskwert beim Start
  struct P156_Conn *conn = nullptr; // gemeinsame Verbindung zum Gerät (IP + Port)
  // Blockweises Lesen: eine Anfrage liest alle nahe beieinander liegenden Lesewerte
  uint16_t blockStart = 0;         // erstes Register der laufenden Leseanfrage
  uint8_t blockCount = 0;          // Anzahl Register
  uint8_t blockFc = 3;             // Funktionscode 3 oder 4
  uint8_t blockMask = 0;           // Bit i = Wert i ist in der Anfrage enthalten
  bool noBlock = false;            // Gerät lehnt Blöcke ab -> Werte einzeln lesen
  bool hdrRead = false;            // Kopf der Antwort schon gelesen
  uint8_t hdr[9] = {};             // MBAP-Kopf + Funktionscode + Bytezahl/Exception
  uint8_t exLogged = 0;            // Bit i = Exception für Wert i schon geloggt
  uint32_t statRequests = 0;       // Leseanfragen seit dem Start
  uint32_t statValues = 0;         // damit gelesene Werte

  ~P156_Instance();
};

// ---- Gemeinsame Verbindungen: alle Tasks mit derselben IP und demselben Port nutzen eine TCP-Verbindung ----
// Es ist immer nur eine Anfrage je Verbindung unterwegs (owner); ein wartender Task kommt als Nächster dran.
struct P156_Conn
{
  char ip[16] = {};
  uint16_t port = 502;
  WiFiClient client;
  int connFd = -1;                  // ESP32: Socket im Aufbau (-1 = kein Aufbau)
  uint32_t connStart = 0;           // Start des laufenden Aufbaus
  uint32_t nextConnectAllowed = 0;  // frühester nächster Versuch (nur gültig wenn connAttempts > 0)
  uint8_t connAttempts = 0;         // Verbindungsversuche seit der letzten gültigen Antwort
  uint16_t tid = 0;                 // letzte Transaktions-ID dieser Verbindung
  uint8_t refCount = 0;             // Anzahl Tasks, die diese Verbindung nutzen
  taskIndex_t owner = INVALID_TASK_INDEX;     // Task mit laufender Anfrage
  taskIndex_t lastOwner = INVALID_TASK_INDEX; // Task der letzten Anfrage
  taskIndex_t waiting = INVALID_TASK_INDEX;   // Task, der auf die Verbindung wartet

  ~P156_Conn()
  {
#ifdef ESP32
    if (connFd >= 0)
      lwip_close(connFd);
#endif
    client.stop();
  }
};

#define P156_MAX_CONN 8
P156_Conn *p156_getConn(const char *ip, uint16_t port);
void p156_putConn(P156_Conn *c);
uint16_t p156_nextTid(P156_Conn *c);
bool p156_acquire(P156_Instance *inst);
void p156_release(P156_Instance *inst);
static P156_Conn *p156_conns[P156_MAX_CONN] = {};

// Verbindung zu ip:port holen (vorhandene teilen oder neu anlegen)
P156_Conn *p156_getConn(const char *ip, uint16_t port)
{
  for (uint8_t i = 0; i < P156_MAX_CONN; ++i)
  {
    P156_Conn *c = p156_conns[i];
    if (c && (c->port == port) && (strcmp(c->ip, ip) == 0))
    {
      c->refCount++;
      return c;
    }
  }
  for (uint8_t i = 0; i < P156_MAX_CONN; ++i)
  {
    if (p156_conns[i] == nullptr)
    {
      P156_Conn *c = new (std::nothrow) P156_Conn();
      if (!c)
        return nullptr;
      safe_strncpy(c->ip, ip, sizeof(c->ip));
      c->port = port;
      c->refCount = 1;
      p156_conns[i] = c;
      return c;
    }
  }
  return nullptr;
}

// Verbindung nicht mehr nutzen; die letzte Nutzung schließt sie
void p156_putConn(P156_Conn *c)
{
  if (!c)
    return;
  if (c->refCount > 1)
  {
    c->refCount--;
    return;
  }
  for (uint8_t i = 0; i < P156_MAX_CONN; ++i)
    if (p156_conns[i] == c)
      p156_conns[i] = nullptr;
  delete c;
}

uint16_t p156_nextTid(P156_Conn *c)
{
  c->tid = (c->tid >= 65535) ? 1 : c->tid + 1;
  return c->tid;
}

// Verbindung für eine Anfrage belegen. false = ein anderer Task fragt gerade (oder ist zuerst dran)
bool p156_acquire(P156_Instance *inst)
{
  P156_Conn *c = inst->conn;
  const taskIndex_t me = inst->taskIndex;
  if ((c->owner != INVALID_TASK_INDEX) && (c->owner != me))
  {
    c->waiting = me;
    return false;
  }
  if ((c->owner == INVALID_TASK_INDEX) && (c->waiting != INVALID_TASK_INDEX) && (c->waiting != me) && (c->lastOwner == me))
    return false; // der wartende Task kommt zuerst dran
  c->owner = me;
  if (c->waiting == me)
    c->waiting = INVALID_TASK_INDEX;
  return true;
}

// Verbindung freigeben (keine Anfrage dieses Tasks mehr unterwegs)
void p156_release(P156_Instance *inst)
{
  P156_Conn *c = inst->conn;
  if (c && (c->owner == inst->taskIndex))
  {
    c->owner = INVALID_TASK_INDEX;
    c->lastOwner = inst->taskIndex;
  }
}

P156_Instance::~P156_Instance()
{
  if (conn)
  {
    if (conn->owner == taskIndex)
      conn->owner = INVALID_TASK_INDEX;
    if (conn->waiting == taskIndex)
      conn->waiting = INVALID_TASK_INDEX;
    p156_putConn(conn);
    conn = nullptr;
  }
}

// Ergebnis von p156_connect()
#define P156_CONN_OK 1      // verbunden, Anfrage kann gesendet werden
#define P156_CONN_BUSY 0    // Aufbau läuft – im nächsten Takt erneut prüfen
#define P156_CONN_FAIL -1   // Aufbau eben fehlgeschlagen
#define P156_CONN_WAIT -2   // Wartezeit nach Fehlversuch oder kein Netzwerk
#define P156_CONNECT_TIMEOUT 1000
#define P156_BACKOFF_MAX 30000

bool p156_sendRequest(P156_Instance *inst, uint8_t qIdx);
uint8_t p156_parseBlock(P156_Instance *inst, uint8_t &exCode);
void p156_buildBlock(P156_Instance *inst, uint8_t qIdx);
void p156_deviceLost(P156_Instance *inst);
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

    {
      // Status seit dem Start des Tasks
      P156_Instance *inst = static_cast<P156_Instance *>(getPluginTaskData(event->TaskIndex));
      if (inst)
      {
        addFormSubHeader(F("Status"));
        addRowLabel(F("Leseanfragen"));
        addHtmlInt(inst->statRequests);
        addRowLabel(F("Gelesene Werte"));
        addHtmlInt(inst->statValues);
        addRowLabel(F("Blocklesen"));
        addHtml(inst->noBlock ? F("aus (Geraet lehnt Bloecke ab, Werte einzeln)") : F("an"));
        addRowLabel(F("Neuverbindungen"));
        addHtmlInt(inst->reconnectCount);
        if (inst->conn)
        {
          addRowLabel(F("Verbindung"));
          addHtml(inst->conn->refCount > 1 ? strformat(F("gemeinsam mit %d weiteren Task(s)"), inst->conn->refCount - 1) : String(F("eigene")));
        }
      }
    }

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
    initPluginTaskData(event->TaskIndex, new (std::nothrow) P156_Instance());
    P156_Instance *inst = static_cast<P156_Instance *>(getPluginTaskData(event->TaskIndex));
    if (!inst)
      break;

    IPAddress ipAddr((uint32_t)P156_IP_LONG);
    String ipStr = ipAddr.toString();
    if (ipStr == F("0.0.0.0"))
      ipStr = F("192.168.1.1");
    inst->taskIndex = event->TaskIndex;
    inst->conn = p156_getConn(ipStr.c_str(), (P156_PORT > 0) ? (uint16_t)P156_PORT : 502);
    if (!inst->conn)
    {
      clearPluginTaskData(event->TaskIndex);
      break;
    }
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
      inst->gotValue[i] = false;
      inst->initVal[i] = UserVar.getFloat(event->TaskIndex, i);
    }
    const uint32_t def[] = {200, 1000, 10000, 60000};
    for (uint8_t g = 0; g < P156_NR_PAUSE_GROUPS; ++g)
    {
      inst->pauseMs[g] = p156_getPauseMs(g, P156_PAUSES);
      if (inst->pauseMs[g] < 50)
        inst->pauseMs[g] = def[g];
    }
    // Alle Werte gleich nach dem Start einmal abfragen, nicht erst nach Ablauf ihrer Pausenzeit
    inst->initTime = millis();
    for (uint8_t i = 0; i < P156_NR_OUTPUT_VALUES; ++i)
      inst->lastSend[i] = inst->initTime - inst->pauseMs[inst->pauseGroup[i]] - 1;
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
      log += inst->conn->ip;
      log += F(" Port=");
      log += inst->conn->port;
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
      p156_release(inst); // Verbindung bleibt für andere Tasks offen, wird mit der letzten Nutzung geschlossen
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
      // Nach dem Start erst senden, wenn jeder aktive Lesewert einmal echt gelesen wurde
      // (sonst gingen z. B. Zählerstände kurz als 0 an die Controller).
      // Sicherheitsnetz: nach 2 Minuten wieder senden, auch wenn ein Register nie antwortet.
      if (timePassedSince(inst->initTime) < 120000)
      {
        for (uint8_t i = 0; i < P156_NR_OUTPUT_VALUES; ++i)
        {
          if ((inst->mode[i] == P156_MODE_READ) && (inst->regAddr[i] != 0) && !inst->gotValue[i])
            return false;
        }
      }
      const bool lost = (inst->conn->connAttempts >= 3);
      for (uint8_t i = 0; i < P156_NR_OUTPUT_VALUES; ++i)
      {
        if (inst->mode[i] == P156_MODE_READ) // Schreibwerte behalten den gesetzten Taskwert
          UserVar.setFloat(event->TaskIndex, i, lost ? 0.0f : inst->values[i]);
      }
      // Gerät seit mehreren Versuchen nicht erreichbar: Lesewerte stehen lokal auf 0 (Rules sehen
      // "keine Leistung"), an die Controller wird aber nichts gesendet - eine 0 würde dort
      // Zählerstände und Auswertungen verfälschen. Mit der ersten gültigen Antwort geht es weiter.
      if (lost)
        return false;
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
          inst->conn->connAttempts = 0; // gültige Antwort – Verbindung ist in Ordnung
        p156_writeResult(inst, inst->activeWrite, (res == 1) ? 0 : ((res == 2) ? exCode : P156_WERR_TIMEOUT));
      }
      else if (timePassedSince(inst->requestTime) > 500)
      {
        inst->pendingQuery = P156_PENDING_NONE;
        inst->conn->client.clear();
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
      if ((conn != P156_CONN_BUSY) && ((conn != P156_CONN_OK) || p156_acquire(inst)))
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
      uint8_t exCode = 0;
      const uint8_t res = p156_parseBlock(inst, exCode);
      if (res == 1)
      {
        // alle Werte des Blocks sind frisch gelesen
        inst->pendingQuery = 255;
        inst->errorCount = 0;
        inst->conn->connAttempts = 0; // gültige Antwort – Verbindung ist in Ordnung
        const uint32_t now = millis();
        for (uint8_t i = 0; i < P156_NR_OUTPUT_VALUES; ++i)
        {
          if (inst->blockMask & (1 << i))
          {
            inst->gotValue[i] = true;
            inst->lastSend[i] = now;
            inst->statValues++;
          }
        }
        inst->queryIndex = (qIdx + 1) % P156_NR_OUTPUT_VALUES;
      }
      else if (res == 2)
      {
        // Gerät antwortet mit Modbus-Exception
        inst->pendingQuery = 255;
        inst->errorCount = 0;
        inst->conn->connAttempts = 0; // das Gerät antwortet – Verbindung ist in Ordnung
        if (!inst->noBlock && (inst->blockMask & ~(1 << qIdx)))
        {
          // Block abgelehnt (z. B. nicht belegte Register in der Lücke): ab jetzt einzeln lesen, sofort erneut
          inst->noBlock = true;
          if (loglevelActiveFor(LOG_LEVEL_INFO))
            addLogMove(LOG_LEVEL_INFO, strformat(F("P156: Geraet lehnt Blocklesen ab (Exception %d), lese Werte einzeln"), exCode));
        }
        else
        {
          inst->lastSend[qIdx] = millis();
          inst->queryIndex = (qIdx + 1) % P156_NR_OUTPUT_VALUES;
          if (!(inst->exLogged & (1 << qIdx)) && loglevelActiveFor(LOG_LEVEL_ERROR))
          {
            inst->exLogged |= (1 << qIdx);
            addLogMove(LOG_LEVEL_ERROR, strformat(F("P156: Wert %d (Register %u): Exception %d"), qIdx + 1, (unsigned)inst->regAddr[qIdx], exCode));
          }
        }
      }
      else if ((res == 3) || (timePassedSince(inst->requestTime) > 500))
      {
        // ungültige Antwort oder Timeout — give up, advance to next query
        inst->pendingQuery = 255;
        inst->hdrRead = false;
        inst->conn->client.clear();
        inst->lastSend[qIdx] = millis();
        inst->queryIndex = (qIdx + 1) % P156_NR_OUTPUT_VALUES;
        inst->errorCount++;
        if (inst->errorCount > 10)
          p156_deviceLost(inst);
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
          if ((conn == P156_CONN_OK) && !p156_acquire(inst))
            break; // anderer Task dieses Geräts fragt gerade – im nächsten Takt erneut
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
          p156_buildBlock(inst, qIdx);
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
              p156_deviceLost(inst);
          }
          break;
        }
      }
    }
    if (inst->pendingQuery == P156_PENDING_NONE)
      p156_release(inst);
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
  if (inst->conn->connFd >= 0)
  {
    lwip_close(inst->conn->connFd);
    inst->conn->connFd = -1;
  }
#endif
  if (inst->conn->connAttempts >= 3)
  {
    // Gerät seit mehreren Versuchen nicht erreichbar: keine alten Werte weitermelden
    for (uint8_t i = 0; i < P156_NR_OUTPUT_VALUES; ++i)
      inst->values[i] = 0.0f;
  }
  inst->reconnectCount++;
  if (loglevelActiveFor(LOG_LEVEL_INFO))
  {
    String l = F("P156: connect ");
    l += inst->conn->ip;
    l += ':';
    l += inst->conn->port;
    l += F(" failed (");
    l += reason;
    if (err != 0)
    {
      l += ' ';
      l += err;
    }
    l += F("), next try in ");
    l += p156_backoffMs(inst->conn->connAttempts) / 1000;
    l += 's';
    addLogMove(LOG_LEVEL_INFO, l);
  }
}

// Nicht-blockierender Verbindungsaufbau, wird jeden Takt aufgerufen solange eine Anfrage ansteht.
// Rückgabe: P156_CONN_OK / _BUSY / _FAIL / _WAIT (siehe oben)
int8_t p156_connect(P156_Instance *inst)
{
#ifdef ESP32
  if (inst->conn->connFd >= 0)
  {
    // Aufbau läuft: ohne zu warten prüfen, ob der Socket fertig ist
    fd_set wset;
    FD_ZERO(&wset);
    FD_SET(inst->conn->connFd, &wset);
    struct timeval tv = {0, 0};
    const int res = lwip_select(inst->conn->connFd + 1, nullptr, &wset, nullptr, &tv);
    if (res == 0)
    {
      if (timePassedSince(inst->conn->connStart) < P156_CONNECT_TIMEOUT)
        return P156_CONN_BUSY;
      p156_connectFailed(inst, F("timeout"), 0);
      return P156_CONN_FAIL;
    }
    int sockErr = 0;
    socklen_t len = sizeof(sockErr);
    if (res < 0)
      sockErr = errno;
    else if (lwip_getsockopt(inst->conn->connFd, SOL_SOCKET, SO_ERROR, &sockErr, &len) < 0)
      sockErr = errno;
    if (sockErr != 0)
    {
      p156_connectFailed(inst, F("error"), sockErr);
      return P156_CONN_FAIL;
    }
    // Verbunden: Socket wie bei NetworkClient::connect() wieder blockierend schalten und übergeben
    const int fd = inst->conn->connFd;
    inst->conn->connFd = -1;
    lwip_fcntl(fd, F_SETFL, lwip_fcntl(fd, F_GETFL, 0) & ~O_NONBLOCK);
    inst->conn->client = WiFiClient(fd);
    inst->conn->client.setConnectionTimeout(500); // Lese-/Schreib-Timeout wie bisher
    inst->rxLen = 0;
    return P156_CONN_OK;
  }
#endif

  if (inst->conn->client.connected())
    return P156_CONN_OK;
  // Kein Versuch ohne Netzwerk (verhindert Absturz beim Booten)
  if (!NetworkConnected(0))
    return P156_CONN_WAIT;
  if ((inst->conn->connAttempts > 0) && !timeOutReached(inst->conn->nextConnectAllowed))
    return P156_CONN_WAIT;

  inst->conn->client.stop(); // Reste einer abgebrochenen Verbindung freigeben
  if (inst->conn->connAttempts < 255)
    inst->conn->connAttempts++;
  inst->conn->nextConnectAllowed = millis() + p156_backoffMs(inst->conn->connAttempts);

#ifdef ESP32
  const uint32_t addr = ipaddr_addr(inst->conn->ip);
  if (addr == IPADDR_NONE)
  {
    p156_connectFailed(inst, F("bad IP"), 0);
    return P156_CONN_FAIL;
  }
  struct sockaddr_in sa = {};
  sa.sin_family = AF_INET;
  sa.sin_port = htons(inst->conn->port);
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
  inst->conn->connFd = fd;
  inst->conn->connStart = millis();
  return P156_CONN_BUSY; // Ergebnis im nächsten Takt per select() prüfen
#else
  if (!inst->conn->client.connect(inst->conn->ip, inst->conn->port, 500)) // ESP8266: blockierend, max. 500 ms
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
  if (!inst->conn->client.connected())
    return false;
  inst->sendCount = p156_nextTid(inst->conn); // Transaktions-ID je Verbindung
  (void)qIdx; // Register und Anzahl stehen in blockStart/blockCount (p156_buildBlock)
  inst->hdrRead = false;
  byte req[12];
  req[0] = (uint8_t)(inst->sendCount >> 8);
  req[1] = (uint8_t)inst->sendCount;
  req[2] = 0;
  req[3] = 0;
  req[4] = 0;
  req[5] = 6;
  req[6] = inst->unitId;
  req[7] = inst->blockFc;
  req[8] = (uint8_t)(inst->blockStart >> 8);
  req[9] = (uint8_t)inst->blockStart;
  req[10] = 0;
  req[11] = inst->blockCount;
  inst->requestTime = millis(); // record send time for response timeout
  inst->statRequests++;
  inst->conn->client.write(req, sizeof(req));
  return true;
}

// Anzahl Register eines Datentyps
static uint8_t p156_regCount(uint8_t type)
{
  return (type >= P156_TYPE_U32) ? 2 : 1;
}

#define P156_BLOCK_MAX 32 // höchstens so viele Register in einer Leseanfrage

// Leseanfrage für Wert qIdx zusammenstellen: alle anderen Lesewerte mit demselben Funktionscode,
// die zusammen mit ihm in höchstens P156_BLOCK_MAX Registern liegen, werden mitgelesen.
void p156_buildBlock(P156_Instance *inst, uint8_t qIdx)
{
  uint32_t start = inst->regAddr[qIdx];
  uint32_t end = start + p156_regCount(inst->dataType[qIdx]) - 1;
  uint8_t mask = (1 << qIdx);
  if (!inst->noBlock)
  {
    for (uint8_t j = 0; j < P156_NR_OUTPUT_VALUES; ++j)
    {
      if ((j == qIdx) || (inst->regAddr[j] == 0) || (inst->mode[j] != P156_MODE_READ) || (inst->readFc[j] != inst->readFc[qIdx]))
        continue;
      const uint32_t s2 = (inst->regAddr[j] < start) ? inst->regAddr[j] : start;
      const uint32_t eJ = inst->regAddr[j] + p156_regCount(inst->dataType[j]) - 1;
      const uint32_t e2 = (eJ > end) ? eJ : end;
      if ((e2 - s2 + 1) <= P156_BLOCK_MAX)
      {
        start = s2;
        end = e2;
        mask |= (1 << j);
      }
    }
  }
  inst->blockStart = (uint16_t)start;
  inst->blockCount = (uint8_t)(end - start + 1);
  inst->blockFc = inst->readFc[qIdx];
  inst->blockMask = mask;
}

// Registerinhalt (2 oder 4 Bytes ab p) nach Datentyp umrechnen
static float p156_decode(uint8_t type, const uint8_t *p)
{
  const uint8_t h1 = p[0], l1 = p[1];
  const uint8_t h2 = (type >= P156_TYPE_U32) ? p[2] : 0, l2 = (type >= P156_TYPE_U32) ? p[3] : 0;
  float v = 0.0f;
  switch (type)
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
  case P156_TYPE_U32WS: // Word-Swapped: h2/l2=HighWord, h1/l1=LowWord (Varta)
    v = (uint32_t)(((uint32_t)h2 << 24) | ((uint32_t)l2 << 16) | ((uint32_t)h1 << 8) | (uint32_t)l1);
    break;
  case P156_TYPE_S32WS:
    v = (int32_t)(((uint32_t)h2 << 24) | ((uint32_t)l2 << 16) | ((uint32_t)h1 << 8) | (uint32_t)l1);
    break;
  }
  return v;
}

// Antwort auf die laufende Leseanfrage auswerten (nicht blockierend).
// Rückgabe: 0 = noch nicht vollständig, 1 = OK (Werte übernommen), 2 = Modbus-Exception (exCode), 3 = ungültig
uint8_t p156_parseBlock(P156_Instance *inst, uint8_t &exCode)
{
  if (!inst)
    return 3;
  if (!inst->hdrRead)
  {
    // Kopf: Transaktions-ID (2), Protokoll (2), Länge (2), Unit (1), Funktionscode (1), Bytezahl bzw. Exception-Code (1)
    if (inst->conn->client.available() < 9)
      return 0;
    for (uint8_t i = 0; i < 9; ++i)
      inst->hdr[i] = inst->conn->client.read();
    if ((uint16_t)((inst->hdr[0] << 8) | inst->hdr[1]) != inst->sendCount)
    {
      inst->conn->client.clear(); // Antwort auf eine ältere Anfrage
      return 3;
    }
    if (inst->hdr[7] & 0x80)
    {
      exCode = inst->hdr[8];
      return 2;
    }
    if ((inst->hdr[7] != inst->blockFc) || (inst->hdr[8] != inst->blockCount * 2))
    {
      inst->conn->client.clear();
      return 3;
    }
    inst->hdrRead = true;
  }
  const uint8_t n = inst->hdr[8];
  if (inst->conn->client.available() < n)
    return 0;
  uint8_t data[2 * P156_BLOCK_MAX];
  for (uint8_t i = 0; i < n; ++i)
    data[i] = inst->conn->client.read();
  inst->hdrRead = false;
  for (uint8_t i = 0; i < P156_NR_OUTPUT_VALUES; ++i)
  {
    if (!(inst->blockMask & (1 << i)))
      continue;
    const uint16_t off = (uint16_t)(inst->regAddr[i] - inst->blockStart) * 2;
    if (off + 2u * p156_regCount(inst->dataType[i]) > n)
      return 3;
    inst->values[i] = p156_decode(inst->dataType[i], &data[off]);
  }
  return 1;
}

// Gerät antwortet seit mehreren Anfragen nicht: Verbindung neu aufbauen. Die Lesewerte stehen lokal auf 0,
// an die Controller wird bis zur nächsten gültigen Antwort nichts gesendet (wie nach 3 Fehlversuchen beim Verbinden).
void p156_deviceLost(P156_Instance *inst)
{
  inst->errorCount = 0;
  inst->hdrRead = false;
  inst->conn->client.clear();
  inst->conn->client.stop();
  for (uint8_t i = 0; i < P156_NR_OUTPUT_VALUES; ++i)
    inst->values[i] = 0.0f;
  if (inst->conn->connAttempts < 3)
    inst->conn->connAttempts = 3;
  inst->reconnectCount++;
  p156_release(inst);
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
  if (!inst->conn->client.connected())
    return false;
  inst->sendCount = p156_nextTid(inst->conn); // Transaktions-ID je Verbindung

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

  inst->conn->client.clear(); // verspätete Bytes einer abgebrochenen Anfrage verwerfen
  inst->rxLen = 0;
  inst->requestTime = millis();
  return inst->conn->client.write(req, len) == len;
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
    if (inst->conn->client.available() <= 0)
      return 0;
    const int n = inst->conn->client.read(inst->rxBuf + inst->rxLen, need - inst->rxLen);
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