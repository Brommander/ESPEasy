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
//    [2] Old common read function code (0=Func03, 1=Func04). Only used by old configurations:
//        "Read 03" (function 0) reads with 04 if this is 1. It is set to 0 on save.
//    [3-6] Data type query 0-3 (0=U16,1=S16,2=U32,3=S32,4=Float)
//    [7] Pause groups packed (bits 0-1=Q0, 2-3=Q1, 4-5=Q2, 6-7=Q3)
//        Function per value (bits 8-9=Q0, 10-11=Q1, 12-13=Q2, 14-15=Q3):
//          0=03 read, 1=06 write, 2=16 write, 3=04 read
//  PCONFIG_LONG:
//    [0] IP as uint32
//    [1] reg0(bits0-15) | reg1(bits16-31)
//    [2] reg2(bits0-15) | reg3(bits16-31)
//    [3] pause1(8b,x100ms)|pause2(8b,x100ms)|pause3(8b,x1000ms)|pause4(8b,x1000ms)
//  getPluginCustomArgName (index 0..16):
//    [0]    = IP
//    [1..4] = Register address query 0..3
//    [5..8] = Pause time group 1..4 (ms)
//    [9..12]= Pause group query 0..3
//    [13..16]= Function query 0..3
//
//  Function code per value: 03/04 = read, 06/16 = write cyclically.
//  Cyclic writing (function code 06 or 16 for a value):
//    The task value is written to the register in the interval of its pause group (data type as configured).
//    Set the value: [<TaskName>].MBSet,<value nr 1-4>,<value>  (rules, HTTP /control?cmd=..., MQTT <sysname>/cmd)
//    or TaskValueSet (only if "Allow TaskValueSet on all plugins" is enabled in the Advanced Settings).
//    After start nothing is written until a value has been set.
//    Rules event WriteOK only when the value changed or after an error, WriteError on every error.
//
//  Writing (command, e.g. from rules, HTTP /control?cmd=... or MQTT <sysname>/cmd):
//    [<TaskName>].MBWrite,<register>,<value>[,<data type>[,<FC>]]
//      Data type: u16, s16, u32, s32, float, u32ws, s32ws (or 0..6).
//                 Default: data type of the query with this register, otherwise u16.
//      FC:        6 (Write Single Register) or 16 (Write Multiple Registers).
//                 Default: 6 for 16-bit types, 16 for 32-bit types.
//    Without [<TaskName>]. the command applies to the first active P156 task.
//    Write jobs have priority over reading (queue with 4 entries).
//    Result as rules event:
//      <TaskName>#WriteOK=<register>,<value>
//      <TaskName>#WriteError=<register>,<code>
//        Code 1..11 = Modbus exception, 253 = queue full,
//        254 = no connection, 255 = no/invalid response (timeout 500 ms)
//
//  Connecting (ESP32): non-blocking. connect() returns at once, the result is checked in the
//  following 100 ms cycles without waiting (timeout 1 s). If the connect fails or the device then
//  sends no valid response, the next attempt waits 1, 2, 4, 8, 16, then 30 s.
//  So a device that is unreachable or busy (e.g. one that allows only one Modbus client) does not slow down the ESP.
//  ESP8266: blocking connect (max. 500 ms), but with the same wait times.
//  Shared connection: all P156 tasks with the same IP and port use one TCP connection
//  (many devices allow only very few Modbus clients). Only one request per connection is on the
//  wire; if another task is waiting, it is served next. Transaction IDs are assigned per connection.
//  Block reading: when a read value is due, all other read values with the same function code that
//  lie within 32 registers together are read in the same request (they count as freshly read).
//  Registers of other tasks of the same connection (same unit ID, same function code) are read as
//  well and handed to these tasks - several tasks for one device need no additional requests.
//  If the device answers a block with an exception, this task reads its values one by one again.
//  After 3 failed attempts in a row the read values are 0 locally (for rules); nothing is sent to
//  the controllers until the next valid response.
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
#define P156_MODE_RAW_READ04 3 // stored only; internally P156_MODE_READ with readFc = 4

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
  return (uint8_t)((((uint16_t)pg) >> (8 + qIdx * 2)) & 0x03); // stored raw value 0..3
}

// Selection in the form: 0=03 read, 1=04 read, 2=06 write, 3=16 write
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
  return (oldFunc == 1) ? 1 : 0; // old configuration with common FC 04
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
  double value = 0.0; // for event/log only
  uint8_t slot = 255; // value index for cyclic writing, 255 = command MBWrite
};

#define P156_CONN_MEMBERS 8 // max. number of tasks sharing one connection

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
  uint8_t readFc[P156_NR_OUTPUT_VALUES] = {3, 3, 3, 3}; // read function code per value (3 or 4)
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
  float lastOk[P156_NR_OUTPUT_VALUES] = {NAN, NAN, NAN, NAN}; // last value successfully written cyclically
  uint8_t lastErr[P156_NR_OUTPUT_VALUES] = {0, 0, 0, 0};      // last reported error code (0 = none)
  bool armed[P156_NR_OUTPUT_VALUES] = {false, false, false, false}; // write value set since start
  bool gotValue[P156_NR_OUTPUT_VALUES] = {false, false, false, false}; // read value received at least once since start
  uint32_t initTime = 0;                                               // time of PLUGIN_INIT
  float initVal[P156_NR_OUTPUT_VALUES] = {0, 0, 0, 0};              // task value at start
  struct P156_Conn *conn = nullptr; // shared connection to the device (IP + port)
  uint8_t connSlot = 255;           // slot in conn->members
  uint32_t connGen = 0;             // id of this registration at the connection
  uint8_t otherMask[P156_CONN_MEMBERS] = {}; // values of other tasks in the current request (per slot)
  uint32_t otherGen[P156_CONN_MEMBERS] = {};  // id of the other task when the request was built
  // Block reading: one request reads all read values that are close together
  uint16_t blockStart = 0;         // first register of the current read request
  uint8_t blockCount = 0;          // number of registers
  uint8_t blockFc = 3;             // function code 3 or 4
  uint8_t blockMask = 0;           // bit i = value i is part of the request
  bool noBlock = false;            // device rejects blocks -> read values one by one
  bool hdrRead = false;            // response header already read
  uint8_t hdr[9] = {};             // MBAP header + function code + byte count/exception
  uint8_t exLogged = 0;            // bit i = exception for value i already logged
  uint32_t statRequests = 0;       // read requests since start
  uint32_t statValues = 0;         // values read by these requests

  ~P156_Instance();
};

// ---- Shared connections: all tasks with the same IP and port use one TCP connection ----
// Only one request per connection is on the wire (owner); a waiting task is served next.
struct P156_Conn
{
  char ip[16] = {};
  uint16_t port = 502;
  WiFiClient client;
  int connFd = -1;                  // ESP32: socket being connected (-1 = none)
  uint32_t connStart = 0;           // start of the current connect
  uint32_t nextConnectAllowed = 0;  // earliest next attempt (only valid if connAttempts > 0)
  uint8_t connAttempts = 0;         // connect attempts since the last valid response
  uint16_t tid = 0;                 // last transaction ID of this connection
  uint8_t refCount = 0;             // number of tasks using this connection
  taskIndex_t owner = INVALID_TASK_INDEX;     // task with a pending request
  taskIndex_t lastOwner = INVALID_TASK_INDEX; // task of the last request
  taskIndex_t waiting = INVALID_TASK_INDEX;   // task waiting for the connection
  P156_Instance *members[P156_CONN_MEMBERS] = {}; // tasks of this connection (for shared blocks)
  uint32_t memberGen[P156_CONN_MEMBERS] = {};
  uint32_t gen = 0;

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
void p156_joinConn(P156_Instance *inst);
bool p156_blockHasOthers(P156_Instance *inst, uint8_t qIdx);
static P156_Conn *p156_conns[P156_MAX_CONN] = {};

// Get the connection to ip:port (share an existing one or create a new one)
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

// Stop using the connection; the last user closes it
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

// Occupy the connection for a request. false = another task is requesting (or goes first)
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
    return false; // the waiting task goes first
  c->owner = me;
  if (c->waiting == me)
    c->waiting = INVALID_TASK_INDEX;
  return true;
}

// Release the connection (no request of this task pending)
void p156_release(P156_Instance *inst)
{
  P156_Conn *c = inst->conn;
  if (c && (c->owner == inst->taskIndex))
  {
    c->owner = INVALID_TASK_INDEX;
    c->lastOwner = inst->taskIndex;
  }
}

// Register the task at the connection (so other tasks can read its registers as well)
void p156_joinConn(P156_Instance *inst)
{
  P156_Conn *c = inst->conn;
  for (uint8_t i = 0; i < P156_CONN_MEMBERS; ++i)
  {
    if (c->members[i] == nullptr)
    {
      c->members[i] = inst;
      c->memberGen[i] = ++c->gen;
      inst->connSlot = i;
      inst->connGen = c->memberGen[i];
      return;
    }
  }
}

P156_Instance::~P156_Instance()
{
  if (conn)
  {
    if ((connSlot < P156_CONN_MEMBERS) && (conn->members[connSlot] == this))
    {
      conn->members[connSlot] = nullptr;
      conn->memberGen[connSlot] = 0;
    }
    if (conn->owner == taskIndex)
      conn->owner = INVALID_TASK_INDEX;
    if (conn->waiting == taskIndex)
      conn->waiting = INVALID_TASK_INDEX;
    p156_putConn(conn);
    conn = nullptr;
  }
}

// Result of p156_connect()
#define P156_CONN_OK 1      // connected, request can be sent
#define P156_CONN_BUSY 0    // connecting - check again in the next cycle
#define P156_CONN_FAIL -1   // connect just failed
#define P156_CONN_WAIT -2   // waiting after a failed attempt, or no network
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

    // ---- Connection ----
    addFormHeader(F("Connection"));
    addFormTextBox(F("IPv4 Address"), getPluginCustomArgName(P156_ARG_IP), ipStr, 15);
    addFormNote(F("IP address of the Modbus device"));
    addFormNumericBox(F("Port"), P156_PORT_LBL, P156_PORT, 1, 65535);
    addFormNote(F("Typical: 502 or 1502"));
    addFormNumericBox(F("Unit ID"), P156_UID_LBL, P156_UID, 1, 255);
    addFormNote(F("Typical: 1-255"));

    // ---- Pause groups ----
    addFormHeader(F("Pause Groups"));
    addFormNote(F("Group 1/2: steps of 100 ms, group 3/4: steps of 1000 ms"));
    for (uint8_t g = 0; g < P156_NR_PAUSE_GROUPS; ++g)
    {
      char buf[10];
      snprintf(buf, sizeof(buf), "%lu", (unsigned long)pau[g]);
      addFormTextBox(String(F("Group ")) + (g + 1) + F(" (ms)"),
                     getPluginCustomArgName(P156_ARG_PAU1 + g), buf, 8);
    }

    // ---- Values ----
    addFormHeader(F("Values"));

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
        F("Group 1"),
        F("Group 2"),
        F("Group 3"),
        F("Group 4"),
    };
    const __FlashStringHelper *modeOpts[] = {
        F("03 - Read Holding Registers"),
        F("04 - Read Input Registers"),
        F("06 - Write Single Register (16 bit)"),
        F("16 - Write Multiple Registers (16/32 bit, float)"),
    };

    for (uint8_t i = 0; i < P156_NR_OUTPUT_VALUES; ++i)
    {
      addFormSubHeader(String(F("---- Value ")) + (i + 1) + F(" ----"));

      char buf[8];
      snprintf(buf, sizeof(buf), "%u", p156_getReg(i, P156_REGS01, P156_REGS23));
      addFormTextBox(F("Register Address"), getPluginCustomArgName(P156_ARG_REG0 + i), buf, 6);
      addFormNote(F("PDU address 1-65535 (0 = value not used). 4xxxxx notation: address minus 400001"));

      {
        FormSelectorOptions msel(NR_ELEMENTS(modeOpts), modeOpts);
        msel.addFormSelector(F("Function Code"),
                             getPluginCustomArgName(P156_ARG_MODE0 + i),
                             p156_rawToUi(p156_getMode(i, P156_PGROUPS), P156_FUNC));
      }

      {
        FormSelectorOptions tsel(NR_ELEMENTS(dtOpts), dtOpts);
        switch (i)
        {
        case 0:
          tsel.addFormSelector(F("Data Type"), P156_DT0_LBL, P156_DT0);
          break;
        case 1:
          tsel.addFormSelector(F("Data Type"), P156_DT1_LBL, P156_DT1);
          break;
        case 2:
          tsel.addFormSelector(F("Data Type"), P156_DT2_LBL, P156_DT2);
          break;
        case 3:
          tsel.addFormSelector(F("Data Type"), P156_DT3_LBL, P156_DT3);
          break;
        }
      }

      {
        FormSelectorOptions gsel(NR_ELEMENTS(grpOpts), grpOpts);
        gsel.addFormSelector(F("Pause Group"),
                             getPluginCustomArgName(P156_ARG_GRP0 + i),
                             p156_getGroup(i, P156_PGROUPS));
      }
    }
    addFormNote(F("Function code 06/16: the task value is written to the register in the interval of the pause group. Set the value with "
                  "[TaskName].MBSet,ValueNr,Value (e.g. from rules or /control?cmd=...). After start nothing is "
                  "written until a value has been set. 06 works with 16-bit types only, otherwise 16 is used."));

    {
      // Status since the start of the task
      P156_Instance *inst = static_cast<P156_Instance *>(getPluginTaskData(event->TaskIndex));
      if (inst)
      {
        addFormSubHeader(F("Status"));
        addRowLabel(F("Read requests"));
        addHtmlInt(inst->statRequests);
        addRowLabel(F("Values read"));
        addHtmlInt(inst->statValues);
        addRowLabel(F("Block reading"));
        addHtml(inst->noBlock ? F("off (device rejects blocks, values are read one by one)") : F("on"));
        addRowLabel(F("Reconnects"));
        addHtmlInt(inst->reconnectCount);
        if (inst->conn)
        {
          addRowLabel(F("Connection"));
          addHtml(inst->conn->refCount > 1 ? strformat(F("shared with %d other task(s)"), inst->conn->refCount - 1) : String(F("own")));
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
    P156_FUNC = 0; // read function code is now stored per value (see below)
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
        inst->mode[i] = P156_MODE_FC16; // 32 bit / float do not fit into FC 06
      inst->lastOk[i] = NAN;
      inst->lastErr[i] = 0;
      // Write value: write only after a value was set after start (MBSet or changed task value)
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
    // Request all values right after start, not only after their pause time
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
    p156_joinConn(inst);
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
      p156_release(inst); // connection stays open for other tasks, the last user closes it
    }
    break;
  }

  case PLUGIN_WRITE:
  {
    const String command = parseString(string, 1);
    if (equals(command, F("mbset")))
    {
      // MBSet,<value nr 1-4>,<value>: set the write value (written in the interval of the pause group)
      success = true;
      P156_Instance *inst = static_cast<P156_Instance *>(getPluginTaskData(event->TaskIndex));
      int32_t nr = 0;
      ESPEASY_RULES_FLOAT_TYPE d = 0;
      String err;
      if (!inst || !inst->myInit)
        err = F("task not initialised");
      else if (!validIntFromString(parseString(string, 2), nr) || (nr < 1) || (nr > P156_NR_OUTPUT_VALUES))
        err = F("value number must be 1-4");
      else if (inst->mode[nr - 1] == P156_MODE_READ)
        err = F("value is set to read");
      else if (!validDoubleFromString(parseStringKeepCase(string, 3), d))
        err = F("invalid value");
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
    success = true; // command belongs to P156 - errors are logged, not passed on to other plugins
    P156_Instance *inst = static_cast<P156_Instance *>(getPluginTaskData(event->TaskIndex));
    String err;
    if (!inst || !inst->myInit)
      err = F("task not initialised");
    else if (!p156_queueWrite(inst, string, err) && err.isEmpty())
      err = F("unknown error");
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
      // After start, send only when every active read value has really been read once
      // (otherwise e.g. energy counters would briefly be sent as 0 to the controllers).
      // Safety net: send again after 2 minutes, even if a register never answers.
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
        if (inst->mode[i] == P156_MODE_READ) // write values keep the task value that was set
          UserVar.setFloat(event->TaskIndex, i, lost ? 0.0f : inst->values[i]);
      }
      // Device not reachable for several attempts: read values are 0 locally (rules see
      // "no power"), but nothing is sent to the controllers - a 0 would corrupt
      // counters and statistics there. Sending resumes with the first valid response.
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
          inst->conn->connAttempts = 0; // valid response - connection is fine
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
      // (while connecting, the job stays in the queue;
      //  while waiting after failed attempts, it is finished with 254 at once)
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
        // all values of the block are freshly read
        inst->pendingQuery = 255;
        inst->errorCount = 0;
        inst->conn->connAttempts = 0; // valid response - connection is fine
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
        // device answers with a Modbus exception
        inst->pendingQuery = 255;
        inst->errorCount = 0;
        inst->conn->connAttempts = 0; // the device answers - connection is fine
        if (!inst->noBlock && p156_blockHasOthers(inst, qIdx))
        {
          // block rejected (e.g. undefined registers in a gap): read one by one from now on, retry at once
          inst->noBlock = true;
          if (loglevelActiveFor(LOG_LEVEL_INFO))
            addLogMove(LOG_LEVEL_INFO, strformat(F("P156: device rejects block reading (exception %d), reading values one by one"), exCode));
        }
        else
        {
          inst->lastSend[qIdx] = millis();
          inst->queryIndex = (qIdx + 1) % P156_NR_OUTPUT_VALUES;
          if (!(inst->exLogged & (1 << qIdx)) && loglevelActiveFor(LOG_LEVEL_ERROR))
          {
            inst->exLogged |= (1 << qIdx);
            addLogMove(LOG_LEVEL_ERROR, strformat(F("P156: value %d (register %u): exception %d"), qIdx + 1, (unsigned)inst->regAddr[qIdx], exCode));
          }
        }
      }
      else if ((res == 3) || (timePassedSince(inst->requestTime) > 500))
      {
        // invalid response or timeout — give up, advance to next query
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
            // Cyclic write: get and encode the task value (before connecting)
            const float v = UserVar.getFloat(event->TaskIndex, qIdx);
            if (!inst->armed[qIdx] && (v != inst->initVal[qIdx]) && !isnan(v))
              inst->armed[qIdx] = true; // task value was changed (e.g. TaskValueSet)
            String err;
            if (!inst->armed[qIdx] || isnan(v) || !p156_encodeValue(v, inst->dataType[qIdx], cyc, err))
            {
              if (!err.isEmpty() && loglevelActiveFor(LOG_LEVEL_ERROR))
              {
                String l = F("P156 write value ");
                l += qIdx + 1;
                l += F(": ");
                l += err;
                addLogMove(LOG_LEVEL_ERROR, l);
              }
              inst->lastSend[qIdx] = millis(); // no (valid) value yet - next round
              inst->queryIndex = (qIdx + 1) % P156_NR_OUTPUT_VALUES;
              break;
            }
            cyc.reg = (uint16_t)inst->regAddr[qIdx];
            cyc.fc = (inst->mode[qIdx] == P156_MODE_FC6) ? 6 : 16;
            cyc.slot = qIdx;
          }

          const int8_t conn = p156_connect(inst);
          if ((conn == P156_CONN_BUSY) || (conn == P156_CONN_WAIT))
            break; // connecting or waiting - do not block, retry later
          if ((conn == P156_CONN_OK) && !p156_acquire(inst))
            break; // another task of this device is requesting - retry in the next cycle
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

// Wait time after the n-th attempt without a valid response: 1, 2, 4, 8, 16, 30, 30 ... s
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
    // device not reachable for several attempts: do not report old values
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

// Non-blocking connect, called every cycle while a request is pending.
// Returns P156_CONN_OK / _BUSY / _FAIL / _WAIT (see above)
int8_t p156_connect(P156_Instance *inst)
{
#ifdef ESP32
  if (inst->conn->connFd >= 0)
  {
    // connecting: check without waiting whether the socket is ready
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
    // connected: switch the socket back to blocking (as NetworkClient::connect() does) and hand it over
    const int fd = inst->conn->connFd;
    inst->conn->connFd = -1;
    lwip_fcntl(fd, F_SETFL, lwip_fcntl(fd, F_GETFL, 0) & ~O_NONBLOCK);
    inst->conn->client = WiFiClient(fd);
    inst->conn->client.setConnectionTimeout(500); // read/write timeout
    inst->rxLen = 0;
    return P156_CONN_OK;
  }
#endif

  if (inst->conn->client.connected())
    return P156_CONN_OK;
  // no attempt without network (prevents a crash during boot)
  if (!NetworkConnected(0))
    return P156_CONN_WAIT;
  if ((inst->conn->connAttempts > 0) && !timeOutReached(inst->conn->nextConnectAllowed))
    return P156_CONN_WAIT;

  inst->conn->client.stop(); // free the remains of an aborted connection
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
  return P156_CONN_BUSY; // check the result with select() in the next cycle
#else
  if (!inst->conn->client.connect(inst->conn->ip, inst->conn->port, 500)) // ESP8266: blocking, max. 500 ms
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
  inst->sendCount = p156_nextTid(inst->conn); // transaction ID per connection
  (void)qIdx; // register and count are in blockStart/blockCount (p156_buildBlock)
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

// Number of registers of a data type
static uint8_t p156_regCount(uint8_t type)
{
  return (type >= P156_TYPE_U32) ? 2 : 1;
}

#define P156_BLOCK_MAX 32 // max. number of registers in one read request

// Build the read request for value qIdx: all other read values with the same function code
// that lie within P156_BLOCK_MAX registers together with it are read as well.
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
  // also read registers of other tasks of the same connection (same unit ID and function code)
  for (uint8_t k = 0; k < P156_CONN_MEMBERS; ++k)
  {
    inst->otherMask[k] = 0;
    P156_Instance *m = inst->conn->members[k];
    if (inst->noBlock || !m || (m == inst) || !m->myInit || m->noBlock || (m->unitId != inst->unitId))
      continue;
    for (uint8_t j = 0; j < P156_NR_OUTPUT_VALUES; ++j)
    {
      if ((m->regAddr[j] == 0) || (m->mode[j] != P156_MODE_READ) || (m->readFc[j] != inst->readFc[qIdx]))
        continue;
      const uint32_t s2 = (m->regAddr[j] < start) ? m->regAddr[j] : start;
      const uint32_t eJ = m->regAddr[j] + p156_regCount(m->dataType[j]) - 1;
      const uint32_t e2 = (eJ > end) ? eJ : end;
      if ((e2 - s2 + 1) <= P156_BLOCK_MAX)
      {
        start = s2;
        end = e2;
        inst->otherMask[k] |= (1 << j);
      }
    }
    inst->otherGen[k] = inst->conn->memberGen[k];
  }
  inst->blockStart = (uint16_t)start;
  inst->blockCount = (uint8_t)(end - start + 1);
  inst->blockFc = inst->readFc[qIdx];
  inst->blockMask = mask;
}

// does the current request contain values other than qIdx (own or of other tasks)?
bool p156_blockHasOthers(P156_Instance *inst, uint8_t qIdx)
{
  if (inst->blockMask & ~(1 << qIdx))
    return true;
  for (uint8_t k = 0; k < P156_CONN_MEMBERS; ++k)
    if (inst->otherMask[k])
      return true;
  return false;
}

// Convert register content (2 or 4 bytes at p) according to the data type
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

// Evaluate the response to the current read request (non-blocking).
// Returns 0 = not complete yet, 1 = OK (values taken over), 2 = Modbus exception (exCode), 3 = invalid
uint8_t p156_parseBlock(P156_Instance *inst, uint8_t &exCode)
{
  if (!inst)
    return 3;
  if (!inst->hdrRead)
  {
    // header: transaction ID (2), protocol (2), length (2), unit (1), function code (1), byte count or exception code (1)
    if (inst->conn->client.available() < 9)
      return 0;
    for (uint8_t i = 0; i < 9; ++i)
      inst->hdr[i] = inst->conn->client.read();
    if ((uint16_t)((inst->hdr[0] << 8) | inst->hdr[1]) != inst->sendCount)
    {
      inst->conn->client.clear(); // response to an older request
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
  // take over values of other tasks from the same response
  const uint32_t now = millis();
  for (uint8_t k = 0; k < P156_CONN_MEMBERS; ++k)
  {
    if (!inst->otherMask[k])
      continue;
    P156_Instance *m = inst->conn->members[k];
    if (!m || (inst->conn->memberGen[k] != inst->otherGen[k]) || !m->myInit)
      continue; // task was restarted in the meantime
    for (uint8_t j = 0; j < P156_NR_OUTPUT_VALUES; ++j)
    {
      if (!(inst->otherMask[k] & (1 << j)))
        continue;
      const uint16_t off = (uint16_t)(m->regAddr[j] - inst->blockStart) * 2;
      if (off + 2u * p156_regCount(m->dataType[j]) > n)
        continue;
      m->values[j] = p156_decode(m->dataType[j], &data[off]);
      m->gotValue[j] = true;
      m->lastSend[j] = now;
      m->statValues++;
    }
  }
  return 1;
}

// Device has not answered several requests: reconnect. The read values are 0 locally,
// nothing is sent to the controllers until the next valid response (as after 3 failed connect attempts).
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

// Data type from text: u16, s16, u32, s32, float, u32ws, s32ws or digit 0..6; -1 = invalid
static int p156_parseType(const String &s)
{
  if (s.isEmpty())
    return -2; // not given
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
  // MBWrite,<register>,<value>[,<data type>[,<FC>]]
  int32_t reg = 0;
  if (!validIntFromString(parseString(cmd, 2), reg) || (reg < 0) || (reg > 65535))
  {
    err = F("invalid register");
    return false;
  }
  ESPEASY_RULES_FLOAT_TYPE d = 0;
  if (!validDoubleFromString(parseStringKeepCase(cmd, 3), d))
  {
    err = F("invalid value");
    return false;
  }
  int type = p156_parseType(parseString(cmd, 4));
  if (type == -1)
  {
    err = F("invalid data type");
    return false;
  }
  if (type == -2)
  {
    // use the data type of the query with this register, otherwise U16
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
      err = F("FC must be 6 or 16");
      return false;
    }
    if ((fc == 6) && (job.nRegs != 1))
    {
      err = F("FC 6 only for 16-bit types");
      return false;
    }
    job.fc = (uint8_t)fc;
  }

  if (inst->writeCount >= P156_WRITE_QUEUE)
  {
    err = F("queue full");
    p156_writeResult(inst, job, P156_WERR_QUEUE_FULL);
    return false;
  }
  const uint8_t slot = (inst->writeHead + inst->writeCount) % P156_WRITE_QUEUE;
  inst->writeQueue[slot] = job;
  inst->writeCount++;
  return true;
}

// Convert a value into 1 or 2 register words (range is checked). Sets value, nRegs, words.
bool p156_encodeValue(double d, int type, P156_WriteJob &job, String &err)
{
  job.value = d;
  uint32_t u = 0;
  switch (type)
  {
  case P156_TYPE_U16:
    if ((d < 0) || (d > 65535))
    {
      err = F("value outside 0..65535");
      return false;
    }
    job.nRegs = 1;
    job.words[0] = (uint16_t)lround(d);
    break;
  case P156_TYPE_S16:
    if ((d < -32768) || (d > 32767))
    {
      err = F("value outside -32768..32767");
      return false;
    }
    job.nRegs = 1;
    job.words[0] = (uint16_t)(int16_t)lround(d);
    break;
  case P156_TYPE_U32:
  case P156_TYPE_U32WS:
    if ((d < 0) || (d > 4294967295.0))
    {
      err = F("value outside 0..4294967295");
      return false;
    }
    u = (uint32_t)llround(d);
    break;
  case P156_TYPE_S32:
  case P156_TYPE_S32WS:
    if ((d < -2147483648.0) || (d > 2147483647.0))
    {
      err = F("value outside the S32 range");
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
      // word-swapped: first register = low word (as for reading)
      job.words[0] = (uint16_t)(u & 0xFFFF);
      job.words[1] = (uint16_t)(u >> 16);
    }
    else
    {
      // standard (also float): first register = high word
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
  inst->sendCount = p156_nextTid(inst->conn); // transaction ID per connection

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

  inst->conn->client.clear(); // discard late bytes of an aborted request
  inst->rxLen = 0;
  inst->requestTime = millis();
  return inst->conn->client.write(req, len) == len;
}

// 0 = not complete yet, 1 = OK, 2 = Modbus exception (exCode), 3 = invalid response
uint8_t p156_parseWriteResponse(P156_Instance *inst, uint8_t &exCode)
{
  for (;;)
  {
    // exception response: 9 bytes, normal response to FC 6/16: 12 bytes
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
    // cyclic write: report OK only if the value changed or there was an error before
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
        return; // same error as last time - do not report in every cycle
      inst->lastErr[job.slot] = code;
    }
  }

  const String taskName = getTaskDeviceName(inst->taskIndex);
  const String valStr = doubleToString(job.value, 3, true);

  if (code == 0)
  {
    // register is also read? Then read it again at once so the task value is correct
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
      l += F(" error ");
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