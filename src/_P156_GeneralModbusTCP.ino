#include "_Plugin_Helper.h"

#ifdef USES_P156

// #######################################################################################################
// ######################## Plugin 156: General Modbus TCP ########################
// #######################################################################################################
//  PCONFIG:
//    [0] Port
//    [1] Unit ID
//    [2] FuncCode-Index (0=Func03, 1=Func04)
//    [3-6] Datentyp Query 0-3 (0=U16,1=S16,2=U32,3=S32,4=Float)
//    [7] Pause-Gruppen gepackt (bits 0-1=Q0, 2-3=Q1, 4-5=Q2, 6-7=Q3)
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
// #######################################################################################################

#define PLUGIN_156
#define PLUGIN_ID_156 156
#define PLUGIN_NAME_156 "Modbus TCP Generic"

#define P156_ARG_IP 0
#define P156_ARG_REG0 1
#define P156_ARG_PAU1 5
#define P156_ARG_GRP0 9

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
struct P156_Instance : public PluginTaskData_base
{
  WiFiClient client;
  char ip[16] = {};
  uint16_t port = 502;
  uint8_t unitId = 1;
  uint8_t funcCode = 3;
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
};

bool p156_sendRequest(P156_Instance *inst, uint8_t qIdx);
bool p156_parseValues(P156_Instance *inst, uint8_t qIdx);

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
    {
      const __FlashStringHelper *opts[] = {
          F("03 - Holding Register"),
          F("04 - Input Register"),
      };
      FormSelectorOptions sel(NR_ELEMENTS(opts), opts);
      sel.addFormSelector(F("Funktionscode"), P156_FUNC_LBL, P156_FUNC);
    }

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

    for (uint8_t i = 0; i < P156_NR_OUTPUT_VALUES; ++i)
    {
      addFormSubHeader(String(F("---- Wert ")) + (i + 1) + F(" ----"));

      char buf[8];
      snprintf(buf, sizeof(buf), "%u", p156_getReg(i, P156_REGS01, P156_REGS23));
      addFormTextBox(F("Registeradresse"), getPluginCustomArgName(P156_ARG_REG0 + i), buf, 6);
      addFormNote(F("PDU-Adresse 0-65535. 4xxxxx-Notation: Adresse minus 400001"));

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
    P156_FUNC = getFormItemInt(P156_FUNC_LBL);
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
    inst->funcCode = (P156_FUNC == 1) ? 4 : 3;

    int dtArr[] = {P156_DT0, P156_DT1, P156_DT2, P156_DT3};
    for (uint8_t i = 0; i < P156_NR_OUTPUT_VALUES; ++i)
    {
      inst->regAddr[i] = p156_getReg(i, P156_REGS01, P156_REGS23);
      inst->dataType[i] = (uint8_t)dtArr[i];
      inst->pauseGroup[i] = p156_getGroup(i, P156_PGROUPS);
      inst->values[i] = 0.0f;
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
    inst->pendingQuery = 255;
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
      log += F(" Func=0");
      log += inst->funcCode;
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

  case PLUGIN_READ:
  {
    P156_Instance *inst = static_cast<P156_Instance *>(getPluginTaskData(event->TaskIndex));
    if (inst && inst->myInit)
    {
      for (uint8_t i = 0; i < P156_NR_OUTPUT_VALUES; ++i)
        UserVar.setFloat(event->TaskIndex, i, inst->values[i]);
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
    if (inst->pendingQuery < P156_NR_OUTPUT_VALUES)
    {
      // Phase 1: pending request — try to read response
      uint8_t qIdx = inst->pendingQuery;
      bool ok = p156_parseValues(inst, qIdx);
      if (ok)
      {
        inst->pendingQuery = 255;
        inst->errorCount = 0;
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
          bool ok = p156_sendRequest(inst, qIdx);
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

bool p156_sendRequest(P156_Instance *inst, uint8_t qIdx)
{
  if (!inst)
    return false;
  // Skip connect attempt if WiFi not ready yet (prevents crash at boot)
  if (!NetworkConnected(0))
    return false;
  if (!inst->client.connected())
  {
    if (!inst->client.connect(inst->ip, inst->port, 500)) // 500ms TCP connect timeout
    {
      // Log only every 10th error (avoids log flood when device is offline)
      if (loglevelActiveFor(LOG_LEVEL_INFO) && (inst->errorCount % 10 == 0))
      {
        String l = F("P156: connect failed ");
        l += inst->ip;
        l += ':';
        l += inst->port;
        addLogMove(LOG_LEVEL_INFO, l);
      }
      return false;
    }
  }
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
  req[7] = inst->funcCode;
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

#endif // USES_P156