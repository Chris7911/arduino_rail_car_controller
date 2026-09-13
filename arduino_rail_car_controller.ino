/*
  Arduino UNO 滑軌車往返控制器
  ---------------------------------------------
  INPUT
  D2 = Input A：啟動 / 停止按鈕
  D3 = Input B：右側極限開關
  D4 = Input C：左側極限開關
  D5 = Input D：緊急停止

  OUTPUT
  D8  = Relay A：外部 12V 燈號
  D9  = Relay B：馬達正轉 / 往右
  D10 = Relay C：馬達反轉 / 往左

  Relay Module：ACTIVE HIGH
  ---------------------------------------------

  運轉邏輯：
  1. 按 Input A 啟動
  2. 一般啟動時先往右
  3. 撞右限位 -> 全部 OFF -> 休息 30 分鐘
  4. 30 分鐘後 -> 往左
  5. 撞左限位 -> 全部 OFF -> 休息 30 分鐘
  6. 30 分鐘後 -> 往右
  7. 持續循環
  8. 再按 Input A -> 正常停止
  9. Input D -> 緊急停止 / 鎖定
 10. 緊急解除後，必須重新按 Input A 才能啟動

  額外安全功能：
  A. 啟動位置判斷：
     - 右限位已 ON -> 啟動時只能往左
     - 左限位已 ON -> 啟動時只能往右
     - 左右限位同時 ON -> 故障，不允許啟動
  B. 左右移動各有最長運轉時間。
     超時即停止全部輸出並進入 FAULT 狀態。

  注意：
  - millis() 用於 30 分鐘等待，不阻塞主程式。
  - 本程式假設 Relay ON = HIGH / OFF = LOW。
  - 緊急停止建議另外以硬體安全回路直接切斷馬達接觸器/驅動器。
*/

// ================================
// 腳位
// ================================

const byte PIN_START     = 2;   // Input A
const byte PIN_RIGHT     = 3;   // Input B
const byte PIN_LEFT      = 4;   // Input C
const byte PIN_EMERGENCY = 5;   // Input D

const byte PIN_RELAY_A   = 8;   // 12V 燈號
const byte PIN_RELAY_B   = 9;   // 馬達往右
const byte PIN_RELAY_C   = 10;  // 馬達往左

// Active HIGH relay
const byte RELAY_ON  = HIGH;
const byte RELAY_OFF = LOW;

// ================================
// 參數
// ================================

// 到達端點後休息 30 分鐘
const unsigned long REST_TIME = 30UL * 60UL * 1000UL;

// 左右單向最大允許運轉時間
// 預設 5 分鐘，可依實際滑軌距離調整
const unsigned long MAX_TRAVEL_TIME = 5UL * 60UL * 1000UL;

// 正反轉切換的最小斷開時間
const unsigned long DIRECTION_DEAD_TIME = 100UL;

// 按鍵防彈跳
const unsigned long DEBOUNCE_TIME = 50UL;

// ================================
// 狀態
// ================================

enum State {
  STATE_IDLE,

  STATE_MOVING_RIGHT,
  STATE_REST_RIGHT,

  STATE_MOVING_LEFT,
  STATE_REST_LEFT,

  STATE_EMERGENCY,
  STATE_FAULT
};

State currentState = STATE_IDLE;

// ================================
// Timer
// ================================

unsigned long restStartTime  = 0;
unsigned long travelStartTime = 0;

// ================================
// START 按鈕 debounce
// ================================

bool lastStartReading = HIGH;
bool stableStartState = HIGH;
unsigned long lastStartChangeTime = 0;

// ================================
// 函式宣告
// ================================

void allRelaysOff();
void moveRight();
void moveLeft();
void startRightRest();
void startLeftRest();
void enterEmergency();
void enterFault(const char* reason);

bool startButtonPressed();

bool rightLimitActive();
bool leftLimitActive();
bool emergencyActive();

void startFromCurrentPosition();

// ================================
// SETUP
// ================================

void setup()
{
  pinMode(PIN_START, INPUT_PULLUP);
  pinMode(PIN_RIGHT, INPUT_PULLUP);
  pinMode(PIN_LEFT, INPUT_PULLUP);
  pinMode(PIN_EMERGENCY, INPUT_PULLUP);

  pinMode(PIN_RELAY_A, OUTPUT);
  pinMode(PIN_RELAY_B, OUTPUT);
  pinMode(PIN_RELAY_C, OUTPUT);

  // 開機第一件事：所有輸出關閉
  allRelaysOff();

  Serial.begin(9600);

  // 開機時如果緊急停止已經被按下，直接進入 EMERGENCY
  if (emergencyActive())
  {
    currentState = STATE_EMERGENCY;
    Serial.println("EMERGENCY ACTIVE AT POWER-UP");
  }
  else if (rightLimitActive() && leftLimitActive())
  {
    // 左右同時觸發，視為異常
    currentState = STATE_FAULT;
    Serial.println("FAULT: BOTH LIMIT SWITCHES ACTIVE");
  }
  else
  {
    currentState = STATE_IDLE;
    Serial.println("SYSTEM READY");
  }
}

// ================================
// LOOP
// ================================

void loop()
{
  // ------------------------------------------------
  // 最高優先權：緊急停止
  // ------------------------------------------------
  if (emergencyActive())
  {
    enterEmergency();

    // 持續鎖住，直到按鈕解除
    return;
  }

  // ------------------------------------------------
  // EMERGENCY 狀態
  // 緊急解除後不自動啟動
  // ------------------------------------------------
  if (currentState == STATE_EMERGENCY)
  {
    allRelaysOff();

    // 緊急解除後回 IDLE
    currentState = STATE_IDLE;

    Serial.println("Emergency released.");
    Serial.println("Press START to resume.");

    // 避免按鈕解除的機械震動造成誤觸
    delay(200);
    return;
  }

  // ------------------------------------------------
  // FAULT 狀態
  // 必須按 START 才清除故障，再重新判斷位置
  // ------------------------------------------------
  if (currentState == STATE_FAULT)
  {
    allRelaysOff();

    if (startButtonPressed())
    {
      // 如果還有異常，不能清除
      if (rightLimitActive() && leftLimitActive())
      {
        Serial.println("FAULT REMAINS: BOTH LIMITS ACTIVE");
      }
      else if (emergencyActive())
      {
        enterEmergency();
      }
      else
      {
        Serial.println("FAULT CLEARED.");
        currentState = STATE_IDLE;

        // 本次按鍵只用來清故障，不立即啟動
        // 下一次按 START 才啟動，避免誤動作。
      }
    }

    return;
  }

  // ------------------------------------------------
  // START / STOP 按鈕
  // ------------------------------------------------
  if (startButtonPressed())
  {
    // IDLE -> 啟動
    if (currentState == STATE_IDLE)
    {
      startFromCurrentPosition();
    }
    else
    {
      // 運轉中 / 休息中 -> 停止
      allRelaysOff();
      currentState = STATE_IDLE;

      Serial.println("NORMAL STOP");
    }
  }

  // ------------------------------------------------
  // 狀態機
  // ------------------------------------------------
  switch (currentState)
  {
    // ================================================
    // IDLE
    // ================================================
    case STATE_IDLE:
      allRelaysOff();
      break;

    // ================================================
    // 往右
    // ================================================
    case STATE_MOVING_RIGHT:

      // 最高優先：右限位
      if (rightLimitActive())
      {
        Serial.println("RIGHT LIMIT REACHED");
        startRightRest();
        break;
      }

      // 超時保護
      if (millis() - travelStartTime >= MAX_TRAVEL_TIME)
      {
        enterFault("RIGHT TRAVEL TIMEOUT");
        break;
      }

      break;

    // ================================================
    // 右側休息 30 分鐘
    // ================================================
    case STATE_REST_RIGHT:

      allRelaysOff();

      if (millis() - restStartTime >= REST_TIME)
      {
        // 再次確認沒有誤觸限位
        if (leftLimitActive() && rightLimitActive())
        {
          enterFault("BOTH LIMITS ACTIVE AFTER RIGHT REST");
        }
        else
        {
          Serial.println("RIGHT REST FINISHED");
          Serial.println("MOVE LEFT");

          moveLeft();
          currentState = STATE_MOVING_LEFT;
        }
      }

      break;

    // ================================================
    // 往左
    // ================================================
    case STATE_MOVING_LEFT:

      if (leftLimitActive())
      {
        Serial.println("LEFT LIMIT REACHED");
        startLeftRest();
        break;
      }

      // 超時保護
      if (millis() - travelStartTime >= MAX_TRAVEL_TIME)
      {
        enterFault("LEFT TRAVEL TIMEOUT");
        break;
      }

      break;

    // ================================================
    // 左側休息 30 分鐘
    // ================================================
    case STATE_REST_LEFT:

      allRelaysOff();

      if (millis() - restStartTime >= REST_TIME)
      {
        if (leftLimitActive() && rightLimitActive())
        {
          enterFault("BOTH LIMITS ACTIVE AFTER LEFT REST");
        }
        else
        {
          Serial.println("LEFT REST FINISHED");
          Serial.println("MOVE RIGHT");

          moveRight();
          currentState = STATE_MOVING_RIGHT;
        }
      }

      break;

    case STATE_EMERGENCY:
    case STATE_FAULT:
      allRelaysOff();
      break;
  }
}

// ================================================================
// 根據目前位置安全啟動
// ================================================================

void startFromCurrentPosition()
{
  bool right = rightLimitActive();
  bool left  = leftLimitActive();

  if (right && left)
  {
    enterFault("CANNOT START: BOTH LIMITS ACTIVE");
    return;
  }

  if (right)
  {
    // 在最右端，所以不能繼續往右
    Serial.println("START AT RIGHT LIMIT -> MOVE LEFT");

    moveLeft();
    currentState = STATE_MOVING_LEFT;
    return;
  }

  if (left)
  {
    // 在最左端，所以不能繼續往左
    Serial.println("START AT LEFT LIMIT -> MOVE RIGHT");

    moveRight();
    currentState = STATE_MOVING_RIGHT;
    return;
  }

  // 沒有任何限位觸發 -> 預設往右
  Serial.println("START -> MOVE RIGHT");

  moveRight();
  currentState = STATE_MOVING_RIGHT;
}

// ================================================================
// 往右
// Relay A ON：點亮外部 12V 燈
// Relay B ON：馬達正轉
// Relay C OFF
// ================================================================

void moveRight()
{
  // 先確保反轉 relay 關閉
  digitalWrite(PIN_RELAY_C, RELAY_OFF);

  // 等待正反轉之間的斷開時間
  delay(DIRECTION_DEAD_TIME);

  // 燈亮
  digitalWrite(PIN_RELAY_A, RELAY_ON);

  // 正轉
  digitalWrite(PIN_RELAY_B, RELAY_ON);

  travelStartTime = millis();

  Serial.println("MOTOR RIGHT + 12V LAMP ON");
}

// ================================================================
// 往左
// Relay A ON：點亮外部 12V 燈
// Relay B OFF
// Relay C ON：馬達反轉
// ================================================================

void moveLeft()
{
  // 先確保正轉 relay 關閉
  digitalWrite(PIN_RELAY_B, RELAY_OFF);

  delay(DIRECTION_DEAD_TIME);

  // 燈亮
  digitalWrite(PIN_RELAY_A, RELAY_ON);

  // 反轉
  digitalWrite(PIN_RELAY_C, RELAY_ON);

  travelStartTime = millis();

  Serial.println("MOTOR LEFT + 12V LAMP ON");
}

// ================================================================
// 所有輸出 OFF
// ================================================================

void allRelaysOff()
{
  digitalWrite(PIN_RELAY_A, RELAY_OFF);
  digitalWrite(PIN_RELAY_B, RELAY_OFF);
  digitalWrite(PIN_RELAY_C, RELAY_OFF);
}

// ================================================================
// 右側休息
// ================================================================

void startRightRest()
{
  allRelaysOff();

  restStartTime = millis();
  currentState = STATE_REST_RIGHT;

  Serial.println("RIGHT REST = 30 MINUTES");
}

// ================================================================
// 左側休息
// ================================================================

void startLeftRest()
{
  allRelaysOff();

  restStartTime = millis();
  currentState = STATE_REST_LEFT;

  Serial.println("LEFT REST = 30 MINUTES");
}

// ================================================================
// 緊急停止
// ================================================================

void enterEmergency()
{
  allRelaysOff();
  currentState = STATE_EMERGENCY;

  Serial.println("!!! EMERGENCY STOP !!!");
}

// ================================================================
// 故障
// ================================================================

void enterFault(const char* reason)
{
  allRelaysOff();
  currentState = STATE_FAULT;

  Serial.print("!!! FAULT: ");
  Serial.println(reason);
}

// ================================================================
// Input 狀態
// 使用 INPUT_PULLUP，所以 LOW = ON / 被按下
// ================================================================

bool rightLimitActive()
{
  return digitalRead(PIN_RIGHT) == LOW;
}

bool leftLimitActive()
{
  return digitalRead(PIN_LEFT) == LOW;
}

bool emergencyActive()
{
  return digitalRead(PIN_EMERGENCY) == LOW;
}

// ================================================================
// START 按鈕 Debounce
// 按下（HIGH -> LOW）時回傳 true
// ================================================================

bool startButtonPressed()
{
  bool reading = digitalRead(PIN_START);

  if (reading != lastStartReading)
  {
    lastStartChangeTime = millis();
  }

  if ((millis() - lastStartChangeTime) > DEBOUNCE_TIME)
  {
    if (reading != stableStartState)
    {
      stableStartState = reading;

      if (stableStartState == LOW)
      {
        lastStartReading = reading;
        return true;
      }
    }
  }

  lastStartReading = reading;

  return false;
}
