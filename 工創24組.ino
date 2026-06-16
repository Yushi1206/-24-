#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

// ============================================================
// 1. 硬體腳位設定
// ============================================================

// 5 顆循跡感測器，順序為：左 → 右
// 編碼方式：
// 左一 = bit4
// 左二 = bit3
// 中間 = bit2
// 右二 = bit1
// 右一 = bit0
//
// 例如：
// 00100 = 中間壓線
// 10000 = 最左側壓線
// 00001 = 最右側壓線
// 11111 = 全黑
// 00000 = 完全失線
const int SENSOR_PINS[5] = {13, 21, 14, 27, 26};

// L298N 左馬達
const int ENA_PIN = 32;
const int IN1_PIN = 25;
const int IN2_PIN = 33;

// L298N 右馬達
const int ENB_PIN = 4;
const int IN3_PIN = 16;
const int IN4_PIN = 17;

// RGB LED
const int LED_R = 15;
const int LED_G = 2;
const int LED_B = 5;

// 你的 RGB LED 是共陽極，因此 LOW 亮、HIGH 滅
const bool RGB_COMMON_ANODE = true;


// ============================================================
// 2. 系統狀態
// ============================================================

enum SystemState {
  STATE_SAFE,   // 綠燈，安全停止
  STATE_READY,  // 藍燈，起跑倒數
  STATE_AUTO    // 紅燈，自動循跡
};

SystemState currentState = STATE_SAFE;


// ============================================================
// 3. 行駛模式
// ============================================================

enum DriveMode {
  MODE_PD,      // 第一階段：PD 循跡，適合前半段小彎
  MODE_TABLE    // 第二階段：32-state 查表，適合後半段固定彎
};

DriveMode driveMode = MODE_PD;


// ============================================================
// 4. 自動切換設定
// ============================================================

// 目前採用「出發後 9 秒」自動切到第二階段查表模式。
// 這個時間是從 enterAutoState() 之後開始算，
// 不是從開機、綠燈或按下 G 的瞬間開始算。
const bool ENABLE_AUTO_TABLE_BY_TIME = true;
const unsigned long AUTO_TABLE_SWITCH_TIME = 9000;


// ============================================================
// 5. 第一階段：PD 模式參數
// ============================================================

// 第一段目標：快速通過小彎，但不要太激進。
// 如果小彎轉過頭：
// 1. 降低 KP_CURVE
// 2. 降低 MAX_STEERING_U
// 3. 降低 CURVE_SPEED
const float STRAIGHT_SPEED = 255.0f;
const float NORMAL_SPEED   = 230.0f;
const float CURVE_SPEED    = 190.0f;
const float SHARP_SPEED    = 158.0f;

// 一般馬達死區補償。
// L298N + 馬達在 PWM 太低時可能不會動，
// 所以只要輸出不為 0 且低於 MOTOR_DEADZONE，就補到這個值。
const int MOTOR_DEADZONE = 125;

// PD 參數。
// KP_STRAIGHT：線接近中心時使用，避免直線抖動。
// KP_CURVE：一般彎道使用。
// KP_SHARP：偏差很大時使用。
const float KP_STRAIGHT = 2.4f;
const float KP_CURVE    = 6.1f;
const float KP_SHARP    = 8.3f;
const float KD_VALUE    = 2.5f;

// 轉向預留量。
// 轉向越大，baseSpeed 會稍微下降，避免左右輪加總超過 255。
const float TURN_RESERVE_RATIO = 0.20f;

// 最大轉向輸出。
// 若小彎偶爾轉過頭，可以先把 120 降到 110。
const float MAX_STEERING_U = 120.0f;

// READY 狀態倒數時間。
// 如果想按 G 後更快起跑，可改 500 或 0。
const unsigned long READY_DELAY_TIME = 3000;

// 起跑後前 600 ms 忽略起跑線 / 全黑線誤判。
const unsigned long START_LINE_IGNORE_TIME = 600;


// ============================================================
// 6. 第一階段：失線與全黑線處理
// ============================================================

// 完全失線後，前 LOST_SPIN_DELAY ms 先沿用最後方向輕微修正。
// 超過後再強一點旋轉搜尋。
const unsigned long LOST_GRACE_TIME = 260;
const unsigned long LOST_SPIN_DELAY = 140;

const int LOST_TURN_OUTER_SPEED = 190;
const int LOST_TURN_INNER_SPEED = 40;

const int LOST_SPIN_OUTER_SPEED = 160;
const int LOST_SPIN_INNER_SPEED = -60;

const int SEARCH_SPIN_SPEED = 115;

// 全黑線通過速度。
// 用於起跑線、終點線或粗黑線。
const int FULL_BLACK_SPEED = 145;
const unsigned long FULL_BLACK_HOLD_TIME = 35;


// ============================================================
// 7. 第一階段：直角彎輔助
// ============================================================

// 第一階段主要用於前半段小彎，直角觸發要保守。
// RIGHT_ANGLE_CONFIRM_COUNT = 3 表示連續 3 次偵測到極端偏差才進直角模式。
const int RIGHT_ANGLE_KICK_OUTER_SPEED = 255;
const int RIGHT_ANGLE_KICK_INNER_SPEED = -65;
const unsigned long RIGHT_ANGLE_KICK_TIME = 180;

const int RIGHT_ANGLE_HOLD_OUTER_SPEED = 220;
const int RIGHT_ANGLE_HOLD_INNER_SPEED = 45;

const int RIGHT_ANGLE_LOST_OUTER_SPEED = 210;
const int RIGHT_ANGLE_LOST_INNER_SPEED = -55;

const unsigned long RIGHT_ANGLE_PIVOT_START_TIME = 360;
const int RIGHT_ANGLE_PIVOT_OUTER_SPEED = 225;
const int RIGHT_ANGLE_PIVOT_INNER_SPEED = -95;

const unsigned long RIGHT_ANGLE_FORCE_START_TIME = 560;
const int RIGHT_ANGLE_FORCE_OUTER_SPEED = 205;
const int RIGHT_ANGLE_FORCE_INNER_SPEED = -120;

const unsigned long RIGHT_ANGLE_MIN_TIME = 320;
const unsigned long RIGHT_ANGLE_MAX_TIME = 950;

const int CENTER_CONFIRM_COUNT = 3;

const int RIGHT_ANGLE_CONFIRM_COUNT = 3;
int rightAngleRightCount = 0;
int rightAngleLeftCount = 0;


// ============================================================
// 8. 第一階段：出彎限速
// ============================================================

// 直角模式結束後，短暫限制速度，避免剛出彎就衝出去。
const unsigned long CORNER_EXIT_SLOW_TIME = 120;
const float CORNER_EXIT_SPEED_LIMIT = 205.0f;
unsigned long cornerExitTime = 0;


// ============================================================
// 9. 第二階段：32-state 查表模式參數
// ============================================================

// 第二階段使用較高死區補償，避免查表低速輸出推不動馬達。
const int TABLE_MOTOR_DEADZONE = 145;

// 第二階段基礎速度
const int T_SPEED_STRAIGHT = 215;
const int T_SPEED_NORMAL   = 195;
const int T_SPEED_SLOW     = 165;

// 輕微偏移修正
const int T_SLIGHT_OUTER_SPEED = 210;
const int T_SLIGHT_INNER_SPEED = 165;

// 中度偏移修正
const int T_MEDIUM_OUTER_SPEED = 215;
const int T_MEDIUM_INNER_SPEED = 145;

// 急彎查表修正。
// 這裡允許內輪反轉，但真正輸出時有些情況會用 Raw，避免被死區放大。
const int T_HARD_OUTER_SPEED = 200;
const int T_HARD_INNER_SPEED = -140;

// 第二階段失線搜尋
const unsigned long T_LOST_SOFT_TIME = 140;
const unsigned long T_LOST_HARD_TIME = 420;

const int T_LOST_SOFT_OUTER_SPEED = 180;
const int T_LOST_SOFT_INNER_SPEED = 70;

const int T_LOST_HARD_OUTER_SPEED = 170;
const int T_LOST_HARD_INNER_SPEED = -90;


// ============================================================
// 10. 第二階段：直角個案修正
// ============================================================

// 第二階段若看到 10000 或 00001，代表可能是很急的直角。
// 進入 tableRightAngleMode 後，會短暫鎖定大差速轉彎。
bool tableRightAngleMode = false;
int tableRightAngleDir = 0;
unsigned long tableRightAngleStartTime = 0;
int tableCenterSeenCount = 0;

// 直角剛開始的短暫 Kick。
// 使用 Raw 輸出，避免 -110 被死區補償放大。
const int T_RIGHT_ANGLE_KICK_OUTER_SPEED = 235;
const int T_RIGHT_ANGLE_KICK_INNER_SPEED = -110;
const unsigned long T_RIGHT_ANGLE_KICK_TIME = 120;

// Kick 後的 Hold。
// 內輪設 0，避免長時間原地自轉。
const int T_RIGHT_ANGLE_HOLD_OUTER_SPEED = 215;
const int T_RIGHT_ANGLE_HOLD_INNER_SPEED = 0;

// 直角模式中若完全失線，用較溫和的反轉找線。
const int T_RIGHT_ANGLE_LOST_OUTER_SPEED = 185;
const int T_RIGHT_ANGLE_LOST_INNER_SPEED = -75;

const unsigned long T_RIGHT_ANGLE_MIN_TIME = 180;
const unsigned long T_RIGHT_ANGLE_MAX_TIME = 620;

const int T_CENTER_CONFIRM_COUNT = 2;


// ============================================================
// 11. 全域控制變數
// ============================================================

unsigned long readyStartTime = 0;
unsigned long autoStartTime = 0;
unsigned long fullBlackStartTime = 0;

bool handlingFullBlack = false;

// 第一階段直角狀態
bool rightAngleMode = false;
int rightAngleDir = 0;
unsigned long rightAngleStartTime = 0;
int centerSeenCount = 0;

// 最後一次明顯轉向方向。
// -1 = 左轉
//  0 = 無明顯方向
//  1 = 右轉
int lastTurnDir = 0;

// 第一階段 PD 記憶
float pdLastError = 0;
float pdLastLeftT = 0;
float pdLastRightT = 0;
unsigned long pdLostStartTime = 0;

// 第二階段查表記憶。
// 只保留方向，失線時用來決定往哪邊找線。
int tableLastDirection = 0;
unsigned long tableLostStartTime = 0;


// ============================================================
// 12. 函式宣告
// ============================================================

void followLine();
void followLinePD(int state);
void followLineTable(int state);
void updateAutoDriveMode();
void switchDriveMode(DriveMode newMode);
void resetControlMemory();

void startTableRightAngle(int dir);
bool handleTableRightAngle(int state);

void setThrust(float leftThrust, float rightThrust);
void setThrustRaw(float leftThrust, float rightThrust);
void setThrustTable(float leftThrust, float rightThrust);
void setThrustTableRaw(float leftThrust, float rightThrust);
void applyMotorOutput(float leftThrust, float rightThrust, int deadzone);
void stopMotors();

int getSensorState();

void updateASL();
void setRGB(bool r, bool g, bool b);

void enterSafeState();
void enterReadyState();
void enterAutoState();
void checkBluetooth();

void driveTable(float leftT, float rightT, int dir);
void driveTableRaw(float leftT, float rightT, int dir);


// ============================================================
// 13. 初始化
// ============================================================

void setup() {
  Serial.begin(115200);

  // 感測器輸入
  for (int i = 0; i < 5; i++) {
    pinMode(SENSOR_PINS[i], INPUT);
  }

  // 馬達方向腳
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);

  // RGB LED
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);

  // ESP32 PWM
  ledcAttach(ENA_PIN, 5000, 8);
  ledcAttach(ENB_PIN, 5000, 8);

  // 先啟動藍牙，再亮綠燈。
  // 這樣綠燈代表藍牙也已經開始廣播。
  SerialBT.begin("NCKU_BT_Robot");

  Serial.println("【系統就緒】傳統藍牙廣播中：NCKU_BT_Robot");
  Serial.println("指令：G=起跑, S=煞車, P=PD模式, T=查表模式");

  enterSafeState();
}


// ============================================================
// 14. 主迴圈
// ============================================================

void loop() {
  checkBluetooth();

  switch (currentState) {
    case STATE_SAFE:
      stopMotors();
      break;

    case STATE_READY:
      stopMotors();

      if (millis() - readyStartTime >= READY_DELAY_TIME) {
        enterAutoState();
      }
      break;

    case STATE_AUTO:
      followLine();
      break;
  }
}


// ============================================================
// 15. 藍牙控制
// ============================================================

void checkBluetooth() {
  while (SerialBT.available()) {
    char cmd = SerialBT.read();

    if (cmd == 'S' || cmd == 's') {
      Serial.println(">>> 收到 S：進入 SAFE，停車 <<<");
      enterSafeState();
    }
    else if ((cmd == 'G' || cmd == 'g') && currentState == STATE_SAFE) {
      Serial.println(">>> 收到 G：進入 READY，等待倒數 <<<");
      enterReadyState();
    }
    else if (cmd == 'P' || cmd == 'p') {
      switchDriveMode(MODE_PD);
      Serial.println(">>> 手動切換：第一階段 PD 模式 <<<");
    }
    else if (cmd == 'T' || cmd == 't') {
      switchDriveMode(MODE_TABLE);
      Serial.println(">>> 手動切換：第二階段查表模式 <<<");
    }
  }
}


// ============================================================
// 16. 狀態切換
// ============================================================

void enterSafeState() {
  currentState = STATE_SAFE;
  resetControlMemory();
  stopMotors();
  updateASL();
}

void enterReadyState() {
  currentState = STATE_READY;
  readyStartTime = millis();
  resetControlMemory();
  stopMotors();
  updateASL();
}

void enterAutoState() {
  currentState = STATE_AUTO;

  // autoStartTime 是自動切換 9 秒的起算點。
  autoStartTime = millis();

  switchDriveMode(MODE_PD);
  updateASL();

  Serial.println(">>> 出發！預設第一階段 PD 模式 <<<");
}

void switchDriveMode(DriveMode newMode) {
  driveMode = newMode;

  // 切換模式時，清掉舊模式的特殊狀態，避免互相干擾。
  rightAngleMode = false;
  centerSeenCount = 0;
  rightAngleRightCount = 0;
  rightAngleLeftCount = 0;

  tableRightAngleMode = false;
  tableCenterSeenCount = 0;

  pdLostStartTime = 0;
  tableLostStartTime = 0;

  handlingFullBlack = false;
  fullBlackStartTime = 0;

  // 從 PD 進入 TABLE 時，把最近轉向方向交給查表模式。
  if (newMode == MODE_TABLE) {
    tableLastDirection = lastTurnDir;
  }
}

void resetControlMemory() {
  driveMode = MODE_PD;

  handlingFullBlack = false;
  fullBlackStartTime = 0;

  rightAngleMode = false;
  rightAngleDir = 0;
  rightAngleStartTime = 0;
  centerSeenCount = 0;

  rightAngleRightCount = 0;
  rightAngleLeftCount = 0;

  tableRightAngleMode = false;
  tableRightAngleDir = 0;
  tableRightAngleStartTime = 0;
  tableCenterSeenCount = 0;

  cornerExitTime = 0;
  lastTurnDir = 0;

  pdLastError = 0;
  pdLastLeftT = 0;
  pdLastRightT = 0;
  pdLostStartTime = 0;

  tableLastDirection = 0;
  tableLostStartTime = 0;
}


// ============================================================
// 17. ASL 狀態燈
// ============================================================

void setRGB(bool r, bool g, bool b) {
  if (RGB_COMMON_ANODE) {
    digitalWrite(LED_R, r ? LOW : HIGH);
    digitalWrite(LED_G, g ? LOW : HIGH);
    digitalWrite(LED_B, b ? LOW : HIGH);
  } else {
    digitalWrite(LED_R, r ? HIGH : LOW);
    digitalWrite(LED_G, g ? HIGH : LOW);
    digitalWrite(LED_B, b ? HIGH : LOW);
  }
}

void updateASL() {
  if (currentState == STATE_SAFE) {
    setRGB(false, true, false);   // 綠燈
  }
  else if (currentState == STATE_READY) {
    setRGB(false, false, true);   // 藍燈
  }
  else if (currentState == STATE_AUTO) {
    setRGB(true, false, false);   // 紅燈
  }
}


// ============================================================
// 18. 感測器讀取
// ============================================================

int getSensorState() {
  int state = 0;

  for (int i = 0; i < 5; i++) {
    if (digitalRead(SENSOR_PINS[i]) == LOW) {
      state |= (1 << (4 - i));
    }
  }

  return state;
}


// ============================================================
// 19. 模式自動切換
// ============================================================

void updateAutoDriveMode() {
  if (driveMode == MODE_TABLE) return;
  if (!ENABLE_AUTO_TABLE_BY_TIME) return;

  if (millis() - autoStartTime > AUTO_TABLE_SWITCH_TIME) {
    switchDriveMode(MODE_TABLE);
    Serial.println(">>> 自動切換：第二階段查表模式 <<<");
  }
}


// ============================================================
// 20. 主循跡入口
// ============================================================

void followLine() {
  int state = getSensorState();

  updateAutoDriveMode();

  if (driveMode == MODE_PD) {
    followLinePD(state);
  } else {
    followLineTable(state);
  }
}


// ============================================================
// 21. 第一階段：PD 循跡
// ============================================================

void followLinePD(int state) {
  bool startProtection = (millis() - autoStartTime < START_LINE_IGNORE_TIME);

  // ------------------------------------------------------------
  // A. 第一階段直角模式
  // ------------------------------------------------------------
  if (rightAngleMode) {
    unsigned long elapsed = millis() - rightAngleStartTime;

    bool centerDetected = (state & 0b00100);
    bool stableExitState = (
      state == 0b00100 ||
      state == 0b01100 ||
      state == 0b00110 ||
      state == 0b01110
    );

    // 直角中完全失線：照直角方向繼續找線。
    if (state == 0b00000) {
      if (rightAngleDir > 0) {
        setThrustRaw(RIGHT_ANGLE_LOST_OUTER_SPEED, RIGHT_ANGLE_LOST_INNER_SPEED);
        pdLastLeftT = RIGHT_ANGLE_LOST_OUTER_SPEED;
        pdLastRightT = RIGHT_ANGLE_LOST_INNER_SPEED;
      } else {
        setThrustRaw(RIGHT_ANGLE_LOST_INNER_SPEED, RIGHT_ANGLE_LOST_OUTER_SPEED);
        pdLastLeftT = RIGHT_ANGLE_LOST_INNER_SPEED;
        pdLastRightT = RIGHT_ANGLE_LOST_OUTER_SPEED;
      }
      return;
    }

    // 必須過最短時間後，才允許看到中心線就退出。
    if (elapsed >= RIGHT_ANGLE_MIN_TIME && centerDetected && stableExitState) {
      centerSeenCount++;
    } else {
      centerSeenCount = 0;
    }

    // 退出直角模式
    if (centerSeenCount >= CENTER_CONFIRM_COUNT || elapsed > RIGHT_ANGLE_MAX_TIME) {
      rightAngleMode = false;
      centerSeenCount = 0;
      cornerExitTime = millis();
      rightAngleRightCount = 0;
      rightAngleLeftCount = 0;
    }
    else {
      int outerSpeed;
      int innerSpeed;
      bool useRaw = false;

      if (elapsed < RIGHT_ANGLE_KICK_TIME) {
        outerSpeed = RIGHT_ANGLE_KICK_OUTER_SPEED;
        innerSpeed = RIGHT_ANGLE_KICK_INNER_SPEED;
        useRaw = true;
      }
      else if (elapsed > RIGHT_ANGLE_FORCE_START_TIME && !centerDetected) {
        outerSpeed = RIGHT_ANGLE_FORCE_OUTER_SPEED;
        innerSpeed = RIGHT_ANGLE_FORCE_INNER_SPEED;
        useRaw = true;
      }
      else if (elapsed > RIGHT_ANGLE_PIVOT_START_TIME && !centerDetected) {
        outerSpeed = RIGHT_ANGLE_PIVOT_OUTER_SPEED;
        innerSpeed = RIGHT_ANGLE_PIVOT_INNER_SPEED;
        useRaw = true;
      }
      else {
        outerSpeed = RIGHT_ANGLE_HOLD_OUTER_SPEED;
        innerSpeed = RIGHT_ANGLE_HOLD_INNER_SPEED;
      }

      if (rightAngleDir > 0) {
        if (useRaw) setThrustRaw(outerSpeed, innerSpeed);
        else setThrust(outerSpeed, innerSpeed);

        pdLastLeftT = outerSpeed;
        pdLastRightT = innerSpeed;
      } else {
        if (useRaw) setThrustRaw(innerSpeed, outerSpeed);
        else setThrust(innerSpeed, outerSpeed);

        pdLastLeftT = innerSpeed;
        pdLastRightT = outerSpeed;
      }

      return;
    }
  }

  // ------------------------------------------------------------
  // B. 全黑線短暫穿越
  // ------------------------------------------------------------
  if (handlingFullBlack) {
    if (millis() - fullBlackStartTime < FULL_BLACK_HOLD_TIME) {
      setThrust(FULL_BLACK_SPEED, FULL_BLACK_SPEED);
      return;
    } else {
      handlingFullBlack = false;
    }
  }

  // ------------------------------------------------------------
  // C. 完全失線
  // ------------------------------------------------------------
  if (state == 0b00000) {
    if (pdLostStartTime == 0) {
      pdLostStartTime = millis();
    }

    unsigned long lostDuration = millis() - pdLostStartTime;

    int searchDir = lastTurnDir;

    if (searchDir == 0) {
      if (pdLastError > 0) searchDir = 1;
      else if (pdLastError < 0) searchDir = -1;
    }

    if (lostDuration < LOST_SPIN_DELAY) {
      if (searchDir > 0) {
        setThrust(LOST_TURN_OUTER_SPEED, LOST_TURN_INNER_SPEED);
      }
      else if (searchDir < 0) {
        setThrust(LOST_TURN_INNER_SPEED, LOST_TURN_OUTER_SPEED);
      }
      else {
        setThrust(pdLastLeftT * 0.85f, pdLastRightT * 0.85f);
      }
    }
    else if (lostDuration < LOST_GRACE_TIME) {
      if (searchDir > 0) {
        setThrust(LOST_SPIN_OUTER_SPEED, LOST_SPIN_INNER_SPEED);
      }
      else if (searchDir < 0) {
        setThrust(LOST_SPIN_INNER_SPEED, LOST_SPIN_OUTER_SPEED);
      }
      else {
        setThrust(130, 130);
      }
    }
    else {
      if (searchDir > 0) {
        setThrust(SEARCH_SPIN_SPEED, -SEARCH_SPIN_SPEED);
      }
      else if (searchDir < 0) {
        setThrust(-SEARCH_SPIN_SPEED, SEARCH_SPIN_SPEED);
      }
      else {
        setThrust(130, 130);
      }
    }

    return;
  } else {
    pdLostStartTime = 0;
  }

  // ------------------------------------------------------------
  // D. 起跑線 / 全黑線
  // ------------------------------------------------------------
  if (state == 0b11111) {
    if (startProtection) {
      setThrust(220, 220);
      return;
    }

    handlingFullBlack = true;
    fullBlackStartTime = millis();
    setThrust(FULL_BLACK_SPEED, FULL_BLACK_SPEED);
    return;
  }

  // ------------------------------------------------------------
  // E. 計算 PD 誤差
  // ------------------------------------------------------------
  int weights[5] = {-24, -12, 0, 12, 24};

  int activeSensors = 0;
  float errorSum = 0;

  bool extremeLeft = false;
  bool extremeRight = false;

  for (int i = 0; i < 5; i++) {
    if (digitalRead(SENSOR_PINS[i]) == LOW) {
      errorSum += weights[i];
      activeSensors++;

      if (i == 0) extremeLeft = true;
      if (i == 4) extremeRight = true;
    }
  }

  if (activeSensors == 0) return;

  float currentError = errorSum / activeSensors;
  float absError = currentError >= 0 ? currentError : -currentError;

  if (currentError > 8) {
    lastTurnDir = 1;
  } else if (currentError < -8) {
    lastTurnDir = -1;
  }

  float baseSpeed;
  float Kp;

  if (absError <= 4) {
    baseSpeed = STRAIGHT_SPEED;
    Kp = KP_STRAIGHT;
  }
  else if (absError <= 12) {
    baseSpeed = NORMAL_SPEED;
    Kp = KP_STRAIGHT;
  }
  else if (absError <= 20) {
    baseSpeed = CURVE_SPEED;
    Kp = KP_CURVE;
  }
  else {
    baseSpeed = SHARP_SPEED;
    Kp = KP_SHARP;
  }

  if (cornerExitTime > 0 && millis() - cornerExitTime < CORNER_EXIT_SLOW_TIME) {
    if (baseSpeed > CORNER_EXIT_SPEED_LIMIT) {
      baseSpeed = CORNER_EXIT_SPEED_LIMIT;
    }
  }

  float derivative = currentError - pdLastError;
  float steeringU = Kp * currentError + KD_VALUE * derivative;

  pdLastError = currentError;

  // ------------------------------------------------------------
  // F. 第一階段直角偵測
  // ------------------------------------------------------------
  if (!startProtection && extremeRight && currentError > 18) {
    rightAngleRightCount++;
  } else {
    rightAngleRightCount = 0;
  }

  if (!startProtection && extremeLeft && currentError < -18) {
    rightAngleLeftCount++;
  } else {
    rightAngleLeftCount = 0;
  }

  if (!startProtection && rightAngleRightCount >= RIGHT_ANGLE_CONFIRM_COUNT) {
    rightAngleMode = true;
    rightAngleDir = 1;
    lastTurnDir = 1;
    rightAngleStartTime = millis();
    centerSeenCount = 0;
    rightAngleRightCount = 0;
    rightAngleLeftCount = 0;

    pdLastLeftT = RIGHT_ANGLE_KICK_OUTER_SPEED;
    pdLastRightT = RIGHT_ANGLE_KICK_INNER_SPEED;

    setThrustRaw(RIGHT_ANGLE_KICK_OUTER_SPEED, RIGHT_ANGLE_KICK_INNER_SPEED);
    return;
  }

  if (!startProtection && rightAngleLeftCount >= RIGHT_ANGLE_CONFIRM_COUNT) {
    rightAngleMode = true;
    rightAngleDir = -1;
    lastTurnDir = -1;
    rightAngleStartTime = millis();
    centerSeenCount = 0;
    rightAngleRightCount = 0;
    rightAngleLeftCount = 0;

    pdLastLeftT = RIGHT_ANGLE_KICK_INNER_SPEED;
    pdLastRightT = RIGHT_ANGLE_KICK_OUTER_SPEED;

    setThrustRaw(RIGHT_ANGLE_KICK_INNER_SPEED, RIGHT_ANGLE_KICK_OUTER_SPEED);
    return;
  }

  // ------------------------------------------------------------
  // G. 一般 PD 差速輸出
  // ------------------------------------------------------------
  steeringU = constrain(steeringU, -MAX_STEERING_U, MAX_STEERING_U);

  float steeringAbs = steeringU >= 0 ? steeringU : -steeringU;
  float turnReserve = steeringAbs * TURN_RESERVE_RATIO;
  float maxAllowedBase = 255.0f - turnReserve;

  if (baseSpeed > maxAllowedBase) {
    baseSpeed = maxAllowedBase;
  }

  baseSpeed = constrain(baseSpeed, 150.0f, 255.0f);

  float leftT = baseSpeed + steeringU;
  float rightT = baseSpeed - steeringU;

  leftT = constrain(leftT, -255.0f, 255.0f);
  rightT = constrain(rightT, -255.0f, 255.0f);

  pdLastLeftT = leftT;
  pdLastRightT = rightT;

  setThrust(leftT, rightT);
}


// ============================================================
// 22. 第二階段：查表模式輔助
// ============================================================

void driveTable(float leftT, float rightT, int dir) {
  if (dir != 0) {
    tableLastDirection = dir;
  }

  setThrustTable(leftT, rightT);
}

void driveTableRaw(float leftT, float rightT, int dir) {
  if (dir != 0) {
    tableLastDirection = dir;
  }

  setThrustTableRaw(leftT, rightT);
}


// ============================================================
// 23. 第二階段：直角個案模式
// ============================================================

void startTableRightAngle(int dir) {
  tableRightAngleMode = true;
  tableRightAngleDir = dir;
  tableRightAngleStartTime = millis();
  tableCenterSeenCount = 0;
  tableLastDirection = dir;

  if (dir > 0) {
    driveTableRaw(T_RIGHT_ANGLE_KICK_OUTER_SPEED, T_RIGHT_ANGLE_KICK_INNER_SPEED, 1);
  } else {
    driveTableRaw(T_RIGHT_ANGLE_KICK_INNER_SPEED, T_RIGHT_ANGLE_KICK_OUTER_SPEED, -1);
  }
}

bool handleTableRightAngle(int state) {
  if (!tableRightAngleMode) return false;

  unsigned long elapsed = millis() - tableRightAngleStartTime;

  bool centerDetected = (state & 0b00100);
  bool stableExitState = (
    state == 0b00100 ||
    state == 0b01100 ||
    state == 0b00110 ||
    state == 0b01110
  );

  if (elapsed >= T_RIGHT_ANGLE_MIN_TIME && centerDetected && stableExitState) {
    tableCenterSeenCount++;
  } else {
    tableCenterSeenCount = 0;
  }

  if (tableCenterSeenCount >= T_CENTER_CONFIRM_COUNT || elapsed > T_RIGHT_ANGLE_MAX_TIME) {
    tableRightAngleMode = false;
    tableCenterSeenCount = 0;
    return false;
  }

  int outerSpeed;
  int innerSpeed;
  bool useRaw = false;

  if (state == 0b00000) {
    outerSpeed = T_RIGHT_ANGLE_LOST_OUTER_SPEED;
    innerSpeed = T_RIGHT_ANGLE_LOST_INNER_SPEED;
    useRaw = true;
  }
  else if (elapsed < T_RIGHT_ANGLE_KICK_TIME) {
    outerSpeed = T_RIGHT_ANGLE_KICK_OUTER_SPEED;
    innerSpeed = T_RIGHT_ANGLE_KICK_INNER_SPEED;
    useRaw = true;
  }
  else {
    outerSpeed = T_RIGHT_ANGLE_HOLD_OUTER_SPEED;
    innerSpeed = T_RIGHT_ANGLE_HOLD_INNER_SPEED;
  }

  if (tableRightAngleDir > 0) {
    if (useRaw) driveTableRaw(outerSpeed, innerSpeed, 1);
    else driveTable(outerSpeed, innerSpeed, 1);
  } else {
    if (useRaw) driveTableRaw(innerSpeed, outerSpeed, -1);
    else driveTable(innerSpeed, outerSpeed, -1);
  }

  return true;
}


// ============================================================
// 24. 第二階段：32-state 查表循跡
// ============================================================

void followLineTable(int state) {
  bool startProtection = (millis() - autoStartTime < START_LINE_IGNORE_TIME);

  // 直角個案模式優先權最高。
  if (handleTableRightAngle(state)) {
    return;
  }

  // ------------------------------------------------------------
  // A. 完全失線
  // ------------------------------------------------------------
  if (state == 0b00000) {
    if (tableLostStartTime == 0) {
      tableLostStartTime = millis();
    }

    unsigned long lostTime = millis() - tableLostStartTime;

    if (tableLastDirection > 0) {
      if (lostTime < T_LOST_SOFT_TIME) {
        driveTable(T_LOST_SOFT_OUTER_SPEED, T_LOST_SOFT_INNER_SPEED, 1);
      }
      else if (lostTime < T_LOST_HARD_TIME) {
        driveTableRaw(T_LOST_HARD_OUTER_SPEED, T_LOST_HARD_INNER_SPEED, 1);
      }
      else {
        driveTableRaw(150, -120, 1);
      }
    }
    else if (tableLastDirection < 0) {
      if (lostTime < T_LOST_SOFT_TIME) {
        driveTable(T_LOST_SOFT_INNER_SPEED, T_LOST_SOFT_OUTER_SPEED, -1);
      }
      else if (lostTime < T_LOST_HARD_TIME) {
        driveTableRaw(T_LOST_HARD_INNER_SPEED, T_LOST_HARD_OUTER_SPEED, -1);
      }
      else {
        driveTableRaw(-120, 150, -1);
      }
    }
    else {
      driveTable(150, 150, 0);
    }

    return;
  } else {
    tableLostStartTime = 0;
  }

  // ------------------------------------------------------------
  // B. 全黑線
  // ------------------------------------------------------------
  if (state == 0b11111) {
    if (startProtection) {
      driveTable(165, 165, 0);
    } else {
      driveTable(155, 155, 0);
    }
    return;
  }

  // ------------------------------------------------------------
  // C. 32 種狀態查表
  // ------------------------------------------------------------
  switch (state) {
    case 0b00100:
      driveTable(T_SPEED_STRAIGHT, T_SPEED_STRAIGHT, 0);
      return;

    case 0b01110:
      driveTable(T_SPEED_NORMAL, T_SPEED_NORMAL, 0);
      return;

    case 0b01100:
      driveTable(T_SLIGHT_INNER_SPEED, T_SLIGHT_OUTER_SPEED, -1);
      return;

    case 0b01000:
    case 0b11100:
      driveTable(T_MEDIUM_INNER_SPEED, T_MEDIUM_OUTER_SPEED, -1);
      return;

    case 0b11000:
      driveTable(T_HARD_INNER_SPEED, T_HARD_OUTER_SPEED, -1);
      return;

    case 0b10000:
      startTableRightAngle(-1);
      return;

    case 0b00110:
      driveTable(T_SLIGHT_OUTER_SPEED, T_SLIGHT_INNER_SPEED, 1);
      return;

    case 0b00010:
    case 0b00111:
      driveTable(T_MEDIUM_OUTER_SPEED, T_MEDIUM_INNER_SPEED, 1);
      return;

    case 0b00011:
      driveTable(T_HARD_OUTER_SPEED, T_HARD_INNER_SPEED, 1);
      return;

    case 0b00001:
      startTableRightAngle(1);
      return;

    case 0b11110:
    case 0b11101:
    case 0b11010:
    case 0b10100:
      driveTable(T_HARD_INNER_SPEED, T_SPEED_NORMAL, -1);
      return;

    case 0b01111:
    case 0b10111:
    case 0b01011:
    case 0b00101:
      driveTable(T_SPEED_NORMAL, T_HARD_INNER_SPEED, 1);
      return;

    case 0b10110:
    case 0b10010:
    case 0b10011:
      driveTable(T_SPEED_NORMAL, T_SPEED_SLOW, 1);
      return;

    case 0b01101:
    case 0b01001:
    case 0b11001:
      driveTable(T_SPEED_SLOW, T_SPEED_NORMAL, -1);
      return;

    default:
      driveTable(T_SPEED_SLOW, T_SPEED_SLOW, tableLastDirection);
      return;
  }
}


// ============================================================
// 25. 馬達控制
// ============================================================

void stopMotors() {
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, LOW);

  ledcWrite(ENA_PIN, 0);
  ledcWrite(ENB_PIN, 0);
}

void applyMotorOutput(float leftThrust, float rightThrust, int deadzone) {
  leftThrust = constrain(leftThrust, -255.0f, 255.0f);
  rightThrust = constrain(rightThrust, -255.0f, 255.0f);

  if (leftThrust > 0) {
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);
  }
  else if (leftThrust < 0) {
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, HIGH);
  }
  else {
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, LOW);
  }

  if (rightThrust > 0) {
    digitalWrite(IN3_PIN, HIGH);
    digitalWrite(IN4_PIN, LOW);
  }
  else if (rightThrust < 0) {
    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, HIGH);
  }
  else {
    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, LOW);
  }

  int leftPWM = abs((int)leftThrust);
  int rightPWM = abs((int)rightThrust);

  // deadzone = 0 時代表 Raw 模式，不做死區補償。
  if (deadzone > 0) {
    if (leftPWM > 0 && leftPWM < deadzone) {
      leftPWM = deadzone;
    }

    if (rightPWM > 0 && rightPWM < deadzone) {
      rightPWM = deadzone;
    }
  }

  leftPWM = constrain(leftPWM, 0, 255);
  rightPWM = constrain(rightPWM, 0, 255);

  ledcWrite(ENA_PIN, leftPWM);
  ledcWrite(ENB_PIN, rightPWM);
}

void setThrust(float leftThrust, float rightThrust) {
  applyMotorOutput(leftThrust, rightThrust, MOTOR_DEADZONE);
}

void setThrustRaw(float leftThrust, float rightThrust) {
  applyMotorOutput(leftThrust, rightThrust, 0);
}

void setThrustTable(float leftThrust, float rightThrust) {
  applyMotorOutput(leftThrust, rightThrust, TABLE_MOTOR_DEADZONE);
}

void setThrustTableRaw(float leftThrust, float rightThrust) {
  applyMotorOutput(leftThrust, rightThrust, 0);
}