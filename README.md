# Arduino-Based Smart Desk Lamp
**课程**：电工电子工程基础
**作者**：黄宸源，付裕康，尚志远 | 华中科技大学电气与电子工程学院 电工基地2025级
**项目日期**：2026年2月
**开源协议**：MIT License

## 项目概述
本项目基于 Arduino 主控板开发一款多功能智能台灯，融合**环境光感应自动调光**、**人体红外感应开关**、**语音控制**三大核心功能，实现了“人来灯亮、人走灯灭、亮暗自适应、语音可调光”的智能化体验。项目覆盖从原理图设计、KiCad PCB 绘制、代码编写到硬件组装调试的全流程，是电工电子课程的核心实践成果。

## 核心功能
- ✅ 自动感应：人体红外传感器检测到人时自动开灯，无人 30 秒后自动关灯。
- ✅ 自适应调光：光敏电阻采集环境光强度，通过 PWM 信号动态调节 LED 亮度。
- ✅ 语音控制：支持“开灯”“关灯”“调亮”“调暗”等基础语音指令识别。
- ✅ 手动模式：预留按键，可切换至手动常亮/常关模式。

## 硬件清单 (BOM)
| 元器件名称 | 型号/规格 | 数量 |
1	主控板	Arduino Uno R3	1
2	LED 发光二极管	5V 	1
3	光敏电阻	套件中型号	1
4	人体红外传感器	HC-SR505 人体感应	1
5	OLED 显示屏	0.96 寸 I2C 128×64	1
6	蓝牙串口模块	HC-05 通用	1
7	按键	物理按键，套件中包含	1
8	面包板 + 杜邦线	公对母、公对公	1 套
9	电源	Arduino USB 5V 与5V 2A电源适配器供电	1
10	三极管	S8050	1
11	电阻	10kΩ和1kΩ	1|

## 系统架构
五、电路连接说明
LED → D9
光敏电阻 → A0
人体感应 → D2
按键 → D8
OLED SDA→A4，SCL→A5
蓝牙模块 TX→10，RX→11
    六、代码及注释
// 引入必要的硬件驱动库：
// Wire.h → 驱动OLED屏幕的I2C通信
// Adafruit_GFX.h/Adafruit_SSD1306.h → OLED屏幕显示控制
// SoftwareSerial.h → 语音模块的软串口通信
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SoftwareSerial.h>

// ==================== 硬件参数定义（给硬件引脚/屏幕参数起名字，方便理解） ====================
#define SCREEN_WIDTH 128    // OLED屏幕宽度（像素）
#define SCREEN_HEIGHT 64    // OLED屏幕高度（像素）
#define OLED_ADDR 0x3C      // OLED屏幕的I2C地址（大部分是0x3C，少数是0x3D）

#define LDR_PIN A0          // 光敏电阻接A0引脚（检测环境明暗）
#define LED_PIN 9           // LED灯接9引脚（支持PWM调光）
#define PIR_PIN 2           // 人体感应传感器接2引脚（检测有人/无人）
#define KEY_PIN 8           // 物理按键接8引脚（D8）

// 语音模块串口配置：RX接10引脚，TX接11引脚（软串口，不占用硬件串口）
SoftwareSerial voiceSerial(10, 11);
// 初始化OLED屏幕对象（指定宽、高、通信方式）
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ==================== 全局变量（存放程序运行中需要用到的数据） ====================
int ldrVal = 0;             // 光敏电阻读取的值（0-1023，值越小环境越暗）
int ledVal = 0;             // LED最终亮度值（0-255，0=灭，255=最亮）
bool hasPerson = false;     // 是否检测到有人（true=有人，false=无人）

// 模式控制变量（互斥：同一时间只有一个模式生效）
bool manualOn = false;      // 手动模式开（蓝牙/语音/按键开灯）
bool manualOff = false;     // 强制关灯（最高优先级）
bool autoMode = false;      // 自动模式开（光敏+人体感应自动调光）
bool btConnected = true;    // 蓝牙是否连接（10秒没收到指令则认为断开）

unsigned long lastBtCmdTime = 0;  // 最后一次收到蓝牙指令的时间（毫秒）
int manualBright = 200;           // 手动模式默认亮度（你要求的上限200）

unsigned long lastMotionTime = 0; // 最后一次检测到人体的时间（毫秒）
const unsigned long MANUAL_TIMEOUT = 30000;  // 手动模式超时：30秒无人自动关灯
const unsigned long AUTO_TIMEOUT = 10000;    // 自动模式超时：10秒无人自动关灯

// 番茄钟相关变量
bool pomoRunning = false;   // 番茄钟是否运行（true=运行，false=停止）
bool pomoWorkPhase = true;  // 番茄钟阶段：true=工作，false=休息
unsigned long pomoStartTime = 0; // 番茄钟开始时间（毫秒）
int pomoWorkSec = 25 * 60;  // 工作时长：25分钟（转成秒方便计算）
int pomoRestSec = 5 * 60;   // 休息时长：5分钟（转成秒方便计算）

int pomoRemainMin = 0;      // 番茄钟剩余分钟数
int pomoRemainSec = 0;      // 番茄钟剩余秒数

// 按键防抖变量（防止按键按一下触发多次）
bool lastKey = HIGH;        // 上一次按键状态（HIGH=没按，LOW=按下）
unsigned long lastKeyTime = 0;  // 上一次按键触发的时间
const int debounce = 50;    // 防抖时间：50毫秒（按按键后50ms内不重复触发）

// ==================== setup函数：程序启动时只运行一次（初始化硬件） ====================
void setup() {
  Serial.begin(9600);       // 硬件串口初始化（波特率9600，用于蓝牙通信）
  voiceSerial.begin(9600);  // 软串口初始化（波特率9600，用于语音模块通信）
  voiceSerial.setTimeout(50); // 语音指令读取超时：50毫秒

  // 设置引脚模式：INPUT=输入（读取数据），OUTPUT=输出（控制设备）
  pinMode(LDR_PIN, INPUT);      // 光敏电阻：输入（读环境亮度）
  pinMode(LED_PIN, OUTPUT);     // LED灯：输出（控亮度）
  pinMode(PIR_PIN, INPUT);      // 人体感应：输入（读有人/无人）
  pinMode(KEY_PIN, INPUT_PULLUP); // 按键：上拉输入（没按=HIGH，按下=LOW）

  // 初始化OLED屏幕
  Wire.begin();  // 启动I2C通信
  bool oledOk = oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR); // 尝试用0x3C地址初始化
  if (!oledOk) oledOk = oled.begin(SSD1306_SWITCHCAPVCC, 0x3D); // 失败则试0x3D

  if (oledOk) { // 初始化成功
    oled.clearDisplay();    // 清屏
    oled.setTextColor(WHITE); // 设置文字颜色：白色
    oled.setTextSize(1);    // 设置文字大小：1号（越小越清晰）
  }
  delay(500);  // 延时500ms，让硬件稳定
}

// ==================== loop函数：程序启动后无限循环运行（核心逻辑） ====================
void loop() {
  // ==================== 物理按键控制逻辑（最高优先级） ====================
  int key = digitalRead(KEY_PIN); // 读取当前按键状态（HIGH/LOW）
  // 按键触发条件：当前按下（LOW）、上一次没按（HIGH）、超过防抖时间
  if (key == LOW && lastKey == HIGH && millis() - lastKeyTime > debounce) {
    lastKeyTime = millis(); // 更新按键触发时间
    // 核心逻辑：灯亮则灭，灯灭则亮
    if (ledVal != 0) { // 如果LED亮度≠0（灯亮）
      manualOff = true;    // 强制关灯
      manualOn = false;    // 关闭手动模式
      autoMode = false;    // 关闭自动模式
      pomoRunning = false; // 关闭番茄钟
    } else { // 如果LED亮度=0（灯灭）
      manualOn = true;     // 打开手动模式
      manualOff = false;   // 取消强制关灯
      autoMode = false;    // 关闭自动模式
      pomoRunning = false; // 关闭番茄钟
      manualBright = 200;  // 开灯默认亮度：200（你要求的上限）
      lastMotionTime = millis(); // 重置超时时间（避免刚开灯就超时）
    }
  }
  lastKey = key; // 更新上一次按键状态

  // ==================== 依次执行各功能模块 ====================
  parseBtCommand();      // 处理蓝牙指令（核心：AUTO/POMO/B/M/D等）
  parseVoiceCommand();   // 处理语音模块指令
  updateBtStatus();      // 更新蓝牙连接状态
  updatePomodoro();      // 更新番茄钟计时（工作/休息切换）
  calculatePomoRemain(); // 计算番茄钟剩余时间

  // ==================== 人体感应检测（番茄钟运行时不检测） ====================
  if (!pomoRunning) {
    bool currPir = digitalRead(PIR_PIN) == HIGH; // 读取人体感应状态
    if (currPir) { // 如果检测到有人
      lastMotionTime = millis(); // 更新最后一次有人的时间
      hasPerson = true;          // 标记：有人
    } else { // 如果没检测到有人
      hasPerson = false;         // 标记：无人
    }
  }

  // ==================== 计算LED最终亮度 + 控制LED + 更新屏幕 ====================
  calculateLedBrightness(); // 核心：根据模式计算LED亮度
  analogWrite(LED_PIN, ledVal); // 控制LED亮度（PWM输出）
  updateOLED(); // 更新OLED屏幕显示内容
  delay(30);  // 延时30ms，让程序运行更稳定（减少卡顿）
}

// ==================== calculateLedBrightness：计算LED最终亮度（核心逻辑） ====================
void calculateLedBrightness() {
  // 1. 强制关灯（最高优先级）：只要manualOff=true，灯就灭
  if (manualOff) {
    ledVal = 0;
    return; // 直接返回，不执行后面的逻辑
  }

  // 2. 番茄钟模式：允许手动调亮度（你要求的核心功能）
  if (pomoRunning) {
    if (manualOn) {
      ledVal = manualBright; // 番茄钟运行时，用蓝牙调的亮度（B/M/D）
    } else {
      ledVal = pomoWorkPhase ? 200 : 50; // 没手动调时，工作亮（200）、休息暗（50）
    }
    return;
  }

  // 3. 超时判断：自动/手动模式超时则关灯
  unsigned long nowMs = millis(); // 当前时间（毫秒）
  // 自动模式超时：10秒没人；手动模式超时：30秒没人
  bool timeout = autoMode ? (nowMs - lastMotionTime > AUTO_TIMEOUT) : (nowMs - lastMotionTime > MANUAL_TIMEOUT);
  if (timeout) {
    ledVal = 0;           // 超时关灯
    manualOn = false;     // 关闭手动模式
    manualOff = true;     // 标记强制关灯
    return;
  }

  // 4. 手动模式：用蓝牙/语音/按键设置的亮度
  if (manualOn) {
    ledVal = manualBright;
    return;
  }

  // 5. 自动模式（有人时）：根据光敏电阻调光
  if (autoMode && hasPerson) {
    ldrVal = analogRead(LDR_PIN); // 读取光敏电阻值（0-1023）
    // 映射：环境越暗（ldrVal越小）→ LED越亮（200）；环境越亮→ LED越暗（50）
    ledVal = map(ldrVal, 0, 1023, 200, 50);
    ledVal = constrain(ledVal, 50, 200); // 限制亮度在50~200之间（你要求的范围）
    return;
  }

  // 6. 默认状态：关灯
  ledVal = 0;
}

// ==================== parseBtCommand：处理蓝牙串口指令（你最关心的部分） ====================
void parseBtCommand() {
  if (Serial.available() == 0) return; // 如果串口没数据，直接返回

  String cmd = Serial.readStringUntil('\n'); // 读取蓝牙指令（直到换行）
  cmd.trim(); // 去掉指令前后的空格/换行
  cmd.toUpperCase(); // 转成大写（避免大小写问题，比如auto/AUTO都能识别）

  lastBtCmdTime = millis(); // 更新最后一次蓝牙指令时间（标记蓝牙连接）
  btConnected = true;       // 标记蓝牙已连接

  // ========== 1. AUTO：进入自动模式（强制生效） ==========
  if (cmd == "AUTO") {
    manualOff = false;
    manualOn = false;
    pomoRunning = false;  // 自动模式必须停止番茄钟
    autoMode = true;
    lastMotionTime = millis();
    return;
  }

  // ========== 2. POMO：启动番茄钟 ==========
  if (cmd == "POMO") {
    pomoRunning = true;
    pomoWorkPhase = true;   // 初始阶段：工作
    pomoStartTime = millis(); // 记录番茄钟开始时间
    manualOff = false;
    manualOn = true;    // 启动后允许手动调光
    autoMode = false;
    lastMotionTime = millis();
    return;
  }

  // ========== 3. POMOSTOP：停止番茄钟（只有这个指令能停番茄钟） ==========
  if (cmd == "POMOSTOP") {
    pomoRunning = false;
    return;
  }

  // ========== 4. WORKxx：设置工作时长（比如WORK30=30分钟） ==========
  if (cmd.startsWith("WORK")) {
    int m = cmd.substring(4).toInt(); // 截取WORK后面的数字
    if (m >= 1 && m <= 99) pomoWorkSec = m * 60; // 转成秒，限制1-99分钟
    return;
  }

  // ========== 5. RESTxx：设置休息时长（比如REST10=10分钟） ==========
  if (cmd.startsWith("REST")) {
    int m = cmd.substring(4).toInt(); // 截取REST后面的数字
    if (m >= 1 && m <= 99) pomoRestSec = m * 60; // 转成秒，限制1-99分钟
    return;
  }

  // ========== 6. OFF：关灯 ==========
  if (cmd == "OFF") {
    manualOff = true;
    manualOn = false;
    autoMode = false;
    pomoRunning = false;
  }

  // ========== 7. ON：开灯（手动模式，最亮200） ==========
  else if (cmd == "ON") {
    manualOff = false;
    manualOn = true;
    autoMode = false;
    manualBright = 200;
    lastMotionTime = millis();
  }

  // ========== 8. 亮度调节：番茄钟运行时也能调，不退出番茄钟！ ==========
  else if (cmd == "D") { // D=暗（50）
    manualOff = false;
    manualOn = true;
    autoMode = false;
    manualBright = 50;
    lastMotionTime = millis();
  }
  else if (cmd == "M") { // M=中（110）
    manualOff = false;
    manualOn = true;
    autoMode = false;
    manualBright = 110;
    lastMotionTime = millis();
  }
  else if (cmd == "B") { // B=亮（200）
    manualOff = false;
    manualOn = true;
    autoMode = false;
    manualBright = 200;
    lastMotionTime = millis();
  }
}

// ==================== parseVoiceCommand：处理语音模块指令 ====================
void parseVoiceCommand() {
  if (voiceSerial.available() == 0) return; // 语音串口没数据，直接返回

  String vc = voiceSerial.readStringUntil('\n'); // 读取语音指令
  vc.trim(); // 去掉空格/换行
  vc.toUpperCase(); // 转大写

  lastBtCmdTime = millis(); // 更新蓝牙连接时间（语音和蓝牙共用连接状态）
  btConnected = true;

  // 语音指令逻辑和蓝牙一致
  if (vc.indexOf("AUTO") != -1) { // 指令包含AUTO=进入自动模式
    manualOff = false;
    manualOn = false;
    pomoRunning = false;
    autoMode = true;
    lastMotionTime = millis();
    return;
  }
  if (vc.indexOf("OFF") != -1) { // 包含OFF=关灯
    manualOff = true;
    manualOn = false;
    autoMode = false;
    pomoRunning = false;
  }
  else if (vc.indexOf("ON") != -1) { // 包含ON=开灯
    manualOff = false;
    manualOn = true;
    autoMode = false;
    manualBright = 200;
    lastMotionTime = millis();
  }
  else if (vc.indexOf("D") != -1) { // 包含D=暗（50）
    manualOff = false;
    manualOn = true;
    autoMode = false;
    manualBright = 50;
    lastMotionTime = millis();
  }
  else if (vc.indexOf("M") != -1) { // 包含M=中（110）
    manualOff = false;
    manualOn = true;
    autoMode = false;
    manualBright = 110;
    lastMotionTime = millis();
  }
  else if (vc.indexOf("B") != -1) { // 包含B=亮（200）
    manualOff = false;
    manualOn = true;
    autoMode = false;
    manualBright = 200;
    lastMotionTime = millis();
  }
}

// ==================== updatePomodoro：更新番茄钟状态（工作/休息切换） ====================
void updatePomodoro() {
  if (!pomoRunning) return; // 番茄钟没运行，直接返回

  unsigned long now = millis(); // 当前时间
  unsigned long elapsedSec = (now - pomoStartTime) / 1000; // 已运行秒数
  // 目标时长：工作阶段=工作时长，休息阶段=休息时长
  int targetSec = pomoWorkPhase ? pomoWorkSec : pomoRestSec;

  // 已运行时间≥目标时长，切换阶段
  if (elapsedSec >= targetSec) {
    pomoWorkPhase = !pomoWorkPhase; // 翻转阶段（工作→休息，休息→工作）
    pomoStartTime = now; // 重置番茄钟开始时间
  }
}

// ==================== calculatePomoRemain：计算番茄钟剩余时间 ====================
void calculatePomoRemain() {
  if (!pomoRunning) { // 番茄钟没运行，剩余时间=0
    pomoRemainMin = 0;
    pomoRemainSec = 0;
    return;
  }

  unsigned long now = millis();
  unsigned long elapsedSec = (now - pomoStartTime) / 1000; // 已运行秒数
  int targetSec = pomoWorkPhase ? pomoWorkSec : pomoRestSec; // 目标时长
  int remainSec = targetSec - elapsedSec; // 剩余秒数

  pomoRemainMin = max(0, remainSec / 60); // 剩余分钟（转成分钟，最小0）
  pomoRemainSec = max(0, remainSec % 60); // 剩余秒数（取余数，最小0）
}

// ==================== updateBtStatus：更新蓝牙连接状态 ====================
void updateBtStatus() {
  // 10秒没收到蓝牙/语音指令，认为蓝牙断开
  btConnected = (millis() - lastBtCmdTime) < 10000;
}

// ==================== updateOLED：更新OLED屏幕显示内容 ====================
void updateOLED() {
  oled.clearDisplay(); // 清屏（每次更新前先清屏）

  // 第1行：显示当前模式 + 蓝牙状态
  oled.setCursor(0, 0); // 设置光标位置：x=0，y=0（左上角）
  oled.print("Mode: ");
  if (autoMode) oled.print("AUTO");       // 自动模式
  else if (manualOn) oled.print("MANUAL");// 手动模式
  else oled.print("OFF");                 // 关灯
  oled.setCursor(70, 0); // 光标x=70，y=0
  oled.print("BT:"); oled.print(btConnected ? "CON" : "DIS"); // CON=连接，DIS=断开

  // 第2行：显示LED亮度 + 人体感应状态
  oled.setCursor(0, 12); // y=12（换行，OLED每行约12像素）
  oled.print("B:"); oled.print(ledVal); // B=Brightness（亮度）
  oled.print("  Human:"); oled.print(hasPerson ? "YES" : "NO"); // YES=有人，NO=无人

  // 第3行：显示番茄钟状态
  oled.setCursor(0, 24);
  oled.print("POMO:");
  if (pomoRunning) oled.print(pomoWorkPhase ? "WORK" : "REST"); // WORK=工作，REST=休息
  else oled.print("STOP"); // STOP=停止

  // 第4行：显示工作/休息时长
  oled.setCursor(0, 36);
  oled.print("W:"); oled.print(pomoWorkSec / 60); // W=Work（工作分钟）
  oled.print("m R:"); oled.print(pomoRestSec / 60); oled.print("m"); // R=Rest（休息分钟）

  // 第5行：显示番茄钟剩余时间
  oled.setCursor(0, 48);
  if (pomoRunning) {
    oled.print(pomoWorkPhase ? "W " : "R "); // 标记当前阶段
    if (pomoRemainMin < 10) oled.print("0"); // 补0（比如9分钟→09）
    oled.print(pomoRemainMin);
    oled.print(":");
    if (pomoRemainSec < 10) oled.print("0"); // 补0（比如5秒→05）
    oled.print(pomoRemainSec);
  }

  oled.display(); // 把内容显示到OLED屏幕上（必须调用，否则不显示）
}
