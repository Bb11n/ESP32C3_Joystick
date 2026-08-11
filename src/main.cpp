/*
 * 工程：ESP32-C3 摇杆 BLE 遥控器（深度睡眠版）
 *
 * 功能：
 * 1. BLE Server，设备名称保持为 IG_REMOTE_S3，兼容现有 IG 接口。
 * 2. IG 连接后持续发送当前三字节协议 AA 01 XX。
 * 3. 初始/重新连接后发送 AA 01 FF。
 * 4. 摇杆方向改变后锁存命令；摇杆回中不清除上一条命令。
 * 5. 摇杆 SW 短按：AA 01 00（主页）。
 * 6. 摇杆 SW 长按 2 秒：发送 5 次 AA 01 FF，松开后进入深度睡眠。
 * 7. 深度睡眠后按一下 SW：GPIO3 唤醒，重新运行 setup()、初始化 BLE 并广播。
 *
 * 接线：
 * VRX -> GPIO4
 * VRY -> GPIO5
 * SW  -> GPIO3
 * VCC -> 3.3V
 * GND -> GND
 *
 * 建议：GPIO3 与 3.3V 之间增加 10kΩ 外部上拉电阻，提高深度睡眠唤醒可靠性。
 */

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "esp_sleep.h"
#include "driver/gpio.h"

#if !CONFIG_IDF_TARGET_ESP32C3
#error "请选择 ESP32C3 Dev Module 编译本工程"
#endif

// ========================== 硬件引脚 ==========================
#define JOY_X_PIN   4
#define JOY_Y_PIN   0
#define JOY_SW_PIN  3

// ========================== BLE 接口 ==========================
#define DEVICE_NAME "IG_REMOTE_S3"

#define SERVICE_UUID \
  "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"

#define CHARACTERISTIC_UUID_RX \
  "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"

#define CHARACTERISTIC_UUID_TX \
  "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

// ========================== 协议命令 ==========================
constexpr uint8_t CMD_HOME  = 0x00;
constexpr uint8_t CMD_DOWN  = 0x01;
constexpr uint8_t CMD_LEFT  = 0x02;
constexpr uint8_t CMD_RIGHT = 0x03;
constexpr uint8_t CMD_UP    = 0x04;
constexpr uint8_t CMD_BLANK = 0xFF;

// ========================== 时间参数 ==========================
constexpr unsigned long SEND_INTERVAL_MS      = 20;
constexpr unsigned long JOYSTICK_INTERVAL_MS  = 2;
constexpr unsigned long BUTTON_DEBOUNCE_MS    = 15;
constexpr unsigned long LONG_PRESS_MS         = 2000;

// ========================== 摇杆参数 ==========================
int centerX = 2048;
int centerY = 2048;
constexpr int DIRECTION_THRESHOLD = 350;

// ========================== BLE 状态 ==========================
BLEServer* bleServer = nullptr;
BLECharacteristic* txCharacteristic = nullptr;

volatile bool deviceConnected = false;
volatile bool restartAdvertisingRequested = false;

uint8_t currentCommand = CMD_BLANK;
unsigned long lastSendTime = 0;
unsigned long lastJoystickTime = 0;

// ========================== 按键状态 ==========================
bool lastRawButtonState = HIGH;
bool stableButtonState = HIGH;
unsigned long buttonChangeTime = 0;
unsigned long buttonPressStartTime = 0;
bool longPressTriggered = false;

// 深度睡眠期间保留
RTC_DATA_ATTR uint32_t bootCount = 0;

// ========================== BLE 回调 ==========================
class RemoteServerCallbacks : public BLEServerCallbacks {
 public:
  void onConnect(BLEServer* server) override {
    (void)server;
    deviceConnected = true;
    currentCommand = CMD_BLANK;
    lastSendTime = 0;

    Serial.println();
    Serial.println("================================");
    Serial.println("IG 连接成功");
    Serial.println("开始持续发送：AA 01 FF");
    Serial.println("================================");
  }

  void onDisconnect(BLEServer* server) override {
    (void)server;
    deviceConnected = false;
    restartAdvertisingRequested = true;
    currentCommand = CMD_BLANK;

    Serial.println();
    Serial.println("================================");
    Serial.println("IG 连接断开");
    Serial.println("准备重新启动 BLE 广播");
    Serial.println("================================");
  }
};

// ========================== 协议发送 ==========================
void printProtocol(uint8_t command) {
  Serial.printf("AA 01 %02X\n", command);
}

void sendProtocol(uint8_t command) {
  if (!deviceConnected || txCharacteristic == nullptr) {
    return;
  }

  const uint8_t frame[3] = {0xAA, 0x01, command};
  txCharacteristic->setValue(const_cast<uint8_t*>(frame), sizeof(frame));
  txCharacteristic->notify();
  printProtocol(command);
}

void sendProtocolSeveralTimes(uint8_t command, int count) {
  if (!deviceConnected) {
    return;
  }

  for (int i = 0; i < count; ++i) {
    sendProtocol(command);
    delay(20);
  }
}

void changeCommand(uint8_t newCommand, const char* name) {
  if (currentCommand == newCommand) {
    return;
  }

  currentCommand = newCommand;

  Serial.println();
  Serial.print("切换指令：");
  Serial.println(name);
  Serial.print("当前协议：");
  printProtocol(currentCommand);

  sendProtocol(currentCommand);
  lastSendTime = millis();
}

// ========================== 摇杆校准 ==========================
void calibrateJoystick() {
  long totalX = 0;
  long totalY = 0;

  Serial.println();
  Serial.println("请保持摇杆回中，正在校准……");
  delay(300);

  for (int i = 0; i < 100; ++i) {
    totalX += analogRead(JOY_X_PIN);
    totalY += analogRead(JOY_Y_PIN);
    delay(2);
  }

  centerX = static_cast<int>(totalX / 100);
  centerY = static_cast<int>(totalY / 100);

  Serial.print("X 中心值：");
  Serial.println(centerX);
  Serial.print("Y 中心值：");
  Serial.println(centerY);
}

// ========================== BLE 初始化 ==========================
void initializeBLE() {
  Serial.println();
  Serial.println("开始初始化 BLE……");

  BLEDevice::init(DEVICE_NAME);

  bleServer = BLEDevice::createServer();
  bleServer->setCallbacks(new RemoteServerCallbacks());

  BLEService* service = bleServer->createService(SERVICE_UUID);

  txCharacteristic = service->createCharacteristic(
      CHARACTERISTIC_UUID_TX,
      BLECharacteristic::PROPERTY_READ |
      BLECharacteristic::PROPERTY_NOTIFY);

  txCharacteristic->addDescriptor(new BLE2902());

  const uint8_t initialFrame[3] = {0xAA, 0x01, CMD_BLANK};
  txCharacteristic->setValue(
      const_cast<uint8_t*>(initialFrame), sizeof(initialFrame));

  service->createCharacteristic(
      CHARACTERISTIC_UUID_RX,
      BLECharacteristic::PROPERTY_WRITE |
      BLECharacteristic::PROPERTY_WRITE_NR);

  service->start();

  BLEAdvertising* advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(SERVICE_UUID);
  advertising->setScanResponse(true);
  advertising->start();

  currentCommand = CMD_BLANK;
  deviceConnected = false;
  restartAdvertisingRequested = false;

  Serial.println("BLE 初始化完成");
  Serial.println("设备名称：IG_REMOTE_S3");
  Serial.println("BLE 广播已启动");
  Serial.println("等待 IG 连接……");
}

void updateAdvertising() {
  if (!restartAdvertisingRequested) {
    return;
  }

  restartAdvertisingRequested = false;
  delay(500);

  BLEDevice::startAdvertising();

  Serial.println();
  Serial.println("BLE 广播已重新启动");
  Serial.println("等待 IG 重新连接……");
}

// ========================== 深度睡眠 ==========================
bool configureWakeup() {
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);

  gpio_pullup_en(GPIO_NUM_3);
  gpio_pulldown_dis(GPIO_NUM_3);

  const esp_err_t result = esp_deep_sleep_enable_gpio_wakeup(
      1ULL << JOY_SW_PIN,
      ESP_GPIO_WAKEUP_GPIO_LOW);

  Serial.print("GPIO3 唤醒配置结果：");
  Serial.println(static_cast<int>(result));

  if (result != ESP_OK) {
    Serial.println("GPIO3 唤醒配置失败");
    return false;
  }

  Serial.println("GPIO3 低电平唤醒配置成功");
  return true;
}

void enterDeepSleep() {
  Serial.println();
  Serial.println("================================");
  Serial.println("检测到长按 2 秒");
  Serial.println("准备进入深度睡眠");
  Serial.println("================================");

  currentCommand = CMD_BLANK;
  sendProtocolSeveralTimes(CMD_BLANK, 5);

  Serial.println("请松开摇杆按键……");
  while (digitalRead(JOY_SW_PIN) == LOW) {
    delay(10);
  }

  delay(200);

  const int pinLevel = digitalRead(JOY_SW_PIN);
  Serial.print("进入睡眠前 GPIO3 电平：");
  Serial.println(pinLevel);

  if (pinLevel != HIGH) {
    Serial.println("GPIO3 没有恢复高电平，取消睡眠");
    return;
  }

  if (!configureWakeup()) {
    Serial.println("唤醒源配置失败，取消睡眠");
    return;
  }

  Serial.println();
  Serial.println("正式进入深度睡眠");
  Serial.println("BLE 连接会断开");
  Serial.println("睡眠后按一下 SW 即可唤醒");
  Serial.flush();
  delay(100);

  esp_deep_sleep_start();
}

void processWakeupReason() {
  const esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();

  Serial.println();
  Serial.print("启动次数：");
  Serial.println(bootCount);
  Serial.print("唤醒原因编号：");
  Serial.println(static_cast<int>(cause));

  if (cause == ESP_SLEEP_WAKEUP_GPIO) {
    Serial.println("GPIO3 按键唤醒成功");
    Serial.println("请松开唤醒按键……");

    while (digitalRead(JOY_SW_PIN) == LOW) {
      delay(10);
    }

    delay(200);
    Serial.println("按键已经松开");
    Serial.println("继续校准摇杆并重新初始化 BLE");
  } else {
    Serial.println("正常上电或复位启动");
  }
}

// ========================== 输入处理 ==========================
void updateJoystick() {
  const unsigned long now = millis();

  if (now - lastJoystickTime < JOYSTICK_INTERVAL_MS) {
    return;
  }
  lastJoystickTime = now;

  const int xValue = analogRead(JOY_X_PIN);
  const int yValue = analogRead(JOY_Y_PIN);
  const int dx = xValue - centerX;
  const int dy = yValue - centerY;

  // 回中不清除上一条命令
  if (abs(dx) < DIRECTION_THRESHOLD &&
      abs(dy) < DIRECTION_THRESHOLD) {
    return;
  }

  if (abs(dx) > abs(dy)) {
    if (dx < -DIRECTION_THRESHOLD) {
      changeCommand(CMD_LEFT, "左：AA 01 02");
    } else if (dx > DIRECTION_THRESHOLD) {
      changeCommand(CMD_RIGHT, "右：AA 01 03");
    }
  } else {
    if (dy < -DIRECTION_THRESHOLD) {
      changeCommand(CMD_UP, "上：AA 01 04");
    } else if (dy > DIRECTION_THRESHOLD) {
      changeCommand(CMD_DOWN, "下：AA 01 01");
    }
  }
}

void updateJoystickButton() {
  const bool rawState = digitalRead(JOY_SW_PIN);
  const unsigned long now = millis();

  if (rawState != lastRawButtonState) {
    lastRawButtonState = rawState;
    buttonChangeTime = now;
  }

  if ((now - buttonChangeTime >= BUTTON_DEBOUNCE_MS) &&
      rawState != stableButtonState) {
    stableButtonState = rawState;

    if (stableButtonState == LOW) {
      buttonPressStartTime = now;
      longPressTriggered = false;
      Serial.println();
      Serial.println("摇杆按键按下");
    } else {
      if (!longPressTriggered) {
        Serial.println("摇杆按键短按：返回主页");
        changeCommand(CMD_HOME, "主页：AA 01 00");
      }
      longPressTriggered = false;
    }
  }

  if (stableButtonState == LOW &&
      !longPressTriggered &&
      now - buttonPressStartTime >= LONG_PRESS_MS) {
    longPressTriggered = true;
    enterDeepSleep();
  }
}

void sendCurrentCommandContinuously() {
  if (!deviceConnected) {
    return;
  }

  const unsigned long now = millis();
  if (now - lastSendTime < SEND_INTERVAL_MS) {
    return;
  }

  lastSendTime = now;
  sendProtocol(currentCommand);
}

// ========================== Arduino 入口 ==========================
void setup() {
  Serial.begin(115200);
  delay(500);

  ++bootCount;

  pinMode(JOY_SW_PIN, INPUT_PULLUP);
  gpio_pullup_en(GPIO_NUM_3);
  gpio_pulldown_dis(GPIO_NUM_3);
  analogReadResolution(12);

  Serial.println();
  Serial.println("================================");
  Serial.println("ESP32-C3 摇杆 BLE 深度睡眠版");
  Serial.println("短按 SW：AA 01 00");
  Serial.println("长按 SW 2 秒：进入深度睡眠");
  Serial.println("睡眠后按一下 SW：唤醒");
  Serial.println("================================");

  processWakeupReason();
  calibrateJoystick();

  lastRawButtonState = digitalRead(JOY_SW_PIN);
  stableButtonState = lastRawButtonState;
  buttonChangeTime = millis();
  buttonPressStartTime = millis();
  longPressTriggered = false;

  initializeBLE();
}

void loop() {
  updateAdvertising();
  updateJoystickButton();
  updateJoystick();
  sendCurrentCommandContinuously();
  delay(1);
}
