/*
 * ESP32 差速小车固件 — N20 编码器电机 + L298N 驱动
 *
 * 通信协议（Wi-Fi UDP，ESP32 作为热点）：
 *   PC → ESP32:  "M <左PWM> <右PWM>\n"    例: "M 150 150\n"（前进）
 *   ESP32 → PC:  "E <左脉冲> <右脉冲>\n"   每 50ms 发送一次
 *   ESP32 → PC:  "I <ax> <ay> <az> <gx> <gy> <gz>\n"  IMU 数据（如有）
 *
 * 电脑连接热点 car-esp32（密码 carcontrol），ESP32 地址固定为
 * 192.168.4.1。电机和编码器使用 UDP 8888；雷达原始字节流使用
 * TCP 8889。
 *
 * 引脚接线：
 *   L298N: ENA=26, IN1=27, IN2=14, IN3=32, IN4=33, ENB=25
 *   编码器: 左A=13, 左B=23, 右A=18, 右B=19
 */

#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>

// ===== Wi-Fi / UDP =====
const char *WIFI_SSID = "car-esp32";
const char *WIFI_PASSWORD = "carcontrol";  // 至少 8 个字符
const uint16_t UDP_PORT = 8888;
const uint16_t LIDAR_TCP_PORT = 8889;
const unsigned long COMMAND_TIMEOUT = 500;  // 超过 500ms 未收到命令则停车
const unsigned long IMU_SEND_INTERVAL = 20; // 50 Hz

// 雷达 UART2：雷达 TX → GPIO16，雷达 RX ← GPIO17。
const int LIDAR_RX_PIN = 16;
const int LIDAR_TX_PIN = 17;
const uint32_t LIDAR_BAUDRATE = 150000;

// GY-521 / MPU-6050 I2C。模块请接 3V3，避免把 5V 逻辑送进 ESP32 GPIO。
const int IMU_SDA_PIN = 21;
const int IMU_SCL_PIN = 22;
const uint8_t MPU6050_ADDR_LOW = 0x68;
const uint8_t MPU6050_ADDR_HIGH = 0x69;
const uint8_t MPU6050_REG_ACCEL_XOUT_H = 0x3B;
const uint8_t MPU6050_REG_GYRO_CONFIG = 0x1B;
const uint8_t MPU6050_REG_ACCEL_CONFIG = 0x1C;
const uint8_t MPU6050_REG_CONFIG = 0x1A;
const uint8_t MPU6050_REG_PWR_MGMT_1 = 0x6B;

WiFiUDP udp;
WiFiServer lidar_server(LIDAR_TCP_PORT);
WiFiClient lidar_client;
HardwareSerial LidarSerial(2);
IPAddress pc_ip;
uint16_t pc_port = 0;
bool pc_connected = false;
unsigned long last_command = 0;
unsigned long last_imu_send = 0;
uint8_t mpu_address = 0;
bool mpu_ready = false;

// ===== 电机引脚 =====
const int ENA = 26, IN1 = 27, IN2 = 14;  // 左电机
const int ENB = 25, IN3 = 32, IN4 = 33;  // 右电机

// ===== 编码器引脚 =====
// 左编码器不再使用 GPIO34/35：它们没有内部上拉，会令霍尔信号悬空。
const int ENC_L_A = 13, ENC_L_B = 23;
const int ENC_R_A = 18, ENC_R_B = 19;

// ===== 编码器计数器 =====
volatile long enc_left  = 0;
volatile long enc_right = 0;

// ===== 电机参数 =====
const int PWM_FREQ = 5000;
const int PWM_RES  = 8;   // 0-255
const int PWM_MAX  = 255;

// ===== 定时发送 =====
unsigned long last_send = 0;
const unsigned long SEND_INTERVAL = 50;  // ms

// ===== 前置声明（Arduino 预处理会重排函数顺序） =====
enum Motor { LEFT, RIGHT };
void set_motor(Motor side, int pwm);
void stop();
void handle_udp();
void send_encoder();
void handle_lidar_relay();
bool init_mpu6050();
bool read_mpu6050(float& ax, float& ay, float& az,
                  float& gx, float& gy, float& gz);
void send_imu();

void IRAM_ATTR on_enc_left() {
  if (digitalRead(ENC_L_B)) enc_left++; else enc_left--;
}

void IRAM_ATTR on_enc_right() {
  if (digitalRead(ENC_R_B)) enc_right++; else enc_right--;
}


void setup() {
  Serial.begin(115200);

  // ESP32 自己创建热点；电脑端 wifi_bridge.py 的默认地址就是 192.168.4.1。
  WiFi.mode(WIFI_AP);
  if (!WiFi.softAP(WIFI_SSID, WIFI_PASSWORD)) {
    Serial.println("Failed to start Wi-Fi AP");
  }
  udp.begin(UDP_PORT);
  lidar_server.begin();

  // 先扩大接收缓冲区，再打开 UART，避免 Wi-Fi 调度造成雷达字节丢失。
  LidarSerial.setRxBufferSize(4096);
  LidarSerial.begin(LIDAR_BAUDRATE, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);

  Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);
  Wire.setClock(400000);
  mpu_ready = init_mpu6050();
  Serial.print("Wi-Fi AP ready: ");
  Serial.println(WiFi.softAPIP());

  // ---- 电机引脚 ----
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  ledcSetup(0, PWM_FREQ, PWM_RES); ledcAttachPin(ENA, 0);
  ledcSetup(1, PWM_FREQ, PWM_RES); ledcAttachPin(ENB, 1);

  // ---- 编码器引脚 ----
  pinMode(ENC_L_A, INPUT_PULLUP); pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP); pinMode(ENC_R_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), on_enc_left,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), on_enc_right, CHANGE);

  stop();
  Serial.println("ESP32 ready (UDP)");
}


void loop() {
  // ---- 1) 接收 PC 的 UDP 电机命令 ----
  handle_udp();

  // ---- 2) 双向转发雷达 UART 和电脑 TCP ----
  handle_lidar_relay();

  // Wi-Fi 断开或电脑程序退出时，绝不能保持上一次 PWM 继续行驶。
  if (pc_connected && millis() - last_command > COMMAND_TIMEOUT) {
    stop();
  }

  // ---- 3) 定时发送编码器数据 ----
  if (millis() - last_send >= SEND_INTERVAL) {
    last_send = millis();
    send_encoder();
  }

  if (mpu_ready && millis() - last_imu_send >= IMU_SEND_INTERVAL) {
    last_imu_send = millis();
    send_imu();
  }
}


// ==================== Wi-Fi UDP ====================

void handle_udp() {
  int packet_size;
  while ((packet_size = udp.parsePacket()) > 0) {
    char packet[64];
    int n = udp.read(packet, min(packet_size, (int)sizeof(packet) - 1));
    if (n <= 0) continue;
    packet[n] = '\0';

    int left, right;
    if (sscanf(packet, "M %d %d", &left, &right) == 2) {
      pc_ip = udp.remoteIP();
      pc_port = udp.remotePort();
      pc_connected = true;
      last_command = millis();
      set_motor(LEFT, left);
      set_motor(RIGHT, right);
    }
  }
}

void send_encoder() {
  if (!pc_connected) return;

  long left, right;
  noInterrupts();
  left = enc_left;
  right = enc_right;
  interrupts();

  char message[48];
  int n = snprintf(message, sizeof(message), "E %ld %ld\n", left, right);
  if (n <= 0 || n >= (int)sizeof(message)) return;

  udp.beginPacket(pc_ip, pc_port);
  udp.write((const uint8_t *)message, n);
  udp.endPacket();
}

void handle_lidar_relay() {
  // 电脑端 lidar_node 以 TCP 客户端连接到 ESP32:8889。
  if (!lidar_client || !lidar_client.connected()) {
    if (lidar_client) lidar_client.stop();
    WiFiClient candidate = lidar_server.available();
    if (candidate) {
      lidar_client = candidate;
      lidar_client.setNoDelay(true);
      Serial.println("LiDAR TCP client connected");
    }
    return;
  }

  // 电脑 → 雷达：转发启动/停止等雷达控制命令。
  while (lidar_client.available() && LidarSerial.availableForWrite()) {
    LidarSerial.write(lidar_client.read());
  }

  // 雷达 → 电脑：保持原始二进制字节，不插入日志或文本协议。
  uint8_t buffer[256];
  size_t count = 0;
  while (count < sizeof(buffer) && LidarSerial.available()) {
    int value = LidarSerial.read();
    if (value >= 0) buffer[count++] = static_cast<uint8_t>(value);
  }
  if (count > 0) lidar_client.write(buffer, count);
}


// ==================== GY-521 / MPU-6050 ====================

bool write_mpu_register(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(mpu_address);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool init_mpu6050() {
  // AD0 接 GND 时地址为 0x68，接 VCC 时为 0x69；两种都自动尝试。
  for (uint8_t address : {MPU6050_ADDR_LOW, MPU6050_ADDR_HIGH}) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      mpu_address = address;
      break;
    }
  }
  if (mpu_address == 0) {
    Serial.println("MPU-6050 not found; IMU telemetry disabled");
    return false;
  }

  // 退出睡眠，使用陀螺仪 X 轴 PLL 时钟；量程为 ±2g、±250 deg/s。
  if (!write_mpu_register(MPU6050_REG_PWR_MGMT_1, 0x01) ||
      !write_mpu_register(MPU6050_REG_CONFIG, 0x03) ||
      !write_mpu_register(MPU6050_REG_GYRO_CONFIG, 0x00) ||
      !write_mpu_register(MPU6050_REG_ACCEL_CONFIG, 0x00)) {
    Serial.println("MPU-6050 configuration failed");
    return false;
  }
  Serial.printf("MPU-6050 ready at I2C 0x%02X\n", mpu_address);
  return true;
}

bool read_mpu6050(float& ax, float& ay, float& az,
                  float& gx, float& gy, float& gz) {
  Wire.beginTransmission(mpu_address);
  Wire.write(MPU6050_REG_ACCEL_XOUT_H);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  uint8_t received = Wire.requestFrom(mpu_address, static_cast<uint8_t>(14), static_cast<uint8_t>(1));
  if (received != 14) {
    return false;
  }

  int16_t raw[7];
  for (int i = 0; i < 7; ++i) {
    raw[i] = static_cast<int16_t>((Wire.read() << 8) | Wire.read());
  }

  constexpr float GRAVITY = 9.80665f;
  constexpr float DEG_TO_RAD_F = 0.017453292519943295f;
  // ACCEL_CONFIG=0 → 16384 LSB/g；GYRO_CONFIG=0 → 131 LSB/(deg/s)。
  ax = raw[0] / 16384.0f * GRAVITY;
  ay = raw[1] / 16384.0f * GRAVITY;
  az = raw[2] / 16384.0f * GRAVITY;
  gx = raw[4] / 131.0f * DEG_TO_RAD_F;
  gy = raw[5] / 131.0f * DEG_TO_RAD_F;
  gz = raw[6] / 131.0f * DEG_TO_RAD_F;
  return true;
}

void send_imu() {
  if (!pc_connected) return;

  float ax, ay, az, gx, gy, gz;
  if (!read_mpu6050(ax, ay, az, gx, gy, gz)) return;

  char message[128];
  int n = snprintf(message, sizeof(message), "I %.4f %.4f %.4f %.5f %.5f %.5f\n",
                   ax, ay, az, gx, gy, gz);
  if (n <= 0 || n >= (int)sizeof(message)) return;

  udp.beginPacket(pc_ip, pc_port);
  udp.write((const uint8_t *)message, n);
  udp.endPacket();
}


// ==================== 电机控制 ====================

void set_motor(Motor side, int pwm) {
  pwm = constrain(pwm, -PWM_MAX, PWM_MAX);

  int en, in1, in2;
  if (side == LEFT)  { en = 0; in1 = IN1; in2 = IN2; }
  else               { en = 1; in1 = IN3; in2 = IN4; }

  if (pwm > 0) {
    digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  } else if (pwm < 0) {
    digitalWrite(in1, LOW);  digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);  digitalWrite(in2, LOW);  // 刹车
  }
  ledcWrite(en, abs(pwm));
}

void stop() {
  set_motor(LEFT, 0);
  set_motor(RIGHT, 0);
}
