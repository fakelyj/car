#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP32QRCodeReader.h>
#include <MPU6050_tockn.h>

// ==========================================
// 1. 硬件引脚与对象定义 (已更新最新标定数据)
// ==========================================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define I2C_SDA 41
#define I2C_SCL 42

#define LED_PIN 48

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
MPU6050 mpu6050(Wire);
ESP32QRCodeReader reader(CAMERA_MODEL_AI_THINKER);

const int sensors[] = { 20, 19, 2, 1, 14 };  // 最左 -> 最右
// 既然插反了，我们就直接在源头把名字互换，后面的逻辑就清爽了
const int motorL = 47;
const int motorR = 45;
const int revPinL = 40;
const int revPinR = 39;
const int channelL = 0;
const int channelR = 1;
const int ledPin = 21;
const int freq = 5000;    // 20kHz 频率，消除电机低频啸叫
const int resolution1 = 8;  // 8位分辨率，速度控制范围 0~255

const int minVals[] = { 277, 270, 170, 274, 271 };       // 最新白底极小值
const int maxVals[] = { 2747, 2593, 1620, 3240, 3487 };  // 最新黑线极大值
float weights[] = { -2.0, -1.0, 0.0, 1.0, 2.0 };

// ==========================================
// 2. 核心控制参数
// ==========================================
int baseSpeed = 255;        // 基础前进速度
int climb_baseSpeed = 200;  // 爬坡补偿速度 (稍微给大点，防止溜车)
int turnfast = 145;         // 转弯外侧速度
int turnslow = -145;        // 转弯内侧速度
int delay_zero = 200;       // 通用停顿时间
int blacklin = 500;         // 灰度黑线阈值 (数据已映射至0-1000，500为完美中值)
int turnDelayTime = 280;    // 转弯前的“过线冲刺延时”(毫秒)。数值越大，冲得越深。
int delay_zero_h = 2000;
int c_5black_time = 2000;
int black_C = 400;
int last_speed = 60;
int h_speed = 100;

float Kp = 280.0, Kd = 220.0, lastError = 0;
float Kp_gyro = 12.0, Kd_gyro = 8.0;
float last_error_gyro = 0;
float turn_pwr_kp = 2.5;  //转向k值
int turn_pwr_speed = 100;
float target_gyro = 180;   // 目标航向角
float tar_yaw = 350;       // 环岛出环相对目标角
float filter_alpha = 0.2;  // 滤波系数：建议从 0.3 开始调
float filtered_yaw = 0.0;  // 存放过滤后的纯净航向角
int inc_speed_L = 45;
int inc_speed_R = 105;
int inc_delay = 1800;  // 进环岛盲跑时间
int outc_delay = 800;  // 出环岛盲跑时间


// ==========================================
// 3. 全局状态变量
// ==========================================
int qr = 1;               // 扫码结果 (决定后续路口与停车策略)
int count = -1;           // 核心状态机阶段计数器
int blackCount = 0;       // 当前压线传感器数量
bool isFinished = false;  // 起点扫码是否完成
bool isTurning = false;   // 是否处于转弯状态
unsigned long turnStartTime = 0;
unsigned long last_cross_time = 0;

int grid_lines_crossed = 0;  // 用于记录终点停车区的横线数
float display_yaw = 0;
float display_error = 0;

// ==========================================
// 函数声明
// ==========================================
void checkSerialCommands() {
  if (Serial.available() > 0) {
    // 读取串口字符串，直到遇到回车符
    String input = Serial.readStringUntil('\n');
    input.trim();  // 砍掉首尾多余的空格或换行符

    if (input.length() > 0) {
      // 1. 解析光电循迹 Kp (输入格式: kp=1.5)
      if (input.startsWith("kp=") || input.startsWith("KP=")) {
        Kp = input.substring(3).toFloat();
        Serial.print("✅ 光电 Kp 已实时更新为: ");
        Serial.println(Kp);
      }
      // 2. 解析光电循迹 Kd (输入格式: kd=0.5)
      else if (input.startsWith("kd=") || input.startsWith("KD=")) {
        Kd = input.substring(3).toFloat();
        Serial.print("✅ 光电 Kd 已实时更新为: ");
        Serial.println(Kd);
      }
      // 3. 解析陀螺仪 Kp (输入格式: kgp=2.5)
      else if (input.startsWith("kgp=")) {
        Kp_gyro = input.substring(4).toFloat();
        Serial.print("🚀 陀螺仪 Kp 已实时更新为: ");
        Serial.println(Kp_gyro);
      }
      // 4. 解析陀螺仪 Kd (输入格式: kgd=0.8)
      else if (input.startsWith("kgd=")) {
        Kd_gyro = input.substring(4).toFloat();
        Serial.print("🚀 陀螺仪 Kd 已实时更新为: ");
        Serial.println(Kd_gyro);
      } else {
        Serial.println("❌ 未知指令！请使用格式: kp=1.5, kd=0.5, kgp=2.0, kgd=0.8");
      }
    }
  }
}

// ==========================================
// 初始化 Setup
// ==========================================


// ====================================
// OLED 屏幕实时遥测函数
// ==========================================
void displa() {
  static unsigned long lastNormalDisplay = 0;
  if (millis() - lastNormalDisplay > 200) {  // 限制刷新率在 5Hz，防卡顿
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.print("ST:");
    display.print(count);
    display.print(" Q:");
    display.println(qr);


    display.setTextSize(1);
    display.setCursor(0, 25);
    display.print("Rel Yaw: ");
    display.print(display_yaw);
    display.println(" deg");

    display.setCursor(0, 38);
    display.print("Black_CN: ");
    display.print(blackCount);

    display.setCursor(0, 51);
    display.print("Error: ");
    display.print(display_error);

    display.display();
    lastNormalDisplay = millis();
  }
}

// ==========================================================
// 核心底盘驱动 (终极破案：交叉互换 + 极性补偿版)
// ==========================================================
void applySpeed(int left, int right) {
  left = constrain(left, -255, 255);
  right = constrain(right, -255, 255);

  // 1. 将大脑下达的 left 指令，发给物理右侧引脚(控制真实的左履带)
  if (left >= 0) {
    digitalWrite(revPinR, HIGH);  // 物理左履带：高电平向前
    ledcWrite(channelR, left);    // ⚠️ 向 R通道 写入 left 的值
  } else {
    digitalWrite(revPinR, LOW);   // 物理左履带：低电平向后
    ledcWrite(channelR, abs(left));
  }

  // 2. 将大脑下达的 right 指令，发给物理左侧引脚(控制真实的右履带)
  if (right >= 0) {
    digitalWrite(revPinL, LOW);   // 物理右履带：低电平向前
    ledcWrite(channelL, right);   // ⚠️ 向 L通道 写入 right 的值
  } else {
    digitalWrite(revPinL, HIGH);  // 物理右履带：高电平向后
    ledcWrite(channelL, abs(right));
  }
}
// ==========================================================
// 自定义可视化陀螺仪零偏校准 (官方库极速安全版)
// ==========================================================
void customGyroCalib() {
  Serial.println(">>> 开始高精度陀螺仪校准 <<<");
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("Gyro Calibrating...");
  display.display();

  float rx = 0, ry = 0, rz = 0;
  int samples = 100;

  // 1. 先将陀螺仪旧的误差清零，保证采样纯净
  mpu6050.setGyroOffsets(0.0, 0.0, 0.0);

  for (int i = 0; i < samples; i++) {
    // 2. 调用官方库极其稳定的更新函数 (耗时<1ms)
    mpu6050.update();

    // 3. 累加当前的原始角速度
    rx += mpu6050.getGyroX();
    ry += mpu6050.getGyroY();
    rz += mpu6050.getGyroZ();

    // 4. 每 100 次刷新一次屏幕 (避免频繁刷屏拖慢速度)
    if (i % 100 == 0) {
      Serial.print("Sampling: ");
      Serial.print(i);
      Serial.println(" / 100");

      display.fillRect(0, 20, 128, 44, SSD1306_BLACK);  // 局部清屏
      display.setCursor(0, 20);
      display.print("Sample: ");
      display.print(i);
      display.println(" / 100");

      // 进度条动画
      int barW = map(i, 0, samples, 0, 120);
      display.drawRect(4, 40, 120, 10, SSD1306_WHITE);
      display.fillRect(4, 40, barW, 10, SSD1306_WHITE);
      display.display();
    }

    // 5. 喂狗喘息时间
  }

  // 6. 计算平均误差，并重新注入给核心大脑
  mpu6050.setGyroOffsets(rx / samples, ry / samples, rz / samples);

  Serial.println(">>> 陀螺仪校准完毕！ <<<");
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(20, 20);
  display.print("DONE!");
  display.display();
}


// ==========================================================
// 陀螺仪闭环动态转弯系统 (防过冲 + 自动刹车)
// ==========================================================
void executeGyroTurn(float angle_offset) {
  mpu6050.update();
  float start_yaw = mpu6050.getAngleZ();
  float target_yaw = start_yaw + angle_offset;  // 计算出空间中的绝对目标角度
  unsigned long turn_timeout = millis();

  // 死死咬住循环，直到转到位
  while (true) {
    mpu6050.update();
    float current_yaw = mpu6050.getAngleZ();
    float error = target_yaw - current_yaw;  // 计算还差多少度

    // 1. 到位检测：误差小于 2 度，认为转弯精准到位，立刻跳出！
    if (abs(error) <= 2.0) break;

    // 2. P 控制器：根据剩余误差动态调整履带转速 (离目标越近，转得越慢)
    // 基础力量设为 100(防止摩擦力卡死)，外加误差比例，最高不超过 255
    int turn_pwr = constrain(turn_pwr_speed + abs(error) * Kp_gyro, 100, 255);

    // 3. 执行旋转动作
    if (error > 0) {
      // 误差为正，需要继续向左转
      applySpeed(turn_pwr, -turn_pwr);
    } else {
      // 误差为负，需要继续向右转
      applySpeed(-turn_pwr, turn_pwr);
    }

    // 4. 救命呼吸阀与超时防死机保护 (2.5秒强行跳出)
    if (millis() - turn_timeout > 2000) break;
  }

  // 5. 到位后的“电子点刹”，强制锁死履带，消除惯性滑动！
  applySpeed(0, 0);
  delay(100);
}
void setup() {
  pinMode(0, INPUT_PULLUP);
  pinMode(revPinL, OUTPUT);
  pinMode(revPinR, OUTPUT);
  pinMode(ledPin, OUTPUT);
  ledcSetup(channelL, freq, resolution1);
  ledcSetup(channelR, freq, resolution1);
  ledcAttachPin(motorL, channelL);
  ledcAttachPin(motorR, channelR);
  Wire.begin(I2C_SDA, I2C_SCL);
  Serial.begin(115200);
  Serial.setTimeout(10);

  mpu6050.begin();




  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED init failed");
    for (;;)
      ;
  }

  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(20, 20);
  display.print("READY!");
  display.display();

  for (int i = 0; i < 5; i++) pinMode(sensors[i], INPUT);
}
// ==========================================
// 主循环 Loop
// ==========================================
void loop() {

  // 2. 传感器基础采样与数据映射
  float sensorMapped[5];
  float sum = 0, weightedSum = 0;
  blackCount = 0;

  for (int i = 0; i < 5; i++) {
    int raw = analogRead(sensors[i]);
    sensorMapped[i] = constrain(map(raw, minVals[i], maxVals[i], 1000, 0), 0, 1000);
    if (sensorMapped[i] > blacklin) blackCount++;
    weightedSum += (float)sensorMapped[i] * weights[i];
    sum += sensorMapped[i];
  }

  int finalL = baseSpeed, finalR = baseSpeed;


  // // ==========================================================
  // // 🏔️ MPU6050 被动爬坡拦截器 (通杀台阶、减速带、斜坡)
  // // ==========================================================
  // mpu6050.update();
  // float current_pitch = mpu6050.getAngleX();  // 确认 X 轴是俯仰角(Pitch)

  // // 车头翘起大于 12 度触发 (可根据实车悬挂和平地倾角微调此阈值)
  // if (current_pitch > 12.0) {
  //   float climb_target_yaw = mpu6050.getAngleZ();
  //   while (true) {
  //     mpu6050.update();
  //     float realtime_pitch = mpu6050.getAngleX();
  //     float current_yaw = mpu6050.getAngleZ();

  //     // 坡道上强制锁定航向，防止跑偏掉下桥
  //     float error_gyro = climb_target_yaw - current_yaw;
  //     float diff_gyro = (Kp_gyro * error_gyro) + (Kd_gyro * (error_gyro - last_error_gyro));
  //     last_error_gyro = error_gyro;

  //     applySpeed(constrain(climb_baseSpeed + (int)diff_gyro, 0, 255),
  //                constrain(climb_baseSpeed - (int)diff_gyro, 0, 255));

  //     // OLED 提示爬坡状态
  //     if (millis() % 200 < 100) {
  //       display.clearDisplay();
  //       display.setTextSize(2);
  //       display.setCursor(10, 20);
  //       display.print("CLIMBING!");
  //       display.display();
  //     }

  //     // 退出条件：车身恢复平坦 (Pitch 小于 5 度)
  //     if (realtime_pitch < 5.0) {
  //       applySpeed(255, 255);
  //       delay(300);  // 给一脚大油门冲出坡顶边缘
  //       break;
  //     }
  //   }
  // }

  // ==========================================================
  // 【特种任务拦截区】(完成后强制 return 重新进入 loop)
  // ==========================================================

  // --- 阶段 6：寻线进环岛 ---
  // --- 阶段 5：穿越灰度干扰方框 (全盲直冲) ---
  if (count == 5) {
    mpu6050.update();
    float target_yaw_41 = mpu6050.getAngleZ();  // 锁定刚进方框时的黄金航向
    unsigned long st41_time = millis();
    int phase_41 = 0;

    while (count == 5) {

      // 陀螺仪强行锁死直线，绝对无视地面的噪点干扰
      applySpeed(h_speed+5, h_speed+5);

      if (phase_41 == 0) {
        // 阶段 0：闭眼盲冲期。强制盲冲 1000 毫秒（⚠️请根据方框实际长度微调这个时间）
        // 目的是确保车身完全进入干扰区，不被起点的横线再次误触发
        if (millis() - st41_time > c_5black_time) phase_41 = 1;
      } else if (phase_41 == 1) {
        // 阶段 1：睁眼寻找对岸的丁字路口
        int current_black = 0;
        for (int i = 0; i < 5; i++) {
          if (analogRead(sensors[i]) > blacklin) current_black++;
        }
        // 丁字路口特征：一根长横线，至少会有 4 个以上的传感器同时看到黑色
        if (current_black >= 4) {
          phase_41 = 2;
          st41_time = millis();
        }
      } else if (phase_41 == 2) {
        // 阶段 2：踩到丁字路口后，冲过横线防抖
        if (millis() - st41_time > 400) {
          count = 6;  // 🎉 成功穿越干扰区！把状态推进到 6
          break;
        }
      }

      // OLED 屏幕实时显示干扰区状态
    }
    return;  // 强制打断 loop
  }
  // 🌟 修正：将时间戳记录放在循环外，只记录进入状态6的瞬间时间
  unsigned long st6_time = millis();

  while (count == 6) {
    float loop_sum = 0, loop_wSum = 0;

    // 1. 读取传感器并映射
    for (int i = 0; i < 5; i++) {
      int raw = analogRead(sensors[i]);
      sensorMapped[i] = constrain(map(raw, minVals[i], maxVals[i], 1000, 0), 0, 1000);
      loop_sum += sensorMapped[i];
      loop_wSum += sensorMapped[i] * weights[i];
    }

    // 2. PID 计算
    float error = (loop_sum > black_C) ? (loop_wSum / loop_sum) : lastError;
    float correction = Kp * error + Kd * (error - lastError);
    lastError = error;

    // 3. 差速输出
    applySpeed(constrain(baseSpeed + (int)correction, 0, 255), constrain(baseSpeed - (int)correction, 0, 255));

    // 4. 屏蔽 800ms 防止上一个路口误触发（现在它能正常工作了！）
    if (millis() - st6_time > 800) {
      // 最左侧和中间同时压线，确认圆环切点
      // 最左侧和中间同时压线，确认圆环切点
      if (sensorMapped[0] > blacklin && sensorMapped[2] > blacklin) {
        mpu6050.update();
        float entry_yaw = mpu6050.getAngleZ();
        // 🌟 优化 1：非阻塞盲入圆环（保持陀螺仪神经活跃！）
        applySpeed(30, 255); // 假设这是左转入环的差速
        unsigned long blind_start = millis();
        while (millis() - blind_start < inc_delay) {
          mpu6050.update(); // 在盲入期间，死死咬住角度更新！
        }
        lastError = 0; 
        // 正式进入环内循迹
        while (true) {
          mpu6050.update();
          float relative_yaw = mpu6050.getAngleZ() - entry_yaw;
          // 🌟 优化 2：加上绝对值，无论左转环还是右转环都能完美识别！
          // 比如 tar_yaw 设为 250 (留一点提前量切出)
          if (abs(relative_yaw) >= abs(tar_yaw)) { 
            
            // 🌟 优化 3：非阻塞盲出圆环
            applySpeed(205, 205);
            blind_start = millis();
            while (millis() - blind_start < outc_delay) {
              mpu6050.update(); // 出环时也保持更新，为下一个路口留好底子
            }
            count = 7;  // 切入连续转弯路口网格区
            break;      // 打破 while(true)
          }
          
          // --- 环内常规 PID 循迹 ---
          float loop_sum = 0, loop_wSum = 0;
          for (int i = 0; i < 5; i++) {
            int raw = analogRead(sensors[i]);
            sensorMapped[i] = constrain(map(raw, minVals[i], maxVals[i], 1000, 0), 0, 1000);
            loop_sum += sensorMapped[i];
            loop_wSum += sensorMapped[i] * weights[i];
          }
          
          // 注意：你在环内的防丢线阈值设了 400，如果是故意的请保留，否则建议用统一定义的 black_C
          float error = (loop_sum > 400) ? (loop_wSum / loop_sum) : lastError;
          float correction = Kp * error + Kd * (error - lastError);
          lastError = error;
          
          applySpeed(constrain(baseSpeed + (int)correction, 0, 255), 
                     constrain(baseSpeed - (int)correction, 0, 255));
        }
        
        break;  // 成功逃脱状态 6 的大 while 循环
      }
    }
  }
  //   display_error = error;
  //   displa();
  // }


  // while (is_in_circle) {
  //   mpu6050.update();
  //   float relative_yaw = mpu6050.getAngleZ() - entry_yaw;
  //   display_yaw = relative_yaw;

  //   if (relative_yaw >= tar_yaw) {  // 满足出环角度
  //     applySpeed(160, 160);
  //     delay(outc_delay);  // 盲出圆环
  //     is_in_circle = false;
  //     count = 7;  // 切入连续转弯路口网格区
  //     break;
  //   }

  //  float loop_sum = 0, loop_wSum = 0;
  //  for (int i = 0; i < 5; i++) {
  //    int raw = analogRead(sensors[i]);
  //    sensorMapped[i] = constrain(map(raw, minVals[i], maxVals[i], 1000, 0), 0, 1000);
  //    loop_sum += sensorMapped[i];
  //    loop_wSum += sensorMapped[i] * weights[i];
  //  }
  //  float error = (loop_sum > black_C) ? (loop_wSum / loop_sum) : lastError;
  //  float correction = Kp * error + Kd * (error - lastError);
  //  lastError = error;

  // applySpeed(constrain(140 + (int)correction, 0, 255), constrain(140 - (int)correction, 0, 255));
  //  display_error = error;
  //  displa();
  // --- 阶段 72：连续转弯后的 1秒 PD 视觉巡线对齐 ---
  if (count == 72) {
      count = 8;  // 时间到，姿态对齐完成，切入陀螺仪导航的全盲冲刺阶段
  }

  // --- 阶段 8：陀螺仪全盲断线冲刺区 ---
  if (count == 8) {
    target_gyro = mpu6050.getAngleZ();
    unsigned long st8_time = millis();
    int gyro_phase = 0;

    while (count == 8) {
       mpu6050.update();
       float error_gyro = target_gyro - mpu6050.getAngleZ();
       float diff_gyro = (Kp_gyro * error_gyro) + (Kd_gyro * (error_gyro - last_error_gyro));
       last_error_gyro = error_gyro;

       applySpeed(constrain(climb_baseSpeed + (int)diff_gyro, 0, 255), constrain(climb_baseSpeed - (int)diff_gyro, 0, 255));

      if (gyro_phase == 0) {
        // 消隐盲跑期
        if (millis() - st8_time > 1500) gyro_phase = 1;
      } else if (gyro_phase == 1) {
        // 捕获对岸黑线
        int current_black = 0;
        for (int i = 0; i < 5; i++) {
          if (analogRead(sensors[i]) > blacklin) current_black++;
        }
        if (current_black <= 3) {
          gyro_phase = 2;
          st8_time = millis();
        }
      } else if (gyro_phase == 2) {
        // 捕获后短暂延时，确保车尾过线
        if (millis() - st8_time > 200) {
          count = 99;
          break;
        }
      }
      displa();
    }
    return;
  }

  // --- 阶段 99：终点网格智能入库 ---
  if (count == 99) {
    // 🌟 1. 必须在这里保留常规的 PID 循迹，让它稳稳地开！
    float error = (sum > black_C) ? (weightedSum / sum) : lastError;
    float correction = Kp * error + Kd * (error - lastError);
    lastError = error;

    // 基础速度降到 120，像个老司机一样慢慢找车位
    applySpeed(constrain(120 + (int)correction, 0, 255), 
               constrain(120 - (int)correction, 0, 255));

    // 🌟 2. 横线检测与“冷却时间”消抖
    // 只有距离上一次压线超过了 500ms（冷却完毕），才允许再次计数
    if (millis() - last_cross_time > 500) {
      
      if (blackCount >= 3) { // 确认踩到横线
        grid_lines_crossed++;
        last_cross_time = millis(); // 刷新冷却时间，接下来 500ms 内不会重复计数
        
        // 🌟 3. 到达目标，拉手刹熄火！
        if (grid_lines_crossed == 5) { // 这里的 5 最好换成你二维码解出来的变量
          applySpeed(0, 0); // 立刻断电
          
          display.clearDisplay();
          display.setTextSize(2);
          display.setCursor(10, 20);
          display.print("MISSION");
          display.setCursor(10, 40);
          display.print("DONE!");
          display.display();
          
          // 彻底锁死，神仙来了也别想让轮子动一下
          while (true) {
            delay(1000); 
          }
        }
      }
    }
  }

  // ==========================================================
  // 【常规非阻塞状态机】 (路口转弯 & PD常规巡线)
  // ==========================================================

  // ... 后面正常的起步扫码和 PD 寻线代码保持不变 ...
  // 【状态：起点安全扫码】
  // 扫码与实体按键双重发车机制
  if (count == -1) {
    while (!isFinished) {
      applySpeed(0, 0);
      reader.setup();
      reader.beginOnCore(1);
      sensor_t *s = esp_camera_sensor_get();
      s->set_hmirror(s, 1);
      struct QRCodeData qrCodeData;
      delay(1000);
      customGyroCalib();  // 陀螺仪校准时自动喂狗

      while (true) {
        // --- 方式 A：摄像头扫码启动 ---
        digitalWrite(ledPin, HIGH);
        if (reader.receiveQrCode(&qrCodeData, 100)) {
          if (qrCodeData.valid) {
            // 扫码成功且数据有效
            qr = atoi((const char *)qrCodeData.payload);
            Serial.print("QR Valid! Target QR: ");
            Serial.println(qr);
            isFinished = true;
            count = 0;
            digitalWrite(ledPin, LOW);
            break;
          } else {
            // 🚨 加入的无效扫码报错逻辑
            Serial.print("Invalid QR Payload: ");
            Serial.println((const char *)qrCodeData.payload);
          }
        }

        // --- 方式 B：实体 BOOT 按键强制启动 ---
        if (digitalRead(0) == LOW) {

          qr = 11;  // 强制赋予测试或比赛预案数值
          Serial.println("BOOT Button Pressed! Forced Start: qr=11");
          isFinished = true;
          count = 0;
          digitalWrite(ledPin, LOW);
          break;
        }
        checkSerialCommands();
      }
    }
    displa();
  }
  // ==========================================
  // 十字路口检测与闭环转向
  // ==========================================
  if (blackCount >= 3) {
    // 1. 过线冲刺延时（把车身送到路口正中心）
    if (count != 4) {
      applySpeed(baseSpeed, baseSpeed);
      delay(turnDelayTime);  // 之前加的那个参数，比如 150
      applySpeed(0, 0);
      delay(100);  // 停稳车身，准备起旋
    }

    // 2. 根据状态机翻译出“绝对转弯角度” (左+90，右-90)
    float target_angle = 0;
    if (count == 7) { target_angle = -90; }  // 7状态：左转
    else if (count == 71) {
      target_angle = 90;
    }  // 71状态：右转
    else {
      // 常规路口判断
      if (count == 0 || count == 3) {
        if (qr == 11 || qr == 12 || qr == 13) target_angle = 90;  // 左
        else target_angle = -90;                                  // 右
      } else if (count == 1 || count == 2) {
        if (qr == 11 || qr == 12 || qr == 13) target_angle = -90;  // 右
        else target_angle = 90;                                    // 左
      }
    }

    // 3. 执行空间制导闭环转向！(代码会在这里死死卡住，直到转出极其完美的角度)
    if (count != 4) {
      executeGyroTurn(target_angle);
    }

    // 4. 转向完成，无缝切换状态机
    if (count == 7) {
      count = 71;
    } else if (count == 71) {
      count = 72;
      turnStartTime = millis();
    }  // 给后面的视觉拉直保护用
    else {
      count++;
      // 5. 盲跑逃离十字路口，防止重复触发黑线
      delay(200);



      blackCount = 0;
      lastError = 0;  // 清除历史误差，重新开始笔直循迹
    }
    // 【状态：老本行 PD 巡线】
  } else {
    float error = 0;

    // 🛡️ 虚线抗干扰保护机制
    if (sum > black_C) {
      error = weightedSum / sum;  // 正常压到黑线，计算加权误差
    } else {
      error = lastError;  // 所有传感器都没看到线 (踩到虚线全白处)，沿用上一次的姿态冲刺过去
    }

    display_error = error;
    float correction = Kp * error + Kd * (error - lastError);
    lastError = error;

    finalL = constrain(baseSpeed + (int)correction, 0, 255);
    finalR = constrain(baseSpeed - (int)correction, 0, 255);
  }


  // 最终向电机输出动力
  //smartLED(finalL, finalR, isTurning);
  applySpeed(finalL, finalR);
}



