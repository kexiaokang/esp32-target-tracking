/*
 * WebTrack —— 亮度目标跟踪 + 网页实时画面调参
 *
 * 为什么要有这一版：
 *   ESP32-CAM 的排针是朝下插进 MB 母座的，插上之后 IO13/IO14 就被吃掉了，
 *   没法一边插着电脑一边接舵机。所以正确做法是：
 *     1) 插 MB 烧录这个程序
 *     2) 拔下来装到结构上，IO13/IO14 接舵机，外部 5V 供电
 *     3) 手机/电脑连板子发的热点，浏览器打开网页看画面、调参数
 *   全程不需要串口。
 *
 * 热点：ESP32CAM-TRACK   密码：12345678   地址：http://192.168.4.1
 *
 * 识别原理（只看亮度，不看颜色）：
 *   GC2145 内部是 YUV，转 RGB565 时色度有延迟，高对比边缘会出彩色镶边，
 *   按颜色判红会得到一大堆假目标。亮度是最干净的通道，所以：
 *     缩成 80x60 → Otsu 自动定亮暗分界 → 取最大连通域 → 算质心
 *   拿一个和背景反差大的东西晃就行，不用管它是什么颜色。
 *
 * 接线：IO13 = 上下 TILT，IO14 = 左右 PAN；舵机外部 5V 且与板子共地；SD 卡不插
 * 核心：esp32:esp32 2.0.17   开发板：AI Thinker ESP32-CAM
 */

#include <esp_camera.h>
#include <driver/ledc.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <esp_system.h>
#include "img_converters.h"

// ==================== 摄像头引脚 ====================
#define PWDN_GPIO_NUM   32
#define RESET_GPIO_NUM  -1
#define XCLK_GPIO_NUM   0
#define SIOD_GPIO_NUM   26
#define SIOC_GPIO_NUM   27
#define Y9_GPIO_NUM     35
#define Y8_GPIO_NUM     34
#define Y7_GPIO_NUM     39
#define Y6_GPIO_NUM     36
#define Y5_GPIO_NUM     21
#define Y4_GPIO_NUM     19
#define Y3_GPIO_NUM     18
#define Y2_GPIO_NUM      5
#define VSYNC_GPIO_NUM  25
#define HREF_GPIO_NUM   23
#define PCLK_GPIO_NUM   22

// ==================== 舵机 ====================
#define TILT_PIN       13
#define PAN_PIN        14
#define TILT_CHANNEL   LEDC_CHANNEL_2
#define PAN_CHANNEL    LEDC_CHANNEL_3
#define SERVO_TIMER    LEDC_TIMER_1
#define SERVO_DUTY_MAX 16383

// 板载补光灯：GPIO4。要用 PWM 调，否则一开就是死白
#define LED_PIN        4
#define LED_CHANNEL    LEDC_CHANNEL_4
#define LED_TIMER      LEDC_TIMER_2
#define LED_DUTY_MAX   1023

// ==================== 热点 ====================
const char *AP_SSID = "ESP32CAM-TRACK";
const char *AP_PASS = "12345678";

// ==================== 参数（网页上实时改） ====================
#define DS  4
#define DW  80
#define DH  60

float panGain   = 220.0f;   // 误差满格时每秒转多少度（帧率提上来后可以更激进）
float tiltGain  = 170.0f;
float deadzone  = 0.02f;    // 死区 2%：再大就会出现"物体偏了一点舵机不动"的迟钝感
float slewDps   = 300.0f;
// 机械限位还没实测过，先收紧到 ±30°（90±30），避免第一次跟踪就顶死舵机。
// 试出真实范围后在网页上改。
float panMin = 60.0f,  panMax = 120.0f;
float tiltMin = 60.0f, tiltMax = 120.0f;
int   panSign  = 1;
int   tiltSign = 1;    // 上下方向：原来 -1 是反的（点"上"画面往下），改成 +1
int   minAreaPct = 1;
int   maxAreaPct = 55;

// 识别模式：0 = 亮度（Otsu 自动分亮暗），1 = 自动找最红，2 = 标定色度
int   detMode  = 1;
int   refU = 110, refV = 160;   // 标定出来的目标色度（红：U 低 V 高）
int   colorTol = 40;            // 色度距离容差
int   ledBright  = 0;      // 补光灯亮度 0~100，默认关
bool  sweepWhenLost = true;
// 上电默认【不给舵机输出 PWM】。原因：机械中点未必是 90°，上电直接写 90 会顶到
// 机械死点，舵机就一直堵转发烫、嗡嗡响。必须先让用户手动找到安全范围再使能。
bool  servoEnabled = false;
bool  tracking  = false;   // 默认不跟踪，避免一上电就乱转

// ==================== 状态 ====================
// 画面叠加用的颜色（YUV）。必须定义在文件靠前的位置：Arduino 会自动把函数原型
// 插到文件顶部，如果这个结构体在后面，原型里出现 YuvColor 就会编译报错。
struct YuvColor { uint8_t y, u, v; };
static const YuvColor COL_BOX   = {235,  60,  60};   // 绿（U、V 都低）
static const YuvColor COL_CENT  = {235, 190,  60};   // 青（U 高 V 低）
static const YuvColor COL_CROSS = {235,  60, 190};   // 黄（U 低 V 高）

struct Target {
  bool found;
  int cx, cy;              // 质心（原图坐标）
  int area;                // 连通域面积（降采样格子数）
  int x0, y0, x1, y1;      // 包围盒（原图坐标）
  int thr;                 // Otsu 阈值
  bool dark;               // true=追暗目标
};
static Target tgt;

// panPos 现在只由 servoTask 写（单一写者），loop 只读；panWant 反之。
// 4 字节对齐的 float 在 ESP32 上单次读写是原子的，不需要额外加锁。
static float panPos = 90.0f, tiltPos = 90.0f;
static float panWant = 90.0f, tiltWant = 90.0f;
// 跟踪时的【角速度】(度/秒)。摄像头 7fps 只能 143ms 给一次误差，
// 如果直接改 panWant（位置式），舵机追到目标后就停住等下一帧 —— 还是"一顿一顿"。
// 改成给速度，由 50Hz 的 servoTask 每 20ms 积分一次，舵机就是连续滑动的。
static float panVel = 0.0f, tiltVel = 0.0f;
static float filEx = 0.0f, filEy = 0.0f;
static uint32_t lastSeen = 0, lastMs = 0, lastPrint = 0, fpsT0 = 0, lastViewer = 0, lastJpg = 0;
static int   frameCount = 0, sweepDir = 1;
static int   curW = 320, curH = 240;
static bool  locked = false;
static bool  camOk = false;          // 摄像头是否就绪，没就绪时网页照常能打开并显示原因
// 上一帧目标位置，用来让检测"粘住"同一个物体而不是每帧换一个
static int      prevCx = -1, prevCy = -1;
static uint32_t prevMs = 0;
static bool     calRequest = false;
// 调试用：当前画面中心的颜色、匹配到的格子数、全图最红处的红度
static int      ctrU = 128, ctrV = 128;
static int      lastFg = 0, lastMaxRed = 0, lastMedRed = 0;
// 流畅模式的帧率：loop 里不抓帧，帧是在 /jpg 接口里抓的，所以要在那里单独计数。
static float    fastFps = 0;
static uint32_t fastCnt = 0, fastT0 = 0;

static uint8_t  lum[DW * DH];
static uint8_t  dU[DW * DH], dV[DW * DH];   // 色度：红色 = V 高、U 低
static int16_t  redScore[DW * DH];
static bool     mask[DW * DH];
static int16_t  labelBuf[DW * DH];
static int16_t  stackBuf[DW * DH];

static uint8_t *sjpg = NULL;
static size_t   sjpgLen = 0;
static WebServer server(80);

// 重启诊断：RTC 内存掉电不丢，能数出板子重启了几次。
// 如果这个数字在网页上一直往上涨，说明板子正在反复复位（多半是舵机把 5V 拉塌了）。
RTC_DATA_ATTR int bootCount = 0;
static char resetReason[24] = "?";

static void readResetReason() {
  bootCount++;
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON:  strcpy(resetReason, "上电");      break;
    case ESP_RST_SW:       strcpy(resetReason, "软件重启");   break;
    case ESP_RST_PANIC:    strcpy(resetReason, "程序崩溃");   break;
    case ESP_RST_INT_WDT:  strcpy(resetReason, "中断看门狗"); break;
    case ESP_RST_TASK_WDT: strcpy(resetReason, "任务看门狗"); break;
    case ESP_RST_WDT:      strcpy(resetReason, "看门狗");     break;
    case ESP_RST_BROWNOUT: strcpy(resetReason, "欠压复位");   break;
    case ESP_RST_DEEPSLEEP:strcpy(resetReason, "深睡唤醒");   break;
    case ESP_RST_SDIO:     strcpy(resetReason, "SDIO");      break;
    default:               strcpy(resetReason, "未知");      break;
  }
}

static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

// ==================== 舵机 ====================
// 不能用 Arduino 的 ledcWrite()：它按 HIGH_SPEED_MODE 写，而定时器在 LOW_SPEED_MODE，
// 会报 "LEDC is not initialized" 且舵机完全不动。
static void servoWrite(ledc_channel_t ch, float deg) {
  deg = clampf(deg, 0.0f, 180.0f);
  uint32_t us = 500u + (uint32_t)(deg * 2000.0f / 180.0f);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, ch, us * SERVO_DUTY_MAX / 20000u);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, ch);
}

// 只在"使能舵机"时才把 IO13/IO14 接到 PWM 上
// 两个舵机接反是极常见的情况（尤其按接线表接的时候），加个软件对调开关：
// 不用重新插线，网页上点一下就换过来。
static bool swapServo = false;

static void servoAttach() {
  ledc_channel_config_t c = {};
  c.speed_mode = LEDC_LOW_SPEED_MODE;
  c.timer_sel  = SERVO_TIMER;
  c.intr_type  = LEDC_INTR_DISABLE;
  int tiltGpio = swapServo ? PAN_PIN  : TILT_PIN;
  int panGpio  = swapServo ? TILT_PIN : PAN_PIN;
  c.channel    = TILT_CHANNEL; c.gpio_num = tiltGpio; ledc_channel_config(&c);
  c.channel    = PAN_CHANNEL;  c.gpio_num = panGpio;  ledc_channel_config(&c);
}

static void servoInit() {
  ledc_timer_config_t t = {};
  t.speed_mode      = LEDC_LOW_SPEED_MODE;
  t.duty_resolution = LEDC_TIMER_14_BIT;
  t.timer_num       = SERVO_TIMER;
  t.freq_hz         = 50;
  t.clk_cfg         = LEDC_AUTO_CLK;
  ledc_timer_config(&t);

  // 关键：这里【不配置通道】，IO13/IO14 保持普通输入（高阻）。
  // 之前配置了通道但不写占空比，引脚会输出恒定低电平 —— 舵机收到一条"没有脉冲"的
  // 信号线，有的舵机会把它当成极值一路顶到机械限位，然后堵转发烫。
  // 高阻 = 真正的"没有信号"，舵机才会老实松开。
  pinMode(TILT_PIN, INPUT);
  pinMode(PAN_PIN, INPUT);
}

// 松开舵机：把引脚从 PWM 矩阵上摘下来，恢复高阻
static void servoRelease() {
  ledcDetachPin(TILT_PIN);
  ledcDetachPin(PAN_PIN);
  pinMode(TILT_PIN, INPUT);
  pinMode(PAN_PIN, INPUT);
}

// ==================== 舵机平滑任务（50Hz） ====================
// 这是"舵机一顿一顿"的正解。
// 原来 panPos 只在 loop 里更新，而 loop 每轮要抓帧+识别，摄像头 7fps 就是每秒 7 步：
// 舵机走一步、停 143ms、再走一步 —— 看起来就是"卡"，听着就是齿轮一顿一顿地响。
// 现在拆成独立任务，每 20ms 走一小步去追 panWant，跟摄像头帧率彻底解耦：
// 不管画面多少帧，舵机都是连续平滑地过去。
static void servoTask(void *arg) {
  (void)arg;
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    if (servoEnabled) {
      // 第一层：把"角速度"积分成目标位置。摄像头 7fps 给的速度值在 143ms 里不变，
      // 但这里每 20ms 就积一次，所以目标是连续往前走的，不是跳变。
      panWant  += panVel  * 0.02f;
      tiltWant += tiltVel * 0.02f;
      if (panWant  < panMin)  { panWant  = panMin;  panVel  = 0; }
      if (panWant  > panMax)  { panWant  = panMax;  panVel  = 0; }
      if (tiltWant < tiltMin) { tiltWant = tiltMin; tiltVel = 0; }
      if (tiltWant > tiltMax) { tiltWant = tiltMax; tiltVel = 0; }

      // 第二层：实际位置限速追目标。手动按钮直接改 panWant 时靠这层平滑过去，
      // 跟踪时目标已经是连续的了，这层基本不生效，等于直通。
      float step = slewDps * 0.02f;
      float dp = panWant  - panPos;
      float dt2 = tiltWant - tiltPos;
      if (dp >  step) dp =  step;
      if (dp < -step) dp = -step;
      if (dt2 >  step) dt2 =  step;
      if (dt2 < -step) dt2 = -step;
      panPos  += dp;
      tiltPos += dt2;
      servoWrite(PAN_CHANNEL,  panPos);
      servoWrite(TILT_CHANNEL, tiltPos);
    }
    vTaskDelayUntil(&last, pdMS_TO_TICKS(20));
  }
}

static void ledInit() {
  ledc_timer_config_t t = {};
  t.speed_mode      = LEDC_LOW_SPEED_MODE;
  t.duty_resolution = LEDC_TIMER_10_BIT;
  t.timer_num       = LED_TIMER;
  t.freq_hz         = 5000;
  t.clk_cfg         = LEDC_AUTO_CLK;
  ledc_timer_config(&t);

  ledc_channel_config_t c = {};
  c.speed_mode = LEDC_LOW_SPEED_MODE;
  c.timer_sel  = LED_TIMER;
  c.intr_type  = LEDC_INTR_DISABLE;
  c.channel    = LED_CHANNEL;
  c.gpio_num   = LED_PIN;
  ledc_channel_config(&c);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LED_CHANNEL, 0);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LED_CHANNEL);
}

static void ledApply() {
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LED_CHANNEL, (uint32_t)ledBright * LED_DUTY_MAX / 100);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LED_CHANNEL);
}

// ==================== 摄像头 ====================
// 现在两种传感器都支持，换模块不用改代码：
//   GC2145 —— 没有 JPEG 编码器，所有画质调节都是空函数，只能 RGB565
//   OV3660 —— 有 JPEG，亮度/对比度/饱和度/白平衡/曝光/增益全都能调
static const char *sensorName(int pid) {
  switch (pid) {
    case 0x26:   return "OV2640";
    case 0x3660: return "OV3660";
    case 0x5640: return "OV5640";
    case 0x2145: return "GC2145";
    case 0x9b:   return "OV3660";
    default:     return "未知";
  }
}

static int camPid = 0;
static const char *camName = "?";

// 两种工作模式：
//   fastMode=false  跟踪模式：YUV422 原始像素 -> 能算色度做颜色识别，但网页预览要靠
//                   软件 JPEG 编码（fmt2jpg 一次 100ms+），所以预览只有 7~10fps。
//   fastMode=true   流畅模式：JPEG 硬件直出 -> 摄像头自己就把画面压好了，取出来直接发，
//                   CPU 零成本，帧率能到 20fps 以上。代价是做不了颜色识别，只能手动控制舵机。
// 这两种格式没法共存：sensor 一次只能出一种格式，切换要 deinit + 重新 init。
static bool fastMode = false;

static bool cameraTry(int xclkMhz, bool jpeg) {
  camera_config_t cfg = {};
  cfg.ledc_channel = LEDC_CHANNEL_0;
  cfg.ledc_timer   = LEDC_TIMER_0;
  cfg.pin_d0 = Y2_GPIO_NUM;  cfg.pin_d1 = Y3_GPIO_NUM;
  cfg.pin_d2 = Y4_GPIO_NUM;  cfg.pin_d3 = Y5_GPIO_NUM;
  cfg.pin_d4 = Y6_GPIO_NUM;  cfg.pin_d5 = Y7_GPIO_NUM;
  cfg.pin_d6 = Y8_GPIO_NUM;  cfg.pin_d7 = Y9_GPIO_NUM;
  cfg.pin_xclk = XCLK_GPIO_NUM;   cfg.pin_pclk = PCLK_GPIO_NUM;
  cfg.pin_vsync = VSYNC_GPIO_NUM; cfg.pin_href = HREF_GPIO_NUM;
  cfg.pin_sccb_sda = SIOD_GPIO_NUM; cfg.pin_sccb_scl = SIOC_GPIO_NUM;
  cfg.pin_pwdn = PWDN_GPIO_NUM;   cfg.pin_reset = RESET_GPIO_NUM;
  cfg.xclk_freq_hz   = xclkMhz * 1000000;
  // 分辨率按模式分开，这是帧率的关键：
  //   JPEG 模式：QVGA 一帧才 4KB，I2S 随便搬，320x240 能跑 30fps。
  //   YUV 模式：一帧 320*240*2 = 153KB，I2S 实际吞吐约 2.25MB/s，
  //             实测抓一帧要 68ms（=15fps 天花板）。降到 QQVGA 后
  //             数据量只剩 38KB，抓帧降到 ~17ms，能跑 30fps 以上。
  //             代价是分辨率低一半，但对"跟踪一个明显的彩色物体"完全够用，
  //             而且网格还是 80x60，识别逻辑一点没变。
  cfg.frame_size     = jpeg ? FRAMESIZE_QVGA : FRAMESIZE_QQVGA;
  cfg.jpeg_quality   = jpeg ? 10 : 12;   // 流畅模式压狠一点：帧更小，WiFi 发得更快
  cfg.fb_location    = CAMERA_FB_IN_PSRAM;
  cfg.grab_mode      = CAMERA_GRAB_LATEST;
  if (jpeg) {
    // JPEG 直出：sensor 内部硬件压缩，CPU 完全不参与
    cfg.pixel_format = PIXFORMAT_JPEG;
    cfg.fb_count     = 3;      // 多留一个缓冲，DMA 抓新帧时旧的还在发
  } else {
    // YUV422：拿原始像素做颜色识别。OV3660 这版驱动 RGB565 实际没生效
    // （拿到的仍是 YUV，按 RGB 解就是一片彩虹），YUV 做颜色识别反而更直接。
    cfg.pixel_format = PIXFORMAT_YUV422;
    cfg.fb_count     = 2;
  }

  esp_err_t err = esp_camera_init(&cfg);
  if (err != ESP_OK) {
    Serial.printf("   XCLK %dMHz 初始化失败 0x%x\n", xclkMhz, err);
    return false;
  }
  sensor_t *s = esp_camera_sensor_get();
  camPid  = s->id.PID;
  camName = sensorName(camPid);
  Serial.printf("   传感器 %s（PID 0x%04x），XCLK %dMHz，%s\n",
                camName, camPid, xclkMhz, jpeg ? "JPEG 直出" : "YUV422");

  // OV3660 的曝光/白平衡是真的能调（GC2145 那些在驱动里全是空函数，调了没反应）
  if (camPid == 0x3660 || camPid == 0x9b) {
    s->set_whitebal(s, 1);        // 自动白平衡
    s->set_exposure_ctrl(s, 1);   // 自动曝光
    s->set_gain_ctrl(s, 1);       // 自动增益
    s->set_brightness(s, 0);      // -2..2
    s->set_contrast(s, 0);
    s->set_saturation(s, 0);      // -2..2，调大颜色更浓
    Serial.println("   OV3660：已开启自动白平衡/曝光/增益");
  }
  return true;
}

static bool initCamera(bool jpeg) {
  // 现在两种模式都是 24MHz 起步：JPEG 一帧 4KB，YUV(QQVGA) 一帧 38KB，
  // 数据量都在 I2S 能扛的范围内，之前 QVGA YUV 在 24MHz 下刷 VSYNC 溢出的问题
  // 随着分辨率降到 QQVGA 已经不存在了。
  if (cameraTry(24, jpeg)) return true;
  if (cameraTry(20, jpeg)) return true;
  if (cameraTry(10, jpeg)) return true;
  Serial.println("!! 摄像头两种时钟都起不来。查排线 / SD卡 / 供电");
  return false;
}

// 切换模式：摄像头必须先关掉再按新格式重开，不能原地改 pixel_format
static bool reinitCamera(bool jpeg) {
  if (sjpg) { free(sjpg); sjpg = NULL; sjpgLen = 0; }
  esp_camera_deinit();
  delay(60);
  bool ok = initCamera(jpeg);
  if (ok) {
    camera_fb_t *p = esp_camera_fb_get();
    if (p) { curW = p->width; curH = p->height; esp_camera_fb_return(p); }
  }
  return ok;
}

// ==================== 识别 ====================
// 像素格式 YUV422（YUYV）：每 2 个像素占 4 字节 = Y0 U Y1 V。
// 按小端 16 位字读：字0 = Y0 | (U<<8)，字1 = Y1 | (V<<8)。
// 所以低字节永远是 Y，高字节在 U/V 之间交替（偶数像素是 U，奇数像素是 V）。
static inline int yOf(uint16_t c) { return c & 0xFF; }

// 降采样步长 = 画面宽 / 网格宽。QVGA(320) -> 4，QQVGA(160) -> 2。
// 网格始终是 80x60，所以识别逻辑不用改，只是每个格子覆盖的实际面积变了。
static int gDs = DS;

static void buildFeatures(const uint16_t *px, int w, int h) {
  int ds = w / DW;
  if (ds < 1) ds = 1;
  gDs = ds;
  const int n = ds * ds;
  for (int by = 0; by < DH; by++) {
    for (int bx = 0; bx < DW; bx++) {
      int sy = 0, su = 0, sv = 0, nu = 0, nv = 0;
      const int x0 = bx * ds;                 // ds 是偶数，x0 一定是偶数
      for (int yy = 0; yy < ds; yy++) {
        const uint16_t *row = px + (size_t)(by * ds + yy) * w + x0;
        for (int xx = 0; xx < ds; xx++) {
          uint16_t c = row[xx];
          sy += yOf(c);
          int ch = c >> 8;
          if ((x0 + xx) & 1) { sv += ch; nv++; }
          else               { su += ch; nu++; }
        }
      }
      int i = by * DW + bx;
      lum[i] = sy / n;
      dU[i] = nu ? su / nu : 128;
      dV[i] = nv ? sv / nv : 128;
    }
  }
}

// Otsu：从直方图自动找亮暗分界，光照变了自动跟着变，不用调参
static int otsu(const uint32_t *hist, int total) {
  double sum = 0;
  for (int i = 0; i < 256; i++) sum += (double)i * hist[i];
  double sumB = 0;
  int wB = 0, best = 128;
  double maxVar = -1;
  for (int i = 0; i < 256; i++) {
    wB += hist[i];
    if (!wB) continue;
    int wF = total - wB;
    if (!wF) break;
    sumB += (double)i * hist[i];
    double mB = sumB / wB;
    double mF = (sum - sumB) / wF;
    double var = (double)wB * wF * (mB - mF) * (mB - mF);
    if (var > maxVar) { maxVar = var; best = i; }
  }
  return best;
}

// 一键标定：取画面正中一小块的平均色度（U,V）当目标色，顺便把容差算出来。
// 用色度而不是 RGB：亮度变了色度基本不变，光照影响小得多。
static void calibrateColor() {
  const int cw = 16, ch = 16;                  // 降采样坐标，16x16 = 原图 64x64
  int x0 = DW / 2 - cw / 2, y0 = DH / 2 - ch / 2;
  long su = 0, sv = 0;
  int n = 0;
  for (int y = y0; y < y0 + ch; y++) {
    for (int x = x0; x < x0 + cw; x++) {
      int i = y * DW + x;
      su += dU[i]; sv += dV[i];
      n++;
    }
  }
  if (!n) return;
  refU = su / n; refV = sv / n;

  long vu = 0, vv = 0;
  for (int y = y0; y < y0 + ch; y++) {
    for (int x = x0; x < x0 + cw; x++) {
      int i = y * DW + x;
      int a = dU[i] - refU, b = dV[i] - refV;
      vu += a * a; vv += b * b;
    }
  }
  float sigma = sqrtf((vu + vv) / (2.0f * n));
  colorTol = constrain((int)(sigma * 2.5f) + 15, 15, 100);

  prevCx = prevCy = -1;
  Serial.printf("标定完成：目标色度 U=%d V=%d  离散度 %.1f  容差自动设为 %d\n",
                refU, refV, sigma, colorTol);
}

static void findTarget() {
  const int N = DW * DH;
  tgt.found = false;

  int fg = 0;
  if (detMode == 1) {
    // ---- 自动找最红：不需要标定 ----
    // 红度 = V - U（色度差）。灰/白/黑 U≈V，红度接近 0；红色 V 高 U 低，能到 50 以上。
    // 关键：室内暖光会让整幅画面的红度整体抬高（实测白墙也有 15~25），
    // 所以不能用一个固定下限，否则暖光环境会整片被当成红色。
    // 做法：取【中位数】当背景基准，阈值 = 中位数 + 一定余量。
    int maxRed = 0;
    uint16_t histR[511] = {0};           // redScore 范围 -255..255，偏移 255 存
    for (int i = 0; i < N; i++) {
      int rd = (int)dV[i] - (int)dU[i];
      redScore[i] = rd;
      if (rd > maxRed) maxRed = rd;
      histR[rd + 255]++;
    }
    int med = 0, acc = 0;
    for (int k = 0; k < 511; k++) {
      acc += histR[k];
      if (acc * 2 >= N) { med = k - 255; break; }
    }
    int span = maxRed - med;
    int thr = med + (span > 0 ? (span * 55 / 100) : 0);
    if (thr < med + 14) thr = med + 14;   // 至少要比背景高 14，否则噪点也算
    tgt.thr = thr;
    tgt.dark = false;
    for (int i = 0; i < N; i++) {
      bool on = redScore[i] >= thr;
      mask[i] = on;
      labelBuf[i] = -1;
      if (on) fg++;
    }
    lastMaxRed = maxRed;
    lastMedRed = med;
    if (fg * 100 > N * maxAreaPct) return;   // 红满屏 = 背景
  } else if (detMode == 2) {
    // ---- 标定色度模式：和标定色比 U/V 平面上的距离 ----
    int tol2 = colorTol * colorTol;
    for (int i = 0; i < N; i++) {
      int du = (int)dU[i] - refU, dv = (int)dV[i] - refV;
      bool on = (du * du + dv * dv) <= tol2;
      mask[i] = on;
      labelBuf[i] = -1;
      if (on) fg++;
    }
    tgt.thr = colorTol;
    tgt.dark = false;
    if (fg * 100 > N * maxAreaPct) return;
  } else {
    // ---- 亮度模式：Otsu 自动分亮暗 ----
    uint32_t hist[256] = {0};
    for (int i = 0; i < N; i++) hist[lum[i]]++;
    int thr = otsu(hist, N);
    tgt.thr = thr;

    int nDark = 0;
    for (int i = 0; i <= thr; i++) nDark += hist[i];
    int nBright = N - nDark;

    bool darkMode = (nDark <= nBright);
    tgt.dark = darkMode;
    fg = darkMode ? nDark : nBright;
    if (fg * 100 < N * minAreaPct) return;
    if (fg * 100 > N * maxAreaPct) return;

    for (int i = 0; i < N; i++) {
      mask[i] = darkMode ? (lum[i] <= thr) : (lum[i] > thr);
      labelBuf[i] = -1;
    }
  }

  int bestArea = 0, bestSx = 0, bestSy = 0, bestLabel = -1;
  int bx0 = 0, by0 = 0, bx1 = 0, by1 = 0;
  int nextLabel = 0;
  long bestScore = -1;

  // 上一帧的目标位置。用来给"离它近的连通域"加分 —— 否则画面里只要有一块比目标
  // 更大的暗区（墙、椅子、阴影），就会一直锁在它上面，怎么晃都不动。
  bool hasPrev = (prevCx >= 0) && (millis() - prevMs < 1500);

  for (int i = 0; i < N; i++) {
    if (!mask[i] || labelBuf[i] >= 0) continue;
    int sp = 0;
    stackBuf[sp++] = (int16_t)i;
    labelBuf[i] = nextLabel;
    int area = 0, sx = 0, sy = 0;
    int x0 = DW, y0 = DH, x1 = -1, y1 = -1;
    while (sp > 0) {
      int p = stackBuf[--sp];
      int x = p % DW, y = p / DW;
      area++; sx += x; sy += y;
      if (x < x0) x0 = x;
      if (x > x1) x1 = x;
      if (y < y0) y0 = y;
      if (y > y1) y1 = y;
      if (x > 0      && mask[p - 1]  && labelBuf[p - 1]  < 0) { labelBuf[p - 1]  = nextLabel; stackBuf[sp++] = p - 1; }
      if (x < DW - 1 && mask[p + 1]  && labelBuf[p + 1]  < 0) { labelBuf[p + 1]  = nextLabel; stackBuf[sp++] = p + 1; }
      if (y > 0      && mask[p - DW] && labelBuf[p - DW] < 0) { labelBuf[p - DW] = nextLabel; stackBuf[sp++] = p - DW; }
      if (y < DH - 1 && mask[p + DW] && labelBuf[p + DW] < 0) { labelBuf[p + DW] = nextLabel; stackBuf[sp++] = p + DW; }
    }
    // 面积下限：用绝对格子数兜底（12 格 = 原图 48x48 像素左右），
    // 这样滑块拉到 0 也不会因为 1% 这种比例把远处的小目标直接判死。
    int minA = N * minAreaPct / 100;
    if (minA < 12) minA = 12;
    if (area >= minA) {
      long score = area;
      if (hasPrev) {
        int cx = sx / area, cy = sy / area;
        // prevCx/prevCy 存的是【原图像素坐标】，cx/cy 是【网格坐标】，
        // 必须除以当前实际的降采样步长 gDs 才能同量纲比较。
        // 之前写死 DS=4，QQVGA 下 gDs 是 2，除错了 —— "离上一帧近"的加分全乱，
        // 目标会在几个候选连通域之间乱跳，表现就是左右有时正常、有时抽风。
        int dx = cx - prevCx / gDs, dy = cy - prevCy / gDs;
        if (dx * dx + dy * dy <= 24 * 24) score *= 4;   // 靠近上一帧 → 强加分
      }
      if (score > bestScore) {
        bestScore = score;
        bestArea = area; bestSx = sx; bestSy = sy; bestLabel = nextLabel;
        bx0 = x0; by0 = y0; bx1 = x1; by1 = y1;
      }
    }
    nextLabel++;
  }

  lastFg = fg;
  if (bestLabel < 0) return;
  if (bestArea * 100 < N * minAreaPct) return;
  // 贴在四边的巨块基本是背景/墙面，不是目标
  if (bx0 <= 1 && by0 <= 1 && bx1 >= DW - 2 && by1 >= DH - 2) return;

  for (int i = 0; i < N; i++) if (labelBuf[i] != bestLabel) mask[i] = false;

  tgt.found = true;
  tgt.cx = (bestSx / bestArea) * gDs;
  tgt.cy = (bestSy / bestArea) * gDs;
  tgt.area = bestArea;
  tgt.x0 = bx0 * gDs; tgt.y0 = by0 * gDs;
  tgt.x1 = bx1 * gDs + gDs - 1; tgt.y1 = by1 * gDs + gDs - 1;
  prevCx = tgt.cx; prevCy = tgt.cy; prevMs = millis();
}

// ==================== 画面叠加 ====================
// 缓冲是 YUV422（YUYV）：一个字里低字节是 Y，高字节是 U 或 V（偶数像素 U、奇数像素 V）。
// 所以画线不能直接写 16 位 RGB565 值，要按像素写 Y 和对应的那半个色度。
// 注意：YuvColor 必须定义在文件最前面（见上方），否则 Arduino 自动生成的函数原型找不到它。

static inline void putYUV(uint16_t *row, int x, YuvColor c) {
  int ch = (x & 1) ? c.v : c.u;
  row[x] = (uint16_t)(c.y | ((uint16_t)ch << 8));
}
static void hline(uint16_t *p, int w, int h, int x0, int x1, int y, YuvColor c) {
  if (y < 0 || y >= h) return;
  if (x0 < 0) x0 = 0;
  if (x1 >= w) x1 = w - 1;
  uint16_t *row = p + (size_t)y * w;
  for (int x = x0; x <= x1; x++) putYUV(row, x, c);
}
static void vline(uint16_t *p, int w, int h, int x, int y0, int y1, YuvColor c) {
  if (x < 0 || x >= w) return;
  if (y0 < 0) y0 = 0;
  if (y1 >= h) y1 = h - 1;
  for (int y = y0; y <= y1; y++) putYUV(p + (size_t)y * w, x, c);
}
static void drawBox(uint16_t *p, int w, int h, int x0, int y0, int x1, int y1, YuvColor c) {
  for (int t = 0; t < 2; t++) {
    hline(p, w, h, x0 - t, x1 + t, y0 - t, c);
    hline(p, w, h, x0 - t, x1 + t, y1 + t, c);
    vline(p, w, h, x0 - t, y0 - t, y1 + t, c);
    vline(p, w, h, x1 + t, y0 - t, y1 + t, c);
  }
}

// ==================== 预览降采样（只影响画面，不影响识别） ====================
// 识别的准确度靠 320x240 全分辨率，但网页预览没必要那么大：
// fmt2jpg 的耗时跟像素数成正比，320x240 要 100ms+，160x120 只要 25ms 左右。
// 像素少 4 倍换 4 倍编码速度，画面糊一点但帧率翻几倍 —— 识别精度一点没损失。
#define HALF_W 160
#define HALF_H 120
static uint8_t *yuvHalf = NULL;

// YUYV 2:1 水平 + 2:1 垂直降采样。
// 源一行是 [Y0 U][Y1 V][Y2 U][Y3 V]...，每 2 个字 = 1 对 = 2 个像素。
// 目标每 2 个字也是 1 对，所以水平每 2 对取第 1 对：d[0]=s[0], d[1]=s[1]。
// 这样 U/V 的配对关系完全保持，不会出现"只有 U 没有 V"的偏色。
static void downYuv2(const uint8_t *src, int sw, int sh, uint8_t *dst) {
  const int dw = sw / 2;
  for (int y = 0; y < sh / 2; y++) {
    const uint16_t *s = (const uint16_t *)src + (size_t)(y * 2) * sw;  // 只取偶数行
    uint16_t *d = (uint16_t *)dst + (size_t)y * dw;
    for (int x = 0; x < dw / 2; x++) {
      d[x * 2]     = s[x * 4];       // Y(2x)   + U
      d[x * 2 + 1] = s[x * 4 + 1];   // Y(2x+1) + V
    }
  }
}


// ==================== 网页 ====================
static const char INDEX[] PROGMEM = R"rawliteral(<!DOCTYPE html><html><head><meta charset=utf-8>
<meta name=viewport content="width=device-width,initial-scale=1"><title>ESP32-CAM 云台跟踪</title>
<style>
*{box-sizing:border-box}body{font-family:Inter,system-ui,"Microsoft YaHei",sans-serif;background:#0b1018;color:#e7edf6;margin:0;padding:32px;max-width:1440px;margin:auto}body:before{content:"◎  TRACK / VISION";display:block;font-size:22px;font-weight:750;letter-spacing:3px;padding-bottom:24px;border-bottom:1px solid #253044;margin-bottom:24px}body:after{content:"ESP32-CAM  /  VISION CONTROL SYSTEM";display:block;color:#52647c;font-size:10px;letter-spacing:3px;margin-top:32px}#big{font-size:15px;font-weight:600;margin:0 0 18px;padding:14px 18px;background:#121d2b;border:1px solid #26384c;border-radius:12px}.ok{color:#43dab2}.no{color:#f7ba69}img{width:100%;min-height:300px;aspect-ratio:4/3;object-fit:contain;border:1px solid #2c3d52;border-radius:16px;background:radial-gradient(ellipse at center,#192d3a,#080d14);display:block;margin:18px 0}h3{font-size:14px;color:#d9e6f5;margin:26px 0 12px;font-weight:650;border-top:1px solid #243044;padding-top:20px;letter-spacing:1px}.row{display:flex;align-items:center;gap:12px;margin:10px 0;font-size:13px;padding:8px 12px;background:#111b29;border-radius:10px}.row label{width:190px;color:#9aaec7}.row input[type=range]{flex:1;min-width:40px;accent-color:#3dd6b0;height:5px}.row b{width:46px;text-align:right;color:#48dfbc;font-family:monospace}button{cursor:pointer;background:#1a293c;color:#dae7f5;border:1px solid #30445e;border-radius:8px;padding:11px 16px;font-size:13px;margin-right:8px;transition:background .2s}button:hover{background:#28415c;border-color:#43dab2}button:active{transform:translateY(1px)}#tg{background:#127866;border-color:#24ad90}pre{font-size:12px;line-height:1.8;background:#0c1521;padding:18px;border:1px solid #223247;border-radius:12px;color:#90a6c0;overflow-x:auto;white-space:pre-wrap}#stat{color:#9aaec7}main{display:grid;grid-template-columns:minmax(0,1.35fr) minmax(340px,1fr);gap:28px}.controls{background:#0e1723;border:1px solid #233147;border-radius:16px;padding:0 20px 20px}.controls h3:first-child{border:0;margin-top:0}.monitor{position:sticky;top:24px;align-self:start}@media(max-width:900px){main{display:block}.monitor{position:static}.controls{margin-top:24px}}@media(max-width:600px){body{padding:16px}.row{flex-wrap:wrap}.row label{width:130px}button{padding:10px;font-size:12px}img{min-height:220px}}

/* Offline SVG icons; pseudo-elements survive live button text updates. */
button::before,h3::before{content:"";display:inline-block;width:17px;height:17px;margin-right:9px;vertical-align:-3px;background:currentColor;mask:var(--icon) center/contain no-repeat;-webkit-mask:var(--icon) center/contain no-repeat}h3::before{color:#43dab2;width:19px;height:19px}
[data-icon="camera"]{--icon:url("data:image/svg+xml,%3Csvg%20xmlns%3D%22http%3A%2F%2Fwww.w3.org%2F2000%2Fsvg%22%20viewBox%3D%220%200%2024%2024%22%20fill%3D%22none%22%20stroke%3D%22black%22%20stroke-width%3D%221.7%22%20stroke-linecap%3D%22round%22%20stroke-linejoin%3D%22round%22%3E%3Cpath%20d%3D%22M14%207l4-4h3v18h-3l-4-4M3%206h11v12H3z%22%2F%3E%3C%2Fsvg%3E")}
[data-icon="target"]{--icon:url("data:image/svg+xml,%3Csvg%20xmlns%3D%22http%3A%2F%2Fwww.w3.org%2F2000%2Fsvg%22%20viewBox%3D%220%200%2024%2024%22%20fill%3D%22none%22%20stroke%3D%22black%22%20stroke-width%3D%221.7%22%20stroke-linecap%3D%22round%22%20stroke-linejoin%3D%22round%22%3E%3Cpath%20d%3D%22M12%202v4m0%2012v4M2%2012h4m12%200h4%20M19%2012a7%207%200%201%201-14%200%207%207%200%200%201%2014%200%20M15%2012a3%203%200%201%201-6%200%203%203%200%200%201%206%200%22%2F%3E%3C%2Fsvg%3E")}
[data-icon="sliders"]{--icon:url("data:image/svg+xml,%3Csvg%20xmlns%3D%22http%3A%2F%2Fwww.w3.org%2F2000%2Fsvg%22%20viewBox%3D%220%200%2024%2024%22%20fill%3D%22none%22%20stroke%3D%22black%22%20stroke-width%3D%221.7%22%20stroke-linecap%3D%22round%22%20stroke-linejoin%3D%22round%22%3E%3Cpath%20d%3D%22M4%203v18M12%203v18M20%203v18M1%208h6M9%2016h6M17%208h6%22%2F%3E%3C%2Fsvg%3E")}
[data-icon="color"]{--icon:url("data:image/svg+xml,%3Csvg%20xmlns%3D%22http%3A%2F%2Fwww.w3.org%2F2000%2Fsvg%22%20viewBox%3D%220%200%2024%2024%22%20fill%3D%22none%22%20stroke%3D%22black%22%20stroke-width%3D%221.7%22%20stroke-linecap%3D%22round%22%20stroke-linejoin%3D%22round%22%3E%3Cpath%20d%3D%22M12%203C9%207%205%2011%205%2015a7%207%200%200%200%2014%200c0-4-4-8-7-12z%22%2F%3E%3C%2Fsvg%3E")}
[data-icon="area"]{--icon:url("data:image/svg+xml,%3Csvg%20xmlns%3D%22http%3A%2F%2Fwww.w3.org%2F2000%2Fsvg%22%20viewBox%3D%220%200%2024%2024%22%20fill%3D%22none%22%20stroke%3D%22black%22%20stroke-width%3D%221.7%22%20stroke-linecap%3D%22round%22%20stroke-linejoin%3D%22round%22%3E%3Cpath%20d%3D%22M8%203H3v5m13-5h5v5M3%2016v5h5m13-5v5h-5%20M8%208h8v8H8z%22%2F%3E%3C%2Fsvg%3E")}
[data-icon="power"]{--icon:url("data:image/svg+xml,%3Csvg%20xmlns%3D%22http%3A%2F%2Fwww.w3.org%2F2000%2Fsvg%22%20viewBox%3D%220%200%2024%2024%22%20fill%3D%22none%22%20stroke%3D%22black%22%20stroke-width%3D%221.7%22%20stroke-linecap%3D%22round%22%20stroke-linejoin%3D%22round%22%3E%3Cpath%20d%3D%22M12%202v10M6%205a9%209%200%201%200%2012%200%22%2F%3E%3C%2Fsvg%3E")}
[data-icon="move"]{--icon:url("data:image/svg+xml,%3Csvg%20xmlns%3D%22http%3A%2F%2Fwww.w3.org%2F2000%2Fsvg%22%20viewBox%3D%220%200%2024%2024%22%20fill%3D%22none%22%20stroke%3D%22black%22%20stroke-width%3D%221.7%22%20stroke-linecap%3D%22round%22%20stroke-linejoin%3D%22round%22%3E%3Cpath%20d%3D%22M12%202v20M2%2012h20M8%206l4-4%204%204M8%2018l4%204%204-4M6%208l-4%204%204%204m12-8%204%204-4%204%22%2F%3E%3C%2Fsvg%3E")}
[data-icon="limit"]{--icon:url("data:image/svg+xml,%3Csvg%20xmlns%3D%22http%3A%2F%2Fwww.w3.org%2F2000%2Fsvg%22%20viewBox%3D%220%200%2024%2024%22%20fill%3D%22none%22%20stroke%3D%22black%22%20stroke-width%3D%221.7%22%20stroke-linecap%3D%22round%22%20stroke-linejoin%3D%22round%22%3E%3Cpath%20d%3D%22M5%203v18M19%203v18M8%2012h8m-5-3-3%203%203%203m2-6%203%203-3%203%22%2F%3E%3C%2Fsvg%3E")}
[data-icon="swap"]{--icon:url("data:image/svg+xml,%3Csvg%20xmlns%3D%22http%3A%2F%2Fwww.w3.org%2F2000%2Fsvg%22%20viewBox%3D%220%200%2024%2024%22%20fill%3D%22none%22%20stroke%3D%22black%22%20stroke-width%3D%221.7%22%20stroke-linecap%3D%22round%22%20stroke-linejoin%3D%22round%22%3E%3Cpath%20d%3D%22M3%207h18l-4-4M21%2017H3l4%204%22%2F%3E%3C%2Fsvg%3E")}
[data-icon="home"]{--icon:url("data:image/svg+xml,%3Csvg%20xmlns%3D%22http%3A%2F%2Fwww.w3.org%2F2000%2Fsvg%22%20viewBox%3D%220%200%2024%2024%22%20fill%3D%22none%22%20stroke%3D%22black%22%20stroke-width%3D%221.7%22%20stroke-linecap%3D%22round%22%20stroke-linejoin%3D%22round%22%3E%3Cpath%20d%3D%22M3%2011l9-8%209%208M5%2010v11h14V10M10%2021v-7h4v7%22%2F%3E%3C%2Fsvg%3E")}
[data-icon="sun"]{--icon:url("data:image/svg+xml,%3Csvg%20xmlns%3D%22http%3A%2F%2Fwww.w3.org%2F2000%2Fsvg%22%20viewBox%3D%220%200%2024%2024%22%20fill%3D%22none%22%20stroke%3D%22black%22%20stroke-width%3D%221.7%22%20stroke-linecap%3D%22round%22%20stroke-linejoin%3D%22round%22%3E%3Cpath%20d%3D%22M12%202v2m0%2016v2M2%2012h2m16%200h2M5%205l2%202m10%2010%202%202M5%2019l2-2M17%207l2-2M16%2012a4%204%200%201%201-8%200%204%204%200%200%201%208%200%22%2F%3E%3C%2Fsvg%3E")}
</style></head><body>
<main><section class="monitor"><div id=big class=no>连接中…</div>
<div class="row" style="flex-wrap:wrap;gap:8px 0;margin-bottom:4px">
<button data-icon="camera" id=mf onclick="setMode(1)">流畅模式（JPEG 直出，20fps+）</button>
<button data-icon="target" id=mt onclick="setMode(0)">跟踪模式（YUV 识别，7~10fps）</button>
</div>
<img id=v alt="">
<div id=stat></div>
</section><section class="controls"><h3 data-icon="sliders">云台参数</h3>
<div class="row"><label>增益 (度/秒)</label><input type=range min=10 max=400 value=220 oninput="set('g='+this.value);this.nextElementSibling.textContent=this.value"><b>220</b></div>
<div class="row"><label>死区</label><input type=range min=0 max=20 value=4 oninput="set('d='+(this.value/100));this.nextElementSibling.textContent=(this.value/100).toFixed(2)"><b>0.04</b></div>
<div class="row"><label>限速 (度/秒)</label><input type=range min=30 max=400 value=300 oninput="set('n='+this.value);this.nextElementSibling.textContent=this.value"><b>300</b></div>
<h3 data-icon="target">识别方式</h3>
<div class="row" style="flex-wrap:wrap;gap:8px 0">
<button data-icon="color" onclick="set('md=1')">自动找最红（不用标定）</button>
<button data-icon="color" onclick="set('md=2')">标定色（精确）</button>
<button data-icon="sun" onclick="set('md=0')">亮度</button>
</div>
<div class="row" style="flex-wrap:wrap;gap:8px 0">
<button data-icon="color" onclick="set('cal=1')">标定颜色（物体对准黄十字再点）</button>
</div>
<div class="row"><label>颜色容差（标定色模式）</label><input type=range min=10 max=160 value=70 oninput="set('tol='+this.value);this.nextElementSibling.textContent=this.value"><b>70</b></div>
<h3 data-icon="area">面积门限</h3>
<div class="row"><label>最小面积 %</label><input type=range min=0 max=20 value=1 oninput="set('a='+this.value);this.nextElementSibling.textContent=this.value"><b>1</b></div>
<div class="row"><label>最大面积 %</label><input type=range min=10 max=90 value=55 oninput="set('x='+this.value);this.nextElementSibling.textContent=this.value"><b>55</b></div>
<h3 data-icon="power">舵机</h3>
<div class="row" style="flex-wrap:wrap;gap:8px 0">
<button data-icon="power" id=seb onclick="set('se='+(seOn?0:1))">使能舵机</button>
<button data-icon="target" id=tg onclick="set('e='+(tgOn?0:1))">开始跟踪</button>
<button data-icon="home" onclick="set('r=1')">回中</button>
</div>
<h3 data-icon="move">手动微调（每次 5°，会停掉跟踪）</h3>
<div class="row" style="flex-wrap:wrap;gap:8px 0">
<button data-icon="move" onclick="set('mv=3')">上 ▲</button>
<button data-icon="move" onclick="set('mv=4')">下 ▼</button>
<button data-icon="move" onclick="set('mv=1')">左 ◀</button>
<button data-icon="move" onclick="set('mv=2')">右 ▶</button>
</div>
<h3 data-icon="limit">机械限位（听到顶住的声音就往回收 5°）</h3>
<div class="row"><label>左右下限</label><input type=range min=0 max=180 value=60 oninput="set('lo='+this.value);this.nextElementSibling.textContent=this.value"><b>60</b></div>
<div class="row"><label>左右上限</label><input type=range min=0 max=180 value=120 oninput="set('hi='+this.value);this.nextElementSibling.textContent=this.value"><b>120</b></div>
<div class="row"><label>上下下限</label><input type=range min=0 max=180 value=60 oninput="set('tlo='+this.value);this.nextElementSibling.textContent=this.value"><b>60</b></div>
<div class="row"><label>上下上限</label><input type=range min=0 max=180 value=120 oninput="set('thi='+this.value);this.nextElementSibling.textContent=this.value"><b>120</b></div>
<h3 data-icon="swap">方向</h3>
<div class="row" style="flex-wrap:wrap;gap:8px 0">
<button data-icon="swap" onclick="set('p=1')">左右取反</button>
<button data-icon="swap" onclick="set('t=1')">上下取反</button>
<button data-icon="power" onclick="set('sw=1')">舵机接线对调</button>
</div>
<div class="row"><label>补光灯 0~100</label><input type=range min=0 max=100 value=0 oninput="set('l='+this.value);this.nextElementSibling.textContent=this.value"><b>0</b></div>
<pre>绿框=目标  青十字=质心  黄十字=画面中心

第一次用：上电舵机是松的（不响不烫），可先用手摆到大概中间；
点「使能舵机」后用 上下左右 一点点试，听到顶住的嗡嗡声就往回收 5°；
把试出来的范围填进「机械限位」；最后点「开始跟踪」。

舵机一直响不转 = 顶死了，马上点「松开舵机」。</pre>
</section></main><script>
var pend=null,last=0,tgOn=false,seOn=false;
// 切换工作模式。摄像头要 deinit 再按新格式重开，会阻塞半秒左右，
// 所以先把提示打出来，等请求回来再刷新页面（刷新后按钮高亮才准）。
function setMode(fm){
 var b=document.getElementById('big');
 b.textContent='切换中…（摄像头要重新初始化，约 0.5 秒）';b.className='no';
 fetch('/set?fm='+fm).then(r=>r.text()).then(function(){
   setTimeout(function(){location.reload();},250);
 }).catch(function(){setTimeout(function(){location.reload();},700);});
}
function set(q){var t=Date.now();
 if(t-last>150){last=t;fetch('/set?'+q);}
 else{clearTimeout(pend);pend=setTimeout(function(){last=Date.now();fetch('/set?'+q);},160);}}
// ---- 画面刷新：上一张显示完立刻请求下一张，中间不留任何延迟 ----
// 这是"最快轮询"。浏览器的 onload 本身就是天然节流阀：板子发多快就收多快。
// 以前是 setTimeout(tick,20)，而且每帧还要先等一次 /api —— 两次 HTTP 往返
// 直接把帧率砍掉一半，现在把状态查询彻底拆出去了。
var fBusy=false,fErr=0;
function frameTick(){
 if(fBusy)return;
 fBusy=true;
 var im=document.getElementById('v');
 im.onload=function(){fBusy=false;fErr=0;frameTick();};
 im.onerror=function(){fBusy=false;fErr++;setTimeout(frameTick,fErr>3?800:120);};
 im.src='/jpg?r='+Math.random();
}
// ---- 状态查询：每秒一次，跟画面完全独立 ----
function statusTick(){
 var ac=new AbortController();var to=setTimeout(function(){ac.abort()},3000);
 fetch('/api',{signal:ac.signal}).then(r=>r.json()).then(d=>{
  clearTimeout(to);
  var b=document.getElementById('big');
  var mn=d.md==1?'自动红':(d.md==2?'标定色度':'亮度');
  // 按钮文字一律跟板子的真实状态走，不在本地猜，避免"看着开了其实没开"
  seOn=!!d.se; tgOn=!!d.tr;
  document.getElementById('seb').textContent=seOn?'松开舵机':'使能舵机';
  document.getElementById('tg').textContent=tgOn?'停止跟踪':'开始跟踪';
  // 模式按钮高亮：当前在哪个模式就点亮哪个
  document.getElementById('mf').style.background=d.fm?'#1d4ed8':'#252a32';
  document.getElementById('mt').style.background=d.fm?'#252a32':'#1d4ed8';
  if(!d.camok){b.textContent='摄像头没就绪 —— 查排线（FPC 卡扣有没有压紧）和 SD 卡槽是否空';b.className='no';}
  else if(d.fm){b.textContent='流畅模式 · JPEG 硬件直出 · '+d.fps+' fps（不做颜色识别，用下面方向按钮手动控制舵机）';b.className='ok';}
  else if(!d.se){b.textContent='舵机未使能（现在是松的，不响不烫）—— 先点「使能舵机」';b.className='no';}
  else if(!d.tr){b.textContent='跟踪已关 —— 先把机械限位调好，再点「开始跟踪」';b.className='no';}
  else if(d.lock){b.textContent='锁定 · '+mn+' · 面积 '+d.area;b.className='ok';}
  else if(d.md==1){b.textContent='画面里没找到明显的红色（最红值 '+d.mred+'，要 ≥20）—— 把红物体靠近些、换个亮点的光';b.className='no';}
  else if(d.md==2){b.textContent='没找到这个颜色 —— 物体对准黄十字点「标定」，或把容差调大';b.className='no';}
  else {b.textContent='搜索中 —— 把反差大的东西放到镜头前';b.className='no';}
  document.getElementById('stat').innerHTML = d.fm
   ? '<pre>流畅模式：JPEG 硬件直出，不做颜色识别\n帧率 '+d.fps+' fps   PAN '+d.pan+'°  TILT '+d.tilt+'°'+
     '\n舵机 '+(d.se?'已使能':'已松开')+
     '\n运行 '+d.up+' 秒   启动第 '+d.bc+' 次   上次重启：'+d.rr+
     '\n摄像头 '+d.cam+'（PID 0x'+d.pid.toString(16)+'）</pre>'
   : '<pre>中心 ('+d.cx+','+d.cy+')  模式 '+mn+
     '  阈值 '+(d.md==2?d.tol:d.thr)+'  误差 '+d.ex+' / '+d.ey+
     '\n画面中心色度 U='+d.ccu+' V='+d.ccv+'（红：V 明显大于 U）'+
     '\n最红值 '+d.mred+'   匹配格子 '+d.fg+
     '\n误差 ex='+d.ex+'  ey='+d.ey+'   速度('+d.pv+', '+d.tv+')   目标角('+d.pw+', '+d.tw+')'+
     '\n当前配置：左右'+(d.ps>0?'正':'反')+'  上下'+(d.ts>0?'正':'反')+'  接线'+(d.sw?'【已对调】':'正常')+
     '\nPAN '+d.pan+'°  TILT '+d.tilt+'°  '+d.fps+' fps   标定色度 U='+d.cu+' V='+d.cv+
     '\n舵机 '+(d.se?'已使能':'已松开')+'   跟踪 '+(d.tr?'开':'关')+
     '\n运行 '+d.up+' 秒   启动第 '+d.bc+' 次   上次重启：'+d.rr+
     '\n摄像头 '+d.cam+'（PID 0x'+d.pid.toString(16)+'）</pre>';
 }).catch(()=>{clearTimeout(to);
  document.getElementById('big').textContent='连不上：电脑/手机要连热点 ESP32CAM-TRACK（密码 12345678），地址 192.168.4.1';});
}
frameTick();
statusTick();
setInterval(statusTick,1000);
</script></body></html>)rawliteral";

static void handleApi() {
  char buf[800];
  float fps = fastMode ? fastFps
                       : (frameCount * 1000.0f / max(1.0f, (float)(millis() - fpsT0)));
  snprintf(buf, sizeof(buf),
           "{\"lock\":%d,\"dark\":%d,\"cx\":%d,\"cy\":%d,\"area\":%d,\"thr\":%d,"
           "\"ex\":%.2f,\"ey\":%.2f,\"pan\":%.1f,\"tilt\":%.1f,\"fps\":%.1f,"
           "\"se\":%d,\"tr\":%d,\"bc\":%d,\"rr\":\"%s\",\"up\":%lu,\"cam\":\"%s\",\"pid\":%d,"
           "\"md\":%d,\"cu\":%d,\"cv\":%d,\"tol\":%d,\"camok\":%d,\"fm\":%d,"
           "\"pv\":%.1f,\"tv\":%.1f,\"pw\":%.1f,\"tw\":%.1f,"
           "\"ps\":%d,\"ts\":%d,\"sw\":%d,"
           "\"fg\":%d,\"mred\":%d,\"mmed\":%d,\"ccu\":%d,\"ccv\":%d}",
           tgt.found ? 1 : 0, tgt.dark ? 1 : 0, tgt.cx, tgt.cy, tgt.area, tgt.thr,
           (float)tgt.cx / curW - 0.5f, (float)tgt.cy / curH - 0.5f,
           panPos, tiltPos, fps, servoEnabled ? 1 : 0, tracking ? 1 : 0,
           bootCount, resetReason, (unsigned long)(millis() / 1000),
           camName, camPid, detMode, refU, refV, colorTol, camOk ? 1 : 0, fastMode ? 1 : 0,
           panVel, tiltVel, panWant, tiltWant,
           panSign, tiltSign, swapServo ? 1 : 0,
           lastFg, lastMaxRed, lastMedRed, ctrU, ctrV);
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", buf);
}

static void handleJpg() {
  lastViewer = millis();
  server.sendHeader("Cache-Control", "no-store");
  if (fastMode) {
    // 流畅模式：sensor 输出就是 JPEG，抓一帧直接发，不经过任何软件编码。
    // 这一步省掉的正是跟踪模式里最贵的 fmt2jpg()（QVGA 一次 100ms+）。
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb) {
      // 必须用 send_P：它接受显式长度，会正确写 Content-Length。
      // 之前用 setContentLength + send(200,...,"") + sendContent 是错的 ——
      // WebServer::send() 内部会用那个空字符串的长度 0 覆盖掉 Content-Length，
      // 浏览器收到 "Content-Length: 0"，后面的 JPEG 数据全被当成下一个响应，
      // 解码出来就是上下错位的撕裂画面（用户看到的就是这个）。
      server.send_P(200, "image/jpeg", (PGM_P)fb->buf, fb->len);
      esp_camera_fb_return(fb);
      // 帧率统计：每秒结算一次，网页上那个 fps 才准
      fastCnt++;
      uint32_t t = millis();
      if (fastT0 == 0) fastT0 = t;
      if (t - fastT0 >= 1000) {
        fastFps = fastCnt * 1000.0f / (t - fastT0);
        fastCnt = 0; fastT0 = t;
      }
    } else {
      server.send(503, "text/plain", "no frame");
    }
    return;
  }
  if (sjpg && sjpgLen) server.send_P(200, "image/jpeg", (const char *)sjpg, sjpgLen);
  else server.send(503, "text/plain", "no frame");
}

static void handleSet() {
  // 切换工作模式。必须最先处理：后面所有识别相关参数在流畅模式下都没有意义。
  // 这个请求会阻塞 200~600ms（摄像头要重新初始化），网页那边要等它回来再刷新。
  if (server.hasArg("fm")) {
    bool want = (server.arg("fm").toInt() != 0);
    if (want != fastMode) {
      fastMode = want;
      tracking = false;               // 流畅模式没有识别结果，跟踪自动关掉
      prevCx = prevCy = -1;
      camOk = reinitCamera(fastMode);
      Serial.printf("模式切换 -> %s，摄像头%s\n",
                    fastMode ? "流畅(JPEG直出)" : "跟踪(YUV识别)", camOk ? "就绪" : "失败");
    }
    server.sendHeader("Cache-Control", "no-store");
    server.send(200, "text/plain", camOk ? "ok" : "camfail");
    return;
  }
  // 两个舵机接反时，软件对调输出引脚（不用重新插线）
  if (server.hasArg("sw")) {
    swapServo = !swapServo;
    servoRelease();
    if (servoEnabled) {
      servoAttach();
      servoWrite(PAN_CHANNEL,  panPos);
      servoWrite(TILT_CHANNEL, tiltPos);
    }
  }
  if (server.hasArg("g")) { panGain = constrain(server.arg("g").toFloat(), 1, 400); tiltGain = panGain * 0.75f; }
  if (server.hasArg("d")) deadzone = constrain(server.arg("d").toFloat(), 0, 0.4f);
  if (server.hasArg("n")) slewDps  = constrain(server.arg("n").toFloat(), 10, 600);
  if (server.hasArg("a")) minAreaPct = constrain(server.arg("a").toInt(), 0, 50);
  if (server.hasArg("x")) maxAreaPct = constrain(server.arg("x").toInt(), 5, 95);
  if (server.hasArg("md")) { detMode = constrain(server.arg("md").toInt(), 0, 2); prevCx = prevCy = -1; }
  if (server.hasArg("tol")) colorTol = constrain(server.arg("tol").toInt(), 5, 160);
  if (server.hasArg("cal")) calRequest = true;   // 下一帧用最新画面做标定
  if (server.hasArg("l")) { ledBright = constrain(server.arg("l").toInt(), 0, 100); ledApply(); }
  if (server.hasArg("p")) panSign  = -panSign;
  if (server.hasArg("t")) tiltSign = -tiltSign;
  if (server.hasArg("r")) { panWant = tiltWant = 90; panVel = tiltVel = 0; filEx = filEy = 0; prevCx = prevCy = -1; }
  if (server.hasArg("e")) {
    bool want = (server.arg("e").toInt() != 0);
    // 点"开始跟踪"时如果舵机还没使能，自动帮忙使能 —— 否则 tracking 会被
    // "必须已使能"这个条件吃掉，按钮看着像开了其实没开，舵机一动不动。
    if (want && !servoEnabled) {
      servoEnabled = true;
      panPos = panWant = 90.0f;
      tiltPos = tiltWant = 90.0f;
      panVel = tiltVel = 0;
      servoAttach();
      servoWrite(PAN_CHANNEL, panPos);
      servoWrite(TILT_CHANNEL, tiltPos);
    }
    tracking = want && servoEnabled;
    prevCx = prevCy = -1;        // 重新开始跟踪时清掉"上一帧目标"
  }
  // 使能 / 松开舵机。松开后占空比归零，舵机不通电，可以上手掰
  if (server.hasArg("se")) {
    servoEnabled = (server.arg("se").toInt() != 0);
    if (servoEnabled) {
      // 固定回 90°：用户确认过 90° 一定能转到，不用记住上次位置
      panPos = panWant = 90.0f;
      tiltPos = tiltWant = 90.0f;
      panVel = tiltVel = 0;
      filEx = filEy = 0;
      servoAttach();                 // 先接上 PWM，再写角度
      servoWrite(PAN_CHANNEL, panPos);
      servoWrite(TILT_CHANNEL, tiltPos);
    } else {
      tracking = false;
      servoRelease();
    }
  }
  // 机械限位，边试边收窄
  if (server.hasArg("lo")) panMin  = constrain(server.arg("lo").toFloat(), 0, 180);
  if (server.hasArg("hi")) panMax  = constrain(server.arg("hi").toFloat(), 0, 180);
  if (server.hasArg("tlo")) tiltMin = constrain(server.arg("tlo").toFloat(), 0, 180);
  if (server.hasArg("thi")) tiltMax = constrain(server.arg("thi").toFloat(), 0, 180);
  if (panMin > panMax - 10) panMin = panMax - 10;
  if (tiltMin > tiltMax - 10) tiltMin = tiltMax - 10;
  // 手动微调：每次 5°。微调时先把跟踪停掉，否则下一帧就被控制环覆盖了。
  if (server.hasArg("mv")) {
    tracking = false;
    panVel = tiltVel = 0;          // 手动操作时先停掉速度积分，否则会和按钮抢方向
    int m = server.arg("mv").toInt();
    // 基于 panWant（目标）累加，不是 panPos（实际）：连点几下能稳定累加 5°×N，
    // 用实际位置的话舵机还在路上时基准一直在变，点几下会跑偏。
    if (m == 1) panWant  = clampf(panWant  - 5, panMin,  panMax);
    if (m == 2) panWant  = clampf(panWant  + 5, panMin,  panMax);
    if (m == 3) tiltWant = clampf(tiltWant - 5, tiltMin, tiltMax);
    if (m == 4) tiltWant = clampf(tiltWant + 5, tiltMin, tiltMax);
  }
  server.send(200, "text/plain", "ok");
}

// 热点优先启动。为什么放在最前面：
// WiFi 发射瞬间要拉 300mA 左右，如果供电本来就紧张，等摄像头、舵机都初始化完再开
// 热点，很可能在开热点那一刻电压塌下去直接复位，手机永远搜不到。
// 先开热点 = 上电 1 秒内就能被搜到，而且失败能立刻看到。
static bool startAP() {
  WiFi.persistent(false);
  WiFi.mode(WIFI_AP);
  // 发射功率降到 11dBm：射频启动瞬间的电流峰值明显下降，近距离完全够用。
  // 独立供电时如果还是搜不到，多半就是这里省下来的余量也不够，必须分开供电。
  WiFi.setTxPower(WIFI_POWER_11dBm);

  for (int i = 1; i <= 5; i++) {
    if (WiFi.softAP(AP_SSID, AP_PASS)) {
      Serial.printf("热点已启动：%s  密码 %s  ->  http://%s  通道 %d  MAC %s\n",
                    AP_SSID, AP_PASS, WiFi.softAPIP().toString().c_str(),
                    WiFi.channel(), WiFi.softAPmacAddress().c_str());
      return true;
    }
    Serial.printf("!! softAP 第 %d 次失败，重试\n", i);
    delay(500);
  }
  Serial.println("!! 热点起不来。多半是供电不足：给板子单独一路 5V（≥1A），和舵机分开");
  return false;
}

// 开机自检灯语。拔下来独立供电时看不到串口，靠这颗灯判断板子状态：
//   慢闪 5 下 + 长亮 1 下  = 程序起来了、热点也起来了
//   反复慢闪 5 下          = 在反复重启
//   完全不闪               = 板子没跑起来
// 亮度固定 33%，不刺眼（用 LEDC 调光，不能用 digitalWrite 全亮）。
static void ledLevel(int pct) {
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LED_CHANNEL, (uint32_t)pct * LED_DUTY_MAX / 100);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LED_CHANNEL);
}

static void ledBlink(int times, int onMs, int offMs) {
  for (int i = 0; i < times; i++) {
    ledLevel(33);
    delay(onMs);
    ledLevel(0);
    delay(offMs);
  }
}

void setup() {
  Serial.begin(115200);
  ledInit();                     // 先建好补光灯的 PWM，才能用 1/3 亮度闪
  ledBlink(5, 200, 200);         // 慢闪五下，证明程序起来了
  delay(300);
  Serial.println("\n=== WebTrack：亮度跟踪 + 网页调参 ===");
  readResetReason();
  Serial.printf("本次启动是第 %d 次，上次重启原因：%s\n", bootCount, resetReason);
  Serial.printf("PSRAM: %u\n", psramFound() ? (unsigned)ESP.getPsramSize() : 0u);

  // 1) 热点最先起来
  bool apOk = startAP();
  if (!apOk) {
    // 热点都起不来，继续跑也没意义（网页打不开），重启再来一次
    Serial.println("热点启动失败，3 秒后重启重试");
    delay(3000);
    ESP.restart();
  }
  ledBlink(1, 500, 0);           // 长亮一下 = 热点已就绪

  // 等射频稳定 1.5 秒再去初始化摄像头。两者同时启动会叠加电流峰值，
  // 供电紧张时容易在摄像头初始化那一瞬间塌电压。
  delay(1500);

  // 2) 舵机
  servoInit();
  ledApply();                    // ledInit() 在 setup 开头已经调过了

  // 3) 摄像头。注意：这里【不能】失败就死循环 —— 那样网页服务永远起不来，
  //    用户只会看到"搜不到热点/打不开网页"，完全不知道是摄像头的问题。
  //    现在改成：先让网页起来，摄像头失败就在网页上显示原因，后台每秒重试。
  camOk = initCamera(fastMode);
  if (camOk) {
    camera_fb_t *probe = esp_camera_fb_get();
    if (probe) { curW = probe->width; curH = probe->height; esp_camera_fb_return(probe); }
    Serial.printf("画面 %dx%d，传感器 %s\n", curW, curH, camName);
  } else {
    Serial.println("摄像头起不来，但网页照常开 —— 网页上会显示原因");
  }

  WiFi.setSleep(false);
  MDNS.begin("esp32cam");

  server.on("/", []() { server.send_P(200, "text/html", INDEX); });
  server.on("/api", handleApi);
  server.on("/jpg", handleJpg);
  server.on("/set", handleSet);
  server.begin();
  Serial.println("网页就绪。连热点 ESP32CAM-TRACK（密码 12345678），浏览器开 192.168.4.1");

  // 舵机平滑任务固定跑在 core 0（WiFi 也在这颗核上，但它只在 20ms 里占几十微秒，
  // 优先级给 3 足够及时，又不会把 WiFi 饿着）。loop 在 core 1，互不打扰。
  xTaskCreatePinnedToCore(servoTask, "servo", 3072, NULL, 3, NULL, 0);

  lastMs = fpsT0 = lastPrint = lastSeen = millis();
}

void loop() {
  server.handleClient();
  uint32_t usEncode = 0;

  // 串口发 'd' 吐一张 64x48 缩略图，用来在没有网页时确认画面到底拍到了什么。
  // 只对 YUV422 有效：流畅模式下缓冲是 JPEG，按 YUV 解会全是乱码。
  if (Serial.available()) {
    int c = Serial.read();
    if ((c == 'd' || c == 'D') && !fastMode) {
      camera_fb_t *fbd = esp_camera_fb_get();
      if (fbd) {
        const int W = 64, H = 48;
        const uint16_t *pd = (const uint16_t *)fbd->buf;
        Serial.printf("\nDUMP %d %d\n", W, H);
        char line[W * 5 + 2];
        const int step = fbd->width / (W / 2);      // 每对 YUYV 之间隔多少像素
        for (int oy = 0; oy < H; oy++) {
          const uint16_t *row = pd + (size_t)(oy * fbd->height / H) * fbd->width;
          int p = 0;
          // 连续取两个像素（Y0 U / Y1 V），这样电脑端能还原出真实彩色
          for (int ox = 0; ox < W; ox++) {
            int src = (ox / 2) * step + (ox % 2);
            p += sprintf(line + p, "%04X ", row[src]);
          }
          line[p] = 0;
          Serial.println(line);
        }
        Serial.printf("END\nSTAT ctr U=%d V=%d maxred=%d fg=%d area=%d thr=%d found=%d\n",
                      ctrU, ctrV, lastMaxRed, lastFg, tgt.area, tgt.thr, tgt.found ? 1 : 0);
        esp_camera_fb_return(fbd);
      }
    }
  }

  // 摄像头没起来就每秒重试一次，网页和热点保持可用
  if (!camOk) {
    static uint32_t lastTry = 0;
    if (millis() - lastTry > 1000) {
      lastTry = millis();
      camOk = initCamera(fastMode);
      if (camOk) Serial.println("摄像头重试成功");
    }
    delay(50);
    return;
  }

  // ---- 流畅模式：画面由 /jpg 接口自己抓帧直发，这里完全不碰摄像头 ----
  // 之所以不在这里抓帧再缓存：JPEG 帧 8KB 左右，每帧 memcpy 一次是白费力气，
  // 而且 loop 抓帧会和 /jpg 抢 fb，反而更慢。让请求线程自己去抓是最省的路子。
  if (fastMode) {
    uint32_t nowF = millis();
    float dtF = (nowF - lastMs) / 1000.0f;
    lastMs = nowF;
    if (dtF <= 0.0f || dtF > 0.5f) dtF = 0.05f;

    // 流畅模式下没有识别结果，"丢失扫视"没有意义 —— 只响应方向按钮，
    // 否则舵机会自己转个不停，看着像失控。
    // 舵机位置由 servoTask 以 50Hz 平滑追过去，这里不再直接写 PWM。

    static uint32_t lastPrintF = 0;
    if (nowF - lastPrintF >= 1000) {
      Serial.printf("流畅模式 PAN=%.1f TILT=%.1f 堆=%u\n",
                    panPos, tiltPos, (unsigned)ESP.getFreeHeap());
      lastPrintF = nowF;
    }
    delay(2);          // 让出 CPU 给 WiFi 协议栈，别把发送线程饿死
    return;
  }

  // 分段计时，用来定位到底卡在哪：抓帧 / 降采样 / 识别 / 编码
  uint32_t tGrab0 = micros();
  camera_fb_t *fb = esp_camera_fb_get();
  uint32_t usGrab = micros() - tGrab0;
  if (!fb) { delay(20); return; }
  curW = fb->width; curH = fb->height;
  uint32_t tB0 = micros();
  buildFeatures((const uint16_t *)fb->buf, curW, curH);
  uint32_t usFeat = micros() - tB0;
  if (calRequest) { calibrateColor(); calRequest = false; }
  {
    int ci = (DH / 2) * DW + DW / 2;      // 画面正中那一格的色度，标定前先看一眼
    ctrU = dU[ci]; ctrV = dV[ci];
  }
  uint32_t tF0 = micros();
  findTarget();
  uint32_t usFind = micros() - tF0;

  uint32_t now = millis();
  bool viewer = (now - lastViewer) < 4000;   // 4 秒内有人看才做 JPEG 编码
  // 编码降到 160x120 后只要 25ms 左右，可以编得更勤：45ms 一次 ≈ 22fps。
  // 之前是 140ms（因为 320x240 编码要 100ms+），这就是画面卡顿的根源。
  bool encode = viewer && (now - lastJpg >= 45);

  if (encode) {
    uint16_t *p = (uint16_t *)fb->buf;
    if (tgt.found) {
      drawBox(p, curW, curH, tgt.x0, tgt.y0, tgt.x1, tgt.y1, COL_BOX);
      hline(p, curW, curH, tgt.cx - 6, tgt.cx + 6, tgt.cy, COL_CENT);
      vline(p, curW, curH, tgt.cx, tgt.cy - 6, tgt.cy + 6, COL_CENT);
    }
    hline(p, curW, curH, curW / 2 - 8, curW / 2 + 8, curH / 2, COL_CROSS);
    vline(p, curW, curH, curW / 2, curH / 2 - 8, curH / 2 + 8, COL_CROSS);

    // 编码：QQVGA 时画面本身就是 160x120，直接编；万一是 QVGA 才降采样。
    uint8_t *encSrc = fb->buf;
    int encW = curW, encH = curH;
    if (curW > HALF_W) {
      if (!yuvHalf) yuvHalf = (uint8_t *)ps_malloc(HALF_W * HALF_H * 2);
      if (yuvHalf) {
        downYuv2(fb->buf, curW, curH, yuvHalf);
        encSrc = yuvHalf; encW = HALF_W; encH = HALF_H;
      }
    }
    uint32_t tE0 = micros();
    if (sjpg) { free(sjpg); sjpg = NULL; sjpgLen = 0; }
    if (!fmt2jpg(encSrc, encW * encH * 2, encW, encH,
                 PIXFORMAT_YUV422, 55, &sjpg, &sjpgLen)) {
      if (sjpg) { free(sjpg); }        // 失败时也可能已经分配了，不清会漏内存
      sjpg = NULL; sjpgLen = 0;
    }
    usEncode = micros() - tE0;
    lastJpg = now;
  }
  esp_camera_fb_return(fb);

  // 控制律已经改成输出角速度，不再需要 dt；lastMs 仍要更新，切模式时算间隔用。
  lastMs = now;

  locked = tgt.found && tracking && servoEnabled;
  if (!servoEnabled || !tracking) {
    // 舵机没使能、或者跟踪关着：位置一律不动，画面照常刷新
    filEx = filEy = 0;
    panVel = tiltVel = 0;
  } else if (locked) {
    lastSeen = now;
    float ex = (float)tgt.cx / curW - 0.5f;
    float ey = (float)tgt.cy / curH - 0.5f;
    filEx = filEx * 0.25f + ex * 0.75f;   // 更信当前帧，减小滞后（原来 0.65 有拖尾感）
    filEy = filEy * 0.25f + ey * 0.75f;
    // 输出【角速度】而不是位置增量 —— 这是关键改动。
    // 摄像头 7fps 意味着这里 143ms 才执行一次，如果直接改 panWant，
    // 舵机追到就停住等下一帧，看起来就是一顿一顿。改成给速度后，
    // servoTask 每 20ms 拿这个速度积分一次，舵机就是连续滑动的。
    panVel  = (fabs(filEx) > deadzone) ? panGain  * (filEx * panSign)  : 0.0f;
    tiltVel = (fabs(filEy) > deadzone) ? tiltGain * (filEy * tiltSign) : 0.0f;
    // 大误差时别甩飞：角速度不超过限速值
    panVel  = clampf(panVel,  -slewDps, slewDps);
    tiltVel = clampf(tiltVel, -slewDps, slewDps);
  } else {
    filEx = filEy = 0;
    panVel = tiltVel = 0;
    if (sweepWhenLost && now - lastSeen > 1200) {
      // 丢失目标扫视：同样给速度，边界处换向
      if (panWant >= panMax - 2.0f) sweepDir = -1;
      if (panWant <= panMin + 2.0f) sweepDir =  1;
      panVel = sweepDir * 30.0f;
    }
  }

  // 舵机由 servoTask 以 50Hz 平滑追踪 panWant，这里不再直接写 PWM。
  // 以前在这里按 loop 频率（7~11fps）更新，舵机每秒只走 7~11 步，所以一顿一顿。

  frameCount++;
  if (now - lastPrint >= 1000) {
    float fps = frameCount * 1000.0f / (now - fpsT0);
    const char *st = !servoEnabled ? "待机" : (!tracking ? "暂停" : (locked ? "锁定" : "搜索"));
    Serial.printf("%s 面积=%4d 中心=(%3d,%3d) 阈值=%3d PAN=%.1f TILT=%.1f %.1ffps\n",
                  st, tgt.area, tgt.cx, tgt.cy, tgt.thr, panPos, tiltPos, fps);
    // 耗时分解：抓帧/降采样特征/找目标/编码。哪一项大就说明卡在哪。
    Serial.printf("   抓帧 %lu us  特征 %lu us  找目标 %lu us  编码 %lu us\n",
                  (unsigned long)usGrab, (unsigned long)usFeat,
                  (unsigned long)usFind, (unsigned long)usEncode);
    frameCount = 0; fpsT0 = now; lastPrint = now;
  }
}
