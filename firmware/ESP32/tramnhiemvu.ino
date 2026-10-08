#include <WiFi.h>
#include <WiFiUdp.h>

// =====================================================
// WIFI / MINIPC
// =====================================================
const char* WIFI_SSID = "VK2-101";

// ===== MỖI NGÀY KIỂM TRA IP MINIPC VÀ CHỈ SỬA DÒNG NÀY =====
const char* MINIPC_IP = "192.168.13.17";

const uint16_t MINIPC_PORT = 5005;
const uint16_t ESP32_LOCAL_PORT = 5006;

WiFiUDP udp;


// =====================================================
// HY-SRF05 PINS
// =====================================================
#define TRIG1 25
#define ECHO1 34

#define TRIG2 17
#define ECHO2 19


// =====================================================
// FILTER
// =====================================================
#define N 5

float buf1[N] = {0};
float buf2[N] = {0};

int idx1 = 0;
int idx2 = 0;

bool fill1 = false;
bool fill2 = false;

float ema1 = -1;
float ema2 = -1;

const float EMA_ALPHA = 0.25;

// Tạm giữ ngưỡng đã test
const float DETECT_ON_CM  = 100.0;
const float DETECT_OFF_CM = 120.0;

const int ON_CONFIRM  = 4;
const int OFF_CONFIRM = 5;

int onCount1 = 0;
int offCount1 = 0;
int onCount2 = 0;
int offCount2 = 0;

bool detected1 = false;
bool detected2 = false;

// Để chỉ gửi khi trạng thái THỰC SỰ thay đổi
bool lastSent1 = false;
bool lastSent2 = false;

bool firstState1 = true;
bool firstState2 = true;


// =====================================================
// RAW DISTANCE
// =====================================================
float readRaw(int trigPin, int echoPin)
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(3);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duration =
      pulseIn(echoPin, HIGH, 30000);

  if (duration == 0)
    return -1;

  float cm = duration * 0.0343f / 2.0f;

  if (cm < 2.0 || cm > 400.0)
    return -1;

  return cm;
}


// =====================================================
// MEDIAN
// =====================================================
float median5(float *a)
{
  float x[N];

  for (int i = 0; i < N; i++)
    x[i] = a[i];

  for (int i = 0; i < N - 1; i++)
  {
    for (int j = i + 1; j < N; j++)
    {
      if (x[j] < x[i])
      {
        float t = x[i];
        x[i] = x[j];
        x[j] = t;
      }
    }
  }

  return x[N / 2];
}


// =====================================================
// FILTER UPDATE
// =====================================================
float updateFilter(
    float raw,
    float *buf,
    int &idx,
    bool &filled,
    float &ema)
{
  if (raw < 0)
    return ema;

  buf[idx] = raw;
  idx++;

  if (idx >= N)
  {
    idx = 0;
    filled = true;
  }

  if (!filled)
  {
    if (ema < 0)
      ema = raw;
    else
      ema =
          EMA_ALPHA * raw +
          (1.0 - EMA_ALPHA) * ema;

    return ema;
  }

  float med = median5(buf);

  if (ema < 0)
    ema = med;
  else
    ema =
        EMA_ALPHA * med +
        (1.0 - EMA_ALPHA) * ema;

  return ema;
}


// =====================================================
// DETECTION + HYSTERESIS
// =====================================================
void updateDetection(
    float distance,
    bool &detected,
    int &onCount,
    int &offCount)
{
  if (distance < 0)
    return;

  if (!detected)
  {
    if (distance <= DETECT_ON_CM)
    {
      onCount++;

      if (onCount >= ON_CONFIRM)
      {
        detected = true;
        onCount = 0;
        offCount = 0;
      }
    }
    else
    {
      onCount = 0;
    }
  }
  else
  {
    if (distance >= DETECT_OFF_CM)
    {
      offCount++;

      if (offCount >= OFF_CONFIRM)
      {
        detected = false;
        onCount = 0;
        offCount = 0;
      }
    }
    else
    {
      offCount = 0;
    }
  }
}


// =====================================================
// UDP SEND + ACK
// =====================================================
bool sendEvent(const char* msg)
{
  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("[NET] WiFi disconnected");
    return false;
  }

  for (int attempt = 1; attempt <= 3; attempt++)
  {
    Serial.print("[TX] ");
    Serial.print(msg);
    Serial.print(" attempt=");
    Serial.println(attempt);

    udp.beginPacket(MINIPC_IP, MINIPC_PORT);
    udp.print(msg);
    udp.endPacket();

    unsigned long t0 = millis();

    while (millis() - t0 < 600)
    {
      int packetSize = udp.parsePacket();

      if (packetSize > 0)
      {
        char buf[160];
        int len = udp.read(buf, sizeof(buf) - 1);

        if (len > 0)
        {
          buf[len] = '\0';

          Serial.print("[RX] ");
          Serial.println(buf);

          String expected = "ACK:";
          expected += msg;

          if (String(buf) == expected)
          {
            Serial.println("[NET] ACK PASS");
            return true;
          }
        }
      }

      delay(10);
    }

    Serial.println("[NET] ACK timeout");
  }

  Serial.println("[NET] EVENT FAILED");
  return false;
}


// =====================================================
// WIFI CONNECT
// =====================================================
void connectWiFi()
{
  Serial.print("[WIFI] Connecting to ");
  Serial.println(WIFI_SSID);

  WiFi.mode(WIFI_STA);

  // Wi-Fi mở, không password
  WiFi.begin(WIFI_SSID);

  unsigned long t0 = millis();

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");

    if (millis() - t0 > 20000)
    {
      Serial.println();
      Serial.println("[WIFI] CONNECT TIMEOUT");
      return;
    }
  }

  Serial.println();
  Serial.println("[WIFI] CONNECTED");

  Serial.print("[WIFI] ESP32 IP = ");
  Serial.println(WiFi.localIP());

  udp.begin(ESP32_LOCAL_PORT);

  Serial.print("[UDP] local port = ");
  Serial.println(ESP32_LOCAL_PORT);

  sendEvent("ESP32_ONLINE");
}


// =====================================================
// SETUP
// =====================================================
void setup()
{
  Serial.begin(115200);

  pinMode(TRIG1, OUTPUT);
  pinMode(ECHO1, INPUT);

  pinMode(TRIG2, OUTPUT);
  pinMode(ECHO2, INPUT);

  digitalWrite(TRIG1, LOW);
  digitalWrite(TRIG2, LOW);

  delay(1000);

  Serial.println();
  Serial.println("=================================");
  Serial.println(" ESP32 2x SRF05 WIFI MONITOR V1");
  Serial.println("=================================");

  connectWiFi();
}


// =====================================================
// LOOP
// =====================================================
void loop()
{
  // Reconnect Wi-Fi nếu mất mạng
  if (WiFi.status() != WL_CONNECTED)
  {
    connectWiFi();
  }

  // ---------------- SENSOR 1 ----------------
  float raw1 = readRaw(TRIG1, ECHO1);

  float d1 = updateFilter(
      raw1,
      buf1,
      idx1,
      fill1,
      ema1);

  updateDetection(
      d1,
      detected1,
      onCount1,
      offCount1);

  // chống nhiễu chéo
  delay(45);

  // ---------------- SENSOR 2 ----------------
  float raw2 = readRaw(TRIG2, ECHO2);

  float d2 = updateFilter(
      raw2,
      buf2,
      idx2,
      fill2,
      ema2);

  updateDetection(
      d2,
      detected2,
      onCount2,
      offCount2);

  delay(70);


  // =================================================
  // SENSOR 1 EVENT
  // =================================================
  if (firstState1 || detected1 != lastSent1)
  {
    const char* msg =
        detected1 ? "SENSOR1_OBJECT"
                  : "SENSOR1_CLEAR";

    if (sendEvent(msg))
    {
      lastSent1 = detected1;
      firstState1 = false;
    }
  }


  // =================================================
  // SENSOR 2 EVENT
  // =================================================
  if (firstState2 || detected2 != lastSent2)
  {
    const char* msg =
        detected2 ? "SENSOR2_OBJECT"
                  : "SENSOR2_CLEAR";

    if (sendEvent(msg))
    {
      lastSent2 = detected2;
      firstState2 = false;
    }
  }


  // =================================================
  // SERIAL MONITOR
  // =================================================
  Serial.print("S1 RAW=");
  Serial.print(raw1, 1);

  Serial.print(" FIL=");
  Serial.print(d1, 1);

  Serial.print(
      detected1 ? " [OBJECT]" : " [CLEAR]");

  Serial.print("   |   ");

  Serial.print("S2 RAW=");
  Serial.print(raw2, 1);

  Serial.print(" FIL=");
  Serial.print(d2, 1);

  Serial.println(
      detected2 ? " [OBJECT]" : " [CLEAR]");

  delay(40);
}