// main.cpp
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <Ticker.h>

// WiFi
const char* ssid = "Tầng 3";
const char* password = "12345678";

// Motor pins (L298N)
const int motorPins[4][2] = {
  {25, 26}, // Motor 1
  {33, 32}, // Motor 2
  {18, 5},  // Motor 3
  {21, 19}  // Motor 4
};

const int pwmChannels[4][2] = {
  {0, 1}, {2, 3}, {4, 5}, {6, 7}
};

const int freq = 30000;
const int resolution = 8;

// Robot geometry
const float lx = 0.075;
const float ly = 0.1275;
const float l = lx + ly;
const float r = 0.03;


// Encoder & PID
volatile int encoderCounts[4] = {0, 0, 0, 0};
float actualSpeeds[4] = {0, 0, 0, 0};
float targetSpeeds[4] = {0, 0, 0, 0};
float pidOutput[4] = {0, 0, 0, 0};

float Kp = 30.0, Ki = 0.0, Kd = 0.0;
float prevError[4] = {0, 0, 0, 0};
float integral[4] = {0, 0, 0, 0};
float dt = 0.05; // 50ms sampling

// Web server
AsyncWebServer server(80);

// ---------------- Hàm encoder ----------------
void IRAM_ATTR countEncoder0() { encoderCounts[0]++; }
void IRAM_ATTR countEncoder1() { encoderCounts[1]++; }
void IRAM_ATTR countEncoder2() { encoderCounts[2]++; }
void IRAM_ATTR countEncoder3() { encoderCounts[3]++; }

void setupPWM() {
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 2; j++) {
      ledcSetup(pwmChannels[i][j], freq, resolution);
      ledcAttachPin(motorPins[i][j], pwmChannels[i][j]);
    }
  }
}

void driveMotor(int i, float value) {
  value = constrain(value, -255, 255);
  if (value > 0) {
    ledcWrite(pwmChannels[i][0], value);
    ledcWrite(pwmChannels[i][1], 0);
  } else {
    ledcWrite(pwmChannels[i][0], 0);
    ledcWrite(pwmChannels[i][1], -value);
  }
}

void computePID() {
  for (int i = 0; i < 4; i++) {
    float error = targetSpeeds[i] - actualSpeeds[i];
    integral[i] += error * dt;
    float derivative = (error - prevError[i]) / dt;
    pidOutput[i] = Kp * error + Ki * integral[i] + Kd * derivative;
    prevError[i] = error;
    driveMotor(i, pidOutput[i]);
  }
}

void updateSpeeds() {
  for (int i = 0; i < 4; i++) {
    actualSpeeds[i] = ((float)encoderCounts[i] / 20.0) / dt * 2 * PI; // rad/s
    encoderCounts[i] = 0;
  }
}

Ticker controlLoop;
void controlTask() {
  updateSpeeds();
  computePID();
}

void computeWheelSpeeds(float vx, float vy, float omega) {
  targetSpeeds[0] = (1.0/r)*(vx - vy - l * omega);
  targetSpeeds[1] = (1.0/r)*(vx + vy + l * omega);
  targetSpeeds[2] = (1.0/r)*(vx - vy + l * omega);
  targetSpeeds[3] = (1.0/r)*(vx + vy - l * omega);
}

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
  Serial.println(WiFi.localIP());

  if (!LittleFS.begin()) {
    Serial.println("LittleFS mount failed");
    return;
  }

  setupPWM();

    // Gắn ngắt encoder

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(LittleFS, "/index.html", "text/html");
  });

  server.on("/cmd", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("vx") && request->hasParam("vy") && request->hasParam("omega")) {
      float vx = request->getParam("vx")->value().toFloat();
      float vy = request->getParam("vy")->value().toFloat();
      float omega = request->getParam("omega")->value().toFloat();
      computeWheelSpeeds(vx, vy, omega);
      request->send(200, "text/plain", "OK");
    } else {
      request->send(400, "text/plain", "Missing parameters");
    }
  });

  
  server.begin();
  controlLoop.attach(dt, controlTask);
}

void loop() {
  // Không cần xử lý gì ở đây
}