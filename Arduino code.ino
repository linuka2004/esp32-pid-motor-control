// 4.9.2026---->WiFi PID Controller

#include <WiFi.h>

#define ENB_PIN         25
#define ENC_A_PIN       18
#define ENC_B_PIN       19
#define SAMPLE_TIME_MS  50
#define PWM_FREQ        5000
#define PWM_BITS        8
#define LOG_INTERVAL_MS 100
#define MIN_PWM         0
#define MAX_PWM         255

// --- WiFi Settings ---
const char* ssid = "Dialog 4G 464";        //Router name  "Dialog 4G 128"
const char* password = "5c3e74e1"; // Password
const int port = 8080;

WiFiServer server(port);
WiFiClient client;

float PPR = 350.0;

double Kp = 2.5;
double Ki = 1.5;
double Kd = 0.03;
double targetRPM = 250.0;

volatile long encoderCount = 0;
long lastEncoderCount      = 0;
double measuredRPM         = 0.0;
double pidOutput           = 0.0;
double errorSum            = 0.0;
double lastError           = 0.0;
unsigned long lastTime     = 0;
unsigned long lastLogTime  = 0;

void IRAM_ATTR encoderISR_A() {
  if (digitalRead(ENC_B_PIN) == HIGH) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}

void setMotorPWM(int pwmValue) {
  pwmValue = constrain(pwmValue, MIN_PWM, MAX_PWM);
  ledcWrite(ENB_PIN, pwmValue);
}

double calculateRPM() {
  long currentCount = encoderCount;
  long delta        = currentCount - lastEncoderCount;
  lastEncoderCount  = currentCount;
  double rpm = ((double)delta / PPR) / (SAMPLE_TIME_MS / 60000.0);
  return abs(rpm);
}

double computePID(double setpoint, double measured) {
  double dt     = SAMPLE_TIME_MS / 1000.0;
  double error  = setpoint - measured;

  errorSum     += error * dt;
  errorSum      = constrain(errorSum, -300.0, 300.0); 

  double dError = (error - lastError) / dt;
  lastError     = error;

  double output = (Kp * error) + (Ki * errorSum) + (Kd * dError);
  return constrain(output, MIN_PWM, MAX_PWM);
}

void setup() {
  Serial.begin(115200);
  delay(500);

  // --- Connect to WiFi ---
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n--- WiFi Connected ---");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());  // <-- YOU NEED THIS IP FOR THE GUI
  
  server.begin();
  Serial.println("TCP Server started on port 8080.");

  ledcAttach(ENB_PIN, PWM_FREQ, PWM_BITS);
  ledcWrite(ENB_PIN, 0);

  pinMode(ENC_A_PIN, INPUT_PULLUP);
  pinMode(ENC_B_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), encoderISR_A, RISING);

  // Reset state
  encoderCount     = 0;
  lastEncoderCount = 0;
  errorSum         = 0.0;
  lastError        = 0.0;
  lastTime         = millis();
  lastLogTime      = millis();
}

void loop() {
  unsigned long now = millis();

  // --- Handle WiFi Client Connection ---
  if (server.hasClient()) {
    if (!client || !client.connected()) {
      if (client) client.stop();
      client = server.available();
      Serial.println(">>> GUI Client Connected!");
    } else {
      server.available().stop(); // Reject additional clients if one is already connected
    }
  }

  // --- PID Computation Loop ---
  if (now - lastTime >= SAMPLE_TIME_MS) {
    lastTime    = now;
    measuredRPM = calculateRPM();
    pidOutput   = computePID(targetRPM, measuredRPM);
    setMotorPWM((int)pidOutput);
  }

  // --- Sending Data to GUI ---
  if (now - lastLogTime >= LOG_INTERVAL_MS) {
    lastLogTime = now;
    String logMsg = "Setpoint:" + String(targetRPM, 1) + 
                    ",Measured:" + String(measuredRPM, 1) + 
                    ",PWM:" + String((int)pidOutput) + "\n";
    
    if (client && client.connected()) {
      client.print(logMsg); // Send over WiFi
    }
    Serial.print(logMsg);   // Keep sending to Serial Monitor for debugging
  }

  // --- Receiving Data from GUI ---
  if (client && client.connected() && client.available()) {
    String input = client.readStringUntil('\n');
    input.trim();
    
    int firstComma = input.indexOf(',');
    if (firstComma > 0) {
      int secondComma = input.indexOf(',', firstComma + 1);
      int thirdComma  = input.indexOf(',', secondComma + 1);
      
      if (secondComma > 0 && thirdComma > 0) {
        targetRPM = input.substring(0, firstComma).toDouble();
        Kp = input.substring(firstComma + 1, secondComma).toDouble();
        Ki = input.substring(secondComma + 1, thirdComma).toDouble();
        Kd = input.substring(thirdComma + 1).toDouble();
        
        errorSum  = 0.0;
        lastError = 0.0;
        
        Serial.print(">>> Updated via WiFi -> RPM: "); Serial.print(targetRPM);
        Serial.print(" | Kp: "); Serial.print(Kp);
        Serial.print(" | Ki: "); Serial.print(Ki);
        Serial.print(" | Kd: "); Serial.println(Kd);
      }
    }
  }
}