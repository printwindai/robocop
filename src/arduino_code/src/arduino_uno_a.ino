#include <WiFiS3.h>
#include <WiFiUDP.h>

const char* SSID = "BT-9FAKX7";
const char* PASS = "ngPJgRakPLe4ab";

const uint16_t UDP_PORT = 5555;   // Board A

WiFiUDP udp;

// CNC Shield v3 pin mapping
const int AXIS_COUNT = 3;
const char AXIS_NAMES[AXIS_COUNT] = {'X', 'Y', 'Z'};

// STEP pins: X=2, Y=3, Z=4
const int STEP_PINS[AXIS_COUNT] = {2, 3, 4};
// DIR pins:  X=5, Y=6, Z=7
const int DIR_PINS[AXIS_COUNT]  = {5, 6, 7};

// ENABLE pin (active LOW)
const int EN_PIN = 8;

void enableMotors(bool enable) {
  digitalWrite(EN_PIN, enable ? LOW : HIGH); // LOW = enable
}

int axisIndexFromChar(char c) {
  c = toupper(c);
  for (int i = 0; i < AXIS_COUNT; i++) {
    if (AXIS_NAMES[i] == c) return i;
  }
  return -1;
}

// Blocking move: steps can be positive or negative
void moveAxis(int axisIndex, long steps, unsigned int us_per_step) {
  if (axisIndex < 0 || axisIndex >= AXIS_COUNT) return;
  if (steps == 0) return;

  int dir = (steps > 0) ? HIGH : LOW;
  long count = labs(steps);

  int stepPin = STEP_PINS[axisIndex];
  int dirPin  = DIR_PINS[axisIndex];

  digitalWrite(dirPin, dir);

  enableMotors(true);

  for (long i = 0; i < count; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(us_per_step);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(us_per_step);
  }

  enableMotors(false); // optional: disable between moves
}

void stopAll() {
  // For this simple blocking implementation, STOP is just "disable drivers"
  enableMotors(false);
}

void connectWiFi() {
  Serial.print("Connecting to WiFi SSID: ");
  Serial.println(SSID);

WiFi.begin(SSID, PASS);

// Wait for connection AND IP
while (WiFi.status() != WL_CONNECTED) {
  Serial.print(".");
  delay(500);
}

Serial.println("\nWiFi connected!");

// Force DHCP renew
IPAddress ip = WiFi.localIP();
while (ip == INADDR_NONE || ip.toString() == "0.0.0.0") {
  Serial.println("Waiting for IP...");
  delay(500);
  ip = WiFi.localIP();
}

  Serial.println("WiFi connected!");
  Serial.print("A IP: ");
  Serial.println(ip);
  Serial.print("A UDP port: ");
  Serial.println(UDP_PORT);
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }

  Serial.println("Board A booting...");

  // Configure pins
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, HIGH); // disable by default

  for (int i = 0; i < AXIS_COUNT; i++) {
    pinMode(STEP_PINS[i], OUTPUT);
    pinMode(DIR_PINS[i], OUTPUT);
    digitalWrite(STEP_PINS[i], LOW);
    digitalWrite(DIR_PINS[i], LOW);
  }

  connectWiFi();
  udp.begin(UDP_PORT);
}

void loop() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    char buf[80];
    int len = udp.read(buf, sizeof(buf) - 1);
    buf[len] = 0;

    Serial.print("RX(A): ");
    Serial.println(buf);

    // MOVE X <steps> <us_per_step>
    // MOVE Y <steps> <us_per_step>
    // MOVE Z <steps> <us_per_step>
    char axisChar;
    long steps = 0;
    unsigned int us_per_step = 800; // default microseconds between steps

    if (sscanf(buf, "MOVE %c %ld %u", &axisChar, &steps, &us_per_step) == 3) {
      int axisIndex = axisIndexFromChar(axisChar);
      if (axisIndex != -1) {
        moveAxis(axisIndex, steps, us_per_step);
      }
    } else if (strncmp(buf, "STOP", 4) == 0) {
      stopAll();
    }
  }
}
