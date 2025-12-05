#include <WiFiS3.h>            // Include UNO R4 WiFi networking support (ESP32-S3 coprocessor)
#include <WiFiUDP.h>           // Include UDP helper class for sending/receiving UDP packets

const char* SSID = "robot_net";       // Your Wi-Fi SSID (network name)
const char* PASS = "strongpassword";  // Your Wi-Fi password

const uint16_t UDP_PORT = 5556;       // UDP port this board listens on (A=5555, B=5556)

WiFiUDP udp;                           // Create a global UDP socket object

const int MOTOR_COUNT = 3;             // Number of motors controlled by this board
const int PWM_PINS[MOTOR_COUNT] = {5, 6, 9};  // PWM output pins for motors 0, 1, 2 (EDIT to match wiring)
const int DIR_PINS[MOTOR_COUNT] = {4, 7, 8};  // Direction pins for motors 0, 1, 2 (EDIT to match wiring)

unsigned long stop_at[MOTOR_COUNT] = {0, 0, 0}; // Per-motor absolute stop times (in millis since boot); 0 = inactive

void setMotor(int i, int pwm) {        // Set motor i speed using PWM (0..255)
  if (i < 0 || i >= MOTOR_COUNT) return;       // Ignore invalid motor index
  pwm = constrain(pwm, 0, 255);                // Clamp PWM to 0..255 for safety
  digitalWrite(DIR_PINS[i], HIGH);             // Fix direction to "forward" for this demo
  analogWrite(PWM_PINS[i], pwm);               // Output PWM to the motor’s speed pin
}

void stopMotor(int i) {                 // Immediately stop motor i
  if (i < 0 || i >= MOTOR_COUNT) return;       // Ignore invalid motor index
  analogWrite(PWM_PINS[i], 0);                 // Set PWM to 0 (stop)
  stop_at[i] = 0;                               // Clear its scheduled stop time
}

void stopAll() {                         // Stop all motors
  for (int i = 0; i < MOTOR_COUNT; i++) stopMotor(i); // Loop over all motors and stop each
}

void setup() {                           // Arduino setup: runs once after reset/power-up
  Serial.begin(115200);                  // Start serial port for debug prints at 115200 baud

  for (int i = 0; i < MOTOR_COUNT; i++) {   // Configure all motor pins
    pinMode(PWM_PINS[i], OUTPUT);           // Make PWM pin an output
    pinMode(DIR_PINS[i], OUTPUT);           // Make direction pin an output
    stopMotor(i);                            // Ensure motor starts stopped
    digitalWrite(DIR_PINS[i], HIGH);         // Default direction "forward"
  }

  WiFi.begin(SSID, PASS);                // Begin connecting to the Wi-Fi network
  while (WiFi.status() != WL_CONNECTED) {   // Wait here until Wi-Fi connection is established
    delay(200);                              // Brief pause between connection checks
  }

  udp.begin(UDP_PORT);                   // Start listening for UDP packets on the chosen port

  Serial.print("A IP: ");                // Print a label for the IP (change to "B" on the 2nd board if you like)
  Serial.println(WiFi.localIP());        // Print this board’s assigned IP address
  Serial.print("A UDP port: ");          // Print a label for the UDP port
  Serial.println(UDP_PORT);              // Print the UDP port number
}

void loop() {                            // Main loop: runs repeatedly
  int packetSize = udp.parsePacket();    // Check if a UDP packet has arrived; returns its size (0 if none)
  if (packetSize) {                      // If at least one packet is pending…
    char buf[80];                        // Allocate a small buffer to hold the incoming text command
    int len = udp.read(buf, sizeof(buf) - 1); // Read up to buffer-1 bytes from the packet
    buf[len] = 0;                        // Null-terminate so we can treat it as a C-string

    Serial.print("RX: "); Serial.println(buf); // DEBUG: print the received command to Serial

    // Supported command formats:
    //  1) "SPIN M<idx> <pwm> <ms>"    e.g. "SPIN M1 180 1500" → spin motor 1 at PWM=180 for 1500 ms
    //  2) "SPINALL <pwm> <ms>"        e.g. "SPINALL 160 2000" → spin ALL motors for 2000 ms
    //  3) "STOP"                      immediately stop all motors
    if (strncmp(buf, "SPIN M", 6) == 0) {       // If command begins with "SPIN M"
      int idx = 0, pwm = 150, dur = 1000;       // Default values in case parsing fails
      if (sscanf(buf, "SPIN M%d %d %d", &idx, &pwm, &dur) == 3) { // Extract motor index, PWM, duration
        if (idx >= 0 && idx < MOTOR_COUNT) {    // Check motor index is valid
          setMotor(idx, pwm);                   // Apply the requested PWM to that motor
          stop_at[idx] = millis() + (unsigned long)dur; // Schedule auto-stop time = now + duration
        }
      }
    } else if (strncmp(buf, "SPINALL", 7) == 0) { // If command is "SPINALL"
      int pwm = 150, dur = 1000;                  // Defaults
      if (sscanf(buf, "SPINALL %d %d", &pwm, &dur) == 2) { // Extract PWM and duration
        pwm = constrain(pwm, 0, 255);             // Clamp PWM for safety
        unsigned long tstop = millis() + (unsigned long)dur; // Compute shared stop time once
        for (int i = 0; i < MOTOR_COUNT; i++) {   // Apply to all motors
          setMotor(i, pwm);                        // Set each motor to the same PWM
          stop_at[i] = tstop;                      // Schedule the same auto-stop time
        }
      }
    } else if (strncmp(buf, "STOP", 4) == 0) {   // If command is "STOP"
      stopAll();                                  // Immediately stop every motor
    }
  }

  unsigned long now = millis();          // Read current time in milliseconds since boot
  for (int i = 0; i < MOTOR_COUNT; i++) {      // Check all motors for elapsed timers
    if (stop_at[i] && now > stop_at[i]) {      // If timer is active and has expired…
      stopMotor(i);                             // …stop that motor
    }
  }
}
