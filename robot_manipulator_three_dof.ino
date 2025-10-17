#include <Stepper.h>

#define STEPS_PER_REV 2048

// Define 3 steppers for a 3-DOF arm
Stepper motors[3] = {
  Stepper(STEPS_PER_REV, 22, 23, 24, 25),   // Joint 1
  Stepper(STEPS_PER_REV, 26, 27, 28, 29),   // Joint 2
  Stepper(STEPS_PER_REV, 30, 31, 32, 33)    // Joint 3
};

float currentAngles[3] = {0, 0, 0};  // Current joint angles in degrees

// ---------- Parse incoming CSV "a1,a2,a3" ----------
bool parseAngles(String str, float angles[3]) {
  int comma1 = str.indexOf(',');
  int comma2 = str.indexOf(',', comma1 + 1);
  if (comma1 == -1 || comma2 == -1) return false;

  angles[0] = str.substring(0, comma1).toFloat();
  angles[1] = str.substring(comma1 + 1, comma2).toFloat();
  angles[2] = str.substring(comma2 + 1).toFloat();
  return true;
}

// ---------- Move motors to target angles ----------
void moveToAngles(float targetAngles[3]) {
  for (int i = 0; i < 3; i++) {
    float deltaDeg = targetAngles[i] - currentAngles[i];
    long steps = (long)(deltaDeg / 360.0 * STEPS_PER_REV);

    if (steps != 0) {
      Serial.print("Joint "); Serial.print(i+1);
      Serial.print(" moving "); Serial.print(steps);
      Serial.println(" steps");
      motors[i].step(steps);
      currentAngles[i] = targetAngles[i];
    }
  }
  Serial.println("OK");   // Send ack to MATLAB
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  for (int i = 0; i < 3; i++) {
    motors[i].setSpeed(50);  // Increase speed (RPM)
  }
  Serial.println("READY");   // MATLAB waits for this exact word
}

void loop() {
  static String input = "";

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {  // full line received
      input.trim();
      if (input.length() > 0) {
        float targetAngles[3];
        if (parseAngles(input, targetAngles)) {
          moveToAngles(targetAngles);
        } else {
          Serial.println("INVALID");
        }
      }
      input = "";  // clear for next line
    } else {
      input += c;
    }
  }
}
