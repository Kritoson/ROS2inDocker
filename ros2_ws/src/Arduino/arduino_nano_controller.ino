#include <Servo.h>

// ============================================================================
//   Professional Motor PWM Controller for Robot
//   Rotation + Throttle   → Servo pulses
//   Brush Motor PWM       → Servo pulse (sabit duty)
//   Includes: Failsafe, Ramp, Serial protocol from Jetson
// ============================================================================

// ------------------------- PWM Pin Definitions -------------------------------
const int PIN_ROTATION = 3;
const int PIN_THROTTLE = 5;
const int PIN_BRUSH    = 6;

// ------------------------- Target PWM Values (µs) ---------------------------
volatile int targetRotation = 1500;
volatile int targetThrottle = 1500;
volatile int targetBrush    = 1500;

// ------------------------- Actual PWM Output (ramplı) ------------------------
int currentRotation = 1500;
int currentThrottle = 1500;
int currentBrush    = 1500;

// ------------------------- Failsafe Timer -----------------------------------
unsigned long lastCommandTime = 0;
const unsigned long FAILSAFE_TIMEOUT = 1000;   // ms

// ------------------------- Ramp Configuration -------------------------------
const int RAMP_STEP = 10;       // µs başına artış
const int RAMP_INTERVAL = 5;    // ms

// ------------------------- Servo Instances -----------------------------------
Servo rotationServo;
Servo throttleServo;
Servo brushServo;

// ============================================================================
//                              SERIAL PROTOCOL
//     Jetson → “T1500”, “R1600”, “B1”, “B0” şeklinde komut gönderir
// ============================================================================
void processSerial() {
    if (Serial.available()) {
        char c = Serial.read();

        if (c == 'R') {             // Rotation
            targetRotation = Serial.parseInt();
        }
        else if (c == 'T') {        // Throttle
            targetThrottle = Serial.parseInt();
        }
        else if (c == 'B') {        // Brush (0/1)
            int v = Serial.parseInt();
            targetBrush = (v == 1 ? 1500 : 1500);  // Brush sabit 1500 µs
        }

        lastCommandTime = millis(); // Failsafe reset
    }
}

// ============================================================================
//                                FAILSAFE
// ============================================================================
void checkFailsafe() {
    if (millis() - lastCommandTime > FAILSAFE_TIMEOUT) {
        targetRotation = 1500;
        targetThrottle = 1500;
        targetBrush    = 1500;
    }
}

// ============================================================================
//                                 RAMP LOGIC
// ============================================================================
void applyRamp() {
    static unsigned long lastRamp = 0;

    if (millis() - lastRamp < RAMP_INTERVAL) return;
    lastRamp = millis();

    auto ramp = [](int current, int target) {
        if (current < target) current += RAMP_STEP;
        else if (current > target) current -= RAMP_STEP;
        return current;
    };

    currentRotation = ramp(currentRotation, targetRotation);
    currentThrottle = ramp(currentThrottle, targetThrottle);
    currentBrush    = ramp(currentBrush, targetBrush);

    // Servo library handles pulse timing; we just update targets
    rotationServo.writeMicroseconds(currentRotation);
    throttleServo.writeMicroseconds(currentThrottle);
    brushServo.writeMicroseconds(currentBrush);
}

// ============================================================================
//                                    SETUP
// ============================================================================
void setup() {
    Serial.begin(115200);

    rotationServo.attach(PIN_ROTATION);
    throttleServo.attach(PIN_THROTTLE);
    brushServo.attach(PIN_BRUSH);

    lastCommandTime = millis();  // Failsafe init
}

// ============================================================================
//                                    LOOP
// ============================================================================
void loop() {
    processSerial();
    checkFailsafe();
    applyRamp();
}
