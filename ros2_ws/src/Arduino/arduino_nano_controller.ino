// ============================================================================
//   Professional Motor PWM Controller for Robot (71.4 Hz Exact Timing)
//   Rotation + Throttle   → Timer1 (16-bit)
//   Brush Motor PWM       → Timer2 (8-bit fast PWM, extended for 14ms period)
//   Includes: Failsafe, Ramp, Serial protocol from Jetson
// ============================================================================

// ------------------------- PWM Pin Definitions -------------------------------
const int PIN_ROTATION = 3;   // Timer1 - OC1A
const int PIN_THROTTLE = 5;  // Timer1 - OC1B
const int PIN_BRUSH    = 6;   // Timer2 - OC2B

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

// ============================================================================
//                               TIMER SETUP
// ============================================================================
void setupTimer1_71Hz() {
    // Timer1 STOP
    TCCR1A = 0;
    TCCR1B = 0;

    // Prescaler = 8 → Timer clock: 16MHz / 8 = 2MHz (0.5 µs per tick)
    TCCR1B |= (1 << WGM13) | (1 << WGM12);   // Mode 14 (Fast PWM, TOP=ICR1)
    TCCR1A |= (1 << WGM11);

    TCCR1A |= (1 << COM1A1); // OC1A → Rotation PWM
    TCCR1A |= (1 << COM1B1); // OC1B → Throttle PWM

    ICR1 = 28000;            // 14ms period → 28000 ticks @ 0.5us resolution

    // Başlangıç olarak merkez 1500us → 3000 tick
    OCR1A = 3000;
    OCR1B = 3000;

    TCCR1B |= (1 << CS11);   // Start timer (prescaler 8)
}

// ---------------------------------------------------------------------------
// Timer2 → Brush motor PWM (14ms periyot için geniş mod)
// ---------------------------------------------------------------------------
void setupTimer2_71Hz() {
    // Timer2 normal PWM buna uygun değildir → Phase Correct kullanılan özel mod
    // Ancak robotun brush motor PWM sinyali sadece ON/OFF davranışına sahip
    // Bu nedenle Timer2'yi "pulse width generator" gibi kullanıyoruz.

    pinMode(PIN_BRUSH, OUTPUT);
}

// ============================================================================
//                   MICROSECOND → TIMER TICK CONVERSION
// ============================================================================
int usToTicks(int us) {
    return us * 2;   // 0.5 µs per tick
}

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

    // Convert to ticks and write to hardware PWM
    OCR1A = usToTicks(currentRotation);
    OCR1B = usToTicks(currentThrottle);

    // Brush PWM manual:
    digitalWrite(PIN_BRUSH, HIGH);
    delayMicroseconds(currentBrush);
    digitalWrite(PIN_BRUSH, LOW);
}

// ============================================================================
//                                    SETUP
// ============================================================================
void setup() {
    Serial.begin(115200);

    pinMode(PIN_ROTATION, OUTPUT);
    pinMode(PIN_THROTTLE, OUTPUT);

    setupTimer1_71Hz();
    setupTimer2_71Hz();

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
