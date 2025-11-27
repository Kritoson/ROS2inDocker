// ======================================================
//  PWM + Periferik Kontrol – Acceleration Ramp (2 seconds)
// ======================================================

const int PIN_THROTTLE = 5;
const int PIN_ROTATION = 3;
const int PIN_BRUSH     = 6;

const int PIN_VACUUM_RELAY     = 7;
const int PIN_HYDROPHORE_RELAY = 8;

// Hedef PWM değerleri (Jetson'dan gelen)
volatile int targetThrottlePWM = 1500;
volatile int targetRotationPWM = 1500;
volatile int targetBrushPWM    = 1000;

// Ramp uygulanmış gerçek PWM’ler
int currentThrottlePWM = 1500;
int currentRotationPWM = 1500;
int currentBrushPWM    = 1000;

bool brushState = false;
bool vacuumState = false;
bool hydroState  = false;

// Ramp parametreleri
const unsigned long RAMP_TIME_MS = 2000; // 2 seconds
const unsigned long PWM_FRAME_US = 15000; // 15ms period
const int RAMP_STEPS = RAMP_TIME_MS / (PWM_FRAME_US / 1000); // ≈133

unsigned long lastPulseTime = 0;

void setup() {
    Serial.begin(115200);

    pinMode(PIN_THROTTLE, OUTPUT);
    pinMode(PIN_ROTATION, OUTPUT);
    pinMode(PIN_BRUSH, OUTPUT);

    pinMode(PIN_VACUUM_RELAY, OUTPUT);
    pinMode(PIN_HYDROPHORE_RELAY, OUTPUT);

    digitalWrite(PIN_VACUUM_RELAY, HIGH);
    digitalWrite(PIN_HYDROPHORE_RELAY, HIGH);

    Serial.println("MSG: Arduino Motor Controller with 2s Ramp Ready.");
}

// ------------------------------------------------------
//      TELEMETRY
// ------------------------------------------------------
void sendStatus(const char *msg)
{
    Serial.print("STATUS ");
    Serial.println(msg);
}


// ------------------------------------------------------
//      SERIAL PARSER
// ------------------------------------------------------
void parseCommand(char *line)
{
    char cmd = line[0];
    int val = atoi(line + 1);

    switch(cmd)
    {
        case 'T':
            targetThrottlePWM = constrain(val, 1000, 2000);
            sendStatus("OK Throttle Target");
            break;

        case 'R':
            targetRotationPWM = constrain(val, 1000, 2000);
            sendStatus("OK Rotation Target");
            break;

        case 'B':
            if (val == 2) brushState = !brushState;
            else brushState = (val == 1);

            targetBrushPWM = brushState ? 1700 : 1000;
            sendStatus("OK Brush");
            break;

        case 'V':
            if (val == 2) vacuumState = !vacuumState;
            else vacuumState = (val == 1);

            digitalWrite(PIN_VACUUM_RELAY, vacuumState ? LOW : HIGH);
            sendStatus("OK Vacuum");
            break;

        case 'H':
            if (val == 2) hydroState = !hydroState;
            else hydroState = (val == 1);

            digitalWrite(PIN_HYDROPHORE_RELAY, hydroState ? LOW : HIGH);
            sendStatus("OK Hydrophore");
            break;

        default:
            Serial.print("ERR Unknown: ");
            Serial.println(line);
            break;
    }
}

void readSerial()
{
    static char buf[32];
    static uint8_t idx = 0;

    while (Serial.available())
    {
        char c = Serial.read();
        if (c == '\n') {
            buf[idx] = 0;
            parseCommand(buf);
            idx = 0;
        }
        else if (idx < sizeof(buf) - 1) {
            buf[idx++] = c;
        }
    }
}


// ------------------------------------------------------
//      ACCELERATION RAMP (2 seconds)
// ------------------------------------------------------
int ramp_step(int current, int target)
{
    if (current == target) return current;

    int diff = target - current;
    int step = diff / RAMP_STEPS;

    if (step == 0)
        step = (diff > 0 ? 1 : -1); // minimum 1 birim hareket

    return current + step;
}


// ------------------------------------------------------
//      PWM SIGNAL GENERATION
// ------------------------------------------------------
void generatePWM(int pin, int pwm)
{
    digitalWrite(pin, HIGH);
    delayMicroseconds(pwm);
    digitalWrite(pin, LOW);
}


// ------------------------------------------------------
//      MAIN LOOP
// ------------------------------------------------------
void loop()
{
    readSerial();

    // PWM frame time
    unsigned long now = micros();
    if (now - lastPulseTime >= PWM_FRAME_US)
    {
        // ---- Apply ramping ----
        currentThrottlePWM = ramp_step(currentThrottlePWM, targetThrottlePWM);
        currentRotationPWM = ramp_step(currentRotationPWM, targetRotationPWM);
        currentBrushPWM    = ramp_step(currentBrushPWM,    targetBrushPWM);

        // ---- Generate pulses ----
        generatePWM(PIN_THROTTLE, currentThrottlePWM);
        generatePWM(PIN_ROTATION, currentRotationPWM);
        generatePWM(PIN_BRUSH,    currentBrushPWM);

        lastPulseTime = now;
    }
}
