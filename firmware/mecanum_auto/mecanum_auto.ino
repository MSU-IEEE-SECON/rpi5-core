// mecanum_auto.ino — the original mecanum.ino (manual Bluetooth controller
// driving, unchanged) PLUS an AUTO mode where the Raspberry Pi drives over USB.
//
// Everything added for AUTO mode is marked  // [AUTO]
// The untouched original is in firmware/mecanum/mecanum.ino for comparison.
//
// HOW IT WORKS
//   MANUAL (default at boot): exactly like before, sticks drive the robot.
//   Press Y (triangle on a PS controller) -> AUTO: the robot follows "D vx vy wz"
//     commands from the Pi on the USB serial port.
//   Leaving AUTO, any of these:
//     - press Y again or press B
//     - move any stick (you grab control instantly)
//     - controller disconnects
//   Safety in AUTO: if the Pi's commands stop for AUTO_TIMEOUT_MS -> motors stop.
//   AUTO needs a connected controller (it's the kill switch).
//
//   The ESP32 reports its state to the Pi a few times a second:
//     S <MANUAL|AUTO> <fl> <fr> <bl> <br>
//   The Pi uses that to start/stop its autonomy when you press Y.

#include <Bluepad32.h>
#include "auto_drive.h"  // [AUTO] protocol parsing + mixing (unit-tested on a PC)

// [AUTO] forward declarations (the Arduino IDE usually adds these itself)
void setAutoMode(bool on, const char* why);
void sendStatus();
void stopMotors();

// --- MOTOR PIN DEFINITIONS (2 wires per motor) ---
// Front Left Motor
const int FL_1 = 2;
const int FL_2 = 4;

// Front Right Motor
const int FR_1 = 33;
const int FR_2 = 25;

// Back Left Motor
const int BL_1 = 16;
const int BL_2 = 17;

// Back Right Motor
const int BR_1 = 26;
const int BR_2 = 27;

ControllerPtr myControllers[BP32_MAX_CONTROLLERS];

// --- SENSITIVITY MULTIPLIERS ---
float translationSensitivity = 1.0; // Scales throttle and strafe (0.2 to 2.0)
float rotationSensitivity = 1.0;    // Scales rotation (0.2 to 2.0)
unsigned long lastDpadPress = 0;    // Debounce timer for D-pad inputs

// --- [AUTO] SETTINGS --------------------------------------------------------
const unsigned long AUTO_TIMEOUT_MS = 300;  // stop if no Pi command for this long
const int AUTO_MIN_PWM = 60;      // smallest PWM that actually turns the wheels (tune!)
const int STICK_OVERRIDE = 80;    // stick deflection (of 511) that grabs control back
const unsigned long STATUS_PERIOD_MS = 200;

// --- [AUTO] STATE -----------------------------------------------------------
bool autoMode = false;
DriveCmd autoCmd = {0, 0, 0};
unsigned long lastAutoCmdMs = 0;
bool haveAutoCmd = false;
bool lastY = false, lastB = false;
unsigned long lastYToggleMs = 0;   // Y debounce: one press = one toggle
const unsigned long Y_DEBOUNCE_MS = 500;
unsigned long lastStatusMs = 0;
Wheels lastWheels = {0, 0, 0, 0};
char rxBuf[64];
uint8_t rxLen = 0;

void onConnectedController(ControllerPtr ctl) {
    for (int i = 0; i < BP32_MAX_CONTROLLERS; i++) {
        if (myControllers[i] == nullptr) {
            myControllers[i] = ctl;
            Serial.print("Controller connected at index: ");
            Serial.println(i);
            return;
        }
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    for (int i = 0; i < BP32_MAX_CONTROLLERS; i++) {
        if (myControllers[i] == ctl) {
            myControllers[i] = nullptr;
            Serial.println("Controller disconnected!");
            setAutoMode(false, "controller disconnected");  // [AUTO]
            stopMotors();
            return;
        }
    }
}

// Helper function to drive an individual motor with a 2-wire setup (-255 to 255 speed)
void driveMotor(int pin1, int pin2, int speed) {
    speed = constrain(speed, -255, 255);
    if (speed > 0) {
        analogWrite(pin1, speed);
        analogWrite(pin2, 0);
    } else if (speed < 0) {
        analogWrite(pin1, 0);
        analogWrite(pin2, -speed);
    } else {
        analogWrite(pin1, 0);
        analogWrite(pin2, 0);
    }
}

// [AUTO] drive all four and remember what we sent (for status reports)
void driveWheels(int fl, int fr, int bl, int br) {
    driveMotor(FL_1, FL_2, fl);
    driveMotor(FR_1, FR_2, fr);
    driveMotor(BL_1, BL_2, bl);
    driveMotor(BR_1, BR_2, br);
    lastWheels = {constrain(fl, -255, 255), constrain(fr, -255, 255),
                  constrain(bl, -255, 255), constrain(br, -255, 255)};
}

void stopMotors() {
    driveWheels(0, 0, 0, 0);
}

// --- [AUTO] mode switching + Pi serial ----------------------------------------
void setAutoMode(bool on, const char* why) {
    if (on == autoMode) return;
    autoMode = on;
    haveAutoCmd = false;          // wait for a fresh command from the Pi
    autoCmd = {0, 0, 0};
    stopMotors();
    Serial.print(on ? "MODE AUTO (" : "MODE MANUAL (");
    Serial.print(why);
    Serial.println(")");
    sendStatus();                 // tell the Pi right away
}

void sendStatus() {
    Serial.print("S ");
    Serial.print(autoMode ? "AUTO" : "MANUAL");
    Serial.print(' '); Serial.print(lastWheels.fl);
    Serial.print(' '); Serial.print(lastWheels.fr);
    Serial.print(' '); Serial.print(lastWheels.bl);
    Serial.print(' '); Serial.println(lastWheels.br);
    lastStatusMs = millis();
}

// Read whatever the Pi has sent; act on complete lines. Never blocks.
void readPiSerial() {
    while (Serial.available() > 0) {
        char c = (char)Serial.read();
        if (c == '\n') {
            rxBuf[rxLen] = '\0';
            DriveCmd cmd;
            if (parseDriveLine(rxBuf, &cmd)) {
                autoCmd = cmd;
                lastAutoCmdMs = millis();
                haveAutoCmd = true;
            }
            rxLen = 0;
        } else if (c != '\r') {
            if (rxLen < sizeof(rxBuf) - 1) {
                rxBuf[rxLen++] = c;
            } else {
                rxLen = 0;  // line too long: drop it
            }
        }
    }
}

void setup() {
    Serial.begin(115200);

    // Initialize motor pins as outputs
    pinMode(FL_1, OUTPUT);
    pinMode(FL_2, OUTPUT);

    pinMode(FR_1, OUTPUT);
    pinMode(FR_2, OUTPUT);

    pinMode(BL_1, OUTPUT);
    pinMode(BL_2, OUTPUT);

    pinMode(BR_1, OUTPUT);
    pinMode(BR_2, OUTPUT);

    stopMotors();

    BP32.setup(&onConnectedController, &onDisconnectedController);
    BP32.forgetBluetoothKeys();

    Serial.println("Robot Ready. Connect your controller.");
    Serial.println("Use D-Pad Up/Down for Translation Sensitivity, Left/Right for Rotation Sensitivity.");
    Serial.println("[AUTO] Press Y to let the Pi drive. Y/B or any stick takes control back.");
}

void loop() {
    BP32.update();
    readPiSerial();  // [AUTO]

    bool controllerActive = false;

    for (int i = 0; i < BP32_MAX_CONTROLLERS; i++) {
        ControllerPtr ctl = myControllers[i];
        if (ctl && ctl->isConnected()) {
            controllerActive = true;

            // --- [AUTO] Y toggles AUTO, B always drops back to MANUAL (edge-triggered) ---
            bool y = ctl->y(), b = ctl->b();
            // Bluetooth can drop a held button for one packet, which looked like two presses
            // (AUTO then straight back to MANUAL 0.2 s later). Ignore Y re-presses within 500 ms.
            if (y && !lastY && (lastYToggleMs == 0 || millis() - lastYToggleMs > Y_DEBOUNCE_MS)) {
                lastYToggleMs = millis() | 1;  // never 0 (0 = no press yet)
                setAutoMode(!autoMode, "Y button");
            }
            if (b && !lastB) setAutoMode(false, "B button");
            lastY = y;
            lastB = b;

            // --- D-PAD SENSITIVITY ADJUSTMENT (with simple debounce) ---
            uint8_t dpad = ctl->dpad();
            if (millis() - lastDpadPress > 250) { // 250ms debounce
                if (dpad & DPAD_UP) {
                    translationSensitivity += 0.1;
                    if (translationSensitivity > 2.0) translationSensitivity = 2.0;
                    Serial.print("Translation Sensitivity: ");
                    Serial.println(translationSensitivity);
                    lastDpadPress = millis();
                } else if (dpad & DPAD_DOWN) {
                    translationSensitivity -= 0.1;
                    if (translationSensitivity < 0.2) translationSensitivity = 0.2;
                    Serial.print("Translation Sensitivity: ");
                    Serial.println(translationSensitivity);
                    lastDpadPress = millis();
                }

                if (dpad & DPAD_RIGHT) {
                    rotationSensitivity += 0.1;
                    if (rotationSensitivity > 2.0) rotationSensitivity = 2.0;
                    Serial.print("Rotation Sensitivity: ");
                    Serial.println(rotationSensitivity);
                    lastDpadPress = millis();
                } else if (dpad & DPAD_LEFT) {
                    rotationSensitivity -= 0.1;
                    if (rotationSensitivity < 0.2) rotationSensitivity = 0.2;
                    Serial.print("Rotation Sensitivity: ");
                    Serial.println(rotationSensitivity);
                    lastDpadPress = millis();
                }
            }

            // Read joystick axes (Bluepad32 axes typically range from -511 to 511)
            int axisX = ctl->axisX();   // Left stick X (Strafe)
            int axisY = ctl->axisY();   // Left stick Y (Forward/Backward)
            int axisRX = ctl->axisRX(); // Right stick X (Rotation)

            // [AUTO] touching a stick hands control straight back to the human
            if (autoMode && (abs(axisX) > STICK_OVERRIDE || abs(axisY) > STICK_OVERRIDE ||
                             abs(axisRX) > STICK_OVERRIDE)) {
                setAutoMode(false, "stick override");
            }

            if (autoMode) {
                // --- [AUTO] follow the Pi, with a watchdog ---
                if (haveAutoCmd && millis() - lastAutoCmdMs <= AUTO_TIMEOUT_MS) {
                    Wheels w = autoWheels(autoCmd, AUTO_MIN_PWM);
                    driveWheels(w.fl, w.fr, w.bl, w.br);
                } else {
                    stopMotors();  // Pi went quiet (or hasn't started yet)
                }
                break;  // only one controller drives
            }

            // Apply a small deadzone to prevent joystick drift
            if (abs(axisX) < 20) axisX = 0;
            if (abs(axisY) < 20) axisY = 0;
            if (abs(axisRX) < 20) axisRX = 0;

            // Map inputs from joystick range to motor speed range (-255 to 255)
            // Note: Invert Y axis because pushing up yields negative values on gamepads
            float strafe = map(axisX, -511, 511, -255, 255) * translationSensitivity;
            float throttle = map(axisY, -511, 511, 255, -255) * translationSensitivity;
            float rotation = map(axisRX, -511, 511, -255, 255) * rotationSensitivity;

            // Mecanum Kinematics Mixing Formula
            float fl = throttle + strafe + rotation;
            float fr = throttle - strafe - rotation;
            float bl = throttle - strafe + rotation;
            float br = throttle + strafe - rotation;

            // Drive the 4 motors
            driveWheels((int)fl, (int)fr, (int)bl, (int)br);  // [AUTO] same as before, also records values
        }
    }

    if (!controllerActive) {
        setAutoMode(false, "no controller");  // [AUTO]
        stopMotors();
    }

    if (millis() - lastStatusMs >= STATUS_PERIOD_MS) sendStatus();  // [AUTO]

    delay(15); // Small loop refresh delay
}
