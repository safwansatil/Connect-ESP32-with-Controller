//Hello
#include <Bluepad32.h>
#include <Arduino.h>
#include <BluetoothSerial.h>

// ----- Motor PWM Pins & Channels -----
const int R1PWM_pin = 19;  // Right motor forward
const int R2PWM_pin = 21;  // Right motor reverse
const int L1PWM_pin = 23;  // Left motor forward
const int L2PWM_pin = 22;  // Left motor reverse

const int R1_channel = 0;
const int R2_channel = 1;
const int L1_channel = 2;
const int L2_channel = 3;

// ----- Global Variables -----
ControllerPtr myControllers[BP32_MAX_GAMEPADS];

// currentSpeed: positive = forward, negative = reverse.
int currentSpeed = 0;          
const int maxSpeed = 255;      // Maximum PWM value
const int accel    = 30;       // Acceleration increment
const int decel    = 20;       // Deceleration step

// Steering variable from left joystick X-axis (raw value, approx. -511 to +512)
int steering = 0;

// ----- Update Motor Outputs with Throttle and Steering Mixing -----
// When throttle is nonzero, we mix in a steering offset.
// When throttle is zero, the left joystick rotates the car in place.
void updateMotorOutputs() {
  // Remove inversion so that positive joystick values mean turning right.
  float steerNorm = ((float)steering) / 512.0; 
  // Compute a turning adjustment value (up to 50% of maxSpeed)
  int turnAdjustment = (int)(0.8 * maxSpeed * fabs(steerNorm));

  if (currentSpeed != 0) {
    // --- Mixing Mode: Throttle + Steering ---
    if (currentSpeed > 0) { // Forward motion
      int leftMotor, rightMotor;
      if (steerNorm > 0) {
        // Joystick right: left motor speeds up, right motor slows down → turn right.
        // leftMotor  = currentSpeed + turnAdjustment;
        // rightMotor = currentSpeed - turnAdjustment;
        leftMotor  = currentSpeed - turnAdjustment;
        rightMotor = currentSpeed + turnAdjustment;
      } else {
        // Joystick left: left motor slows down, right motor speeds up → turn left.
        // leftMotor  = currentSpeed - turnAdjustment;
        // rightMotor = currentSpeed + turnAdjustment;
        leftMotor  = currentSpeed + turnAdjustment;
        rightMotor = currentSpeed - turnAdjustment;
      }
      leftMotor  = constrain(leftMotor, 0, maxSpeed);
      rightMotor = constrain(rightMotor, 0, maxSpeed);
      // Apply forward outputs.
      ledcWrite(L1_channel, leftMotor);
      ledcWrite(R1_channel, rightMotor);
      // Ensure reverse channels are off.
      ledcWrite(L2_channel, 0);
      ledcWrite(R2_channel, 0);
    }
    else { // Reverse motion (currentSpeed < 0)
      int absSpeed = -currentSpeed;
      int leftMotor, rightMotor;
      if (steerNorm > 0) {
        // Joystick right: for reverse, left motor gets higher PWM → turn right.
        // leftMotor  = absSpeed + turnAdjustment;
        // rightMotor = absSpeed - turnAdjustment;
        leftMotor  = absSpeed + turnAdjustment;
        rightMotor = absSpeed - turnAdjustment;
      } else {
        // Joystick left: for reverse, left motor gets lower PWM → turn left.
        // leftMotor  = absSpeed - turnAdjustment;
        // rightMotor = absSpeed + turnAdjustment;
        leftMotor  = absSpeed - turnAdjustment;
        rightMotor = absSpeed + turnAdjustment;
      }
      leftMotor  = constrain(leftMotor, 0, maxSpeed);
      rightMotor = constrain(rightMotor, 0, maxSpeed);
      // Apply reverse outputs.
      ledcWrite(L2_channel, leftMotor);
      ledcWrite(R2_channel, rightMotor);
      // Ensure forward channels are off.
      ledcWrite(L1_channel, 0);
      ledcWrite(R1_channel, 0);
    }
  }
  else {
    // --- In-Place Rotation Mode (no throttle) ---
    // Use the computed turnAdjustment as the rotation speed.
    int rotSpeed = turnAdjustment;
    if (steerNorm > 0) {
      // Joystick right: rotate right in place.
      // Left motor forward, right motor reverse.
      // ledcWrite(L1_channel, rotSpeed);
      // ledcWrite(R2_channel, rotSpeed);
      // ledcWrite(L2_channel, 0);
      // ledcWrite(R1_channel, 0);
      ledcWrite(L2_channel, rotSpeed);
      ledcWrite(R1_channel, rotSpeed);
      ledcWrite(L1_channel, 0);
      ledcWrite(R2_channel, 0);
    }
    else if (steerNorm < 0) {
      // Joystick left: rotate left in place.
      // Left motor reverse, right motor forward.
      // ledcWrite(L2_channel, rotSpeed);
      // ledcWrite(R1_channel, rotSpeed);
      // ledcWrite(L1_channel, 0);
      // ledcWrite(R2_channel, 0);
      ledcWrite(L1_channel, rotSpeed);
      ledcWrite(R2_channel, rotSpeed);
      ledcWrite(L2_channel, 0);
      ledcWrite(R1_channel, 0);
    }
    else {
      // No steering input.
      ledcWrite(L1_channel, 0);
      ledcWrite(R1_channel, 0);
      ledcWrite(L2_channel, 0);
      ledcWrite(R2_channel, 0);
    }
  }
}

// ----- Bluepad32 Controller Callbacks -----
void onConnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      myControllers[i] = ctl;
      Serial.printf("Controller connected at index %d\n", i);
      return;
    }
  }
  Serial.println("Controller connected, but no empty slot available!");
}

void onDisconnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      Serial.printf("Controller disconnected from index %d\n", i);
      myControllers[i] = nullptr;
      return;
    }
  }
  Serial.println("Controller disconnected but not found!");
}

// ----- Process Gamepad Input -----
void processGamepad(ControllerPtr ctl) {
  // Button bit masks (PS4-style)
  bool squarePressed = (ctl->buttons() & 0x0004); // Emergency brake
  bool circlePressed = (ctl->buttons() & 0x0002);   // Modifier for fixed-speed override
  bool l1Pressed     = (ctl->buttons() & 0x0010);   // Left shoulder button
  bool r1Pressed     = (ctl->buttons() & 0x0020);   // Right shoulder button
  bool l2Pressed     = (ctl->buttons() & 0x0040);   // Forward throttle trigger
  bool r2Pressed     = (ctl->buttons() & 0x0080);   // Reverse throttle trigger

  // Emergency Brake: Square stops the car immediately.
  if (squarePressed) {
    currentSpeed = 0;
    updateMotorOutputs();
    Serial.println("Emergency Brake Activated!");
    return;
  }

  // Fixed-Speed Override: Hold Circle + R1 or Circle + L1.
  if (circlePressed && r1Pressed) {
    currentSpeed = (int)(0.8 * maxSpeed);
    Serial.printf("Fixed forward speed: %d\n", currentSpeed);
  }
  else if (circlePressed && l1Pressed) {
    currentSpeed = - (int)(0.8 * maxSpeed);
    Serial.printf("Fixed reverse speed: %d\n", currentSpeed);
  }
  else {
    // Use triggers for throttle control.
    if (l2Pressed) { // Forward acceleration.
      if (currentSpeed < 0) { // If moving reverse, decelerate first.
        currentSpeed += decel;
        if (currentSpeed > 0)
          currentSpeed = 0;
      } else {
        currentSpeed += accel;
        if (currentSpeed > maxSpeed)
          currentSpeed = maxSpeed;
      }
      Serial.printf("Accelerating Forward: Speed = %d\n", currentSpeed);
    }
    else if (r2Pressed) { // Reverse acceleration.
      if (currentSpeed > 0) { // If moving forward, decelerate first.
        currentSpeed -= decel;
        if (currentSpeed < 0)
          currentSpeed = 0;
      } else {
        currentSpeed -= accel;
        if (currentSpeed < -maxSpeed)
          currentSpeed = -maxSpeed;
      }
      Serial.printf("Accelerating Reverse: Speed = %d\n", currentSpeed);
    }
    else {
      // No trigger pressed: gradually coast toward zero.
      if (currentSpeed > 0) {
        currentSpeed -= decel;
        if (currentSpeed < 0)
          currentSpeed = 0;
      }
      else if (currentSpeed < 0) {
        currentSpeed += decel;
        if (currentSpeed > 0)
          currentSpeed = 0;
      }
    }
  }

  // Process left joystick for steering.
  int rawAxisX = ctl->axisX();
  if (abs(rawAxisX) < 25) { // Deadzone
    rawAxisX = 0;
  }
  steering = rawAxisX;
  Serial.printf("Steering (raw): %d\n", steering);

  updateMotorOutputs();
}

// ----- Process All Connected Controllers -----
void processControllers() {
  for (auto ctl : myControllers) {
    if (ctl && ctl->isConnected() && ctl->hasData()) {
      processGamepad(ctl);
    }
  }
}

// ----- Arduino Setup -----
void setup() {
  Serial.begin(115200);

  // Setup LEDC channels for PWM (frequency 5000 Hz, 8-bit resolution)
  ledcSetup(R1_channel, 5000, 8);
  ledcSetup(R2_channel, 5000, 8);
  ledcSetup(L1_channel, 5000, 8);
  ledcSetup(L2_channel, 5000, 8);

  // Attach PWM channels to the respective pins.
  ledcAttachPin(R1PWM_pin, R1_channel);
  ledcAttachPin(R2PWM_pin, R2_channel);
  ledcAttachPin(L1PWM_pin, L1_channel);
  ledcAttachPin(L2PWM_pin, L2_channel);

  Serial.printf("Bluepad32 Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t* addr = BP32.localBdAddress();
  Serial.printf("Bluetooth Address: %02X:%02X:%02X:%02X:%02X:%02X\n",
                addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

  // Setup Bluepad32 with controller connection callbacks.
  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.forgetBluetoothKeys();
  BP32.enableVirtualDevice(false);
}

// ----- Main Loop -----
void loop() {
  // Update Bluepad32 and process new controller data.
  bool dataUpdated = BP32.update();
  if (dataUpdated)
    processControllers();

  // Reduced delay for lower latency.
  delay(10);
}
