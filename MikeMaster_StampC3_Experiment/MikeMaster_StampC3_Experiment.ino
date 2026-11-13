/*
  MikeMaster_StampC3: a simple master for the robot running MikeFirmware.

  Upload THIS sketch to the M5StampC3. MikeFirmware goes on the robot's 32U4.
  Open Serial Monitor at 115200 baud to see the results.

  Choose demonstrations by commenting/uncommenting calls in setup(), below.
  The selected demos run ONCE, in order. Press Reset to run them again.
  The line-following demo is currently enabled.

  This sketch deliberately uses ordinary functions, global variables and
  waiting loops. The 32U4 still runs its own controllers while this master waits.
  The master does not need to send a command every 10 ms to run the speed PIDs.
*/

#include <Wire.h>
#include "i2c_datatypes.h" // Same packet definitions as MikeFirmware.

const int SDA_PIN = 8;       // Defaults for the original M5StampC3 board.
const int SCL_PIN = 9;       // Change these if your robot connection uses other pins.
const int STOP_BUTTON_PIN = 3; // StampC3 button: LOW while pressed.
const uint32_t ROBOT_STARTUP_TIMEOUT_MS = 15000;
const float HALF_PI_RAD = 1.57079632679f;

// Each get...() function fills one of these global variables. You can then
// print or use its fields, for example pose.x or wheelSpeed.counts_per_second[0].
RobotSurface_t surface;
RobotEncoders_t encoders;
RobotPose_t pose;
RobotWheelSpeed_t wheelSpeed;
RobotGotoStatus_t gotoStatus;
RobotPIDGains_t leftPIDGains;
RobotPIDGains_t rightPIDGains;


// Class to implement hysteresis so that any noise on
// the sensors can be ignored. Low and high
// thresholds need to be set with appropraite values.
class Hysteresis_c {
  public:
    Hysteresis_c () {}

    uint16_t lo_threshold;
    uint16_t hi_threshold;
    bool state;

    void updateState( uint16_t reading ) {

      if ( state == HIGH ) { // high? looking for low
        if ( reading < lo_threshold ) state = LOW;
      } else {
        if ( reading > hi_threshold ) state = HIGH;
      }
    }

    bool getState() {
      return state;
    }
};

// For deciding surface as black or white
Hysteresis_c left_sensor;
Hysteresis_c right_sensor;

// For deciding if an IR signal is received from the other robot
Hysteresis_c ir_signal;


void setup() {
  Serial.begin(115200);
  delay(1000); // Give the Serial Monitor a moment to connect.

  // StampC3 user input button
  pinMode(STOP_BUTTON_PIN, INPUT_PULLUP);

  // StampC3 RGB LED
  setupLED();

  // IR signaling setup
  pinMode(6, OUTPUT);   // Both pins 6,7 need to be set to output and high
  pinMode(7, OUTPUT);   // to enable IR transmission.
  digitalWrite(6, LOW); // Start off (on white surface, no in target area)
  digitalWrite(7, HIGH); // pins 6 and 7 are logically AND'd, so leave one high

  // Pins 4, 0 1 are the IR signal sensors, which are
  // read with analogRead( ) and ACTIVE_LOW (low value means a signal is present)
  pinMode(4, INPUT);  // back-right
  pinMode(0, INPUT);  // back-left
  pinMode(1, INPUT);  // forward


  // Connect SDA to the robot's SDA, SCL to SCL, and share ground through your
  // robot interface. Wire.begin() has no slave address here: this is the master.
  Wire.setTimeOut(50); // Bound each failed I2C transaction to about 50 ms.
  if (!Wire.begin(SDA_PIN, SCL_PIN, 400000)) {
    stopAndHalt("Could not start the StampC3 I2C bus.");
  }


  waitForRobot(); // The robot may still be starting when both boards power up.
  stopRobot(); // Also stops an earlier command if only the StampC3 was reset.

  waitWithStop(200);

  setLED( 255, 255, 255, 100);

  playTone(880, 200); // Frequency in Hz, duration in milliseconds.

  left_sensor.lo_threshold = 1500;
  left_sensor.hi_threshold = 1800;
  left_sensor.state = HIGH;

  right_sensor.lo_threshold = 1500;
  right_sensor.hi_threshold = 1800;
  right_sensor.state = HIGH;

  ir_signal.lo_threshold = 1000;
  ir_signal.hi_threshold = 3000; // Max is 4095
  ir_signal.state = HIGH;


  // Was checking for sensible surface sensor reading values
  // for lo/hi hysteresis threshold values
  //  while(true) {
  //    getSurface();
  //    Serial.print( surface.reading[0] );
  //    Serial.print(",");
  //    Serial.print( surface.reading[4] );
  //    Serial.print("\n");
  //     delay(50);
  //  }

  //  // Was checking IR signaling
  //  while( true ) {
  //     Serial.print( analogRead( 0 ) );
  //     Serial.print(",");
  //     Serial.print( analogRead( 1 ) );
  //     Serial.print(",");
  //     Serial.print( analogRead( 4 ) );
  //     Serial.print("\n");
  //     delay(50);
  //  }

}




void loop() {
  runExperiment();

}



void runExperiment() {

  // Update sensor readings and historesis state
  getSurface();
  left_sensor.updateState( surface.reading[0] );
  right_sensor.updateState( surface.reading[4] );

  // Check whether the surface is black or white.
  // Use LED to indicate which surface we are on
  // If on the surface, send a signal to other robot
  bool surface = false;
  if ( left_sensor.getState() || left_sensor.getState() ) {
    setLED( 0, 255, 0, 100 ); // led green
    surface = true;

    digitalWrite( 6, HIGH ); // enable IR signal

  } else { // Either low
    setLED( 255, 0, 0, 100 ); // led red
    surface = false;

    digitalWrite( 6, LOW ); // disable IR signal

  }

  // Decide current state of received signal
  bool signal = LOW;
  ir_signal.updateState( analogRead( 1 ) ); // get forward sensor

  if ( ir_signal.getState() )  { // signal in high state (no signal)
    signal = false;
  } else {
    signal = true;
  }

  // Decision logic table
  if ( signal && surface ) {          // On surface at same time as other robot (reverse)
    setWheelSpeed( -300, -300 );

  } else if ( !signal && surface ) {  // On surface, but other robot isn't (stop)
    setWheelSpeed( 0, 0 );

  } else if ( !signal && !surface ) { // Not on surface, neither is other robot (forwards line following)
    lineFollowing();

  } else if ( signal && !surface ) {  // Other robot is on surface, this one is't (forwards, line following)
    lineFollowing();

  }

  // Update period
  delay(50);

}

void lineFollowing() {

  float fwd_vel   = 300.0;
  float turn_vel  = 0.0;


  // Get line centre
  getSurface();

  // Difference between left/right middle sensors
  int16_t diff = (int16_t)surface.reading[1] - (int16_t)surface.reading[3];
  turn_vel = (float)diff;
  turn_vel /= 1500;     // This might need tweaking
  turn_vel *= 200;      // scaling to an appropriate velocity

  setWheelSpeed( fwd_vel - turn_vel, fwd_vel + turn_vel );

}



// --------------------------------------------------------------------------
// Demonstrations. Start reading here before looking at the I2C details below.
// --------------------------------------------------------------------------

void demoSensors() {
  Serial.println("Sensor demo: 5 seconds, with the motors stopped.");
  stopRobot();
  for (int sample = 0; sample < 50; sample++) {
    getSurface();
    getEncoders();
    getPose();
    Serial.print("Surface start us: ");
    Serial.print(surface.timestamp_us);
    Serial.print("  DN1..DN5 us:");
    for (int sensor = 0; sensor < 5; sensor++) {
      Serial.print(' ');
      Serial.print(surface.reading[sensor]);
    }
    Serial.print("  Encoders L/R: ");
    Serial.print(encoders.count[0]);
    Serial.print(' ');
    Serial.println(encoders.count[1]);
    printPose();
    waitWithStop(100);
  }
  // Surface readings are raw RC times, not calibrated reflectance percentages.
  // Encoder counts are cumulative: setPose() does not reset those counts.
}

void demoBuzzer() {
  Serial.println("Buzzer demo: two short tones.");
  playTone(880, 200); // Frequency in Hz, duration in milliseconds.
  waitWithStop(300);
  playTone(660, 200);
  waitWithStop(300);
  playTone(0, 0);    // Frequency zero stops the buzzer.
}

void demoMotorPWM() {
  Serial.println("PWM demo: forwards for 2 seconds, then backwards for 2 seconds.");
  // PWM is NOT a measured speed. The allowed range is -200 to +200.
  // Left is the first argument; right is the second. Positive is forwards.
  setMotorPWM(40, 40);
  waitWithStop(2000);
  stopRobot();
  waitWithStop(500);
  setMotorPWM(-40, -40);
  waitWithStop(2000);
  stopRobot();
  waitWithStop(500);
}

void demoWheelSpeed() {
  Serial.println("Speed demo: demand 300 counts/second on each wheel for 5 seconds.");
  for (int sample = 0; sample < 50; sample++) {
    // The robot now uses its speed PIDs to choose the motor PWM.
    // These are encoder counts/second, NOT PWM and NOT millimetres/second.
    // Repeating the same demand refreshes the 15-second command timeout.
    setWheelSpeed(300.0f, 300.0f);
    getWheelSpeed();
    printWheelSpeed();
    waitWithStop(100);
  }
  stopRobot(); // A zero PWM command cancels speed control and stops both motors.
  waitWithStop(500);
}

void demoHeadingTurn() {
  Serial.println("Heading demo: turn towards +90 degrees with zero forward velocity.");
  stopRobot();
  waitWithStop(500);
  setPose(0.0f, 0.0f, 0.0f); // Call the present position/heading the origin.
  waitWithStop(50);          // Let the 32U4 apply that requested pose.
  for (int sample = 0; sample < 80; sample++) {
    setHeading(HALF_PI_RAD, 0.0f);
    getPose();
    printPose();
    waitWithStop(100);
  }
  // This demo observes the heading for 8 seconds, then stops. It does not claim
  // the target was reached. The gains need tuning on your physical robot.
  stopRobot();
  waitWithStop(500);
}

void demoHeadingDrive() {
  Serial.println("Heading/drive demo: steer towards +90 degrees while moving at 50 mm/s.");
  stopRobot();
  waitWithStop(500);
  setPose(0.0f, 0.0f, 0.0f);
  waitWithStop(50);
  for (int sample = 0; sample < 50; sample++) {
    // Heading is absolute in the odometry frame. Unlike setWheelSpeed(), the
    // second argument here is forward body velocity in millimetres/second.
    setHeading(HALF_PI_RAD, 50.0f);
    getPose();
    printPose();
    waitWithStop(100);
  }
  stopRobot();
  waitWithStop(500);
}

void demoGoto() {
  Serial.println("Goto demo: x=150 mm, y=100 mm, final heading=+90 degrees.");
  stopRobot();
  waitWithStop(500);
  setPose(0.0f, 0.0f, 0.0f);
  waitWithStop(50);
  goToPose(150.0f, 100.0f, HALF_PI_RAD);

  // Send the destination ONCE, then observe its status. Status reads do not
  // refresh the command timeout, so an unfinished move times out after 15 s.
  // This makes the demo bounded without hiding the firmware's timeout behaviour.
  // Do not blindly put goToPose() in this loop: sending a target after completion
  // starts a new move. Longer tasks need a deliberate refresh policy.
  unsigned long started = millis();
  while (millis() - started < 20000) {
    getGotoStatus();
    Serial.print("Goto request ");
    Serial.print(gotoStatus.request_id);
    Serial.print("  state=");
    Serial.print(gotoStatus.state);
    Serial.print("  phase=");
    Serial.print(gotoStatus.phase);
    Serial.print("  elapsed ms=");
    Serial.print(gotoStatus.elapsed_ms);
    Serial.print("  distance error mm=");
    Serial.print(gotoStatus.distance_error_mm, 2);
    Serial.print("  final heading error rad=");
    Serial.println(gotoStatus.heading_error_rad, 3);

    // State: 0 idle, 1 running, 2 complete, 3 cancelled, 4 timeout, 5 error.
    // Phase: 0 pending, 1 align, 2 drive, 3 final heading, 4 settling.
    // Errors can be NaN just after acceptance, before the first control update.
    if (gotoStatus.state == GOTO_STATUS_COMPLETE) {
      Serial.println("Goto completed according to the robot's odometry.");
      stopRobot();
      return;
    }
    if (gotoStatus.state != GOTO_STATUS_RUNNING) {
      stopAndHalt("Goto ended without completion; see the state printed above.");
    }
    waitWithStop(100);
  }
  stopAndHalt("Master stopped waiting for goto status after 20 seconds.");
}

// --------------------------------------------------------------------------
// Small robot functions. Each command fills a packet and sends its bytes.
// The struct fields and register numbers are defined in i2c_datatypes.h.
// --------------------------------------------------------------------------

void setMotorPWM(int16_t left, int16_t right) {
  RobotMotors_t command = {{left, right}};
  sendCommand(REGISTER_SET_MOTORS, (const uint8_t *)&command, sizeof(command));
}

void stopRobot() {
  setMotorPWM(0, 0); // Cancels speed, heading or goto control as well as stopping PWM.
}

void setWheelSpeed(float left, float right) {
  RobotWheelSpeedDemand_t command = {{left, right}};
  sendCommand(REGISTER_SET_WHEEL_SPEED, (const uint8_t *)&command, sizeof(command));
}

void setLeftPIDGains(float kp, float ki, float kd) {
  RobotPIDGains_t command = {kp, ki, kd};
  sendCommand(REGISTER_SET_LEFT_PID_GAINS, (const uint8_t *)&command, sizeof(command));
}

void setRightPIDGains(float kp, float ki, float kd) {
  RobotPIDGains_t command = {kp, ki, kd};
  sendCommand(REGISTER_SET_RIGHT_PID_GAINS, (const uint8_t *)&command, sizeof(command));
}

void getLeftPIDGains() {
  readData(REGISTER_GET_LEFT_PID_GAINS, (uint8_t *)&leftPIDGains, sizeof(leftPIDGains));
}

void getRightPIDGains() {
  readData(REGISTER_GET_RIGHT_PID_GAINS, (uint8_t *)&rightPIDGains, sizeof(rightPIDGains));
}

void demoPIDGains() {
  stopRobot();
  // Edit these numbers to try other gains. Changes last until the robot resets.
  // Each valid write resets just that wheel's PID history at its next update.
  setLeftPIDGains(0.10f, 2.5f, 0.0f);
  setRightPIDGains(0.10f, 2.5f, 0.0f);
  // An I2C ACK alone does not prove the gains were valid: read them back.
  getLeftPIDGains();
  getRightPIDGains();
  Serial.print("Left Kp, Ki, Kd: ");
  Serial.print(leftPIDGains.kp, 4); Serial.print(", ");
  Serial.print(leftPIDGains.ki, 4); Serial.print(", ");
  Serial.println(leftPIDGains.kd, 6);
  Serial.print("Right Kp, Ki, Kd: ");
  Serial.print(rightPIDGains.kp, 4); Serial.print(", ");
  Serial.print(rightPIDGains.ki, 4); Serial.print(", ");
  Serial.println(rightPIDGains.kd, 6);
}

void setHeading(float heading_rad, float forward_mm_s) {
  RobotHeadingDemand_t command = {heading_rad, forward_mm_s};
  sendCommand(REGISTER_SET_HEADING, (const uint8_t *)&command, sizeof(command));
}

void goToPose(float x_mm, float y_mm, float theta_rad) {
  RobotGotoCommand_t command = {x_mm, y_mm, theta_rad};
  sendCommand(REGISTER_SET_GOTO, (const uint8_t *)&command, sizeof(command));
}

void setPose(float x_mm, float y_mm, float theta_rad) {
  // Changes the estimated pose; it does NOT move the robot to these coordinates.
  // It cancels a running goto because the coordinate frame is being changed.
  RobotPose_t command = {x_mm, y_mm, theta_rad};
  sendCommand(REGISTER_SET_POSE, (const uint8_t *)&command, sizeof(command));
}

void playTone(uint16_t frequency_hz, uint16_t duration_ms) {
  RobotBuzzer_t command = {frequency_hz, duration_ms};
  sendCommand(REGISTER_SET_BUZZER, (const uint8_t *)&command, sizeof(command));
}

void getSurface() {
  readData(REGISTER_GET_SURFACE, (uint8_t *)&surface, sizeof(surface));
}

void getEncoders() {
  readData(REGISTER_GET_ENCODERS, (uint8_t *)&encoders, sizeof(encoders));
}

void getPose() {
  readData(REGISTER_GET_POSE, (uint8_t *)&pose, sizeof(pose));
}

void getWheelSpeed() {
  readData(REGISTER_GET_WHEEL_SPEED, (uint8_t *)&wheelSpeed, sizeof(wheelSpeed));
}

void getGotoStatus() {
  readData(REGISTER_GET_GOTO_STATUS, (uint8_t *)&gotoStatus, sizeof(gotoStatus));
}

void printPose() {
  Serial.print("Pose: x mm="); Serial.print(pose.x, 2);
  Serial.print("  y mm="); Serial.print(pose.y, 2);
  Serial.print("  theta rad="); Serial.println(pose.theta, 3);
}

void printWheelSpeed() {
  if (wheelSpeed.interval_us == 0) {
    Serial.println("Waiting for the first wheel-speed measurement.");
    return;
  }
  Serial.print("Measured counts/s L/R: ");
  Serial.print(wheelSpeed.counts_per_second[0], 1);
  Serial.print(' ');
  Serial.print(wheelSpeed.counts_per_second[1], 1);
  Serial.print("  sample end us="); Serial.print(wheelSpeed.timestamp_us);
  Serial.print("  interval us="); Serial.println(wheelSpeed.interval_us);
}

// --------------------------------------------------------------------------
// Startup readiness and the I2C exchange.
// --------------------------------------------------------------------------

void waitForRobot() {
  setLED(255, 255, 0, 100); // Yellow: waiting for the robot, without needing USB.
  const uint32_t started = millis();
  while (uint32_t(millis() - started) < ROBOT_STARTUP_TIMEOUT_MS) {
    // An address-only probe asks whether the slave is listening. It sends no
    // register or command, so it cannot start the motors or renew their timeout.
    // Do not use selectRegister() here: it deliberately halts on the first error.
    Wire.beginTransmission(ROBOT_I2C_ADDR);
    if (Wire.endTransmission() == 0) return;
    waitWithStop(50); // Retry gently; the button and serial stop still work.
  }
  stopAndHalt("Robot did not respond during startup. Check robot power and I2C wiring.");
}

void selectRegister(uint8_t reg) {
  Wire.beginTransmission(ROBOT_I2C_ADDR); // Address 0x08 is the robot.
  Wire.write(reg);                      // One byte selects the operation.
  if (Wire.endTransmission() != 0) {
    stopAndHalt("I2C register selection failed. Check robot power and connection.");
  }
}

void sendCommand(uint8_t reg, const uint8_t *data, size_t length) {
  selectRegister(reg);
  // MikeFirmware expects TWO separate writes: register first, payload second.
  // Combining the register and payload into one write will not work.
  Wire.beginTransmission(ROBOT_I2C_ADDR);
  size_t written = Wire.write(data, length);
  uint8_t error = Wire.endTransmission();
  if (written != length || error != 0) {
    stopAndHalt("I2C command write failed.");
  }
  // An acknowledgement means the transaction arrived. It does not prove a
  // physical movement happened or that an out-of-range command was accepted.
}

void readData(uint8_t reg, uint8_t *data, size_t length) {
  selectRegister(reg);
  size_t received = Wire.requestFrom((uint8_t)ROBOT_I2C_ADDR, length);
  if (received != length) {
    while (Wire.available()) Wire.read(); // Discard an incomplete reply.
    stopAndHalt("I2C reply had the wrong length.");
  }
  for (size_t i = 0; i < length; i++) {
    int value = Wire.read();
    if (value < 0) stopAndHalt("I2C reply ended unexpectedly.");
    data[i] = (uint8_t)value;
  }
  // The casts at the call sites mean: treat this struct as a sequence of bytes.
  // & supplies its address; sizeof supplies its byte count. Both processors
  // use the same packed layout, little-endian integers and 32-bit floats.
}

// --------------------------------------------------------------------------
// Stop handling. Keeping waits in short pieces lets the button/Serial work.
// --------------------------------------------------------------------------

void waitWithStop(unsigned long duration_ms) {
  const uint32_t started = millis();
  while (uint32_t(millis() - started) < duration_ms) {
    if (digitalRead(STOP_BUTTON_PIN) == LOW) stopAndHalt("Button pressed.");
    while (Serial.available()) {
      char letter = Serial.read();
      if (letter == 's' || letter == 'S') stopAndHalt("Serial stop requested.");
    }
    delay(10);
  }
}

void stopAndHalt(const char *reason) {
  setLED(255, 0, 0, 100); // Red: this run has stopped; press Reset to try again.
  Serial.println(reason);
  // Try a raw zero-PWM command here. Do not call sendCommand(): if the bus has
  // failed, calling its error handler from itself would repeat forever.
  Wire.beginTransmission(ROBOT_I2C_ADDR);
  Wire.write((uint8_t)REGISTER_SET_MOTORS);
  uint8_t error = Wire.endTransmission();
  if (error == 0) {
    const uint8_t zeros[4] = {0, 0, 0, 0}; // Two int16_t PWM values, both zero.
    Wire.beginTransmission(ROBOT_I2C_ADDR);
    Wire.write(zeros, sizeof(zeros));
    error = Wire.endTransmission();
  }
  if (error == 0) Serial.println("Stop sent. Press Reset to run again.");
  else Serial.println("Stop could not be confirmed; the robot's command timeout remains active.");
  while (true) delay(1000); // End this run, including any later demos in setup().
}

// --------------------------------------------------------------------------
// StampC3 LED helper functions
// --------------------------------------------------------------------------
uint8_t LED_rOffset;
uint8_t LED_gOffset;
uint8_t LED_bOffset;
rmt_data_t led_data[24];

bool setupLED() {
  LED_rOffset = (0x12 >> 4) & 0x03;
  LED_gOffset = (0x12 >> 2) & 0x03;
  LED_bOffset = 0x12 & 0x03;
  rmt_reserve_memsize_t rmt_mem = RMT_MEM_NUM_BLOCKS_1;
  if (rmtInit(2, RMT_TX_MODE, rmt_mem, 10000000) == false) {
    return false;
  }

  for (int i = 0; i < 24; i++) {
    led_data[i].level0 = 1;
    led_data[i].duration0 = 4;
    led_data[i].level1 = 0;
    led_data[i].duration1 = 8;
  }

  setLED(255, 128, 178, 20);
  return true;
}

bool updateLED() {
  // A status indicator must not hold up startup or a motor-stop command.
  // rmtWrite returns false on a timeout; the rest of the sketch can continue.
  return rmtWrite(2, led_data, 24, 20);
}

void setLED(uint8_t r, uint8_t g, uint8_t b, uint8_t brightness) {
  r = (uint8_t)constrain(r, 0, 255);
  g = (uint8_t)constrain(g, 0, 255);
  b = (uint8_t)constrain(b, 0, 255);
  brightness = constrain(brightness, 0, 255);

  uint8_t p[3];
  p[LED_rOffset] = r * brightness / 255;
  p[LED_gOffset] = g * brightness / 255;
  p[LED_bOffset] = b * brightness / 255;

  uint32_t color = p[0] << 16 | p[1] << 8 | p[2];
  for (int bit = 0; bit < 24; bit++) {
    if (color & (1 << (23 - bit))) {
      led_data[bit].level0 = 1;
      led_data[bit].duration0 = 8;
      led_data[bit].level1 = 0;
      led_data[bit].duration1 = 4;
    } else {
      led_data[bit].level0 = 1;
      led_data[bit].duration0 = 4;
      led_data[bit].level1 = 0;
      led_data[bit].duration1 = 8;
    }
  }
  updateLED();
}
