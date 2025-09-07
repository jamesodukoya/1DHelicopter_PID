#define MPU6050_ADDR 0x68  // I2C address of MPU6050

// === Global variables ===
volatile bool imu_ready = false;      // Set every 1 ms by Timer3 ISR
volatile unsigned long tick_count = 0; // Global millisecond counter (used for debug and timing)

// Complementary filter and angle estimation
float theta_complementary = 0.0;  // Filtered angle in radians
float actual_angle = 0.0;         // Final smoothed angle in degrees
float alpha = 0.98;               // Weight for complementary filter
float dt = 0.001;                 // 1 ms sampling interval

// Raw IMU readings
int16_t accel[3];  // ax, ay, az
int16_t gyro[3];   // gx, gy, gz

// PID control variables
float target_angle = 0.0;           // Desired beam angle (degrees)
float Kp = 3.0, Ki = 0.0, Kd = 0.0; // PID gains
float error = 0.0, prev_error = 0.0;
float integral = 0.0, derivative = 0.0;
float pid_output = 0.0;

// Signal filtering (LPF) variables
float filtered_ay = 0.0, filtered_az = 0.0;
float filtered_gyro_x = 0.0;
float smoothed_angle = 0.0;
float accel_alpha = 0.9, gyro_alpha = 0.9, angle_alpha = 0.9;  // Filter constants

// Joystick ADC and control variables
volatile int ADC1_value = 0, ADC9_value = 0;  // Raw ADC values
volatile bool adc1_done = false, adc9_done = false;
volatile bool reading_adc1 = true;
volatile bool joystick_ready = false;
const int CENTER = 512;              // Center of joystick range (10-bit)
const int JOY_DEADZONE = 50;         // Threshold to ignore small movements
const int JOY_DEBOUNCE_MS = 300;     // Min interval between angle updates
volatile unsigned long joy_timer = 0;
unsigned long last_joy_move = 0;

// === I2C utility functions for MPU6050 ===
void i2c_init() {
  TWSR = 0x00;
  TWBR = 72;  // SCL frequency ~100kHz at 16 MHz
}

void i2c_start() {
  TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
  while (!(TWCR & (1 << TWINT)));
}

void i2c_stop() {
  TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
}

void i2c_write(uint8_t data) {
  TWDR = data;
  TWCR = (1 << TWINT) | (1 << TWEN);
  while (!(TWCR & (1 << TWINT)));
}

uint8_t i2c_read_ack() {
  TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
  while (!(TWCR & (1 << TWINT)));
  return TWDR;
}

uint8_t i2c_read_nack() {
  TWCR = (1 << TWINT) | (1 << TWEN);
  while (!(TWCR & (1 << TWINT)));
  return TWDR;
}

// === Initialize MPU6050 (wake from sleep) ===
void mpu6050_init() {
  i2c_start();
  i2c_write(MPU6050_ADDR << 1);
  i2c_write(0x6B);   // PWR_MGMT_1 register
  i2c_write(0x00);   // Clear sleep bit
  i2c_stop();
}

// === Read raw accelerometer and gyroscope values ===
void mpu6050_read_raw() {
  i2c_start();
  i2c_write(MPU6050_ADDR << 1);
  i2c_write(0x3B);  // Start reading from ACCEL_XOUT_H
  i2c_start();
  i2c_write((MPU6050_ADDR << 1) | 0x01);  // Restart as read

  for (int i = 0; i < 3; i++) {
    uint8_t hi = i2c_read_ack();
    uint8_t lo = i2c_read_ack();
    accel[i] = (hi << 8) | lo;
  }

  for (int i = 0; i < 3; i++) {
    uint8_t hi = i2c_read_ack();
    uint8_t lo = (i == 2) ? i2c_read_nack() : i2c_read_ack();
    gyro[i] = (hi << 8) | lo;
  }

  i2c_stop();
}

// === 1 kHz Timer3 Compare Match ISR for control timing ===
ISR(TIMER3_COMPA_vect) {
  imu_ready = true;
  tick_count++;
}

// === Timer4 Compare Match ISR (1ms resolution) for joystick debounce ===
ISR(TIMER4_COMPA_vect) {
  joy_timer++;
}

// === ADC conversion complete ISR (for joystick read) ===
ISR(ADC_vect) {
  if (reading_adc1) {
    ADC1_value = ADC;
    adc1_done = true;
    reading_adc1 = false;

    // Switch to ADC9 next
    ADMUX = 0b01000001;
    ADCSRB = 0b00001000;
    ADCSRA |= (1 << ADSC);  // Start ADC9
  } else {
    ADC9_value = ADC;
    adc9_done = true;
    joystick_ready = true;
  }
}

// === Configure Timer3 for 1 kHz control loop ===
void timer1_init() {
  cli();
  TCCR3A = 0x00;
  TCCR3B = 0b00001011;  // CTC mode, prescaler = 64
  OCR3A = 250;          // 16 MHz / 64 / 250 = 1 kHz
  TIMSK3 = (1 << OCIE3A);
  sei();
}

// === Timer4 for 1ms tick (joystick debounce) ===
void timer4_init() {
  TCCR4A = 0x00;
  TCCR4B = 0b00001011;
  OCR4A = 249;
  TIMSK4 = (1 << OCIE4A);
}

// === PWM setup for motor control ===
void pwm_setup() {
  DDRA = 0b00000011;     // PA0, PA1 as outputs (direction control)
  DDRB |= (1 << PB5);    // PB5 as output (PWM enable)

  TCCR1A = 0b10000011;   // Fast PWM 10-bit
  TCCR1B = 0b00001011;   // Fast PWM mode, prescaler = 64
}

// === ADC configuration for interrupt-driven reads ===
void adc_setup() {
  ADMUX = 0b01000001;     // AVcc ref, ADC1
  ADCSRA = 0b10001110;    // Enable, auto-trigger off, interrupt enabled
  ADCSRB = 0b00000000;    // MUX5 = 0
}

// === Start joystick ADC cycle (ADC1 → ADC9) ===
void startJoystickCycle() {
  reading_adc1 = true;
  adc1_done = adc9_done = false;
  ADMUX = 0b01000001;
  ADCSRB = 0b00000000;
  ADCSRA |= (1 << ADSC);
}

// === Arduino setup ===
void setup() {
  Serial.begin(9600);
  i2c_init();
  mpu6050_init();
  timer1_init();
  timer4_init();
  pwm_setup();
  adc_setup();
  startJoystickCycle();
}

// === Main loop ===
void loop() {
  // Handle incoming serial input (manual angle setting)
  static String inputString = "";
  static bool inputComplete = false;

  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') inputComplete = true;
    else if (inChar != '\r') inputString += inChar;
  }

  if (inputComplete) {
    float user_angle = inputString.toFloat();
    if (user_angle >= 0.0 && user_angle <= 180.0) {
      target_angle = user_angle;
      Serial.print("Updated target_angle to: ");
      Serial.println(target_angle);
    } else {
      Serial.println("Invalid angle. Use 0–180.");
    }
    inputString = "";
    inputComplete = false;
  }

  // Joystick input mapped to target angle
  if (joystick_ready) {
    joystick_ready = false;

    bool moved = abs(ADC1_value - CENTER) > JOY_DEADZONE ||
                 abs(ADC9_value - CENTER) > JOY_DEADZONE;

    if (moved && (joy_timer - last_joy_move > JOY_DEBOUNCE_MS)) {
      last_joy_move = joy_timer;

      int y_offset = ADC9_value - CENTER;
      float delta_angle = y_offset / 50.0;  // Tuning factor

      target_angle += delta_angle;
      if (target_angle < 0) target_angle = 0;
      if (target_angle > 180) target_angle = 180;

      Serial.print("Joystick adjusted angle: ");
      Serial.println(target_angle);
    }

    startJoystickCycle(); // Start next joystick read
  }

  // Sensor reading and PID control loop (1 kHz)
  if (imu_ready) {
    imu_ready = false;

    mpu6050_read_raw();

    // Low-pass filter on gyro and accel
    filtered_gyro_x = gyro_alpha * filtered_gyro_x + (1.0 - gyro_alpha) * (gyro[0] / 131.0);
    filtered_ay = accel_alpha * filtered_ay + (1.0 - accel_alpha) * (accel[1] / 16384.0);
    filtered_az = accel_alpha * filtered_az + (1.0 - accel_alpha) * (accel[2] / 16384.0);

    // Estimate angle using complementary filter
    float delta_theta = filtered_gyro_x * dt * 3.14159 / 180.0;
    float theta_accel = atan2(-filtered_ay, filtered_az);
    theta_complementary = alpha * (theta_complementary + delta_theta) + (1.0 - alpha) * theta_accel;

    actual_angle = theta_complementary * 180.0 / 3.14159 + 90.0;
    smoothed_angle = angle_alpha * smoothed_angle + (1.0 - angle_alpha) * actual_angle;
    actual_angle = smoothed_angle;

    // PID controller
    error = target_angle - actual_angle;
    integral += error * dt;
    if (integral > 100.0) integral = 100.0;
    if (integral < -100.0) integral = -100.0;
    derivative = (error - prev_error) / dt;
    pid_output = Kp * error + Ki * integral + Kd * derivative;
    prev_error = error;

    // Motor control via PWM
    PORTB |= (1 << PB5);      // Enable motor
    PORTA = 0b00000000;       // Fixed direction (upward lift)

    int pwm_signal = (pid_output > 0) ? (int)pid_output : 0;
    if (pwm_signal > 0 && pwm_signal < 50) pwm_signal = 50;
    if (pwm_signal > 1023) pwm_signal = 1023;
    OCR1A = pwm_signal;       // Apply PWM output
  }

  // Debug output every 100 ms
  static unsigned long last_print = 0;
  if (tick_count - last_print >= 100) {
    last_print = tick_count;
    Serial.print("Angle: ");
    Serial.print(actual_angle, 2);
    Serial.print(" deg | Target: ");
    Serial.print(target_angle, 2);
    Serial.print(" deg | PID: ");
    Serial.println(pid_output);
  }
}