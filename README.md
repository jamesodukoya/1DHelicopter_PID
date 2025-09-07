# PID-Controlled Ducted-Fan Angular Positioning System

## Project Overview

This project demonstrates the implementation of a closed-loop control system for a ducted-fan-powered pivot arm using the ATmega2560 microcontroller. The system maintains precise angular positioning through real-time PID control, sensor fusion, and embedded software architecture operating at 1 kHz control frequency.

## Technical Objectives

- **Closed-Loop Control**: Implement a functional PID controller for single-degree-of-freedom angular positioning
- **Sensor Fusion**: Develop complementary filter algorithm combining accelerometer and gyroscope data for robust angle estimation
- **Real-Time Processing**: Execute control loop at 1 kHz with interrupt-driven architecture
- **Human-Machine Interface**: Integrate serial commands and analog joystick for real-time parameter adjustment

## System Architecture

### Hardware Configuration
- **Microcontroller**: ATmega2560 (16 MHz)
- **IMU Sensor**: MPU6050 6-DOF (accelerometer + gyroscope)
- **Motor Driver**: Cytron MD30C high-current controller
- **Power Supply**: 11.1V LiPo battery
- **Mechanical Structure**: Custom acrylic pivot arm with minimal flex design

### Control System Design
- **Control Frequency**: 1 kHz via Timer3 compare match interrupt
- **Sensor Communication**: I2C protocol at 100 kHz for MPU6050 data acquisition
- **Motor Control**: 10-bit Fast PWM using Timer1 with directional logic
- **User Interface**: Dual-input system (9600 baud serial + analog joystick with ADC interrupts)

![Pivot arm PID control](https://drive.google.com/uc?export=view&id=1CUofeUzQoliEHK2aM78wfnkrIGs3jkWs)

## Algorithm Implementation

### Complementary Filter for Angle Estimation

The system fuses accelerometer and gyroscope measurements using a complementary filter to achieve robust angle estimation despite sensor limitations:

```c
// Complementary filter implementation
float delta_theta = filtered_gyro_x * dt * 3.14159 / 180.0;
float theta_accel = atan2(-filtered_ay, filtered_az);
theta_complementary = alpha * (theta_complementary + delta_theta) + (1.0 - alpha) * theta_accel;
```

**Key Parameters:**
- **Filter Coefficient (α)**: 0.98 (98% gyroscope, 2% accelerometer weighting)
- **Sampling Rate**: 1000 Hz for optimal sensor fusion
- **Noise Mitigation**: Low-pass filtering on raw sensor data before fusion

### PID Controller Design

The discrete PID controller operates with integral windup protection and directional constraints:

```c
// PID computation with clamping
error = target_angle - actual_angle;
integral += error * dt;
if (integral > 100.0) integral = 100.0;  // Anti-windup clamping
if (integral < -100.0) integral = -100.0;
derivative = (error - prev_error) / dt;
pid_output = Kp * error + Ki * integral + Kd * derivative;
```

## Performance Characteristics

### Control System Metrics
- **Update Rate**: 1000 Hz control loop execution
- **Angle Resolution**: ~0.1° precision with complementary filtering
- **Response Time**: Sub-second settling for small angle changes
- **Control Range**: 0° to 90° operational envelope

### Experimental Tuning Results

| PWM Value | Steady-State Angle | Control Authority |
|-----------|-------------------|-------------------|
| 200       | ~15°              | Low power         |
| 250       | ~25°              | Medium power      |
| 300       | ~35°              | High power        |
| 350       | ~41°              | Near saturation   |

### PID Gain Analysis

**Optimized Parameters (Empirically Determined):**
- **Kp = 3.5**: Proportional gain balancing responsiveness vs stability
- **Ki = 0.8**: Integral gain with anti-windup protection
- **Kd = 1.2**: Derivative gain for overshoot reduction

## Embedded Systems Features

### Real-Time Architecture
- **Timer3**: 1 kHz control loop interrupt (CTC mode, prescaler = 64)
- **Timer4**: 1 ms timing base for user interface debouncing
- **Timer1**: 10-bit Fast PWM motor control (prescaler = 64)
- **ADC Interrupts**: Non-blocking joystick input acquisition

### I2C Communication Protocol
Custom I2C implementation for MPU6050 sensor interface:
- **Clock Speed**: 100 kHz (TWBR = 72)
- **Data Acquisition**: Sequential read of 6 sensor registers
- **Error Handling**: Proper acknowledgment and stop condition management

### Memory Management
- **Volatile Variables**: Proper interrupt-safe variable declarations
- **Stack Optimization**: Efficient ISR design minimizing overhead
- **Real-Time Constraints**: Deterministic execution timing

## Technical Challenges and Solutions

### System Limitations Identified
1. **Unidirectional Actuation**: No downward thrust capability limiting control authority
2. **Sensor Noise**: Motor vibrations affecting accelerometer measurements
3. **Nonlinear Dynamics**: Complex relationship between PWM input and angular output
4. **Mechanical Damping**: Absence of natural damping leading to oscillatory behavior

### Mitigation Strategies Implemented
- **Signal Filtering**: Multi-stage low-pass filtering on raw sensor data
- **Integral Windup Protection**: Clamping integral term to ±100
- **Feedforward Compensation**: PWM-to-angle mapping for improved response
- **Noise Rejection**: Complementary filter design emphasizing gyroscope data

## Code Architecture

### Interrupt Service Routines
```c
// 1 kHz control timing
ISR(TIMER3_COMPA_vect) {
    imu_ready = true;
    tick_count++;
}

// ADC conversion complete for joystick
ISR(ADC_vect) {
    // Dual-channel ADC state machine
    // ADC1 (x-axis) → ADC9 (y-axis) sequence
}
```

### Modular Function Design
- **I2C Communication**: `i2c_start()`, `i2c_write()`, `i2c_read_ack()`, `i2c_stop()`
- **Sensor Interface**: `mpu6050_init()`, `mpu6050_read_raw()`
- **System Configuration**: `timer1_init()`, `pwm_setup()`, `adc_setup()`

## Future Enhancements

### Hardware Improvements
- **Bidirectional Actuation**: Reversible motor or dual-fan configuration
- **Enhanced Sensors**: Higher-resolution encoders or optical positioning
- **Mechanical Damping**: Passive damping elements for improved stability

### Software Enhancements
- **Advanced Filtering**: Kalman filter implementation for optimal state estimation
- **Adaptive Control**: Gain scheduling based on operating conditions
- **Model Predictive Control**: Advanced control algorithms for nonlinear systems

## Technical Specifications

### Performance Metrics
- **Control Loop Frequency**: 1000 Hz
- **Sensor Update Rate**: 1000 Hz (I2C communication)
- **PWM Resolution**: 10-bit (1024 levels)
- **Angle Estimation Accuracy**: ±0.5° typical
- **System Response Time**: <500 ms to 90% of setpoint

### Resource Utilization
- **Program Memory**: ~8KB of 256KB available
- **SRAM Usage**: ~2KB of 8KB available  
- **Timer Resources**: 3 of 6 timers utilized
- **ADC Channels**: 2 of 16 channels used

---