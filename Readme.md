# Autonomous Stabilizing Drone — Firmware

Arduino-based flight controller firmware for a self-built quadcopter. Reads inertial data from an MPU6050, fuses it with a Kalman filter for stable angle estimates, and runs a cascaded (angle → rate) PID controller to keep the aircraft level, with pilot commands coming in over a PPM RC receiver.

**Demo video:** [Live flight demonstration and explanation](https://www.youtube.com/watch?v=yZt6ehGUBP8)

## Hardware

- **Flight controller:** Arduino (any board with `Wire`, `Servo`, and an external interrupt pin)
- **IMU:** MPU6050 (I2C address `0x68`) — 3-axis accelerometer + 3-axis gyroscope
- **RC receiver:** PPM output, 4 channels (roll, pitch, throttle, yaw), wired to the interrupt pin
- **Motors/ESCs:** 4x, driven as standard RC servo signals (1000–2000 µs pulse width)

### Motor layout (X configuration)

| Motor | Pin | Wire Color |
|---|---|---|
| Front Right | 11 | White |
| Front Left | 9 | Green |
| Back Right | 10 | Yellow |
| Back Left | 8 | Orange |

- **Interrupt pin (PPM in):** 2
- **Status LED:** pin 13 (lit during startup/calibration)

## Dependencies

- [`Wire`](https://www.arduino.cc/en/reference/wire) — built-in, I2C communication with the MPU6050
- [`PPMReader`](https://github.com/nikhilnarayana/PPMReader) — decodes the PPM signal into per-channel values
- [`Servo`](https://www.arduino.cc/reference/en/libraries/servo/) — built-in, drives the ESCs

## How it works

1. **Startup (`setup()`)**
   - Initializes serial (57600 baud) and I2C at 400 kHz
   - Wakes the MPU6050 out of sleep mode, configures the accelerometer/gyro registers
   - Samples the gyro 2000 times at rest to compute per-axis bias (calibration), and takes a single accelerometer reading to zero the roll/pitch angle offsets
   - Attaches the four ESC signals and arms them at zero throttle

2. **Sensor fusion (`gyroSignals()` + `kalman()`)**
   - Reads raw accelerometer and gyro registers over I2C each loop
   - Derives roll/pitch angle from the accelerometer via `atan`, and rate-of-rotation from the gyro
   - Removes the calibration bias from each gyro axis
   - Combines the (noisy but drift-free) accelerometer angle with the (smooth but drifting) gyro rate through a 1D Kalman filter to get a stable roll/pitch angle estimate each loop

3. **Control loop (`loop()`)**
   - Reads the 4 PPM channels (roll, pitch, throttle, yaw) via `getReceiverInput()`
   - Converts stick input into a **desired angle** for roll/pitch and a **desired rate** for yaw
   - **Outer loop:** compares desired angle to the Kalman-filtered angle to produce a desired angular *rate*
   - **Inner loop:** a PID controller (`getPIDEffort()`) drives the actual gyro rate toward that desired rate, independently for roll, pitch, and yaw
   - PID gains are tuned per axis (`PRateRoll`, `IRatePitch`, `DRateYaw`, etc.), with integral windup and output clamped to ±400
   - Roll, pitch, and yaw efforts are mixed with throttle using standard X-quad mixing equations to get each motor's power
   - Motor power is clamped to a safe range, mapped to a servo angle (0–180°), and written out
   - If throttle drops below a cutoff threshold, all motors are set to idle and the PID integrators are reset (prevents integral windup while landed/disarmed)

4. **Loop timing**
   - The main loop is throttled to run at a fixed 4000 µs (250 Hz) cycle using a busy-wait on `micros()`, keeping the PID's fixed 0.004 s timestep accurate

## Tuning

PID gains are hardcoded near the top of the file:

```cpp
float PRateRoll = 0.46;
float PRatePitch = -0.4;
float PRateYaw = 0.005;

float IRateRoll = 0;
float IRatePitch = -0.25;
float IRateYaw = 0;

float DRateRoll = 0.007;
float DRatePitch = -0.007;
float DRateYaw = 0;
```

Several `Serial.print` debug blocks are left commented out throughout `loop()` — uncomment the relevant block to log angles, rates, PID effort, or per-motor power while tuning.

## Safety notes

- Always remove propellers when bench-testing or re-tuning PID gains.
- `throttleCutOff` (1050) disarms the motors and resets the PID integrators whenever the throttle stick is low — don't rely on this as a substitute for a proper arm/disarm switch.
- Motor power is hard-clamped between `minPower` (1180) and 2000 µs to avoid stalling a motor mid-flight.
