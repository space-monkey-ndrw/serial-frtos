<!-- ABOUT THE PROJECT -->
## About The Project

### Built With

Hardware:
- Raspberry Pi 5 8GB Single-board computer (SBC) https://www.raspberrypi.com/documentation/computers/processors.html#bcm2712
- STM32G431RBT6(U?) Nucleo-64 Nucleo-G431RB Microcontroller Unit (MCU)
- Nucleo-F446RE https://www.digikey.com/en/products/detail/stmicroelectronics/NUCLEO-F446RE/5347712
- Geekwork X1202 4-Cell 18650 5.1V 5A UPS HAT https://wiki.geekworm.com/X1202
- Slamtec RPLiDAR A1M8 LiDAR https://www.adafruit.com/product/4010
- Adafruit LSM6DSOX + LIS3MDL FeatherWing - Precision 9-DoF IMU https://www.adafruit.com/product/4565
   *** NOTE *** the SDA and SCL silkscreen labels are sometimes wrong on this board - pin labeled SDA is actually SCL, and labeled SCL is actually SDA.  Check FeatherWing standard to confirm. **
   - LSM6DSOX has flexible data rates and ranges. For the accelerometer: ±2/±4/±8/±16 g at 1.6 Hz to 6.7KHz update rate. For the gyroscope: ±125/±250/±500/±1000/±2000 dps at 12.5 Hz to 6.7 KHz
   - LIS3MDL magnetic sensor features a configurable output data rate (ODR) ranging from 0.625 Hz up to 1000 Hz
- 4x18650 batteries https://www.centralcomputer.com/icr-18650-1pack-3-7v-2600mah-battery-w-flat-cap.html
- Adafruit Chassis Kit https://www.adafruit.com/product/2939
- Adafruit TB6612 1.2A DC/Stepper Motor Driver Breakout Board https://www.adafruit.com/product/2448
- SanDisk Ultra 64GB MicroSD card C10 XC I

Software:
- Linux Ubuntu 24.04 LTS Noble Numbat (Server on RPi 5) https://ubuntu.com/blog/tag/ubuntu-24-04-lts
- ROS2 Jazzy Jalisco https://docs.ros.org/en/jazzy/index.html
- (deprecated) Micro-ROS on MCU
- FreeRTOS on MCU
- STM32CubeIDE v. 2.0.0
- STM32CubeProgrammer v. 2.22.0
- STM32CubeMX v. 6.16.1
- VSCode v. 1.110.1

<!-- INTERFACE CONTROL DOCUMENT -->
## Interface Control Document

1. System Architecture Overview
High-Level Controller: Raspberry Pi 5 (Ubuntu 24.04 / ROS 2).
Low-Level Controller: STM32 G432RB (NUCLEO-64).
Primary Communication: USB 2.0 (Serial) and I2C.

2. Physical Interface Map
Interface       From        To              Protocol            Purpose
USB-A 1         RPi 5       RPLIDAR A1      UART (via USB)      Laser Scan Data
USB-A 2         RPi 5       STM32           UART / micro-ROS    Fused IMU & Encoders
I2C1            STM32       Adafruit IMU    I2C (100kHz)        Raw Accel/Gyro/Mag
Power           X1202       RPi 5           GPIO/USB-C          5V/5A Power Delivery


3. Software Interface & Data Protocols
3.1. STM32 to IMU (Local Bus)
    Protocol: I2C.
    Sample Rate: 200 Hz (Fixed).
    Data Processed: STM32 runs Madgwick Filter locally using its Floating Point Unit (FPU).
3.2. STM32 to Raspberry Pi (Transport Layer)
    Protocol: micro-ROS (Recommended) or UART Serial (115,200 or 921,600 baud).
    Message Type: sensor_msgs/msg/Imu
    Payload Definition:
        header.stamp: STM32 clock time (synchronized via ROS 2).
        orientation: Quaternion (q_w, q_x, q_y, q_z) (𝑥,𝑦,𝑧,𝑤) derived from Madgwick filter.
        angular_velocity: Raw Gyroscope data (w_x, w_y, w_z) (𝑟𝑎𝑑/𝑠).
        linear_acceleration: Raw Accelerometer data (a_x, a_y, a_z) (𝑚/𝑠2).
3.3. RPLIDAR to Raspberry Pi
    Protocol: Standard Slamtec ROS 2 Driver.
    Message Type: sensor_msgs/msg/LaserScan
    Topic: /scan

4. Logic & Fusion Flow
Component       Input Data                      Algorithm                   Output
STM32           Raw 9-DoF IMU                   Madgwick Filter             Filtered Orientation
RPi 5 (EKF)     STM32 Orientation + Wheel Odom  EKF (robot_localization)    Fused Odometry (/odom)
RPi 5 (SLAM)    Fused Odometry + Laser Scan     Slamtec / Cartographer      Occupancy Grid (Map)


5. Electrical Requirements
- Voltage: The Geekworm X1202 must provide a steady 5V to the Pi 5.
- Logic Levels: The STM32 G432RB is 3.3V logic. Ensure the Adafruit IMU is powered via the 3.3V pin on the Nucleo to avoid level-shifting issues.
- Grounding: Ensure a common ground between the STM32 and the Raspberry Pi if they are powered by different ports on the X1202.

6. Safety & Fail-Safes
- Watchdog: If the Raspberry Pi stops heartbeat communication, the STM32 must command motor velocity to 0.
- LIDAR Stall: If the RPLIDAR rotation speed drops below 5Hz, the SLAM node should flag a "Quality Degraded" warning.

Nucleo-G431RB: Primary I2C Pins (Arduino Connector)
The default I2C1 interface pins, which are most easily accessed on the Arduino headers, are:
SCL (Serial Clock): PB8 (Arduino D15)
SDA (Serial Data): PB9 (Arduino D14)
Alternative I2C Pin Mappings
I2C1: PB6 (SCL), PB7 (SDA)
I2C2: PA9 (SCL), PA10 (SDA)
I2C3: PB3 (SCL), PA8 (SDA)

Wiring:
TB6612 -> wire color -> Nucleo-64 -> STM32 -> code
PWMA		brown		D7			PA8			PWMA_Pin and PWMA_GPIO_Port aka TIM1_CH1 aka PWMA
AIN2		red			D4			PB5			AIN2_Pin and AIN2_GPIO_Port aka AIN2
AIN1		orange		D5			PB4			AIN1_Pin and AIN1_GPIO_Port aka AIN1
STBY		blue		D6			PB10		STBY_Pin and STBY_GPIO_Port aka STBY
BIN1		green		D2			PA10		BIN1_Pin and BIN1_GPIO_Port aka BIN1
BIN2		yellow		D9			PC7			BIN2_Pin and BIN2_GPIO_Port aka BIN2
PWMB		white		D8			PA9			PWMB_Pin and PWMB_GPIO_Port aka TIM1_CH2 aka PWMB
GND			purple		GND			GND
Vcc			gray		3.3V		3.3V

Adafruit LSM6DSOX + LIS3MDL FeatherWing - Precision 9-DoF IMU
IMU     -> wire color -> Nucleo-64  -> code
SDA*        header (f)      SCL
SCL*        header (f)      SDA
3.3V        orange          3V3
GND         green           GND
* note - SDA and SCL silkscreen labels are swapped on this IMU *


<!-- ROADMAP -->
## Roadmap

- [ ] CORDIC Integration: Use STM32 LL_CORDIC_Config to accelerate the 1/sqrt(x) normalization required for every Madgwick step.
- [ ] Pack data (C struct, 4x float for Quaternions, encode with COBS
- [ ] Send data (UART with DMA for zero-CPU-overhead communication, LL UART IDLE interrupt that triggers packet transmission)
- [ ] Receive data
- [ ] decode COBS packet
- [ ] unpack C struct
- [ ] CRC16 validation
- [ ] Wrap into ROS2 Node
- [ ] Publish ROS2 message
- [ ] update SLAM
- [ ] Update motor controls
- [ ] pack motor controls into packed C struct
- [ ] send motor control data (packed struct) to MCU
