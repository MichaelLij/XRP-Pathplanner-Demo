# XRP PathPlanner Configuration Reference

## Overview

This XRP has already been configured and prepared for testing. No imaging, firmware setup, or network configuration is required prior to use.

This document contains:

- Startup instructions
- Connection instructions
- Driver controls
- XRP hardware specifications
- PathPlanner robot configuration values
- PathPlanner default constraints
- Testing notes and battery considerations

---

# Startup and Connection

## Powering On

1. Verify batteries are installed.
2. Turn the XRP power switch ON.
3. Leave the robot stationary for approximately 3–5 seconds while the onboard IMU completes calibration.
4. Wait for startup to complete before moving the robot.

---

## Connecting to the Robot

Connect your PC to the following WiFi network:

```text
XRP-8355-b635
```

Once connected:

1. Open the project in VS Code.
2. Deploy code using the WPILib "Simulate Robot Code".
3. Open the "Robot Simulation", this will act as your Driver Station and dashboard.
4. Open your dashboard by going to "NetworkTables" -> "Transitory values" -> "SmartDashboard"
5. Calibrate the gyro before teleop and autonomous testing.
   - Place robot on flat ground.
   - Press Y to calibrate gyro bias.
   - Verify:
       - Integrated IMU Heading
       - Pose Rotation
       - Encoder readings

---

# Control Scheme

## Driving

| Control | Function |
|----------|----------|
| Left Stick Y (Axis 1) | Forward / Reverse |
| Right Stick X (Axis 4) | Turn Left / Right |
| Left Trigger (Axis 2) | Drive speed multiplier |

Drive speed is calculated as:

```java
0.6 + (0.4 * LeftTrigger)
```

Resulting in:

| Trigger Position | Drive Speed Multiplier |
|------------------|------------------------|
| Released | 0.60 |
| Half Pressed | 0.80 |
| Fully Pressed | 1.00 |

---

## Buttons

| Button | Function |
|----------|----------|
| A | Move arm to 45° |
| A Released | Return arm to 90° |
| B | Move arm to 0° |
| B Released | Return arm to 90° |
| X | Reset gyro and heading |
| Y | Calibrate gyro bias |

---

# XRP Hardware Information

| Parameter | Value |
|------------|------------|
| Drive Type | Differential Drive |
| Wheel Diameter | 60 mm (2.3622 in) |
| Wheel Radius | 0.030 m |
| Track Width | 0.155 m |
| Drive Gear Reduction | 48.75 : 1 |
| Encoder CPR | 12 |
| Counts Per Wheel Revolution | 585 |

---

# PathPlanner Robot Configuration

## Robot Properties

| Parameter | Value |
|------------|------------|
| Robot Mass | 0.400 kg |
| Robot MOI | 0.014 kg·m² |
| Track Width | 0.155 m |

---

## Bumper Configuration

| Parameter | Value |
|------------|------------|
| Bumper Width | 0.190 m |
| Bumper Length | 0.190 m |
| Bumper Offset X | 0.000 m |
| Bumper Offset Y | 0.000 m |

---

## Module Configuration

| Parameter | Value |
|------------|------------|
| Wheel Radius | 0.030 m |
| Drive Gearing | 48.75 |
| True Max Drive Speed | 1.000 m/s |
| Wheel COF | 0.700 |
| Drive Motor | MiniCIM |
| Drive Current Limit | 2 A |

---

# PathPlanner Default Constraints

| Parameter | Value |
|------------|------------|
| Max Velocity | 0.700 m/s |
| Max Acceleration | 0.560 m/s² |
| Max Angular Velocity | 500 deg/s |
| Max Angular Acceleration | 1000 deg/s² |
| Nominal Voltage | 6.0 V |

---

# Drivetrain Heading Hold Configuration

Current experimentally tuned values:

| Parameter | Value |
|------------|------------|
| P | 0.055 |
| I | 0.000 |
| D | 0.650 |
| Max Correction | 0.375 |

These values were tuned specifically for the XRP drivetrain and may require adjustment if drivetrain components are changed.

---

# Battery Disclaimer

All measurements and PathPlanner values listed in this document were obtained using:

- Fresh Energizer non‑rechargeable AA batteries
- Standard XRP drivetrain

These values should be treated as baseline values for a fully charged robot.

As battery charge decreases:

- Maximum velocity will decrease
- Maximum acceleration will decrease
- Maximum angular velocity will decrease
- Maximum angular acceleration will decrease
- Autonomous path performance may vary slightly

Because of this, exact PathPlanner behavior may differ between battery sets or at lower battery percentages.

---

# PathPlanner Notes

- Configure PathPlanner as a differential drive robot.
- Holonomic mode must remain disabled.
- Robot code and PathPlanner must use the same:
  - Track Width
  - Wheel Radius
  - Drive Gearing
- IMU should be calibrated before autonomous testing.
- If turning accuracy degrades, verify:
  - Gyro calibration
  - Track width
  - True max drive speed
  - Wheel slippage
  - Battery charge level
  - Encoder operation

---

# Testing Procedure

Before autonomous testing:

1. Power on robot.
2. Connect to:
   ```text
   XRP-8355-b635
   ```
3. Deploy code.
4. Place robot on flat ground.
5. Calibrate gyro.
7. Open "NetworkTables" tab at the top left, different from the table.
8. Hover "SmartDashboard" and select "SendableChooser[0]".
9. Select your desired auto.
10. Select "Robot State" to be "Autonomous".

Note:
If you want to do teleop testing, steps 1-5 are also the teleop testing procedure except at the end you select "Robot State" to be "Teleoperated".
