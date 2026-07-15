// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.xrp.XRPGyro;
import edu.wpi.first.wpilibj.xrp.XRPMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  private static final double kGearRatio =
      (30.0 / 14.0) * (28.0 / 16.0) * (36.0 / 9.0) * (26.0 / 8.0); // 48.75:1
  private static final double kCountsPerMotorShaftRev = 12.0;
  private static final double kCountsPerRevolution = kCountsPerMotorShaftRev * kGearRatio; // 585.0
  private static final double kWheelDiameterInch = 2.3622; // 60 mm

  private final PIDController m_headingController = new PIDController(0.03, 0.0, 0.001);

  private double m_targetHeadingDeg = 0.0;

  private static final double kTurnDeadband = 0.08;
  private static final double kForwardDeadband = 0.05;
  private static final double kMaxCorrection = 0.25;

  // The XRP has the left and right motors set to
  // channels 0 and 1 respectively
  private final XRPMotor m_leftMotor = new XRPMotor(0);
  private final XRPMotor m_rightMotor = new XRPMotor(1);

  // The XRP has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder m_leftEncoder = new Encoder(4, 5);
  private final Encoder m_rightEncoder = new Encoder(6, 7);

  // Set up the differential drive controller
  private final DifferentialDrive m_diffDrive =
      new DifferentialDrive(m_leftMotor::set, m_rightMotor::set);

  // Set up the XRPGyro
  private final XRPGyro m_gyro = new XRPGyro();

  // Set up the BuiltInAccelerometer
  private final BuiltInAccelerometer m_accelerometer = new BuiltInAccelerometer();

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    SendableRegistry.addChild(m_diffDrive, m_leftMotor);
    SendableRegistry.addChild(m_diffDrive, m_rightMotor);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotor.setInverted(true);

    // Use inches as unit for encoder distances
    m_leftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);
    m_rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);
    resetEncoders();
  }

  public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    m_diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate);
  }

  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  public int getLeftEncoderCount() {
    return m_leftEncoder.get();
  }

  public int getRightEncoderCount() {
    return m_rightEncoder.get();
  }

  public double getLeftDistanceInch() {
    return m_leftEncoder.getDistance();
  }

  public double getRightDistanceInch() {
    return m_rightEncoder.getDistance();
  }

  public double getAverageDistanceInch() {
    return (getLeftDistanceInch() + getRightDistanceInch()) / 2.0;
  }

  /**
   * The acceleration in the X-axis.
   *
   * @return The acceleration of the XRP along the X-axis in Gs
   */
  public double getAccelX() {
    return m_accelerometer.getX();
  }

  /**
   * The acceleration in the Y-axis.
   *
   * @return The acceleration of the XRP along the Y-axis in Gs
   */
  public double getAccelY() {
    return m_accelerometer.getY();
  }

  /**
   * The acceleration in the Z-axis.
   *
   * @return The acceleration of the XRP along the Z-axis in Gs
   */
  public double getAccelZ() {
    return m_accelerometer.getZ();
  }

  /**
   * Current angle of the XRP around the X-axis.
   *
   * @return The current angle of the XRP in degrees
   */
  public double getGyroAngleX() {
    return m_gyro.getAngleX();
  }

  /**
   * Current angle of the XRP around the Y-axis.
   *
   * @return The current angle of the XRP in degrees
   */
  public double getGyroAngleY() {
    return m_gyro.getAngleY();
  }

  /**
   * Current angle of the XRP around the Z-axis.
   *
   * @return The current angle of the XRP in degrees
   */
  public double getGyroAngleZ() {
    return m_gyro.getAngleZ();
  }

  /** Reset the gyro. */
  public void resetGyro() {
    m_gyro.reset();
    m_targetHeadingDeg = 0.0;
    m_headingController.reset();
  }

  public double getHeadingDegrees() {
    return m_gyro.getAngle();
  }

  public double getGyroRateZ() {
    return m_gyro.getRateZ();
  }

  public void arcadeDriveGyroAssist(double xaxisSpeed, double zaxisRotate) {

    // Driver is intentionally turning
    if (Math.abs(zaxisRotate) > kTurnDeadband) {
      m_targetHeadingDeg = getHeadingDegrees();
      arcadeDrive(xaxisSpeed, zaxisRotate);
      return;
    }

    // Robot is stopped
    if (Math.abs(xaxisSpeed) < kForwardDeadband) {
      m_targetHeadingDeg = getHeadingDegrees();
      arcadeDrive(0.0, 0.0);
      return;
    }

    // Driving straight, use gyro correction
    double correction =
        m_headingController.calculate(
            getHeadingDegrees(),
            m_targetHeadingDeg);

    // Clamp correction
    correction = Math.max(
        -kMaxCorrection,
        Math.min(kMaxCorrection, correction));

    arcadeDrive(xaxisSpeed, correction);
  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("Gyro Heading Deg", getHeadingDegrees());
    SmartDashboard.putNumber("Gyro Rate Z", getGyroRateZ());

    SmartDashboard.putNumber("Left Distance Inch", getLeftDistanceInch());
    SmartDashboard.putNumber("Right Distance Inch", getRightDistanceInch());
    SmartDashboard.putNumber("Average Distance Inch", getAverageDistanceInch());

    SmartDashboard.putNumber(
        "Left Right Distance Difference",
        getLeftDistanceInch() - getRightDistanceInch());
    SmartDashboard.putNumber("Heading", getHeadingDegrees());
    SmartDashboard.putNumber("Left Count", getLeftEncoderCount());
    SmartDashboard.putNumber("Right Count", getRightEncoderCount());
  }
}
