// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.xrp.XRPGyro;
import edu.wpi.first.wpilibj.xrp.XRPMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;


public class Drivetrain extends SubsystemBase {
  private static final double kGearRatio =
      (30.0 / 14.0) * (28.0 / 16.0) * (36.0 / 9.0) * (26.0 / 8.0); // 48.75:1
  private static final double kCountsPerMotorShaftRev = 12.0;
  private static final double kCountsPerRevolution = kCountsPerMotorShaftRev * kGearRatio; // 585.0
  private static final double kWheelDiameterInch = 2.3622; // 60 mm

  //P: 0.055  D: 0.65
  private final PIDController m_headingController = new PIDController(0.055, 0.0, 0.65);

  private double m_targetHeadingDeg = 0.0;

  private static final double kTurnDeadband = 0.08; 
  private static final double kForwardDeadband = 0.05;
  private static final double kMaxCorrection = 0.375; // 0.375
  private static final double kTrackWidthMeters = 0.155;

  private double m_imuHeadingDeg = 0.0;
  private double m_gyroBiasDegPerSec = 0.0;
  private double m_lastImuTimestamp = Timer.getFPGATimestamp();

  private static final double kGyroRateDeadband = 0.3; // deg/sec
  private static final double kMaxSpeedMetersPerSec = 0.7;

  private final DifferentialDriveKinematics m_Kinematics =
      new DifferentialDriveKinematics(kTrackWidthMeters);

  private final DifferentialDriveOdometry m_odometry;

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

  public final ChassisSpeeds getRobotRelativeSpeeds() {
    return m_Kinematics.toChassisSpeeds(getWheelSpeeds());
  }

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

    m_odometry =
        new DifferentialDriveOdometry(
          Rotation2d.fromDegrees(getHeadingDegrees()),
          0,
          0);

    RobotConfig config = null;

    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
    }

    if (config != null) {
      AutoBuilder.configure(
        this::getPose,
        this::resetPose,
        this::getRobotRelativeSpeeds,
        (speeds, feedforwards) -> driveRobotRelative(speeds),
        new PPLTVController(0.02),
        config,
        () -> {
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this);
    }
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
    m_imuHeadingDeg = 0.0;
    m_targetHeadingDeg = 0.0;
    m_headingController.reset();
    m_lastImuTimestamp = Timer.getFPGATimestamp();
  }


  public double getHeadingDegrees() {
    return m_imuHeadingDeg;
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

  public void driveRobotRelative(ChassisSpeeds speeds) {

      DifferentialDriveWheelSpeeds wheelSpeeds = m_Kinematics.toWheelSpeeds(speeds);

      double leftPercent = wheelSpeeds.leftMetersPerSecond/kMaxSpeedMetersPerSec;

      double rightPercent = wheelSpeeds.rightMetersPerSecond/kMaxSpeedMetersPerSec;

      double max = Math.max(Math.abs(leftPercent), Math.abs(rightPercent));

      if (max > 1.0) {
        leftPercent /= max;
        rightPercent /=max;
      }

      m_leftMotor.set(leftPercent);
      m_rightMotor.set(rightPercent);
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void resetPose(Pose2d pose) {
    resetEncoders();
    resetGyro();

    m_odometry.resetPosition(
        Rotation2d.fromDegrees(getHeadingDegrees()),
        0,
        0,
        pose);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        Units.inchesToMeters(m_leftEncoder.getRate()),
        Units.inchesToMeters(m_rightEncoder.getRate())
    );
  }

  public void calibrateGyroBias() {
    m_gyro.reset();

    double sum = 0.0;
    int samples = 300;

    for (int i = 0; i < samples; i++) {
      double rate = m_gyro.getRateZ();
      System.out.println("Sample " + i + ": " + rate);
      sum += rate;
      Timer.delay(0.01);
    }

    m_gyroBiasDegPerSec = sum / samples;

    m_imuHeadingDeg = 0.0;
    m_targetHeadingDeg = 0.0;
    m_headingController.reset();
    m_lastImuTimestamp = Timer.getFPGATimestamp();
  }

  private void updateIntegratedImuHeading() {
    double now = Timer.getFPGATimestamp();
    double dt = now - m_lastImuTimestamp;
    m_lastImuTimestamp = now;

    if (dt <= 0.0 || dt > 0.1) {
      return;
    }

    double rate = m_gyro.getRateZ() - m_gyroBiasDegPerSec;

    if (Math.abs(rate) < kGyroRateDeadband) {
      rate = 0.0;
    }

    m_imuHeadingDeg += rate * dt;
  }


@Override
public void periodic() {
  updateIntegratedImuHeading();

  double correction =
    m_headingController.calculate(
        getHeadingDegrees(),
        m_targetHeadingDeg);

  SmartDashboard.putNumber("Target Heading", m_targetHeadingDeg);
  SmartDashboard.putNumber("Heading Error",
      m_targetHeadingDeg - getHeadingDegrees());
  SmartDashboard.putNumber("Correction", correction);

  m_odometry.update(
      Rotation2d.fromDegrees(getHeadingDegrees()),
      Units.inchesToMeters(getLeftDistanceInch()),
      Units.inchesToMeters(getRightDistanceInch()));

  SmartDashboard.putNumber("Raw Gyro Angle", m_gyro.getAngle());
  SmartDashboard.putNumber("Raw Gyro Rate Z", m_gyro.getRateZ());
  SmartDashboard.putNumber("Gyro Bias", m_gyroBiasDegPerSec);
  SmartDashboard.putNumber("Integrated IMU Heading", getHeadingDegrees());

  SmartDashboard.putNumber("Left Distance Inch", getLeftDistanceInch());
  SmartDashboard.putNumber("Right Distance Inch", getRightDistanceInch());
  SmartDashboard.putNumber("Left Rate", m_leftEncoder.getRate());
  SmartDashboard.putNumber("Right Rate", m_rightEncoder.getRate());

  SmartDashboard.putNumber("Pose X", getPose().getX());
  SmartDashboard.putNumber("Pose Y", getPose().getY());
  SmartDashboard.putNumber("Pose Rotation", getPose().getRotation().getDegrees());
  SmartDashboard.putNumber("Robot Velocity", getRobotRelativeSpeeds().vxMetersPerSecond);
  SmartDashboard.putNumber("Robot Angular Velocity", getRobotRelativeSpeeds().omegaRadiansPerSecond * 57.2958);

}
}
