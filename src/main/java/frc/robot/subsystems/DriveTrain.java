// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/** DriveTrain Conventions:
 * Shooter is at front of robot
 * Intake is at back of robot
 * Positive speed values should move forward
 * Forward movement should result in positive encoder readings
 * Gyro reads positive angle for clockwise rotation
 * Standard units for control classes are meters
 */

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  public static final double kMaxSpeed = 3.0; // meters per second
  public static final double kMaxAngularSpeed = 2 * Math.PI; // one rotation per second

  private static final double kTrackWidth = 0.57; // meters
  private static final double kWheelRadius = Constants.INCHES_TO_METERS * 3.0; // meters
  private static final int kEncoderResolution = 2048; // cycles per revolution

  private final WPI_VictorSPX leftMotor0;
  private final WPI_VictorSPX leftMotor1;
  private final WPI_VictorSPX rightMotor0;
  private final WPI_VictorSPX rightMotor1;

  private final SpeedControllerGroup m_left;
  private final SpeedControllerGroup m_right;

  private final DifferentialDrive m_drive;
  private final DifferentialDriveKinematics m_kinematics;

  private final Encoder m_leftEncoder;
  private final Encoder m_rightEncoder;

  private final Gyro m_gyro;

  /** Creates a new DriveTrain. */
  public DriveTrain() {
    leftMotor0 = new WPI_VictorSPX(Constants.DRIVE_LEFT_VICTORSPX0);
    leftMotor1 = new WPI_VictorSPX(Constants.DRIVE_LEFT_VICTORSPX1);
    rightMotor0 = new WPI_VictorSPX(Constants.DRIVE_RIGHT_VICTORSPX0);
    rightMotor1 = new WPI_VictorSPX(Constants.DRIVE_RIGHT_VICTORSPX1);

    leftMotor0.setNeutralMode(NeutralMode.Brake);
    leftMotor1.setNeutralMode(NeutralMode.Brake);
    rightMotor0.setNeutralMode(NeutralMode.Brake);
    rightMotor1.setNeutralMode(NeutralMode.Brake);

    m_left = new SpeedControllerGroup(leftMotor0, leftMotor1);
    m_right = new SpeedControllerGroup(rightMotor0, rightMotor1);

    m_left.setInverted(true);
    m_right.setInverted(true);

    m_drive = new DifferentialDrive(m_left, m_right);
    m_kinematics = new DifferentialDriveKinematics(kTrackWidth);

    m_leftEncoder = new Encoder(Constants.ENCODER_LEFT0, Constants.ENCODER_LEFT1);
    m_rightEncoder = new Encoder(Constants.ENCODER_RIGHT0, Constants.ENCODER_RIGHT1);

    m_leftEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);
    m_rightEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);

    m_leftEncoder.setReverseDirection(true);
    m_rightEncoder.setReverseDirection(false);

    m_leftEncoder.reset();
    m_rightEncoder.reset();

    m_gyro = new ADXRS450_Gyro();
    m_gyro.calibrate();
    m_gyro.reset();

    // Subsystem on SmartDashboard
    addChild("Drive", m_drive);
  }

  /**
   * Arcade style driving
   * Currently ineffective, consider extending to implement alongside tank drive
   * 
   * @param speed in range [-1.0, 1.0]
   * @param rotate in range [-1.0, 1.0]
   */
  public void arcadeDrive(double speed, double rotate) {
    var wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(speed, 0, rotate));
    tankDrive(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
  }

  /**
   * Tank style driving (consider adding balancing factors or limiting speeds)
   * 
   * @param left in range [-1.0, 1.0]
   * @param right in range [-1.0, 1.0]
   */
  public void tankDrive(double left, double right) {
    m_drive.tankDrive(left, right);
  }

  /**
   * Stops the drivetrain motors
   * Consider using the stopMotor() method in DifferentialDrive class
   */
  public void stop() {
    m_drive.tankDrive(0, 0);
  }

  /**
   * Gets the robot's current heading in degrees since last reset
   * Counterclockwise is a positive angle, clockwise is a negative angle
   * Consider returning in range [-180, 180]
   * 
   * @return current heading with unlimited range
   */
  public double getHeading() {
    return m_gyro.getAngle();
  }

  /**
   * Gets the robot's turn rate
   * 
   * @return angular velocity in degrees per second with unlimited range
   */
  public double getTurnRate() {
    return m_gyro.getRate();
  }

  /**
   * Resets the gyro to zero heading
   * Forgoes calibration because it takes too long in a match
   * Calibrate during instantiation (when the robot turns on)
   */
  public void resetGyro() {
    m_gyro.reset();
  }

  /**
   * Returns the distance reading from the left encoder
   * Consider converting reading to inches for SmartDashboard
   * 
   * @return left encoder distance in meters
   */
  public double getLeftEncoderDistance() {
    return m_leftEncoder.getDistance();
  }

  /**
   * Returns distance reading from the right encoder
   * Consider converting reading to inches for SmartDashboard
   * 
   * @return right encoder distance in meters
   */
  public double getRightEncoderDistance() {
    return m_rightEncoder.getDistance();
  }

  public double getAverageEncoderDistance() {
    return (getLeftEncoderDistance() + getRightEncoderDistance()) / 2;
  }

  /**
   * Returns velocity reading from the left encoder
   * Consider converting reading to inches / second for SmartDashboard
   * 
   * @return left encoder rate in meters per second
   */
  public double getLeftEncoderRate() {
    return m_leftEncoder.getRate();
  }

  /**
   * Returns velocity reading from the right encoder
   * Consider converting reading to inches / second for Smartdashboard
   * 
   * @return right encoder rate in meters per second
   */
  public double getRightEncoderRate() {
    return m_rightEncoder.getRate();
  }

  /**
   * Resets the encoder readings to zero
   * Use when beginning any pathing for accurate readings
   */
  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  public void log() {
    SmartDashboard.putNumber("Left motor output", m_left.get());
    SmartDashboard.putNumber("Right motor output", m_right.get());
    SmartDashboard.putNumber("Heading", m_gyro.getAngle());
    SmartDashboard.putNumber("Angular Velocity", m_gyro.getRate());
    SmartDashboard.putNumber("Left Encoder Distance", getLeftEncoderDistance());
    SmartDashboard.putNumber("Right Encoder Distance", getRightEncoderDistance());
    SmartDashboard.putNumber("Left Encoder Rate", getLeftEncoderRate());
    SmartDashboard.putNumber("Right Encoder Rate", getRightEncoderRate());
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    log();
  }
}
