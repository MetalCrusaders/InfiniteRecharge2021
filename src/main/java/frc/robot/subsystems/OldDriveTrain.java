// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class OldDriveTrain extends SubsystemBase {

  private final WPI_VictorSPX leftMotor0;
  private final WPI_VictorSPX leftMotor1;
  private final WPI_VictorSPX rightMotor0;
  private final WPI_VictorSPX rightMotor1;

  private final SpeedController m_left;
  private final SpeedController m_right;

  private final Encoder m_encoder0;
  private final Encoder m_encoder1;

  private final Gyro m_gyro;

  private final DifferentialDrive m_drive;

  private final double kLeft = 1.1;

  /** Creates a new DriveTrain. */
  public OldDriveTrain() {
    leftMotor0 = new WPI_VictorSPX(Constants.DRIVE_LEFT_VICTORSPX0);
    leftMotor1 = new WPI_VictorSPX(Constants.DRIVE_LEFT_VICTORSPX1);
    rightMotor0 = new WPI_VictorSPX(Constants.DRIVE_RIGHT_VICTORSPX0);
    rightMotor1 = new WPI_VictorSPX(Constants.DRIVE_RIGHT_VICTORSPX1);

    m_left = new SpeedControllerGroup(leftMotor0, leftMotor1);
    m_right = new SpeedControllerGroup(rightMotor0, rightMotor1);

    m_encoder0 = new Encoder(Constants.ENCODER_LEFT0, Constants.ENCODER_LEFT1);
    m_encoder1 = new Encoder(Constants.ENCODER_RIGHT0, Constants.ENCODER_RIGHT1);

    m_encoder0.setDistancePerPulse(1 / Constants.ENCODER_TO_INCHES);
    m_encoder1.setDistancePerPulse(-1 / Constants.ENCODER_TO_INCHES);

    m_gyro = new ADXRS450_Gyro();

    m_drive = new DifferentialDrive(m_left, m_right);

    // Sensors on SmartDashboard
    addChild("Drive", m_drive);
    addChild("Gyro", (Sendable) m_gyro);
  }

  /** 
   * Arcade style driving for the DriveTrain
   * 
   * @param speed in range [-1.0, 1.0]
   * @param rotate in range [-1.0, 1.0]. Clockwise is positive.
   */
  public void arcadeDrive(double speed, double rotate) {
    m_drive.arcadeDrive(speed, rotate);
  }

  /**
   * Tank style driving for the DriveTrain
   * 
   * @param left in range [-1.0, 1.0]
   * @param right in range [-1.0, 1.0]
   */
  public void tankDrive(double left, double right) {
    if(left > 0 && right > 0) {
      m_drive.tankDrive(0.8 * left, 0.8 * kLeft * right);
    }
    else {
      m_drive.tankDrive(0.8 * left, 0.8 * right);
    }
  }

  /** Stops the drivetrain motors */
  public void stop() {
    m_drive.tankDrive(0, 0);
  }
  
  /**
   * Gets the robot's heading
   * 
   * @return The robot's heading in degrees.
   */
  public double getHeading() {
    return m_gyro.getAngle();
  }

  /**
   * Gets the robot's turn rate
   * 
   * @return The robot's angular velocity in degrees / sec.
   */
  public double getGyroRate() {
    return m_gyro.getRate();
  }

  /** Resets the gyro to a zero state */
  public void resetGyro() {
    m_gyro.reset();
    // m_gyro.calibrate();
  }

  /** Returns the distance from the left encoder */
  public double getLeftEncoderDistance() {
    return m_encoder0.getDistance();
  }

  /** Returns the distance from the right encoder */
  public double getRightEncoderDistance() {
    return m_encoder1.getDistance();
  }

  public double getAverageEncoderDistance() {
    return (getLeftEncoderDistance() + getRightEncoderDistance()) / 2;
  }

  /** Returns the current rate of the left encoder */
  public double getLeftEncoderRate() {
    return m_encoder0.getRate();
  }

  /** Returns the current rate of the right encoder */
  public double getRightEncoderRate() {
    return m_encoder1.getRate();
  }

  /** Resets the encoders to a zero state */
  public void resetEncoders() {
    m_encoder0.reset();
    m_encoder1.reset();
  }

  /** Puts information in the SmartDashboard */
  public void log() {
    SmartDashboard.putNumber("Left Motor Output", m_left.get());
    SmartDashboard.putNumber("Right Motor Output", m_right.get());
    SmartDashboard.putNumber("Heading", m_gyro.getAngle());
    SmartDashboard.putNumber("Turn Rate", m_gyro.getRate());
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
