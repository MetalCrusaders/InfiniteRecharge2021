// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class DriveTrain extends SubsystemBase {

  private final WPI_VictorSPX leftMotor1;
  private final WPI_VictorSPX leftMotor2;
  private final WPI_VictorSPX rightMotor1;
  private final WPI_VictorSPX rightMotor2;

  private final SpeedController m_left;
  private final SpeedController m_right;

  private final Gyro m_gyro;

  private final DifferentialDrive m_drive;

  private final double kLeft = 1.1;

  /** Creates a new DriveTrain. */
  public DriveTrain() {
    leftMotor1 = new WPI_VictorSPX(Constants.DRIVE_LEFT_VICTORSPX0);
    leftMotor2 = new WPI_VictorSPX(Constants.DRIVE_LEFT_VICTORSPX1);
    rightMotor1 = new WPI_VictorSPX(Constants.DRIVE_RIGHT_VICTORSPX0);
    rightMotor2 = new WPI_VictorSPX(Constants.DRIVE_RIGHT_VICTORSPX1);

    m_left = new SpeedControllerGroup(leftMotor1, leftMotor2);
    m_right = new SpeedControllerGroup(rightMotor1, rightMotor2);

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
    m_drive.stopMotor();
  }
  
  /**
   * Gets the robot's heading
   * 
   * @return The robot's heading in degrees.
   */
  public double getHeading() {
    return m_gyro.getAngle();
  }

  /** Resets the gyro to a zero state */
  public void resetGyro() {
    m_gyro.reset();
    m_gyro.calibrate();
  }

  /** Puts information in the SmartDashboard */
  public void log() {
    SmartDashboard.putNumber("Left Speed", m_left.get());
    SmartDashboard.putNumber("Right Speed", m_right.get());
    SmartDashboard.putNumber("Gyro", m_gyro.getAngle());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    log();
  }
}
