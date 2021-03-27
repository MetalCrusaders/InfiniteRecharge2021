// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/** DriveTrain Conventions:
 * Shooter is at front of robot
 * Intake is at back of robot
 * Positive speed values should move forward
 * Forward movement should result in positive encoder readings
 * CCW rotation should result in positive gyro readings
 * Standard units for control classes are meters
 * 
 * This characterization drivetrain implements PIDControllers and FeedForward loops
 * based on results from the frc-characterization tool to have more accurate control over
 * both autonomous paths and direct controller input.
 */

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CharDriveTrain extends SubsystemBase {
  public static final double kMaxSpeed = 3.0; // meters per second
  public static final double kMaxAngularSpeed = 2 * Math.PI; // one rotation per second

  private static final double kTrackWidth = Constants.INCHES_TO_METERS * 21.0; // meters
  private static final double kWheelRadius = Constants.INCHES_TO_METERS * 3.0; // meters
  private static final int kEncoderResolution = 8192; // cycles per revolution

  private static final double left_kP = 0;
  private static final double left_kI = 0;
  private static final double left_kD = 0;

  private static final double right_kP = 0;
  private static final double right_kI = 0;
  private static final double right_kD = 0;

  private static final double kS = 0;
  private static final double kV = 0;

  private final WPI_VictorSPX leftLeader;
  private final WPI_VictorSPX leftFollower;
  private final WPI_VictorSPX rightLeader;
  private final WPI_VictorSPX rightFollower;

  private final SpeedControllerGroup m_left;
  private final SpeedControllerGroup m_right;

  private final PIDController m_leftPidController;
  private final PIDController m_rightPidController;

  private final DifferentialDriveKinematics m_kinematics;

  private final DifferentialDriveOdometry m_odometry;

  private final SimpleMotorFeedforward m_feedforward;

  private final Encoder m_leftEncoder;
  private final Encoder m_rightEncoder;

  private final Gyro m_gyro;

  /** Creates a new DriveTrain. */
  public CharDriveTrain() {
    leftLeader = new WPI_VictorSPX(Constants.DRIVE_LEFT_VICTORSPX0);
    leftFollower = new WPI_VictorSPX(Constants.DRIVE_LEFT_VICTORSPX1);
    rightLeader = new WPI_VictorSPX(Constants.DRIVE_RIGHT_VICTORSPX0);
    rightFollower = new WPI_VictorSPX(Constants.DRIVE_RIGHT_VICTORSPX1);

    leftLeader.setNeutralMode(NeutralMode.Brake);
    leftFollower.setNeutralMode(NeutralMode.Brake);
    rightLeader.setNeutralMode(NeutralMode.Brake);
    rightFollower.setNeutralMode(NeutralMode.Brake);

    m_left = new SpeedControllerGroup(leftLeader, leftFollower);
    m_right = new SpeedControllerGroup(rightLeader, rightFollower);

    m_left.setInverted(true);
    m_right.setInverted(false);

    m_leftPidController = new PIDController(left_kP, left_kI, left_kD);
    m_rightPidController = new PIDController(right_kP, right_kI, right_kD);

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
    
    m_kinematics = new DifferentialDriveKinematics(kTrackWidth);

    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());

    m_feedforward = new SimpleMotorFeedforward(kS, kV);
  }

  /**
   * Sets the desired wheel speeds
   * 
   * @param speeds The desired wheel speeds (in meters per second, I believe)
   */
  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    double leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond);
    double rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond);

    double leftOutput = m_leftPidController.calculate(m_leftEncoder.getRate(), speeds.leftMetersPerSecond);
    double rightOutput = m_rightPidController.calculate(m_rightEncoder.getRate(), speeds.rightMetersPerSecond);
    
    m_left.setVoltage(leftFeedforward + leftOutput);
    m_right.setVoltage(rightFeedforward + rightOutput);
  }

  /**
   * Arcade drive implementation utilizing feedforward and PID loops
   * 
   * @param xSpeed linear velocity in m/s
   * @param rotation angular velocity in rad/s
   */
  public void drive(double xSpeed, double rotation) {
    var wheelspeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rotation));
    setSpeeds(wheelspeeds);
  }

  public void updateOdometry() {
    m_odometry.update(
      m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
